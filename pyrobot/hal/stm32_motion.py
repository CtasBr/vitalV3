from __future__ import annotations

import struct
import threading
import time
from collections.abc import Iterable

import serial

from proto.motion import MotionState, MoveSegment, SegmentId
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.encoder_bus import ExternalEncoderBus
from pyrobot.hal.encoder_offsets import load_offsets, save_offsets
from pyrobot.hal.encoder_zmq import EncoderZmqClient
from pyrobot.hal.fault_codes import mcu_fault_message
from pyrobot.hal.motion_bus import MotionBus
from pyrobot.kinematics.forward import joint_deg_to_pose
from pyrobot.kinematics.inverse import pose_to_joint_deg
from pyrobot.motion.execute_chain import execute_segment_chain
from pyrobot.motion.planner import (
    feed_for_linear,
    plan_home_move,
    plan_joint_move,
    plan_pose_move,
)
from pyrobot.motion.segment_chain import plan_joint_move_chain, split_move_segment
from pyrobot.hal.stm32_protocol import (
    PKT_ESTOP,
    PKT_FAULT,
    PKT_HEARTBEAT,
    PKT_MOVE_SEGMENT,
    PKT_PING,
    PKT_PONG,
    PKT_RESET_FAULT,
    PKT_SEGMENT_DONE,
    PKT_TELEMETRY,
    build_raw,
    cobs_decode,
    cobs_encode,
    parse_raw,
)


class Stm32MotionBus(MotionBus):
    """
    Motion bus over STM32 UART protocol (COBS + CRC).
    Uses current firmware packet set: MOVE_SEGMENT, TELEMETRY, SEGMENT_DONE, ESTOP, HEARTBEAT.
    """

    def __init__(self, config: RobotConfig | None = None, timeout_s: float = 0.5) -> None:
        self._cfg = config or load_config()
        self._ser = serial.Serial(
            self._cfg.motion.port,
            self._cfg.motion.baudrate,
            timeout=timeout_s,
        )
        # Drop any boot/banner leftovers on port open.
        time.sleep(0.05)
        self._ser.reset_input_buffer()
        self._seq = 1
        self._last_state = MotionState(node="stm32_motion")
        self._last_done: tuple[int, list[int]] | None = None
        self._fault_code = 0
        self._enc_offset_deg = load_offsets(self._cfg)
        self._q_enc_raw_deg = [90.0, 90.0, 0.0, 0.0]
        self._last_pos_steps = [0, 0, 0, 0]
        self._encoder_bus: ExternalEncoderBus | None = None
        self._encoder_zmq: EncoderZmqClient | None = None
        self._uart_lock = threading.RLock()

    def set_encoder_zmq(self, client: EncoderZmqClient) -> None:
        """Use ZMQ encoders.state for A/B (from encoder_daemon). Does not open UART encoder ports."""
        self._encoder_zmq = client

    def close(self) -> None:
        if self._encoder_bus is not None:
            self._encoder_bus.close()
            self._encoder_bus = None
        if self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> Stm32MotionBus:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        return self._seq

    def _send_packet(self, pkt_type: int, payload: bytes = b"", seq: int | None = None) -> int:
        s = self._next_seq() if seq is None else seq & 0xFFFF
        raw = build_raw(pkt_type, s, payload)
        wire = cobs_encode(raw)
        with self._uart_lock:
            if not self._ser.is_open:
                raise serial.SerialException("motion uart closed")
            self._ser.write(wire)
            self._ser.flush()
        return s

    def _read_frame(self, timeout_s: float) -> tuple[int, int, bytes]:
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            try:
                b = self._ser.read(1)
            except serial.SerialException as exc:
                raise OSError(f"read failed: {exc}") from exc
            if not b:
                continue
            if b[0] == 0:
                if not buf:
                    continue
                frame = bytes(buf)
                buf.clear()
                try:
                    raw = cobs_decode(frame)
                    return parse_raw(raw)
                except ValueError:
                    # USB CDC can produce occasional partial/garbled frame boundaries.
                    # Ignore bad frame and continue reading the stream.
                    continue
            else:
                buf.extend(b)
        raise TimeoutError("no uart frame")

    def _handle_packet(self, pkt_type: int, seq: int, payload: bytes) -> None:
        if pkt_type == PKT_TELEMETRY:
            self._handle_telemetry(payload)
            return
        if pkt_type == PKT_SEGMENT_DONE:
            done = list(struct.unpack("<iiii", payload[:16])) if len(payload) >= 16 else [0, 0, 0, 0]
            self._last_done = (seq, done)
            self._last_state = self._last_state.model_copy(
                update={"segment_id_done": seq, "in_motion": False}
            )
            return
        if pkt_type == PKT_FAULT:
            code = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else -1
            self._fault_code = code
            self._last_state = self._last_state.model_copy(
                update={
                    "fault_code": code,
                    "fault_message": mcu_fault_message(code),
                    "in_motion": False,
                }
            )
            return
        if pkt_type in (PKT_PONG, PKT_HEARTBEAT):
            return

    def _get_encoder_bus(self) -> ExternalEncoderBus:
        if self._encoder_bus is None:
            self._encoder_bus = ExternalEncoderBus(self._cfg)
        return self._encoder_bus

    def read_encoder_ab_deg(self) -> tuple[float, float]:
        """Read real A/B encoder angles (legacy transform, before file offset)."""
        return self._get_encoder_bus().read_ab()

    def _step_deg_from_pos(self, pos_steps: list[int]) -> list[float]:
        dps = self._cfg.kinematics.deg_per_step
        return [p * dps for p in pos_steps]

    def _build_q_enc_deg(self, step_deg: list[float]) -> list[float]:
        """
        A/B: ZMQ EncoderState (already calibrated) if attached, else direct UART + offset.
        C/D: STM32 step counters + offset from file.
        """
        cd = [
            step_deg[2] + self._enc_offset_deg[2],
            step_deg[3] + self._enc_offset_deg[3],
        ]
        if self._encoder_zmq is not None:
            self._encoder_zmq.poll(timeout_ms=0)
            ab = self._encoder_zmq.ab_deg()
            if ab is not None:
                self._q_enc_raw_deg = [ab[0], ab[1], step_deg[2], step_deg[3]]
                return [ab[0], ab[1], cd[0], cd[1]]
        try:
            ab = self.read_encoder_ab_deg()
            self._q_enc_raw_deg = [ab[0], ab[1], step_deg[2], step_deg[3]]
            return [
                ab[0] + self._enc_offset_deg[0],
                ab[1] + self._enc_offset_deg[1],
                cd[0],
                cd[1],
            ]
        except (RuntimeError, serial.SerialException, OSError):
            self._q_enc_raw_deg = list(step_deg)
            return [
                step_deg[0] + self._enc_offset_deg[0],
                step_deg[1] + self._enc_offset_deg[1],
                cd[0],
                cd[1],
            ]

    def _handle_telemetry(self, payload: bytes) -> None:
        if len(payload) < 24:
            return
        pos = list(struct.unpack("<iiii", payload[:16]))
        in_motion_mask = payload[16]
        fault = payload[17]
        # Keep host-side fault state aligned with current telemetry.
        # Firmware currently reports latched fault in telemetry byte.
        self._fault_code = int(fault)
        self._last_pos_steps = pos
        step_deg = self._step_deg_from_pos(pos)
        q_enc = self._build_q_enc_deg(step_deg)
        in_motion = in_motion_mask != 0
        self._last_state = MotionState(
            node="stm32_motion",
            q_cmd_deg=self._last_state.q_cmd_deg,
            q_enc_deg=q_enc,
            q_vel_deg_s=self._last_state.q_vel_deg_s,
            in_motion=in_motion,
            segment_id_active=self._last_state.segment_id_active,
            segment_id_done=self._last_state.segment_id_done,
            fault_code=self._fault_code,
            fault_message=mcu_fault_message(self._fault_code),
        )

    def pump(self, timeout_s: float = 0.02) -> None:
        """Drain all pending COBS frames (telemetry, SEGMENT_DONE, HB echo, …)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                with self._uart_lock:
                    if not self._ser.is_open or self._ser.in_waiting == 0:
                        break
                    pkt_type, seq, payload = self._read_frame(0.003)
            except TimeoutError:
                break
            except (serial.SerialException, OSError):
                break
            self._handle_packet(pkt_type, seq, payload)

    def _read_frame_locked(self, timeout_s: float) -> tuple[int, int, bytes]:
        with self._uart_lock:
            if not self._ser.is_open:
                raise serial.SerialException("motion uart closed")
            return self._read_frame(timeout_s)

    def ping(self, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_s
        # Retry a few times because USB CDC/ST-Link can drop/garble occasional frames.
        while time.monotonic() < deadline:
            seq = self._send_packet(PKT_PING)
            attempt_deadline = min(deadline, time.monotonic() + 0.6)
            while time.monotonic() < attempt_deadline:
                try:
                    pkt_type, rx_seq, payload = self._read_frame_locked(0.25)
                except TimeoutError:
                    continue
                except (serial.SerialException, OSError):
                    return False
                self._handle_packet(pkt_type, rx_seq, payload)
                if pkt_type == PKT_PONG and rx_seq == seq:
                    return True
        return False

    def tick_heartbeat(self) -> None:
        """Send heartbeat without blocking (motion_daemon watchdog keepalive)."""
        self._send_packet(
            PKT_HEARTBEAT,
            struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF),
        )

    def send_heartbeat(self, timeout_s: float = 1.0) -> bool:
        seq = self._send_packet(PKT_HEARTBEAT, struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                pkt_type, rx_seq, payload = self._read_frame_locked(0.3)
            except TimeoutError:
                continue
            except (serial.SerialException, OSError):
                return False
            self._handle_packet(pkt_type, rx_seq, payload)
            if pkt_type == PKT_HEARTBEAT and rx_seq == seq:
                return True
        return False

    def move_joints(
        self,
        q_deg: list[float],
        vmax_deg_s: float = 30.0,
        amax_deg_s2: float = 90.0,
    ) -> SegmentId:
        del vmax_deg_s, amax_deg_s2
        return self.move_to_joints_deg(q_deg)

    def move_to_joints_deg(
        self,
        q_target_deg: list[float],
        q_current_deg: list[float] | None = None,
        *,
        feed_mm_min: float = 300.0,
        cartesian_delta_mm: tuple[float, float, float] | None = None,
    ) -> SegmentId:
        """Absolute joint targets (deg); planner converts to steps + periods."""
        q_cur = list(q_current_deg if q_current_deg is not None else self.state.q_enc_deg)
        seg = plan_joint_move(
            list(q_target_deg),
            q_cur,
            self._cfg.kinematics,
            cartesian_delta_mm=cartesian_delta_mm,
            feed_mm_min=feed_mm_min,
        )
        return self.move_steps(seg.axis_steps, seg.period_us)

    def move_to_pose_mm(
        self,
        pose_mm: list[float],
        q_current_deg: list[float] | None = None,
        *,
        feed_mm_min: float = 300.0,
        rapid: bool = False,
        d_target_deg: float | None = None,
    ) -> SegmentId:
        """G0/G1: Cartesian target -> joint IK -> segmented MOVE with A/B closed-loop."""
        q_cur = list(q_current_deg if q_current_deg is not None else self.state.q_enc_deg)
        feed = feed_for_linear(rapid=rapid, feed_mm_min=feed_mm_min)
        link = self._cfg.kinematics.link_length_mm
        joints = pose_to_joint_deg(
            pose_mm[0],
            pose_mm[1],
            pose_mm[2],
            link_length_mm=link,
        )
        d = q_cur[3] if d_target_deg is None else d_target_deg
        q_tgt = [joints.a, joints.b, joints.c, d]
        cur_pose = joint_deg_to_pose(q_cur[0], q_cur[1], q_cur[2], link_length_mm=link)
        dx = pose_mm[0] - cur_pose.x
        dy = pose_mm[1] - cur_pose.y
        dz = pose_mm[2] - cur_pose.z
        segments = plan_joint_move_chain(
            q_tgt,
            q_cur,
            self._cfg,
            cartesian_delta_mm=(dx, dy, dz),
            feed_mm_min=feed,
        )
        return self._execute_segment_chain(segments, q_cur, q_tgt)

    def _execute_segment_chain(
        self,
        segments: list[MoveSegment],
        q_start_deg: list[float],
        q_target_deg: list[float],
    ) -> SegmentId:
        if not segments:
            return 0
        st = execute_segment_chain(
            self,
            segments,
            self._cfg,
            q_start_deg=q_start_deg,
            q_target_deg=q_target_deg,
        )
        self._last_state = st
        return segments[-1].segment_id

    def home(self, feed_mm_min: float = 300.0) -> SegmentId:
        """G28: move to encoders.home_deg (default 90/90/0/0)."""
        del feed_mm_min
        q_cur = list(self.state.q_enc_deg)
        q_tgt = list(self._cfg.encoders.home_deg)
        seg = plan_home_move(q_cur, self._cfg)
        segments = split_move_segment(seg, self._cfg.motion.max_steps_per_segment)
        return self._execute_segment_chain(segments, q_cur, q_tgt)

    def current_pose_mm(self) -> list[float]:
        q = self.state.q_enc_deg
        pose = joint_deg_to_pose(
            q[0],
            q[1],
            q[2],
            link_length_mm=self._cfg.kinematics.link_length_mm,
        )
        return [pose.x, pose.y, pose.z]

    def move_steps(self, steps: list[int], arr: list[int] | None = None) -> SegmentId:
        if len(steps) != 4:
            raise ValueError("steps must have length 4")
        # hard safety clamp for direct manual commands
        max_abs_steps = int(self._cfg.motion.max_abs_steps_cmd)
        if max_abs_steps <= 0:
            raise ValueError("motion.max_abs_steps_cmd must be > 0")
        if arr is None:
            arr = [5000, 5000, 5000, 5000]
        if len(arr) != 4:
            raise ValueError("arr must have length 4")

        signs = self._cfg.motion.step_sign_list()
        steps = [signs[i] * steps[i] for i in range(4)]
        if any(abs(s) > max_abs_steps for s in steps):
            raise ValueError(
                f"step command too large {steps}; max abs per axis is {max_abs_steps}"
            )

        if all(s == 0 for s in steps):
            return 0

        payload = struct.pack(
            "<iiiiIIII",
            steps[0],
            steps[1],
            steps[2],
            steps[3],
            arr[0],
            arr[1],
            arr[2],
            arr[3],
        )
        seg_id = self._send_packet(PKT_MOVE_SEGMENT, payload)
        q_cur = list(self.state.q_enc_deg)
        q_cmd = [q_cur[i] + steps[i] * self._cfg.kinematics.deg_per_step for i in range(4)]
        self._last_state = self._last_state.model_copy(
            update={"segment_id_active": seg_id, "in_motion": True, "q_cmd_deg": q_cmd}
        )
        return seg_id

    def stream_segments(self, segments: Iterable[MoveSegment]) -> None:
        for seg in segments:
            payload = struct.pack(
                "<iiiiIIII",
                seg.axis_steps[0],
                seg.axis_steps[1],
                seg.axis_steps[2],
                seg.axis_steps[3],
                seg.period_us[0],
                seg.period_us[1],
                seg.period_us[2],
                seg.period_us[3],
            )
            seg_id = self._send_packet(PKT_MOVE_SEGMENT, payload, seq=seg.segment_id)
            self._last_state = self._last_state.model_copy(
                update={"segment_id_active": seg_id, "in_motion": True}
            )

    def wait_done(self, segment_id: SegmentId, timeout_s: float = 30.0) -> MotionState:
        deadline = time.monotonic() + timeout_s
        next_hb_at = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            # Keep firmware watchdog satisfied while waiting for segment completion.
            if now >= next_hb_at:
                self._send_packet(
                    PKT_HEARTBEAT, struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF)
                )
                next_hb_at = now + 0.25
            self.pump(0.2)
            st = self._last_state
            if st.fault_code != 0:
                return st
            if st.segment_id_done == segment_id:
                return st
        return self._last_state

    def estop(self) -> None:
        self._send_packet(PKT_ESTOP)
        # best effort wait for fault/ack
        for _ in range(5):
            self.pump(0.2)

    def reset_fault(self, timeout_s: float = 1.0) -> bool:
        seq = self._send_packet(PKT_RESET_FAULT)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                pkt_type, rx_seq, payload = self._read_frame_locked(0.25)
            except TimeoutError:
                continue
            except (serial.SerialException, OSError):
                return False
            self._handle_packet(pkt_type, rx_seq, payload)
            if pkt_type == PKT_FAULT and rx_seq == seq:
                code = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else -1
                return code == 0
        return False

    def zero_encoders(
        self,
        home_deg: list[float] | None = None,
        *,
        hardware_zero: bool = True,
    ) -> dict[str, object]:
        """
        Calibrate encoder frame to home pose.

        1) Optional AT+ZERO on encoder hardware (current pose -> device 0).
        2) Software offset so calibrated angle equals home_deg (default 90/90/0/0).
        """
        for _ in range(5):
            self.pump(0.05)
        target_home = home_deg if home_deg is not None else list(self._cfg.encoders.home_deg)
        if len(target_home) != 4:
            raise ValueError("home_deg must have length 4")

        enc = self._get_encoder_bus()
        if hardware_zero:
            enc.hardware_zero()

        # A/B: UART encoders (always direct for calibration, even when motion uses ZMQ).
        ab = enc.read_ab()
        cd_raw = self._step_deg_from_pos(self._last_pos_steps)
        self._q_enc_raw_deg = [ab[0], ab[1], cd_raw[0], cd_raw[1]]
        self._enc_offset_deg = [target_home[i] - self._q_enc_raw_deg[i] for i in range(4)]
        save_offsets(self._cfg, self._enc_offset_deg)
        q_enc = self._build_q_enc_deg(self._q_enc_raw_deg)
        self._last_state = self._last_state.model_copy(update={"q_enc_deg": q_enc})
        return {
            "hardware_zero": hardware_zero,
            "home_deg": target_home,
            "robot_ab_deg": [ab[0], ab[1]],
            "offset_deg": list(self._enc_offset_deg),
            "calibrated_deg": q_enc,
        }

    @property
    def state(self) -> MotionState:
        self.pump(0.01)
        step_deg = list(self._q_enc_raw_deg)
        if len(step_deg) == 4:
            q_enc = self._build_q_enc_deg(step_deg)
            self._last_state = self._last_state.model_copy(update={"q_enc_deg": q_enc})
        return self._last_state

