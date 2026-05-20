from __future__ import annotations

import struct
import time
from collections.abc import Iterable

import serial

from proto.motion import MotionState, MoveSegment, SegmentId
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.motion_bus import MotionBus
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

    def close(self) -> None:
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
        self._ser.write(wire)
        self._ser.flush()
        return s

    def _read_frame(self, timeout_s: float) -> tuple[int, int, bytes]:
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            b = self._ser.read(1)
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
                    "fault_message": "mcu_fault" if code else "",
                    "in_motion": False,
                }
            )
            return
        if pkt_type in (PKT_PONG, PKT_HEARTBEAT):
            return

    def _handle_telemetry(self, payload: bytes) -> None:
        if len(payload) < 24:
            return
        pos = list(struct.unpack("<iiii", payload[:16]))
        in_motion_mask = payload[16]
        fault = payload[17]
        # Keep host-side fault state aligned with current telemetry.
        # Firmware currently reports latched fault in telemetry byte.
        self._fault_code = int(fault)
        deg_per_step = self._cfg.kinematics.deg_per_step
        q_enc = [p * deg_per_step for p in pos]
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
            fault_message="mcu_fault" if self._fault_code else "",
        )

    def pump(self, timeout_s: float = 0.02) -> None:
        try:
            pkt_type, seq, payload = self._read_frame(timeout_s)
        except TimeoutError:
            return
        self._handle_packet(pkt_type, seq, payload)

    def ping(self, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_s
        # Retry a few times because USB CDC/ST-Link can drop/garble occasional frames.
        while time.monotonic() < deadline:
            seq = self._send_packet(PKT_PING)
            attempt_deadline = min(deadline, time.monotonic() + 0.6)
            while time.monotonic() < attempt_deadline:
                try:
                    pkt_type, rx_seq, payload = self._read_frame(0.25)
                except TimeoutError:
                    continue
                self._handle_packet(pkt_type, rx_seq, payload)
                if pkt_type == PKT_PONG and rx_seq == seq:
                    return True
        return False

    def send_heartbeat(self, timeout_s: float = 1.0) -> bool:
        seq = self._send_packet(PKT_HEARTBEAT, struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                pkt_type, rx_seq, payload = self._read_frame(0.3)
            except TimeoutError:
                continue
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
        # Safety-first transitional behavior:
        # at current firmware stage we treat move_joints input as step deltas (not degrees).
        # Proper IK/planner->segments path will replace this.
        steps = [int(round(v)) for v in q_deg]
        return self.move_steps(steps)

    def move_steps(self, steps: list[int], arr: list[int] | None = None) -> SegmentId:
        if len(steps) != 4:
            raise ValueError("steps must have length 4")
        # hard safety clamp for direct manual commands
        max_abs_steps = int(self._cfg.motion.max_abs_steps_cmd)
        if max_abs_steps <= 0:
            raise ValueError("motion.max_abs_steps_cmd must be > 0")
        if any(abs(s) > max_abs_steps for s in steps):
            raise ValueError(
                f"step command too large {steps}; max abs per axis is {max_abs_steps}"
            )
        if arr is None:
            arr = [5000, 5000, 5000, 5000]
        if len(arr) != 4:
            raise ValueError("arr must have length 4")

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
        deg_per_step = self._cfg.kinematics.deg_per_step
        q_cmd = [s * deg_per_step for s in steps]
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
                pkt_type, rx_seq, payload = self._read_frame(0.25)
            except TimeoutError:
                continue
            self._handle_packet(pkt_type, rx_seq, payload)
            if pkt_type == PKT_FAULT and rx_seq == seq:
                code = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else -1
                return code == 0
        return False

    @property
    def state(self) -> MotionState:
        self.pump(0.01)
        return self._last_state

