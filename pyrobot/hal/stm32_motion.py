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
                try:
                    raw = cobs_decode(bytes(buf))
                    return parse_raw(raw)
                finally:
                    buf.clear()
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
                update={"fault_code": code, "fault_message": "mcu_fault", "in_motion": False}
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
            fault_code=fault if fault else self._fault_code,
            fault_message="mcu_fault" if (fault or self._fault_code) else "",
        )

    def pump(self, timeout_s: float = 0.02) -> None:
        try:
            pkt_type, seq, payload = self._read_frame(timeout_s)
        except TimeoutError:
            return
        self._handle_packet(pkt_type, seq, payload)

    def ping(self, timeout_s: float = 2.0) -> bool:
        seq = self._send_packet(PKT_PING)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                pkt_type, rx_seq, payload = self._read_frame(0.5)
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
        # temporary direct mapping: q_deg interpreted as target delta via steps.
        # Proper planner to generate segments lives in next step.
        deg_per_step = self._cfg.kinematics.deg_per_step
        steps = [int(q / deg_per_step) for q in q_deg]
        arr = [5000, 5000, 5000, 5000]
        payload = struct.pack("<iiiiIIII", steps[0], steps[1], steps[2], steps[3], *arr)
        seg_id = self._send_packet(PKT_MOVE_SEGMENT, payload)
        self._last_state = self._last_state.model_copy(update={"segment_id_active": seg_id, "in_motion": True})
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
        while time.monotonic() < deadline:
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

    @property
    def state(self) -> MotionState:
        self.pump(0.01)
        return self._last_state

