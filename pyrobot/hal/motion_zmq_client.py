from __future__ import annotations

import time

import zmq

from proto.motion import MotionCommand, MotionState
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.zmq_bus import ZmqSubscriber, pack_model, unpack_model


class MotionZmqClient:
    """
    Send MotionCommand to motion_daemon (REQ/REP) and watch motion.state (PUB/SUB).
    Use when motion_daemon owns the STM32 UART.
    """

    def __init__(self, config: RobotConfig | None = None) -> None:
        self._cfg = config or load_config()
        self._ctx = zmq.Context()
        self._req = self._ctx.socket(zmq.REQ)
        self._req.setsockopt(zmq.LINGER, 0)
        # Daemon must reply quickly (move completion is tracked on motion.state SUB).
        self._req.setsockopt(zmq.RCVTIMEO, 10_000)
        self._req.connect(self._cfg.motion_cmd_uri())
        self._sub = ZmqSubscriber(self._ctx, self._cfg.motion_state_uri(), conflate=True)

    def close(self) -> None:
        self._sub.close()
        self._req.close()
        self._ctx.term()

    def __enter__(self) -> MotionZmqClient:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def drain_state(self, timeout_ms: int = 50) -> MotionState | None:
        """Drop queued telemetry; return latest sample if any."""
        last: MotionState | None = None
        while True:
            st = self._sub.recv_model(MotionState, timeout_ms=timeout_ms)
            if st is None:
                break
            last = st
            timeout_ms = 0
        return last

    def send_command(self, cmd: MotionCommand) -> MotionState:
        self.drain_state()
        self._req.send(pack_model(cmd))
        data = self._req.recv()
        return unpack_model(data, MotionState)

    @property
    def state(self) -> MotionState | None:
        return self._sub.recv_model(MotionState, timeout_ms=200)

    def wait_done(self, segment_id: int | None, timeout_s: float = 30.0) -> MotionState:
        deadline = time.monotonic() + timeout_s
        last: MotionState | None = None
        saw_motion = False

        if segment_id is None:
            while time.monotonic() < deadline:
                st = self._sub.recv_model(MotionState, timeout_ms=200)
                if st is None:
                    continue
                last = st
                if st.in_motion:
                    saw_motion = True
                elif saw_motion or st.fault_code != 0:
                    return st
            if last is not None:
                return last
            raise TimeoutError("motion did not complete")

        while time.monotonic() < deadline:
            st = self._sub.recv_model(MotionState, timeout_ms=200)
            if st is None:
                continue
            last = st
            if st.fault_code != 0:
                return st
            if st.segment_id_active == segment_id or st.in_motion:
                saw_motion = True
            if st.segment_id_done == segment_id:
                return st
            if saw_motion and not st.in_motion:
                return st

        if last is not None:
            return last
        raise TimeoutError(f"segment {segment_id} did not complete within {timeout_s}s")
