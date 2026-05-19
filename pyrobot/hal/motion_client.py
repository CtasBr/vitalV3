from __future__ import annotations

import time
from collections.abc import Iterable

import zmq

from proto.motion import MotionCommand, MotionState, MoveSegment, SegmentId
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.motion_bus import MotionBus
from pyrobot.hal.zmq_bus import ZmqSubscriber, pack_model, unpack_model


class ZmqMotionClient(MotionBus):
    """REQ/REP to motion daemon + SUB for telemetry."""

    def __init__(self, config: RobotConfig | None = None) -> None:
        self._cfg = config or load_config()
        self._ctx = zmq.Context()
        self._req = self._ctx.socket(zmq.REQ)
        self._req.connect(self._cfg.motion_cmd_uri())
        self._sub = ZmqSubscriber(self._ctx, self._cfg.motion_state_uri())
        self._latest = MotionState(node="motion_client")

    def _request(self, cmd: MotionCommand) -> MotionState:
        self._req.send(pack_model(cmd))
        data = self._req.recv()
        return unpack_model(data, MotionState)

    def move_joints(
        self,
        q_deg: list[float],
        vmax_deg_s: float = 30.0,
        amax_deg_s2: float = 90.0,
    ) -> SegmentId:
        reply = self._request(
            MotionCommand(
                kind="move_joints",
                target_q_deg=q_deg,
                max_vel_mm_s=vmax_deg_s,
                max_acc_mm_s2=amax_deg_s2,
                node="motion_client",
            )
        )
        assert reply.segment_id_active is not None
        return reply.segment_id_active

    def stream_segments(self, segments: Iterable[MoveSegment]) -> None:
        for seg in segments:
            self._request(
                MotionCommand(kind="stream_segment", segment=seg, node="motion_client")
            )

    def wait_done(self, segment_id: SegmentId, timeout_s: float = 30.0) -> MotionState:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            snap = self._poll_state()
            if snap.segment_id_done == segment_id and not snap.in_motion:
                return snap
            time.sleep(0.01)
        return self.state

    def estop(self) -> None:
        self._request(MotionCommand(kind="estop", node="motion_client"))

    def _poll_state(self) -> MotionState:
        msg = self._sub.recv_model(MotionState, timeout_ms=50)
        if msg is not None:
            self._latest = msg
        return self._latest

    @property
    def state(self) -> MotionState:
        return self._poll_state()

    def close(self) -> None:
        self._sub.close()
        self._req.close()
        self._ctx.term()
