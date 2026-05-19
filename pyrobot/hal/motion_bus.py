from __future__ import annotations

import abc
import time
from collections.abc import Iterable

from proto.motion import MotionCommand, MotionState, MoveSegment, SegmentId


class MotionBus(abc.ABC):
    """Abstract motion interface for planner / skills."""

    @abc.abstractmethod
    def move_joints(
        self,
        q_deg: list[float],
        vmax_deg_s: float = 30.0,
        amax_deg_s2: float = 90.0,
    ) -> SegmentId:
        ...

    @abc.abstractmethod
    def stream_segments(self, segments: Iterable[MoveSegment]) -> None:
        ...

    @abc.abstractmethod
    def wait_done(self, segment_id: SegmentId, timeout_s: float = 30.0) -> MotionState:
        ...

    @abc.abstractmethod
    def estop(self) -> None:
        ...

    @property
    @abc.abstractmethod
    def state(self) -> MotionState:
        """Latest telemetry snapshot (non-blocking)."""
        ...

    def move_joints_blocking(
        self,
        q_deg: list[float],
        vmax_deg_s: float = 30.0,
        amax_deg_s2: float = 90.0,
        timeout_s: float = 30.0,
    ) -> MotionState:
        seg_id = self.move_joints(q_deg, vmax_deg_s=vmax_deg_s, amax_deg_s2=amax_deg_s2)
        return self.wait_done(seg_id, timeout_s=timeout_s)

    @staticmethod
    def now_ns() -> int:
        return time.time_ns()
