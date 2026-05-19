from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

from proto.common import SchemaHeader

SegmentId = int


class MoveSegment(BaseModel):
    """One trajectory segment for the motion MCU buffer."""

    segment_id: SegmentId
    axis_steps: list[int] = Field(min_length=4, max_length=4)
    period_us: list[int] = Field(min_length=4, max_length=4)
    accel_steps: int = 5


class MotionCommand(SchemaHeader):
    """Host → motion bridge."""

    kind: Literal["move_joints", "stream_segment", "estop", "home", "reset_fault"] = "move_joints"
    target_q_deg: list[float] | None = Field(default=None, min_length=4, max_length=4)
    target_pose_mm: list[float] | None = Field(
        default=None,
        description="[x, y, z] in base frame",
        min_length=3,
        max_length=3,
    )
    max_vel_mm_s: float = 50.0
    max_acc_mm_s2: float = 200.0
    segment: MoveSegment | None = None
    deadline_ms: int | None = None


class MotionState(SchemaHeader):
    """Motion bridge → subscribers @ telemetry_hz."""

    q_cmd_deg: list[float] = Field(default_factory=lambda: [90.0, 90.0, 0.0, 0.0])
    q_enc_deg: list[float] = Field(default_factory=lambda: [90.0, 90.0, 0.0, 0.0])
    q_vel_deg_s: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    in_motion: bool = False
    segment_id_active: SegmentId | None = None
    segment_id_done: SegmentId | None = None
    fault_code: int = 0
    fault_message: str = ""
