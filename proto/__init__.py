"""Message contracts (Pydantic). Single source of truth for inter-process messages."""

from proto.common import Heartbeat, SchemaHeader
from proto.motion import (
    MotionCommand,
    MotionState,
    MoveSegment,
    SegmentId,
)
from proto.vision import DepthFrame, Detection, EncoderState, RgbFrame

__all__ = [
    "DepthFrame",
    "Detection",
    "EncoderState",
    "Heartbeat",
    "MotionCommand",
    "MotionState",
    "MoveSegment",
    "RgbFrame",
    "SchemaHeader",
    "SegmentId",
]
