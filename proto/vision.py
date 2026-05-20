from __future__ import annotations

from pydantic import BaseModel, Field

from proto.common import SchemaHeader


class EncoderState(SchemaHeader):
    angle_a_deg: float
    angle_b_deg: float


class RgbFrame(SchemaHeader):
    """Metadata only — pixels live in shared memory."""

    shm_name: str
    frame_id: int
    width: int
    height: int
    channels: int = 3


class DepthFrame(SchemaHeader):
    shm_name: str
    frame_id: int
    width: int
    height: int
    quantize_mm: int = 2


class Detection(SchemaHeader):
    class_name: str
    confidence: float = Field(ge=0.0, le=1.0)
    bbox_xyxy: list[float] = Field(min_length=4, max_length=4)
    mask_shm_name: str | None = None
    source: str = "yolo"


class VisionState(SchemaHeader):
    """vision_daemon → UI / skills."""

    detections: list[Detection] = Field(default_factory=list)
    tof_distance_mm: float | None = None
    offset_mm: list[float] | None = Field(default=None, min_length=3, max_length=3)
