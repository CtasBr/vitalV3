from __future__ import annotations

from typing import Literal

from pydantic import Field

from proto.common import SchemaHeader


class EncoderCommand(SchemaHeader):
    """Host → encoder_daemon (REQ/REP)."""

    kind: Literal["zero"] = "zero"
    hardware_zero: bool = True


class EncoderZeroResult(SchemaHeader):
    """encoder_daemon → host after zero."""

    ok: bool = True
    hardware_zero: bool = True
    home_deg: list[float] = Field(min_length=4, max_length=4)
    robot_ab_deg: list[float] = Field(min_length=2, max_length=2)
    offset_deg: list[float] = Field(min_length=4, max_length=4)
    calibrated_deg: list[float] = Field(min_length=4, max_length=4)
