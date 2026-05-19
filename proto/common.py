from __future__ import annotations

import time

from pydantic import BaseModel, Field


class SchemaHeader(BaseModel):
    schema_version: int = 1
    timestamp_ns: int = Field(default_factory=lambda: time.time_ns())
    seq: int = 0
    node: str = ""


class Heartbeat(SchemaHeader):
    """Watchdog / liveness."""

    status: str = "ok"
    uptime_s: float = 0.0
