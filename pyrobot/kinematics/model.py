from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PoseMm:
    x: float
    y: float
    z: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class JointDeg:
    """Joint angles in degrees: A, B, C (base), D (tool/extruder)."""

    a: float
    b: float
    c: float
    d: float = 0.0

    def as_list(self) -> list[float]:
        return [self.a, self.b, self.c, self.d]
