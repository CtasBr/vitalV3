from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Literal

from proto.motion import MotionCommand

GcodeKind = Literal["g0", "g1", "g28", "unknown"]

_WORD = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


@dataclass(frozen=True)
class ParsedGcode:
    kind: GcodeKind
    x: float | None = None
    y: float | None = None
    z: float | None = None
    e: float | None = None
    f: float | None = None
    raw: str = ""


def parse_gcode_line(line: str) -> ParsedGcode | None:
    """
    Parse one G-code line (G0/G1/G28). Comments after ';' are stripped.
    Returns None for empty or non-motion lines.
    """
    raw = line.strip()
    if not raw:
        return None
    code_part = raw.split(";", 1)[0].strip()
    if not code_part:
        return None

    upper = code_part.upper()
    kind: GcodeKind = "unknown"
    if upper.startswith("G28"):
        kind = "g28"
    elif upper.startswith("G0"):
        kind = "g0"
    elif upper.startswith("G1"):
        kind = "g1"
    else:
        return None

    words: dict[str, float] = {}
    for m in _WORD.finditer(code_part):
        words[m.group(1).upper()] = float(m.group(2))

    return ParsedGcode(
        kind=kind,
        x=words.get("X"),
        y=words.get("Y"),
        z=words.get("Z"),
        e=words.get("E"),
        f=words.get("F"),
        raw=raw,
    )


def gcode_to_motion_command(
    line: str,
    *,
    current_pose_mm: list[float] | None = None,
    current_q_deg: list[float] | None = None,
) -> MotionCommand | None:
    """
    Map G0/G1/G28 to MotionCommand.
    G0/G1: missing axes keep current pose; E maps to joint D target delta (absolute E as D deg).
    """
    parsed = parse_gcode_line(line)
    if parsed is None:
        return None

    if parsed.kind == "g28":
        return MotionCommand(kind="home", node="gcode")

    pose = list(current_pose_mm) if current_pose_mm is not None else [250.0, 0.0, 250.0]
    if parsed.x is not None:
        pose[0] = parsed.x
    if parsed.y is not None:
        pose[1] = parsed.y
    if parsed.z is not None:
        pose[2] = parsed.z

    q_tgt: list[float] | None = None
    if parsed.e is not None:
        q_tgt = list(current_q_deg) if current_q_deg is not None else [90.0, 90.0, 0.0, 0.0]
        q_tgt[3] = parsed.e

    rapid = parsed.kind == "g0"
    feed = parsed.f
    return MotionCommand(
        kind="linear_move",
        node="gcode",
        target_pose_mm=pose,
        target_q_deg=q_tgt,
        feed_mm_min=feed if feed is not None else 300.0,
        rapid=rapid,
    )
