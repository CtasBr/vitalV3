from __future__ import annotations

from pyrobot.behaviour.gcode import gcode_to_motion_command, parse_gcode_line


def test_parse_g28() -> None:
    p = parse_gcode_line("G28 ; home")
    assert p is not None
    assert p.kind == "g28"


def test_parse_g1_with_feed() -> None:
    p = parse_gcode_line("G1 X260 Y0 Z240 F600")
    assert p is not None
    assert p.kind == "g1"
    assert p.x == 260.0
    assert p.f == 600.0


def test_gcode_to_motion_g28() -> None:
    cmd = gcode_to_motion_command("G28")
    assert cmd is not None
    assert cmd.kind == "home"


def test_gcode_to_motion_g1_partial_axes() -> None:
    cmd = gcode_to_motion_command(
        "G1 Z200 F300",
        current_pose_mm=[250.0, 0.0, 250.0],
    )
    assert cmd is not None
    assert cmd.kind == "linear_move"
    assert cmd.target_pose_mm == [250.0, 0.0, 200.0]
    assert cmd.feed_mm_min == 300.0
    assert cmd.rapid is False
