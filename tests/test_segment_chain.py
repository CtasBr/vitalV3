from proto.motion import MoveSegment
from pyrobot.config.load_config import load_config
from pyrobot.motion.segment_chain import (
    ab_closed_loop_step_correction,
    max_abs_axis_steps,
    split_move_segment,
)


def test_split_large_segment() -> None:
    seg = MoveSegment(
        segment_id=1,
        axis_steps=[2400, 0, 0, 0],
        period_us=[5000, 5000, 5000, 5000],
    )
    parts = split_move_segment(seg, 600)
    assert len(parts) == 4
    assert sum(p.axis_steps[0] for p in parts) == 2400
    assert all(max_abs_axis_steps(p.axis_steps) <= 600 for p in parts)


def test_split_small_unchanged() -> None:
    seg = MoveSegment(segment_id=2, axis_steps=[10, -5, 0, 0], period_us=[5000] * 4)
    parts = split_move_segment(seg, 600)
    assert len(parts) == 1
    assert parts[0].axis_steps == [10, -5, 0, 0]


def test_ab_closed_loop_correction() -> None:
    cfg = load_config()
    corr_a, corr_b = ab_closed_loop_step_correction(
        [91.0, 90.0, 0.0, 0.0],
        [90.0, 89.0, 0.0, 0.0],
        cfg.kinematics.deg_per_step,
        gain=1.0,
    )
    assert corr_a > 0
    assert corr_b > 0
