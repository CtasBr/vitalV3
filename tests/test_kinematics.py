from __future__ import annotations

import pytest

from pyrobot.kinematics.forward import joint_deg_to_pose
from pyrobot.kinematics.inverse import UnreachablePoseError, is_reachable, pose_to_joint_deg


def test_home_pose_roundtrip() -> None:
    j = pose_to_joint_deg(250.0, 0.0, 250.0, link_length_mm=250.0)
    assert j.a == pytest.approx(90.0, abs=0.5)
    assert j.b == pytest.approx(90.0, abs=0.5)
    assert j.c == pytest.approx(0.0, abs=0.5)

    pose = joint_deg_to_pose(j.a, j.b, j.c, link_length_mm=250.0)
    assert pose.x == pytest.approx(250.0, rel=0.01)
    assert pose.y == pytest.approx(0.0, abs=1.0)
    assert pose.z == pytest.approx(250.0, rel=0.01)


def test_unreachable_pose() -> None:
    assert not is_reachable(600.0, 0.0, 0.0, link_length_mm=250.0)
    with pytest.raises(UnreachablePoseError):
        pose_to_joint_deg(600.0, 0.0, 0.0, link_length_mm=250.0)


def test_joint_coupling_steps() -> None:
    from pyrobot.motion.planner import joint_delta_to_steps

    steps = joint_delta_to_steps([100.0, 80.0, 10.0, 0.0], [90.0, 90.0, 0.0, 0.0], 0.01875)
    # da=10 -> 533 steps; db = -10+10=0; dc=10 -> 533
    assert steps[0] == int(round(10.0 / 0.01875))
    assert steps[1] == 0
    assert steps[2] == int(round(10.0 / 0.01875))


def test_cartesian_periods_scale_with_feed() -> None:
    from pyrobot.motion.planner import cartesian_delta_periods_us

    slow = cartesian_delta_periods_us(100.0, 0.0, 0.0, [100, 0, 0, 0], 60.0)
    fast = cartesian_delta_periods_us(100.0, 0.0, 0.0, [100, 0, 0, 0], 6000.0)
    assert fast[0] < slow[0]
    assert fast[0] >= 500
