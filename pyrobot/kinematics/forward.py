from __future__ import annotations

import math

from pyrobot.kinematics.model import JointDeg, PoseMm


def joint_deg_to_pose(
    a: float,
    b: float,
    c: float,
    *,
    link_length_mm: float = 250.0,
) -> PoseMm:
    """Joint angles (deg) -> Cartesian pose (mm). Inverse of pose_to_joint_deg."""
    la = lb = link_length_mm
    br = math.radians(b)
    l = math.sqrt(max(0.0, 2.0 * la * lb * (1.0 - math.cos(br))))
    tilt = math.radians(a - (180.0 - b) / 2.0)
    z = l * math.sin(tilt)
    r = l * math.cos(tilt)
    cr = math.radians(c)
    x = r * math.cos(cr)
    y = r * math.sin(cr)
    return PoseMm(x=x, y=y, z=z)


def joint_to_pose_mm(joint: JointDeg, **kwargs: float) -> PoseMm:
    return joint_deg_to_pose(joint.a, joint.b, joint.c, **kwargs)
