from __future__ import annotations

import math

from pyrobot.kinematics.model import JointDeg, PoseMm


class UnreachablePoseError(ValueError):
    """Target pose is outside the arm workspace."""


def max_reach_mm(link_length_mm: float) -> float:
    return 2.0 * link_length_mm


def is_reachable(
    x: float,
    y: float,
    z: float,
    *,
    link_length_mm: float = 250.0,
) -> bool:
    la = lb = link_length_mm
    l_sq = x * x + y * y + z * z
    if l_sq < 1e-12:
        return True
    l = math.sqrt(l_sq)
    if l > max_reach_mm(link_length_mm) + 1e-6:
        return False
    cos_b = 1.0 - l_sq / (2.0 * la * lb)
    return -1.0 <= cos_b <= 1.0


def pose_to_joint_deg(
    x: float,
    y: float,
    z: float,
    *,
    link_length_mm: float = 250.0,
    c_added_deg: float = 0.0,
) -> JointDeg:
    """
    Cartesian pose (mm) -> joint angles (deg).
    Legacy RobotArmCalculator direct kinematics (README).
    """
    if not is_reachable(x, y, z, link_length_mm=link_length_mm):
        raise UnreachablePoseError(f"pose ({x}, {y}, {z}) is unreachable")

    la = lb = link_length_mm
    l_sq = x * x + y * y + z * z
    if l_sq < 1e-12:
        return JointDeg(a=90.0, b=90.0, c=c_added_deg)

    l = math.sqrt(l_sq)
    cos_b = 1.0 - l_sq / (2.0 * la * lb)
    cos_b = max(-1.0, min(1.0, cos_b))
    b = math.degrees(math.acos(cos_b))
    z_clamped = max(-1.0, min(1.0, z / l))
    a = (180.0 - b) / 2.0 + math.degrees(math.asin(z_clamped))
    c = math.degrees(math.atan2(y, x)) + c_added_deg
    return JointDeg(a=a, b=b, c=c)


def pose_mm_to_joint_deg(pose: PoseMm, **kwargs: float) -> JointDeg:
    return pose_to_joint_deg(pose.x, pose.y, pose.z, **kwargs)
