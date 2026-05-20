from __future__ import annotations

import math

from proto.motion import MoveSegment
from pyrobot.config.load_config import KinematicsConfig, RobotConfig
from pyrobot.kinematics.forward import joint_deg_to_pose
from pyrobot.kinematics.inverse import pose_to_joint_deg
from pyrobot.kinematics.model import PoseMm


DEFAULT_PERIOD_US = 1000
RAPID_FEED_MM_MIN = 3000.0
# MCU motor.c clamps TIM ARR to [500, 20000] (microseconds per step).
MCU_MIN_PERIOD_US = 500
MCU_MAX_PERIOD_US = 20000
DEFAULT_MOVE_PERIOD_US = 5000


def joint_delta_to_steps(
    q_target_deg: list[float],
    q_current_deg: list[float],
    deg_per_step: float,
) -> list[int]:
    """
    Legacy delta: B steps include coupling with A (db = b - b_prev + da).
    """
    if len(q_target_deg) != 4 or len(q_current_deg) != 4:
        raise ValueError("joint vectors must have length 4")
    if deg_per_step <= 0:
        raise ValueError("deg_per_step must be > 0")

    da = q_target_deg[0] - q_current_deg[0]
    db = q_target_deg[1] - q_current_deg[1] + da
    dc = q_target_deg[2] - q_current_deg[2]
    dd = q_target_deg[3] - q_current_deg[3]
    inv = 1.0 / deg_per_step
    return [int(da * inv), int(db * inv), int(dc * inv), int(dd * inv)]


def cartesian_delta_periods_us(
    dx_mm: float,
    dy_mm: float,
    dz_mm: float,
    axis_steps: list[int],
    feed_mm_min: float,
    *,
    default_period_us: int = DEFAULT_PERIOD_US,
) -> list[int]:
    """Sync axis step periods from Cartesian travel time (legacy formula)."""
    if len(axis_steps) != 4:
        raise ValueError("axis_steps must have length 4")
    if feed_mm_min <= 0:
        feed_mm_min = 300.0

    dist = math.sqrt(dx_mm * dx_mm + dy_mm * dy_mm + dz_mm * dz_mm)
    t_s = dist / (feed_mm_min / 60.0) if dist > 1e-6 else 0.001

    periods: list[int] = []
    for steps in axis_steps:
        if steps == 0:
            periods.append(default_period_us)
        else:
            raw = int(t_s / abs(steps) * 1_000_000.0)
            periods.append(
                max(MCU_MIN_PERIOD_US, min(MCU_MAX_PERIOD_US, raw)),
            )
    return periods


def default_axis_periods_us(axis_steps: list[int], *, period_us: int = DEFAULT_MOVE_PERIOD_US) -> list[int]:
    """Fixed step period for joint-space moves (G28 / small homing). Matches uart_pkt_step bringup."""
    return [period_us if s != 0 else DEFAULT_PERIOD_US for s in axis_steps]


def plan_joint_move(
    q_target_deg: list[float],
    q_current_deg: list[float],
    kinematics: KinematicsConfig,
    *,
    cartesian_delta_mm: tuple[float, float, float] | None = None,
    feed_mm_min: float = 300.0,
    segment_id: int = 1,
) -> MoveSegment:
    steps = joint_delta_to_steps(q_target_deg, q_current_deg, kinematics.deg_per_step)
    if cartesian_delta_mm is not None:
        periods = cartesian_delta_periods_us(
            *cartesian_delta_mm,
            steps,
            feed_mm_min,
        )
    else:
        periods = [5000, 5000, 5000, 5000]
    return MoveSegment(segment_id=segment_id, axis_steps=steps, period_us=periods)


def plan_pose_move(
    target_pose_mm: list[float],
    q_current_deg: list[float],
    kinematics: KinematicsConfig,
    *,
    feed_mm_min: float = 300.0,
    d_target_deg: float | None = None,
    segment_id: int = 1,
) -> MoveSegment:
    if len(target_pose_mm) != 3:
        raise ValueError("target_pose_mm must be [x, y, z]")
    link = kinematics.link_length_mm
    joints = pose_to_joint_deg(
        target_pose_mm[0],
        target_pose_mm[1],
        target_pose_mm[2],
        link_length_mm=link,
    )
    d = q_current_deg[3] if d_target_deg is None else d_target_deg
    q_target = [joints.a, joints.b, joints.c, d]

    cur_pose = joint_deg_to_pose(q_current_deg[0], q_current_deg[1], q_current_deg[2], link_length_mm=link)
    dx = target_pose_mm[0] - cur_pose.x
    dy = target_pose_mm[1] - cur_pose.y
    dz = target_pose_mm[2] - cur_pose.z

    return plan_joint_move(
        q_target,
        q_current_deg,
        kinematics,
        cartesian_delta_mm=(dx, dy, dz),
        feed_mm_min=feed_mm_min,
        segment_id=segment_id,
    )


def plan_home_move(
    q_current_deg: list[float],
    cfg: RobotConfig,
    *,
    feed_mm_min: float = 300.0,
    segment_id: int = 1,
) -> MoveSegment:
    """G28: move joints to encoders.home_deg (joint-space steps, fixed step period)."""
    del feed_mm_min
    q_target = list(cfg.encoders.home_deg)
    steps = joint_delta_to_steps(q_target, q_current_deg, cfg.kinematics.deg_per_step)
    periods = default_axis_periods_us(steps)
    return MoveSegment(segment_id=segment_id, axis_steps=steps, period_us=periods)


def feed_for_linear(*, rapid: bool, feed_mm_min: float | None) -> float:
    if feed_mm_min is not None and feed_mm_min > 0:
        return feed_mm_min
    return RAPID_FEED_MM_MIN if rapid else 300.0
