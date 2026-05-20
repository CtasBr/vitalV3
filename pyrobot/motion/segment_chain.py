from __future__ import annotations

from proto.motion import MoveSegment
from pyrobot.config.load_config import RobotConfig
from pyrobot.motion.planner import joint_delta_to_steps, plan_joint_move


def max_abs_axis_steps(steps: list[int]) -> int:
    return max((abs(s) for s in steps), default=0)


def split_move_segment(segment: MoveSegment, max_steps_per_segment: int) -> list[MoveSegment]:
    """Split one MOVE_SEGMENT into smaller chunks (MCU runs each block synchronously)."""
    if max_steps_per_segment <= 0:
        return [segment]
    steps = list(segment.axis_steps)
    peak = max_abs_axis_steps(steps)
    if peak <= max_steps_per_segment:
        return [segment]

    n = (peak + max_steps_per_segment - 1) // max_steps_per_segment
    parts: list[list[int]] = []
    prev = [0, 0, 0, 0]
    for i in range(1, n + 1):
        frac = i / n
        cumulative = [int(round(s * frac)) for s in steps]
        delta = [cumulative[j] - prev[j] for j in range(4)]
        prev = cumulative
        if any(d != 0 for d in delta):
            parts.append(delta)

    if not parts:
        return [segment]

    out: list[MoveSegment] = []
    base_id = segment.segment_id
    for idx, delta in enumerate(parts):
        out.append(
            MoveSegment(
                segment_id=base_id + idx,
                axis_steps=delta,
                period_us=list(segment.period_us),
                accel_steps=segment.accel_steps,
            )
        )
    return out


def lerp_joint_deg(q0: list[float], q1: list[float], t: float) -> list[float]:
    return [q0[i] + (q1[i] - q0[i]) * t for i in range(4)]


def ab_closed_loop_step_correction(
    q_waypoint_deg: list[float],
    q_actual_deg: list[float],
    deg_per_step: float,
    *,
    gain: float = 1.0,
    max_corr_deg: float = 3.0,
) -> tuple[int, int]:
    """Extra A/B steps to apply at start of next chunk (encoder fusion)."""
    err_a = max(-max_corr_deg, min(max_corr_deg, (q_waypoint_deg[0] - q_actual_deg[0]) * gain))
    err_b = max(-max_corr_deg, min(max_corr_deg, (q_waypoint_deg[1] - q_actual_deg[1]) * gain))
    inv = 1.0 / deg_per_step
    return int(err_a * inv), int(err_b * inv)


def plan_joint_move_chain(
    q_target_deg: list[float],
    q_start_deg: list[float],
    cfg: RobotConfig,
    *,
    cartesian_delta_mm: tuple[float, float, float] | None = None,
    feed_mm_min: float = 300.0,
) -> list[MoveSegment]:
    """Full joint move as one or more MCU segments."""
    whole = plan_joint_move(
        q_target_deg,
        q_start_deg,
        cfg.kinematics,
        cartesian_delta_mm=cartesian_delta_mm,
        feed_mm_min=feed_mm_min,
    )
    return split_move_segment(whole, cfg.motion.max_steps_per_segment)


def apply_ab_correction_to_segment(
    segment: MoveSegment,
    corr_a: int,
    corr_b: int,
) -> MoveSegment:
    steps = list(segment.axis_steps)
    steps[0] += corr_a
    steps[1] += corr_b
    return segment.model_copy(update={"axis_steps": steps})


def chain_waypoints(q_start: list[float], q_end: list[float], n: int) -> list[list[float]]:
    if n <= 1:
        return [list(q_end)]
    return [lerp_joint_deg(q_start, q_end, (i + 1) / n) for i in range(n)]
