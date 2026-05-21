from __future__ import annotations

from proto.motion import MoveSegment, MotionState
from pyrobot.config.load_config import RobotConfig
from pyrobot.hal.move_control import is_cancelled
from pyrobot.motion.segment_chain import (
    ab_closed_loop_step_correction,
    apply_ab_correction_to_segment,
    chain_waypoints,
    split_move_segment,
)


def execute_segment_chain(
    bus: object,
    segments: list[MoveSegment],
    cfg: RobotConfig,
    *,
    q_start_deg: list[float],
    q_target_deg: list[float],
    timeout_s_per_segment: float | None = None,
) -> MotionState:
    """
    Run MCU segments sequentially (wait for each SEGMENT_DONE).
    Optional A/B closed-loop correction between chunks.
    """
    if not segments:
        return bus.state  # type: ignore[attr-defined]

    seg_timeout = timeout_s_per_segment
    if seg_timeout is None:
        seg_timeout = float(cfg.motion.segment_wait_timeout_s)

    n = len(segments)
    waypoints = chain_waypoints(q_start_deg, q_target_deg, n)
    dps = cfg.kinematics.deg_per_step
    gain = cfg.motion.closed_loop_ab_gain
    max_corr = cfg.motion.closed_loop_max_corr_deg

    last_st: MotionState = bus.state  # type: ignore[attr-defined]
    for idx, raw_seg in enumerate(segments):
        if is_cancelled():
            return last_st
        seg = raw_seg
        if cfg.motion.closed_loop_ab and idx > 0:
            q_act = list(last_st.q_enc_deg)
            corr_a, corr_b = ab_closed_loop_step_correction(
                waypoints[idx - 1],
                q_act,
                dps,
                gain=gain,
                max_corr_deg=max_corr,
            )
            if corr_a or corr_b:
                seg = apply_ab_correction_to_segment(seg, corr_a, corr_b)

        seg_id = bus.move_steps(seg.axis_steps, seg.period_us)  # type: ignore[attr-defined]
        if seg_id == 0:
            continue
        last_st = bus.wait_done(seg_id, timeout_s=seg_timeout)  # type: ignore[attr-defined]
        if is_cancelled():
            return last_st
        if last_st.fault_code != 0:
            return last_st
        if last_st.segment_id_done != seg_id:
            return last_st
        bus.pump(0.05)  # type: ignore[attr-defined]
        if hasattr(bus, "tick_heartbeat"):
            bus.tick_heartbeat()  # type: ignore[attr-defined]

    return last_st
