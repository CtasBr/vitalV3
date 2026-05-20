from __future__ import annotations

from pyrobot.config.load_config import RobotConfig
from pyrobot.hal.encoder_bus import ExternalEncoderBus
from pyrobot.hal.encoder_offsets import load_offsets, save_offsets


def zero_encoders_at_pose(
    cfg: RobotConfig,
    enc: ExternalEncoderBus,
    *,
    hardware_zero: bool = True,
    cd_raw_deg: list[float] | None = None,
) -> dict[str, object]:
    """
    Calibrate encoder frame to ``encoders.home_deg`` (default 90/90/0/0).

    A/B: optional AT+ZERO, then software offset from current robot-frame read.
    C/D: use ``cd_raw_deg`` from motion step counters (no UART encoders).
    """
    if hardware_zero:
        enc.hardware_zero()

    ab = enc.read_ab()
    home = list(cfg.encoders.home_deg)
    if len(home) != 4:
        raise ValueError("encoders.home_deg must have length 4")

    if cd_raw_deg is None:
        cd_raw_deg = [0.0, 0.0]
    if len(cd_raw_deg) != 2:
        raise ValueError("cd_raw_deg must have length 2")

    raw = [ab[0], ab[1], float(cd_raw_deg[0]), float(cd_raw_deg[1])]
    offset_deg = [home[i] - raw[i] for i in range(4)]
    save_offsets(cfg, offset_deg)
    calibrated_deg = [raw[i] + offset_deg[i] for i in range(4)]

    return {
        "ok": True,
        "hardware_zero": hardware_zero,
        "home_deg": home,
        "robot_ab_deg": [ab[0], ab[1]],
        "offset_deg": offset_deg,
        "calibrated_deg": calibrated_deg,
    }


def cd_raw_from_motion_state(
    q_enc_deg: list[float],
    offsets: list[float],
) -> list[float]:
    """Undo file offset for C/D axes from latest motion.state q_enc_deg."""
    if len(q_enc_deg) != 4 or len(offsets) != 4:
        return [0.0, 0.0]
    return [q_enc_deg[2] - offsets[2], q_enc_deg[3] - offsets[3]]
