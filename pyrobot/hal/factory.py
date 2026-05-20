from __future__ import annotations

from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.fake_motion import FakeMotionBus
from pyrobot.hal.motion_bus import MotionBus
from pyrobot.hal.stm32_motion import Stm32MotionBus


def create_motion_bus(config: RobotConfig | None = None) -> MotionBus:
    cfg = config or load_config()
    backend = cfg.motion.backend.lower().strip()
    if backend == "fake":
        return FakeMotionBus(cfg)
    if backend == "stm32":
        return Stm32MotionBus(cfg)
    if backend == "legacy":
        raise NotImplementedError("legacy backend is not wired in pyrobot yet")
    raise ValueError(f"Unknown motion.backend={cfg.motion.backend!r}")

