from pyrobot.config.load_config import load_config
from pyrobot.hal.factory import create_motion_bus
from pyrobot.hal.fake_motion import FakeMotionBus


def test_factory_fake_backend() -> None:
    cfg = load_config()
    cfg.motion.backend = "fake"
    bus = create_motion_bus(cfg)
    try:
        assert isinstance(bus, FakeMotionBus)
    finally:
        bus.shutdown()

