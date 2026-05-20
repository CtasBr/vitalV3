from pathlib import Path

from pyrobot.config.load_config import load_config


def test_load_config_default() -> None:
    cfg = load_config()
    assert cfg.schema_version == 1
    assert cfg.kinematics.link_length_mm == 250.0
    assert cfg.motion.backend in {"fake", "stm32", "legacy"}
    assert cfg.motion.max_abs_steps_cmd > 0
    assert set(cfg.motion.soft_limits_steps.keys()) == {"a", "b", "c", "d"}


def test_load_config_from_path() -> None:
    path = Path(__file__).resolve().parents[1] / "config" / "robot.yaml"
    cfg = load_config(path)
    assert cfg.kinematics.home.z == 250.0


def test_topic_uri() -> None:
    cfg = load_config()
    assert cfg.motion_state_uri().startswith("ipc:///tmp/robot/")
    assert cfg.vision_detections_uri().endswith("vision.detections")


def test_vision_config() -> None:
    cfg = load_config()
    assert cfg.vision.frame_dir == "/tmp/robot/frames"
    assert cfg.vision.display_size == [640, 480]
