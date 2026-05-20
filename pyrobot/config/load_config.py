from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


class ZmqTopics(BaseModel):
    motion_cmd: str
    motion_state: str
    encoders_state: str
    camera_rgb: str
    tof_depth: str
    vision_detections: str
    world_objects: str
    skills_cmd: str
    heartbeat: str


class ZmqConfig(BaseModel):
    ipc_dir: str
    topics: ZmqTopics


class MotionConfig(BaseModel):
    backend: str = "fake"
    port: str = ""
    baudrate: int = 115200
    num_axes: int = 4
    telemetry_hz: int = 100
    heartbeat_hz: int = 10
    watchdog_ms: int = 200
    max_abs_steps_cmd: int = 500
    soft_limits_steps: dict[str, tuple[int, int]] = Field(
        default_factory=lambda: {
            "a": (-4800, 4800),
            "b": (-4800, 4800),
            "c": (-4800, 4800),
            "d": (-4800, 4800),
        }
    )


class EncodersConfig(BaseModel):
    port_a: str
    port_b: str
    baudrate: int = 9600


class TofConfig(BaseModel):
    port: str
    baudrate: int = 115200
    quantize: int = 2
    delta_mm: int = 38


class CameraConfig(BaseModel):
    index: int = 0
    width: int = 1920
    height: int = 1080


class HomePose(BaseModel):
    x: float
    y: float
    z: float


class KinematicsConfig(BaseModel):
    link_length_mm: float = 250.0
    home: HomePose
    deg_per_step: float = 0.01875


class LimitsConfig(BaseModel):
    x: list[float]
    y: list[float]
    z: list[float]


class ToolheadsConfig(BaseModel):
    angle_between_deg: float = 45.0
    mid_x: list[float]
    table_len_mm: float = 200.0


class SimulationConfig(BaseModel):
    fake_motion_hz: int = 100
    joint_limits_deg: dict[str, list[float]]


class RobotConfig(BaseModel):
    schema_version: int = 1
    zmq: ZmqConfig
    motion: MotionConfig
    encoders: EncodersConfig
    tof: TofConfig
    camera: CameraConfig
    kinematics: KinematicsConfig
    limits: LimitsConfig
    toolheads: ToolheadsConfig
    simulation: SimulationConfig

    def topic_uri(self, topic: str) -> str:
        return f"ipc://{self.zmq.ipc_dir}/{topic}"

    def motion_state_uri(self) -> str:
        return self.topic_uri(self.zmq.topics.motion_state)

    def motion_cmd_uri(self) -> str:
        return self.topic_uri(self.zmq.topics.motion_cmd)


def load_config(path: Path | str | None = None) -> RobotConfig:
    """Load robot.yaml. Default: config/robot.yaml at repo root."""
    if path is None:
        path = _repo_root() / "config" / "robot.yaml"
    else:
        path = Path(path)

    with path.open(encoding="utf-8") as f:
        raw: dict[str, Any] = yaml.safe_load(f)

    return RobotConfig.model_validate(raw)


@lru_cache(maxsize=1)
def load_config_cached(path: str | None = None) -> RobotConfig:
    return load_config(path)
