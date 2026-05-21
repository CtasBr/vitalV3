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
    encoders_cmd: str = "encoders.cmd"
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
    max_steps_per_segment: int = 600
    closed_loop_ab: bool = True
    closed_loop_ab_gain: float = 1.0
    closed_loop_max_corr_deg: float = 4.0
    mcu_watchdog_idle_ms: int = 2500
    move_timeout_s: float = 50.0
    segment_wait_timeout_s: float = 65.0
    soft_limits_steps: dict[str, tuple[int, int]] = Field(
        default_factory=lambda: {
            "a": (-4800, 4800),
            "b": (-4800, 4800),
            "c": (-4800, 4800),
            "d": (-4800, 4800),
        }
    )
    # +1 / -1 per axis: match motor DIR to joint +angle (encoder frame)
    step_sign: dict[str, int] = Field(
        default_factory=lambda: {"a": 1, "b": 1, "c": 1, "d": 1}
    )

    def step_sign_list(self) -> list[int]:
        keys = ("a", "b", "c", "d")
        return [int(self.step_sign.get(k, 1)) for k in keys]


class EncodersConfig(BaseModel):
    port_a: str
    port_b: str
    baudrate: int = 9600
    poll_hz: int = 20
    home_deg: list[float] = Field(default_factory=lambda: [90.0, 90.0, 0.0, 0.0], min_length=4, max_length=4)
    offsets_file: str = "config/encoders_offsets.json"


class TofConfig(BaseModel):
    port: str
    baudrate: int = 115200
    quantize: int = 2
    delta_mm: int = 38


class CameraConfig(BaseModel):
    index: int = 0
    width: int = 1920
    height: int = 1080
    # auto | avfoundation (macOS) | v4l2 (Linux) | default
    backend: str = "auto"


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


class UiConfig(BaseModel):
    host: str = "0.0.0.0"
    port: int = 8080
    jog_step_mm: float = 10.0
    voice_feed_mm_min: float = 800.0


class VisionConfig(BaseModel):
    frame_dir: str = "/tmp/robot/frames"
    loop_hz: int = 10
    display_size: list[int] = Field(default_factory=lambda: [640, 480], min_length=2, max_length=2)
    camera_enabled: bool = True
    tof_enabled: bool = True
    tof_fps: int = 10
    tof_disp: int = 3
    yolo_enabled: bool = True
    yolo_model: str = "yolov8s.pt"
    yolo_every_n_frames: int = 2


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
    ui: UiConfig = Field(default_factory=UiConfig)
    vision: VisionConfig = Field(default_factory=VisionConfig)

    def topic_uri(self, topic: str) -> str:
        return f"ipc://{self.zmq.ipc_dir}/{topic}"

    def motion_state_uri(self) -> str:
        return self.topic_uri(self.zmq.topics.motion_state)

    def motion_cmd_uri(self) -> str:
        return self.topic_uri(self.zmq.topics.motion_cmd)

    def encoders_state_uri(self) -> str:
        return self.topic_uri(self.zmq.topics.encoders_state)

    def vision_detections_uri(self) -> str:
        return self.topic_uri(self.zmq.topics.vision_detections)

    def encoders_cmd_uri(self) -> str:
        return self.topic_uri(self.zmq.topics.encoders_cmd)


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
