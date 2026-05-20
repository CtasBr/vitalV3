from __future__ import annotations

import threading
import time
from typing import Any

import zmq

from proto.motion import MotionCommand, MotionState
from proto.vision import EncoderState, VisionState
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.zmq_bus import ZmqSubscriber, unpack_model
from pyrobot.kinematics.forward import joint_deg_to_pose


class RobotStateCache:
    """Background SUB to motion.state + encoders.state."""

    def __init__(self, config: RobotConfig | None = None) -> None:
        self._cfg = config or load_config()
        self._motion = MotionState(node="ui")
        self._encoders: EncoderState | None = None
        self._vision = VisionState(node="ui")
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="ui-state")

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        ctx = zmq.Context()
        motion_sub = ZmqSubscriber(ctx, self._cfg.motion_state_uri(), conflate=True)
        enc_sub = ZmqSubscriber(ctx, self._cfg.encoders_state_uri(), conflate=True)
        vis_sub = ZmqSubscriber(ctx, self._cfg.vision_detections_uri(), conflate=True)
        try:
            while not self._stop.is_set():
                m = motion_sub.recv_model(MotionState, timeout_ms=100)
                e = enc_sub.recv_model(EncoderState, timeout_ms=0)
                v = vis_sub.recv_model(VisionState, timeout_ms=0)
                with self._lock:
                    if m is not None:
                        self._motion = m
                    if e is not None:
                        self._encoders = e
                    if v is not None:
                        self._vision = v
                time.sleep(0.02)
        finally:
            motion_sub.close()
            enc_sub.close()
            vis_sub.close()
            ctx.term()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            motion = self._motion.model_copy()
            enc = self._encoders
            vision = self._vision.model_copy()

        link = self._cfg.kinematics.link_length_mm
        q = motion.q_enc_deg
        pose = joint_deg_to_pose(q[0], q[1], q[2], link_length_mm=link)
        home = self._cfg.kinematics.home

        return {
            "motion": motion.model_dump(mode="json"),
            "encoders": enc.model_dump(mode="json") if enc else None,
            "pose_mm": {"x": pose.x, "y": pose.y, "z": pose.z},
            "home_mm": {"x": home.x, "y": home.y, "z": home.z},
            "joints_deg": {
                "a": q[0],
                "b": q[1],
                "c": q[2],
                "d": q[3],
            },
            "link_length_mm": link,
            "vision": vision.model_dump(mode="json"),
        }
