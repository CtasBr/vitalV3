from __future__ import annotations

import zmq

from proto.vision import EncoderState
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.zmq_bus import ZmqSubscriber


class EncoderZmqClient:
    """Subscribe to encoders.state — single source of truth for A/B joint angles."""

    def __init__(self, ctx: zmq.Context, config: RobotConfig | None = None) -> None:
        self._cfg = config or load_config()
        self._sub = ZmqSubscriber(ctx, self._cfg.encoders_state_uri())
        self._latest: EncoderState | None = None

    def poll(self, timeout_ms: int = 0) -> bool:
        st = self._sub.recv_model(EncoderState, timeout_ms=timeout_ms)
        if st is None:
            return False
        self._latest = st
        return True

    def ab_deg(self) -> tuple[float, float] | None:
        if self._latest is None:
            return None
        return self._latest.angle_a_deg, self._latest.angle_b_deg

    @property
    def latest(self) -> EncoderState | None:
        return self._latest

    def close(self) -> None:
        self._sub.close()
