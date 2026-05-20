from __future__ import annotations

import zmq

from proto.encoders import EncoderCommand, EncoderZeroResult
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.zmq_bus import pack_model, unpack_model


class EncoderZmqClient:
    """REQ/REP to encoder_daemon (e.g. zero encoders while daemon holds UART)."""

    def __init__(self, config: RobotConfig | None = None, ctx: zmq.Context | None = None) -> None:
        self._cfg = config or load_config()
        self._owns_ctx = ctx is None
        self._ctx = ctx or zmq.Context()
        self._req = self._ctx.socket(zmq.REQ)
        self._req.setsockopt(zmq.LINGER, 0)
        self._req.setsockopt(zmq.RCVTIMEO, 15_000)
        self._req.connect(self._cfg.encoders_cmd_uri())

    def close(self) -> None:
        self._req.close()
        if self._owns_ctx:
            self._ctx.term()

    def __enter__(self) -> EncoderZmqClient:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def zero_encoders(self, *, hardware_zero: bool = True) -> EncoderZeroResult:
        self._req.send(
            pack_model(
                EncoderCommand(
                    kind="zero",
                    hardware_zero=hardware_zero,
                    node="encoder_zmq_client",
                )
            )
        )
        data = self._req.recv()
        return unpack_model(data, EncoderZeroResult)
