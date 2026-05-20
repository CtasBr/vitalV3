from __future__ import annotations

import os
import signal
import time

import structlog
import zmq

from proto.vision import EncoderState
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.encoder_bus import ExternalEncoderBus, transform_legacy_ab
from pyrobot.hal.encoder_offsets import load_offsets
from pyrobot.hal.zmq_bus import ZmqPublisher

log = structlog.get_logger(node="encoder_daemon")


def _ensure_ipc_dir(ipc_dir: str) -> None:
    os.makedirs(ipc_dir, exist_ok=True)
    for name in os.listdir(ipc_dir):
        path = os.path.join(ipc_dir, name)
        if os.path.isfile(path) or os.path.islink(path):
            try:
                os.unlink(path)
            except OSError:
                pass


def read_encoder_state(cfg: RobotConfig | None = None) -> EncoderState:
    robot_cfg = load_config() if cfg is None else cfg

    offsets = load_offsets(robot_cfg)
    with ExternalEncoderBus(robot_cfg) as enc:
        raw_a, raw_b = enc.read_raw_ab()
    ab = transform_legacy_ab(raw_a, raw_b)
    return EncoderState(
        node="encoder_daemon",
        angle_a_deg=ab[0] + offsets[0],
        angle_b_deg=ab[1] + offsets[1],
    )


def main() -> None:
    structlog.configure(
        processors=[
            structlog.processors.add_log_level,
            structlog.processors.TimeStamper(fmt="iso"),
            structlog.dev.ConsoleRenderer(),
        ],
    )
    cfg = load_config()
    _ensure_ipc_dir(cfg.zmq.ipc_dir)

    ctx = zmq.Context()
    pub = ZmqPublisher(ctx, cfg.encoders_state_uri())
    period = 1.0 / max(cfg.encoders.poll_hz, 1)

    running = True

    def _stop(*_args: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    log.info("started", state=cfg.encoders_state_uri(), hz=cfg.encoders.poll_hz)
    try:
        while running:
            try:
                st = read_encoder_state(cfg)
                pub.publish(st)
            except Exception as exc:
                log.warning("encoder_read_failed", error=str(exc))
            time.sleep(period)
    finally:
        pub.close()
        ctx.term()
        log.info("stopped")


if __name__ == "__main__":
    main()
