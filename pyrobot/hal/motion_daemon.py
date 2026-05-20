from __future__ import annotations

import os
import signal
import sys
import time

import structlog
import zmq

from proto.motion import MotionCommand, MotionState
from pyrobot.config.load_config import load_config
from pyrobot.hal.encoder_zmq import EncoderZmqClient
from pyrobot.hal.factory import create_motion_bus
from pyrobot.hal.motion_bus import MotionBus
from pyrobot.hal.stm32_motion import Stm32MotionBus
from pyrobot.hal.zmq_bus import ZmqPublisher, ZmqReplyServer

log = structlog.get_logger(node="motion_daemon")


def _ensure_ipc_dir(ipc_dir: str) -> None:
    os.makedirs(ipc_dir, exist_ok=True)
    for name in os.listdir(ipc_dir):
        path = os.path.join(ipc_dir, name)
        if os.path.isfile(path) or os.path.islink(path):
            try:
                os.unlink(path)
            except OSError:
                pass


def _attach_encoder_zmq(bus: MotionBus, enc: EncoderZmqClient) -> None:
    if isinstance(bus, Stm32MotionBus):
        bus.set_encoder_zmq(enc)


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
    enc_zmq = EncoderZmqClient(ctx, cfg)
    bus = create_motion_bus(cfg)
    _attach_encoder_zmq(bus, enc_zmq)

    pub = ZmqPublisher(ctx, cfg.motion_state_uri())
    rep = ZmqReplyServer(ctx, cfg.motion_cmd_uri())

    running = True
    enc_warned = False
    enc_ready = False

    def _stop(*_args: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    log.info(
        "started",
        cmd=cfg.motion_cmd_uri(),
        state=cfg.motion_state_uri(),
        encoders=cfg.encoders_state_uri(),
        backend=cfg.motion.backend,
    )
    period = 1.0 / cfg.motion.telemetry_hz

    try:
        while running:
            enc_zmq.poll(timeout_ms=0)
            if enc_zmq.ab_deg() is None:
                if not enc_warned:
                    log.warning(
                        "no_encoder_zmq_yet",
                        hint="start first: python -m pyrobot.hal.encoder_daemon",
                    )
                    enc_warned = True
            elif not enc_ready:
                ab = enc_zmq.ab_deg()
                log.info("encoder_zmq_ready", a_deg=ab[0], b_deg=ab[1])
                enc_ready = True

            if isinstance(bus, Stm32MotionBus):
                bus.pump(0)
            st = bus.state
            pub.publish(st.model_copy(update={"node": "motion_daemon"}))

            if rep.poll(0):
                cmd = rep.recv_command(MotionCommand)
                if cmd.kind == "estop":
                    bus.estop()
                    rep.send_reply(bus.state)
                elif cmd.kind == "reset_fault" and isinstance(bus, Stm32MotionBus):
                    bus.reset_fault()
                    rep.send_reply(bus.state)
                elif cmd.kind == "home" and hasattr(bus, "home"):
                    seg = bus.home()  # type: ignore[attr-defined]
                    rep.send_reply(MotionState(segment_id_active=seg, node="motion_daemon"))
                elif cmd.kind == "move_joints" and cmd.target_q_deg is not None:
                    seg = bus.move_joints(
                        cmd.target_q_deg,
                        vmax_deg_s=cmd.max_vel_mm_s,
                        amax_deg_s2=cmd.max_acc_mm_s2,
                    )
                    rep.send_reply(MotionState(segment_id_active=seg, node="motion_daemon"))
                elif cmd.kind == "stream_segment" and cmd.segment is not None:
                    bus.stream_segments([cmd.segment])
                    rep.send_reply(bus.state)
                else:
                    rep.send_reply(bus.state)
            else:
                time.sleep(period)
    finally:
        close = getattr(bus, "close", None)
        if callable(close):
            close()
        enc_zmq.close()
        pub.close()
        rep.close()
        ctx.term()
        log.info("stopped")


if __name__ == "__main__":
    main()
    sys.exit(0)
