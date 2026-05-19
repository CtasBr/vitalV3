from __future__ import annotations

import os
import signal
import sys
import time

import structlog
import zmq

from proto.motion import MotionCommand, MotionState
from pyrobot.config.load_config import load_config
from pyrobot.hal.fake_motion import FakeMotionBus
from pyrobot.hal.zmq_bus import ZmqPublisher, ZmqReplyServer

log = structlog.get_logger(node="fake_motion_daemon")


def _ensure_ipc_dir(ipc_dir: str) -> None:
    os.makedirs(ipc_dir, exist_ok=True)
    # ZMQ IPC paths are files; stale sockets from crashed runs
    for name in os.listdir(ipc_dir):
        path = os.path.join(ipc_dir, name)
        if os.path.isfile(path) or os.path.islink(path):
            try:
                os.unlink(path)
            except OSError:
                pass


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
    bus = FakeMotionBus(cfg)
    pub = ZmqPublisher(ctx, cfg.motion_state_uri())
    rep = ZmqReplyServer(ctx, cfg.motion_cmd_uri())

    running = True

    def _stop(*_args: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    log.info("started", cmd=cfg.motion_cmd_uri(), state=cfg.motion_state_uri())
    period = 1.0 / cfg.motion.telemetry_hz

    try:
        while running:
            # Telemetry
            pub.publish(bus.state)

            # Commands (non-blocking poll)
            if rep.poll(0):
                cmd = rep.recv_command(MotionCommand)
                if cmd.kind == "estop":
                    bus.estop()
                    rep.send_reply(bus.state)
                elif cmd.kind == "reset_fault":
                    bus.reset_fault()
                    rep.send_reply(bus.state)
                elif cmd.kind == "home":
                    seg = bus.home()
                    rep.send_reply(
                        MotionState(segment_id_active=seg, node="fake_motion_daemon")
                    )
                elif cmd.kind == "move_joints" and cmd.target_q_deg is not None:
                    seg = bus.move_joints(
                        cmd.target_q_deg,
                        vmax_deg_s=cmd.max_vel_mm_s,
                        amax_deg_s2=cmd.max_acc_mm_s2,
                    )
                    rep.send_reply(
                        MotionState(segment_id_active=seg, node="fake_motion_daemon")
                    )
                elif cmd.kind == "stream_segment" and cmd.segment is not None:
                    bus.stream_segments([cmd.segment])
                    rep.send_reply(bus.state)
                else:
                    rep.send_reply(bus.state)
            else:
                time.sleep(period)
    finally:
        bus.shutdown()
        pub.close()
        rep.close()
        ctx.term()
        log.info("stopped")


if __name__ == "__main__":
    main()
    sys.exit(0)
