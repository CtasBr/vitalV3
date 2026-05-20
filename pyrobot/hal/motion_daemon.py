from __future__ import annotations

import os
import signal
import sys
import time

import structlog
import zmq

from proto.motion import MotionCommand, MotionState, SegmentId
from pyrobot.config.load_config import load_config
from pyrobot.hal.encoder_zmq import EncoderZmqClient
from pyrobot.hal.factory import create_motion_bus
from pyrobot.hal.motion_bus import MotionBus
from pyrobot.hal.stm32_motion import Stm32MotionBus
from pyrobot.behaviour.gcode import gcode_to_motion_command
from pyrobot.hal.zmq_bus import ZmqPublisher, ZmqReplyServer
from pyrobot.motion.planner import feed_for_linear

log = structlog.get_logger(node="motion_daemon")


def _fresh_state(bus: MotionBus, *, segment_id_active: SegmentId | None = None) -> MotionState:
    st = bus.state
    update: dict[str, object] = {
        "node": "motion_daemon",
        "timestamp_ns": time.time_ns(),
    }
    if segment_id_active is not None:
        update["segment_id_active"] = segment_id_active
    return st.model_copy(update=update)


def _dispatch_motion(bus: MotionBus, cmd: MotionCommand) -> SegmentId | None:
    """Execute motion command; return segment id if a move was started."""
    if isinstance(bus, Stm32MotionBus) and bus.state.fault_code != 0:
        log.warning(
            "move_rejected_mcu_fault",
            fault_code=bus.state.fault_code,
            fault_message=bus.state.fault_message,
            hint="python -m pyrobot.hal.motion_cli reset-fault",
        )
        return None

    if cmd.kind == "gcode" and cmd.gcode_line:
        pose = getattr(bus, "current_pose_mm", None)
        pose_mm = pose() if callable(pose) else None
        q = bus.state.q_enc_deg
        sub = gcode_to_motion_command(cmd.gcode_line, current_pose_mm=pose_mm, current_q_deg=q)
        if sub is None:
            return None
        return _dispatch_motion(bus, sub)

    if cmd.kind in ("home", "g28"):
        home = getattr(bus, "home", None)
        if callable(home):
            return home(feed_mm_min=cmd.feed_mm_min)
        return None

    if cmd.kind == "linear_move" and cmd.target_pose_mm is not None:
        move_pose = getattr(bus, "move_to_pose_mm", None)
        if callable(move_pose):
            feed = feed_for_linear(rapid=cmd.rapid, feed_mm_min=cmd.feed_mm_min)
            d_tgt = cmd.target_q_deg[3] if cmd.target_q_deg and len(cmd.target_q_deg) == 4 else None
            return move_pose(
                cmd.target_pose_mm,
                feed_mm_min=feed,
                rapid=cmd.rapid,
                d_target_deg=d_tgt,
            )
        return None

    if cmd.kind == "move_joints" and cmd.target_q_deg is not None:
        return bus.move_joints(
            cmd.target_q_deg,
            vmax_deg_s=cmd.max_vel_mm_s,
            amax_deg_s2=cmd.max_acc_mm_s2,
        )

    return None


def _ensure_ipc_dir(ipc_dir: str, motion_topics: tuple[str, ...]) -> None:
    """Remove stale motion IPC sockets only (do not touch encoder_daemon endpoints)."""
    os.makedirs(ipc_dir, exist_ok=True)
    for topic in motion_topics:
        path = os.path.join(ipc_dir, topic)
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
    _ensure_ipc_dir(
        cfg.zmq.ipc_dir,
        (cfg.zmq.topics.motion_cmd, cfg.zmq.topics.motion_state),
    )

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
    hb_interval = 1.0 / max(1, cfg.motion.heartbeat_hz)
    next_hb_at = 0.0
    last_fault_log_at = 0.0

    def _stm32_keepalive(bus_stm: Stm32MotionBus, *, drain_s: float = 0.05) -> None:
        bus_stm.tick_heartbeat()
        bus_stm.pump(drain_s)

    def _recover_watchdog_fault(bus_stm: Stm32MotionBus) -> bool:
        if bus_stm.state.fault_code != 3:
            return False
        bus_stm.reset_fault()
        for _ in range(3):
            _stm32_keepalive(bus_stm, drain_s=0.08)
            time.sleep(hb_interval)
        bus_stm.pump(0.1)
        cleared = bus_stm.state.fault_code == 0
        if cleared:
            log.info("heartbeat_watchdog_recovered")
        return cleared

    try:
        if isinstance(bus, Stm32MotionBus):
            bus.pump(0.1)
            if bus.state.fault_code != 0:
                log.warning(
                    "mcu_fault_on_start",
                    fault_code=bus.state.fault_code,
                    fault_message=bus.state.fault_message,
                )
                bus.reset_fault()
                bus.pump(0.1)
            if bus.send_heartbeat(timeout_s=1.0):
                log.info("mcu_heartbeat_ok")
            else:
                log.warning("mcu_heartbeat_no_echo", port=cfg.motion.port)
            for _ in range(5):
                _stm32_keepalive(bus)
                time.sleep(hb_interval)
            log.info("heartbeat_keepalive_started", hz=cfg.motion.heartbeat_hz, interval_s=hb_interval)

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
                now = time.monotonic()
                if now >= next_hb_at:
                    _stm32_keepalive(bus)
                    next_hb_at = now + hb_interval
                else:
                    bus.pump(0.01)

            st = _fresh_state(bus)
            if isinstance(bus, Stm32MotionBus) and st.fault_code == 3:
                _recover_watchdog_fault(bus)
                st = _fresh_state(bus)

            pub.publish(st)
            if st.fault_code != 0 and time.monotonic() - last_fault_log_at > 2.0:
                log.warning(
                    "mcu_fault_telemetry",
                    fault_code=st.fault_code,
                    fault_message=st.fault_message,
                )
                last_fault_log_at = time.monotonic()

            if rep.poll(0):
                cmd = rep.recv_command(MotionCommand)
                if cmd.kind == "estop":
                    bus.estop()
                    if isinstance(bus, Stm32MotionBus):
                        bus.pump(0.05)
                    rep.send_reply(_fresh_state(bus))
                elif cmd.kind == "reset_fault" and isinstance(bus, Stm32MotionBus):
                    bus.reset_fault()
                    for _ in range(3):
                        _stm32_keepalive(bus)
                        time.sleep(hb_interval)
                    rep.send_reply(_fresh_state(bus))
                elif cmd.kind == "stream_segment" and cmd.segment is not None:
                    bus.stream_segments([cmd.segment])
                    if isinstance(bus, Stm32MotionBus):
                        bus.pump(0.05)
                    rep.send_reply(_fresh_state(bus))
                else:
                    seg = _dispatch_motion(bus, cmd)
                    if isinstance(bus, Stm32MotionBus):
                        bus.pump(0.05)
                    if seg is not None and seg != 0:
                        log.info("segment_started", segment_id=seg, cmd=cmd.kind)
                        rep.send_reply(_fresh_state(bus, segment_id_active=seg))
                    else:
                        rep.send_reply(_fresh_state(bus))
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
