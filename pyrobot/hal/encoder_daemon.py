from __future__ import annotations

import os
import signal
import time

import structlog
import zmq

from proto.encoders import EncoderCommand, EncoderZeroResult
from proto.vision import EncoderState
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.encoder_bus import ExternalEncoderBus, transform_legacy_ab
from pyrobot.hal.encoder_calibrate import cd_raw_from_motion_state, zero_encoders_at_pose
from pyrobot.hal.encoder_offsets import load_offsets
from pyrobot.hal.motion_zmq_client import MotionZmqClient
from pyrobot.hal.zmq_bus import ZmqPublisher, ZmqReplyServer

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


def read_encoder_state(enc: ExternalEncoderBus, cfg: RobotConfig) -> EncoderState:
    offsets = load_offsets(cfg)
    raw_a, raw_b = enc.read_raw_ab()
    ab = transform_legacy_ab(raw_a, raw_b)
    return EncoderState(
        node="encoder_daemon",
        angle_a_deg=ab[0] + offsets[0],
        angle_b_deg=ab[1] + offsets[1],
    )


def _run_zero(enc: ExternalEncoderBus, cfg: RobotConfig, *, hardware_zero: bool) -> EncoderZeroResult:
    cd_raw = [0.0, 0.0]
    offsets = load_offsets(cfg)
    try:
        with MotionZmqClient(cfg) as motion:
            st = motion.drain_state() or motion.state
            if st is not None and len(st.q_enc_deg) == 4:
                cd_raw = cd_raw_from_motion_state(st.q_enc_deg, offsets)
    except Exception as exc:
        log.warning("zero_cd_from_motion_failed", error=str(exc))

    data = zero_encoders_at_pose(cfg, enc, hardware_zero=hardware_zero, cd_raw_deg=cd_raw)
    return EncoderZeroResult(node="encoder_daemon", **data)


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
    rep = ZmqReplyServer(ctx, cfg.encoders_cmd_uri())
    period = 1.0 / max(cfg.encoders.poll_hz, 1)

    running = True
    last_good: EncoderState | None = None

    def _stop(*_args: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    log.info(
        "started",
        state=cfg.encoders_state_uri(),
        cmd=cfg.encoders_cmd_uri(),
        hz=cfg.encoders.poll_hz,
    )
    try:
        with ExternalEncoderBus(cfg) as enc:
            while running:
                if rep.poll(0):
                    cmd = rep.recv_command(EncoderCommand)
                    if cmd.kind == "zero":
                        try:
                            result = _run_zero(enc, cfg, hardware_zero=cmd.hardware_zero)
                            log.info(
                                "zero_encoders_ok",
                                offset_deg=result.offset_deg,
                                calibrated_deg=result.calibrated_deg,
                            )
                            rep.send_reply(result)
                        except Exception as exc:
                            log.error("zero_encoders_failed", error=str(exc))
                            rep.send_reply(
                                EncoderZeroResult(
                                    node="encoder_daemon",
                                    ok=False,
                                    hardware_zero=cmd.hardware_zero,
                                    home_deg=list(cfg.encoders.home_deg),
                                    robot_ab_deg=[0.0, 0.0],
                                    offset_deg=load_offsets(cfg),
                                    calibrated_deg=list(cfg.encoders.home_deg),
                                )
                            )
                    continue

                try:
                    st = read_encoder_state(enc, cfg)
                    last_good = st
                    pub.publish(st)
                except Exception as exc:
                    if last_good is not None:
                        pub.publish(last_good)
                        log.warning("encoder_read_failed_using_last", error=str(exc))
                    else:
                        log.warning("encoder_read_failed", error=str(exc))
                time.sleep(period)
    finally:
        rep.close()
        pub.close()
        ctx.term()
        log.info("stopped")


if __name__ == "__main__":
    main()
