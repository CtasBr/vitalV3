from __future__ import annotations

import argparse
import json
import sys
import time

import zmq

from proto.motion import MotionCommand
from pyrobot.behaviour.gcode import gcode_to_motion_command
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.encoder_bus import ExternalEncoderBus, transform_legacy_ab
from pyrobot.hal.encoder_offsets import load_offsets
from pyrobot.hal.factory import create_motion_bus
from pyrobot.hal.motion_zmq_client import MotionZmqClient
from pyrobot.hal.stm32_motion import Stm32MotionBus
from pyrobot.motion.planner import feed_for_linear


def _print_json(obj: object) -> None:
    if hasattr(obj, "model_dump"):
        print(json.dumps(obj.model_dump(mode="json"), ensure_ascii=False, indent=2))
    else:
        print(json.dumps(obj, ensure_ascii=False, indent=2))


def _use_daemon(cfg: RobotConfig, direct: bool) -> bool:
    if direct:
        return False
    if cfg.motion.backend == "fake":
        return False
    return True


def _build_g28_cmd() -> MotionCommand:
    return MotionCommand(kind="home", node="motion_cli")


def _build_linear_cmd(
    x: float,
    y: float,
    z: float,
    *,
    rapid: bool,
    feed_mm_min: float | None,
) -> MotionCommand:
    return MotionCommand(
        kind="linear_move",
        node="motion_cli",
        target_pose_mm=[x, y, z],
        feed_mm_min=feed_for_linear(rapid=rapid, feed_mm_min=feed_mm_min),
        rapid=rapid,
    )


def _ensure_mcu_ready(client: MotionZmqClient, timeout_s: float = 2.0) -> MotionState:
    """Clear latched MCU fault (e.g. heartbeat watchdog) before a move."""
    st = client.drain_state()
    if st is None:
        st = client.state
    if st is None:
        raise TimeoutError("no motion.state from motion_daemon (is it running?)")
    if st.fault_code == 0:
        return st

    st = client.send_command(MotionCommand(kind="reset_fault", node="motion_cli"))
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if st.fault_code == 0:
            return st
        sample = client.state
        if sample is not None:
            st = sample
    return st


def _run_via_daemon(cfg: RobotConfig, cmd: MotionCommand, timeout_s: float) -> None:
    with MotionZmqClient(cfg) as client:
        if cmd.kind not in ("estop", "reset_fault", "state"):
            st = _ensure_mcu_ready(client)
            if st.fault_code != 0:
                print(
                    json.dumps(
                        {
                            "error": "mcu_fault_not_cleared",
                            "fault_code": st.fault_code,
                            "fault_message": st.fault_message,
                            "hint": "restart: python -m pyrobot.hal.motion_daemon",
                        },
                        ensure_ascii=False,
                        indent=2,
                    )
                )
                return

        reply = client.send_command(cmd)
        if reply.cmd_rejected:
            print(
                json.dumps(
                    {"error": "cmd_rejected", "fault_message": reply.fault_message},
                    ensure_ascii=False,
                    indent=2,
                )
            )
            return
        if cmd.kind in ("home", "g28", "linear_move", "gcode", "move_joints"):
            final = client.wait_move_busy(timeout_s=timeout_s)
            _print_json(final)
            return
        _print_json(reply)


def _run_via_bus(bus: object, cmd_name: str, args: argparse.Namespace, cfg: RobotConfig) -> None:
    if args.cmd == "ping":
        if isinstance(bus, Stm32MotionBus):
            print(bus.ping())
        else:
            print("ping unsupported for this backend")
        return

    if args.cmd == "heartbeat":
        if isinstance(bus, Stm32MotionBus):
            print(bus.send_heartbeat())
        else:
            print("heartbeat unsupported for this backend")
        return

    if args.cmd == "state":
        _print_json(bus.state)
        return

    if args.cmd == "move-steps":
        steps = [args.a, args.b, args.c, args.d]
        if isinstance(bus, Stm32MotionBus):
            seg = bus.move_steps(steps, [args.arr] * 4)
        else:
            seg = bus.move_joints([float(v) for v in steps])
        st = bus.wait_done(seg, timeout_s=args.timeout)
        _print_json(st)
        return

    if args.cmd == "estop":
        bus.estop()
        print("OK estop sent")
        if args.show_state:
            _print_json(bus.state)
        return

    if args.cmd == "reset-fault":
        if isinstance(bus, Stm32MotionBus):
            print(bus.reset_fault())
        else:
            reset = getattr(bus, "reset_fault", None)
            if callable(reset):
                reset()
                print(True)
            else:
                print("reset-fault unsupported for this backend")
        return

    if args.cmd == "zero-encoders":
        if isinstance(bus, Stm32MotionBus):
            result = bus.zero_encoders(hardware_zero=not args.no_hardware_zero)
            _print_json(result)
            _print_json(bus.state)
        else:
            print("zero-encoders unsupported for this backend")
        return

    if args.cmd == "g28":
        home = getattr(bus, "home", None)
        if not callable(home):
            print("g28/home unsupported for this backend")
            return
        seg = home()
        st = bus.wait_done(seg, timeout_s=30.0)
        _print_json(st)
        return

    if args.cmd in ("g0", "g1"):
        move_pose = getattr(bus, "move_to_pose_mm", None)
        if not callable(move_pose):
            print("linear move unsupported for this backend")
            return
        rapid = args.cmd == "g0"
        feed = None if rapid else args.f
        seg = move_pose(
            [args.x, args.y, args.z],
            feed_mm_min=feed_for_linear(rapid=rapid, feed_mm_min=feed),
            rapid=rapid,
        )
        st = bus.wait_done(seg, timeout_s=args.timeout)
        _print_json(st)
        return

    if args.cmd == "gcode":
        pose_fn = getattr(bus, "current_pose_mm", None)
        pose_mm = pose_fn() if callable(pose_fn) else [250.0, 0.0, 250.0]
        sub = gcode_to_motion_command(
            args.line,
            current_pose_mm=pose_mm,
            current_q_deg=bus.state.q_enc_deg,
        )
        if sub is None:
            print("unsupported or empty gcode line")
            return
        if sub.kind in ("home", "g28"):
            seg = bus.home(feed_mm_min=sub.feed_mm_min)  # type: ignore[attr-defined]
        elif sub.target_pose_mm is not None:
            move_pose = getattr(bus, "move_to_pose_mm", None)
            if not callable(move_pose):
                print("linear move unsupported")
                return
            feed = feed_for_linear(rapid=sub.rapid, feed_mm_min=sub.feed_mm_min)
            d_tgt = sub.target_q_deg[3] if sub.target_q_deg else None
            seg = move_pose(
                sub.target_pose_mm,
                feed_mm_min=feed,
                rapid=sub.rapid,
                d_target_deg=d_tgt,
            )
        else:
            print("gcode did not produce a motion command")
            return
        st = bus.wait_done(seg, timeout_s=args.timeout)
        _print_json(st)
        return


def _daemon_command(args: argparse.Namespace, cfg: RobotConfig) -> MotionCommand | None:
    if args.cmd == "estop":
        return MotionCommand(kind="estop", node="motion_cli")
    if args.cmd == "reset-fault":
        return MotionCommand(kind="reset_fault", node="motion_cli")
    if args.cmd == "g28":
        return _build_g28_cmd()
    if args.cmd == "g0":
        return _build_linear_cmd(args.x, args.y, args.z, rapid=True, feed_mm_min=None)
    if args.cmd == "g1":
        return _build_linear_cmd(args.x, args.y, args.z, rapid=False, feed_mm_min=args.f)
    if args.cmd == "gcode":
        st_client = MotionZmqClient(cfg)
        try:
            pose_mm = [250.0, 0.0, 250.0]
            cur = st_client.state
            if cur is not None and len(cur.q_enc_deg) == 4:
                q = cur.q_enc_deg
                from pyrobot.kinematics.forward import joint_deg_to_pose

                p = joint_deg_to_pose(
                    q[0], q[1], q[2], link_length_mm=cfg.kinematics.link_length_mm
                )
                pose_mm = [p.x, p.y, p.z]
                q_deg = q
            else:
                q_deg = [90.0, 90.0, 0.0, 0.0]
            return gcode_to_motion_command(
                args.line,
                current_pose_mm=pose_mm,
                current_q_deg=q_deg,
            )
        finally:
            st_client.close()
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Motion HAL CLI")
    parser.add_argument("--config", default=None, help="Path to robot.yaml")
    parser.add_argument(
        "--direct",
        action="store_true",
        help="Open STM32 UART directly (stop motion_daemon first)",
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping")
    sub.add_parser("heartbeat")
    sub.add_parser("state")

    p_move = sub.add_parser("move-steps")
    p_move.add_argument("a", type=int)
    p_move.add_argument("b", type=int)
    p_move.add_argument("c", type=int)
    p_move.add_argument("d", type=int)
    p_move.add_argument("--arr", type=int, default=5000)
    p_move.add_argument("--timeout", type=float, default=15.0)

    p_estop = sub.add_parser("estop")
    p_estop.add_argument("--show-state", action="store_true")
    sub.add_parser("reset-fault")
    p_zero = sub.add_parser("zero-encoders")
    p_zero.add_argument(
        "--no-hardware-zero",
        action="store_true",
        help="Skip AT+ZERO on encoder devices (software offset only)",
    )
    sub.add_parser("enc-state")

    sub.add_parser("g28")
    p_g1 = sub.add_parser("g1", help="Linear move to X Y Z (mm), G1 feed")
    p_g1.add_argument("x", type=float)
    p_g1.add_argument("y", type=float)
    p_g1.add_argument("z", type=float)
    p_g1.add_argument("--f", type=float, default=300.0, help="Feed mm/min")
    p_g1.add_argument("--timeout", type=float, default=30.0)

    p_g0 = sub.add_parser("g0", help="Rapid linear move to X Y Z (mm)")
    p_g0.add_argument("x", type=float)
    p_g0.add_argument("y", type=float)
    p_g0.add_argument("z", type=float)
    p_g0.add_argument("--timeout", type=float, default=30.0)

    p_gc = sub.add_parser("gcode", help="Run one G-code line (G0/G1/G28)")
    p_gc.add_argument("line", type=str)
    p_gc.add_argument("--timeout", type=float, default=30.0)

    args = parser.parse_args()
    cfg = load_config(args.config)

    if args.cmd == "enc-state":
        offsets = load_offsets(cfg)
        with ExternalEncoderBus(cfg) as enc:
            raw_a, raw_b = enc.read_raw_ab()
            ab = transform_legacy_ab(raw_a, raw_b)
        calibrated = [ab[0] + offsets[0], ab[1] + offsets[1]]
        _print_json(
            {
                "raw_deg": [raw_a, raw_b],
                "robot_ab_deg": [ab[0], ab[1]],
                "offset_ab_deg": [offsets[0], offsets[1]],
                "calibrated_ab_deg": calibrated,
                "home_ab_deg": [cfg.encoders.home_deg[0], cfg.encoders.home_deg[1]],
                "note": "calibrated_ab_deg = robot_ab_deg + offset_ab_deg; use zero-encoders at home pose",
            }
        )
        return

    daemon_cmds = {"g28", "g0", "g1", "gcode", "estop", "reset-fault", "state"}
    use_daemon = _use_daemon(cfg, args.direct)

    if use_daemon and args.cmd in daemon_cmds:
        if args.cmd == "state":
            with MotionZmqClient(cfg) as client:
                st = client.state
                if st is None:
                    print("timeout waiting for motion.state", file=sys.stderr)
                    sys.exit(1)
                _print_json(st)
            return
        cmd = _daemon_command(args, cfg)
        if cmd is None:
            print(f"command {args.cmd} not supported via daemon", file=sys.stderr)
            sys.exit(1)
        timeout = getattr(args, "timeout", 30.0)
        try:
            _run_via_daemon(cfg, cmd, timeout_s=timeout)
        except zmq.error.Again:
            print(
                "motion_daemon not responding on ZMQ (timeout 10s). "
                "If it is running, restart it (Ctrl+C): python -m pyrobot.hal.motion_daemon. "
                "Then retry the command.",
                file=sys.stderr,
            )
            sys.exit(1)
        return

    if use_daemon and args.cmd not in daemon_cmds:
        print(
            f"'{args.cmd}' needs --direct (UART). Stop motion_daemon or use move via ZMQ later.",
            file=sys.stderr,
        )
        sys.exit(1)

    bus = create_motion_bus(cfg)
    try:
        _run_via_bus(bus, args.cmd, args, cfg)
    except Exception as exc:
        if "multiple access on port" in str(exc).lower() or "device disconnected" in str(exc).lower():
            print(
                "STM32 port busy (motion_daemon running?). "
                "Use daemon mode (default) or stop motion_daemon and retry with --direct.",
                file=sys.stderr,
            )
        raise
    finally:
        close = getattr(bus, "close", None)
        if callable(close):
            close()


if __name__ == "__main__":
    main()
