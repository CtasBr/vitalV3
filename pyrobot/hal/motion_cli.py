from __future__ import annotations

import argparse
import json

from pyrobot.config.load_config import load_config
from pyrobot.hal.factory import create_motion_bus
from pyrobot.hal.stm32_motion import Stm32MotionBus


def _print_state(bus: object) -> None:
    st = getattr(bus, "state")
    print(json.dumps(st.model_dump(mode="json"), ensure_ascii=False, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description="Motion HAL CLI")
    parser.add_argument("--config", default=None, help="Path to robot.yaml")
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

    args = parser.parse_args()
    cfg = load_config(args.config)
    bus = create_motion_bus(cfg)
    try:
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
            _print_state(bus)
            return

        if args.cmd == "move-steps":
            steps = [args.a, args.b, args.c, args.d]
            if isinstance(bus, Stm32MotionBus):
                seg = bus.move_steps(steps, [args.arr] * 4)
            else:
                seg = bus.move_joints([float(v) for v in steps])
            st = bus.wait_done(seg, timeout_s=args.timeout)
            print(json.dumps(st.model_dump(mode="json"), ensure_ascii=False, indent=2))
            return

        if args.cmd == "estop":
            bus.estop()
            print("OK estop sent")
            if args.show_state:
                _print_state(bus)
            return
    finally:
        close = getattr(bus, "close", None)
        if callable(close):
            close()


if __name__ == "__main__":
    main()

