#!/usr/bin/env python3
"""Subscribe to encoder ZMQ topic and print states (debug)."""

from __future__ import annotations

import argparse
import json
import time

import zmq

from proto.vision import EncoderState
from pyrobot.config.load_config import load_config
from pyrobot.hal.zmq_bus import ZmqSubscriber


def main() -> None:
    parser = argparse.ArgumentParser(description="Print encoder state from ZMQ")
    parser.add_argument("--config", default=None)
    parser.add_argument("--count", type=int, default=10)
    args = parser.parse_args()

    cfg = load_config(args.config)
    ctx = zmq.Context()
    sub = ZmqSubscriber(ctx, cfg.encoders_state_uri())
    try:
        for _ in range(args.count):
            st = sub.recv_model(EncoderState, timeout_ms=2000)
            if st is None:
                print("timeout")
                continue
            print(json.dumps(st.model_dump(mode="json"), ensure_ascii=False))
            time.sleep(0.05)
    finally:
        sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
