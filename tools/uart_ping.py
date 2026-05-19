#!/usr/bin/env python3
"""M2 test: echo + PING/PONG на motion UART (на NUCLEO = /dev/cu.usbmodem*)."""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    raise SystemExit(1) from None


def main() -> None:
    p = argparse.ArgumentParser(description="vital_motion M2 UART test")
    p.add_argument(
        "port",
        nargs="?",
        help="Serial port, e.g. /dev/cu.usbmodem14103",
    )
    p.add_argument("-b", "--baud", type=int, default=115200)
    args = p.parse_args()

    port = args.port
    if not port:
        import glob

        candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
        if not candidates:
            print("No /dev/cu.usbmodem* — pass port explicitly", file=sys.stderr)
            raise SystemExit(1)
        port = candidates[0]
        print(f"Using {port}")

    with serial.Serial(port, args.baud, timeout=0.5) as ser:
        time.sleep(0.3)
        if ser.in_waiting:
            print("--- boot ---")
            print(ser.read(ser.in_waiting).decode("utf-8", errors="replace"), end="")

        ser.write(b"PING\n")
        time.sleep(0.1)
        reply = ser.read(64).decode("utf-8", errors="replace")
        print(f"PING -> {reply!r}")
        if "PONG" not in reply:
            print("FAIL: expected PONG", file=sys.stderr)
            raise SystemExit(1)

        ser.write(b"ABC")
        time.sleep(0.1)
        echo = ser.read(16)
        print(f"echo ABC -> {echo!r}")
        if echo != b"ABC":
            print("FAIL: echo mismatch", file=sys.stderr)
            raise SystemExit(1)

    print("OK")


if __name__ == "__main__":
    main()
