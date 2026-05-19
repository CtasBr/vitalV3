#!/usr/bin/env python3
"""M3: send STEP command to vital_motion firmware."""

from __future__ import annotations

import argparse
import glob
import sys
import time

try:
    import serial
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    raise SystemExit(1) from None


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("steps", type=int, help="step count, negative = reverse")
    p.add_argument("arr", type=int, nargs="?", default=5000, help="TIM1 ARR (speed)")
    p.add_argument("port", nargs="?", help="/dev/cu.usbmodem...")
    p.add_argument("-b", "--baud", type=int, default=115200)
    args = p.parse_args()

    port = args.port
    if not port:
        ports = sorted(glob.glob("/dev/cu.usbmodem*"))
        if not ports:
            print("No usbmodem port", file=sys.stderr)
            raise SystemExit(1)
        port = ports[0]

    cmd = f"STEP {args.steps} {args.arr}\n".encode()
    with serial.Serial(port, args.baud, timeout=3.0) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(cmd)
        print(f"Sent: {cmd.decode().strip()}")
        deadline = time.time() + 35.0
        while time.time() < deadline:
            line = ser.readline().decode("utf-8", errors="replace")
            if not line:
                continue
            print(line, end="")
            if "OK STEP" in line:
                print("Done.")
                return
            if "ERR" in line:
                raise SystemExit(1)
    print("Timeout waiting for OK STEP", file=sys.stderr)
    raise SystemExit(1)


if __name__ == "__main__":
    main()
