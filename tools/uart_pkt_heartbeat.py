#!/usr/bin/env python3
"""Send PKT_HEARTBEAT and validate echo heartbeat response."""

from __future__ import annotations

import argparse
import glob
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    raise SystemExit(1) from None

PKT_MAGIC = 0x56
PKT_VERSION = 1
PKT_RAW_SIZE = 54
PKT_HEARTBEAT = 0x3F


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_raw(pkt_type: int, seq: int, payload: bytes = b"") -> bytes:
    raw = bytearray(PKT_RAW_SIZE)
    raw[0] = PKT_MAGIC
    raw[1] = PKT_VERSION
    raw[2] = pkt_type
    struct.pack_into("<H", raw, 4, seq)
    struct.pack_into("<H", raw, 6, len(payload))
    raw[8 : 8 + len(payload)] = payload[:44]
    struct.pack_into("<H", raw, 52, crc16_ccitt(raw[:52]))
    return bytes(raw)


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        b = len(out)
        out.append(0)
        code = 1
        while i < len(data) and data[i] != 0:
            out.append(data[i])
            i += 1
            code += 1
            if code == 0xFF:
                out[b] = code
                b = len(out)
                out.append(0)
                code = 1
        out[b] = code
        if i < len(data) and data[i] == 0:
            i += 1
    out.append(0)
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    r = 0
    while r < len(data):
        code = data[r]
        r += 1
        if code == 0:
            break
        for _ in range(1, code):
            if r >= len(data):
                raise ValueError("truncated cobs")
            out.append(data[r])
            r += 1
        if code < 0xFF and r < len(data):
            out.append(0)
    return bytes(out)


def read_frame(ser: serial.Serial, timeout: float = 2.0) -> bytes:
    buf = bytearray()
    end = time.time() + timeout
    while time.time() < end:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == 0:
            if buf:
                return cobs_decode(bytes(buf))
            continue
        buf.extend(b)
    raise TimeoutError("no frame")


def parse_raw(raw: bytes) -> tuple[int, int]:
    if len(raw) != PKT_RAW_SIZE or raw[0] != PKT_MAGIC:
        raise ValueError("bad raw")
    if struct.unpack_from("<H", raw, 52)[0] != crc16_ccitt(raw[:52]):
        raise ValueError("crc")
    return raw[2], struct.unpack_from("<H", raw, 4)[0]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", help="/dev/cu.usbmodem...")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--seq", type=int, default=77)
    args = ap.parse_args()

    port = args.port or (sorted(glob.glob("/dev/cu.usbmodem*"))[-1])
    payload = struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF)
    tx = cobs_encode(build_raw(PKT_HEARTBEAT, args.seq, payload))

    print(f"Using {port} @ {args.baud}")
    with serial.Serial(port, args.baud, timeout=0.5) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(tx)
        ser.flush()
        print(f"Sent PKT_HEARTBEAT seq={args.seq}")

        deadline = time.time() + 3.0
        while time.time() < deadline:
            frame = read_frame(ser, timeout=1.0)
            try:
                typ, seq = parse_raw(frame)
            except ValueError:
                continue
            if typ != PKT_HEARTBEAT:
                continue
            if seq != args.seq:
                continue
            print(f"OK: heartbeat echo seq={seq}")
            return

    print("FAIL: no heartbeat echo", file=sys.stderr)
    raise SystemExit(1)


if __name__ == "__main__":
    main()

