#!/usr/bin/env python3
"""M4: binary PKT_PING / PKT_PONG over COBS (60 B raw + CRC16-CCITT)."""

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
PKT_RAW_SIZE = 60
PKT_PING = 0x01
PKT_PONG = 0x02


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_raw(pkt_type: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > 44:
        raise ValueError("payload too long")
    raw = bytearray(PKT_RAW_SIZE)
    raw[0] = PKT_MAGIC
    raw[1] = PKT_VERSION
    raw[2] = pkt_type
    raw[3] = 0
    struct.pack_into("<H", raw, 4, seq)
    struct.pack_into("<H", raw, 6, len(payload))
    raw[8 : 8 + len(payload)] = payload
    crc = crc16_ccitt(bytes(raw[:52]))
    struct.pack_into("<H", raw, 52, crc)
    return bytes(raw)


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    read = 0
    while read < len(data):
        block_start = len(out)
        out.append(0)
        code = 1
        while read < len(data) and data[read] != 0:
            out.append(data[read])
            read += 1
            code += 1
            if code == 0xFF:
                out[block_start] = code
                block_start = len(out)
                out.append(0)
                code = 1
        out[block_start] = code
    out.append(0)
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    read = 0
    while read < len(data):
        code = data[read]
        read += 1
        if code == 0:
            break
        for _ in range(1, code):
            if read >= len(data):
                raise ValueError("truncated cobs")
            out.append(data[read])
            read += 1
        if code < 0xFF and read < len(data):
            out.append(0)
    return bytes(out)


def parse_raw(raw: bytes) -> tuple[int, int]:
    if len(raw) != PKT_RAW_SIZE or raw[0] != PKT_MAGIC:
        raise ValueError("bad raw packet")
    expect = crc16_ccitt(raw[:52])
    got = struct.unpack_from("<H", raw, 52)[0]
    if expect != got:
        raise ValueError(f"CRC mismatch {got:#06x} != {expect:#06x}")
    return raw[2], struct.unpack_from("<H", raw, 4)[0]


def read_cobs_frame(ser: serial.Serial, timeout: float = 2.0) -> bytes:
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == 0:
            if buf:
                return cobs_decode(bytes(buf))
            continue
        buf.extend(b)
    raise TimeoutError("no COBS frame")


def main() -> None:
    p = argparse.ArgumentParser()
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

    with serial.Serial(port, args.baud, timeout=0.5) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        raw = build_raw(PKT_PING, seq=42)
        wire = cobs_encode(raw)
        ser.write(wire)
        print(f"Sent PKT_PING seq=42 ({len(wire)} COBS bytes)")
        reply = read_cobs_frame(ser)
        ptype, seq = parse_raw(reply)
        if ptype != PKT_PONG or seq != 42:
            print(f"FAIL: type={ptype:#x} seq={seq}", file=sys.stderr)
            raise SystemExit(1)
        print(f"OK: PKT_PONG seq={seq}")


if __name__ == "__main__":
    main()
