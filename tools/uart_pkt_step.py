#!/usr/bin/env python3
"""M5: send PKT_MOVE_SEGMENT (4-axis payload) and wait PKT_SEGMENT_DONE."""

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
PKT_MOVE_SEGMENT = 0x10
PKT_SEGMENT_DONE = 0x21
PKT_TELEMETRY = 0x20
PKT_FAULT = 0x31


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
    struct.pack_into("<H", raw, 52, crc16_ccitt(raw[:52]))
    return bytes(raw)


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        block_start = len(out)
        out.append(0)
        code = 1
        while i < len(data) and data[i] != 0:
            out.append(data[i])
            i += 1
            code += 1
            if code == 0xFF:
                out[block_start] = code
                block_start = len(out)
                out.append(0)
                code = 1
        out[block_start] = code
        if i < len(data) and data[i] == 0:
            i += 1
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


def parse_raw(raw: bytes) -> tuple[int, int, bytes]:
    if len(raw) != PKT_RAW_SIZE or raw[0] != PKT_MAGIC:
        raise ValueError("bad raw packet")
    got_crc = struct.unpack_from("<H", raw, 52)[0]
    exp_crc = crc16_ccitt(raw[:52])
    if got_crc != exp_crc:
        raise ValueError("crc mismatch")
    ptype = raw[2]
    seq = struct.unpack_from("<H", raw, 4)[0]
    plen = struct.unpack_from("<H", raw, 6)[0]
    return ptype, seq, bytes(raw[8 : 8 + plen])


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("steps_a", type=int)
    p.add_argument("arr_a", type=int, nargs="?", default=5000)
    p.add_argument("--steps-b", type=int, default=0)
    p.add_argument("--steps-c", type=int, default=0)
    p.add_argument("--steps-d", type=int, default=0)
    p.add_argument("--arr-b", type=int, default=5000)
    p.add_argument("--arr-c", type=int, default=5000)
    p.add_argument("--arr-d", type=int, default=5000)
    p.add_argument("port", nargs="?", help="/dev/cu.usbmodem...")
    p.add_argument("-b", "--baud", type=int, default=115200)
    p.add_argument("--seq", type=int, default=100)
    args = p.parse_args()

    port = args.port
    if not port:
        ports = sorted(glob.glob("/dev/cu.usbmodem*"))
        if not ports:
            print("No /dev/cu.usbmodem* found", file=sys.stderr)
            raise SystemExit(1)
        port = ports[-1]

    # 4-axis payload (B/C/D = 0 for current firmware stage)
    payload = struct.pack(
        "<iiiiIIII",
        args.steps_a,
        args.steps_b,
        args.steps_c,
        args.steps_d,
        args.arr_a,
        args.arr_b,
        args.arr_c,
        args.arr_d,
    )
    raw = build_raw(PKT_MOVE_SEGMENT, seq=args.seq, payload=payload)
    wire = cobs_encode(raw)

    print(f"Using {port} @ {args.baud}")
    with serial.Serial(port, args.baud, timeout=0.5) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(wire)
        ser.flush()
        print(
            f"Sent PKT_MOVE_SEGMENT seq={args.seq}, "
            f"steps=[{args.steps_a},{args.steps_b},{args.steps_c},{args.steps_d}], "
            f"arr=[{args.arr_a},{args.arr_b},{args.arr_c},{args.arr_d}]"
        )

        deadline = time.time() + 35.0
        while time.time() < deadline:
            try:
                frame = read_cobs_frame(ser, timeout=1.0)
            except TimeoutError:
                continue
            except ValueError as exc:
                # Partial/garbled frame may appear on USB CDC; keep waiting.
                print(f"skip frame (decode): {exc}")
                continue

            try:
                ptype, seq, payload_rx = parse_raw(frame)
            except ValueError as exc:
                print(f"skip frame (raw): {exc}")
                continue
            if ptype == PKT_TELEMETRY:
                continue
            if ptype == PKT_FAULT:
                code = struct.unpack("<i", payload_rx[:4])[0] if len(payload_rx) >= 4 else 0
                print(f"FAIL: PKT_FAULT seq={seq}, code={code}", file=sys.stderr)
                raise SystemExit(1)
            if ptype != PKT_SEGMENT_DONE:
                continue
            if seq != args.seq:
                print(f"skip done with seq={seq}")
                continue
            done = struct.unpack("<iiii", payload_rx[:16]) if len(payload_rx) >= 16 else (0, 0, 0, 0)
            print(f"OK: PKT_SEGMENT_DONE seq={seq}, done_steps={list(done)}")
            return

    print("FAIL: no PKT_SEGMENT_DONE", file=sys.stderr)
    raise SystemExit(1)


if __name__ == "__main__":
    main()

