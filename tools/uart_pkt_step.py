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
PKT_RESET_FAULT = 0x32
PKT_HEARTBEAT = 0x3F
PKT_ESTOP = 0x30
COBS_BUF_MAX = 200


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
    struct.pack_into("<H", raw, 4, seq & 0xFFFF)
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


def drain_rx(ser: serial.Serial, duration_s: float = 0.25) -> int:
    """Drop telemetry/leftovers so the next COBS frame starts clean."""
    end = time.time() + duration_s
    n = 0
    old_timeout = ser.timeout
    ser.timeout = 0.02
    try:
        while time.time() < end:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                n += len(chunk)
            else:
                time.sleep(0.01)
    finally:
        ser.timeout = old_timeout
    if ser.in_waiting:
        n += len(ser.read(ser.in_waiting))
    return n


def send_packet(ser: serial.Serial, pkt_type: int, seq: int, payload: bytes = b"") -> None:
    wire = cobs_encode(build_raw(pkt_type, seq, payload))
    ser.write(wire)
    ser.flush()


def wait_for_done(
    ser: serial.Serial,
    want_seq: int,
    timeout_s: float,
    *,
    verbose: bool,
) -> tuple[int, tuple[int, int, int, int]]:
    """Read UART until PKT_SEGMENT_DONE(want_seq) or PKT_FAULT."""
    buf = bytearray()
    deadline = time.time() + timeout_s
    last_progress = time.time()
    n_telemetry = 0
    n_skip = 0
    old_timeout = ser.timeout
    ser.timeout = 0.05
    try:
        while time.time() < deadline:
            b = ser.read(1)
            if not b:
                if verbose and time.time() - last_progress > 5.0:
                    left = max(0.0, deadline - time.time())
                    print(
                        f"  … waiting ({left:.0f}s left, "
                        f"telemetry={n_telemetry}, skip={n_skip})"
                    )
                    last_progress = time.time()
                continue
            if b[0] == 0:
                if not buf:
                    continue
                frame = bytes(buf)
                buf.clear()
                try:
                    raw = cobs_decode(frame)
                    ptype, seq, payload_rx = parse_raw(raw)
                except ValueError as exc:
                    n_skip += 1
                    if verbose:
                        print(f"skip frame: {exc}")
                    continue
                if ptype == PKT_TELEMETRY:
                    n_telemetry += 1
                    continue
                if ptype == PKT_FAULT:
                    code = struct.unpack("<i", payload_rx[:4])[0] if len(payload_rx) >= 4 else 0
                    raise RuntimeError(f"PKT_FAULT seq={seq} code={code}")
                if ptype != PKT_SEGMENT_DONE:
                    if verbose:
                        print(f"skip pkt type=0x{ptype:02x} seq={seq}")
                    continue
                if seq != want_seq:
                    if verbose:
                        print(f"skip done seq={seq} (want {want_seq})")
                    continue
                done = (
                    struct.unpack("<iiii", payload_rx[:16])
                    if len(payload_rx) >= 16
                    else (0, 0, 0, 0)
                )
                if verbose and (n_telemetry or n_skip):
                    print(f"  rx stats: telemetry={n_telemetry}, skip={n_skip}")
                return seq, done
            buf.extend(b)
            if len(buf) > COBS_BUF_MAX:
                n_skip += 1
                if verbose:
                    print("COBS resync: buffer overflow, clearing")
                buf.clear()
    finally:
        ser.timeout = old_timeout
    hint = ""
    if n_telemetry > 0 and n_skip == 0:
        hint = " (MCU alive but no DONE — motor stuck? reflash + ESTOP)"
    raise TimeoutError(
        f"no PKT_SEGMENT_DONE seq={want_seq} within {timeout_s:.0f}s{hint}"
    )


def main() -> None:
    p = argparse.ArgumentParser(
        description="Send one MOVE_SEGMENT and wait SEGMENT_DONE (bringup test)."
    )
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
    p.add_argument(
        "--seq",
        type=int,
        default=None,
        help="packet seq (default: auto from time)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=65.0,
        help="wait for SEGMENT_DONE (MCU up to ~30s per moving axis)",
    )
    p.add_argument(
        "--no-drain",
        action="store_true",
        help="do not flush RX before/after (not recommended)",
    )
    p.add_argument(
        "--no-reset",
        action="store_true",
        help="skip PKT_RESET_FAULT before move",
    )
    p.add_argument(
        "--no-estop",
        action="store_true",
        help="skip PKT_ESTOP before move (not recommended after direction change)",
    )
    p.add_argument(
        "--pause-ms",
        type=int,
        default=300,
        help="pause after prep before MOVE (DIR/setup)",
    )
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    port = args.port
    if not port:
        ports = sorted(glob.glob("/dev/cu.usbmodem*"))
        if not ports:
            print("No /dev/cu.usbmodem* found", file=sys.stderr)
            raise SystemExit(1)
        port = ports[-1]

    seq = args.seq if args.seq is not None else (int(time.time() * 1000) & 0xFFFF)
    if seq == 0:
        seq = 1

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

    print(f"Using {port} @ {args.baud}")
    print("Tip: stop motion_daemon first — only one process may use the port.")
    try:
        with serial.Serial(port, args.baud, timeout=0.5) as ser:
            time.sleep(0.15)
            if not args.no_drain:
                dropped = drain_rx(ser, 0.3)
                if args.verbose and dropped:
                    print(f"drained {dropped} stale RX bytes")
                ser.reset_input_buffer()

            if not args.no_estop:
                send_packet(ser, PKT_ESTOP, seq=seq - 3)
                drain_rx(ser, 0.25)
                if args.verbose:
                    print("sent PKT_ESTOP (clear running/timers)")

            if not args.no_reset:
                send_packet(ser, PKT_RESET_FAULT, seq=seq - 1)
                drain_rx(ser, 0.2)
                send_packet(
                    ser,
                    PKT_HEARTBEAT,
                    seq=seq - 2,
                    payload=struct.pack("<I", int(time.time() * 1000) & 0xFFFFFFFF),
                )
                drain_rx(ser, 0.1)

            if args.pause_ms > 0:
                time.sleep(args.pause_ms / 1000.0)

            send_packet(ser, PKT_MOVE_SEGMENT, seq=seq, payload=payload)
            print(
                f"Sent PKT_MOVE_SEGMENT seq={seq}, "
                f"steps=[{args.steps_a},{args.steps_b},{args.steps_c},{args.steps_d}], "
                f"arr=[{args.arr_a},{args.arr_b},{args.arr_c},{args.arr_d}]"
            )

            rx_seq, done = wait_for_done(
                ser, seq, timeout_s=args.timeout, verbose=args.verbose
            )
            print(f"OK: PKT_SEGMENT_DONE seq={rx_seq}, done_steps={list(done)}")
            if not args.no_drain:
                drain_rx(ser, 0.2)
    except serial.SerialException as exc:
        print(f"FAIL: serial error: {exc}", file=sys.stderr)
        print("Is python -m pyrobot.launcher start still running?", file=sys.stderr)
        raise SystemExit(1) from exc
    except TimeoutError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        print(
            "MCU may still be moving or stuck. Retry after reset, or send ESTOP.",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc
    except RuntimeError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
