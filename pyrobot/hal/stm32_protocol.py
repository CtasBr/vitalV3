from __future__ import annotations

import struct

PKT_MAGIC = 0x56
PKT_VERSION = 1
PKT_RAW_SIZE = 54
PKT_PAYLOAD_MAX = 44

PKT_PING = 0x01
PKT_PONG = 0x02
PKT_MOVE_SEGMENT = 0x10
PKT_TELEMETRY = 0x20
PKT_SEGMENT_DONE = 0x21
PKT_ESTOP = 0x30
PKT_FAULT = 0x31
PKT_HEARTBEAT = 0x3F


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_raw(pkt_type: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > PKT_PAYLOAD_MAX:
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


def parse_raw(raw: bytes) -> tuple[int, int, bytes]:
    if len(raw) != PKT_RAW_SIZE or raw[0] != PKT_MAGIC or raw[1] != PKT_VERSION:
        raise ValueError("bad raw packet")
    got_crc = struct.unpack_from("<H", raw, 52)[0]
    exp_crc = crc16_ccitt(raw[:52])
    if got_crc != exp_crc:
        raise ValueError("crc mismatch")
    pkt_type = raw[2]
    seq = struct.unpack_from("<H", raw, 4)[0]
    plen = struct.unpack_from("<H", raw, 6)[0]
    if plen > PKT_PAYLOAD_MAX:
        raise ValueError("bad payload len")
    return pkt_type, seq, bytes(raw[8 : 8 + plen])


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

