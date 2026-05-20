from pyrobot.hal.stm32_protocol import (
    PKT_MOVE_SEGMENT,
    PKT_PING,
    build_raw,
    cobs_decode,
    cobs_encode,
    parse_raw,
)


def test_raw_roundtrip_ping() -> None:
    raw = build_raw(PKT_PING, seq=123, payload=b"")
    typ, seq, payload = parse_raw(raw)
    assert typ == PKT_PING
    assert seq == 123
    assert payload == b""


def test_cobs_roundtrip_with_zeros() -> None:
    payload = bytes([0, 1, 2, 0, 3, 4, 0, 5])
    raw = build_raw(PKT_MOVE_SEGMENT, seq=7, payload=payload)
    wire = cobs_encode(raw)
    # trailing frame delimiter expected
    assert wire[-1] == 0
    dec = cobs_decode(wire[:-1])
    assert dec == raw

