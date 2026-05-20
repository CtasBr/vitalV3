from pyrobot.hal.encoder_bus import parse_angle, transform_legacy_ab


def test_parse_angle_last_value() -> None:
    resp = "noise\nAngle:12.5\nAngle:34.0\n"
    assert parse_angle(resp) == 34.0


def test_parse_angle_missing() -> None:
    assert parse_angle("no angle here") is None


def test_transform_legacy_ab_home_like() -> None:
    # Values that map to home-ish frame after legacy transform.
    a, b = transform_legacy_ab(0.0, 0.0)
    assert a == 90.0
    assert b == 90.0
