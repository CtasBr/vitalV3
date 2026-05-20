from __future__ import annotations

import re
import time

import serial

from pyrobot.config.load_config import RobotConfig, load_config

_ANGLE_RE = re.compile(r"Angle:([\d.\-]+)")


def parse_angle(response: str) -> float | None:
    """Extract last Angle:value from encoder AT response."""
    angles = _ANGLE_RE.findall(response)
    if not angles:
        return None
    try:
        return float(angles[-1])
    except ValueError:
        return None


def transform_legacy_ab(raw_a: float, raw_b: float) -> tuple[float, float]:
    """
    Legacy vitalSoft conversion from device angles to robot A/B degrees.
    Target home pose in this frame is approximately A=90, B=90.
    """
    a = raw_a
    b = raw_b
    if 180 < a <= 360:
        a -= 360
    b = 360 - b
    if 180 < b <= 360:
        b -= 360
    b = 90 - (b - a)
    a = 90 - a
    return a, b


class ExternalEncoderBus:
    """Read axes A/B from external UART encoders (AT protocol)."""

    def __init__(self, config: RobotConfig | None = None, command_wait_s: float = 0.05) -> None:
        self._cfg = config or load_config()
        self._wait_s = command_wait_s
        self._ser_a: serial.Serial | None = None
        self._ser_b: serial.Serial | None = None
        self._initialized = False

    def _ensure_open(self) -> None:
        if self._initialized:
            return
        self._ser_a = serial.Serial(self._cfg.encoders.port_a, self._cfg.encoders.baudrate, timeout=0.2)
        self._ser_b = serial.Serial(self._cfg.encoders.port_b, self._cfg.encoders.baudrate, timeout=0.2)
        time.sleep(0.2)
        self._send_at_both("AT+PRATE=0")
        self._initialized = True

    def _send_at_both(self, command: str) -> tuple[str, str]:
        assert self._ser_a is not None and self._ser_b is not None
        payload = (command + "\r\n").encode()
        self._ser_a.reset_input_buffer()
        self._ser_b.reset_input_buffer()
        self._ser_a.write(payload)
        self._ser_b.write(payload)
        time.sleep(self._wait_s)
        resp_a = self._ser_a.read_all().decode("utf-8", errors="ignore")
        resp_b = self._ser_b.read_all().decode("utf-8", errors="ignore")
        return resp_a, resp_b

    def read_raw_ab(self, retries: int = 5) -> tuple[float, float]:
        """Return raw device angles (before legacy transform)."""
        self._ensure_open()
        last_err: str | None = None
        for _ in range(retries):
            resp_a, resp_b = self._send_at_both("AT+PRATE=0")
            raw_a = parse_angle(resp_a)
            raw_b = parse_angle(resp_b)
            if raw_a is not None and raw_b is not None:
                return raw_a, raw_b
            last_err = f"parse failed: a={resp_a!r} b={resp_b!r}"
            time.sleep(0.05)
        raise RuntimeError(last_err or "encoder read failed")

    def read_ab(self, retries: int = 5) -> tuple[float, float]:
        """Return robot-frame A/B degrees (legacy transform applied)."""
        raw_a, raw_b = self.read_raw_ab(retries=retries)
        return transform_legacy_ab(raw_a, raw_b)

    def hardware_zero(self, settle_s: float = 1.0) -> None:
        """Tell both encoders that current pose is mechanical zero (AT+ZERO)."""
        self._ensure_open()
        self._send_at_both("AT+ZERO")
        time.sleep(settle_s)
        # Keep periodic mode disabled after zeroing.
        self._send_at_both("AT+PRATE=0")

    def close(self) -> None:
        for ser in (self._ser_a, self._ser_b):
            if ser is not None and ser.is_open:
                ser.close()
        self._ser_a = None
        self._ser_b = None
        self._initialized = False

    def __enter__(self) -> ExternalEncoderBus:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
