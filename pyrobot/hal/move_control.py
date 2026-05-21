"""Shared flags between motion_daemon move worker and UART wait loops."""

from __future__ import annotations

import threading

_cancel = threading.Event()


def clear_cancel() -> None:
    _cancel.clear()


def request_cancel() -> None:
    _cancel.set()


def is_cancelled() -> bool:
    return _cancel.is_set()
