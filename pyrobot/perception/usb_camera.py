from __future__ import annotations

import sys
import threading
import time
from typing import Any

import numpy as np

# macOS: default backend often fails; AVFoundation is reliable for USB/FaceTime cams.
_BACKEND_ALIASES: dict[str, int | None] = {
    "default": None,
    "any": None,
    "avfoundation": 1200,  # cv2.CAP_AVFOUNDATION (when cv2 not imported yet)
    "v4l2": 200,  # cv2.CAP_V4L2 — Linux
}


def _resolve_backend(name: str | None) -> int | None:
    import cv2

    if name is None or name == "auto":
        if sys.platform == "darwin":
            return int(cv2.CAP_AVFOUNDATION)
        return None
    key = name.strip().lower()
    if key not in _BACKEND_ALIASES:
        raise ValueError(f"unknown camera backend: {name!r} (use auto, avfoundation, v4l2, default)")
    api = _BACKEND_ALIASES[key]
    if api is None:
        return None
    return int(api)


def open_video_capture(
    camera_index: int,
    *,
    width: int,
    height: int,
    backend: str | None = "auto",
) -> Any:
    """Open camera and verify at least one frame can be read."""
    import cv2

    api = _resolve_backend(backend)
    cap = cv2.VideoCapture(camera_index, api) if api is not None else cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"camera index {camera_index} did not open (backend={backend!r})")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    ok = False
    frame: np.ndarray | None = None
    for _ in range(30):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            break
        time.sleep(0.05)

    if not ok or frame is None:
        cap.release()
        raise RuntimeError(
            f"camera index {camera_index} opened but no frames "
            f"(backend={backend!r}, try tools/list_cameras.py)"
        )
    return cap


class UsbCameraCapture:
    """Background USB camera capture (OpenCV)."""

    def __init__(
        self,
        camera_index: int = 0,
        width: int = 1920,
        height: int = 1080,
        *,
        backend: str | None = "auto",
    ) -> None:
        import cv2

        self._cv2 = cv2
        self.index = camera_index
        self.backend = backend
        self.cap = open_video_capture(
            camera_index, width=width, height=height, backend=backend
        )
        self._latest: np.ndarray | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._fail_reads = 0
        self._thread = threading.Thread(target=self._loop, daemon=True, name="usb-cam")

    def start(self) -> None:
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if ok and frame is not None and frame.size > 0:
                with self._lock:
                    self._latest = frame
                self._fail_reads = 0
            else:
                self._fail_reads += 1
            time.sleep(0.03)

    def get_latest_frame(self) -> np.ndarray | None:
        with self._lock:
            if self._latest is None:
                return None
            return self._latest.copy()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.cap.release()
