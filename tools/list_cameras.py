#!/usr/bin/env python3
"""Probe OpenCV camera indices — find the right ``camera.index`` for robot.yaml.

Usage (from repo root, with vision deps installed):

    python tools/list_cameras.py
    python tools/list_cameras.py --max-index 6 --save-preview
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))


def _backends() -> list[tuple[str, int | None]]:
    import cv2

    out: list[tuple[str, int | None]] = [("default", None)]
    if sys.platform == "darwin":
        out.append(("avfoundation", int(cv2.CAP_AVFOUNDATION)))
    if sys.platform.startswith("linux"):
        out.append(("v4l2", int(cv2.CAP_V4L2)))
    return out


def probe(max_index: int, warmup_reads: int, save_preview: bool) -> int:
    try:
        import cv2
    except ImportError:
        print("Install vision deps: pip install -e '.[vision]'")
        return 1

    preview_dir = _REPO / "debug_files" / "camera_probe"
    if save_preview:
        preview_dir.mkdir(parents=True, exist_ok=True)

    print("OpenCV", cv2.__version__)
    print("Platform:", sys.platform)
    print("Probe indices 0 ..", max_index - 1)
    print()

    working: list[tuple[int, str, tuple[int, int]]] = []

    for idx in range(max_index):
        for bname, api in _backends():
            cap = cv2.VideoCapture(idx, api) if api is not None else cv2.VideoCapture(idx)
            if not cap.isOpened():
                cap.release()
                print(f"  [{idx}] backend={bname:12} — not opened")
                continue

            ok = False
            shape = (0, 0)
            for _ in range(warmup_reads):
                ok, frame = cap.read()
                if ok and frame is not None and frame.size > 0:
                    shape = (frame.shape[1], frame.shape[0])
                    break

            if ok:
                working.append((idx, bname, shape))
                line = f"  [{idx}] backend={bname:12} — OK  {shape[0]}x{shape[1]}"
                if save_preview and frame is not None:
                    path = preview_dir / f"cam{idx}_{bname}.jpg"
                    cv2.imwrite(str(path), frame)
                    line += f"  -> {path}"
                print(line)
            else:
                print(f"  [{idx}] backend={bname:12} — opened, no frames")
            cap.release()

    print()
    if not working:
        print("No working camera found.")
        print("Check: USB connected, macOS Camera permission for Terminal/Python,")
        print("       close FaceTime/Zoom using the camera, try another index.")
        return 1

    best = working[0]
    print("Suggested robot.yaml:")
    print("camera:")
    print(f"  index: {best[0]}")
    if best[1] != "default":
        print(f"  backend: {best[1]}")
    print(f"  width: {best[2][0]}")
    print(f"  height: {best[2][1]}")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe OpenCV camera indices")
    parser.add_argument("--max-index", type=int, default=8, help="Try indices 0 .. N-1")
    parser.add_argument("--warmup-reads", type=int, default=15)
    parser.add_argument("--save-preview", action="store_true", help="Save JPEG per working cam")
    args = parser.parse_args()
    raise SystemExit(probe(args.max_index, args.warmup_reads, args.save_preview))


if __name__ == "__main__":
    main()
