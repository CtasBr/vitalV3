from __future__ import annotations

from pathlib import Path


class FrameStore:
    """Atomic JPEG files for cross-process UI preview."""

    def __init__(self, directory: str | Path) -> None:
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)

    def write_jpeg(self, name: str, data: bytes) -> None:
        path = self.dir / f"{name}.jpg"
        tmp = path.with_suffix(".tmp")
        tmp.write_bytes(data)
        tmp.replace(path)

    def read_jpeg(self, name: str) -> bytes | None:
        path = self.dir / f"{name}.jpg"
        if not path.is_file():
            return None
        return path.read_bytes()
