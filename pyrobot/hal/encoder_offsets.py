from __future__ import annotations

import json
from pathlib import Path

from pyrobot.config.load_config import RobotConfig

_DEFAULT_OFFSETS = [0.0, 0.0, 0.0, 0.0]


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def offsets_path(cfg: RobotConfig) -> Path:
    p = Path(cfg.encoders.offsets_file)
    if p.is_absolute():
        return p
    return _repo_root() / p


def load_offsets(cfg: RobotConfig) -> list[float]:
    path = offsets_path(cfg)
    if not path.exists():
        return list(_DEFAULT_OFFSETS)
    raw = json.loads(path.read_text(encoding="utf-8"))
    vals = raw.get("offset_deg", _DEFAULT_OFFSETS)
    if not isinstance(vals, list) or len(vals) != 4:
        return list(_DEFAULT_OFFSETS)
    return [float(v) for v in vals]


def save_offsets(cfg: RobotConfig, offset_deg: list[float]) -> None:
    if len(offset_deg) != 4:
        raise ValueError("offset_deg must have length 4")
    path = offsets_path(cfg)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps({"offset_deg": [float(v) for v in offset_deg]}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

