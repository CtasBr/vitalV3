from pathlib import Path

from pyrobot.config.load_config import load_config
from pyrobot.hal.encoder_offsets import load_offsets, save_offsets


def test_encoder_offsets_roundtrip(tmp_path: Path) -> None:
    cfg = load_config()
    cfg.encoders.offsets_file = str(tmp_path / "enc_offsets.json")
    assert load_offsets(cfg) == [0.0, 0.0, 0.0, 0.0]
    save_offsets(cfg, [1.0, -2.5, 0.0, 90.0])
    assert load_offsets(cfg) == [1.0, -2.5, 0.0, 90.0]

