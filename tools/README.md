# Tools

Calibration, diagnostics, simulators (to be added).

## Quick start (fake motion)

```bash
# from repo root
uv sync --extra dev
uv run robot-fake-motion   # ZMQ daemon (background)
uv run python -m pyrobot.hal.fake_motion  # inline demo
uv run pytest
```
