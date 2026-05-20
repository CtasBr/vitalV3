from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from proto.motion import MotionCommand
from pyrobot.behaviour.gcode import gcode_to_motion_command
from pyrobot.config.load_config import load_config
from pyrobot.hal.motion_zmq_client import MotionZmqClient
from pyrobot.ui.state_cache import RobotStateCache

_STATIC = Path(__file__).resolve().parent / "static"


class GcodeBody(BaseModel):
    line: str = Field(min_length=1)


def create_app(cache: RobotStateCache | None = None) -> FastAPI:
    cfg = load_config()
    state_cache = cache or RobotStateCache(cfg)
    if cache is None:
        state_cache.start()

    app = FastAPI(title="vitalV3 UI", version="0.1.0")

    @app.get("/api/state")
    def api_state() -> dict[str, Any]:
        return state_cache.snapshot()

    def _send_motion(cmd: MotionCommand) -> dict[str, Any]:
        try:
            with MotionZmqClient(cfg) as client:
                snap = state_cache.snapshot()
                motion = snap["motion"]
                if cmd.kind not in ("estop", "reset_fault") and motion.get("fault_code", 0) != 0:
                    client.send_command(MotionCommand(kind="reset_fault", node="web"))
                if cmd.kind == "gcode" and cmd.gcode_line:
                    sub = gcode_to_motion_command(
                        cmd.gcode_line,
                        current_pose_mm=[
                            snap["pose_mm"]["x"],
                            snap["pose_mm"]["y"],
                            snap["pose_mm"]["z"],
                        ],
                        current_q_deg=[
                            motion["q_enc_deg"][0],
                            motion["q_enc_deg"][1],
                            motion["q_enc_deg"][2],
                            motion["q_enc_deg"][3],
                        ],
                    )
                    if sub is None:
                        raise HTTPException(400, "unsupported gcode line")
                    cmd = sub
                reply = client.send_command(cmd.model_copy(update={"node": "web"}))
                seg = reply.segment_id_active
                if seg is not None and seg != 0:
                    final = client.wait_done(seg, timeout_s=120.0)
                    return {"ok": True, "segment_id": seg, "motion": final.model_dump(mode="json")}
                return {"ok": True, "motion": reply.model_dump(mode="json")}
        except TimeoutError as exc:
            raise HTTPException(504, f"motion timeout: {exc}") from exc
        except Exception as exc:
            raise HTTPException(503, f"motion daemon unavailable: {exc}") from exc

    @app.post("/api/gcode")
    def api_gcode(body: GcodeBody) -> dict[str, Any]:
        return _send_motion(MotionCommand(kind="gcode", gcode_line=body.line.strip(), node="web"))

    @app.post("/api/home")
    def api_home() -> dict[str, Any]:
        return _send_motion(MotionCommand(kind="home", node="web"))

    @app.post("/api/reset-fault")
    def api_reset_fault() -> dict[str, Any]:
        return _send_motion(MotionCommand(kind="reset_fault", node="web"))

    @app.post("/api/estop")
    def api_estop() -> dict[str, Any]:
        return _send_motion(MotionCommand(kind="estop", node="web"))

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(_STATIC / "index.html")

    app.mount("/static", StaticFiles(directory=_STATIC), name="static")
    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="vitalV3 web UI")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--config", default=None)
    args = parser.parse_args()

    cfg = load_config(args.config)
    host = args.host or cfg.ui.host
    port = args.port or cfg.ui.port

    import uvicorn

    uvicorn.run(create_app(), host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()
