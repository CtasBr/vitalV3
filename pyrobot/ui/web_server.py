from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from proto.motion import MotionCommand
from pyrobot.behaviour.gcode import gcode_to_motion_command
from pyrobot.config.load_config import load_config
from pyrobot.hal.encoder_zmq_client import EncoderZmqClient
from pyrobot.hal.motion_zmq_client import MotionZmqClient
from pyrobot.perception.frame_store import FrameStore
from pyrobot.ui.state_cache import RobotStateCache

_STATIC = Path(__file__).resolve().parent / "static"
_FRAME_NAMES = frozenset({"rgb", "depth", "mask", "track"})


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
        snap = state_cache.snapshot()
        snap["ui"] = {
            "jog_step_mm": cfg.ui.jog_step_mm,
            "voice_feed_mm_min": cfg.ui.voice_feed_mm_min,
        }
        return snap

    frames = FrameStore(cfg.vision.frame_dir)

    @app.get("/api/frame/{name}")
    def api_frame(name: str) -> Response:
        if name not in _FRAME_NAMES:
            raise HTTPException(404, f"unknown frame: {name}")
        data = frames.read_jpeg(name)
        if data is None:
            raise HTTPException(404, "no frame yet")
        return Response(content=data, media_type="image/jpeg")

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
                if reply.cmd_rejected:
                    raise HTTPException(
                        409,
                        reply.fault_message or "robot busy (previous move still running)",
                    )
                if cmd.kind in ("home", "g28", "linear_move", "gcode", "move_joints"):
                    final = client.wait_move_busy(timeout_s=120.0)
                    if final.fault_code != 0:
                        return {
                            "ok": False,
                            "fault_code": final.fault_code,
                            "fault_message": final.fault_message,
                            "motion": final.model_dump(mode="json"),
                        }
                    return {"ok": True, "motion": final.model_dump(mode="json")}
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

    @app.post("/api/zero-encoders")
    def api_zero_encoders() -> dict[str, Any]:
        snap = state_cache.snapshot()
        motion = snap.get("motion") or {}
        if motion.get("in_motion"):
            raise HTTPException(409, "robot is moving; wait until idle")
        try:
            with EncoderZmqClient(cfg) as enc_client:
                zero = enc_client.zero_encoders(hardware_zero=True)
            if not zero.ok:
                raise HTTPException(500, "encoder zero failed")
            with MotionZmqClient(cfg) as motion_client:
                st = motion_client.send_command(
                    MotionCommand(kind="reload_encoders", node="web")
                )
            return {
                "ok": True,
                "encoders": zero.model_dump(mode="json"),
                "motion": st.model_dump(mode="json"),
            }
        except HTTPException:
            raise
        except Exception as exc:
            raise HTTPException(
                503,
                f"zero encoders failed (is encoder_daemon running?): {exc}",
            ) from exc

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
