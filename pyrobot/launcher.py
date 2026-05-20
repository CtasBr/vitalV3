from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from pyrobot.config.load_config import load_config

_PID_FILE = Path("/tmp/robot/launcher.pids")


def _python() -> str:
    return sys.executable


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _child_specs(
    with_web: bool,
    with_vision: bool,
    web_host: str | None,
    web_port: int | None,
) -> list[tuple[str, list[str]]]:
    base = [_python(), "-m"]
    specs: list[tuple[str, list[str]]] = [
        ("encoder", base + ["pyrobot.hal.encoder_daemon"]),
        ("motion", base + ["pyrobot.hal.motion_daemon"]),
    ]
    if with_vision:
        specs.append(("vision", base + ["pyrobot.perception.vision_daemon"]))
    if with_web:
        web_cmd = base + ["pyrobot.ui.web_server"]
        if web_host:
            web_cmd += ["--host", web_host]
        if web_port:
            web_cmd += ["--port", str(web_port)]
        specs.append(("web", web_cmd))
    return specs


def _write_pids(procs: list[tuple[str, subprocess.Popen[bytes]]]) -> None:
    _PID_FILE.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"{name} {p.pid}\n" for name, p in procs if p.poll() is None]
    _PID_FILE.write_text("".join(lines), encoding="utf-8")


def _read_pids() -> list[tuple[str, int]]:
    if not _PID_FILE.is_file():
        return []
    out: list[tuple[str, int]] = []
    for line in _PID_FILE.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) == 2:
            out.append((parts[0], int(parts[1])))
    return out


def cmd_start(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    existing = _read_pids()
    if existing and not args.force:
        print("Already running (use --force or: robot stop):", existing)
        return 1

    if args.force:
        cmd_stop(argparse.Namespace(config=args.config))

    specs = _child_specs(
        with_web=not args.no_web,
        with_vision=not args.no_vision,
        web_host=cfg.ui.host,
        web_port=cfg.ui.port,
    )
    procs: list[tuple[str, subprocess.Popen[bytes]]] = []
    print("Starting vitalV3 stack…")
    for name, cmd in specs:
        print(f"  [{name}]", " ".join(cmd))
        proc = subprocess.Popen(
            cmd,
            cwd=_repo_root(),
            env={**os.environ, "PYTHONPATH": str(_repo_root())},
        )
        procs.append((name, proc))
        if name == "encoder":
            time.sleep(1.0)
        elif name == "motion":
            time.sleep(1.5)
        elif name == "vision":
            time.sleep(0.5)

    _write_pids(procs)
    for name, proc in procs:
        if proc.poll() is not None:
            print(f"ERROR: {name} exited with code {proc.returncode}")
            return 1

    if not args.no_web:
        print(f"\nWeb UI: http://{cfg.ui.host}:{cfg.ui.port}")
        if cfg.ui.host == "0.0.0.0":
            print("  (LAN: http://<this-machine-ip>:8080)")
    print("Stop: Ctrl+C here, or: python -m pyrobot.launcher stop")
    try:
        while True:
            time.sleep(1.0)
            for name, proc in procs:
                if proc.poll() is not None:
                    print(f"{name} exited ({proc.returncode}), stopping…")
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        print("\nShutting down…")
        for _, proc in procs:
            proc.send_signal(signal.SIGTERM)
        time.sleep(0.5)
        for _, proc in procs:
            if proc.poll() is None:
                proc.kill()
        _PID_FILE.unlink(missing_ok=True)
    return 0


def cmd_stop(_args: argparse.Namespace) -> int:
    pids = _read_pids()
    if not pids:
        print("No pid file (not running?)")
        return 0
    for name, pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
            print(f"stopped {name} (pid {pid})")
        except ProcessLookupError:
            print(f"{name} (pid {pid}) already gone")
    _PID_FILE.unlink(missing_ok=True)
    return 0


def cmd_status(_args: argparse.Namespace) -> int:
    pids = _read_pids()
    if not pids:
        print("not running")
        return 1
    ok = True
    for name, pid in pids:
        try:
            os.kill(pid, 0)
            print(f"{name}: running (pid {pid})")
        except ProcessLookupError:
            print(f"{name}: dead (pid {pid})")
            ok = False
    return 0 if ok else 1


def main() -> None:
    parser = argparse.ArgumentParser(
        description="vitalV3 — start encoder + motion + vision + web UI",
        prog="robot",
    )
    parser.add_argument("--config", default=None, help="Path to robot.yaml")
    sub = parser.add_subparsers(dest="cmd")

    p_start = sub.add_parser("start", help="Start all daemons (default)")
    p_start.add_argument("--no-web", action="store_true", help="Skip web UI")
    p_start.add_argument("--no-vision", action="store_true", help="Skip vision daemon (no cameras)")
    p_start.add_argument("--force", action="store_true", help="Restart if already running")
    sub.add_parser("stop", help="Stop daemons from pid file")
    sub.add_parser("status", help="Check pid file")

    args = parser.parse_args()
    if args.cmd is None or args.cmd == "start":
        start_ns = (
            args
            if args.cmd == "start"
            else argparse.Namespace(config=args.config, no_web=False, no_vision=False, force=False)
        )
        raise SystemExit(cmd_start(start_ns))
    if args.cmd == "stop":
        raise SystemExit(cmd_stop(args))
    if args.cmd == "status":
        raise SystemExit(cmd_status(args))


if __name__ == "__main__":
    main()
