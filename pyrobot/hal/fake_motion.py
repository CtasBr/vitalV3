from __future__ import annotations

import math
import threading
import time
from collections.abc import Iterable

from proto.motion import MotionState, MoveSegment, SegmentId
from pyrobot.config.load_config import RobotConfig, load_config
from pyrobot.hal.motion_bus import MotionBus


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class FakeMotionBus(MotionBus):
    """
    Simulated 4-DOF joint space with trapezoidal interpolation.
    Replaces STM32 until firmware + motion_bridge are ready.
    """

    def __init__(self, config: RobotConfig | None = None) -> None:
        self._cfg = config or load_config()
        self._q = [90.0, 90.0, 0.0, 0.0]
        self._q_target = list(self._q)
        self._q_vel = [0.0, 0.0, 0.0, 0.0]
        self._in_motion = False
        self._fault_code = 0
        self._segment_id = 0
        self._segment_id_done: SegmentId | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True, name="fake-motion")
        self._sim_thread.start()

    def _sim_loop(self) -> None:
        hz = self._cfg.simulation.fake_motion_hz
        dt = 1.0 / hz
        while not self._stop.is_set():
            with self._lock:
                if self._fault_code != 0:
                    self._in_motion = False
                else:
                    self._step(dt)
            time.sleep(dt)

    def _step(self, dt: float) -> None:
        max_step = 120.0 * dt  # deg/s cap for simulation
        moving = False
        for i in range(4):
            err = self._q_target[i] - self._q[i]
            if abs(err) < 1e-4:
                self._q_vel[i] = 0.0
                continue
            moving = True
            step = _clamp(err, -max_step, max_step)
            self._q[i] += step
            self._q_vel[i] = step / dt

        self._in_motion = moving
        if not moving and self._segment_id_done is None and self._segment_id > 0:
            self._segment_id_done = self._segment_id

    def move_joints(
        self,
        q_deg: list[float],
        vmax_deg_s: float = 30.0,
        amax_deg_s2: float = 90.0,
    ) -> SegmentId:
        del vmax_deg_s, amax_deg_s2  # used by real planner later
        limits = self._cfg.simulation.joint_limits_deg
        keys = ["a", "b", "c", "d"]
        clamped: list[float] = []
        for i, key in enumerate(keys):
            lo, hi = limits[key]
            clamped.append(_clamp(q_deg[i], lo, hi))

        with self._lock:
            self._segment_id += 1
            self._segment_id_done = None
            self._q_target = clamped
            self._in_motion = True
            return self._segment_id

    def stream_segments(self, segments: Iterable[MoveSegment]) -> None:
        for seg in segments:
            # Fake bus: interpret axis_steps as tiny joint deltas (demo only)
            q = list(self._q_target)
            scale = self._cfg.kinematics.deg_per_step
            for i in range(4):
                q[i] += seg.axis_steps[i] * scale
            self.move_joints(q)

    def wait_done(self, segment_id: SegmentId, timeout_s: float = 30.0) -> MotionState:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            st = self.state
            if not st.in_motion and st.segment_id_done == segment_id:
                return st
            if st.fault_code != 0:
                return st
            time.sleep(0.01)
        return self.state

    def estop(self) -> None:
        with self._lock:
            self._fault_code = 1
            self._in_motion = False
            self._q_target = list(self._q)
            self._q_vel = [0.0, 0.0, 0.0, 0.0]

    def reset_fault(self) -> None:
        with self._lock:
            self._fault_code = 0

    @property
    def state(self) -> MotionState:
        with self._lock:
            return MotionState(
                node="fake_motion",
                q_cmd_deg=list(self._q_target),
                q_enc_deg=list(self._q),
                q_vel_deg_s=list(self._q_vel),
                in_motion=self._in_motion,
                segment_id_active=self._segment_id if self._in_motion else None,
                segment_id_done=self._segment_id_done,
                fault_code=self._fault_code,
                fault_message="estop" if self._fault_code else "",
            )

    def home(self) -> SegmentId:
        return self.move_joints([90.0, 90.0, 0.0, 0.0])

    def shutdown(self) -> None:
        self._stop.set()
        self._sim_thread.join(timeout=2.0)

    def __enter__(self) -> FakeMotionBus:
        return self

    def __exit__(self, *args: object) -> None:
        self.shutdown()


def demo_move() -> None:
    """Quick CLI sanity check."""
    with FakeMotionBus() as bus:
        print("Initial:", bus.state.model_dump_json(indent=2))
        seg = bus.move_joints([100.0, 80.0, 15.0, 0.0])
        final = bus.wait_done(seg, timeout_s=10.0)
        print("Done:", final.model_dump_json(indent=2))


if __name__ == "__main__":
    demo_move()
