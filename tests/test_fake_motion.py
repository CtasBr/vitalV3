import subprocess
import sys
import time

from pyrobot.config.load_config import load_config
from pyrobot.hal.fake_motion import FakeMotionBus
from pyrobot.hal.motion_client import ZmqMotionClient


def test_fake_motion_move_and_wait() -> None:
    with FakeMotionBus() as bus:
        seg = bus.move_joints([95.0, 85.0, 5.0, 0.0])
        final = bus.wait_done(seg, timeout_s=5.0)
        assert final.fault_code == 0
        assert not final.in_motion
        assert final.segment_id_done == seg
        for i in range(3):
            assert abs(final.q_enc_deg[i] - final.q_cmd_deg[i]) < 0.5


def test_fake_motion_estop() -> None:
    with FakeMotionBus() as bus:
        bus.move_joints([120.0, 120.0, 0.0, 0.0])
        bus.estop()
        st = bus.state
        assert st.fault_code == 1
        bus.reset_fault()
        assert bus.state.fault_code == 0


def test_zmq_motion_client() -> None:
    proc = subprocess.Popen(
        [sys.executable, "-m", "pyrobot.hal.fake_motion_daemon"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        time.sleep(0.5)
        client = ZmqMotionClient(load_config())
        seg = client.move_joints([100.0, 90.0, 10.0, 0.0])
        st = client.wait_done(seg, timeout_s=8.0)
        assert st.fault_code == 0
        client.close()
    finally:
        proc.terminate()
        proc.wait(timeout=5)
