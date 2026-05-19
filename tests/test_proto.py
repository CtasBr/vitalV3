from proto.motion import MotionCommand, MotionState
from pyrobot.hal.zmq_bus import pack_model, unpack_model


def test_motion_roundtrip() -> None:
    cmd = MotionCommand(
        kind="move_joints",
        target_q_deg=[90.0, 90.0, 0.0, 0.0],
        node="test",
    )
    raw = pack_model(cmd)
    back = unpack_model(raw, MotionCommand)
    assert back.target_q_deg == [90.0, 90.0, 0.0, 0.0]
    assert back.schema_version == 1


def test_motion_state_defaults() -> None:
    st = MotionState()
    assert len(st.q_cmd_deg) == 4
    assert st.fault_code == 0
