from proto.motion import MotionCommand, MotionState
from proto.vision import Detection, VisionState
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


def test_vision_state_roundtrip() -> None:
    st = VisionState(
        detections=[
            Detection(
                class_name="cup",
                confidence=0.9,
                bbox_xyxy=[1.0, 2.0, 3.0, 4.0],
            )
        ],
        tof_distance_mm=320.0,
        offset_mm=[1.0, 2.0, 320.0],
    )
    back = unpack_model(pack_model(st), VisionState)
    assert back.tof_distance_mm == 320.0
    assert back.detections[0].class_name == "cup"
