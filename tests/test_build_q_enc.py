from pyrobot.hal.stm32_motion import Stm32MotionBus


class _FakeEncoderZmq:
    def __init__(self, ab: tuple[float, float]) -> None:
        self._ab = ab

    def poll(self, timeout_ms: int = 0) -> bool:
        del timeout_ms
        return True

    def ab_deg(self) -> tuple[float, float] | None:
        return self._ab


def test_build_q_enc_uses_zmq_ab_without_double_offset() -> None:
    bus = Stm32MotionBus.__new__(Stm32MotionBus)
    bus._enc_offset_deg = [5.0, -5.0, 0.0, 0.0]
    bus._q_enc_raw_deg = [0.0, 0.0, 0.0, 0.0]
    bus._encoder_zmq = _FakeEncoderZmq((90.0, 88.0))  # type: ignore[assignment]
    bus._encoder_bus = None

    q = bus._build_q_enc_deg([10.0, 20.0, 30.0, 40.0])
    assert q[0] == 90.0
    assert q[1] == 88.0
    assert q[2] == 30.0
    assert q[3] == 40.0
