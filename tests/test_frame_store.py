from pyrobot.perception.frame_store import FrameStore


def test_frame_store_atomic_write(tmp_path) -> None:
    store = FrameStore(tmp_path)
    store.write_jpeg("rgb", b"\xff\xd8\xff")
    assert store.read_jpeg("rgb") == b"\xff\xd8\xff"
    assert store.read_jpeg("depth") is None
