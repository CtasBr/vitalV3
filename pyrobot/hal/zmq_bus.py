from __future__ import annotations

from typing import TypeVar

import msgpack
import zmq
from pydantic import BaseModel

T = TypeVar("T", bound=BaseModel)


def pack_model(model: BaseModel) -> bytes:
    return msgpack.packb(model.model_dump(mode="json"), use_bin_type=True)


def unpack_model(data: bytes, model_type: type[T]) -> T:
    raw = msgpack.unpackb(data, raw=False)
    return model_type.model_validate(raw)


class ZmqPublisher:
    def __init__(self, ctx: zmq.Context, uri: str, topic: str = "") -> None:
        self._socket = ctx.socket(zmq.PUB)
        self._socket.bind(uri)
        self._topic = topic.encode() if topic else b""

    def publish(self, model: BaseModel) -> None:
        payload = pack_model(model)
        if self._topic:
            self._socket.send_multipart([self._topic, payload])
        else:
            self._socket.send(payload)

    def close(self) -> None:
        self._socket.close()


class ZmqSubscriber:
    def __init__(self, ctx: zmq.Context, uri: str, topic: str = "", *, conflate: bool = False) -> None:
        self._socket = ctx.socket(zmq.SUB)
        self._socket.connect(uri)
        if conflate:
            self._socket.setsockopt(zmq.CONFLATE, 1)
        if topic:
            self._socket.setsockopt(zmq.SUBSCRIBE, topic.encode())
        else:
            self._socket.setsockopt(zmq.SUBSCRIBE, b"")

    def recv_model(self, model_type: type[T], timeout_ms: int | None = None) -> T | None:
        if timeout_ms is not None:
            if not self._socket.poll(timeout_ms):
                return None
        parts = self._socket.recv_multipart()
        payload = parts[-1]
        return unpack_model(payload, model_type)

    def close(self) -> None:
        self._socket.close()


class ZmqReplyServer:
    def __init__(self, ctx: zmq.Context, uri: str) -> None:
        self._socket = ctx.socket(zmq.REP)
        self._socket.bind(uri)

    def poll(self, timeout_ms: int = 0) -> bool:
        return bool(self._socket.poll(timeout_ms))

    def recv_command(self, model_type: type[T]) -> T:
        data = self._socket.recv()
        return unpack_model(data, model_type)

    def send_reply(self, model: BaseModel) -> None:
        self._socket.send(pack_model(model))

    def close(self) -> None:
        self._socket.close()
