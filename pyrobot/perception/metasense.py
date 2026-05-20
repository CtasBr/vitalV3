"""MetaSense / MaixSense A010 ToF — serial depth frames (legacy protocol)."""

from __future__ import annotations

import queue
import struct
import threading
import time
from collections.abc import Callable

import serial

FRAME_HEAD = b"\x00\xFF"
ALLOWED_TAILS = (0xCC, 0xDD)
ENDIAN = "<"


class MetaSense:
    """Read depth frames into ``tof_data_queue`` (dict with res, frameData, frameID)."""

    def __init__(self, port: str, baudrate: int = 115200, *, timeout: float = 0.05) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.tof_data_queue: queue.Queue[dict[str, object]] = queue.Queue(maxsize=10)
        self._raw_queue: queue.Queue[bytes] = queue.Queue(maxsize=50)
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    def start(self) -> None:
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._reader_loop, daemon=True, name="tof-reader"),
            threading.Thread(target=self._relay_loop, daemon=True, name="tof-relay"),
        ]
        for t in self._threads:
            t.start()

    def terminate(self) -> None:
        self._stop.set()
        if self.ser.is_open:
            self.ser.close()

    def send_cmd(self, cmd: str, *, wait_s: float = 0.15) -> None:
        if not cmd.endswith("\r"):
            cmd += "\r"
        self.ser.write(cmd.encode("ascii"))
        time.sleep(wait_s)

    sendCmd = send_cmd  # legacy alias

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                n = self.ser.in_waiting
            except serial.SerialException:
                break
            try:
                data = self.ser.read(min(4096, n) if n else 256)
            except serial.SerialException:
                break
            if not data:
                time.sleep(0.001)
                continue
            try:
                while self._raw_queue.full():
                    self._raw_queue.get_nowait()
                self._raw_queue.put_nowait(data)
            except queue.Full:
                pass

    def _relay_loop(self) -> None:
        buf = bytearray()
        last_frame_id = -1
        while not self._stop.is_set() or not self._raw_queue.empty():
            try:
                chunk = self._raw_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            buf += chunk
            while True:
                idx = buf.find(FRAME_HEAD)
                if idx < 0:
                    if len(buf) > 8192:
                        buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < 4:
                    break
                data_len = struct.unpack(ENDIAN + "H", buf[2:4])[0]
                frame_len = 2 + 2 + data_len + 2
                if len(buf) < frame_len:
                    break
                frame = bytes(buf[:frame_len])
                del buf[:frame_len]

                tail = frame[-1]
                checksum = frame[-2]
                if tail not in ALLOWED_TAILS or checksum != (sum(frame[:-2]) & 0xFF):
                    continue
                try:
                    res_r = frame[14]
                    res_c = frame[15]
                    frame_id = struct.unpack(ENDIAN + "H", frame[16:18])[0]
                except IndexError:
                    continue
                payload_len = data_len - 16
                data_start = 20
                data_end = data_start + payload_len
                if data_end > len(frame) - 2:
                    continue
                if frame_id == last_frame_id:
                    continue
                last_frame_id = frame_id
                payload = frame[data_start:data_end]
                item = {
                    "frameID": frame_id,
                    "res": [res_r, res_c],
                    "frameData": list(payload),
                }
                try:
                    while self.tof_data_queue.full():
                        self.tof_data_queue.get_nowait()
                    self.tof_data_queue.put_nowait(item)
                except queue.Full:
                    pass


def open_metasense(
    port: str,
    baudrate: int,
    *,
    quantize: int = 2,
    fps: int = 10,
    disp: int = 3,
    on_ready: Callable[[MetaSense], None] | None = None,
) -> MetaSense:
    """Connect and send standard AT init (retry until port available)."""
    last_err: Exception | None = None
    for _ in range(30):
        try:
            dev = MetaSense(port, baudrate)
            if not dev.ser.is_open:
                raise serial.SerialException("port not open")
            dev.start()
            dev.send_cmd(f"AT+DISP={disp}")
            dev.send_cmd(f"AT+UNIT={quantize}")
            dev.send_cmd(f"AT+FPS={fps}")
            if on_ready:
                on_ready(dev)
            return dev
        except (serial.SerialException, OSError) as exc:
            last_err = exc
            time.sleep(0.2)
    raise RuntimeError(f"MetaSense open failed on {port}: {last_err}")
