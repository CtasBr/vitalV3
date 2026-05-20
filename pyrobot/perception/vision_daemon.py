from __future__ import annotations

import signal
import sys
import time

import structlog
import zmq

from proto.vision import Detection, VisionState
from pyrobot.config.load_config import load_config
from pyrobot.hal.zmq_bus import ZmqPublisher
from pyrobot.perception.frame_store import FrameStore
from pyrobot.perception.metasense import open_metasense
from pyrobot.perception.tof_pipeline import (
    ContourTracker,
    compute_offsets_mm,
    denoise_depth_image,
    sobel_edge_detection,
    temporal_filter,
)
from pyrobot.perception.usb_camera import UsbCameraCapture

log = structlog.get_logger(node="vision_daemon")


def _require_vision_deps() -> tuple[object, object]:
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        log.error("vision_deps_missing", hint="pip install -e '.[vision]'")
        raise SystemExit(1) from exc
    return cv2, np


def _draw_yolo_boxes(
    cv2: object,
    img: object,
    boxes: list[tuple[str, float, tuple[int, int, int, int]]],
) -> None:
    for name, conf, (x1, y1, x2, y2) in boxes:
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{name} {conf:.2f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.65
        thick = 2
        (tw, th), _ = cv2.getTextSize(label, font, scale, thick)
        ty = max(y1, th + 8)
        cv2.rectangle(img, (x1, ty - th - 6), (x1 + tw + 6, ty + 2), (0, 255, 0), -1)
        cv2.putText(img, label, (x1 + 3, ty - 4), font, scale, (0, 0, 0), thick, cv2.LINE_AA)


def _jpeg_encode(cv2: object, img: object, quality: int = 80) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("jpeg encode failed")
    return buf.tobytes()


def main() -> None:
    structlog.configure(
        processors=[
            structlog.processors.add_log_level,
            structlog.processors.TimeStamper(fmt="iso"),
            structlog.dev.ConsoleRenderer(),
        ],
    )
    cv2, np = _require_vision_deps()
    cfg = load_config()
    vis = cfg.vision
    frames = FrameStore(vis.frame_dir)

    running = True

    def _stop(*_a: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    ctx = zmq.Context()
    pub = ZmqPublisher(ctx, cfg.vision_detections_uri())

    tof = None
    if vis.tof_enabled:
        try:
            tof = open_metasense(
                cfg.tof.port,
                cfg.tof.baudrate,
                quantize=cfg.tof.quantize,
                fps=vis.tof_fps,
                disp=vis.tof_disp,
            )
            log.info("tof_ready", port=cfg.tof.port)
        except Exception as exc:
            log.warning("tof_unavailable", error=str(exc))

    cam: UsbCameraCapture | None = None
    if vis.camera_enabled:
        try:
            cam = UsbCameraCapture(
                cfg.camera.index,
                cfg.camera.width,
                cfg.camera.height,
                backend=cfg.camera.backend,
            )
            cam.start()
            log.info(
                "camera_ready",
                index=cfg.camera.index,
                backend=cfg.camera.backend,
                size=f"{cfg.camera.width}x{cfg.camera.height}",
            )
        except Exception as exc:
            log.warning("camera_unavailable", error=str(exc))

    yolo = None
    if vis.yolo_enabled:
        try:
            from ultralytics import YOLO

            yolo = YOLO(vis.yolo_model)
            log.info("yolo_ready", model=vis.yolo_model)
        except Exception as exc:
            log.warning("yolo_unavailable", error=str(exc))

    tracker = ContourTracker()
    prev_depth = None
    frame_idx = 0
    display = tuple(vis.display_size)
    last_yolo_boxes: list[tuple[str, float, tuple[int, int, int, int]]] = []

    log.info("started", frames=str(frames.dir))

    try:
        while running:
            detections: list[Detection] = []
            tof_distance: float | None = None
            offset_mm: list[float] | None = None

            if tof is not None:
                try:
                    frame = tof.tof_data_queue.get(timeout=0.5)
                except Exception:
                    frame = None
                if frame is not None:
                    res = frame["res"]
                    depth = np.array(frame["frameData"], dtype=np.uint8).reshape(res[0], res[1])
                    depth_f = temporal_filter(depth, prev_depth)
                    prev_depth = depth_f.copy()
                    depth_d = denoise_depth_image(depth_f)
                    mask = sobel_edge_detection(depth_d, cfg.tof.quantize, cfg.tof.delta_mm)
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    best_cnt, _score = tracker.find_best(contours, depth_d)
                    result_img, cx, cy, med = tracker.draw_result(
                        depth_d, best_cnt, quantize=cfg.tof.quantize
                    )
                    if cx is not None and cy is not None and med is not None:
                        ox, oy, oz = compute_offsets_mm(cx, cy, med, depth_d.shape)
                        offset_mm = [ox, oy, oz]
                        tof_distance = oz

                    color_depth = cv2.applyColorMap(
                        cv2.resize(depth_d, display, interpolation=cv2.INTER_NEAREST),
                        cv2.COLORMAP_JET,
                    )
                    mask_viz = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    if best_cnt is not None:
                        cv2.drawContours(mask_viz, [best_cnt], -1, (0, 255, 255), 3)
                    mask_viz = cv2.resize(mask_viz, display, interpolation=cv2.INTER_NEAREST)
                    track_viz = cv2.resize(result_img, display, interpolation=cv2.INTER_NEAREST)

                    frames.write_jpeg("depth", _jpeg_encode(cv2, color_depth))
                    frames.write_jpeg("mask", _jpeg_encode(cv2, mask_viz))
                    frames.write_jpeg("track", _jpeg_encode(cv2, track_viz))

            if cam is not None:
                rgb = cam.get_latest_frame()
                if rgb is not None:
                    frame_idx += 1
                    if yolo is not None and frame_idx % vis.yolo_every_n_frames == 0:
                        res = yolo(rgb, verbose=False)[0]
                        fresh: list[tuple[str, float, tuple[int, int, int, int]]] = []
                        for box in res.boxes:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            conf = float(box.conf[0])
                            cls_id = int(box.cls[0])
                            name = str(res.names[cls_id])
                            xi1, yi1, xi2, yi2 = int(x1), int(y1), int(x2), int(y2)
                            fresh.append((name, conf, (xi1, yi1, xi2, yi2)))
                        last_yolo_boxes = fresh

                    for name, conf, bbox in last_yolo_boxes:
                        detections.append(
                            Detection(
                                node="vision_daemon",
                                class_name=name,
                                confidence=conf,
                                bbox_xyxy=[
                                    float(bbox[0]),
                                    float(bbox[1]),
                                    float(bbox[2]),
                                    float(bbox[3]),
                                ],
                                source="yolo",
                            )
                        )

                    annotated = rgb.copy()
                    if last_yolo_boxes:
                        _draw_yolo_boxes(cv2, annotated, last_yolo_boxes)
                    small = cv2.resize(annotated, display, interpolation=cv2.INTER_AREA)
                    frames.write_jpeg("rgb", _jpeg_encode(cv2, small))

            pub.publish(
                VisionState(
                    node="vision_daemon",
                    detections=detections,
                    tof_distance_mm=tof_distance,
                    offset_mm=offset_mm,
                )
            )
            time.sleep(1.0 / max(1, vis.loop_hz))
    finally:
        if tof is not None:
            tof.terminate()
        if cam is not None:
            cam.stop()
        pub.close()
        ctx.term()
        log.info("stopped")


if __name__ == "__main__":
    main()
    sys.exit(0)
