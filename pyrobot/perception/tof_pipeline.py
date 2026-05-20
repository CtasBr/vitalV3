"""ToF depth processing (ported from legacy vitalSoft vision)."""

from __future__ import annotations

from collections import deque

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]

MIN_CONTOUR_AREA = 60
MAX_CONTOUR_AREA = 8000
H_FOV_DEG = 60.0
V_FOV_DEG = 49.5
MAX_LOST_FRAMES = 20


def denoise_depth_image(depth_img: np.ndarray) -> np.ndarray:
    assert cv2 is not None
    return cv2.bilateralFilter(depth_img, 7, 50, 50)


def temporal_filter(current: np.ndarray, prev: np.ndarray | None, alpha: float = 0.6) -> np.ndarray:
    assert cv2 is not None
    if prev is None:
        return current
    return cv2.addWeighted(current, alpha, prev, 1.0 - alpha, 0)


def sobel_edge_detection(depth_img: np.ndarray, quantize: int, threshold: int) -> np.ndarray:
    assert cv2 is not None
    depth_mm = depth_img.astype(np.float32) * quantize
    sobelx = cv2.Sobel(depth_mm, cv2.CV_32F, 1, 0, ksize=3)
    sobely = cv2.Sobel(depth_mm, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = cv2.magnitude(sobelx, sobely)
    mask = (magnitude > threshold).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    border = 14
    mask[:border, :] = 0
    mask[-border:, :] = 0
    mask[:, :border] = 0
    mask[:, -border:] = 0
    return mask


def contour_similarity(c1: np.ndarray, c2: np.ndarray) -> float:
    assert cv2 is not None
    if c1 is None or c2 is None:
        return 0.0
    hausdorff = cv2.matchShapes(c1, c2, cv2.CONTOURS_MATCH_I1, 0.0)
    r1 = cv2.boundingRect(c1)
    r2 = cv2.boundingRect(c2)
    cx1 = r1[0] + r1[2] // 2
    cy1 = r1[1] + r1[3] // 2
    cx2 = r2[0] + r2[2] // 2
    cy2 = r2[1] + r2[3] // 2
    center_dist = np.hypot(cx1 - cx2, cy1 - cy2)
    return 1.0 / (1.0 + hausdorff + center_dist / 80.0)


class ContourTracker:
    def __init__(self, history_len: int = 20) -> None:
        self._history: deque[np.ndarray] = deque(maxlen=history_len)
        self._best: np.ndarray | None = None
        self._lost = 0

    def find_best(self, contours: list[np.ndarray], depth_img: np.ndarray) -> tuple[np.ndarray | None, float]:
        assert cv2 is not None
        valid = [c for c in contours if MIN_CONTOUR_AREA <= cv2.contourArea(c) <= MAX_CONTOUR_AREA]
        if not valid:
            if self._best is not None and self._lost < MAX_LOST_FRAMES:
                self._lost += 1
                return self._best, 999.0
            return None, 0.0

        self._lost = 0
        best_score = -1.0
        best_cnt: np.ndarray | None = None
        for cnt in valid:
            score = cv2.contourArea(cnt) / 1000.0
            if self._history:
                sim_sum = sum(contour_similarity(cnt, h) for h in self._history)
                score += sim_sum / len(self._history) * 1.2
            peri = cv2.arcLength(cnt, True)
            circ = 4 * np.pi * cv2.contourArea(cnt) / (peri**2 + 1e-6)
            score += circ * 0.8
            if self._best is not None:
                score += contour_similarity(cnt, self._best) * 3.0
            if score > best_score:
                best_score = score
                best_cnt = cnt

        if best_cnt is not None:
            self._history.append(best_cnt)
            self._best = best_cnt
        return best_cnt, best_score

    def draw_result(
        self,
        depth_img: np.ndarray,
        best_cnt: np.ndarray | None,
        *,
        quantize: int = 2,
    ) -> tuple[np.ndarray, int | None, int | None, int | None]:
        assert cv2 is not None
        img = cv2.cvtColor(depth_img, cv2.COLOR_GRAY2BGR)
        if best_cnt is None:
            cv2.putText(img, "NO STABLE OBJECT", (10, 50), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 0, 255), 2)
            return img, None, None, None

        cv2.drawContours(img, [best_cnt], -1, (0, 0, 255), 3)
        x, y, w, h = cv2.boundingRect(best_cnt)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        m = cv2.moments(best_cnt)
        if m["m00"] != 0:
            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])
        else:
            cx, cy = x + w // 2, y + h // 2
        roi = depth_img[y : y + h, x : x + w]
        mask = np.zeros_like(roi, dtype=np.uint8)
        cv2.drawContours(mask, [best_cnt - [x, y]], -1, 255, -1)
        distances = roi[mask == 255]
        median_dist = int(np.mean(distances) * quantize) if distances.size else 0
        cv2.circle(img, (cx, cy), 6, (255, 255, 0), -1)
        cv2.putText(img, f"{median_dist}mm", (x, y - 10), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 2)
        return img, cx, cy, median_dist


def compute_offsets_mm(
    cx: int,
    cy: int,
    depth_mm: int,
    img_shape: tuple[int, int],
) -> tuple[float, float, float]:
    h, w = img_shape
    cx_img = w / 2.0
    cy_img = h / 2.0
    dx_pix = cx - cx_img
    dy_pix = cy - cy_img
    h_fov = np.deg2rad(H_FOV_DEG)
    v_fov = np.deg2rad(V_FOV_DEG)
    fx = (w / 2.0) / np.tan(h_fov / 2.0)
    fy = (h / 2.0) / np.tan(v_fov / 2.0)
    z_mm = float(depth_mm)
    x_mm = dx_pix * z_mm / fx
    y_mm = dy_pix * z_mm / fy
    return x_mm * 2 - 15, y_mm * -2, z_mm
