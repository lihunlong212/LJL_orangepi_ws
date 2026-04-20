"""
detector_core.py
Detection logic extracted from detect_relaxed_v4.py.
Interactive / camera-open / argparse code is intentionally excluded.
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import numpy as np


@dataclass
class DetectorConfig:
    threshold_method: str = "adaptive"
    min_area: float = 750.0
    approx_eps_ratio: float = 0.02
    aspect_ratio_min: float = 0.65
    aspect_ratio_max: float = 1.50
    angle_tolerance: float = 26.0
    square_ratio_tolerance: float = 0.28
    max_fill_ratio_for_border: float = 0.85
    min_hole_ratio_for_border: float = 0.08
    adaptive_block_size: int = 31
    adaptive_c: int = 1
    morph_kernel_size: int = 3
    black_v_max: int = 100
    black_s_max: int = 110
    black_kernel_size: int = 5
    fused_close_kernel_size: int = 5

    # Circle detection and robustness parameters.
    circle_min_area: float = 140.0
    circle_circularity_min: float = 0.60
    circle_solidity_min: float = 0.70
    circle_aspect_max: float = 1.7
    circle_roi_margin: int = 5
    circle_roi_erode_kernel: int = 5
    circle_center_offset_max_ratio: float = 0.32
    circle_boundary_touch_px: int = 2
    circle_repair_close_kernel: int = 9

    # Relaxed fallback for broken circles caused by reflection.
    circle_relaxed_min_area: float = 70.0
    circle_relaxed_circularity_min: float = 0.34
    circle_relaxed_solidity_min: float = 0.38
    circle_relaxed_aspect_max: float = 2.2
    circle_relaxed_center_offset_max_ratio: float = 0.55
    circle_relaxed_confidence_min: float = 0.14

    # Ratio smoothing / anti-jitter parameters.
    circle_ratio_ema_alpha: float = 0.18
    circle_hold_frames: int = 32
    circle_detect_miss_tolerance: int = 1
    ratio_outlier_percent: float = 0.60
    ratio_window_size: int = 11
    ratio_confidence_min: float = 0.07

    # Rectangle tracking robustness parameters.
    track_pred_decay: float = 0.92
    track_pred_max_step_ratio: float = 0.24
    track_gate_base: float = 0.46
    track_gate_miss_gain: float = 0.07
    track_gate_speed_gain: float = 0.08
    track_gate_speed_norm: float = 60.0
    track_size_ratio_min: float = 0.35
    track_size_ratio_max: float = 2.80
    track_size_ratio_min_miss: float = 0.32
    track_size_ratio_max_miss: float = 2.80
    track_velocity_blend_old: float = 0.66
    track_switch_required_hits: int = 3
    track_pending_iou_min: float = 0.34


@dataclass
class TrackState:
    polygon: Optional[np.ndarray] = None
    bbox: Optional[tuple[int, int, int, int]] = None
    area: float = 0.0
    stable_hits: int = 0
    misses: int = 0
    velocity: tuple[float, float] = (0.0, 0.0)


class StableRectangleTracker:
    def __init__(
        self,
        required_hits: int = 1,
        max_missed: int = 34,
        iou_threshold: float = 0.08,
        smooth_alpha: float = 0.22,
        pred_decay: float = 0.92,
        pred_max_step_ratio: float = 0.24,
        gate_base: float = 0.46,
        gate_miss_gain: float = 0.07,
        gate_speed_gain: float = 0.08,
        gate_speed_norm: float = 60.0,
        size_ratio_min: float = 0.35,
        size_ratio_max: float = 2.80,
        size_ratio_min_miss: float = 0.32,
        size_ratio_max_miss: float = 2.80,
        velocity_blend_old: float = 0.66,
        switch_required_hits: int = 3,
        pending_iou_min: float = 0.34,
    ):
        self.required_hits = required_hits
        self.max_missed = max_missed
        self.iou_threshold = iou_threshold
        self.smooth_alpha = smooth_alpha
        self.pred_decay = pred_decay
        self.pred_max_step_ratio = max(0.05, pred_max_step_ratio)
        self.gate_base = gate_base
        self.gate_miss_gain = gate_miss_gain
        self.gate_speed_gain = gate_speed_gain
        self.gate_speed_norm = max(1.0, gate_speed_norm)
        self.size_ratio_min = size_ratio_min
        self.size_ratio_max = size_ratio_max
        self.size_ratio_min_miss = size_ratio_min_miss
        self.size_ratio_max_miss = size_ratio_max_miss
        self.velocity_blend_old = min(0.95, max(0.05, velocity_blend_old))
        self.switch_required_hits = max(1, switch_required_hits)
        self.pending_iou_min = max(0.05, min(0.9, pending_iou_min))
        self.pending_bbox: Optional[tuple[int, int, int, int]] = None
        self.pending_poly: Optional[np.ndarray] = None
        self.pending_area: float = 0.0
        self.pending_hits: int = 0
        self.state = TrackState()

    @staticmethod
    def _center_of_bbox(box: tuple[int, int, int, int]) -> tuple[float, float]:
        x, y, w, h = box
        return x + w * 0.5, y + h * 0.5

    @staticmethod
    def _translate_polygon(poly: Optional[np.ndarray], dx: float, dy: float) -> Optional[np.ndarray]:
        if poly is None:
            return None
        shift = np.array([dx, dy], dtype=np.float32)
        return poly + shift

    @staticmethod
    def _iou(box_a: tuple[int, int, int, int], box_b: tuple[int, int, int, int]) -> float:
        ax, ay, aw, ah = box_a
        bx, by, bw, bh = box_b
        a_x2, a_y2 = ax + aw, ay + ah
        b_x2, b_y2 = bx + bw, by + bh

        inter_x1 = max(ax, bx)
        inter_y1 = max(ay, by)
        inter_x2 = min(a_x2, b_x2)
        inter_y2 = min(a_y2, b_y2)

        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        if inter_area == 0:
            return 0.0

        area_a = aw * ah
        area_b = bw * bh
        denom = float(area_a + area_b - inter_area)
        return inter_area / denom if denom > 0 else 0.0

    def reset(self) -> None:
        self.pending_bbox = None
        self.pending_poly = None
        self.pending_area = 0.0
        self.pending_hits = 0
        self.state = TrackState()

    def _predict_forward(self) -> None:
        if self.state.bbox is None:
            return
        x, y, w, h = self.state.bbox
        vx, vy = self.state.velocity
        max_step = max(1.0, max(w, h) * self.pred_max_step_ratio)
        speed = float(np.hypot(vx, vy))
        if speed > max_step and speed > 1e-6:
            scale = max_step / speed
            vx *= scale
            vy *= scale

        pred_x = int(round(x + vx))
        pred_y = int(round(y + vy))
        self.state.bbox = (pred_x, pred_y, w, h)
        self.state.polygon = self._translate_polygon(self.state.polygon, vx, vy)
        # Keep short-miss prediction stronger, then decay faster on longer misses.
        decay = 0.97 if self.state.misses <= 2 else self.pred_decay
        self.state.velocity = (vx * decay, vy * decay)

    def update(self, detection: Optional[dict[str, Any]]) -> tuple[TrackState, bool]:
        if detection is None:
            if self.state.bbox is not None:
                self.state.misses += 1
                self._predict_forward()
                if self.state.misses > self.max_missed:
                    self.reset()
            confirmed = self.state.bbox is not None and self.state.stable_hits >= self.required_hits
            return self.state, confirmed

        det_bbox = detection["bbox"]
        det_poly = detection["polygon"].astype(np.float32)
        det_area = float(detection["area"])

        if self.state.bbox is None:
            self.state = TrackState(polygon=det_poly, bbox=det_bbox, area=det_area, stable_hits=1, misses=0)
            self.pending_bbox = None
            self.pending_poly = None
            self.pending_area = 0.0
            self.pending_hits = 0
            return self.state, False

        overlap = self._iou(self.state.bbox, det_bbox)
        old_cx, old_cy = self._center_of_bbox(self.state.bbox)
        det_cx, det_cy = self._center_of_bbox(det_bbox)
        predicted_cx = old_cx + self.state.velocity[0]
        predicted_cy = old_cy + self.state.velocity[1]

        old_w = max(1.0, float(self.state.bbox[2]))
        old_h = max(1.0, float(self.state.bbox[3]))
        speed = float(np.hypot(self.state.velocity[0], self.state.velocity[1]))
        speed_factor = min(1.0, speed / self.gate_speed_norm)
        motion_gate_scale = (
            self.gate_base
            + self.gate_miss_gain * min(6, self.state.misses)
            + self.gate_speed_gain * speed_factor
        )
        motion_gate = max(old_w, old_h) * motion_gate_scale
        dist_to_pred = float(np.hypot(det_cx - predicted_cx, det_cy - predicted_cy))
        det_area_ratio = det_area / max(1.0, self.state.area)
        if self.state.misses > 0:
            size_compatible = self.size_ratio_min_miss <= det_area_ratio <= self.size_ratio_max_miss
        else:
            size_compatible = self.size_ratio_min <= det_area_ratio <= self.size_ratio_max

        if overlap < self.iou_threshold and not (dist_to_pred <= motion_gate and size_compatible):
            # Require candidate consistency before switching to a new target to avoid fly-away boxes.
            if self.pending_bbox is not None and self.pending_poly is not None:
                pending_overlap = self._iou(self.pending_bbox, det_bbox)
                if pending_overlap >= self.pending_iou_min:
                    self.pending_hits += 1
                    blend = 0.35
                    old_pending_bbox = np.array(self.pending_bbox, dtype=np.float32)
                    new_pending_bbox = np.array(det_bbox, dtype=np.float32)
                    merged_pending_bbox = (1.0 - blend) * old_pending_bbox + blend * new_pending_bbox
                    self.pending_bbox = tuple(int(v) for v in merged_pending_bbox)
                    self.pending_poly = (1.0 - blend) * self.pending_poly + blend * det_poly
                    self.pending_area = (1.0 - blend) * self.pending_area + blend * det_area
                else:
                    self.pending_hits = max(1, self.pending_hits - 1)
                    if self.pending_hits == 1:
                        self.pending_bbox = det_bbox
                        self.pending_poly = det_poly
                        self.pending_area = det_area
            else:
                self.pending_bbox = det_bbox
                self.pending_poly = det_poly
                self.pending_area = det_area
                self.pending_hits = 1

            self.state.misses += 1
            self._predict_forward()

            if self.pending_hits >= self.switch_required_hits and self.pending_bbox is not None and self.pending_poly is not None:
                self.state = TrackState(
                    polygon=self.pending_poly,
                    bbox=self.pending_bbox,
                    area=self.pending_area,
                    stable_hits=1,
                    misses=0,
                    velocity=(0.0, 0.0),
                )
                self.pending_bbox = None
                self.pending_poly = None
                self.pending_area = 0.0
                self.pending_hits = 0
                return self.state, False

            if self.state.misses > self.max_missed:
                self.reset()

            confirmed = self.state.bbox is not None and self.state.stable_hits >= self.required_hits
            return self.state, confirmed

        alpha = self.smooth_alpha
        if dist_to_pred > motion_gate * 0.7:
            alpha = min(0.45, self.smooth_alpha + 0.10)
        old_poly = self.state.polygon if self.state.polygon is not None else det_poly
        smoothed_poly = (1.0 - alpha) * old_poly + alpha * det_poly

        old_bbox = np.array(self.state.bbox, dtype=np.float32)
        new_bbox = np.array(det_bbox, dtype=np.float32)
        smoothed_bbox = (1.0 - alpha) * old_bbox + alpha * new_bbox

        smooth_cx, smooth_cy = self._center_of_bbox((int(smoothed_bbox[0]), int(smoothed_bbox[1]), int(smoothed_bbox[2]), int(smoothed_bbox[3])))
        vx = smooth_cx - old_cx
        vy = smooth_cy - old_cy
        blend_old = self.velocity_blend_old
        blend_new = 1.0 - blend_old
        vx = blend_old * self.state.velocity[0] + blend_new * vx
        vy = blend_old * self.state.velocity[1] + blend_new * vy

        self.state.polygon = smoothed_poly
        self.state.bbox = tuple(int(v) for v in smoothed_bbox)
        self.state.area = (1.0 - alpha) * self.state.area + alpha * det_area
        self.state.velocity = (vx, vy)
        self.state.stable_hits += 1
        self.state.misses = 0
        self.pending_bbox = None
        self.pending_poly = None
        self.pending_area = 0.0
        self.pending_hits = 0

        confirmed = self.state.stable_hits >= self.required_hits
        return self.state, confirmed


class RobustRatioFilter:
    def __init__(self, alpha: float, outlier_percent: float, window_size: int, confidence_min: float):
        self.alpha = alpha
        self.outlier_percent = outlier_percent
        self.window = deque(maxlen=max(3, window_size))
        self.confidence_min = confidence_min
        self.value: Optional[float] = None

    def reset(self) -> None:
        self.window.clear()
        self.value = None

    def update(self, raw_ratio: float, confidence: float) -> Optional[float]:
        candidate = float(raw_ratio)

        # Never hard-freeze on low confidence; use smaller update weight instead.
        conf_norm = min(1.0, max(0.0, confidence / max(1e-6, self.confidence_min)))

        if len(self.window) >= 3:
            median_value = float(np.median(np.array(self.window, dtype=np.float32)))
            dynamic_threshold = max(0.01, abs(median_value) * self.outlier_percent)
            deviation = candidate - median_value
            if abs(deviation) > dynamic_threshold:
                # Clip extreme jump instead of rejecting the sample, avoiding low-value lock.
                candidate = median_value + np.sign(deviation) * dynamic_threshold

        self.window.append(candidate)

        effective_alpha = self.alpha * (0.30 + 0.70 * conf_norm)
        if self.value is None:
            self.value = candidate
        else:
            self.value = (1.0 - effective_alpha) * self.value + effective_alpha * candidate

        return self.value


def ensure_odd(value: int, minimum: int = 3) -> int:
    value = max(minimum, int(value))
    return value if value % 2 == 1 else value + 1


def polygon_angles(points: np.ndarray) -> list[float]:
    angles: list[float] = []
    count = len(points)
    for i in range(count):
        prev_pt = points[(i - 1) % count].astype(np.float32)
        curr_pt = points[i].astype(np.float32)
        next_pt = points[(i + 1) % count].astype(np.float32)

        v1 = prev_pt - curr_pt
        v2 = next_pt - curr_pt
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 == 0 or n2 == 0:
            angles.append(0.0)
            continue

        cos_value = float(np.dot(v1, v2) / (n1 * n2))
        cos_value = max(-1.0, min(1.0, cos_value))
        angle = float(np.degrees(np.arccos(cos_value)))
        angles.append(angle)
    return angles


def preprocess(frame: np.ndarray, config: DetectorConfig) -> tuple[np.ndarray, np.ndarray]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    if config.threshold_method == "adaptive":
        block_size = ensure_odd(config.adaptive_block_size)
        binary = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            block_size,
            config.adaptive_c,
        )
    else:
        _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel_size = max(1, int(config.morph_kernel_size))
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

    return gray, binary


def build_black_mask(frame: np.ndarray, config: DetectorConfig) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 0, 0], dtype=np.uint8)
    upper = np.array([180, int(config.black_s_max), int(config.black_v_max)], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    kernel_size = ensure_odd(max(3, config.black_kernel_size))
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return mask


def fuse_binary_masks(primary: np.ndarray, black_mask: np.ndarray, config: DetectorConfig) -> np.ndarray:
    fused = cv2.bitwise_or(primary, black_mask)
    close_size = ensure_odd(max(3, config.fused_close_kernel_size))
    kernel = np.ones((close_size, close_size), dtype=np.uint8)
    fused = cv2.morphologyEx(fused, cv2.MORPH_CLOSE, kernel, iterations=1)
    return fused


def detect_black_rectangle_border(binary: np.ndarray, config: DetectorConfig) -> Optional[dict[str, Any]]:
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return None

    hierarchy = hierarchy[0]
    best: Optional[dict[str, Any]] = None

    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area < config.min_area:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue

        approx = cv2.approxPolyDP(contour, config.approx_eps_ratio * perimeter, True)
        if len(approx) != 4:
            continue
        if not cv2.isContourConvex(approx):
            continue

        points = approx.reshape(-1, 2)
        x, y, w, h = cv2.boundingRect(points)
        if w <= 0 or h <= 0:
            continue

        aspect_ratio = w / float(h)
        if not (config.aspect_ratio_min <= aspect_ratio <= config.aspect_ratio_max):
            continue

        square_bias = max(0.0, 1.0 - abs(aspect_ratio - 1.0) / config.square_ratio_tolerance)
        if square_bias <= 0:
            continue

        right_angles = polygon_angles(points)
        angle_ok = sum(abs(a - 90.0) <= config.angle_tolerance for a in right_angles)
        if angle_ok < 3:
            continue

        fill_ratio = area / float(w * h)
        has_child = hierarchy[i][2] != -1
        hole_ratio = 0.0
        if has_child:
            child_idx = hierarchy[i][2]
            child_area = cv2.contourArea(contours[child_idx])
            if area > 0:
                hole_ratio = max(0.0, min(1.0, child_area / area))

        if not has_child and fill_ratio >= config.max_fill_ratio_for_border:
            continue
        if has_child and hole_ratio < config.min_hole_ratio_for_border:
            continue

        angle_score = max(0.0, 1.0 - (sum(abs(a - 90.0) for a in right_angles) / (4.0 * config.angle_tolerance * 2.0)))
        border_bonus = 1.0 + min(0.4, hole_ratio)
        score = area * (0.55 + 0.45 * square_bias) * (0.7 + 0.3 * angle_score) * border_bonus
        candidate = {
            "contour": contour,
            "polygon": points,
            "bbox": (x, y, w, h),
            "area": area,
            "fill_ratio": fill_ratio,
            "hole_ratio": hole_ratio,
            "has_child": has_child,
            "score": score,
        }

        if best is None or candidate["score"] > best["score"]:
            best = candidate

    return best


def detect_rectangle_with_fallback(binary_primary: np.ndarray, binary_secondary: np.ndarray, config: DetectorConfig) -> Optional[dict[str, Any]]:
    det_primary = detect_black_rectangle_border(binary_primary, config)
    det_secondary = detect_black_rectangle_border(binary_secondary, config)

    if det_primary is None:
        return det_secondary
    if det_secondary is None:
        return det_primary

    return det_primary if det_primary["score"] >= det_secondary["score"] else det_secondary


def build_inner_square_mask(shape: tuple[int, int], polygon: np.ndarray, margin: int, erode_kernel_size: int) -> tuple[np.ndarray, int]:
    mask = np.zeros(shape, dtype=np.uint8)
    poly = np.round(polygon).astype(np.int32).reshape(-1, 1, 2)
    cv2.fillPoly(mask, [poly], 255)

    total_margin = max(0, int(margin))
    if total_margin > 0:
        kernel_size = ensure_odd(max(3, erode_kernel_size))
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        shrink_per_iter = max(1, (kernel_size - 1) // 2)
        iterations = max(1, int(round(total_margin / float(shrink_per_iter))))
        mask = cv2.erode(mask, kernel, iterations=iterations)

    area = int(cv2.countNonZero(mask))
    return mask, area


def contour_touches_boundary(contour: np.ndarray, safe_mask: np.ndarray) -> bool:
    pts = contour.reshape(-1, 2)
    h, w = safe_mask.shape
    xs = np.clip(pts[:, 0], 0, w - 1)
    ys = np.clip(pts[:, 1], 0, h - 1)
    return np.any(safe_mask[ys, xs] == 0)


def detect_circle_in_square(binary: np.ndarray, polygon: np.ndarray, config: DetectorConfig) -> Optional[dict[str, Any]]:
    ratio_square_mask, ratio_square_area = build_inner_square_mask(
        binary.shape,
        polygon,
        margin=0,
        erode_kernel_size=config.circle_roi_erode_kernel,
    )
    square_mask, roi_square_area = build_inner_square_mask(
        binary.shape,
        polygon,
        margin=config.circle_roi_margin,
        erode_kernel_size=config.circle_roi_erode_kernel,
    )
    if ratio_square_area < 1 or roi_square_area < 1:
        return None

    safe_kernel_size = ensure_odd(2 * config.circle_boundary_touch_px + 1)
    safe_kernel = np.ones((safe_kernel_size, safe_kernel_size), dtype=np.uint8)
    safe_mask = cv2.erode(square_mask, safe_kernel, iterations=1)

    inside = cv2.bitwise_and(binary, binary, mask=square_mask)
    repair_kernel_size = ensure_odd(max(3, config.circle_repair_close_kernel))
    repair_kernel = np.ones((repair_kernel_size, repair_kernel_size), dtype=np.uint8)
    inside = cv2.morphologyEx(inside, cv2.MORPH_CLOSE, repair_kernel, iterations=1)
    contours, _ = cv2.findContours(inside, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    square_points = np.round(polygon).astype(np.float32)
    square_center = np.mean(square_points, axis=0)
    sq_x, sq_y, sq_w, sq_h = cv2.boundingRect(square_points.astype(np.int32))
    max_center_offset = max(sq_w, sq_h) * config.circle_center_offset_max_ratio

    strict_best: Optional[dict[str, Any]] = None
    relaxed_best: Optional[dict[str, Any]] = None

    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area > ratio_square_area * 0.82:
            continue

        touches_boundary = contour_touches_boundary(contour, safe_mask)

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            continue

        circularity = float(4.0 * np.pi * area / (perimeter * perimeter))
        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        if hull_area <= 1e-6:
            continue
        solidity = area / hull_area

        rect = cv2.minAreaRect(contour)
        rw, rh = rect[1]
        short_side = max(1e-6, min(rw, rh))
        long_side = max(rw, rh)
        aspect = long_side / short_side
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        if radius < 3:
            continue

        center_offset = float(np.hypot(cx - square_center[0], cy - square_center[1]))

        # Adaptive effective area reduces under-estimation when contour is broken by reflection.
        estimated_circle_area = float(np.pi * radius * radius)
        quality = 0.55 * min(1.0, circularity) + 0.45 * min(1.0, solidity)
        area_coeff = max(0.80, min(0.95, 0.95 - 0.15 * quality))
        effective_area = max(area, area_coeff * estimated_circle_area)
        if effective_area > ratio_square_area * 0.88:
            continue

        ratio = float(effective_area / ratio_square_area)

        # Strict pass: preferred when the contour is complete.
        if (
            effective_area >= config.circle_min_area
            and circularity >= config.circle_circularity_min
            and solidity >= config.circle_solidity_min
            and aspect <= config.circle_aspect_max
            and center_offset <= max_center_offset
            and not touches_boundary
        ):
            aspect_score = max(0.0, 1.0 - abs(aspect - 1.0) / max(0.05, config.circle_aspect_max - 1.0))
            center_score = max(0.0, 1.0 - (center_offset / max(1.0, max_center_offset)))
            confidence = 0.42 * circularity + 0.32 * solidity + 0.16 * aspect_score + 0.10 * center_score
            score = effective_area * confidence
            candidate = {
                "center": (int(cx), int(cy)),
                "radius": float(radius),
                "circle_area": effective_area,
                "square_area": float(ratio_square_area),
                "ratio": ratio,
                "circularity": circularity,
                "solidity": solidity,
                "aspect": aspect,
                "confidence": confidence,
                "score": score,
                "mode": "strict",
            }
            if strict_best is None or score > strict_best["score"]:
                strict_best = candidate
            continue

        # Relaxed fallback: allows broken circles in clean competition scene.
        relaxed_max_center = max(sq_w, sq_h) * config.circle_relaxed_center_offset_max_ratio
        if (
            effective_area >= config.circle_relaxed_min_area
            and circularity >= config.circle_relaxed_circularity_min
            and solidity >= config.circle_relaxed_solidity_min
            and aspect <= config.circle_relaxed_aspect_max
            and center_offset <= relaxed_max_center
            and not touches_boundary
        ):
            aspect_score = max(0.0, 1.0 - abs(aspect - 1.0) / max(0.05, config.circle_relaxed_aspect_max - 1.0))
            center_score = max(0.0, 1.0 - (center_offset / max(1.0, relaxed_max_center)))
            confidence = 0.35 * circularity + 0.30 * solidity + 0.20 * aspect_score + 0.15 * center_score
            if confidence < config.circle_relaxed_confidence_min:
                continue
            score = effective_area * confidence
            candidate = {
                "center": (int(cx), int(cy)),
                "radius": float(radius),
                "circle_area": effective_area,
                "square_area": float(ratio_square_area),
                "ratio": ratio,
                "circularity": circularity,
                "solidity": solidity,
                "aspect": aspect,
                "confidence": confidence,
                "score": score,
                "mode": "relaxed",
            }
            if relaxed_best is None or score > relaxed_best["score"]:
                relaxed_best = candidate

    return strict_best if strict_best is not None else relaxed_best


def draw_overlay(
    frame: np.ndarray,
    state: TrackState,
    confirmed: bool,
    method: str,
    loop_fps: float,
    cam_fps: float,
    circle_info: Optional[dict[str, Any]],
    circle_hold: bool,
) -> None:
    status = "SEARCHING"
    color = (0, 0, 255)

    if state.bbox is not None and state.polygon is not None:
        poly = np.round(state.polygon).astype(np.int32)
        if confirmed and state.misses == 0:
            color = (0, 255, 0)
            status = "BORDER LOCKED"
        elif confirmed and state.misses > 0:
            color = (0, 200, 255)
            status = "BORDER HOLD"
        else:
            color = (0, 220, 255)
            status = "BORDER CANDIDATE"
        cv2.polylines(frame, [poly], True, color, 2, cv2.LINE_AA)

        x, y, w, h = state.bbox
        rect_cx = x + w * 0.5
        rect_cy = y + h * 0.5
        frame_h, frame_w = frame.shape[:2]
        rect_dx = rect_cx - (frame_w * 0.5)
        rect_dy = rect_cy - (frame_h * 0.5)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 1)
        cv2.circle(frame, (int(round(rect_cx)), int(round(rect_cy))), 3, color, -1)
        cv2.putText(
            frame,
            f"hits={state.stable_hits} miss={state.misses}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"rect_dx={rect_dx:+.1f}  rect_dy={rect_dy:+.1f}",
            (16, 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            color,
            2,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            frame,
            "rect_dx=N/A  rect_dy=N/A",
            (16, 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (180, 180, 180),
            2,
            cv2.LINE_AA,
        )

    cv2.putText(frame, status, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
    cv2.putText(frame, f"dx right+, dy down+", (16, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (180, 230, 180), 2, cv2.LINE_AA)

    if circle_info is not None:
        cx, cy = circle_info["center"]
        radius = int(round(circle_info["radius"]))
        circle_color = (255, 220, 0) if circle_hold else (255, 255, 0)
        cv2.circle(frame, (cx, cy), radius, circle_color, 2, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 2, circle_color, -1)

        raw_ratio = float(circle_info.get("ratio", 0.0))
        ratio_smooth = float(circle_info.get("ratio_smooth", raw_ratio))
        shown_ratio = raw_ratio
        confidence = float(circle_info.get("confidence", 0.0))
        mode = str(circle_info.get("mode", "strict"))
        frame_h, frame_w = frame.shape[:2]
        cx_f = float(cx)
        cy_f = float(cy)
        cx_norm = cx_f / max(1.0, float(frame_w))
        cy_norm = cy_f / max(1.0, float(frame_h))
        dx = cx_f - (frame_w / 2.0)
        dy = cy_f - (frame_h / 2.0)

        cv2.putText(frame, f"cir_dx={dx:+.1f}  cir_dy={dy:+.1f}", (16, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.70, circle_color, 2, cv2.LINE_AA)
        cv2.putText(
            frame,
            f"area_ratio={shown_ratio:.4f} ({shown_ratio * 100:.2f}%)",
            (16, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            circle_color,
            2,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            frame,
            "cir_dx=N/A  cir_dy=N/A",
            (16, 112),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (180, 180, 180),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            "area_ratio=N/A",
            (16, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (180, 180, 180),
            2,
            cv2.LINE_AA,
        )
