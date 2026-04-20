from __future__ import annotations

import math
from typing import Optional

import cv2
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Int32

from circle_detector_pkg.detector_core import (
    DetectorConfig,
    RobustRatioFilter,
    StableRectangleTracker,
    preprocess,
    build_black_mask,
    fuse_binary_masks,
    detect_rectangle_with_fallback,
    detect_circle_in_square,
    draw_overlay,
)


class CircleDetectorNode(Node):
    def __init__(self):
        super().__init__('circle_detector')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('show_display', False)

        cam_idx = self.get_parameter('camera_index').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        cam_fps = self.get_parameter('camera_fps').value
        self._show = self.get_parameter('show_display').value

        self._config = DetectorConfig()
        self._tracker = StableRectangleTracker(
            pred_decay=self._config.track_pred_decay,
            pred_max_step_ratio=self._config.track_pred_max_step_ratio,
            gate_base=self._config.track_gate_base,
            gate_miss_gain=self._config.track_gate_miss_gain,
            gate_speed_gain=self._config.track_gate_speed_gain,
            gate_speed_norm=self._config.track_gate_speed_norm,
            size_ratio_min=self._config.track_size_ratio_min,
            size_ratio_max=self._config.track_size_ratio_max,
            size_ratio_min_miss=self._config.track_size_ratio_min_miss,
            size_ratio_max_miss=self._config.track_size_ratio_max_miss,
            velocity_blend_old=self._config.track_velocity_blend_old,
            switch_required_hits=self._config.track_switch_required_hits,
            pending_iou_min=self._config.track_pending_iou_min,
        )
        self._ratio_filter = RobustRatioFilter(
            alpha=self._config.circle_ratio_ema_alpha,
            outlier_percent=self._config.ratio_outlier_percent,
            window_size=self._config.ratio_window_size,
            confidence_min=self._config.ratio_confidence_min,
        )
        self._circle_last: Optional[dict] = None
        self._circle_misses: int = 0

        self._cap: Optional[cv2.VideoCapture] = None
        self._camera_ok: bool = self._open_camera(cam_idx, width, height, cam_fps)
        if not self._camera_ok:
            self.get_logger().error(
                f'Failed to open camera index {cam_idx} (/dev/video{cam_idx}). '
                'Publishing NaN until camera becomes available.'
            )

        self._pub_center = self.create_publisher(PointStamped, '/circle_center', 10)
        self._pub_rank = self.create_publisher(Int32, '/circle_rank', 10)

        self._timer = self.create_timer(1.0 / 15.0, self._timer_callback)

        self.get_logger().info(
            f'CircleDetectorNode started - camera={cam_idx} ({width}x{height}) '
            f'show_display={self._show}'
        )

    def _open_camera(self, index: int, width: int, height: int, fps: int) -> bool:
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        self._cap = cap
        return True

    def _publish_nan(self) -> None:
        now = self.get_clock().now().to_msg()

        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = 'camera'
        pt.point.x = math.nan
        pt.point.y = math.nan
        pt.point.z = 0.0

        rank_msg = Int32()
        rank_msg.data = 0

        self._pub_center.publish(pt)
        self._pub_rank.publish(rank_msg)

    def _timer_callback(self) -> None:
        if not self._camera_ok or self._cap is None:
            cam_idx = self.get_parameter('camera_index').value
            width = self.get_parameter('width').value
            height = self.get_parameter('height').value
            camera_fps = self.get_parameter('camera_fps').value
            self._camera_ok = self._open_camera(cam_idx, width, height, camera_fps)
            if self._camera_ok:
                self.get_logger().info('Camera re-opened successfully.')
            self._publish_nan()
            return

        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warn('cap.read() failed - will attempt re-open next tick.')
            self._camera_ok = False
            self._publish_nan()
            return

        _, binary_primary = preprocess(frame, self._config)
        binary_black = build_black_mask(frame, self._config)
        binary_fused = fuse_binary_masks(binary_primary, binary_black, self._config)

        detection = detect_rectangle_with_fallback(binary_fused, binary_primary, self._config)
        state, confirmed = self._tracker.update(detection)

        circle_current: Optional[dict] = None
        allow_hold = (
            state.misses <= self._config.circle_detect_miss_tolerance
            and state.stable_hits >= max(2, self._tracker.required_hits)
        )
        if state.polygon is not None and (confirmed or allow_hold):
            circle_current = detect_circle_in_square(binary_fused, state.polygon, self._config)

        if circle_current is not None:
            smooth_ratio = self._ratio_filter.update(
                circle_current['ratio'], circle_current['confidence']
            )
            if smooth_ratio is not None:
                circle_current['ratio_smooth'] = smooth_ratio
                self._circle_last = circle_current
                self._circle_misses = 0
        else:
            self._circle_misses += 1
            if self._circle_misses > self._config.circle_hold_frames:
                self._circle_last = None
                self._ratio_filter.reset()

        circle_for_pub = circle_current if circle_current is not None else self._circle_last
        circle_hold = circle_current is None and self._circle_last is not None

        now = self.get_clock().now().to_msg()
        frame_h, frame_w = frame.shape[:2]

        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = 'camera'

        rank_msg = Int32()

        if circle_for_pub is not None:
            cx, cy = circle_for_pub['center']
            pt.point.x = float(cx) - frame_w / 2.0
            pt.point.y = frame_h / 2.0 - float(cy)
            pt.point.z = 0.0
            ratio = circle_for_pub.get('ratio_smooth', circle_for_pub.get('ratio', 0.0))
            if 0.35 <= ratio <= 0.50:
                rank_msg.data = 1
            elif 0.15 <= ratio <= 0.30:
                rank_msg.data = 2
            elif 0.02 <= ratio <= 0.15:
                rank_msg.data = 3
            else:
                rank_msg.data = 0
        else:
            return

        self._pub_center.publish(pt)
        self._pub_rank.publish(rank_msg)

        if self._show:
            vis = frame.copy()
            draw_overlay(
                vis,
                state,
                confirmed,
                self._config.threshold_method,
                0.0,
                0.0,
                circle_for_pub,
                circle_hold,
            )
            cv2.imshow('circle_detector_pkg', vis)
            cv2.waitKey(1)

    def destroy_node(self) -> None:
        if self._cap is not None:
            self._cap.release()
        if self._show:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
