"""ROS 2 node wrapper for AnyGrasp grasp tracking.

This node caches synchronized RGB + depth frames and runs the AnyGrasp tracking
update loop on demand via a Trigger service.
"""

from __future__ import annotations

import os
import sys
import threading
from types import SimpleNamespace
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from tracker import AnyGraspTracker  # type: ignore

def _rotation_matrix_to_quaternion(matrix: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(np.trace(matrix))

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (matrix[2, 1] - matrix[1, 2]) * s
        y = (matrix[0, 2] - matrix[2, 0]) * s
        z = (matrix[1, 0] - matrix[0, 1]) * s
        return float(x), float(y), float(z), float(w)

    if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
        return float(x), float(y), float(z), float(w)

    if matrix[1, 1] > matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
        return float(x), float(y), float(z), float(w)

    s = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
    w = (matrix[1, 0] - matrix[0, 1]) / s
    x = (matrix[0, 2] + matrix[2, 0]) / s
    y = (matrix[1, 2] + matrix[2, 1]) / s
    z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class AnyGraspTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('anygrasp_tracking_node')

        self.declare_parameter('anygrasp_sdk_root', '/dependencies/anygrasp_sdk')
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('filter', 'oneeuro')

        # Camera intrinsics (defaults match SDK demo.py)
        self.declare_parameter('fx', 927.17)
        self.declare_parameter('fy', 927.37)
        self.declare_parameter('cx', 651.32)
        self.declare_parameter('cy', 349.62)
        self.declare_parameter('depth_scale', 1000.0)
        self.declare_parameter('depth_max', 1.5)

        # Initial selection workspace (matches tracking demo selection ranges)
        self.declare_parameter('select_x', [-0.18, 0.18])
        self.declare_parameter('select_y', [-0.12, 0.12])
        self.declare_parameter('select_z', [0.35, 0.55])
        self.declare_parameter('select_count', 5)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None

        self._tracker = self._init_tracker()
        self._grasp_ids: List[int] = []

        self._rgb_sub = Subscriber(self, Image, 'rgb_image')
        self._depth_sub = Subscriber(self, Image, 'depth_image')
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._sync_cb)

        self._srv = self.create_service(Trigger, 'tracking', self._on_tracking)

        self.get_logger().info('AnyGrasp tracking node ready.')

    def _init_tracker(self):
        checkpoint_path = str(self.get_parameter('checkpoint_path').value)
        if not checkpoint_path:
            self.get_logger().warn('Parameter `checkpoint_path` is empty; tracking will fail until set.')

        cfg = SimpleNamespace(
            checkpoint_path=checkpoint_path,
            filter=str(self.get_parameter('filter').value),
            debug=False,
        )

        tracker = AnyGraspTracker(cfg)
        tracker.load_net()
        return tracker

    def _sync_cb(self, rgb_msg: Image, depth_msg: Image) -> None:
        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception:  # noqa: BLE001
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')

        try:
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to decode depth image: {exc}')
            return

        with self._lock:
            self._latest_rgb = rgb
            self._latest_depth = depth

    def _prepare_point_cloud(self, rgb_bgr: np.ndarray, depth: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)
        depth_scale = float(self.get_parameter('depth_scale').value)
        depth_max = float(self.get_parameter('depth_max').value)

        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / depth_scale
        else:
            depth_m = depth.astype(np.float32)

        height, width = depth_m.shape[:2]
        xmap, ymap = np.arange(width), np.arange(height)
        xmap, ymap = np.meshgrid(xmap, ymap)

        points_z = depth_m
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z

        points = np.stack([points_x, points_y, points_z], axis=-1)
        rgb = rgb_bgr[:, :, ::-1].astype(np.float32) / 255.0

        mask = (points_z > 0.0) & (points_z < depth_max)
        points = points[mask].astype(np.float32)
        colors = rgb[mask].astype(np.float32)
        return points, colors

    def _select_initial_grasp_ids(self, curr_gg) -> List[int]:
        x_min, x_max = [float(v) for v in self.get_parameter('select_x').value]
        y_min, y_max = [float(v) for v in self.get_parameter('select_y').value]
        z_min, z_max = [float(v) for v in self.get_parameter('select_z').value]
        select_count = int(self.get_parameter('select_count').value)

        translations = np.asarray(curr_gg.translations)
        mask_x = (translations[:, 0] > x_min) & (translations[:, 0] < x_max)
        mask_y = (translations[:, 1] > y_min) & (translations[:, 1] < y_max)
        mask_z = (translations[:, 2] > z_min) & (translations[:, 2] < z_max)
        candidate_ids = np.where(mask_x & mask_y & mask_z)[0]

        if candidate_ids.size == 0:
            return []

        # Sample across candidates like the demo (every Nth grasp).
        stride = max(1, int(np.ceil(candidate_ids.size / max(1, select_count))))
        selected = candidate_ids[: stride * select_count : stride]
        return [int(i) for i in selected]

    def _on_tracking(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        with self._lock:
            rgb = None if self._latest_rgb is None else self._latest_rgb.copy()
            depth = None if self._latest_depth is None else self._latest_depth.copy()

        if rgb is None or depth is None:
            response.success = False
            response.message = 'No synchronized /rgb_image and /depth_image received yet.'
            return response

        try:
            points, colors = self._prepare_point_cloud(rgb, depth)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'Failed to build point cloud: {exc}'
            return response

        try:
            grasp_ids = self._grasp_ids if self._grasp_ids else [0]
            target_gg, curr_gg, target_grasp_ids, _corres_preds = self._tracker.update(points, colors, grasp_ids)

            if not self._grasp_ids:
                initial_ids = self._select_initial_grasp_ids(curr_gg)
                if not initial_ids:
                    response.success = False
                    response.message = 'No suitable initial grasps found to start tracking.'
                    return response

                self._grasp_ids = initial_ids
                target_gg = curr_gg[self._grasp_ids]
            else:
                self._grasp_ids = [int(i) for i in np.asarray(target_grasp_ids).tolist()]

        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'AnyGrasp tracking update failed: {exc}'
            return response

        if len(target_gg) == 0:
            response.success = False
            response.message = 'Tracking produced no grasps.'
            return response

        translation = np.asarray(target_gg.translations[0]).reshape(3)
        rotation = np.asarray(target_gg.rotation_matrices[0]).reshape(3, 3)
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(rotation)

        response.success = True
        response.message = (
            f'x={translation[0]:.6f} y={translation[1]:.6f} z={translation[2]:.6f} '
            f'qx={qx:.6f} qy={qy:.6f} qz={qz:.6f} qw={qw:.6f}'
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnyGraspTrackingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
