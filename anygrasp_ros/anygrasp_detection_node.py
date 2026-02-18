"""ROS 2 node wrapper for AnyGrasp grasp detection."""

from __future__ import annotations

import os
import sys
import threading
from types import SimpleNamespace
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger



def _rotation_matrix_to_quaternion(matrix: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to (x, y, z, w) quaternion."""

    # Algorithm from https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
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


class AnyGraspDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('anygrasp_detection_node')

        self.declare_parameter('anygrasp_sdk_root', '/dependencies/anygrasp_sdk')
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('max_gripper_width', 0.10)
        self.declare_parameter('gripper_height', 0.03)
        self.declare_parameter('top_down_grasp', False)
        self.declare_parameter('apply_object_mask', True)
        self.declare_parameter('dense_grasp', False)
        self.declare_parameter('collision_detection', True)

        # Camera intrinsics (defaults match SDK demo.py)
        self.declare_parameter('fx', 927.17)
        self.declare_parameter('fy', 927.37)
        self.declare_parameter('cx', 651.32)
        self.declare_parameter('cy', 349.62)
        self.declare_parameter('depth_scale', 1000.0)
        self.declare_parameter('depth_max', 1.0)

        # Workspace limits (defaults match SDK demo.py)
        self.declare_parameter('lims', [-0.19, 0.12, 0.02, 0.15, 0.0, 1.0])

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None

        self._ensure_anygrasp_detection_on_path()
        self._anygrasp = self._init_anygrasp()

        self._rgb_sub = Subscriber(self, Image, 'rgb_image')
        self._depth_sub = Subscriber(self, Image, 'depth_image')
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._sync_cb)

        self._srv = self.create_service(Trigger, 'detection', self._on_detection)

        self.get_logger().info('AnyGrasp detection node ready.')

    def _ensure_anygrasp_detection_on_path(self) -> None:
        sdk_root = str(self.get_parameter('anygrasp_sdk_root').value)
        detection_dir = os.path.join(sdk_root, 'grasp_detection')

        if os.path.isdir(detection_dir) and detection_dir not in sys.path:
            sys.path.append(detection_dir)

    def _init_anygrasp(self):
        try:
            from gsnet import AnyGrasp  # type: ignore
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                'Failed to import AnyGrasp detection module `gsnet`. '
                'Check that the AnyGrasp SDK is available and that '
                '`anygrasp_sdk_root/grasp_detection` is on PYTHONPATH.'
            ) from exc

        checkpoint_path = str(self.get_parameter('checkpoint_path').value)
        if not checkpoint_path:
            self.get_logger().warn('Parameter `checkpoint_path` is empty; detection will fail until set.')

        max_gripper_width = float(self.get_parameter('max_gripper_width').value)
        max_gripper_width = max(0.0, min(0.1, max_gripper_width))

        cfg = SimpleNamespace(
            checkpoint_path=checkpoint_path,
            max_gripper_width=max_gripper_width,
            gripper_height=float(self.get_parameter('gripper_height').value),
            top_down_grasp=bool(self.get_parameter('top_down_grasp').value),
            debug=False,
        )

        anygrasp = AnyGrasp(cfg)
        anygrasp.load_net()
        return anygrasp

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

    def _on_detection(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
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

        lims = list(self.get_parameter('lims').value)

        try:
            gg, _cloud = self._anygrasp.get_grasp(
                points,
                colors,
                lims=lims,
                apply_object_mask=bool(self.get_parameter('apply_object_mask').value),
                dense_grasp=bool(self.get_parameter('dense_grasp').value),
                collision_detection=bool(self.get_parameter('collision_detection').value),
            )
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'AnyGrasp inference failed: {exc}'
            return response

        if len(gg) == 0:
            response.success = False
            response.message = 'No grasps detected.'
            return response

        try:
            gg = gg.nms().sort_by_score()
        except Exception:
            # If SDK version doesn’t provide these, keep original order.
            pass

        translation = np.asarray(gg.translations[0]).reshape(3)
        rotation = np.asarray(gg.rotation_matrices[0]).reshape(3, 3)
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(rotation)

        response.success = True
        response.message = (
            f'x={translation[0]:.6f} y={translation[1]:.6f} z={translation[2]:.6f} '
            f'qx={qx:.6f} qy={qy:.6f} qz={qz:.6f} qw={qw:.6f}'
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnyGraspDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
