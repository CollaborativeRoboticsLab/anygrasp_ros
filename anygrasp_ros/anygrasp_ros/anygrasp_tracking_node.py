"""ROS 2 node wrapper for AnyGrasp grasp tracking.

This node caches synchronized RGB + depth frames and runs the AnyGrasp tracking
update loop on demand via a Trigger service.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

import threading
from types import SimpleNamespace
from typing import List, Optional, Tuple

import numpy as np

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose

from anygrasp_msgs.srv import GetGrasps

from tracker import AnyGraspTracker  # type: ignore

from anygrasp_ros.node_utils import (
    annotate_grasps_on_image,
    camera_info_to_intrinsics,
    get_point_cloud_intrinsics,
    prepare_point_cloud,
    rotation_matrix_to_quaternion,
)


class AnyGraspTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('anygrasp_tracking_node')

        self.declare_parameter('anygrasp_sdk_root', '/dependencies/anygrasp_sdk')
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('filter', 'oneeuro')
        self.declare_parameter('publish_annotated_image', False)

        # Camera intrinsics from topics (if enabled) take precedence over these fx/fy/cx/cy parameters.
        # Depth scale is always read from the parameter since CameraInfo doesn’t include it.
        self.declare_parameter('use_color_camera_info_topic', False)
        self.declare_parameter('color_camera_info_topic_name', '')
        self.declare_parameter('use_depth_camera_info_topic', False)
        self.declare_parameter('depth_camera_info_topic_name', '')

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

        self._params = SimpleNamespace()
        self._color_intrinsics: Optional[SimpleNamespace] = None
        self._depth_intrinsics: Optional[SimpleNamespace] = None
        self._load_parameters()

        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None

        self._tracker = self._init_tracker()
        self._grasp_ids: List[int] = []

        self._setup_camera_info_subscriptions()

        self._annotated_pub = None
        if self._params.publish_annotated_image:
            self._annotated_pub = self.create_publisher(Image, 'annotated_image', 10)

        self._rgb_sub = Subscriber(self, Image, 'rgb_image')
        self._depth_sub = Subscriber(self, Image, 'depth_image')
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._sync_cb)

        self._srv = self.create_service(GetGrasps, 'tracking', self._on_tracking)

        self.get_logger().info('AnyGrasp tracking node ready.')

    def _load_parameters(self) -> None:
        self._params.anygrasp_sdk_root = str(self.get_parameter('anygrasp_sdk_root').value)
        self._params.checkpoint_path = str(self.get_parameter('checkpoint_path').value)
        self._params.filter = str(self.get_parameter('filter').value)
        self._params.publish_annotated_image = bool(self.get_parameter('publish_annotated_image').value)

        self._params.use_color_camera_info_topic = bool(self.get_parameter('use_color_camera_info_topic').value)
        self._params.color_camera_info_topic_name = str(self.get_parameter('color_camera_info_topic_name').value)
        self._params.use_depth_camera_info_topic = bool(self.get_parameter('use_depth_camera_info_topic').value)
        self._params.depth_camera_info_topic_name = str(self.get_parameter('depth_camera_info_topic_name').value)

        self._params.fx = float(self.get_parameter('fx').value)
        self._params.fy = float(self.get_parameter('fy').value)
        self._params.cx = float(self.get_parameter('cx').value)
        self._params.cy = float(self.get_parameter('cy').value)
        self._params.depth_scale = float(self.get_parameter('depth_scale').value)
        self._params.depth_max = float(self.get_parameter('depth_max').value)

        self._params.select_x = [float(v) for v in list(self.get_parameter('select_x').value)]
        self._params.select_y = [float(v) for v in list(self.get_parameter('select_y').value)]
        self._params.select_z = [float(v) for v in list(self.get_parameter('select_z').value)]
        self._params.select_count = int(self.get_parameter('select_count').value)

    def _init_tracker(self):
        if not self._params.checkpoint_path:
            self.get_logger().warn('Parameter `checkpoint_path` is empty; tracking will fail until set.')

        cfg = SimpleNamespace(
            checkpoint_path=self._params.checkpoint_path,
            filter=self._params.filter,
            debug=False,
        )

        tracker = AnyGraspTracker(cfg)
        tracker.load_net()
        return tracker

    def _setup_camera_info_subscriptions(self) -> None:
        self._color_info_sub = None
        self._depth_info_sub = None

        if self._params.use_color_camera_info_topic:
            topic = self._params.color_camera_info_topic_name
            if topic:
                self._color_info_sub = self.create_subscription(CameraInfo, topic, self._on_color_camera_info, 10)
                self.get_logger().info(f'Using color CameraInfo from topic: {topic}')
            else:
                self.get_logger().warn('`use_color_camera_info_topic` is true but `color_camera_info_topic_name` is empty; falling back to fx/fy/cx/cy params.')

        if self._params.use_depth_camera_info_topic:
            topic = self._params.depth_camera_info_topic_name
            if topic:
                self._depth_info_sub = self.create_subscription(CameraInfo, topic, self._on_depth_camera_info, 10)
                self.get_logger().info(f'Using depth CameraInfo from topic: {topic}')
            else:
                self.get_logger().warn('`use_depth_camera_info_topic` is true but `depth_camera_info_topic_name` is empty; falling back to fx/fy/cx/cy params.')

    def _camera_info_to_intrinsics(self, msg: CameraInfo) -> Optional[SimpleNamespace]:
        return camera_info_to_intrinsics(msg)

    def _on_color_camera_info(self, msg: CameraInfo) -> None:
        intr = self._camera_info_to_intrinsics(msg)
        if intr is None:
            self.get_logger().warn('Received invalid color CameraInfo; ignoring.')
            return
        with self._lock:
            self._color_intrinsics = intr

    def _on_depth_camera_info(self, msg: CameraInfo) -> None:
        intr = self._camera_info_to_intrinsics(msg)
        if intr is None:
            self.get_logger().warn('Received invalid depth CameraInfo; ignoring.')
            return
        with self._lock:
            self._depth_intrinsics = intr

    def _get_point_cloud_intrinsics(self) -> Tuple[float, float, float, float]:
        """Return (fx, fy, cx, cy) for projecting depth into 3D.

        Preference order:
        1) depth CameraInfo (if enabled and received)
        2) color CameraInfo (if enabled and received)
        3) fx/fy/cx/cy parameters
        """
        with self._lock:
            return get_point_cloud_intrinsics(
                use_depth_camera_info_topic=bool(self._params.use_depth_camera_info_topic),
                depth_camera_info_topic_name=str(self._params.depth_camera_info_topic_name),
                depth_intrinsics=self._depth_intrinsics,
                use_color_camera_info_topic=bool(self._params.use_color_camera_info_topic),
                color_camera_info_topic_name=str(self._params.color_camera_info_topic_name),
                color_intrinsics=self._color_intrinsics,
                fx=float(self._params.fx),
                fy=float(self._params.fy),
                cx=float(self._params.cx),
                cy=float(self._params.cy),
            )

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
        fx, fy, cx, cy = self._get_point_cloud_intrinsics()
        return prepare_point_cloud(
            rgb_bgr=rgb_bgr,
            depth=depth,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            depth_scale=float(self._params.depth_scale),
            depth_max=float(self._params.depth_max),
        )

    def _publish_annotated_image(self, rgb_bgr: np.ndarray, gg, count: int) -> None:
        if self._annotated_pub is None:
            return

        if rgb_bgr.ndim != 3 or rgb_bgr.shape[2] != 3:
            self.get_logger().warn('RGB image is not 3-channel; skipping annotated image publish.')
            return

        try:
            fx, fy, cx, cy = self._get_point_cloud_intrinsics()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Cannot publish annotated image (intrinsics unavailable): {exc}')
            return

        try:
            translations = np.asarray(gg.translations)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Cannot publish annotated image (missing translations): {exc}')
            return

        try:
            rotation_matrices = np.asarray(gg.rotation_matrices)
        except Exception:
            rotation_matrices = None

        annotated = annotate_grasps_on_image(
            rgb_bgr=rgb_bgr,
            translations=translations,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            count=int(count),
            rotation_matrices=rotation_matrices,
            draw_orientation_axis=True,
            axis_length=0.05,
        )

        msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self._annotated_pub.publish(msg)

    def _select_initial_grasp_ids(self, curr_gg) -> List[int]:
        x_min, x_max = [float(v) for v in self._params.select_x]
        y_min, y_max = [float(v) for v in self._params.select_y]
        z_min, z_max = [float(v) for v in self._params.select_z]
        select_count = int(self._params.select_count)

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

    def _on_tracking(self, request: GetGrasps.Request, response: GetGrasps.Response) -> GetGrasps.Response:
        requested_count = int(request.count)
        target_count = 1 if requested_count <= 0 else requested_count

        with self._lock:
            rgb = None if self._latest_rgb is None else self._latest_rgb.copy()
            depth = None if self._latest_depth is None else self._latest_depth.copy()

        if rgb is None or depth is None:
            response.success = False
            response.message = 'No synchronized /rgb_image and /depth_image received yet.'
            response.poses = []
            return response

        try:
            points, colors = self._prepare_point_cloud(rgb, depth)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'Failed to build point cloud: {exc}'
            response.poses = []
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
            response.poses = []
            return response

        if len(target_gg) == 0:
            response.success = False
            response.message = 'Tracking produced no grasps.'
            response.poses = []
            return response

        count = min(int(len(target_gg)), target_count)

        if self._params.publish_annotated_image:
            self._publish_annotated_image(rgb, target_gg, count)

        poses = []
        for i in range(count):
            translation = np.asarray(target_gg.translations[i]).reshape(3)
            rotation = np.asarray(target_gg.rotation_matrices[i]).reshape(3, 3)
            qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation)

            pose = Pose()
            pose.position.x = float(translation[0])
            pose.position.y = float(translation[1])
            pose.position.z = float(translation[2])
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)
            poses.append(pose)

        response.success = True
        response.poses = poses
        response.message = f'Returned {count} tracked grasp pose(s).'
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
