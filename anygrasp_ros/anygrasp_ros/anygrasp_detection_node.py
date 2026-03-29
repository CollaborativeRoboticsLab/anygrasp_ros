"""ROS 2 node wrapper for AnyGrasp grasp detection."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

import threading
from types import SimpleNamespace
from typing import Optional, Tuple

import numpy as np

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose

from anygrasp_msgs.srv import GetGrasps

from gsnet import AnyGrasp

from anygrasp_ros.node_utils import (
    annotate_grasps_on_image,
    camera_info_to_intrinsics,
    get_point_cloud_intrinsics,
    prepare_point_cloud,
    rotation_matrix_to_quaternion,
)


class AnyGraspDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('anygrasp_detection_node')

        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('max_gripper_width', 0.10)
        self.declare_parameter('gripper_height', 0.03)
        self.declare_parameter('top_down_grasp', False)
        self.declare_parameter('apply_object_mask', True)
        self.declare_parameter('dense_grasp', False)
        self.declare_parameter('collision_detection', True)
        self.declare_parameter('publish_annotated_image', False)

        # Camera intrinsics from topics (if enabled) take precedence over these fx/fy/cx/cy parameters. 
        # Depth scale is always read from the parameter since CameraInfo doesn’t include it.
        self.declare_parameter("use_color_camera_info_topic", False)
        self.declare_parameter("color_camera_info_topic_name", '')
        self.declare_parameter("use_depth_camera_info_topic", False)
        self.declare_parameter("depth_camera_info_topic_name", '')

        # These are only used if `use_color_camera_info_topic` is false. If true, intrinsics will be 
        # read from the specified topic instead.
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

        self._params = SimpleNamespace()
        self._color_intrinsics: Optional[SimpleNamespace] = None
        self._depth_intrinsics: Optional[SimpleNamespace] = None

        self._load_parameters()

        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None

        self._anygrasp = self._init_anygrasp()

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

        self._srv = self.create_service(GetGrasps, 'detection', self._on_detection)

        self.get_logger().info('AnyGrasp detection node ready.')

    def _load_parameters(self) -> None:
        """Load and cache parameters to avoid repeated get_parameter() calls."""
        checkpoint_path = str(self.get_parameter('checkpoint_path').value)
        max_gripper_width = float(self.get_parameter('max_gripper_width').value)

        self._params.checkpoint_path = checkpoint_path
        self._params.max_gripper_width = max(0.0, min(0.1, max_gripper_width))
        self._params.gripper_height = float(self.get_parameter('gripper_height').value)
        self._params.top_down_grasp = bool(self.get_parameter('top_down_grasp').value)
        self._params.apply_object_mask = bool(self.get_parameter('apply_object_mask').value)
        self._params.dense_grasp = bool(self.get_parameter('dense_grasp').value)
        self._params.collision_detection = bool(self.get_parameter('collision_detection').value)
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

        self._params.lims = [float(v) for v in list(self.get_parameter('lims').value)]

    def _init_anygrasp(self):
        if not self._params.checkpoint_path:
            self.get_logger().warn('Parameter `checkpoint_path` is empty; detection will fail until set.')

        cfg = SimpleNamespace(
            checkpoint_path=self._params.checkpoint_path,
            max_gripper_width=self._params.max_gripper_width,
            gripper_height=self._params.gripper_height,
            top_down_grasp=self._params.top_down_grasp,
            debug=False,
        )

        anygrasp = AnyGrasp(cfg)
        anygrasp.load_net()
        return anygrasp

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

        annotated = annotate_grasps_on_image(
            rgb_bgr=rgb_bgr,
            translations=translations,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            count=int(count),
            rotation_matrices=None,
            draw_orientation_axis=False,
        )

        msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self._annotated_pub.publish(msg)

    def _on_detection(self, request: GetGrasps.Request, response: GetGrasps.Response) -> GetGrasps.Response:
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

        lims = list(self._params.lims)

        try:
            gg, _cloud = self._anygrasp.get_grasp(
                points,
                colors,
                lims=lims,
                apply_object_mask=bool(self._params.apply_object_mask),
                dense_grasp=bool(self._params.dense_grasp),
                collision_detection=bool(self._params.collision_detection),
            )
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'AnyGrasp inference failed: {exc}'
            response.poses = []
            return response

        if len(gg) == 0:
            response.success = False
            response.message = 'No grasps detected.'
            response.poses = []
            return response

        try:
            gg = gg.nms().sort_by_score()
        except Exception:
            # If SDK version doesn’t provide these, keep original order.
            pass

        count = min(int(len(gg)), target_count)

        if self._params.publish_annotated_image:
            self._publish_annotated_image(rgb, gg, count)

        poses = []
        for i in range(count):
            translation = np.asarray(gg.translations[i]).reshape(3)
            rotation = np.asarray(gg.rotation_matrices[i]).reshape(3, 3)
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
        response.message = f'Returned {count} grasp pose(s).'
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
