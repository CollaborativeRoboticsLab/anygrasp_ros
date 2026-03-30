"""ROS 2 node for RGBD to PointCloud conversion with optional depth alignment.

This node handles:
- RGB + depth image synchronization
- Camera calibration (intrinsics from topics or parameters)
- Depth-to-color extrinsic alignment (optional)
- Point cloud generation
- Distributes RGBD processing load from detection/tracking nodes

Publishes:
- sensor_msgs/PointCloud2: aligned, colored point cloud
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

import threading
from types import SimpleNamespace
from typing import Optional, Tuple

import numpy as np

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import TransformListener, Buffer

from anygrasp_ros.node_utils import camera_info_to_intrinsics, get_point_cloud_intrinsics


class RGBDToPointCloudNode(Node):
    """Convert synchronized RGBD frames to point cloud with optional alignment."""

    def __init__(self) -> None:
        super().__init__('rgbd_to_pointcloud_node')

        # Image topics
        self.declare_parameter('rgb_image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('pointcloud_topic', '/pointcloud')

        # Camera calibration
        self.declare_parameter('use_color_camera_info_topic', True)
        self.declare_parameter('color_camera_info_topic_name', '/camera/color/camera_info')
        self.declare_parameter('use_depth_camera_info_topic', True)
        self.declare_parameter('depth_camera_info_topic_name', '/camera/depth/camera_info')

        self.declare_parameter('fx', 927.17)
        self.declare_parameter('fy', 927.37)
        self.declare_parameter('cx', 651.32)
        self.declare_parameter('cy', 349.62)
        self.declare_parameter('depth_scale', 1000.0)
        self.declare_parameter('depth_max', 1.5)
        self.declare_parameter('align_depth_to_color', True)

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._params = SimpleNamespace()
        self._color_intrinsics: Optional[SimpleNamespace] = None
        self._depth_intrinsics: Optional[SimpleNamespace] = None
        self._depth_to_color_rotation: Optional[np.ndarray] = None
        self._depth_to_color_translation: Optional[np.ndarray] = None
        
        # Frame names (extracted from image headers on first message)
        self._rgb_frame_id: Optional[str] = None
        self._depth_frame_id: Optional[str] = None
        self._tf_transform_queried = False

        self._load_parameters()

        # Setup TF listener for frame transforms
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None

        # Subscribe to camera info
        self._setup_camera_info_subscriptions()

        # Synchronized RGB + Depth subscription
        rgb_sub = Subscriber(self, Image, self._params.rgb_image_topic)
        depth_sub = Subscriber(self, Image, self._params.depth_image_topic)
        self._sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._on_rgbd_sync)

        # Publishers
        self._pointcloud_pub = self.create_publisher(
            PointCloud2, self._params.pointcloud_topic, 10
        )

        self.get_logger().info('RGBD to PointCloud node ready.')

    def _load_parameters(self) -> None:
        """Load and cache parameters."""
        self._params.rgb_image_topic = str(self.get_parameter('rgb_image_topic').value)
        self._params.depth_image_topic = str(self.get_parameter('depth_image_topic').value)
        self._params.pointcloud_topic = str(self.get_parameter('pointcloud_topic').value)

        self._params.use_color_camera_info_topic = bool(
            self.get_parameter('use_color_camera_info_topic').value
        )
        self._params.color_camera_info_topic_name = str(
            self.get_parameter('color_camera_info_topic_name').value
        )
        self._params.use_depth_camera_info_topic = bool(
            self.get_parameter('use_depth_camera_info_topic').value
        )
        self._params.depth_camera_info_topic_name = str(
            self.get_parameter('depth_camera_info_topic_name').value
        )

        self._params.fx = float(self.get_parameter('fx').value)
        self._params.fy = float(self.get_parameter('fy').value)
        self._params.cx = float(self.get_parameter('cx').value)
        self._params.cy = float(self.get_parameter('cy').value)
        self._params.depth_scale = float(self.get_parameter('depth_scale').value)
        self._params.depth_max = float(self.get_parameter('depth_max').value)
        self._params.align_depth_to_color = bool(self.get_parameter('align_depth_to_color').value)

    def _setup_camera_info_subscriptions(self) -> None:
        """Subscribe to camera info topics if enabled."""
        if self._params.use_color_camera_info_topic:
            topic = self._params.color_camera_info_topic_name
            if topic:
                self.create_subscription(CameraInfo, topic, self._on_color_camera_info, 10)
                self.get_logger().info(f'Using color CameraInfo from: {topic}')

        if self._params.use_depth_camera_info_topic:
            topic = self._params.depth_camera_info_topic_name
            if topic:
                self.create_subscription(CameraInfo, topic, self._on_depth_camera_info, 10)
                self.get_logger().info(f'Using depth CameraInfo from: {topic}')

    def _camera_info_to_intrinsics(self, msg: CameraInfo) -> Optional[SimpleNamespace]:
        """Extract (fx, fy, cx, cy) from CameraInfo."""
        return camera_info_to_intrinsics(msg)

    def _on_color_camera_info(self, msg: CameraInfo) -> None:
        """Update color camera intrinsics."""
        intr = self._camera_info_to_intrinsics(msg)
        if intr is None:
            self.get_logger().warn('Received invalid color CameraInfo.')
            return
        with self._lock:
            self._color_intrinsics = intr

    def _on_depth_camera_info(self, msg: CameraInfo) -> None:
        """Update depth camera intrinsics."""
        intr = self._camera_info_to_intrinsics(msg)
        if intr is None:
            self.get_logger().warn('Received invalid depth CameraInfo.')
            return
        with self._lock:
            self._depth_intrinsics = intr

    def _query_tf_transforms(self, depth_frame: str, color_frame: str) -> None:
        """Query TF tree for depth-to-color transform and cache rotation/translation.
        
        Args:
            depth_frame: Source frame (e.g., 'camera_depth_optical_frame')
            color_frame: Target frame (e.g., 'camera_color_optical_frame')
        """
        import time
        max_wait = 10.0
        start_time = time.time()
        
        while (time.time() - start_time) < max_wait:
            try:
                transform = self._tf_buffer.lookup_transform(
                    color_frame,
                    depth_frame,
                    rclpy.time.Time()
                )
                
                # Extract rotation (quaternion → rotation matrix)
                quat = transform.transform.rotation
                qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
                
                rotation = np.array([
                    [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                    [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
                    [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
                ])
                
                translation = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
                
                with self._lock:
                    self._depth_to_color_rotation = rotation
                    self._depth_to_color_translation = translation
                
                self.get_logger().info(
                    f'TF transform found: {depth_frame} → {color_frame}'
                )
                self.get_logger().info(
                    f'Rotation matrix:\n{rotation}'
                )
                self.get_logger().info(
                    f'Translation vector: {translation}'
                )
                return
            
            except Exception as exc:  # noqa: BLE001
                elapsed = time.time() - start_time
                self.get_logger().debug(
                    f'Waiting for TF transform... ({elapsed:.1f}s): {exc}'
                )
                time.sleep(0.1)
        
        self.get_logger().warn(
            f'TF transform not found within {max_wait:.1f}s. Point cloud will use depth frame only.'
        )

    def _get_depth_to_color_transform(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (rotation_matrix, translation_vector) for depth-to-color transformation."""
        with self._lock:
            return self._depth_to_color_rotation, self._depth_to_color_translation

    def _get_point_cloud_intrinsics(self) -> Tuple[float, float, float, float]:
        """Return (fx, fy, cx, cy) for depth to 3D projection."""
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

    def _on_rgbd_sync(self, rgb_msg: Image, depth_msg: Image) -> None:
        """Process synchronized RGB and depth images."""
        # On first message, extract frame IDs and query TF if alignment is enabled
        if not self._tf_transform_queried and self._params.align_depth_to_color:
            self._rgb_frame_id = rgb_msg.header.frame_id
            self._depth_frame_id = depth_msg.header.frame_id
            self.get_logger().info(
                f'Detected frames: RGB={self._rgb_frame_id}, Depth={self._depth_frame_id}'
            )
            self._query_tf_transforms(self._depth_frame_id, self._rgb_frame_id)
            self._tf_transform_queried = True
        
        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception:  # noqa: BLE001
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')

        try:
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to decode depth: {exc}')
            return

        # Generate point cloud with TF alignment + color lookup projection
        try:
            fx, fy, cx, cy = self._get_point_cloud_intrinsics()
            pointcloud = self._rgbd_to_pointcloud(rgb, depth, fx, fy, cx, cy)
            self._pointcloud_pub.publish(pointcloud)

        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Point cloud generation failed: {exc}')

    def _rgbd_to_pointcloud(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> PointCloud2:
        """Convert RGBD images to PointCloud2 message with TF alignment and color reprojection."""
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / float(self._params.depth_scale)
        else:
            depth_m = depth.astype(np.float32)

        h, w = depth_m.shape[:2]

        xmap, ymap = np.meshgrid(np.arange(w), np.arange(h))

        z = depth_m
        x = (xmap - float(cx)) / float(fx) * z
        y = (ymap - float(cy)) / float(fy) * z

        depth_max = float(self._params.depth_max)
        valid_mask = (z > 0.0) & (z < depth_max)

        if not np.any(valid_mask):
            return PointCloud2()

        points = np.stack([x[valid_mask], y[valid_mask], z[valid_mask]], axis=-1).astype(np.float32)

        # Transform points to color frame if TF transform is available
        frame_id = 'camera_depth_optical_frame'
        if self._params.align_depth_to_color:
            rot, trans = self._get_depth_to_color_transform()
            if rot is not None and trans is not None:
                points = (rot @ points.T).T + trans
                frame_id = self._rgb_frame_id or 'camera_color_optical_frame'

        # Choose color intrinsics (prefer camera info if available)
        if self._color_intrinsics is not None:
            fx_c, fy_c = self._color_intrinsics.fx, self._color_intrinsics.fy
            cx_c, cy_c = self._color_intrinsics.cx, self._color_intrinsics.cy
        else:
            fx_c, fy_c, cx_c, cy_c = fx, fy, cx, cy

        # Project points into color image to pick colors
        valid_project = points[:, 2] > 0.001
        points = points[valid_project]

        u = np.round(points[:, 0] * fx_c / points[:, 2] + cx_c).astype(np.int32)
        v = np.round(points[:, 1] * fy_c / points[:, 2] + cy_c).astype(np.int32)

        uv_mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        points = points[uv_mask]
        u = u[uv_mask]
        v = v[uv_mask]

        if points.shape[0] == 0:
            return PointCloud2()

        # Get colors from the RGB image at projected locations
        rgb_bgr = rgb.astype(np.uint8)
        bgr_colors = rgb_bgr[v, u, :]
        rgb_colors = bgr_colors[:, ::-1]  # BGR -> RGB

        # Build PointCloud2
        pointcloud = PointCloud2()
        pointcloud.header.frame_id = frame_id
        pointcloud.header.stamp = self.get_clock().now().to_msg()
        pointcloud.height = 1
        pointcloud.width = len(points)
        pointcloud.is_dense = False

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        pointcloud.fields = fields
        pointcloud.point_step = 16

        cloud_data = np.zeros((len(points),), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32),
        ])
        cloud_data['x'] = points[:, 0]
        cloud_data['y'] = points[:, 1]
        cloud_data['z'] = points[:, 2]

        rgb_packed = (
            (rgb_colors[:, 0].astype(np.uint32) << 16) |
            (rgb_colors[:, 1].astype(np.uint32) << 8) |
            rgb_colors[:, 2].astype(np.uint32)
        )
        cloud_data['rgb'] = rgb_packed

        pointcloud.data = cloud_data.tobytes()
        pointcloud.row_step = len(pointcloud.data)

        return pointcloud


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RGBDToPointCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
