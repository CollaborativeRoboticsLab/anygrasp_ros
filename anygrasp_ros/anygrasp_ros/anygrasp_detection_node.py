"""ROS 2 node wrapper for AnyGrasp grasp detection."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

import threading
from types import SimpleNamespace
from typing import Optional

import numpy as np

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import MarkerArray

from anygrasp_msgs.srv import GetGrasps
from gsnet import AnyGrasp

from anygrasp_ros.node_utils import create_grasp_markers, rotation_matrix_to_quaternion


class AnyGraspDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('anygrasp_detection_node')

        # Declare parameters
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('max_gripper_width', 0.10)
        self.declare_parameter('gripper_height', 0.03)
        self.declare_parameter('top_down_grasp', False)
        self.declare_parameter('apply_object_mask', True)
        self.declare_parameter('dense_grasp', False)
        self.declare_parameter('collision_detection', True)
        self.declare_parameter('marker_topic', '/anygrasp/detection_markers')
        self.declare_parameter('lims', [-0.19, 0.12, 0.02, 0.15, 0.0, 1.0])
        self.declare_parameter('input_pointcloud', '/pointcloud')

        self._lock = threading.Lock()
        self._params = SimpleNamespace()
        self._load_parameters()

        # Cache latest pointcloud
        self._latest_pointcloud: Optional[PointCloud2] = None

        # Initialize AnyGrasp
        self._anygrasp = self._init_anygrasp()

        self._marker_pub = self.create_publisher(MarkerArray, self._params.marker_topic, 10)

        # Subscribe to pointcloud from rgbd_to_pointcloud_node
        self._pointcloud_sub = self.create_subscription(
            PointCloud2, self._params.input_pointcloud, self._on_pointcloud, 10
        )

        # Create detection service
        self._srv = self.create_service(GetGrasps, '/anygrasp/detection', self._on_detection)

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
        self._params.marker_topic = str(self.get_parameter('marker_topic').value)
        self._params.lims = [float(v) for v in list(self.get_parameter('lims').value)]
        self._params.input_pointcloud = str(self.get_parameter('input_pointcloud').value)

    def _init_anygrasp(self):
        """Initialize AnyGrasp SDK."""
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

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        """Store the latest pointcloud."""
        with self._lock:
            self._latest_pointcloud = msg

    def _pointcloud2_to_arrays(self, msg: PointCloud2) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Convert PointCloud2 message to points and colors arrays.
        
        Expects PointCloud2 with fields: x, y, z, rgb (packed as uint32 0xRRGGBB)
        
        Returns:
            Tuple of (points, colors) or (None, None) if conversion fails
        """
        try:
            # Get raw point data as bytes
            num_points = msg.width
            point_step = msg.point_step
            
            # Extract x, y, z coordinates
            x_offset = 0
            y_offset = 4
            z_offset = 8
            rgb_offset = 12
            
            points = np.zeros((num_points, 3), dtype=np.float32)
            colors = np.zeros((num_points, 3), dtype=np.float32)
            
            for i in range(num_points):
                # Parse xyz
                idx = i * point_step
                points[i, 0] = np.frombuffer(msg.data, dtype=np.float32, count=1, offset=idx + x_offset)[0]
                points[i, 1] = np.frombuffer(msg.data, dtype=np.float32, count=1, offset=idx + y_offset)[0]
                points[i, 2] = np.frombuffer(msg.data, dtype=np.float32, count=1, offset=idx + z_offset)[0]
                
                # Parse RGB (packed as uint32 0xRRGGBB)
                rgb_uint32 = np.frombuffer(msg.data, dtype=np.uint32, count=1, offset=idx + rgb_offset)[0]
                r = (rgb_uint32 >> 16) & 0xFF
                g = (rgb_uint32 >> 8) & 0xFF
                b = rgb_uint32 & 0xFF
                colors[i, 0] = r / 255.0
                colors[i, 1] = g / 255.0
                colors[i, 2] = b / 255.0
            
            # Filter out invalid points (z == 0 or NaN)
            valid_mask = (points[:, 2] > 0) & np.isfinite(points).all(axis=1)
            points = points[valid_mask].astype(np.float32)
            colors = colors[valid_mask].astype(np.float32)
            
            return points, colors

        except Exception as exc:
            self.get_logger().warn(f'Failed to parse PointCloud2: {exc}')
            return None, None

    def _on_detection(self, request: GetGrasps.Request, response: GetGrasps.Response) -> GetGrasps.Response:
        """Handle detection service request."""
        requested_count = int(request.count)
        target_count = 1 if requested_count <= 0 else requested_count

        # Get latest pointcloud
        with self._lock:
            pointcloud = self._latest_pointcloud

        if pointcloud is None:
            self._publish_grasp_markers([], '', None)
            response.success = False
            response.message = 'No pointcloud received yet.'
            response.poses = []
            return response

        # Convert PointCloud2 to arrays
        points, colors = self._pointcloud2_to_arrays(pointcloud)

        if points is None or len(points) == 0:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'Invalid or empty pointcloud.'
            response.poses = []
            return response

        lims = list(self._params.lims)

        # Run AnyGrasp detection
        try:
            gg, _cloud = self._anygrasp.get_grasp(
                points,
                colors,
                lims=lims,
                apply_object_mask=bool(self._params.apply_object_mask),
                dense_grasp=bool(self._params.dense_grasp),
                collision_detection=bool(self._params.collision_detection),
            )
        except Exception as exc:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = f'AnyGrasp inference failed: {exc}'
            response.poses = []
            return response

        if len(gg) == 0:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'No grasps detected.'
            response.poses = []
            return response

        try:
            gg = gg.nms().sort_by_score()
        except Exception:
            # If SDK version doesn't provide these methods, keep original order
            pass

        count = min(int(len(gg)), target_count)

        # Convert grasp objects to poses
        poses = []
        stamped_poses = []
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

            pose_stamped = PoseStamped()
            pose_stamped.header = pointcloud.header
            pose_stamped.pose = pose
            stamped_poses.append(pose_stamped)

        response.success = True
        response.poses = stamped_poses
        response.message = f'Returned {count} grasp pose(s).'
        self._publish_grasp_markers(poses, pointcloud.header.frame_id, pointcloud.header.stamp)
        return response

    def _publish_grasp_markers(self, poses: list[Pose], frame_id: str, stamp) -> None:
        """Publish RViz markers for the current grasp set."""
        marker_frame = frame_id or 'map'
        marker_stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        markers = create_grasp_markers(
            poses=poses,
            frame_id=marker_frame,
            stamp=marker_stamp,
            namespace='detection_grasps',
            color=(0.1, 0.9, 0.2, 0.9),
        )
        self._marker_pub.publish(markers)


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
