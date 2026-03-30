"""ROS 2 node wrapper for AnyGrasp grasp tracking."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

import threading
from types import SimpleNamespace
from typing import List, Optional

import numpy as np

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import MarkerArray

from anygrasp_msgs.srv import GetGraspsTracked
from tracker import AnyGraspTracker  # type: ignore

from anygrasp_ros.node_utils import create_grasp_markers, rotation_matrix_to_quaternion


class AnyGraspTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('anygrasp_tracking_node')

        # Declare parameters
        self.declare_parameter('anygrasp_sdk_root', '/dependencies/anygrasp_sdk')
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('filter', 'oneeuro')
        self.declare_parameter('marker_topic', '/anygrasp/tracking_markers')
        self.declare_parameter('select_x', [-0.18, 0.18])
        self.declare_parameter('select_y', [-0.12, 0.12])
        self.declare_parameter('select_z', [0.35, 0.55])
        self.declare_parameter('select_count', 5)
        self.declare_parameter('input_pointcloud', '/pointcloud')

        self._lock = threading.Lock()
        self._params = SimpleNamespace()
        self._load_parameters()

        # Cache latest pointcloud
        self._latest_pointcloud: Optional[PointCloud2] = None

        # Initialize AnyGrasp tracker
        self._tracker = self._init_tracker()
        self._grasp_ids: List[int] = []

        self._marker_pub = self.create_publisher(MarkerArray, self._params.marker_topic, 10)

        # Subscribe to pointcloud from rgbd_to_pointcloud_node
        self._pointcloud_sub = self.create_subscription(
            PointCloud2, self._params.input_pointcloud, self._on_pointcloud, 10
        )

        # Create tracking service
        self._srv = self.create_service(GetGraspsTracked, '/anygrasp/tracking', self._on_tracking)

        self.get_logger().info('AnyGrasp tracking node ready.')

    def _load_parameters(self) -> None:
        """Load and cache parameters to avoid repeated get_parameter() calls."""
        self._params.anygrasp_sdk_root = str(self.get_parameter('anygrasp_sdk_root').value)
        self._params.checkpoint_path = str(self.get_parameter('checkpoint_path').value)
        self._params.filter = str(self.get_parameter('filter').value)
        self._params.marker_topic = str(self.get_parameter('marker_topic').value)
        self._params.select_x = [float(v) for v in list(self.get_parameter('select_x').value)]
        self._params.select_y = [float(v) for v in list(self.get_parameter('select_y').value)]
        self._params.select_z = [float(v) for v in list(self.get_parameter('select_z').value)]
        self._params.select_count = int(self.get_parameter('select_count').value)
        self._params.input_pointcloud = str(self.get_parameter('input_pointcloud').value)
        
    def _init_tracker(self):
        """Initialize AnyGrasp Tracker SDK."""
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

    def _select_initial_grasp_ids(self, curr_gg) -> List[int]:
        """Select initial grasp IDs based on workspace limits."""
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
            self.get_logger().warn('No grasps found in initial selection workspace.')
            return []

        # Sample across candidates
        stride = max(1, int(np.ceil(candidate_ids.size / max(1, select_count))))
        selected = candidate_ids[: stride * select_count : stride]
        return [int(i) for i in selected]

    def _grasp_to_pose(self, translation, rotation) -> Pose:
        """Convert tracker translation and rotation arrays to a ROS pose."""
        translation = np.asarray(translation).reshape(3)
        rotation = np.asarray(rotation).reshape(3, 3)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation)

        pose = Pose()
        pose.position.x = float(translation[0])
        pose.position.y = float(translation[1])
        pose.position.z = float(translation[2])
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    def _pose_to_stamped(self, pose: Pose, header) -> PoseStamped:
        """Attach the pointcloud header to a grasp pose."""
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = pose
        return pose_stamped

    def _unique_ids(self, ids: list[int]) -> list[int]:
        """Remove duplicate grasp ids while preserving order."""
        unique_ids: list[int] = []
        seen: set[int] = set()
        for grasp_id in ids:
            if grasp_id in seen:
                continue
            seen.add(grasp_id)
            unique_ids.append(grasp_id)
        return unique_ids

    def _on_tracking(
        self,
        request: GetGraspsTracked.Request,
        response: GetGraspsTracked.Response,
    ) -> GetGraspsTracked.Response:
        """Handle tracking service request."""
        requested_count = int(request.count)
        target_count = 1 if requested_count <= 0 else requested_count
        requested_ids = self._unique_ids([int(grasp_id) for grasp_id in request.input_ids])

        # Get latest pointcloud
        with self._lock:
            pointcloud = self._latest_pointcloud

        if pointcloud is None:
            self._publish_grasp_markers([], '', None)
            response.success = False
            response.message = 'No pointcloud received yet.'
            response.ids = []
            response.poses = []
            return response

        # Convert PointCloud2 to arrays
        points, colors = self._pointcloud2_to_arrays(pointcloud)

        if points is None or len(points) == 0:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'Invalid or empty pointcloud.'
            response.ids = []
            response.poses = []
            return response

        # Run AnyGrasp tracking
        try:
            result_ids: List[int] = []
            poses: List[Pose] = []
            stamped_poses: List[PoseStamped] = []
            # If no grasp IDs yet, run detection first to initialize
            if len(self._grasp_ids) == 0:
                # First detection
                curr_gg, _cloud = self._tracker.get_grasp(points, colors)

                if len(curr_gg) == 0:
                    self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
                    response.success = False
                    response.message = 'No grasps detected for initial tracking.'
                    response.ids = []
                    response.poses = []
                    return response

                try:
                    curr_gg = curr_gg.nms().sort_by_score()
                except Exception:
                    pass

                if requested_ids:
                    invalid_ids = [grasp_id for grasp_id in requested_ids if grasp_id < 0 or grasp_id >= len(curr_gg)]
                    if invalid_ids:
                        self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
                        response.success = False
                        response.message = f'Requested input_ids {invalid_ids} are not available in the initial grasp set.'
                        response.ids = []
                        response.poses = []
                        return response
                    self._grasp_ids = list(requested_ids)
                else:
                    # Select initial grasps
                    self._grasp_ids = self._select_initial_grasp_ids(curr_gg)

                    if len(self._grasp_ids) == 0:
                        # Fallback to top grasps if workspace selection failed
                        self._grasp_ids = list(range(min(target_count, len(curr_gg))))

                result_ids = list(self._grasp_ids[:target_count])
                poses = [
                    self._grasp_to_pose(curr_gg.translations[grasp_id], curr_gg.rotation_matrices[grasp_id])
                    for grasp_id in result_ids
                    if grasp_id < len(curr_gg)
                ]
                result_ids = result_ids[: len(poses)]
                stamped_poses = [self._pose_to_stamped(pose, pointcloud.header) for pose in poses]
            else:
                if requested_ids:
                    inactive_ids = [grasp_id for grasp_id in requested_ids if grasp_id not in self._grasp_ids]
                    if inactive_ids:
                        self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
                        response.success = False
                        response.message = f'Requested tracked ids {inactive_ids} are not active.'
                        response.ids = []
                        response.poses = []
                        return response
                    tracked_ids = list(requested_ids)
                else:
                    tracked_ids = list(self._grasp_ids)

                gg = self._tracker.track_grasps(points, colors, tracked_ids)
                surviving_ids = tracked_ids[: len(gg)]

                if requested_ids:
                    lost_ids = set(tracked_ids) - set(surviving_ids)
                    if lost_ids:
                        self._grasp_ids = [grasp_id for grasp_id in self._grasp_ids if grasp_id not in lost_ids]
                else:
                    self._grasp_ids = list(surviving_ids)

                result_ids = surviving_ids[:target_count]
                poses = [
                    self._grasp_to_pose(gg.translations[i], gg.rotation_matrices[i])
                    for i in range(len(result_ids))
                ]
                stamped_poses = [self._pose_to_stamped(pose, pointcloud.header) for pose in poses]

        except Exception as exc:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = f'AnyGrasp tracking failed: {exc}'
            response.ids = []
            response.poses = []
            return response

        if len(poses) == 0:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'No grasps tracked.'
            response.ids = []
            response.poses = []
            return response

        if len(self._grasp_ids) == 0:
            response.success = False
            response.message = 'No tracked grasp IDs available.'
            response.ids = []
            response.poses = []
            return response

        response.success = True
        response.ids = [int(grasp_id) for grasp_id in result_ids]
        response.poses = stamped_poses
        response.message = f'Returned {len(poses)} tracked grasp pose(s).'
        self._publish_grasp_markers(poses, pointcloud.header.frame_id, pointcloud.header.stamp)
        return response

    def _publish_grasp_markers(self, poses: list[Pose], frame_id: str, stamp) -> None:
        """Publish RViz markers for the current tracked grasp set."""
        marker_frame = frame_id or 'map'
        marker_stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        markers = create_grasp_markers(
            poses=poses,
            frame_id=marker_frame,
            stamp=marker_stamp,
            namespace='tracking_grasps',
            color=(1.0, 0.55, 0.1, 0.9),
        )
        self._marker_pub.publish(markers)


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
