"""ROS 2 node wrapper for AnyGrasp grasp detection."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

import threading
import importlib.util
import os
import sys
import time
from types import SimpleNamespace
from typing import Optional

import numpy as np

from sensor_msgs.msg import CameraInfo, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import MarkerArray

from anygrasp_msgs.srv import GetFilteredGrasps, GetGrasps
from anygrasp_ros.node_utils import (
    camera_info_to_intrinsics,
    create_grasp_markers,
    get_precompiled_module_path,
    log_anygrasp_license_status,
    rotation_matrix_to_quaternion,
)


def _load_gsnet_module():
    """Load gsnet module with a fallback to the known precompiled extension path."""
    try:
        import gsnet as module  # type: ignore
        return module
    except Exception:
        # Recover from partially initialized or shadowed gsnet imports.
        module_path = get_precompiled_module_path('gsnet.so')
        if not os.path.isfile(module_path):
            raise

        sys.modules.pop('gsnet', None)
        spec = importlib.util.spec_from_file_location('gsnet', module_path)
        if spec is None or spec.loader is None:
            raise ImportError(f'Unable to create import spec for {module_path}')

        module = importlib.util.module_from_spec(spec)
        sys.modules['gsnet'] = module
        spec.loader.exec_module(module)
        return module


GSNET_MODULE = _load_gsnet_module()

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
        self.declare_parameter('use_color_camera_info_topic', True)
        self.declare_parameter('color_camera_info_topic_name', '/camera/color/camera_info')
        self.declare_parameter('fx', 927.17)
        self.declare_parameter('fy', 927.37)
        self.declare_parameter('cx', 651.32)
        self.declare_parameter('cy', 349.62)

        self._lock = threading.Lock()
        self._params = SimpleNamespace()
        self._load_parameters()
        self._detector_api = 'unknown'
        self._color_intrinsics: Optional[SimpleNamespace] = None

        # Cache latest pointcloud
        self._latest_pointcloud: Optional[PointCloud2] = None

        # Initialize AnyGrasp
        self._anygrasp = self._init_anygrasp()
        log_anygrasp_license_status(
            logger=self.get_logger(),
            module=GSNET_MODULE,
            module_name='gsnet.so',
        )

        self._marker_pub = self.create_publisher(MarkerArray, self._params.marker_topic, 10)

        # Subscribe to pointcloud from rgbd_to_pointcloud_node
        self._pointcloud_sub = self.create_subscription(
            PointCloud2, self._params.input_pointcloud, self._on_pointcloud, 10
        )
        self._setup_camera_info_subscription()

        # Create detection service
        self._srv = self.create_service(GetGrasps, '/anygrasp/detection', self._on_detection)
        self._filtered_srv = self.create_service(
            GetFilteredGrasps,
            '/anygrasp/detection/filtered',
            self._on_filtered_detection,
        )

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
        self._params.use_color_camera_info_topic = bool(
            self.get_parameter('use_color_camera_info_topic').value
        )
        self._params.color_camera_info_topic_name = str(
            self.get_parameter('color_camera_info_topic_name').value
        )
        self._params.fx = float(self.get_parameter('fx').value)
        self._params.fy = float(self.get_parameter('fy').value)
        self._params.cx = float(self.get_parameter('cx').value)
        self._params.cy = float(self.get_parameter('cy').value)

    def _setup_camera_info_subscription(self) -> None:
        """Subscribe to the color CameraInfo stream if requested."""
        if self._params.use_color_camera_info_topic and self._params.color_camera_info_topic_name:
            self.create_subscription(
                CameraInfo,
                self._params.color_camera_info_topic_name,
                self._on_color_camera_info,
                10,
            )
            self.get_logger().info(
                f'Using color CameraInfo from: {self._params.color_camera_info_topic_name}'
            )

    def _on_color_camera_info(self, msg: CameraInfo) -> None:
        """Cache the latest color camera intrinsics."""
        intrinsics = camera_info_to_intrinsics(msg)
        if intrinsics is None:
            self.get_logger().warn('Received invalid color CameraInfo for filtered grasp projection.')
            return

        with self._lock:
            self._color_intrinsics = intrinsics

    def _get_color_intrinsics(self) -> tuple[float, float, float, float]:
        """Return color-frame intrinsics used for grasp reprojection."""
        with self._lock:
            intrinsics = self._color_intrinsics

        if intrinsics is not None:
            return (
                float(intrinsics.fx),
                float(intrinsics.fy),
                float(intrinsics.cx),
                float(intrinsics.cy),
            )

        return (
            float(self._params.fx),
            float(self._params.fy),
            float(self._params.cx),
            float(self._params.cy),
        )

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

        if hasattr(GSNET_MODULE, 'AnyGrasp'):
            anygrasp = GSNET_MODULE.AnyGrasp(cfg)
            if hasattr(anygrasp, 'load_net'):
                anygrasp.load_net()
            self._detector_api = 'legacy_anygrasp'
            self.get_logger().info('Using legacy AnyGrasp API from gsnet.AnyGrasp')
            return anygrasp

        if hasattr(GSNET_MODULE, 'create_detector'):
            detector = GSNET_MODULE.create_detector(cfg)
            if detector is None:
                raise RuntimeError('gsnet.create_detector returned None')
            self._detector_api = 'create_detector'
            self.get_logger().info('Using create_detector API from gsnet module')
            return detector

        raise ImportError('gsnet module does not expose AnyGrasp or create_detector APIs.')

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

    def _project_points_to_pixels(
        self,
        points: np.ndarray,
        image_width: int,
        image_height: int,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Project color-frame 3D points into the selected 2D image plane."""
        fx, fy, cx, cy = self._get_color_intrinsics()

        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if points.size == 0:
            return (
                np.array([], dtype=np.int32),
                np.array([], dtype=np.int32),
                np.array([], dtype=bool),
            )

        z_positive = points[:, 2] > 1e-6
        u = np.zeros(points.shape[0], dtype=np.int32)
        v = np.zeros(points.shape[0], dtype=np.int32)

        valid_indices = np.where(z_positive)[0]
        if valid_indices.size > 0:
            valid_points = points[valid_indices]
            u[valid_indices] = np.round(valid_points[:, 0] * fx / valid_points[:, 2] + cx).astype(np.int32)
            v[valid_indices] = np.round(valid_points[:, 1] * fy / valid_points[:, 2] + cy).astype(np.int32)

        in_bounds = (
            z_positive
            & (u >= 0)
            & (u < int(image_width))
            & (v >= 0)
            & (v < int(image_height))
        )
        return u, v, in_bounds

    def _bbox_mask_from_pixels(
        self,
        u: np.ndarray,
        v: np.ndarray,
        in_bounds: np.ndarray,
        center_x: int,
        center_y: int,
        size_w: int,
        size_h: int,
    ) -> np.ndarray:
        """Return a mask selecting projected points inside the requested bbox."""
        half_w = max(1, int(size_w)) / 2.0
        half_h = max(1, int(size_h)) / 2.0
        min_u = int(np.floor(float(center_x) - half_w))
        max_u = int(np.ceil(float(center_x) + half_w))
        min_v = int(np.floor(float(center_y) - half_h))
        max_v = int(np.ceil(float(center_y) + half_h))

        return (
            in_bounds
            & (u >= min_u)
            & (u <= max_u)
            & (v >= min_v)
            & (v <= max_v)
        )

    def _run_anygrasp_detection(
        self,
        points: np.ndarray,
        colors: np.ndarray,
        lims: list[float],
    ):
        """Run AnyGrasp detection on the provided point subset."""
        if self._detector_api == 'legacy_anygrasp':
            gg, _cloud = self._anygrasp.get_grasp(
                points,
                colors,
                lims=lims,
                apply_object_mask=bool(self._params.apply_object_mask),
                dense_grasp=bool(self._params.dense_grasp),
                collision_detection=bool(self._params.collision_detection),
            )
            return gg

        workspace_mask = (
            (points[:, 0] >= lims[0])
            & (points[:, 0] <= lims[1])
            & (points[:, 1] >= lims[2])
            & (points[:, 1] <= lims[3])
            & (points[:, 2] >= lims[4])
            & (points[:, 2] <= lims[5])
        )
        optional_params = {
            'dense_grasp': bool(self._params.dense_grasp),
            'collision_detection': bool(self._params.collision_detection),
            'region_steering': workspace_mask,
        }
        return self._anygrasp.get_grasp(points, optional_params)

    def _select_grasp_indices_in_bbox(
        self,
        gg,
        image_width: int,
        image_height: int,
        center_x: int,
        center_y: int,
        size_w: int,
        size_h: int,
    ) -> list[int]:
        """Keep grasps whose centers project back into the selected bbox."""
        translations = np.asarray(gg.translations, dtype=np.float32).reshape(-1, 3)
        if translations.size == 0:
            return []

        u, v, in_bounds = self._project_points_to_pixels(translations, image_width, image_height)
        mask = self._bbox_mask_from_pixels(u, v, in_bounds, center_x, center_y, size_w, size_h)
        return [int(index) for index in np.where(mask)[0].tolist()]

    def _build_response_poses(self, gg, header, selected_indices: list[int]) -> tuple[list[Pose], list[PoseStamped]]:
        """Convert selected AnyGrasp outputs to Pose and PoseStamped lists."""
        poses = []
        stamped_poses = []

        for index in selected_indices:
            translation = np.asarray(gg.translations[index]).reshape(3)
            rotation = np.asarray(gg.rotation_matrices[index]).reshape(3, 3)
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
            pose_stamped.header = header
            pose_stamped.pose = pose
            stamped_poses.append(pose_stamped)

        return poses, stamped_poses

    def _on_detection(self, request: GetGrasps.Request, response: GetGrasps.Response) -> GetGrasps.Response:
        """Handle detection service request."""
        start_time = time.perf_counter()
        requested_count = int(request.count)
        target_count = 1 if requested_count <= 0 else requested_count
        self.get_logger().info(
            f'Detection request received: count={requested_count}, target_count={target_count}'
        )

        # Get latest pointcloud
        with self._lock:
            pointcloud = self._latest_pointcloud

        if pointcloud is None:
            self._publish_grasp_markers([], '', None)
            response.success = False
            response.message = 'No pointcloud received yet.'
            response.poses = []
            return self._log_detection_response(response, start_time)

        # Convert PointCloud2 to arrays
        points, colors = self._pointcloud2_to_arrays(pointcloud)

        if points is None or len(points) == 0:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'Invalid or empty pointcloud.'
            response.poses = []
            return self._log_detection_response(response, start_time)

        self.get_logger().info(
            f'Detection input ready: frame={pointcloud.header.frame_id}, points={len(points)}'
        )

        lims = list(self._params.lims)

        # Run AnyGrasp detection
        try:
            gg = self._run_anygrasp_detection(points, colors, lims)
        except Exception as exc:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = f'AnyGrasp inference failed: {exc}'
            response.poses = []
            return self._log_detection_response(response, start_time)

        if len(gg) == 0:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'No grasps detected.'
            response.poses = []
            return self._log_detection_response(response, start_time)

        try:
            gg = gg.nms().sort_by_score()
        except Exception:
            # If SDK version doesn't provide these methods, keep original order
            pass

        count = min(int(len(gg)), target_count)
        selected_indices = list(range(count))
        poses, stamped_poses = self._build_response_poses(gg, pointcloud.header, selected_indices)

        response.success = True
        response.poses = stamped_poses
        response.message = f'Returned {count} grasp pose(s).'
        self._publish_grasp_markers(poses, pointcloud.header.frame_id, pointcloud.header.stamp)
        self._log_detection_response(response, start_time)
        return response

    def _on_filtered_detection(
        self,
        request: GetFilteredGrasps.Request,
        response: GetFilteredGrasps.Response,
    ) -> GetFilteredGrasps.Response:
        """Handle bbox-guided filtered grasp detection requests."""
        start_time = time.perf_counter()
        requested_count = int(request.count)
        target_count = 1 if requested_count <= 0 else requested_count

        self.get_logger().info(
            'Filtered detection request received: '
            f'count={requested_count}, detection_id={int(request.detection_id)}, '
            f'class_id={int(request.class_id)}, class_name="{request.class_name}"'
        )

        if int(request.image_width) <= 0 or int(request.image_height) <= 0:
            response.success = False
            response.message = 'Filtered grasp request requires positive image_width and image_height.'
            response.poses = []
            return self._log_filtered_detection_response(response, start_time)

        if int(request.bbx_size_w) <= 0 or int(request.bbx_size_h) <= 0:
            response.success = False
            response.message = 'Filtered grasp request requires positive bounding-box width and height.'
            response.poses = []
            return self._log_filtered_detection_response(response, start_time)

        with self._lock:
            pointcloud = self._latest_pointcloud

        if pointcloud is None:
            self._publish_grasp_markers([], '', None)
            response.success = False
            response.message = 'No pointcloud received yet.'
            response.poses = []
            return self._log_filtered_detection_response(response, start_time)

        points, colors = self._pointcloud2_to_arrays(pointcloud)
        if points is None or len(points) == 0:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'Invalid or empty pointcloud.'
            response.poses = []
            return self._log_filtered_detection_response(response, start_time)

        u, v, in_bounds = self._project_points_to_pixels(
            points,
            int(request.image_width),
            int(request.image_height),
        )
        bbox_mask = self._bbox_mask_from_pixels(
            u,
            v,
            in_bounds,
            int(request.bbx_center_x),
            int(request.bbx_center_y),
            int(request.bbx_size_w),
            int(request.bbx_size_h),
        )

        filtered_points = points[bbox_mask]
        filtered_colors = colors[bbox_mask]
        if filtered_points.shape[0] == 0:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'No pointcloud samples projected inside the selected bounding box.'
            response.poses = []
            return self._log_filtered_detection_response(response, start_time)

        lims = list(self._params.lims)
        try:
            gg = self._run_anygrasp_detection(filtered_points, filtered_colors, lims)
        except Exception as exc:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = f'AnyGrasp filtered inference failed: {exc}'
            response.poses = []
            return self._log_filtered_detection_response(response, start_time)

        if len(gg) == 0:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'No grasps detected inside the selected bounding box.'
            response.poses = []
            return self._log_filtered_detection_response(response, start_time)

        try:
            gg = gg.nms().sort_by_score()
        except Exception:
            pass

        selected_indices = self._select_grasp_indices_in_bbox(
            gg,
            int(request.image_width),
            int(request.image_height),
            int(request.bbx_center_x),
            int(request.bbx_center_y),
            int(request.bbx_size_w),
            int(request.bbx_size_h),
        )
        if not selected_indices:
            self._publish_grasp_markers([], pointcloud.header.frame_id, pointcloud.header.stamp)
            response.success = False
            response.message = 'AnyGrasp returned grasps, but none reprojected into the selected bounding box.'
            response.poses = []
            return self._log_filtered_detection_response(response, start_time)

        selected_indices = selected_indices[:target_count]
        poses, stamped_poses = self._build_response_poses(gg, pointcloud.header, selected_indices)

        response.success = True
        response.poses = stamped_poses
        response.message = (
            f'Returned {len(stamped_poses)} filtered grasp pose(s) for detection_id={int(request.detection_id)}.'
        )
        self._publish_grasp_markers(poses, pointcloud.header.frame_id, pointcloud.header.stamp)
        return self._log_filtered_detection_response(response, start_time)

    def _log_detection_response(
        self,
        response: GetGrasps.Response,
        start_time: float,
    ) -> GetGrasps.Response:
        """Log detection service response summary before returning it."""
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        pose_count = len(response.poses)
        self.get_logger().info(
            f'Detection response sent: success={response.success}, poses={pose_count}, '
            f'elapsed_ms={elapsed_ms:.1f}, message="{response.message}"'
        )
        return response

    def _log_filtered_detection_response(
        self,
        response: GetFilteredGrasps.Response,
        start_time: float,
    ) -> GetFilteredGrasps.Response:
        """Log filtered detection response summary before returning it."""
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        pose_count = len(response.poses)
        self.get_logger().info(
            f'Filtered detection response sent: success={response.success}, poses={pose_count}, '
            f'elapsed_ms={elapsed_ms:.1f}, message="{response.message}"'
        )
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
