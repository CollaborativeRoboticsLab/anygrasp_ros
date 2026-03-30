"""Shared helpers for AnyGrasp ROS 2 nodes."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Optional, Sequence, Tuple

import numpy as np

try:
	from sensor_msgs.msg import CameraInfo
except Exception:  # pragma: no cover
	CameraInfo = object  # type: ignore

try:
	from geometry_msgs.msg import Pose
	from visualization_msgs.msg import Marker, MarkerArray
except Exception:  # pragma: no cover
	Pose = object  # type: ignore
	Marker = object  # type: ignore
	MarkerArray = object  # type: ignore


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> Tuple[float, float, float, float]:
	"""Convert a 3x3 rotation matrix to (x, y, z, w) quaternion."""

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


def camera_info_to_intrinsics(msg: CameraInfo) -> Optional[SimpleNamespace]:
	"""Extract (fx, fy, cx, cy) from a ROS CameraInfo message."""

	k = getattr(msg, 'k', None)
	if k is None or len(k) != 9:
		return None

	fx = float(k[0])
	fy = float(k[4])
	cx = float(k[2])
	cy = float(k[5])
	if fx <= 0.0 or fy <= 0.0:
		return None

	return SimpleNamespace(fx=fx, fy=fy, cx=cx, cy=cy)


def get_point_cloud_intrinsics(
	*,
	use_depth_camera_info_topic: bool,
	depth_camera_info_topic_name: str,
	depth_intrinsics: Optional[SimpleNamespace],
	use_color_camera_info_topic: bool,
	color_camera_info_topic_name: str,
	color_intrinsics: Optional[SimpleNamespace],
	fx: float,
	fy: float,
	cx: float,
	cy: float,
) -> Tuple[float, float, float, float]:
	"""Select intrinsics for depth->3D projection.

	Preference order:
	1) depth CameraInfo (if enabled and received)
	2) color CameraInfo (if enabled and received)
	3) parameter intrinsics
	"""

	if use_depth_camera_info_topic and depth_camera_info_topic_name:
		if depth_intrinsics is None:
			raise RuntimeError(f'Waiting for CameraInfo on {depth_camera_info_topic_name}')
		return (
			float(depth_intrinsics.fx),
			float(depth_intrinsics.fy),
			float(depth_intrinsics.cx),
			float(depth_intrinsics.cy),
		)

	if use_color_camera_info_topic and color_camera_info_topic_name:
		if color_intrinsics is None:
			raise RuntimeError(f'Waiting for CameraInfo on {color_camera_info_topic_name}')
		return (
			float(color_intrinsics.fx),
			float(color_intrinsics.fy),
			float(color_intrinsics.cx),
			float(color_intrinsics.cy),
		)

	return float(fx), float(fy), float(cx), float(cy)


def prepare_point_cloud(
	*,
	rgb_bgr: np.ndarray,
	depth: np.ndarray,
	fx: float,
	fy: float,
	cx: float,
	cy: float,
	depth_scale: float,
	depth_max: float,
) -> Tuple[np.ndarray, np.ndarray]:
	"""Build point cloud and per-point colors from aligned RGB + depth."""

	if depth.dtype == np.uint16:
		depth_m = depth.astype(np.float32) / float(depth_scale)
	else:
		depth_m = depth.astype(np.float32)

	height, width = depth_m.shape[:2]
	xmap, ymap = np.arange(width), np.arange(height)
	xmap, ymap = np.meshgrid(xmap, ymap)

	points_z = depth_m
	points_x = (xmap - float(cx)) / float(fx) * points_z
	points_y = (ymap - float(cy)) / float(fy) * points_z

	points = np.stack([points_x, points_y, points_z], axis=-1)

	rgb = rgb_bgr[:, :, ::-1].astype(np.float32) / 255.0

	mask = (points_z > 0.0) & (points_z < float(depth_max))
	points = points[mask].astype(np.float32)
	colors = rgb[mask].astype(np.float32)
	return points, colors


def annotate_grasps_on_image(
	*,
	rgb_bgr: np.ndarray,
	translations: np.ndarray,
	fx: float,
	fy: float,
	cx: float,
	cy: float,
	count: int,
	rotation_matrices: Optional[np.ndarray] = None,
	draw_orientation_axis: bool = False,
	axis_length: float = 0.05,
) -> np.ndarray:
	"""Draw grasp centers (and optionally orientation axis) onto an RGB image."""

	import cv2  # local import: only needed when debug publishing is enabled

	annotated = rgb_bgr.copy()
	height, width = annotated.shape[:2]

	translations = np.asarray(translations)
	if translations.ndim != 2 or translations.shape[1] != 3:
		translations = translations.reshape(-1, 3)

	if rotation_matrices is not None:
		rotation_matrices = np.asarray(rotation_matrices)

	for i in range(int(count)):
		if i >= translations.shape[0]:
			break

		x, y, z = [float(v) for v in translations[i].reshape(3)]
		if z <= 0.0:
			continue

		u = int(round(float(fx) * x / z + float(cx)))
		v = int(round(float(fy) * y / z + float(cy)))
		if u < 0 or u >= width or v < 0 or v >= height:
			continue

		cv2.circle(annotated, (u, v), 6, (0, 255, 0), 2)
		cv2.putText(
			annotated,
			str(i),
			(u + 8, v - 8),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.5,
			(0, 255, 0),
			1,
			cv2.LINE_AA,
		)

		if draw_orientation_axis and rotation_matrices is not None and i < rotation_matrices.shape[0]:
			try:
				rot = rotation_matrices[i].reshape(3, 3).astype(np.float32)
				axis = rot[:, 0]
				axis = axis / max(1e-6, float(np.linalg.norm(axis)))
				tip = np.array([x, y, z], dtype=np.float32) + float(axis_length) * axis
				tx, ty, tz = [float(v) for v in tip.reshape(3)]
				if tz > 0.0:
					tu = int(round(float(fx) * tx / tz + float(cx)))
					tv = int(round(float(fy) * ty / tz + float(cy)))
					if 0 <= tu < width and 0 <= tv < height:
						cv2.line(annotated, (u, v), (tu, tv), (0, 255, 255), 2)
			except Exception:
				pass

	return annotated


def create_grasp_markers(
	*,
	poses: Sequence[Pose],
	frame_id: str,
	stamp,
	namespace: str,
	color: Tuple[float, float, float, float],
	arrow_length: float = 0.08,
	arrow_width: float = 0.01,
	text_height: float = 0.03,
	text_offset: float = 0.03,
) -> MarkerArray:
	"""Build RViz markers for grasp poses."""

	markers = MarkerArray()

	delete_all = Marker()
	delete_all.header.frame_id = frame_id
	delete_all.header.stamp = stamp
	delete_all.ns = namespace
	delete_all.id = 0
	delete_all.action = Marker.DELETEALL
	markers.markers.append(delete_all)

	r, g, b, a = [float(v) for v in color]

	for index, pose in enumerate(poses):
		arrow = Marker()
		arrow.header.frame_id = frame_id
		arrow.header.stamp = stamp
		arrow.ns = namespace
		arrow.id = index * 2 + 1
		arrow.type = Marker.ARROW
		arrow.action = Marker.ADD
		arrow.pose = pose
		arrow.scale.x = float(arrow_length)
		arrow.scale.y = float(arrow_width)
		arrow.scale.z = float(arrow_width)
		arrow.color.r = r
		arrow.color.g = g
		arrow.color.b = b
		arrow.color.a = a
		markers.markers.append(arrow)

		label = Marker()
		label.header.frame_id = frame_id
		label.header.stamp = stamp
		label.ns = f'{namespace}_labels'
		label.id = index * 2 + 2
		label.type = Marker.TEXT_VIEW_FACING
		label.action = Marker.ADD
		label.pose.position.x = float(pose.position.x)
		label.pose.position.y = float(pose.position.y)
		label.pose.position.z = float(pose.position.z) + float(text_offset)
		label.pose.orientation.w = 1.0
		label.scale.z = float(text_height)
		label.color.r = r
		label.color.g = g
		label.color.b = b
		label.color.a = a
		label.text = str(index)
		markers.markers.append(label)

	return markers
