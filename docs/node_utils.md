# Node utilities

This package includes shared helpers used by both the detection and tracking nodes.

File:

- `anygrasp_ros/node_utils.py`

## Functions

### `rotation_matrix_to_quaternion(matrix)`

Converts a 3×3 rotation matrix to a quaternion `(x, y, z, w)`.

Used when converting AnyGrasp grasp rotation matrices into `geometry_msgs/Pose`.

### `camera_info_to_intrinsics(msg)`

Extracts pinhole intrinsics from a `sensor_msgs/CameraInfo` message:

- `fx = K[0]`
- `fy = K[4]`
- `cx = K[2]`
- `cy = K[5]`

Returns `None` if the message is missing/invalid.

### `get_point_cloud_intrinsics(...)`

Selects the intrinsics used to project depth into 3D. The selection order is:

1) depth `CameraInfo` (if enabled + received)
2) color `CameraInfo` (if enabled + received)
3) `fx/fy/cx/cy` parameters

If topic-based intrinsics are enabled but not yet received, this function raises a `RuntimeError` (the nodes catch this and return `success=false`).

### `prepare_point_cloud(...)`

Builds a point cloud from aligned RGB + depth inputs:

- Converts depth to meters (if `uint16`, divides by `depth_scale`)
- Projects pixels to 3D using `(fx, fy, cx, cy)`
- Filters points with `0 < z < depth_max`
- Returns `(points, colors)` where:
	- `points` is `N×3` float32 in meters
	- `colors` is `N×3` float32 in RGB in `[0, 1]`

### `annotate_grasps_on_image(...)`

Creates a copy of the input RGB image and draws:

- A circle at each projected grasp center
- The grasp index label
- Optionally, a short orientation axis (when rotation matrices are provided)

This output is published as `annotated_image` when `publish_annotated_image` is enabled.
