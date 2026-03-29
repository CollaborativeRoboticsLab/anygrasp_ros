
# Tracking node

The tracking node subscribes to synchronized RGB + depth images and maintains grasp ID correspondences across frames.

On the first successful service call, it selects initial grasp IDs from the current grasps using a configurable 3D selection box. Subsequent calls return tracked grasps.

## Inputs

- Subscribed topics (can be remapped):
	- `rgb_image` (`sensor_msgs/Image`)
	- `depth_image` (`sensor_msgs/Image`)

- Optional subscribed topics (only used if enabled via parameters):
	- Color `CameraInfo` (`sensor_msgs/CameraInfo`)
	- Depth `CameraInfo` (`sensor_msgs/CameraInfo`)

## Service

- Service name (default in launch): `/anygrasp/tracking`
- Service type: `anygrasp_msgs/srv/GetGrasps`

Request:

- `count`: number of tracked grasp poses requested

Response:

- `poses`: `geometry_msgs/Pose[]` (up to `count` items)
- `success`: boolean
- `message`: status string

## Parameters (high level)

- `checkpoint_path` (string): required path to the AnyGrasp tracking checkpoint.
- `filter` (string): tracker filter option (default `oneeuro`).
- Camera intrinsics:
	- Topic-based intrinsics (preferred when enabled):
		- `use_color_camera_info_topic` (bool)
		- `color_camera_info_topic_name` (string)
		- `use_depth_camera_info_topic` (bool)
		- `depth_camera_info_topic_name` (string)
	- Parameter fallback intrinsics (used when topic-based intrinsics are disabled): `fx`, `fy`, `cx`, `cy`
	- Depth conversion / filtering: `depth_scale`, `depth_max`

- Debug:
	- `publish_annotated_image` (bool): if true, publishes an image with the tracked grasps overlaid.

Initial grasp selection (used only when starting tracking):

- `select_x`: `[min, max]` meters
- `select_y`: `[min, max]` meters
- `select_z`: `[min, max]` meters
- `select_count`: how many grasps to seed for tracking

If no grasps fall inside the selection box, the service returns `success=false`.

Intrinsics selection order when projecting depth into 3D:

1) depth `CameraInfo` (if enabled + received)
2) color `CameraInfo` (if enabled + received)
3) `fx/fy/cx/cy` parameters

If a `use_*_camera_info_topic` flag is enabled but no `CameraInfo` message has been received yet, the service will return `success=false` until intrinsics arrive.

Note: `depth_scale` is always taken from the parameter (it’s not present in `CameraInfo`).

## Outputs

- Service response: tracked grasp poses (`geometry_msgs/Pose[]`)
- Optional debug topic:
	- `annotated_image` (`sensor_msgs/Image`): RGB image with projected grasp centers labeled by index. When available, a short orientation axis is drawn from each center.

Note: `publish_annotated_image` requires OpenCV (e.g. `python3-opencv`) at runtime.

## Example calls

Start / update tracking and request 1 pose:

```bash
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGrasps "{count: 1}"
```

Request 3 poses:

```bash
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGrasps "{count: 3}"
```

