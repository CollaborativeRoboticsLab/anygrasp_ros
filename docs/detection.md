
# Detection node

The detection node subscribes to synchronized RGB + depth images, and returns grasp pose(s) via a service call.

## Inputs

- Subscribed topics (can be remapped):
	- `rgb_image` (`sensor_msgs/Image`)
	- `depth_image` (`sensor_msgs/Image`)

- Optional subscribed topics (only used if enabled via parameters):
	- Color `CameraInfo` (`sensor_msgs/CameraInfo`)
	- Depth `CameraInfo` (`sensor_msgs/CameraInfo`)

The node caches the latest synchronized pair using an approximate time synchronizer.

## Service

- Service name (default in launch): `/anygrasp/detection`
- Service type: `anygrasp_msgs/srv/GetGrasps`

Request:

- `count`: number of grasp poses requested

Response:

- `poses`: `geometry_msgs/Pose[]` (up to `count` items)
- `success`: boolean
- `message`: status string

## Parameters (high level)

- `checkpoint_path` (string): required path to the AnyGrasp detection checkpoint.
- Camera intrinsics:
	- Topic-based intrinsics (preferred when enabled):
		- `use_color_camera_info_topic` (bool)
		- `color_camera_info_topic_name` (string)
		- `use_depth_camera_info_topic` (bool)
		- `depth_camera_info_topic_name` (string)
	- Parameter fallback intrinsics (used when topic-based intrinsics are disabled): `fx`, `fy`, `cx`, `cy`
	- Depth conversion / filtering: `depth_scale`, `depth_max`
- Workspace limits: `lims`.
- Grasp options: `max_gripper_width`, `gripper_height`, `top_down_grasp`, `apply_object_mask`, `dense_grasp`, `collision_detection`.

- Debug:
	- `publish_annotated_image` (bool): if true, publishes an image with the top grasps overlaid.

Intrinsics selection order when projecting depth into 3D:

1) depth `CameraInfo` (if enabled + received)
2) color `CameraInfo` (if enabled + received)
3) `fx/fy/cx/cy` parameters

If a `use_*_camera_info_topic` flag is enabled but no `CameraInfo` message has been received yet, the service will return `success=false` until intrinsics arrive.

Note: `depth_scale` is always taken from the parameter (it’s not present in `CameraInfo`).

## Outputs

- Service response: grasp poses (`geometry_msgs/Pose[]`)
- Optional debug topic:
	- `annotated_image` (`sensor_msgs/Image`): RGB image with projected grasp centers labeled by index.

## Example calls

Request 1 pose:

```bash
ros2 service call /anygrasp/detection anygrasp_msgs/srv/GetGrasps "{count: 1}"
```

Request 5 poses:

```bash
ros2 service call /anygrasp/detection anygrasp_msgs/srv/GetGrasps "{count: 5}"
```

If no synchronized frames have arrived yet, `success` will be `false` and `poses` will be empty.

