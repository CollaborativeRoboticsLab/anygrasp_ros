
# Detection node

The detection node subscribes to synchronized RGB + depth images, and returns grasp pose(s) via a service call.

## Inputs

- Subscribed topics (can be remapped):
	- `rgb_image` (`sensor_msgs/Image`)
	- `depth_image` (`sensor_msgs/Image`)

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
- Camera intrinsics: `fx`, `fy`, `cx`, `cy`, `depth_scale`, `depth_max`.
- Workspace limits: `lims`.
- Grasp options: `max_gripper_width`, `gripper_height`, `top_down_grasp`, `apply_object_mask`, `dense_grasp`, `collision_detection`.

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

