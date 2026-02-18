
# Tracking node

The tracking node subscribes to synchronized RGB + depth images and maintains grasp ID correspondences across frames.

On the first successful service call, it selects initial grasp IDs from the current grasps using a configurable 3D selection box. Subsequent calls return tracked grasps.

## Inputs

- Subscribed topics (can be remapped):
	- `rgb_image` (`sensor_msgs/Image`)
	- `depth_image` (`sensor_msgs/Image`)

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
- Camera intrinsics: `fx`, `fy`, `cx`, `cy`, `depth_scale`, `depth_max`.

Initial grasp selection (used only when starting tracking):

- `select_x`: `[min, max]` meters
- `select_y`: `[min, max]` meters
- `select_z`: `[min, max]` meters
- `select_count`: how many grasps to seed for tracking

If no grasps fall inside the selection box, the service returns `success=false`.

## Example calls

Start / update tracking and request 1 pose:

```bash
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGrasps "{count: 1}"
```

Request 3 poses:

```bash
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGrasps "{count: 3}"
```

