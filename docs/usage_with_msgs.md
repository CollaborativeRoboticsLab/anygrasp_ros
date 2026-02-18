
# Using `anygrasp_msgs/GetGrasps`

Both nodes now expose ROS 2 services using `anygrasp_msgs/srv/GetGrasps` (instead of `std_srvs/Trigger`).

Service definition:

```
int64 count
---
geometry_msgs/Pose[] poses
bool success
string message
```

## What you send

- `count`: how many grasp poses you want returned.
	- `count <= 0` is treated as “return 1”.

## What you get back

- `poses`: list of `geometry_msgs/Pose` for the best grasp(s).
- `success`: `true` if grasps were produced.
- `message`: short human-readable status.

## Quick CLI examples

Detection (returns top-N detected grasps):

```bash
ros2 service call /anygrasp/detection anygrasp_msgs/srv/GetGrasps "{count: 1}"
ros2 service call /anygrasp/detection anygrasp_msgs/srv/GetGrasps "{count: 5}"
```

Tracking (returns top-N tracked grasps):

```bash
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGrasps "{count: 1}"
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGrasps "{count: 3}"
```

Tip: if you don’t know the exact service name in your setup, list services and types:

```bash
ros2 service list
ros2 service type /anygrasp/detection
ros2 service type /anygrasp/tracking
```

