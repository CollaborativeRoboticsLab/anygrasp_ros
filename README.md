# anygrasp_ros

ROS 2 wrappers for AnyGrasp detection and tracking.

## Services

Both nodes expose services using `anygrasp_msgs/srv/GetGrasps`:

- `/anygrasp/detection` (detection)
- `/anygrasp/tracking` (tracking)

Each service takes a `count` in the request and returns `geometry_msgs/Pose[]` in the response.

## More information

- [Service Messages](./docs/usage_with_msgs.md)
- [Grasp Detection](./docs/detection.md)
- [Grasp Tracking](./docs/tracking.md)
