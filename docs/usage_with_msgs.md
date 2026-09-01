
# Using `anygrasp_msgs` Services

Detection and tracking expose different ROS 2 service types:

- Detection: `anygrasp_msgs/srv/GetGrasps`
- Filtered detection: `anygrasp_msgs/srv/GetFilteredGrasps`
- Tracking: `anygrasp_msgs/srv/GetGraspsTracked`

Detection service definition:

```
int64 count
---
geometry_msgs/PoseStamped[] poses
bool success
string message
```

Tracking service definition:

```
int64 count
int64[] input_ids
---
int64[] ids
geometry_msgs/PoseStamped[] poses
bool success
string message
```

Filtered detection service definition:

```
int64 count
int64 detection_id
int16 class_id
string class_name
int16 bbx_center_x
int16 bbx_center_y
int16 bbx_size_w
int16 bbx_size_h
int32 image_width
int32 image_height
---
geometry_msgs/PoseStamped[] poses
bool success
string message
```

## What you send

- `count`: how many grasp poses you want returned.
	- `count <= 0` is treated as “return 1”.
- For filtered detection, pass the selected 2D detection metadata and the bbox in pixel coordinates together with the source image size.

## What you get back

- Detection returns `poses`: list of `geometry_msgs/PoseStamped` for the best grasp(s).
- Filtered detection returns `poses`: list of `geometry_msgs/PoseStamped` for grasps whose centers reproject into the selected bbox.
- Tracking returns `ids` plus `poses`, where each ID matches the stamped pose at the same array index.
- Each returned pose header is copied from the source `PointCloud2` message, so `header.frame_id` matches the pointcloud frame.
- For tracking requests, set `input_ids` to `[]` to auto-seed/update the active set, or provide one or more tracked IDs to update only those grasps.
- `success`: `true` if grasps were produced.
- `message`: short human-readable status.

## Quick CLI examples

Detection (returns top-N detected grasps):

```bash
ros2 service call /anygrasp/detection anygrasp_msgs/srv/GetGrasps "{count: 1}"
ros2 service call /anygrasp/detection anygrasp_msgs/srv/GetGrasps "{count: 5}"
```

Filtered detection (returns grasps for a selected 2D object bbox):

```bash
ros2 service call /anygrasp/detection/filtered anygrasp_msgs/srv/GetFilteredGrasps "{count: 2, detection_id: 7, class_id: 39, class_name: 'bottle', bbx_center_x: 320, bbx_center_y: 240, bbx_size_w: 120, bbx_size_h: 200, image_width: 640, image_height: 480}"
```

Tracking (returns top-N tracked grasps):

```bash
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGraspsTracked "{count: 3, input_ids: []}"
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGraspsTracked "{count: 2, input_ids: [2, 5]}"
```

Tip: if you don’t know the exact service name in your setup, list services and types:

```bash
ros2 service list
ros2 service type /anygrasp/detection
ros2 service type /anygrasp/detection/filtered
ros2 service type /anygrasp/tracking
```

