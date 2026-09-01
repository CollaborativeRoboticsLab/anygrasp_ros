
# Detection Node

## Overview

The `anygrasp_detection_node` performs inference on a PointCloud2 to detect grasp poses. It consumes the pre-processed point cloud from the [RGBD to PointCloud](rgbd_to_pointcloud.md) node and runs AnyGrasp detection on each PointCloud2 frame.

## Pipeline

- **Input**: PointCloud2 (RGB-colored, 3D points with xyz + rgb fields)
- **Processing**: AnyGrasp detection inference
- **Output**: Grasp poses via ROS 2 service

The node maintains a cache of the latest point cloud for service requests.

## Inputs

- Subscribed topics (can be remapped):
  - `input_pointcloud` (`sensor_msgs/PointCloud2`): Pre-aligned, RGB-colored point cloud from the [RGBD preprocessing node](rgbd_to_pointcloud.md) (default: `/pointcloud`)

## Outputs

- `marker_topic` (`visualization_msgs/MarkerArray`): RViz visualization of returned grasp poses (default: `/anygrasp/detection_markers`)

## Service Interface

- **Service name**: `/anygrasp/detection` (can be remapped via launch parameters)
- **Service type**: `anygrasp_msgs/srv/GetGrasps`

### Filtered Service Interface

- **Service name**: `/anygrasp/detection/filtered`
- **Service type**: `anygrasp_msgs/srv/GetFilteredGrasps`

### Request

| Field | Type | Description |
|-------|------|-------------|
| `count` | int | Number of grasp poses to return |

### Response

| Field | Type | Description |
|-------|------|-------------|
| `poses` | geometry_msgs/PoseStamped[] | Array of detected grasp poses (up to `count` items), stamped with the source point cloud header |
| `success` | bool | True if detection succeeded |
| `message` | string | Status/error message |

**Frame Reference**: Each returned pose carries the point cloud frame in `poses[i].header.frame_id` and the point cloud timestamp in `poses[i].header.stamp`.

### Filtered Request

| Field | Type | Description |
|-------|------|-------------|
| `count` | int | Number of grasp poses to return |
| `detection_id` | int | Selected 2D detection identifier, or `-1` when unavailable |
| `class_id` | int | Selected class identifier, or `-1` when unavailable |
| `class_name` | string | Selected class label |
| `bbx_center_x` | int | Bounding-box center x coordinate in pixels |
| `bbx_center_y` | int | Bounding-box center y coordinate in pixels |
| `bbx_size_w` | int | Bounding-box width in pixels |
| `bbx_size_h` | int | Bounding-box height in pixels |
| `image_width` | int | Width of the source image used to pick the bbox |
| `image_height` | int | Height of the source image used to pick the bbox |

### Filtered Response

| Field | Type | Description |
|-------|------|-------------|
| `poses` | geometry_msgs/PoseStamped[] | Array of grasp poses whose centers remain aligned with the selected bbox |
| `success` | bool | True if filtered detection succeeded |
| `message` | string | Status/error message |

## Parameters

### AnyGrasp Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `checkpoint_path` | string | (required) | Path to AnyGrasp detection model checkpoint |
| `max_gripper_width` | float | 0.10 | Maximum gripper opening width (meters) |
| `gripper_height` | float | 0.03 | Gripper height (meters) |
| `top_down_grasp` | bool | false | Restrict grasps to top-down approach |
| `apply_object_mask` | bool | true | Use object segmentation to filter grasps |
| `dense_grasp` | bool | false | Enable dense grasp prediction |
| `collision_detection` | bool | true | Check grasps for collisions |

### Workspace Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lims` | float array | [-0.19, 0.12, 0.02, 0.15, 0.0, 1.0] | Workspace limits [x_min, x_max, y_min, y_max, z_min, z_max] (meters) |

### Point Cloud Input

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_pointcloud` | string | `/pointcloud` | Topic name for colored point cloud input (from [RGBD node](rgbd_to_pointcloud.md)) |

### Filtered Detection Projection

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_color_camera_info_topic` | bool | true | Use the color `CameraInfo` topic for grasp reprojection into image space |
| `color_camera_info_topic_name` | string | `/camera/color/camera_info` | Color `CameraInfo` topic used for bbox reprojection |
| `fx` | float | 927.17 | Fallback focal length x when no `CameraInfo` has been received |
| `fy` | float | 927.37 | Fallback focal length y when no `CameraInfo` has been received |
| `cx` | float | 651.32 | Fallback principal point x when no `CameraInfo` has been received |
| `cy` | float | 349.62 | Fallback principal point y when no `CameraInfo` has been received |

### RViz Visualisation

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_topic` | string | `/anygrasp/detection_markers` | Topic used to publish grasp markers for RViz |

**Note**: Point cloud intrinsics and alignment are handled by the [RGBD to PointCloud node](rgbd_to_pointcloud.md). The filtered detection service additionally reprojects points and candidate grasps into the color image plane using the color-camera intrinsics so it can constrain results to a selected 2D bbox.

## Usage

### Launch

```bash
ros2 launch anygrasp_ros detection.launch.py
```

### Service Calls

Request 1 grasp pose:

```bash
ros2 service call /anygrasp/detection anygrasp_msgs/srv/GetGrasps "{count: 1}"
```

Request 5 grasp poses:

```bash
ros2 service call /anygrasp/detection anygrasp_msgs/srv/GetGrasps "{count: 5}"
```

Request filtered grasps for a selected bbox:

```bash
ros2 service call /anygrasp/detection/filtered anygrasp_msgs/srv/GetFilteredGrasps "{count: 2, detection_id: 7, class_id: 39, class_name: 'bottle', bbx_center_x: 320, bbx_center_y: 240, bbx_size_w: 120, bbx_size_h: 200, image_width: 640, image_height: 480}"
```

### Status Codes

- `success=true, poses=[...]`: Detection succeeded with N grasps
- `success=false, poses=[]`: No point cloud received yet or detection failed
  - Check that RGBD node is running: `ros2 topic hz /pointcloud`
  - Check logs: `ros2 launch anygrasp_ros detection.launch.py 2>&1 | grep -i error`

### RViz

Add a `MarkerArray` display in RViz and subscribe it to `/anygrasp/detection_markers` to inspect grasp arrows and numeric IDs in 3D.

