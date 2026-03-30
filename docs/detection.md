
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

## Publications

- `marker_topic` (`visualization_msgs/MarkerArray`): RViz visualization of returned grasp poses (default: `/anygrasp/detection_markers`)

## Service Interface

- **Service name**: `/anygrasp/detection` (can be remapped via launch parameters)
- **Service type**: `anygrasp_msgs/srv/GetGrasps`

### Request

| Field | Type | Description |
|-------|------|-------------|
| `count` | int | Number of grasp poses to return |

### Response

| Field | Type | Description |
|-------|------|-------------|
| `poses` | geometry_msgs/Pose[] | Array of detected grasp poses (up to `count` items) |
| `success` | bool | True if detection succeeded |
| `message` | string | Status/error message |

**Frame Reference**: All poses are in the point cloud frame (frame_id from PointCloud2 message, typically `color_optical_frame` or `depth_optical_frame`).

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

### RViz Visualization

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_topic` | string | `/anygrasp/detection_markers` | Topic used to publish grasp markers for RViz |

**Note**: Point cloud intrinsics and alignment are handled by the [RGBD to PointCloud node](rgbd_to_pointcloud.md). This detection node consumes the pre-processed output directly.

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

### Status Codes

- `success=true, poses=[...]`: Detection succeeded with N grasps
- `success=false, poses=[]`: No point cloud received yet or detection failed
  - Check that RGBD node is running: `ros2 topic hz /pointcloud`
  - Check logs: `ros2 launch anygrasp_ros detection.launch.py 2>&1 | grep -i error`

### RViz

Add a `MarkerArray` display in RViz and subscribe it to `/anygrasp/detection_markers` to inspect grasp arrows and numeric IDs in 3D.

