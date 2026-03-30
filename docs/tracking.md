
# Tracking Node

## Overview

The `anygrasp_tracking_node` performs inference on a PointCloud2 and tracks grasp identities across frames. It consumes the pre-processed point cloud from the [RGBD to PointCloud](rgbd_to_pointcloud.md) node and runs AnyGrasp tracking on each PointCloud2 frame.

## Pipeline

- **Input**: PointCloud2 (RGB-colored, 3D points with xyz + rgb fields)
- **First Call**: Selects initial grasp IDs from current frame using a 3D workspace selection box
- **Subsequent Calls**: Returns tracked grasp IDs from ongoing tracking
- **Output**: Tracked grasp poses via ROS 2 service

## Inputs

- Subscribed topics (can be remapped):
  - `input_pointcloud` (`sensor_msgs/PointCloud2`): Pre-aligned, RGB-colored point cloud from the [RGBD preprocessing node](rgbd_to_pointcloud.md) (default: `/pointcloud`)

## Publications

- `marker_topic` (`visualization_msgs/MarkerArray`): RViz visualization of tracked grasp poses (default: `/anygrasp/tracking_markers`)

## Service Interface

- **Service name**: `/anygrasp/tracking` (can be remapped via launch parameters)
- **Service type**: `anygrasp_msgs/srv/GetGraspsTracked`

### Request

| Field | Type | Description |
|-------|------|-------------|
| `count` | int | Number of tracked grasp poses to return |
| `input_ids` | int64[] | Specific tracked grasp IDs to update, or an empty list to auto-seed / track the current active set |

### Response

| Field | Type | Description |
|-------|------|-------------|
| `ids` | int64[] | Stable tracked grasp IDs aligned one-to-one with `poses` |
| `poses` | geometry_msgs/Pose[] | Array of tracked grasp poses (up to `count` items) |
| `success` | bool | True if tracking succeeded |
| `message` | string | Status/error message |

**Frame Reference**: All poses are in the point cloud frame (frame_id from PointCloud2 message, typically `color_optical_frame` or `depth_optical_frame`).

## Parameters

### AnyGrasp Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `checkpoint_path` | string | (required) | Path to AnyGrasp tracking model checkpoint |
| `filter` | string | `oneeuro` | Tracking filter type (options: `oneeuro`, `kalman`) |

### Initial Grasp Selection (First Call)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `select_x` | float array | [-0.18, 0.18] | X-axis workspace range for initial grasp selection (meters) |
| `select_y` | float array | [-0.12, 0.12] | Y-axis workspace range for initial grasp selection (meters) |
| `select_z` | float array | [0.35, 0.55] | Z-axis workspace range for initial grasp selection (meters) |
| `select_count` | int | 5 | Number of initial grasps to seed for tracking |

**Note**: If no grasps fall within the selection box on first call, the service returns `success=false`.

### Point Cloud Input

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_pointcloud` | string | `/pointcloud` | Topic name for colored point cloud input (from [RGBD node](rgbd_to_pointcloud.md)) |

### RViz Visualization

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_topic` | string | `/anygrasp/tracking_markers` | Topic used to publish tracked grasp markers for RViz |

**Note**: Point cloud intrinsics and alignment are handled by the [RGBD to PointCloud node](rgbd_to_pointcloud.md). This tracking node consumes the pre-processed output directly.

## Usage

### Launch

```bash
ros2 launch anygrasp_ros tracking.launch.py
```

### Service Calls

Initialize tracking (first call selects grasps from workspace box):

```bash
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGraspsTracked "{count: 3, input_ids: []}"
```

Track specific previously returned IDs in subsequent frames:

```bash
ros2 service call /anygrasp/tracking anygrasp_msgs/srv/GetGraspsTracked "{count: 2, input_ids: [2, 5]}"
```

### Status Codes

- `success=true, ids=[...], poses=[...]`: Tracking succeeded with N grasps and stable IDs
- `success=false, poses=[]`: No point cloud received or no grasps in selection box (first call only)
  - Check that RGBD node is running: `ros2 topic hz /pointcloud`
  - Check logs: `ros2 launch anygrasp_ros tracking.launch.py 2>&1 | grep -i error`

### Request Semantics

- `input_ids = []`: if no active tracked IDs exist, the node seeds them from the current frame; otherwise it updates the current tracked set.
- `input_ids = [id1, id2, ...]`: the node updates only those tracked grasp IDs and returns poses in the same order as the surviving IDs.

### RViz

Add a `MarkerArray` display in RViz and subscribe it to `/anygrasp/tracking_markers` to inspect tracked grasp arrows and numeric IDs in 3D.

