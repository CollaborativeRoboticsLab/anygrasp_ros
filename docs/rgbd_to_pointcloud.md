# RGBD to PointCloud Node

## Overview

The `rgbd_to_pointcloud_node` is a ROS 2 preprocessing node that converts synchronized RGB and depth images into a semantically-aligned point cloud with color information. This node centralizes RGBD processing, making it independent of camera hardware and enabling the detection and tracking nodes to operate on standardized PointCloud2 messages.

**Key Responsibilities:**
- Synchronize RGB and depth image streams
- Query camera intrinsics (from topics or configuration)
- Apply depth-to-color frame alignment using ROS 2 TF transforms
- Generate colored point clouds with proper geometric alignment
- Handle fallback intrinsics for camera-agnostic operation

## Architecture

### Data Pipeline

```
RGB Image Topic          Depth Image Topic
    |                          |
    +----------┬───────────────+
               |
         [Message Filter]
      (ApproximateTimeSynchronizer)
               |
               v
    [Frame ID Detection]
    (from message headers)
               |
               v
    [TF Transform Query]
    (depth_optical_frame → color_optical_frame)
      (cached for stream lifetime)
               |
               v
    [Point Cloud Generation]
    - Build 3D points from depth intrinsics
    - Apply TF rotation + translation
    - Project to color frame
    - Sample RGB colors
               |
               v
        PointCloud2 Message
    (xyz + rgb per point, color_frame_id)
               |
               v
PointCloud Topic Output
```

### Point Cloud Generation Process

1. **Depth-to-3D Projection** (Depth Frame):
   - Convert depth image pixels to 3D points using depth camera intrinsics
   - Formula: `x = (u - cx) / fx * z`, `y = (v - cy) / fy * z`, `z = depth_value / depth_scale`
   - Result: N×3 array of 3D coordinates in depth frame

2. **Frame Alignment** (TF Transform):
   - Query TF tree for `depth_optical_frame` → `color_optical_frame` transform
   - Extract 3×3 rotation matrix from quaternion
   - Apply vectorized transformation: `points_color = R @ points_depth.T + translation`
   - Result: All points now in color frame coordinate system

3. **Color Projection** (Color Frame):
   - Project transformed 3D points into color image plane using color intrinsics
   - Formula: `u_c = fx_c * x / z + cx_c`, `v_c = fy_c * y / z + cy_c`
   - Mask invalid projections (outside image bounds or behind camera)
   - Result: 2D pixel coordinates in color image

4. **Color Sampling**:
   - Sample RGB values from color image at projected pixel coordinates
   - Handle missing/masked regions (set to 0)
   - Pack RGB as uint32: `(R << 16) | (G << 8) | B`
   - Result: Color value per point

5. **Output**:
   - Create PointCloud2 message with:
     - xyz fields (float32, meters)
     - rgb field (uint32, packed 0xRRGGBB)
     - frame_id: color_optical_frame (after alignment) or depth_optical_frame (if alignment disabled)
   - Publish to output topic

### Synchronization

- **Mechanism**: `message_filters.ApproximateTimeSynchronizer`
- **Time Tolerance**: 0.05 seconds (50 ms)
- **Behavior**: Matches RGB and depth messages that arrive within 50 ms of each other
- **Frame Rate**: Nominally 30 Hz (one point cloud per ~33 ms)

### TF-Based Frame Alignment

The node queries the ROS 2 TF transform tree to find the rigid body transformation between depth and color camera frames:

- **Transform**: `depth_optical_frame` → `color_optical_frame`
- **Query Timing**: Performed once on first image sync (lazy initialization)
- **Timeout**: 10 seconds with 0.1s polling interval
- **Caching**: Rotation matrix and translation vector cached for entire stream lifetime
- **Fallback**: If TF query fails, uses config-file fallback extrinsics (if `align_depth_to_color=true`)
- **Logging**: Prints rotation matrix (3×3) and translation vector to ROS logs for verification

**Advantages:**
- Camera-agnostic: Works with any ROS2-compatible camera (RealSense, Azure Kinect, etc.)
- Standard integration: Uses official ROS 2 TF system for frame transforms
- No topic coupling: Doesn't depend on camera-specific extrinsics publishers

## ROS Interface

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `rgb_image_topic` | `sensor_msgs/Image` | RGB image from camera (default: `/camera/color/image_raw`) |
| `depth_image_topic` | `sensor_msgs/Image` | Depth image from camera (default: `/camera/depth/image_rect_raw`) |
| `color_camera_info_topic_name` | `sensor_msgs/CameraInfo` | Color camera intrinsics (optional, if `use_color_camera_info_topic=true`) |
| `depth_camera_info_topic_name` | `sensor_msgs/CameraInfo` | Depth camera intrinsics (optional, if `use_depth_camera_info_topic=true`) |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `pointcloud_topic` | `sensor_msgs/PointCloud2` | Colored, aligned point cloud (default: `/pointcloud`) |

### TF Requirements

The node queries the following TF transform:
- **Source**: `depth_optical_frame` (from depth image header)
- **Target**: `color_optical_frame` (from RGB image header)

This transform must be available in the TF tree for alignment to work. If unavailable within 10 seconds, the node falls back to config-file extrinsics (if available) or disables alignment.

## Parameters

### Image Topics

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rgb_image_topic` | string | `/camera/color/image_raw` | Topic name for RGB images |
| `depth_image_topic` | string | `/camera/depth/image_rect_raw` | Topic name for depth images |
| `pointcloud_topic` | string | `/pointcloud` | Topic name for output point cloud |

### Camera Calibration Source

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_color_camera_info_topic` | bool | true | If true, read color intrinsics from topic; otherwise use `fx/fy/cx/cy` params |
| `color_camera_info_topic_name` | string | `/camera/color/camera_info` | Topic name for color camera info |
| `use_depth_camera_info_topic` | bool | true | If true, read depth intrinsics from topic; otherwise use `fx/fy/cx/cy` params |
| `depth_camera_info_topic_name` | string | `/camera/depth/camera_info` | Topic name for depth camera info |

### Fallback Camera Intrinsics

Used when camera info topics are unavailable or disabled. These are Intel RealSense D435 defaults:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `fx` | float | 927.17 | Color/depth focal length (x-axis, pixels) |
| `fy` | float | 927.37 | Color/depth focal length (y-axis, pixels) |
| `cx` | float | 651.32 | Color/depth principal point (x-axis, pixels) |
| `cy` | float | 349.62 | Color/depth principal point (y-axis, pixels) |

### Depth Processing

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `depth_scale` | float | 1000.0 | Conversion factor: depth_value (12-bit) → meters. D435: 1000.0 |
| `depth_max` | float | 1.5 | Maximum valid depth range (meters). Points beyond this distance are ignored |

### Alignment Control

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `align_depth_to_color` | bool | true | Enable TF-based depth-to-color frame alignment. If false, output point cloud in depth frame (faster, but colors may be misaligned) |

### Fallback Extrinsics (Advanced)

Used when TF query fails and `align_depth_to_color=true`. These are NOT typical parameters to modify:

| Parameter | Type | Fallback | Description |
|-----------|------|----------|-------------|
| `extrinsics_rotation_fallback` | float array | Identity matrix | 3×3 rotation matrix (row-major) |
| `extrinsics_translation_fallback` | float array | [0, 0, 0] | Translation vector (meters) |

**Note**: These should only be used for manual calibration override. TF transforms are the recommended method.

## Usage

### Basic Launch

```bash
ros2 launch anygrasp_ros rgbd.launch.py
```

### With Custom Camera

If using a non-RealSense camera, update `config/config.yaml`:

```yaml
rgbd_to_pointcloud_node:
  ros__parameters:
    rgb_image_topic: '/my_camera/rgb/image_raw'
    depth_image_topic: '/my_camera/depth/image_raw'
    color_camera_info_topic_name: '/my_camera/rgb/camera_info'
    depth_camera_info_topic_name: '/my_camera/depth/camera_info'
```

### With Intrinsics from File

If camera info topics are not available, provide intrinsics directly in config:

```yaml
rgbd_to_pointcloud_node:
  ros__parameters:
    use_color_camera_info_topic: false
    use_depth_camera_info_topic: false
    fx: 800.0  # Your camera focal length
    fy: 800.0
    cx: 320.0
    cy: 240.0
    depth_scale: 1000.0
    depth_max: 5.0
```

### Disable Alignment (Fast Mode)

If you only need unaligned depth clouds (faster processing):

```yaml
rgbd_to_pointcloud_node:
  ros__parameters:
    align_depth_to_color: false
```

## Output Format

### PointCloud2 Structure

- **Frame ID**: 
  - `color_optical_frame` (if alignment enabled)
  - `depth_optical_frame` (if alignment disabled)
- **Point Format**:
  - `x, y, z` (float32, 3 fields of 4 bytes each)
  - `rgb` (uint32, packed 0xRRGGBB)
- **Total Points**: Typically 30,000–600,000 depending on depth image resolution
- **Message Size**: ~0.5–2.4 MB per frame (depending on resolution)
- **Timestamp**: Synchronized RGB/depth acquisition timestamp

### Example PointCloud2 Inspection

```bash
# View point cloud in RViz2
rviz2 -d my_config.rviz

# Inspect point cloud statistics
ros2 topic echo /pointcloud --field count --no-array

# Record to bag for offline analysis
ros2 bag record /pointcloud
```

## Troubleshooting

### No Output on `/pointcloud`

1. **Check image streams**: Verify RGB/depth topics are publishing
   ```bash
   ros2 topic list | grep -E 'rgb|depth'
   ros2 topic hz /camera/color/image_raw
   ```

2. **Check synchronization**: Verify sync tolerance matches timestamp differences
   ```bash
   ros2 topic echo /camera/color/image_raw | grep stamp
   ros2 topic echo /camera/depth/image_raw | grep stamp
   # Timestamps should be within 50 ms
   ```

3. **Check node logs**: Look for TF query failures or intrinsics loading errors
   ```bash
   ros2 launch anygrasp_ros rgbd.launch.py 2>&1 | grep -i error
   ```

### Colors Not Aligned with Geometry

1. **Verify TF transform availability**:
   ```bash
   ros2 run tf2_tools view_frames
   # Check that depth_optical_frame → color_optical_frame path exists
   ```

2. **Check alignment is enabled**:
   ```bash
   ros2 param get /rgbd_to_pointcloud_node align_depth_to_color
   # Should return True
   ```

3. **Review TF query logs**:
   ```bash
   ros2 launch anygrasp_ros rgbd.launch.py 2>&1 | grep -i "rotation matrix\|translation"
   # Should see 3×3 matrix and [x, y, z] translation vector
   ```

### Poor Point Cloud Quality

1. **Check depth range**: Ensure `depth_max` is appropriate for your scene
2. **Check intrinsics**: Verify camera calibration is loaded correctly
   ```bash
   ros2 topic echo /camera/color/camera_info | head -20
   ```
3. **Check framerate**: High-frequency noise or dropped frames may indicate sync issues

## Integration with Detection/Tracking Nodes

The output PointCloud2 is designed for direct consumption by:

- **Detection Node** (`anygrasp_detection_node`): Subscribes to `/pointcloud`
- **Tracking Node** (`anygrasp_tracking_node`): Subscribes to `/pointcloud`

Both nodes operate on the aligned, colored point cloud without knowledge of the underlying camera hardware. To switch cameras, only update the RGBD node parameters in `config/config.yaml`.

## Performance

On an Intel i7 with 30 fps 1280×720 depth/RGB:
- Point cloud generation: ~10–15 ms per frame
- TF query (first frame only): ~5 ms
- Per-frame overhead (thereafter): ~2–3 ms
- Typical latency end-to-end: 50–80 ms (network + processing)

## References

- ROS 2 TF Documentation: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
- sensor_msgs/PointCloud2: https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html
- RealSense SDK (if using D435): https://www.intelrealsense.com/
