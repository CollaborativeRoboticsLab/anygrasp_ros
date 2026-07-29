# Realsense Camera

## Installation

We have configured the devcontainer to install the drivers required during the build process. If you are setting this up yourself (outside the devcontainer) and using a Realsense camera, follow the official instructions here: https://github.com/realsenseai/realsense-ros#installation-on-ubuntu

## Configuration and Customization

We are using a `D435` camera in our system and the configurations we provide in [d435.launch.py](./../../grasping_camera/launch/d435.launch.py) match that.

To introduce a different Realsense camera model, create a separate launch file and change the parameters accordingly.

### Important configuration

The launch file is a thin wrapper around the upstream `realsense2_camera` node, exposed as launch arguments.

Below are the most common parameters you’ll want to tune for grasping:

- **Device selection**
	- `serial_no`: Select a specific camera by serial number (recommended if you may have multiple cameras connected).
	- `usb_port_id`: Select by physical USB port.
	- `device_type`: Select by model (e.g. D435). Useful when you only ever have one connected.

- **Stream enable/disable**
	- `enable_color` (default `true`)
	- `enable_depth` (default `true`)
	- `enable_infra`, `enable_infra1`, `enable_infra2` (defaults `false`)
	- `enable_gyro`, `enable_accel`, `enable_motion` (IMU; defaults `false`)

- **Stream profiles (resolution, fps)**
	- `rgb_camera.color_profile`: e.g. `640,480,30`
	- `depth_module.depth_profile`: e.g. `640,480,30`

- **Depth alignment + point cloud**
	- `align_depth.enable`: Align depth to the color optical frame. Commonly enabled for RGB-D grasping.
	- `pointcloud.enable`: Enable point cloud publishing.
	- `pointcloud.stream_filter`: Which stream to texture the point cloud with (see launch arg description; common is color).

- **TF / frames**
	- `publish_tf`: Enable publishing TF.
	- `base_frame_id`: Root frame id for the camera TF tree (default `link`).
	- `tf_prefix`: Optional prefix prepended to all frame ids (useful with multiple cameras).

- **Filters (often useful in noisy scenes)**
	- `decimation_filter.enable`
	- `spatial_filter.enable`
	- `temporal_filter.enable`
	- `hole_filling_filter.enable`

## Starting the camera

Use the following command to start the camera

```bash
source install/setup.bash
ros2 launch grasping_camera d435.launch.py
```

## Troubleshooting

- **No device found / device disconnects**
	- Try re-plugging the camera and use a USB 3 port/cable.
	- If you are running inside a container, confirm the device is passed through.

- **Permissions issues (Linux)**
	- If you see permission errors accessing the USB device, install the upstream udev rules from the RealSense ROS installation instructions.

- **Topics / frames don’t match expectations**
	- By default, the node is launched with an empty `camera_namespace`, so topics are not automatically nested under `/camera/...` unless you set that namespace yourself.
	- If you use multiple cameras, set `tf_prefix` and/or change `camera_namespace` to avoid collisions.