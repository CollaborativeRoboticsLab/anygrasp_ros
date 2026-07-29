# anygrasp_ros

ROS 2 wrappers for AnyGrasp detection and tracking.

The devcontainer is based on [nvidia/cuda:12.6.0-cudnn-devel-ubuntu22.04](https://hub.docker.com/layers/nvidia/cuda/12.6.0-cudnn-devel-ubuntu22.04/images/sha256-3814ef2c9d46ca559e601374029a576596f016e33ddf48d6e2ad778d21bfa3f0) image and provides

- Pytorch 2.10
- CUDA 12.6
- CUDNN9
- ROS Humble (Base container is ubuntu 22.04)
- [chenxi-wang/MinkowskiEngine](https://github.com/chenxi-wang/MinkowskiEngine.git)
- [CollaborativeRoboticsLab/graspnetAPI](https://github.com/CollaborativeRoboticsLab/graspnetAPI.git)
- [graspnet/anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk.git)
- [Realsense packages](https://github.com/realsenseai/realsense-ros)

## Creating docker network

To have a stable feature id for the anygrasp license, we utilize built-in docker network `bridge` and a fixed mac address. For the dev container, this is represented by following config. Change the given mac address as required.

```json
  "runArgs": [
    "--network=bridge",
    "--mac-address=02:42:de:ad:be:ef"
  ]
```

## Building container

Install VSCode and add the [DevContainer addon](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Clone this repo and open using VSCode. Generally VScode should auto detect, if not press Shift+Ctrl+P to open the command palette and select "DevContainer: Rebuild and Reopen the container" option.

## Connecting to external ROS2 nodes

Due to the usage of `network=bridge`, the devcontainer will not be able to communicate with other ROS2 nodes running on the host machine, other remote machines or other containers with default setting.

We need to update the ROS2 DDS network configuration for all entities to make the anygrasp_ros2 container reachable. [Follow the instructions](./docs/external/dds_configuration.md) to setup the network configuration.

## Adding license

Once the Container is built, run the following command to get the `feature id` and apply for the license [following the steps](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md).

```bash
python -c "from gsnet import get_feature_id; print(get_feature_id())"
```

Once you fill the form and receive the license zip file, unzip and copy it to the `/license` folder within the cloned repo (Not inside the container). Devcontainer has been configured to mount the license folder into the following locations of the container,

- `/dependencies/precompiled/license`

To check the license run following command

```bash
python -c "from gsnet import check_license; check_license('license')"
```

## Adding model weights

Copy the detection and tracking model weights into `weights/detection` and `weights/tracking` folders respectively. These will be mounted into following folders inside the container.

- `/dependencies/precompiled/weights/detection`             allows to run the ros2 packages
- `/dependencies/precompiled/weights/tracking`              allows to run the ros2 packages

This can also be done alongside the prior `Adding License` step.

## Basic testing

Try running the `grasp_detection/demo.py` and `grasp_tracking/demo.py` to confirm the process pipeline is working.

### Start the camera node

Use the following command to start the camera node (realsense D435).

```bash
ros2 launch realsense2_camera rs_launch.py
```

### Starting the anygrasp detection system

Use the following command to start the anygrasp detection system

```bash
ros2 launch anygrasp_ros detection.launch.py
```

### Starting the anygrasp tracking system

Use the following command to start the anygrasp tracking system

```bash
ros2 launch anygrasp_ros tracking.launch.py
```

## Services

The nodes expose these services:

- `/anygrasp/detection` using `anygrasp_msgs/srv/GetGrasps`
- `/anygrasp/tracking` using `anygrasp_msgs/srv/GetGraspsTracked`

Usage:

- Each service takes a `count` in the request.
- Detection returns `geometry_msgs/PoseStamped[]`  
- Tracking returns `int64[] ids` aligned with `geometry_msgs/PoseStamped[]`, and accepts `input_ids` as a list to select specific tracked grasps or `[]` to update the active set.
- Each stamped pose copies the source pointcloud header, so the frame is explicit for downstream motion planning.

## Visualization

Both nodes publish RViz grasp markers as `visualization_msgs/MarkerArray`:

- Detection markers: `/anygrasp/detection_markers`
- Tracking markers: `/anygrasp/tracking_markers`

Add either topic as a `MarkerArray` display in RViz to inspect grasp poses and IDs in 3D.

## More information

- [External container using network=host](./docs/external/dds_configuration.md)
- [Service Messages](./docs/usage_with_msgs.md)
- [Grasp Detection](./docs/detection.md)
- [Grasp Tracking](./docs/tracking.md)
