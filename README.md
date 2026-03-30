# anygrasp_ros

ROS 2 wrappers for AnyGrasp detection and tracking.

The devcontainer is based on [pytorch/pytorch:2.10.0-cuda12.6-cudnn9-devel](https://hub.docker.com/layers/pytorch/pytorch/2.10.0-cuda12.6-cudnn9-devel/images/sha256-df80e10d07cd114c5f33380e3df7b6c5a3caab8481f68509ea652a7c0908316e) image and provides
- Pytorch 2.10
- CUDA 12.6
- CUDNN9
- ROS Jazzy (Base container is ubuntu 24.04)
- [chenxi-wang/MinkowskiEngine](https://github.com/chenxi-wang/MinkowskiEngine.git)
- [CollaborativeRoboticsLab/graspnetAPI](https://github.com/CollaborativeRoboticsLab/graspnetAPI.git)
- [graspnet/anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk.git)

### Creating docker network

To have a stable feature id for the anygrasp license, we utilize built-in docker network `bridge` and a fixed mac address. For the dev container, this is represented by following config. Change the given mac address as required.

```json
  "runArgs": [
    "--network=bridge",
    "--mac-address=02:42:de:ad:be:ef"
  ]
```

### Building container

Install VSCode and add the [DevContainer addon](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Clone this repo and open using VSCode. Generally VScode should auto detect, if not press Shift+Ctrl+P to open the command palette and select "DevContainer: Rebuild and Reopen the container" option.

### Adding license

Once the Container is built, run the `license_checker` function from anygrasp_sdk and apply for the license following the steps from [here](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md). 

Following commands will help to run the `license_checker` within the dev container.

```bash
/dependencies/anygrasp_sdk/license_registration/license_checker -f
```

Once you fill the form and receive the license zip file, unzip and copy it to the `/license` folder within the cloned repo (Not inside the container). Devcontainer has been configured to mount the license folder into the following locations of the container,

- `/dependencies/precompiled/license`

To check the license run following command

```bash
/dependencies/anygrasp_sdk/license_registration/license_checker -c /dependencies/precompiled/license/licenseCfg.json
```

### Adding model weights

Copy the detection and tracking model weights into `weights/detection` and `weights/tracking` folders respectively. These will be mounted into following folders inside the container. 

- `/dependencies/precompiled/weights/detection`             allows to run the ros2 packages
- `/dependencies/precompiled/weights/tracking`              allows to run the ros2 packages

This can also be done alongside the prior `Adding Licesne` step.

### Basic testing

Try running the `grasp_detection/demo.py` and `grasp_tracking/demo.py` to confirm the process pipeline is working.

### Starting the anygrasp detection system

Use the following command to start the anygrasp system

```bash
ros2 launch anygrasp_ros detection.launch.py
```

### Starting the anygrasp tracking system

Use the following command to start the anygrasp system

```bash
ros2 launch anygrasp_ros tracking.launch.py
```

## Services

The nodes expose these services:

- `/anygrasp/detection` using `anygrasp_msgs/srv/GetGrasps`
- `/anygrasp/tracking` using `anygrasp_msgs/srv/GetGraspsTracked`

Each service takes a `count` in the request. Detection returns `geometry_msgs/Pose[]`; tracking returns `int64[] ids` aligned with `geometry_msgs/Pose[]`, and accepts `input_ids` as a list to select specific tracked grasps or `[]` to update the active set.

## Visualization

Both nodes publish RViz grasp markers as `visualization_msgs/MarkerArray`:

- Detection markers: `/anygrasp/detection_markers`
- Tracking markers: `/anygrasp/tracking_markers`

Add either topic as a `MarkerArray` display in RViz to inspect grasp poses and IDs in 3D.

## More information

- [Service Messages](./docs/usage_with_msgs.md)
- [Grasp Detection](./docs/detection.md)
- [Grasp Tracking](./docs/tracking.md)
