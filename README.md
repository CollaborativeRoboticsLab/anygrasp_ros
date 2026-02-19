# anygrasp_ros

ROS 2 wrappers for AnyGrasp detection and tracking.

### Creating docker network

Create the docker [mavclan](https://docs.docker.com/engine/network/drivers/macvlan/) network following [these instructions](./docs/macvlan.md). This limits the host to Linux OS (Windows, WSL and MacOS not supported) but AnyGrasp Licensing requires this apprach when using a docker based implementation.

### Building container

Install VSCode and add the [DevContainer addon](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Clone this repo and open using VSCode. Generally VScode should auto detect, if not press Shift+Ctrl+P to open the command palette and select "DevContainer: Rebuild and Reopen the container" option.

### Adding license

Once the Container is built, run the `license_checker` function from anygrasp_sdk and apply for the license following the steps from [here](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md). 

Following commands will help to run the `license_checker` within the dev container.

```bash
cd /dependencies/anygrasp_sdk/license_registration/
./license_checker -f
```

Once you fill the form and receive the license zip file, unzip and copy it to the `/license` folder within the cloned repo (Not inside the container). Devcontainer has been configured to mount the license folder into the following locations of the container,

- `/home/ubuntu/colcon_ws/license`

To check the license run following command

```bash
cd /dependencies/anygrasp_sdk/license_registration/
./license_checker -c /dependencies/precompiled/license/licenseCfg.json
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

Both nodes expose services using `anygrasp_msgs/srv/GetGrasps`:

- `/anygrasp/detection` (detection)
- `/anygrasp/tracking` (tracking)

Each service takes a `count` in the request and returns `geometry_msgs/Pose[]` in the response.

## More information

- [Service Messages](./docs/usage_with_msgs.md)
- [Grasp Detection](./docs/detection.md)
- [Grasp Tracking](./docs/tracking.md)
