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

Once you fill the form and receive the license zip file, unzip and copy it to the `/license` folder within the cloned repo (Not inside the container). Then rebuild the container. This will automatically copy the license content into the following locations of the container,

- `/dependencies/anygrasp_sdk/license_registration/license`  allows to run license checker
- `/dependencies/anygrasp_sdk/grasp_detection/license`       allows to run the grasp detection
- `/dependencies/anygrasp_sdk/grasp_tracking/license`        allows to run the grasp tracking
- `/home/ubuntu/colcon_ws/license`                           allows to run the ros2 packages

To check the license run following command

```bash
cd /dependencies/anygrasp_sdk/license_registration/
./license_checker -c license/licenseCfg.json
```

### Adding model weights

Copy the detection and tracking model weights into `weights/detection` and `weights/tracking` folders respectively and Rebuild the container. These will be loaded into following folders inside the container. 

- `/dependencies/anygrasp_sdk/grasp_detection/log`       allows to run the grasp detection
- `/dependencies/anygrasp_sdk/grasp_tracking/log`        allows to run the grasp tracking
- `/home/ubuntu/colcon_ws/weights/detection`             allows to run the ros2 packages
- `/home/ubuntu/colcon_ws/weights/tracking`              allows to run the ros2 packages

This can also be done alongside the prior `Adding Licesne` step.

### Basic testing

Try running the `grasp_detection/demo.py` and `grasp_tracking/demo.py` to confirm the process pipeline is working

## Services

Both nodes expose services using `anygrasp_msgs/srv/GetGrasps`:

- `/anygrasp/detection` (detection)
- `/anygrasp/tracking` (tracking)

Each service takes a `count` in the request and returns `geometry_msgs/Pose[]` in the response.

## More information

- [Service Messages](./docs/usage_with_msgs.md)
- [Grasp Detection](./docs/detection.md)
- [Grasp Tracking](./docs/tracking.md)
