# Connecting to external docker containers

## The Port Math for Domain 76

DDS determines its port numbers using a standardized formula based on the Domain ID. The base port is $PB = 7400$, and the domain gain is $DG = 250$.The base port for Domain 76 is calculated as:$$Port_{base} = 7400 + (250 \times 76) = 26400$$Individual nodes (participants) inside your container will use offsets from this base port (e.g., +10, +11, +12, etc.) for unicast discovery and user data. To safely accommodate multiple ROS 2 nodes running inside your container, you should expose a range of about 20 ports.You will need to open UDP ports 26400 through 26420.

## Updating this container (Container that use network=bridge)

Create a copy of the `cyclonedds.xml` and update to match your settings

```bash
cp /home/ubuntu/colcon_ws/src/grasping/docs/external/cyclonedds.xml /home/ubuntu/colcon_ws/src/grasping/cyclonedds.xml
```

Modify the `cyclonedds.xml` with the following information,

```
DEVICE_1_PHYSICAL_IP - set the ip address of the external device
DEVICE_2_PHYSICAL_IP - set the ip address of this device
```

Update the `devcontainer.json` with following values,

```json
{
  "containerEnv": {
    "CYCLONEDDS_URI": "/home/ubuntu/colcon_ws/src/grasping/cyclonedds.xml",
    "ROS_DOMAIN_ID": "76"
  },
  "runArgs": [
    "--privileged",
    "--network=bridge",
    "-p",
    "26400-26420:26400-26420/udp",
  ],
}
```

first values opens up the udp ports, second values sets the ROS_DOMAIN_ID and the last values sents the new config file for cyclonedds.

Save and restart the devcontainer.

## Updating the external container

Create a `cyclonedds.xml` file on the devcontainer

```bash
touch cyclonedds.xml
```

Copy the content of the `external.xml` into the devcontainer of the external device.

Modify the new `cyclonedds.xml` with the following information,

```
DEVICE_2_PHYSICAL_IP - set the ip address of this device (not the externl device) so that it can find this device.
```

once the file is updated, run the following commands in the terminal

```bash
export ROS_DOMAIN_ID=76
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

Next restart the ros2 daemon

```bash
ros2 daemon stop
ros2 daemon start
```

And then run the ros nodes in the terminal. when opening multiple terminals, run these export commands in each.

## Test

Test on this device with `ros2 topic list` and see if the topics are visible