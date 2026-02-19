# Creating the docker network

This devcontainer is configured to attach to a [**macvlan**](https://docs.docker.com/engine/network/drivers/macvlan/) network.

You must create the macvlan network on the **host** (outside the container) before rebuilding the devcontainer.

1) Identify the host NIC that is connected to your robot/LAN (examples: `eth0`, `enp3s0`):

```bash
ip route | grep default
```

2) Create the Docker macvlan network (replace values to match your LAN):

```bash
docker network create -d macvlan \
	--subnet=10.0.0.0/24 \
	--gateway=10.0.0.1 \
	-o parent=enp2s0 \
	anygrasp_macvlan
```

Notes:

- `parent` must be a real host interface on the target subnet.
- Choose an `--ip-range` that is **unused** on your network (or omit `--ip-range` to let Docker allocate from the subnet).
- macvlan makes the container a “real” LAN participant, which is often helpful for ROS 2 discovery and talking to robot hardware.

Cleanup:

```bash
docker network rm anygrasp_macvlan
```