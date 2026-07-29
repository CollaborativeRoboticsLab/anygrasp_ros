# Connecting to external docker containers

This repo supports running the AnyGrasp ROS 2 nodes inside a devcontainer while connecting to other ROS 2 nodes running either,

- (a) in other devcontainers on the same machine
- (b) in devcontainers on a different machine.

The key is to:

- Use the same `ROS_DOMAIN_ID` everywhere.
- Ensure CycloneDDS advertises a reachable IP (`ExternalNetworkAddress`) when Docker/NAT is involved.
- Ensure UDP ports for the domain are reachable (publish in Docker, allow in firewalls).

## The Port Math for Domain 76

DDS determines its port numbers using a standardized formula based on the Domain ID.

- Base port: $PB = 7400$
- Domain gain: $DG = 250$

For Domain 76:

$$Port_{base} = 7400 + (250 \times 76) = 26400$$

Individual nodes (participants) use offsets from this base port (e.g., +10, +11, +12, …) for discovery and user data.

To accommodate multiple ROS 2 nodes in a container, publish a range of ~20 UDP ports:

- UDP `26400` through `26420`

## AnyGrasp devcontainer (recommended: `network=bridge` + published UDP ports)

The AnyGrasp container is typically run in `bridge` mode and publishes the DDS UDP port range so that nodes outside the container (other devcontainers / other machines) can reach it.

Example `devcontainer.json` fragment:

```json
{
  "containerEnv": {
    "ROS_DOMAIN_ID": "76",
    "CYCLONEDDS_URI": "file:///home/ubuntu/colcon_ws/src/anygrasp_ros/bridge.xml"
  },
  "runArgs": [
    "--privileged",
    "--network=bridge",
    "-p",
    "26400-26420:26400-26420/udp"
  ]
}
```

Notes:

- `CYCLONEDDS_URI` should be a `file://` URI.
- If you change the `CYCLONEDDS_URI`, add it to `.gitignore` (it contains machine-specific IPs).

Create a machine-local copy of the template and edit it:

```bash
cp /home/ubuntu/colcon_ws/src/anygrasp_ros/docs/external/bridge.xml /home/ubuntu/colcon_ws/src/anygrasp_ros/bridge.xml
```

## External nodes (other devcontainers / other machines)

For nodes that are *not* inside the AnyGrasp container, the right CycloneDDS config depends on whether those nodes are behind Docker NAT. Following are some common scenarios.

## Scenario 1: Same physical host

Nodes are on the same host machine.

- AnyGrasp runs in this devcontainer.
- Other ROS 2 nodes run in other devcontainers or directly on the host.
- `HOST_IP`: the host machine IP address that *other nodes can reach* (often your LAN IP).

### AnyGrasp container config (host-side)

Edit your copied `bridge.xml`:

- Set `THIS_MACHINE_IP` = `HOST_IP`
- Set `PEER_1_IP` = `HOST_IP`

Rationale: other nodes will peer to `HOST_IP`, and the AnyGrasp container advertises `HOST_IP` as its reachable address (not the container-internal IP).

### Other nodes config

If other nodes run with `network=host`, use the `host.xml` template and set:

- `DEVICE_2_PHYSICAL_IP` = `HOST_IP`

If other nodes run with `network=bridge`, publish UDP `26400-26420` and use a `bridge.xml` copy instead:

- Set `THIS_MACHINE_IP` = `HOST_IP`
- Set `PEER_1_IP` = `HOST_IP`

## Scenario 2: Separate machines

AnyGrasp runs on this machine; a remote machine runs other nodes in devcontainers.

- `HOST_IP`: IP of the machine running the AnyGrasp container
- `REMOTE_IP`: IP of the remote machine

### AnyGrasp container config (host-side)

Edit your copied `bridge.xml`:

- Set `THIS_MACHINE_IP` = `HOST_IP`
- Set `PEER_1_IP` = `REMOTE_IP`

### Remote machine nodes config

If remote nodes run with `network=host`, create a CycloneDDS config from `host.xml` and set:

- `DEVICE_2_PHYSICAL_IP` = `HOST_IP`

If remote nodes run with `network=bridge`, publish UDP `26400-26420` on the remote devcontainer as well and use a `bridge.xml` copy:

- Set `THIS_MACHINE_IP` = `REMOTE_IP`
- Set `PEER_1_IP` = `HOST_IP`

### Network / firewall

- Ensure UDP `26400-26420` is reachable on the AnyGrasp machine.
  - Docker publishes the ports via `-p 26400-26420:26400-26420/udp`.
  - Host firewall (ufw/firewalld) must allow inbound UDP on that range.

## Scenario 3: Mixed (local + remote nodes)

The host machine runs:

- AnyGrasp container, and
- some additional ROS 2 nodes (host processes or other devcontainers)
- The remote machine also runs some ROS 2 nodes.
- `HOST_IP`: IP of the machine running the AnyGrasp container
- `REMOTE_IP`: IP of the remote machine

### AnyGrasp container config (host-side)

Edit your copied `bridge.xml`:

- Set `THIS_MACHINE_IP` = `HOST_IP`
- Set `PEER_1_IP` = `REMOTE_IP`
- (optional) Set `PEER_2_IP` = `HOST_IP` (useful if you want a single config that also bootstraps local nodes via the published ports)

### Local (host machine) non-AnyGrasp nodes config

Use a CycloneDDS config that peers to both sides:

- Peer to `HOST_IP` (to find the AnyGrasp container via the published ports)
- Peer to `REMOTE_IP` (to ensure discovery works even with multicast disabled)

You can do this by copying `docs/external/bridge.xml` and setting:

- `THIS_MACHINE_IP` = `HOST_IP`
- `PEER_1_IP` = `HOST_IP`
- `PEER_2_IP` = `REMOTE_IP`

### Remote machine nodes config

Use `host.xml` and set:

- `DEVICE_2_PHYSICAL_IP` = `HOST_IP`

## Test

From each environment (host, AnyGrasp container, remote machine), run:

```bash
ros2 topic list
```

You should see the same topics (within the same `ROS_DOMAIN_ID`).