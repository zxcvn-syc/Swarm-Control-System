# ROS 1 Jetson Observation Bridge

This directory is an isolated compatibility adapter for the existing Jetson TX2
(`Ubuntu 18.04`, `ROS 1 Melodic`) and the project ROS 2 Humble environment.

For the field operating procedure in Chinese, see
[`机载ROS1兼容桥操作手册.md`](机载ROS1兼容桥操作手册.md).

It transfers only these Jetson-to-ROS-2 observations over an authenticated TCP
connection:

- raw and compressed camera images;
- `CameraInfo`;
- local `PoseStamped`;
- `BatteryState`;
- MAVROS `State`.

It does **not** transfer ARM, DISARM, mode changes, MAVROS services, setpoints,
missions, PX4 parameters, takeoff, landing, or RTL. The ROS 2 receiver publishes
observations only and deliberately has no MAVROS service server. A bridged state
message must never be treated as a controllable MAVROS connection.

## Topology

The Jetson listens and the ROS 2 computer connects to it. This matches the
observed network direction: the ROS 2 VM can reach `192.168.144.60` without
requiring a route from the Jetson back into the VM NAT network.

```text
Jetson ROS 1 topics -> ros1_observation_sender.py :19001
                                  ^
                                  | authenticated TCP, observations only
                                  v
ROS 2 ros2_observation_receiver.py -> canonical ROS 2 topics
```

## Bring-up

1. Copy this directory to the Jetson as `~/swarm-control-bridge`. Do not place
   it inside any existing workspace.
2. Copy `config/jetson_ros1_sender.template.env` to
   `config/jetson_ros1_sender.env`, set mode `0600`, then set a random
   `BRIDGE_TOKEN`. Confirm actual ROS 1 topics with `rostopic list` when the
   camera and MAVROS processes are running. If their message packages are in
   an existing catkin overlay, set `ROS1_OVERLAY_SETUP` to that overlay's
   `devel/setup.bash`; the bridge only sources it and never writes to it.
3. On the ROS 2 computer, copy `config/ros2_receiver.template.env` to
   `config/ros2_receiver.env`, set mode `0600`, and set the **same** token.
   Set `ROS_DOMAIN_ID` to the dedicated domain used by the ROS 2 operator
   nodes; it must not share a domain with SITL or replay processes.
4. Start the Jetson sender, then start the ROS 2 read-only monitor:

```bash
# Jetson
cd ~/swarm-control-bridge
./start_jetson_ros1_sender.sh

# ROS 2 computer
cd ~/Swarm-Control-System-operator-console/bridge
./start_ros2_observation_console.sh
```

The monitor starts the receiver and the local browser dashboard together at
`http://127.0.0.1:18080` by default. It always starts with pilot commands
disabled and does not create ROS 2 MAVROS services. With
`DECODE_COMPRESSED_TO_RAW=false` and `USE_IMAGE_TRANSPORT_DECODER=true`, the
same command also starts the C++ `image_transport` JPEG decoder. For
receiver-only diagnostics, run `./start_ros2_receiver.sh` instead.

5. Verify only observation topics first:

```bash
ros2 topic echo /uav0/mavros/state --once
ros2 topic hz /camera/image
ros2 topic echo /camera/camera_info --once
```

The ROS 2 operator console may display the forwarded state and video. Its pilot
commands must remain disabled when the state originates from this bridge because
the receiver has no ROS 2 MAVROS service endpoints. Use a native ROS 2 MAVROS
connection to the FCU for any later, separately approved control integration.

When the Jetson has a generic UVC camera but no compatible ROS camera driver,
`start_jetson_uvc_camera.sh` publishes its real JPEG frames on the configured
compressed image topic. The ROS 2 monitor preserves the native JPEG for the
dashboard and uses the C++ `image_transport` decoder to reconstruct
`/camera/image` for raw-image consumers without blocking the Python bridge. It
does not synthesize `CameraInfo`; calibration must come from a measured camera
calibration before world-coordinate use.

## ROS 2 smoke test

Before connecting aircraft hardware, the ROS 2 computer can verify the receiver
mapping using the local synthetic sender in `tests/fake_observation_sender.py`.
It has no ROS 1 or MAVLink dependency and is only a test fixture. Run it on a
dedicated ROS domain, then check the published state, pose, battery, and camera
information topics. This validates the network protocol and ROS 2 mapping, but
does not validate a physical camera, FCU, MAVROS, or flight-control path.

## Failure behavior

- Wrong token: the Jetson rejects the peer before any observation is sent.
- Lost connection: the receiver reconnects; no historical image is replayed.
- Queue pressure: the oldest queued observation is dropped, favoring freshness.
- Missing source topics: the sender stays connected but publishes nothing for
  that topic. It does not synthesize a camera, pose, battery, or MAVROS state.
- The shared token is read from the mode-`0600` environment file and passed as
  a process environment variable, rather than as a visible command-line
  argument.
