# PX4 SITL VM Smoke Evidence

Date: 2026-08-20

Environment:

- Ubuntu 22.04.5 VM (`192.168.88.135`)
- ROS 2 Humble, Gazebo Classic 11.10.2, PX4 v1.14.0, MAVROS
- Isolated `ROS_DOMAIN_ID=57`; no external planner publisher was present before the test.

Commands executed after building `swarm_interfaces`, `perception_pkg`,
`scheduler_pkg`, `planning_pkg`, and `containment_pkg` in the isolated VM overlay:

```bash
PX4_SITL_ROOT=$HOME/src/PX4-Autopilot GAZEBO_HEADLESS=true \
  ros2 launch planning_pkg sitl_test.launch.py num_uav:=1 timeout:=120
```

The smoke watchdog received a real Best Effort sample from
`/uav0/mavros/local_position/pose` and exited cleanly. Launch output recorded
PX4 simulator TCP connection, `Ready for takeoff!`, and MAVROS
`CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`.

For the bridge test, `/planned_path` had zero publishers before the test. A
single `nav_msgs/Path` waypoint `(x=2.0, y=-1.0, z=3.0, frame_id=world)` was
published while the PX4 vehicle remained disarmed and no Offboard mode request
was sent. The matching MAVROS setpoint sample is stored in
`cvtrack_domain57_setpoint.yaml`; its `coordinate_frame: 1` is
`FRAME_LOCAL_NED`. MAVROS performs the ROS ENU to MAVLink LOCAL_NED conversion.

The raw pose and `DroneStateArray` output are stored in
`cvtrack_domain57_pose.yaml` and `cvtrack_domain57_bridge.yaml`. They establish
the live feedback path:

```text
/uav0/mavros/local_position/pose -> sitl_pose_bridge -> /drone_pose_external
```

This evidence does not claim vehicle arming, Offboard flight, a two-hour soak,
or a full-system video.
