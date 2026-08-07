# PX4 SITL 仿真环境搭建指南

> Phase 3 (P3) simulation environment for `Swarm-Control-System`.
> Target: Ubuntu 22.04 + ROS2 Humble + Gazebo Classic + PX4-Autopilot SITL.
> Read this end-to-end once before you start — the install sequence matters.

---

## 1. 目标（What you get）

A self-contained dev environment where:

* `ros2 launch planning_pkg planning.launch.py include_sitl:=true`
  brings up **1-3 PX4 iris drones** in a Gazebo simulation, each connected
  to a MAVROS node, with the planner + bridges wired up.
* `/planned_path` published by `planner_node` is forwarded to PX4 as
  `mavros_msgs/PositionTarget`, and the SITL pose comes back as
  `swarm_interfaces/DroneStateArray` on `/drone_pose_external`.

The pipeline is:

```
planner_node -> /planned_path -> px4_offboard_bridge -> /mavros/setpoint_raw/local
                                                            |
                                                            v
                                                       PX4 SITL (gz_iris)
                                                            |
                                                            v
  /drone_pose_external <- sitl_pose_bridge <- /mavros/mocap/pose <-+
```

---

## 2. 在 Ubuntu 22.04 上从零搭建

### 2.1 ROS2 Humble（系统依赖）

Follow the [official install steps](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

```bash
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-humble-desktop python3-colcon-common-extensions \
    python3-pip python3-numpy python3-yaml
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2.2 Gazebo Classic + MAVROS

PX4 SITL uses **Gazebo Classic** (gazebo 11), not Garden — the Gazebo
Garden `gz` transport does not yet have a fully-supported iris plugin.

```bash
sudo apt-get install -y \
    gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-dev \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    geographiclib-tools

# MAVROS needs the GeographicLib datasets for geodesic conversions.
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

> 磁盘空间：本节约 1.5 GB。

### 2.3 PX4-Autopilot SITL

```bash
# Clone the v1.14 release (or any tag compatible with the gz_iris model).
git clone --depth 1 --branch v1.14.0 https://github.com/PX4/PX4-Autopilot.git \
    $HOME/src/PX4-Autopilot
export PX4_SITL_ROOT="$HOME/src/PX4-Autopilot"
echo "export PX4_SITL_ROOT=$PX4_SITL_ROOT" >> ~/.bashrc

cd $PX4_SITL_ROOT
git submodule update --init --recursive --depth 1 \
    src/drivers/gps/devices \
    src/drivers/uavcan/libuavcan \
    src/lib/matrix \
    src/lib/mathlib \
    src/modules/ekf2 \
    src/modules/mc_att_control \
    src/modules/mc_pos_control \
    src/modules/mc_rate_control \
    src/modules/mc_hover_thrust_estimator \
    src/modules/micrortps_bridge \
    src/modules/simulator/simulator_mavlink \
    src/modules/uORB \
    src/modules/simulation \
    Tools/simulation/gazebo-classic/sitl_gazebo-classic

# Build px4_sitl_default (≈15 min on 4 cores, ≈4 GB).
make px4_sitl_default
```

> If `make px4_sitl_default` complains about a missing gazebo headers
> package, install `libgazebo-dev` (already pulled above).

### 2.4 Swarm-Control-System workspace

```bash
cd $HOME
git clone https://github.com/zxcvn-syc/Swarm-Control-System.git
cd Swarm-Control-System
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select swarm_interfaces perception_pkg scheduler_pkg \
    planning_pkg containment_pkg
```

### 2.5 Sanity check

```bash
cd $HOME/Swarm-Control-System
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
PX4_SITL_ROOT=$HOME/src/PX4-Autopilot \
    ros2 launch planning_pkg px4_sitl.launch.py num_uav:=1
```

A Gazebo window should open with an iris drone on the `swarm_field` world,
and you should see MAVROS print:

```
[INFO] [mavros_node]: FCU: connected
```

---

## 3. 在 Docker 里跑（推荐 for CI）

The image `docker/px4_sitl.Dockerfile` ships ROS2 Humble + Gazebo + PX4
pre-built.  Build it once:

```bash
docker build -f docker/px4_sitl.Dockerfile -t swarm-sim:px4-sitl .
```

Notes:

* **Disk space**: ≈ 5 GB after build (PX4 SITL alone is ~3 GB).
* **Build time**: 20-30 min on a 4-core runner.
* **Networking**: use `--net=host` so the PX4 UDP ports + MAVROS can talk
  to each other inside the container.

Run a headless SITL test inside the image:

```bash
docker run --rm -it \
    --net=host \
    -e GAZEBO_HEADLESS=true \
    -e SITL_TIMEOUT=60 \
    -v "$PWD":/workspace -w /workspace \
    swarm-sim:px4-sitl \
    bash -lc "source /opt/ros/humble/setup.bash && \
              source /workspace/ros2_ws/install/setup.bash && \
              PX4_SITL_ROOT=/opt/px4/Firmware \
              ros2 launch planning_pkg sitl_test.launch.py"
```

---

## 4. 怎么验证它跑起来了

### 4.1 MAVROS connected

In another terminal (sourced with `setup.bash`):

```bash
ros2 topic echo /uav0/mavros/state --once
# Expect: connected: true, mode: AUTO_LOITER (or similar), armed: false
```

### 4.2 SITL pose coming back

```bash
ros2 topic hz /uav0/drone_pose_external
# Expect: ~30 Hz steady stream of swarm_interfaces/DroneStateArray
```

### 4.3 Setpoint forwarding

Publish a static path and watch the UAV move:

```bash
ros2 topic pub /planned_path nav_msgs/Path '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "world"}, poses: [
  {header: {frame_id: "world"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
  {header: {frame_id: "world"}, pose: {position: {x: 5.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}}
]}' -1
```

The drone should take off to z=5 and move toward x=5 in the Gazebo
window.  Arming is automatic once PX4 receives the offboard setpoint
stream; if it stays in `AUTO_LOITER` for >5 s, check the MAVROS log for
"OFFBOARD rejected".

### 4.4 End-to-end with the planner

```bash
ros2 launch planning_pkg planning.launch.py include_sitl:=true num_drones:=1
```

You should see (in `output/planner_<timestamp>.log`):

```
[planner_node] tick 0: 1 drones, 1 tasks, publishing path with N poses
[px4_offboard_bridge] got path with N waypoints, sending setpoint
[sitl_pose_bridge] publishing DroneStateArray
```

---

## 5. 故障排查

### 5.1 `PX4_SITL_ROOT not set`

Either `export PX4_SITL_ROOT=$HOME/src/PX4-Autopilot` or pass it inline:
`PX4_SITL_ROOT=$HOME/src/PX4-Autopilot ros2 launch planning_pkg px4_sitl.launch.py`.

### 5.2 `make px4_sitl_default` fails on `ninja: error: unknown target 'install'`

Make sure submodules are initialized (see §2.3).  Without the SITL
Gazebo submodule, the `gazebo-classic` build target is missing.

### 5.3 MAVROS prints `FCU: not connected`

Check that PX4 is actually running:

```bash
tail -50 /tmp/px4_sitl_0.log   # or /tmp/px4_sitl_<i>.log for instance i
```

If you see `INFO  [mavlink] mode Normal` followed by no further output,
the firmware started but MAVROS can't find it — verify the UDP port.
`px4_sitl.launch.py` opens ports `14540, 14550, 14560, …`; MAVROS is
configured to listen on `udp://:<port>@127.0.0.1:<port+17>`.  Mismatched
ports = silent failure.

### 5.4 Gazebo crashes with `libgazebo.so: cannot open shared object file`

`GAZEBO_PLUGIN_PATH` is not set.  Run `simulation/scripts/run_px4_sitl.sh`
instead of the bare binary — the script exports it for you.

### 5.5 SITL connects but drone never arms

`px4_offboard_bridge` must publish setpoints at >= 2 Hz **before** the
drone can enter OFFBOARD mode.  If you publish a path of length 1, the
bridge will publish the same waypoint forever (good); if you publish an
empty path, the bridge stops publishing (bad).

---

## 6. CI 集成（GitHub Actions）

The simulation image is **optional** for the default CI workflow — the
plain ROS2 build (without Gazebo/PX4) runs in seconds.  The SITL test
launch is exercised only when the optional `simulation-test` job runs.
Example step:

```yaml
- name: Build SITL image
  run: docker build -f docker/px4_sitl.Dockerfile -t swarm-sim:px4-sitl .

- name: Run SITL headless test
  run: |
    docker run --rm --net=host \
      -e GAZEBO_HEADLESS=true -e SITL_TIMEOUT=60 \
      -v "$PWD":/workspace -w /workspace \
      swarm-sim:px4-sitl \
      bash -lc "source /opt/ros/humble/setup.bash && \
                source /workspace/ros2_ws/install/setup.bash && \
                PX4_SITL_ROOT=/opt/px4/Firmware \
                ros2 launch planning_pkg sitl_test.launch.py"
```

If the SITL image is too large for your free tier, gate it behind a
manual `workflow_dispatch` or schedule job instead of running on every
push.
