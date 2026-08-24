# 3 机 PX4 SITL + Gazebo Classic 联调脚本

本目录用于在 **单机 Linux（WSL2 / Ubuntu 22.04）** 上启动 **3 架 iris 无人机（PX4 v1.14 SITL）+ Gazebo Classic（headless）+ 3 个 mavros**，并把真实飞控位姿聚合为 `DroneStateArray` 喂给 `enclosure_node`，从而跑通 **层 2（PX4 SITL + Gazebo + mavros + Voronoi 封控）** 全闭环。

> 这是相对于 `simulation/scripts/run_px4_sitl.sh`（仅支持 `NUM_UAV=1`）的 **3 机扩展方案**，与层 1 纯 ROS2 的 `full_loop_demo.launch.py`（mock 位姿）互相独立、可共存。

## 文件说明

| 文件 | 作用 |
|------|------|
| `start_3uav_sitl.sh` | 启动 gzserver + 3 个 iris + 3 个 `px4 -i N`（**不启动 gzclient GUI**），停在前台等待 |
| `start_3uav_ros.sh`  | 起 3 个 mavros → `sitl_state_publisher` → `target_pub` → `enclosure_node` |
| `sitl_state_publisher.py` | 订阅 3 个 `/uav{i}/mavros/local_position/pose`，叠加 2 个 mock UGV，发布 `DroneStateArray` 到 `/drone_states` |

## 前置条件

1. 已按 `scripts/bootstrap_px4_sitl_ubuntu.sh` 装好：Gazebo Classic 11、PX4 v1.14、mavros / mavros_msgs、`swarm_interfaces`、`containment_pkg`。
2. 已编译 PX4：`cd ~/src/PX4-Autopilot && DONT_RUN=1 make px4_sitl_default gazebo-classic`（生成 `build/px4_sitl_default/bin/px4`）。
3. 已编译 ROS2 工作区：`cd ~/ros2_ws && colcon build`。

### ⚠️ 已知编译坑（环境兼容，非仓库 bug，不进 git）

ROS2 Humble 自带的 **mavlink 头是 v2026.6.6**，比 PX4 v1.14 的 `sitl_gazebo-classic` 新。新版给相机/云台类 `mavlink_msg_*_pack_chan` 函数末尾**新增了 device_id 参数**，导致编译报 `too few arguments`。需在本机 `~/src/PX4-Autopilot/.../sitl_gazebo-classic/src/` 下手动给以下调用补 `0`：

- `gazebo_camera_manager_plugin.cpp`：`camera_information` 末尾补 **2 个 0**（camera_device_id + gimbal_device_id），其余 `camera_settings` / `video_stream_status` / `video_stream_information` / `camera_capture_status` 各补 **1 个 0**。
- `gazebo_gimbal_controller_plugin.cpp`：`gimbal_device_information` 末尾补 **2 个 0**（gimbal_device_id + reference_gimbal_device_id），`gimbal_device_attitude_status` 末尾补 **1 个 0**（gimbal_device_id）。

补完后重编即可通过（`[56/56]` 全绿）。

## 端口方案（PX4 v1.14 `px4-rc.mavlink`，已查证）

`start_3uav_sitl.sh` 给每台实例跑 `px4 -i N`（N=1,2,3），所以：

| 实例 N | PX4 监听(offboard) | mavros 绑定 | MAV_SYS_ID |
|--------|-------------------|------------|-----------|
| 1 | **14581** | 14541 | 2 |
| 2 | **14582** | 14542 | 3 |
| 3 | **14583** | 14543 | 4 |

mavros `fcu_url = udp://:{14540+N}@127.0.0.1:{14580+N}`。`start_3uav_ros.sh` 会在运行时用 `ss` 自动探测这些端口，不硬编码。

## 运行步骤（3 个终端）

```bash
# 终端 0：清理残留
pkill -9 -f px4; pkill -9 -f gzserver; pkill -9 -f gazebo; pkill -f mavros_node; sleep 2

# 终端 1：启动 3 机 SITL（无 GUI，日志在 /tmp/gazebo_multi.log）
bash start_3uav_sitl.sh
# 看到 3 行 spawned + "Ready for takeoff!" / pxh> 即可

# 终端 2：启动 ROS 链路
bash start_3uav_ros.sh
# 脚本自动探测端口 → 3 个 mavros 打印 CONNECTED → 起聚合器/enclosure_node

# 终端 3：验证
ros2 topic echo /drone_states        # 5 平台（3 UAV 真实位姿 + 2 UGV）
ros2 topic echo /enclosure_command   # 3 monitor(UAV,25m) + 2 block(UGV,15m)
```

## 已知坑与排查

| 现象 | 根因 | 处理 |
|------|------|------|
| 终端1 一直 `Connection closed by client.` | 用 PX4 自带 `sitl_multiple_run.sh` 会前台拉 `gzclient`（GUI），headless 下崩溃 → cleanup trap 把 px4 全 kill | 本脚本已**抛弃** `sitl_multiple_run.sh`，不再启动 gzclient |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` 下 ROS `setup.bash` 引用未定义变量而 abort | `start_3uav_ros.sh` 已去掉 `set -u`（用 `set -eo pipefail`） |
| `Address already in use` (Gazebo 11345) | 旧 gzserver 残留 | 脚本开头已 `pkill -9` 并等待 11345 释放；仍占用用 `ss -tlnp\|grep 11345` 找 PID `kill -9` |
| `/drone_states` 中 UAV 一直是 fallback(0/3/6) 且 `available:false` | mavros `local_position/pose` 用 **best-effort** QoS，订阅用默认 reliable 收不到 | `sitl_state_publisher.py` 已改用 best-effort 订阅 |
| `/enclosure_command` 为空 | 多为上方 UAV 未 `available` 的连带结果 | 先确认 `/drone_states` 中 3 UAV 已 `available:true` |

## 复现验证结果（2026-08-24）

- 3 个 mavros 全部 `CONNECTED`。
- `/drone_states`：UAV 0/1/2 显示真实小坐标 + `available:true`；UGV 100/101 在 ±15 m 圆周。
- `/enclosure_command`：5 条指令，`num_drones: 5`，3 monitor（layer=0, r=25.0）+ 2 block（layer=1, r=15.0）。

## 备注

- 本方案在真实位姿下验证几何计算正确；若要让 UAV 起飞到 10 m 看 `target_z` 变化，可在任意 `pxh>` 终端执行 `commander takeoff`。
- 该脚本仅用于 Gazebo 内的 SITL 验证，**不是 PX4 Offboard / 真机飞控**（层 3 视觉封控由 RflySim3D 在何泓林机器上完成，见 `../docs/layer3_rflysim_delivery/`）。
