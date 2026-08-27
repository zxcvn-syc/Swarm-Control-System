# 单机 PX4/MAVROS Offboard SITL 复验（2026-08-27）

## 目的与边界

本记录复验 `px4_offboard_bridge` 的受控状态机是否能在**单机 PX4/Gazebo
SITL** 中完成 MAVROS 连接、setpoint 预发、ARM 与 `OFFBOARD` 请求。它与
三机进程稳定性批测分开执行，不代表三机同时 Offboard，也不代表真机飞行。

## 环境

- 源码：`825a072895c9f906bacfc041541d183519dd1769`
- Ubuntu 22.04，Python 3.10.12，ROS 2 Humble
- PX4 v1.14.0，Gazebo Classic 11.10.2，MAVROS
- `ROS_DOMAIN_ID=68`，world 为 `swarm_field.world`
- 启动入口：`ros2 launch planning_pkg px4_sitl.launch.py headless:=true`

该 launch 对桥接器显式传入：

```text
enable_setpoint_streaming:=true
auto_arm:=true
drone_id:=0
```

## 观测结果

桥接器日志按顺序记录：

```text
prestream -> arming -> offboard -> active
```

MAVROS 状态采样为 `connected: true`、`armed: true`、`mode: OFFBOARD`。位姿
采样约为 `(0.011, 0.017, 1.995) m`；`PositionTarget` setpoint 为 `(0, 0, 2) m`。
PX4 日志也记录了 `Armed by external command` 与 `Takeoff detected`。

原始采样与日志已随本分支归档：

- `docs/evidence/px4_offboard_sitl_20260827/state.yaml`
- `docs/evidence/px4_offboard_sitl_20260827/state_final.yaml`
- `docs/evidence/px4_offboard_sitl_20260827/pose.yaml`
- `docs/evidence/px4_offboard_sitl_20260827/setpoint.yaml`
- `docs/evidence/px4_offboard_sitl_20260827/phase.log`
- `docs/evidence/px4_offboard_sitl_20260827/launch.log`

## 限制

该证据只支持“一架仿真无人机接受 hold setpoint 并进入 Offboard”的结论。它不
证明路径任务完成、自动降落、避障、长时间飞行、三机飞控协同、RflySim 等价性或
任何真机能力。验证结束后已向本次启动的进程树发送定向终止信号，并确认无
Gazebo、PX4 或 MAVROS 残留进程。
