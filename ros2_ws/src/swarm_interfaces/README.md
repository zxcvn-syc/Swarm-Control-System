# swarm_interfaces

ROS2 接口包，集中定义异构无人集群封控系统各模块共用的消息和服务类型。

## 概述

本包是 `detector_node` / `tracker_node` / `planner_node` / `scheduler_node`
/ `decision_node` / `control_node` 之间的通信契约来源。任何模块修改
消息结构前必须先在本包提 PR 并完成 colcon build，避免下游节点因字段
不一致而出现静默 bug。

## 当前消息

| 消息                  | 用途                                        | 字段 |
|-----------------------|---------------------------------------------|------|
| `TargetTrack`         | 单个目标的实时轨迹（ID + 像素坐标 + 速度）  | `uint32 target_id`、`float64 x/y`、`float64 vx/vy` |
| `TargetTrackArray`    | 单帧所有确认目标的轨迹打包                  | `std_msgs/Header header`、`TargetTrack[] tracks`、`uint32 frame_idx` |
| `TaskAssignment`      | 任务分配结果                                | `uint32 drone_id`、`uint32 target_id`、`string task_type` |

> `TargetTrackArray` 在 V2 引入：原 `TargetTrack` 单目标字段不变，仅
> 增加数组容器和 `Header`，便于下游订阅者按帧消费。
>
> 坐标约定：`x / y / vx / vy` 均为**像素坐标**（图像平面内目标框中心），
> 与 `cvtrack` 跟踪器的原生输出一致；下游节点需要时再通过 IPM、
> 单应性矩阵或 PnP 投影到世界系。

## 构建

```bash
cd Swarm-Control-System/ros2_ws
colcon build --packages-select swarm_interfaces
source install/setup.bash
ros2 interface show swarm_interfaces/msg/TargetTrackArray
```

## 依赖

* `rosidl_default_generators`（构建时）
* `rosidl_default_runtime`、`std_msgs`、`geometry_msgs`、`builtin_interfaces`（运行时）
* ROS2 Humble

## 后续

* 视项目进展追加 `EnvironmentMap.msg`、`PathPlan.msg`、`MissionPlan.msg`
  等剩余 V1 Topic 接口对应的消息类型。
* 待规划：考虑为 `TargetTrack` 引入可选 `confidence` 字段（来自
  cvtrack 的 detection score），方便下游按可信度过滤。