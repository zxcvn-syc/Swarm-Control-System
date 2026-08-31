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
| `TargetTrack`         | 单个目标的实时轨迹与检测框 | ID、像素中心/速度、类别标签、置信度、`bbox_x1/y1/x2/y2` |
| `TargetTrackArray`    | 单帧所有确认目标的轨迹打包 | `Header`、`tracks`、`frame_idx`、`image_width/height` |
| `TaskAssignment`      | 任务分配结果                                | `uint32 drone_id`、`uint32 target_id`、`string task_type` |
| `EnclosureCommandArray` | 三层 Voronoi 封控目标与心跳 | `Header`、单调 `sequence`、`EnclosureCommand[]` |
| `FlightSafetyStatus` | 封控安全门实时状态 | 锁定/激活状态、目标锁定、链路新鲜度、会话/请求号、故障码 |
| `SafetyControl` | 封控安全门服务 | 手动/自动启用、停用、紧急保持、人工确认复位 |

> `TargetTrackArray` 在 V2 引入数组容器和 `Header`；当前版本同时携带源图像尺寸，
> 使操作台可以按真实像素坐标叠加检测框。
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
* 若后续引入分割掩码或旋转框，应新增专用消息，不能改变当前轴对齐像素框语义。
