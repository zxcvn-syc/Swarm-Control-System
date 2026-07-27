# 调度模块（scheduler_pkg）

## 功能

`scheduler_node` 订阅目标跟踪和无人机状态快照，按最近距离优先为每个目标分配无人机，并发布 `TaskAssignment`。默认使用 greedy 算法；也支持 `assignment_strategy:=hungarian`，当 scipy 不可用或算法失败时回退到 greedy。

## ROS 接口

| 方向 | Topic | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/target_track` | `swarm_interfaces/msg/TargetTrackArray` | 每个 `TargetTrack` 的 `target_id`、`x`、`y`、`confidence`、`is_confirmed` 会被解析并缓存 |
| 订阅 | `/drone_states` | `swarm_interfaces/msg/DroneStateArray` | 仅缓存 `available=true` 的无人机 |
| 发布 | `/task_assignment` | `swarm_interfaces/msg/TaskAssignment` | 每个分配发布一条 `drone_id`、`target_id`、`task_type` |

坐标约定必须由 tracker 与 planner 共同确认。目前 tracker 文档定义 `TargetTrack.x/y` 为图像像素坐标；若调度要使用世界坐标，应在中转/校验节点完成转换后再发布同一接口。

## 参数

默认配置在 `config/scheduler.yaml`：`num_drones=8`、`tick_period=0.5` 秒、`assignment_strategy=greedy`、`max_per_drone=2`。可通过 launch 或 ROS 参数覆盖 topic、任务类型和日志周期。

启动：

```bash
ros2 launch scheduler_pkg scheduler.launch.py
```

## 联调顺序

1. 启动 swarm_interfaces、tracker/中转节点，确认 `ros2 topic echo /target_track` 中 `target_id/x/y/confidence/is_confirmed` 正常。
2. 启动无人机状态发布者，确认 `/drone_states` 的 `available` 语义：不可用无人机不会进入调度缓存；空快照会清空旧缓存。
3. 启动本节点，检查 `/task_assignment` 的数量、ID 和 `task_type`。
4. planner 必须同时订阅 `/target_track`，维护 `target_id -> 坐标` 快照，再用 `/task_assignment` 做目标选择。

## TaskAssignment 坐标字段决议

当前 `TaskAssignment.msg` 只有 `drone_id`、`target_id`、`task_type`，不携带目标坐标。本周联调现场决议采用选项 **B**：不修改消息接口，避免破坏已有消费者；`planner_node` 同时订阅 `/target_track`，维护目标 ID 到最新坐标（及必要置信度）的缓存。何泓林的中转/校验节点继续保证 `/target_track` 的 ID 与坐标一致，若其输出已经是世界坐标，planner 直接使用；若仍是像素坐标，由 planner 或中转节点按统一标定方案转换。该决议需要在三关启动前确认坐标 frame 和转换责任人。
