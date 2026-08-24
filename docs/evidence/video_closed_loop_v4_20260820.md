# 视频输入闭环验证（v4）

日期：2026-08-20

该验证使用实际 `tracker_node` 对交通视频进行 `MOG2 + DeepSORT` 推理，再经真实 `coord_transform_node`、`scheduler_node`、`grid_map_node`、`planner_node` 和 `enclosure_node` 产生后续话题。观测器只订阅并保存话题，从不发布目标、任务、路径或围控指令。

## 结果

- `passed`: `true`
- 非空原始追踪帧：`86`
- 非空世界坐标追踪帧：`86`，全部 `frame_id=world`
- 任务分配：`404`
- 规划路径：`100`
- 围控指令：`100`
- 障碍绕行：`true`；路径绕开列 `21-26`、行 `12-28` 的障碍区域。

原始证据为 [trace.json](video_closed_loop_v4_20260820/trace.json)。其中含输入元数据、每个观测话题的计数和按时间记录的原始轨迹、世界轨迹、任务、路径与围控命令。

## 边界

这是离线视频回放验证。`video_replay_fixture` 只发布相机内参、虚拟相机位姿和障碍掩码，不发布目标轨迹；它不能代表真机相机标定、GPS/视觉定位或飞行结果。本次不以该 trace 生成或交付演示视频，也没有启动 PX4、MAVROS、ARM、Offboard 或任何飞行控制节点。

## 复现

在 ROS 2 Humble 隔离 overlay 中运行：

```bash
./scripts/run_video_closed_loop_demo.sh \
  --install-base /path/to/ros2_ws/install \
  --output-dir output/video_closed_loop \
  --duration 20 --domain-id 89 --detector-backend mog2
```
