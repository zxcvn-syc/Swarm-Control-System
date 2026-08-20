# 两小时挂机测试规范

## 执行

```bash
./scripts/run_soak_test.sh --duration 7200 --video /abs/path/to/input.mp4
```

脚本启动真实三链路八节点栈、每 30 秒记录主启动进程 RSS 和 ROS 节点数量，并生成日志、CSV 与 JSON 报告。测试开始前必须完成工作区构建，且输入视频存在。

必需节点为：`tracker_node`、`coord_transform_node`、`scheduler_node`、`planner_node`、`enclosure_node`、`ugv_state_publisher`、`px4_offboard_bridge`、`sitl_pose_bridge`。启动宽限期后任一节点缺失即判定失败。

## 通过标准

- JSON 的 `status` 为 `PASS`。
- `elapsed_duration_s` 不小于请求时长。
- CSV 最后一行 `launch_alive=1`，RSS 无持续异常增长。
- 日志中无进程提前退出或 ROS traceback。
- 正常收尾先向 ROS launch 发送 SIGINT，等待节点退出；只有未退出时才升级到 TERM/KILL。

## 归档要求

将同一时间戳的 `.log`、`_samples.csv`、`_report.json` 保存在 `output/soak/`，并复制到 `docs/evidence/` 归档。两小时结论只适用于 `status=PASS` 且 `elapsed_duration_s >= requested_duration_s` 的报告；该结论表示 ROS2 headless 集成栈稳定，不表示车辆已解锁、进入 Offboard 或完成真机飞行。
