# 何泓林 CVTrack + RflySim3D 最终验证说明

## 结论

final9 是一轮真实同步联调：RflySim3D 产生 RGB 航拍画面，CVTrack 在线识别蓝色目标车，检测包经 UDP 发送到 ROS VM；ROS2 scheduler、planner、containment 生成决策，命令再由 Rfly 场景执行。

本轮确认了三辆灰色地面车的有效封控命令，不再使用 `NaN` standby 命令。

## 实测数据

- 场景：`rain_3ddisplay`，地图 `3DDisplay`，雨天类型 `5`。
- 视觉：30 秒，1280x720，在线处理 872 帧，平均 29.07 FPS。
- 识别：826 个确认跟踪行，逻辑目标 ID `10001`，目标框来自蓝色语义检测和 BoT-SORT。
- 压力：雨滴 120、模糊核 3、周期性传感器遮挡 46 帧、3 辆动态灰车和 8 辆静态障碍车。
- ROS：`/target_track_world`、`/task_assignment`、`/planned_path`、`/enclosure_command`、`/drone_states`、`/ground_vehicle_states`、`/target_track_truth` 均收到有效消息。
- 封控：每条 `/enclosure_command` 含 `drone_id=0/1/2` 三条有限坐标命令，`enclosure_radius=18.0 m`。

## 视频

- `rfly_final_sync9_20260821.mp4`：原始 Rfly 航拍识别视频。
- `rfly_final_decision_replay_20260821.mp4`：最终汇报视频，左侧航拍画面，右侧世界坐标上帝视角；含 truth、visual、control、预测方向、UAV 和地面车目标点。
- 可视化使用 `34.8 s` 遥测偏移，将 final9 场景遥测与视觉视频对齐。

## 代码修复

- `rfly_live_cvtrack.py`：动态目标车识别、雨雾雪压力、跟踪预测、丢失搜索和 UDP 视觉输入。
- `rfly_ros_scene.py`：Rfly 场景、天气、动态遮挡、UAV 平滑跟随、真值辅助边界和遥测记录。
- `run_ros_chain.sh`：进程组清理、唯一 ROS 节点名、可靠采样和 VM 兼容启动。
- `capture_ros_evidence.py`：用 `rclpy` 直接采集有效消息，避免 VM 上 `ros2 topic echo` 异常文本污染证据。
- `ros2_patches/containment_pkg/enclosure_node.py`：修复单目标多地面车只激活第一辆车的问题。

## 边界

本实验证明“视觉触发的 Rfly 空地协同控制演示”。由于 RflySim Free 当前动态相机换挂没有完成回执，控制层使用 `truth_assist` 作为世界坐标执行源，视觉投影仍保留在遥测中审计。此次没有 ARM、PX4 Offboard 或 MAVROS 飞控闭环。

## 复现

ROS VM：`192.168.88.135`；Windows/Rfly 主机：`192.168.88.1`；ROS Domain：`61`。

```bash
export RFLY_DEMO_ROOT=/home/hhh/Downloads/cvtrack-rfly-enhanced-20260821
export ROS2_WS_ROOT=/home/hhh/Downloads/Swarm-Control-System/ros2_ws
export ROS2_SETUP=$ROS2_WS_ROOT/install_validation/setup.bash
export RFLY_HOST_IP=192.168.88.1
source /opt/ros/humble/setup.bash
source "$ROS2_SETUP"
bash "$RFLY_DEMO_ROOT/scripts/run_ros_chain.sh" 35 rain_3ddisplay
```

Windows 侧使用 `scripts/rfly_live_cvtrack.py`，将 `--udp-host` 设置为 `192.168.88.135`。视频结束后使用 `scripts/make_decision_visualization.py`，并传入相同场景的 `scene_telemetry.jsonl`。
