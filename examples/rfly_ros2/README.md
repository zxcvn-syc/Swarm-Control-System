# CVTrack RflySim 空地协同封控演示说明

## 交付内容

- 总演示视频：`rfly_cvtrack_search_track_containment_demo_20260821.mp4`
- 连续跟踪原片：`rfly_cvtrack_blue_target_containment_final_20260821.mp4`
- 搜索原片：`rfly_restart_motion_check.mp4`
- 跟踪明细：`rfly_cvtrack_blue_target_containment_final_20260821.csv`
- 运行摘要：`rfly_cvtrack_blue_target_containment_final_20260821.json`
- ROS2 日志与话题样本：`logs/`
- 场景逐帧遥测：`logs/scene_telemetry.jsonl`

总演示视频长 90.05 秒、1280x720、20 FPS。它由两个真实 RflySim 在线运行片段无重编码拼接：前 15 秒展示 UAV 1/2/3 搜索视角轮换，后 75 秒展示连续识别、跟踪、机群集合和地面车辆封控。两个原片均保留，不能把拼接视频描述为单次无切换连续实验。

## 场景与行为

- 蓝色车辆 ID 101 是目标车辆，沿 Catmull-Rom 闭合路线高速行驶，并叠加两组横向摆动和周期变速。
- 三架 UAV 分区搜索；单 RGB 传感器按 UAV 1 -> 2 -> 3 轮换，每次换挂保留 2.5 秒稳定窗口。
- 视觉锁定后，主机根据运动方向预测前置位置，另外两架 UAV 向目标周围集合。
- 三辆灰色地面车接收 `/enclosure_command`，围绕 1.1 秒预测目标点形成 18 米、120 度间隔的拦截位置。
- 八辆静态车辆作为障碍物加入栅格地图和 Rfly 场景。
- 目标丢失时保留短时运动预测；超过窗口后 UAV 升高并恢复旋转搜索。

## 本次实测结果

连续跟踪原片的统计：

| 项目 | 结果 |
|---|---:|
| 视频时长 | 75.00 s |
| 视频帧数 | 1500 |
| 在线处理帧 | 4673 |
| 平均在线处理速度 | 62.31 FPS |
| 蓝色语义检测帧 | 4319 |
| 确认跟踪行 | 4319 |
| 稳定逻辑目标 ID | 10001 |
| YOLO 车辆类别辅助确认 | 2 帧 |
| ROS2 有效世界目标 | 1 |
| 地面封控命令 | 3 |
| 封控半径 | 18.0 m |

目标框中心采用画面半宽、半高归一化后的径向误差：全段均值 0.204；10 秒后 0.162；30 秒后 0.147；50 秒后 0.142。大转角时存在短时偏离，但目标保持在视野内并重新回到中部区域。

ROS2 证据：

- `/task_assignment`：`drone_id=0, target_id=1, task_type=track`
- `/enclosure_command`：同时包含 3 条命令，`drone_id=0/1/2`，每条 `enclosure_radius=18.0`
- `/target_track_world`、`/target_track_truth`、`/planned_path`、`/ground_vehicle_states` 均在话题图中存在
- enclosure 日志包含 `1 validated world target(s)` 和 `3 command(s)`

## 坐标和控制边界

这不是纯视觉世界坐标闭环。RflySim Free v4.12 在本机只能稳定提供一条 RGB 流，而且动态 `TargetCopter` 换挂没有“挂载完成”回执。一次对照运行中，动态换挂导致单目投影与仿真真值平均相差约 97.4 米，因此不能把该投影直接发送给封控车辆。

本演示采用以下边界：

1. 蓝车是否被发现、图像框、跟踪状态和丢失恢复由实时 RGB 图像、蓝色语义门和 CVTrack BoT-SORT 产生。
2. 每帧图像投影继续写入 `target_visual`，用于误差审计。
3. 只有视觉检测成立后，控制层才启用 Rfly 场景真值坐标，遥测明确标记 `target_control_source=truth_assist`。
4. `/target_track_world`、UAV 集合和地面封控使用该 `truth_assist` 控制坐标。

因此可以宣称“视觉触发的 Rfly 空地协同控制演示”，不能宣称“未经辅助的纯单目测绘闭环”。要移除辅助，需使用支持多相机时间戳/挂载回执的 Rfly 版本，或接入真实相机内外参、同步 UAV 位姿和标定地面坐标。

## 飞控边界

- 本次使用 Rfly `UE4CtrlAPI` 运动学位置接口驱动场景对象。
- 未连接 MAVROS，未 ARM，未切换 PX4 Offboard。
- 这段视频证明视觉、ROS2 决策话题和 Rfly 场景执行链路，不等价于 PX4 飞控闭环或真机飞行验证。

## 复现要点

Windows 侧需要 RflySim3D、RflySimSDK、Python 3.10+、OpenCV、Ultralytics 和本仓库。WSL/ROS2 侧需要 Humble 以及包含 `scheduler_pkg`、`planning_pkg`、`containment_pkg`、`swarm_interfaces` 的外部工作区。

先启动 RflySim3D，再在 WSL 中运行：

```bash
export RFLY_DEMO_ROOT=/path/to/rfly_ros2_demo
export ROS2_WS_ROOT=/path/to/Swarm-Control-System/ros2_ws
bash "$RFLY_DEMO_ROOT/scripts/run_ros_chain.sh" 82
```

随后在 Windows 中运行：

```powershell
python scripts\rfly_live_cvtrack.py `
  --duration 75 `
  --config scripts\Config.json `
  --weights C:\path\to\yolov8s.pt `
  --output demo.mp4 `
  --csv demo.csv `
  --summary demo.json
```

如 RGB 流冻结在同一纹理，先完全退出两个 `RflySim3D.exe` 进程并重新启动，再做 10-15 秒运动流检查。
