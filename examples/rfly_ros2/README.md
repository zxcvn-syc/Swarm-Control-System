# CVTrack RflySim 空地协同封控演示说明

## 2026-08-21 最终联调结果

最终验证使用 `rain_3ddisplay` 场景真实同步运行：RflySim3D 雨天类型 5、3 辆动态灰车遮挡、8 辆静态障碍车；Windows 侧实时 RGB 识别通过 UDP 发送到 ROS VM，ROS2 决策链再将封控命令回传 Rfly 场景。

| 指标 | 结果 |
|---|---:|
| 在线视觉处理 | 872 帧，29.07 FPS |
| 确认目标跟踪 | 826 行，稳定逻辑 ID `10001` |
| 视觉压力 | 46 个传感器遮挡帧，雨滴/模糊均启用 |
| ROS 有效消息 | 7 类话题全部收到 |
| 封控命令 | 2 次有效消息，每次 3 条有限坐标命令 |
| 封控半径 | 3 条均为 18.0 m |
| 决策视频 | 30 s，1740x720，20 FPS |

本次修复了外部 `containment_pkg` 的单目标多执行车逻辑：原节点用 `min(目标数, 执行车数)` 错误地把 2 辆车写成 standby `NaN`；现在每辆可用地面车都会得到一个有效 Voronoi 环绕点。修复源和 VM 兼容 launcher 位于 `ros2_patches/containment_pkg/`，ROS 采样器为 `scripts/capture_ros_evidence.py`。

最终成片为 `rfly_final_decision_replay_20260821.mp4`；它使用 `34.8 s` 遥测时间偏移对齐视觉视频和场景遥测。控制层仍明确标记为 `truth_assist`：视觉检测触发控制，Rfly 真值坐标用于当前 Free 版本缺少动态相机挂载回执时的世界坐标执行，不能描述为纯单目测绘闭环。未 ARM、未切换 PX4 Offboard、未连接 MAVROS。

## 交付内容

- 总演示视频：`rfly_cvtrack_search_track_containment_demo_20260821.mp4`
- 连续跟踪原片：`rfly_cvtrack_blue_target_containment_final_20260821.mp4`
- 搜索原片：`rfly_restart_motion_check.mp4`
- 跟踪明细：`rfly_cvtrack_blue_target_containment_final_20260821.csv`
- 运行摘要：`rfly_cvtrack_blue_target_containment_final_20260821.json`
- ROS2 日志与话题样本：`logs/`
- 场景逐帧遥测：`logs/scene_telemetry.jsonl`
- 决策可视化脚本：`scripts/make_decision_visualization.py`
- ROS2 有效消息采样器：`scripts/capture_ros_evidence.py`
- 场景预设：`scripts/scenario_presets.json`

总演示视频长 90.05 秒、1280x720、20 FPS。它由两个真实 RflySim 在线运行片段无重编码拼接：前 15 秒展示 UAV 1/2/3 搜索视角轮换，后 75 秒展示连续识别、跟踪、机群集合和地面车辆封控。两个原片均保留，不能把拼接视频描述为单次无切换连续实验。

## 场景与行为

- 蓝色车辆 ID 101 是目标车辆，沿 Catmull-Rom 闭合路线高速行驶，并叠加两组横向摆动和周期变速。
- 三架 UAV 分区搜索；单 RGB 传感器按 UAV 1 -> 2 -> 3 轮换，每次换挂保留 2.5 秒稳定窗口。
- 视觉锁定后，主机根据运动方向预测前置位置，另外两架 UAV 向目标周围集合。
- 三辆灰色地面车接收 `/enclosure_command`，围绕 1.1 秒预测目标点形成 18 米、120 度间隔的拦截位置。
- 八辆静态车辆作为障碍物加入栅格地图和 Rfly 场景。
- 目标丢失时保留短时运动预测；超过窗口后 UAV 升高并恢复旋转搜索。

## 场景与天气压力测试

场景通过 `RFLY_SCENARIO` 选择，预设在 `scripts/scenario_presets.json`：

| 预设 | Rfly 地图 | 天气 | 遮挡/动态障碍 |
|---|---|---|---|
| `clear_grasslands` | `Grasslands` | 晴天 | 关闭 |
| `rain_3ddisplay` | `3DDisplay` | 雨 | 3 辆动态灰车，其中 1 辆进入目标视线 |
| `fog_3ddisplay` | `3DDisplay` | 雾 | 3 辆动态灰车 |
| `snow_3ddisplay` | `3DDisplay` | 雪 | 2 辆动态灰车 |
| `city_clear` | `Changsha` | 晴天 | 4 辆动态灰车 |
| `mountain_clear` | `MoutainRoad` | 晴天 | 3 辆动态灰车 |

WSL/ROS2 链路第二个参数为场景名：

```bash
bash "$RFLY_DEMO_ROOT/scripts/run_ros_chain.sh" 82 rain_3ddisplay
```

天气预设使用 Rfly 官方 WeatherController（天气类型：雨 5、雪 8、雾 10），而遮挡压力来自场景中真实显示的动态车辆。若所选地图不支持天气控制器，日志会保留地图和天气配置，但不能把画面描述为天气特效已生效。

## 决策过程可视化

跟踪视频和场景遥测完成后，生成右侧上帝视角地图、视觉投影、truth_assist 控制点、预测轨迹、三架 UAV 和三辆地面车任务线：

```bash
python scripts/make_decision_visualization.py \
  --input outputs/demo.mp4 \
  --telemetry logs/scene_telemetry.jsonl \
  --output outputs/demo_decision_replay.mp4 \
  --summary outputs/demo_decision_replay.json
```

可视化中的 `X truth` 只用于对照评估，`O visual` 是图像投影，`G control` 是实际控制层输入；如果控制源显示 `truth_assist`，不能将该视频描述为纯视觉世界坐标闭环。

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
export ROS2_SETUP=$ROS2_WS_ROOT/install_validation/setup.bash
export RFLY_HOST_IP=192.168.88.1
bash "$RFLY_DEMO_ROOT/scripts/run_ros_chain.sh" 82 rain_3ddisplay
```

随后在 Windows 中运行：

```powershell
python scripts\rfly_live_cvtrack.py `
  --duration 75 `
  --config scripts\Config.json `
  --weights C:\path\to\yolov8s.pt `
  --scenario rain_3ddisplay `
  --udp-host 192.168.88.135 `
  --output demo.mp4 `
  --csv demo.csv `
  --summary demo.json
```

如 RGB 流冻结在同一纹理，先完全退出两个 `RflySim3D.exe` 进程并重新启动，再做 10-15 秒运动流检查。
