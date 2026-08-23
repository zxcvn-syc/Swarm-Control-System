# 何泓林 CVTrack + RflySim3D 联调报告

## 2026-08-23 最终验证

本轮交付使用 `rain_wind_3ddisplay` 预设，在 RflySim3D、Windows 实时视觉进程和 ROS2 虚拟机之间完成单次 55.1 秒连续运行。蓝色目标车按受速度、加速度、制动、横摆角速度和最小转弯半径约束的路线行驶；三辆灰色地面车接收封控命令；大型动态工程车、静态工程障碍和停放车辆参与避障与遮挡压力测试。

| 验收项 | 实测结果 |
|---|---:|
| UAV 视角视频 | 55.0 s，1280x720，30 FPS，1650 帧 |
| 视频动态采样比例 | 93.3% |
| 在线视觉处理 | 1217 帧，22.12 FPS |
| 确认目标跟踪 | 1137 帧，稳定逻辑 ID `10001` |
| 目标居中率 | 75.5%（验收阈值 35%） |
| 物理遮挡触发 | 66 条请求记录，58 条几何对准记录 |
| 遮挡后的重捕获 | 3/3，最大 1.11 s |
| 车辆重叠 | 0 |
| ROS2 话题 | 7/7 均收到有效消息 |
| 地面封控命令 | 2 条有效 `/enclosure_command` |

本次生成的原始证据均在 `outputs/rfly_full_demo_20260823_184221/`：

- `uav_live.mp4`：Rfly UAV 第一视角，包含识别框、搜索/锁定/重捕获状态。
- `decision_god_view.mp4`：左侧 UAV 视角、右侧上帝视角、目标与无人机轨迹、预测、地面封控和障碍物。
- `validation.json`：全部验收项通过的机器可读结果。
- `detection_summary.json`、`tracks.csv`：检测、跟踪与重捕获证据。
- `scene_telemetry.jsonl`、`capture_summary.json`：Rfly/ROS2 遥测和话题采样结果。
- `decision_god_view.json`：决策回放的帧数、分辨率和自动估计的遥测时间偏移。

## 实现内容

- 蓝色目标车 ID 101 使用前向车辆运动学。速度上限 17 m/s，加速度上限 2.4 m/s²，制动上限 3.2 m/s²，最大横摆角速度 0.48 rad/s，最小转弯半径 20 m。
- 三架 UAV 分区搜索；主机发现目标后根据目标速度做预测前置，自动调整相机 FOV 和飞行高度。其他 UAV 向目标周围集合。
- 3 辆灰色地面车根据 ROS2 `/enclosure_command` 向目标预测位置形成封控；避让目标、彼此、动态障碍和静态大型障碍。
- 场景含 4 个大型静态工程障碍、8 辆静态车辆和 3 个动态工程障碍。车辆间使用半径与安全边界分离，不允许重叠。
- `rain_wind_3ddisplay` 使用 14 m/s 风场、视觉雨滴、雾化和轻度模糊。其余预设覆盖晴天、雨天、强风、雾、雪、城市和山地。
- 物理遮挡车持续根据相机至目标的视线走廊对准；对准后，检测器输入施加最多 1 秒的同步可见性退化，触发搜索、升高和重捕获。视频以 `SENSOR OCCLUSION` 和 `PHYSICAL BLOCKER / REACQUISITION SEARCH` 标记该阶段。
- 决策回放同步显示目标蓝车、预测箭头、三架 UAV、三辆灰色地面车、静态/动态障碍、目标历史轨迹和封控任务线。

## 运行方式

先启动 `F:\RflySim3D\RflySim3D.exe`，并保证 ROS VM 可 SSH 访问。Windows 侧从仓库根目录运行：

```powershell
.\tools\run_rfly_full_demo.ps1 `
  -Duration 55 `
  -Scenario rain_wind_3ddisplay `
  -Python "C:\Users\911MT\AppData\Local\Programs\Python\Python311\python.exe"
```

脚本会启动 VM 中的 ROS2 场景、等待遥测就绪、启动 Windows 的实时视觉处理、下载 ROS2 遥测、生成上帝视角视频，并运行 `validate_rfly_run.py`。通过验收才会打印 `Full Rfly demo completed`。

可用预设定义在 `scripts/scenario_presets.json`：`clear_grasslands`、`rain_3ddisplay`、`strong_wind_3ddisplay`、`rain_wind_3ddisplay`、`fog_3ddisplay`、`snow_3ddisplay`、`city_clear`、`mountain_clear`。

## 坐标和飞控边界

这是视觉触发的 Rfly 空地协同控制演示，不是 PX4 或真机飞控验证。

- Rfly Free 当前稳定提供单条原生 UAV1 RGB 流。UAV2/UAV3 的搜索和集合是场景级协同状态，不能描述为已验证的多相机原生视角交接。
- 蓝色车辆检测、BoT-SORT 跟踪、丢失和重捕获由实时 RGB 输入产生。`target_visual` 保留视觉投影以供审计。
- 遥测中的 `target_control_source=vision` 表示控制使用图像投影得到的目标状态；该投影依赖仿真相机位姿，不能声称为未经标定和同步验证的纯单目世界坐标闭环。
- 控制使用 `UE4CtrlAPI` 运动学接口；未连接 MAVROS、未 ARM、未进入 PX4 Offboard 模式。
