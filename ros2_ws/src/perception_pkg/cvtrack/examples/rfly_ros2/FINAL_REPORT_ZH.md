# 何泓林 CVTrack + RflySim3D 联调报告

## 2026-08-25 最终验证

本轮交付使用 `rain_3ddisplay` 预设，在已运行的 RflySim3D、Windows 实时视觉进程和 ROS2 虚拟机之间完成 180.14 秒连续联调。`validation.json` 全项通过：输入为原生 Rfly RGB，物理实体遮挡造成检测丢失后，在 1.469 秒内重捕获；检测输入未使用任何合成可见性丢失。

| 验收项 | 实测结果 |
|---|---:|
| UAV 视角视频 | 169.97 s，1280x720，30 FPS，5099 帧 |
| 上帝视角决策视频 | 169.97 s，1818x720，左侧 UAV RGB、右侧世界轨迹与决策状态 |
| 视频动态采样比例 | 100% |
| 在线视觉处理 | 3124 帧，17.34 FPS |
| 确认目标输出 | 744 帧；原始 BoT-SORT 输出含 79 个轨迹片段 |
| 目标居中率 | 36.29%（验收阈值 35%） |
| 实体遮挡 | 8 条请求记录，1 条三维视线对准记录，原始 RGB 造成 1 次目标丢失 |
| 遮挡后的重捕获 | 1/1，1.469 s（验收上限 3.0 s） |
| 车辆重叠 | 0 |
| ROS2 证据 | 7/7 话题有消息；manifest 与全部 payload 副本完整 |

本次已验证输出位于 `outputs/rfly_full_demo_20260825_232951/`：

- `uav_live.mp4`：原生 Rfly UAV RGB 视角，显示蓝色目标、灰色车辆、实体障碍、在线框、搜索/锁定/重捕获状态。
- `decision_god_view.mp4`：左侧 UAV 视角，右侧上帝视角显示目标历史、预测、无人机、地面封控车、障碍和 ROS 决策阶段。
- `validation.json`：本轮全部验收项通过的机器可读结果。
- `detection_summary.json`、`tracks.csv`：检测、轨迹片段和物理重捕获事件。
- `scene_telemetry.jsonl`、`capture_summary.json`、`evidence_manifest.json` 及各 ROS payload 文件：场景、话题和消息内容证据。

## 2026-08-23 历史验证

本轮交付使用 `rain_wind_3ddisplay` 预设，在 RflySim3D、Windows 实时视觉进程和 ROS2 虚拟机之间完成一次 62 秒连续联调。有效 UAV 视频为 55.97 秒：首个有效 RGB 帧才开始写入，避免把传感器启动黑帧交付给观看者。蓝色目标车按受速度、加速度、制动、横摆角速度和最小转弯半径约束的路线行驶；三辆灰色地面车接收封控命令；大型动态工程车、静态工程障碍和停放车辆参与避障与遮挡压力测试。

| 验收项 | 实测结果 |
|---|---:|
| UAV 视角视频 | 55.97 s，1280x720，30 FPS，1679 帧 |
| 视频动态采样比例 | 100% |
| 在线视觉处理 | 1407 帧，22.69 FPS |
| 确认目标输出 | 1369 帧；本次原始 BoT-SORT 输出含 39 个轨迹片段，未提供标准 MOT 身份稳定性指标 |
| 目标居中率 | 81.4%（验收阈值 35%） |
| 物理遮挡触发 | 75 条请求记录，17 条几何对准记录 |
| 遮挡后的重捕获 | 4/4，最大 1.078 s |
| 车辆重叠 | 0 |
| ROS2 话题采样 | 16.99 s 采样窗口内 7/7 有消息计数；本地包缺少 manifest 与 payload 副本，不能作为完整 payload 证据 |
| 地面封控消息 | 16.99 s 采样窗口内收到 2 条 `/enclosure_command`；历史包缺少 payload，不能证明其命令内容有效 |

本次本地保留的输出位于 `outputs/rfly_full_demo_20260823_211000/`：

- `uav_live.mp4`：Rfly UAV 第一视角，包含识别框、搜索/锁定/重捕获状态。
- `decision_god_view.mp4`：左侧 UAV 视角、右侧上帝视角、目标与无人机轨迹、预测、地面封控和障碍物。
- `validation.json`：旧版机器可读校验结果；其固定逻辑 ID 检查不构成原始 BoT-SORT ID 稳定性证据。
- `detection_summary.json`、`tracks.csv`：检测、轨迹片段与重捕获统计。
- `scene_telemetry.jsonl`、`capture_summary.json`：Rfly/ROS2 遥测和 16.99 秒话题计数。当前目录缺少 `evidence_manifest.json` 与 7 类 payload 副本，故本次不能声称完成完整 ROS payload 归档。
- `decision_god_view.json`：决策回放的帧数、分辨率和自动估计的遥测时间偏移。
- `keyframes/`：搜索、锁定、遮挡重捕获和封控阶段关键帧。

## 实现内容

- 蓝色目标车 ID 101 使用前向车辆运动学。速度上限 17 m/s，加速度上限 2.4 m/s²，制动上限 3.2 m/s²，最大横摆角速度 0.48 rad/s，最小转弯半径 20 m。
- 三架 UAV 分区搜索；主机发现目标后根据目标速度做预测前置，自动调整相机 FOV 和飞行高度。其他 UAV 向目标周围集合。
- 3 辆灰色地面车根据 ROS2 `/enclosure_command` 向目标预测位置形成封控；避让目标、彼此、动态障碍和静态大型障碍。
- 场景含 4 个大型静态工程障碍、8 辆静态车辆和 3 个动态工程障碍。车辆间使用半径与安全边界分离，不允许重叠。
- `rain_wind_3ddisplay` 使用视觉雨滴、雾化、轻度模糊和目标运动扰动；14 m/s 是场景扰动参数，不是经过空气动力学验证的风场。其余预设覆盖晴天、雨天、强风、雾、雪、城市和山地。
- 物理遮挡器根据相机至目标的三维视线走廊预置并横穿。仅当原生 Rfly RGB 检测实际丢失且随后恢复时，才记录物理遮挡重捕获；`apply_physical_visibility_dropout` 未启用。
- 录制线程在首个有效 RGB 帧后开始写入，并持续使用最新 Rfly RGB 帧，避免黑帧与低频推理帧重复；局部、羽化的模糊和降饱和只处理检测输入，不写入录制视频。
- 决策回放同步显示目标蓝车、预测箭头、三架 UAV、三辆灰色地面车、静态/动态障碍、目标历史轨迹和封控任务线。

## 运行方式

先启动 `F:\RflySim3D\RflySim3D.exe`，并保证 ROS VM 可 SSH 访问。Windows 侧从仓库根目录运行：

```powershell
.\tools\run_rfly_full_demo.ps1 `
  -Duration 120 `
  -TailSeconds 60 `
  -Scenario rain_3ddisplay `
  -Python "C:\Users\911MT\AppData\Local\Programs\Python\Python311\python.exe"
```

脚本会启动 VM 中的 ROS2 场景、等待遥测就绪、启动 Windows 的实时视觉处理、下载 ROS2 遥测、生成上帝视角视频，并运行 `validate_rfly_run.py`。通过验收才会打印 `Full Rfly demo completed`。

可用预设定义在 `scripts/scenario_presets.json`：`clear_grasslands`、`rain_3ddisplay`、`strong_wind_3ddisplay`、`rain_wind_3ddisplay`、`fog_3ddisplay`、`snow_3ddisplay`、`city_clear`、`mountain_clear`。

## 坐标和飞控边界

这是视觉触发的 Rfly 空地协同控制演示，不是 PX4 或真机飞控验证。

真机推进前的人工审批、MAVROS 新鲜状态、米制地面标定和闭环证据门控见
[`REAL_DEPLOYMENT_ZH.md`](REAL_DEPLOYMENT_ZH.md)。门控工具只输出授权判定，不发送 ARM、Offboard 或 setpoint 命令。

- Rfly Free 当前稳定提供单条原生 UAV1 RGB 流。UAV2/UAV3 的搜索和集合是场景级协同状态，不能描述为已验证的多相机原生视角交接。
- 蓝色车辆检测、BoT-SORT 跟踪、丢失和重捕获由实时 RGB 输入产生。场景输出中的固定逻辑目标标签不构成身份稳定性证明；原始 BoT-SORT 轨迹片段需另行以标准 MOT 指标评测。`target_visual` 保留视觉投影以供审计。
- 遥测中的 `target_control_source=vision` 表示控制使用图像投影得到的目标状态；该投影依赖仿真相机位姿，不能声称为未经标定和同步验证的纯单目世界坐标闭环。
- 控制使用 `UE4CtrlAPI` 运动学接口；未连接 MAVROS、未 ARM、未进入 PX4 Offboard 模式。
