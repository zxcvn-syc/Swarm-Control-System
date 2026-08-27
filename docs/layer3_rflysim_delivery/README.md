# CVTrack RflySim3D 空地协同演示

> 此目录保留 2026-08-22 的交付快照，供回溯历史证据。当前可运行源码已完整融合到
> [`ros2_ws/src/perception_pkg/cvtrack`](../../ros2_ws/src/perception_pkg/cvtrack)，
> 其中的 `examples/rfly_ros2` 是唯一应继续维护和运行的 Rfly 演示入口。不要从本目录
> 的 `code/` 启动旧版脚本；融合路径、依赖和验证边界见
> [`docs/integration/hehonglin_cvtrack_rfly_merge.md`](../integration/hehonglin_cvtrack_rfly_merge.md)。

本目录提供 CVTrack、RflySim3D 和 ROS2 的联调脚本。它以蓝色高速目标车为视觉目标，完成 UAV 搜索、视觉锁定、运动预测、遮挡后重捕获、机群集合及灰色地面车封控，并输出 UAV 视频、上帝视角决策回放和可验证遥测。

## 已验证运行

`outputs/rfly_full_demo_20260823_211000/` 为 `rain_wind_3ddisplay` 的最终已通过运行：55.97 秒、30 FPS、1407 个在线视觉帧（22.69 FPS）、1369 个确认跟踪帧、81.4% 目标居中率；4 次物理遮挡后的重捕获均成功，最长 1.078 秒；ROS2 的 7 类话题均收到有效消息，车辆重叠为零，最小安全间隙为 0.001147 m。

详细报告见 `FINAL_REPORT_ZH.md`。

## 组件

- `scripts/rfly_live_cvtrack.py`：读取 Rfly RGB 传感器，应用天气压力，识别蓝色目标，以 CVTrack BoT-SORT 跟踪并把视觉轨迹送往 ROS2。
- `scripts/rfly_ros_scene.py`：驱动 Rfly 场景中的目标车、UAV、灰色地面车和大型障碍；提供预测、搜索、封控和碰撞分离状态。
- `scripts/run_ros_chain.sh`：在 ROS VM 中启动场景、调度、规划、封控及证据采集。
- `scripts/make_decision_visualization.py`：生成左侧 UAV 视角、右侧上帝视角的决策回放。
- `scripts/validate_rfly_run.py`：验证视频动态性、跟踪、居中、ROS2 消息、物理遮挡/重捕获和车辆分离。
- `tools/run_rfly_full_demo.ps1`：Windows 一键编排入口。

## 场景预设

预设位于 `scripts/scenario_presets.json`。

| 预设 | 地图 | 压力 |
|---|---|---|
| `clear_grasslands` | Grasslands | 晴天、无动态障碍 |
| `rain_3ddisplay` | 3DDisplay | 雨、风、动态大型遮挡 |
| `strong_wind_3ddisplay` | 3DDisplay | 强风、动态大型遮挡 |
| `rain_wind_3ddisplay` | Grasslands | 14 m/s 风、雨滴、雾化、模糊、动态大型遮挡 |
| `fog_3ddisplay` | 3DDisplay | 浓雾、模糊、动态大型遮挡 |
| `snow_3ddisplay` | 3DDisplay | 雪、雾化、动态大型遮挡 |
| `city_clear` | OldFactory | 城市/工厂障碍、动态大型遮挡 |
| `mountain_clear` | MountainTerrain | 山地、动态大型遮挡 |

每个非晴天预设包含大型静态工程障碍、动态工程障碍和基于视线走廊的物理遮挡车。几何对准遮挡阶段会在检测输入中触发受限的同步可见性退化，确保可重复验证“丢失后重捕获”；这不是对相机物理光线遮挡精度的主张。

## 快速运行

前置条件：Windows 上启动 RflySim3D、RflySimSDK、Python 3.11、OpenCV、Ultralytics 和 `yolov8s.pt`；ROS VM 具备 ROS2 Humble、相关 swarm 包及 SSH 访问；UDP/TCP relay 已启动。

```powershell
.\tools\run_rfly_full_demo.ps1 `
  -Duration 62 `
  -Scenario rain_wind_3ddisplay `
  -Python "C:\Users\911MT\AppData\Local\Programs\Python\Python311\python.exe"
```

成功时输出目录包含 `uav_live.mp4`、`decision_god_view.mp4`、`validation.json`、`detection_summary.json`、`tracks.csv`、`scene_telemetry.jsonl` 和 `capture_summary.json`。录制的 UAV 画面保留原始 RGB 可读性，遮挡退化只作用于检测输入；视频状态栏仍显示物理遮挡和重捕获阶段。

Windows 编排脚本会在存在物理遮挡事件时，根据首个视频/ROS 遮挡事件自动估计时间偏移，并把 `telemetry_offset_s` 写入 `decision_god_view.json`；没有物理遮挡的场景使用显式的 `0.0s` 默认值，不能把它当作跨主机时钟同步证明。

若脚本提示没有遥测，检查 VM 的 SSH 连通性、`RFLY_*_BRIDGE_*` 环境变量和 relay 进程。若视频静止或黑屏，重启 RflySim3D 后先运行短时采集确认传感器持续更新。

## 控制边界

视觉检测和跟踪来自实时 Rfly RGB 输入，遥测中的 `target_control_source=vision` 表示控制使用图像投影得到的目标状态。该投影依赖仿真相机位姿，不能被描述为未经标定和同步验证的纯单目世界坐标闭环。场景控制使用 `UE4CtrlAPI` 运动学接口，未执行 MAVROS、ARM、PX4 Offboard 或真机飞控操作。
