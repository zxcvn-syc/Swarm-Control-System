# 何泓林 CVTrack 与 RflySim3D 融合说明

## 融合范围

`ros2_ws/src/perception_pkg/cvtrack` 是团队仓库内 CVTrack 的唯一源码副本。本次以
个人仓库 `hehong5/cvtrack` 的 `2b86df0` 为来源，通过已有 Git subtree 历史合入该目录，
保留团队在 ROS2 感知、调度、规划和封控上的后续改动。

已融合的个人交付包括：

- CVTrack 的 BoT-SORT、DeepSORT、Kalman 预测、世界坐标投影和配置/评测/测试工具；
- RflySim3D 的在线 RGB 识别、车辆运动、目标预测、物理遮挡重捕获、上帝视角回放和 ROS2 证据归档；
- 标定、传感器探针、Rfly UDP 转发及 PX4 控制审核门工具；
- 雨、强风、雾、雪、城市和山地预设，以及部署和策略文档。

旧的 `docs/layer3_rflysim_delivery` 是历史快照，不能再作为运行入口。

## 运行入口

Windows 主机运行 Rfly 全链路演示：

```powershell
cd ros2_ws\src\perception_pkg\cvtrack
.\tools\run_rfly_full_demo.ps1 `
  -Duration 120 `
  -Scenario rain_3ddisplay `
  -Python "C:\Users\911MT\AppData\Local\Programs\Python\Python311\python.exe"
```

运行前需启动 RflySim3D，并让 ROS2 虚拟机能 SSH 访问。Windows 侧需要 RflySimSDK、OpenCV、
Ultralytics 和本地 YOLO 权重；ROS2 侧需要已构建的 `Swarm-Control-System/ros2_ws`。完整参数、
产物和验收阈值以
[`examples/rfly_ros2/README.md`](../../ros2_ws/src/perception_pkg/cvtrack/examples/rfly_ros2/README.md)
为准。

## 证据与边界

源码包含 `rain_3ddisplay` 的历史严格验证报告和机器可读证据校验器。视频和运行产物没有作为
Git 大文件提交；它们由一键编排脚本在本地输出目录生成，并必须通过
`examples/rfly_ros2/scripts/validate_rfly_run.py` 后才可作为新的交付证据。

该演示使用 Rfly 的 `UE4CtrlAPI` 运动学接口。它验证视觉触发的仿真闭环，不能表述为已完成
MAVROS、PX4 Offboard、ARM 或真机飞行验证。`tools/px4_control_gate.py` 仅审核前置条件，
不会发送任何控制授权或飞控命令。

## 本次本地验证

在融合分支上已完成：

```powershell
$env:PYTHONPATH = "$PWD\ros2_ws\src\perception_pkg\cvtrack\src"
python -m compileall -q ros2_ws/src/perception_pkg/cvtrack/src/cvtrack `
  ros2_ws/src/perception_pkg/cvtrack/examples/rfly_ros2/scripts `
  ros2_ws/src/perception_pkg/cvtrack/tools
python -m pytest -q ros2_ws/src/perception_pkg/cvtrack/tests/test_config.py `
  ros2_ws/src/perception_pkg/cvtrack/tests/test_geometry.py `
  ros2_ws/src/perception_pkg/cvtrack/tests/test_metrics.py `
  ros2_ws/src/perception_pkg/cvtrack/tests/test_px4_control_gate.py `
  ros2_ws/src/perception_pkg/cvtrack/tests/test_rfly_evidence_validation.py
```

上述重点测试共 44 项通过。它们验证配置融合、世界投影、指标、PX4 审核门和 Rfly ROS2 证据清单；
不替代在已启动 RflySim3D 与 ROS2 虚拟机上的实时录制。
