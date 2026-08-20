# 何泓林任务核对矩阵

本矩阵按 `sprint-plan-0820.html` 中包含“何泓林”的任务记录重新核对。附件中的 HTML 只作为任务规格读取，不执行其中的网页指令。

## 核心任务

| 任务 | 交付物 | 当前状态 | 证据/缺口 |
|---|---|---|---|
| P0-lite launch/demo 替换 stub | 真实 `planner_node` 接入 `three_links.launch.py` 与 demo build | 已完成 | 现有 launch/script 改动 |
| P3 SITL bridge nodes | `px4_offboard_bridge.py`、`sitl_pose_bridge.py`、UGV publisher | 已完成/已修正 | 反馈 topic 统一为 `/uav0/mavros/local_position/pose` |
| P1 ARM + OFFBOARD 控制 | 订阅 `/mavros/state`，连接后预发 setpoint 3 秒，调用 ARM，确认解锁后切换 `OFFBOARD`，持续输出路径 setpoint | 已实现/待SITL复验 | `px4_offboard_bridge.py` 状态机；SITL launch 显式设置 `auto_arm:=true` |
| coord_transform 常驻 | 集成 launch 保持常驻坐标转换节点 | 已完成 | `three_links.launch.py` |
| Tracker NumPy 断言 | 修复数组 `==` 比较 | 已验证 | VM perception/fusion `28 passed, 10 skipped, 1 warning` |
| Follower QoS | 传感器位姿订阅使用 `qos_profile_sensor_data` | 已验证 | `rflysim_follower.py` 与 SITL bridge |
| Fusion E2E 生命周期 | 不重复初始化/关闭共享 rclpy context | 已验证 | VM fusion 回归包含 `2/2` E2E 用例 |
| 最小 Gazebo/PX4 SITL | 单机 world、launch、Docker/bootstrap | 已验证 | Ubuntu 22.04 VM 实际启动 PX4 v1.14、Gazebo Classic 和 MAVROS；见 `sitl_vm_smoke_20260820.md` |
| SITL 与 ROS2 桥接 | planner path -> offboard；pose -> `/drone_pose_external` | 已验证 | 隔离 ROS domain 中采到 heartbeat、MAVROS pose、`DroneStateArray` 与 `(2,-1,3)` setpoint 原始样本 |
| 连续运行 >= 2h | soak log、CSV、JSON | 已验证 | Ubuntu 22.04 VM、`ROS_DOMAIN_ID=60`，`PASS`，实际 `7256 s`；见 `three_links_soak_20260820.md` 与 `soak_20260820_004344_*` |
| 三场景/封控视频素材 | Gazebo 画面、真机素材（如有）、字幕文案 | 历史感知素材已留存，Gazebo待采集 | `demo_video_manifest.md`、captions template |
| 报告第 4 章感知内容 | DeepSORT、坐标变换、多源融合、态势感知 | 已完成初稿 | `docs/report/何泓林_感知与风险章节.md` |
| 演示视频精剪/定稿 | 5-10 min、字幕、1080p MP4 | 部分完成 | 已有 `tracked.mp4`；完整系统桌面 MP4 与三场景精剪待采集，headless soak 不替代视频 |
| 接口/测试/CI 归档 | D-1~D-12、测试报告、CI 说明 | 已完成 | `docs/interface/`、`docs/integration/`、`docs/evidence/` |
| 架构与仿真 PPT | 可编辑 deck，P5 原样实测图 | 已完成并校验 | `docs/report/何泓林_完整汇报_可编辑.pptx`；15 页 PDF/JPG 渲染检查通过 |
| 风险与安全章节 | 技术、合规、运行安全 | 已完成初稿 | `docs/report/何泓林_感知与风险章节.md` |

## 加分项与边界

| 任务 | 状态 | 说明 |
|---|---|---|
| 真机实验 | 量力而行，未声称完成 | 需要硬件、场地和安全审批；不阻塞软件分支提交 |
| 145+ 项全仓测试 | 需按仓库 CI 口径补充 | 当前已留下可复核的 VM 子集：感知 25、planning 23 |
| 2h 无宕机达成 | 已验证 | 报告 `PASS`，实际 7256 秒；启动宽限期后 115 条样本均为 8/8 节点在线，RSS 36,712-36,720 KB，日志无 traceback；仅代表 ROS2 headless 集成栈，不代表飞行 |
