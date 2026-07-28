# 个人贡献明细

> 本周（联调冲刺周）由 5 位同学按子包分工推进，所有产出均已 commit 并 push 到 `origin/main`。本文档按成员逐项列出任务、文件、关键 commit 与亮点，便于评审与复盘。

---

## 1. 马子越 — 调度组（Scheduler）

- **关键 commit**：`830a941` `feat(scheduler): ROS2 fallback + parse helpers + node tests`
- **任务摘要**：在 ROS2 不可用的单测环境下实现 `scheduler_pkg` 节点，提供目标/无人机解析、优先级与策略归一化等纯函数，并在缺少 ROS2 依赖时回退到最小桩，使节点测试无需真实 ROS 即可运行。
- **产出文件**：
  - `ros2_ws/src/scheduler_pkg/scheduler_pkg/scheduler_node.py`
  - `ros2_ws/src/scheduler_pkg/setup.py`
  - `ros2_ws/src/scheduler_pkg/config/scheduler.yaml`
  - `ros2_ws/src/scheduler_pkg/scheduler_pkg/README.md`
- **产出亮点**：
  - ROS2 缺失时自动 fallback，单测无需真实 ROS；`parse_targets / parse_drones / target_priority / normalize_strategy` 均为纯函数，便于逐条独立测试。
  - 调度回调 `on_target` / `on_drone` 改为快照替换语义，避免增量推送导致的状态漂移，并在无人机列表为空时清空调度结果。

---

## 2. 陈思睿 — 封控组（Containment）

- **关键 commit**：`ae68129` `feat(containment): event-driven enclosure_node with dirty-flag throttling`
- **任务摘要**：把 `containment_pkg` 升级为事件驱动模型，配合脏标记 + 定时节流抑制高频抖动；订阅追踪目标与无人机状态，输出统一封控指令。
- **产出文件**：
  - `ros2_ws/src/containment_pkg/containment_pkg/enclosure_node.py`
  - `ros2_ws/src/containment_pkg/setup.py`
  - `ros2_ws/src/containment_pkg/config/`
  - `ros2_ws/src/containment_pkg/launch/`
  - `ros2_ws/src/containment_pkg/tests/test_dynamic_voronoi.py`
  - `ros2_ws/src/containment_pkg/tests/test_enclosure_node.py`
  - `ros2_ws/src/containment_pkg/README.md`
  - `docs/integration/three_link_enclosure.md`
- **产出亮点**：
  - 脏标记 + 定时节流的事件驱动逻辑；订阅 `/target_track` 与 `/drone_states`，保留 `/enclosure_targets` 兼容输入，发布 `/enclosure_command`。
  - 9 项 pytest 全绿，含动态 Voronoi 退化场景；联调阶段协助修复 `enclosure_node.py:87` 的 C 风格 debug 调用残留。

---

## 3. 程维好 — 规划组（Planning）

- **关键 commit**：`f652dc8` `feat(planning): add planning_pkg with A* and D* Lite`
- **任务摘要**：从零搭建 `planning_pkg`，实现 A* 与 D* Lite 双算法，提供可注入栅格地图与障碍接口的 `planner_node`，并以 `planner_stub` 支撑轻量场景。
- **产出文件**：
  - `ros2_ws/src/planning_pkg/` 完整 14 文件包（约 2306 行）
  - `ros2_ws/src/planner_stub/`
- **产出亮点**：
  - A* 采用 8 邻接 + 欧氏距离 + recovery；D* Lite 暴露 `plan() / update_obstacles() / get_path()` 接口，23 条测试全绿，含动态插墙后路径改变的回归用例。
  - `planner_node` 订阅 `/task_assignment` 与 `/grid_map`，发布 `DroneStateArray` 与 `nav_msgs/Path`；联调中修复 `planner_stub_node` 误用 `DroneStateArray.header` 的问题。

---

## 4. 杨诗钰 — 感知组（Perception）

- **关键 commit**：`558d0c6` `feat(perception): stabilize tracker_node tests + launch + config`
- **任务摘要**：稳定 `cvtrack` 组件在 ROS2 节点形态下的测试与启动，补齐配置与 launch，使追踪节点可在真实 ROS2 环境中被拉起并稳定通过测试。
- **产出文件**：
  - `ros2_ws/src/perception_pkg/cvtrack/src/cvtrack/runner.py` 等 6 个 cvtrack 修订
  - `ros2_ws/src/perception_pkg/cvtrack/src/cvtrack/__init__.py`（新增）
  - `ros2_ws/src/perception_pkg/config/perception.yaml`
  - `ros2_ws/src/perception_pkg/launch/perception.launch.py`
  - `ros2_ws/src/perception_pkg/tests/conftest.py`
  - `ros2_ws/src/perception_pkg/tests/test_tracker_node.py`
  - `ros2_ws/src/perception_pkg/tests/test_yolo_inference_speed.py`
  - `docs/integration/perception_link_stability.md`
- **产出亮点**：
  - 解决 6 个 `tracker_node` 长期失败用例：`declare_parameter` 缺失、`get_clock().now().to_msg()` 链断裂、`get_logger` 缺失、`_debug_pub` / `_metrics_recorder` 缺失。
  - 15/15 `tracker_node` 测试通过 + 2/2 `coord_transform` 测试通过 + 10 项 skipped（待视频素材）；单测在 LightweightNode 桩下离线完成。

---

## 5. 何泓林 — 联调总集（Integration Lead）

- **关键 commit**：`7df2bbb` `docs+scripts(integration): three-link end-to-end integration`
- **任务摘要**：拉通“感知 → 调度 → 规划”和“感知 → 封控”三条链路，统一接口、文档与启动脚本，撰写联调手册与故障排查指南，并完成端到端三关测试。
- **产出文件**：
  - `docs/integration/interface_alignment.md`
  - `docs/integration/logging.md`
  - `docs/integration/three_link_integration.md`
  - `docs/integration/troubleshooting.md`
  - `launch/`
  - `ros2_ws/launch/`
  - `scripts/three_links_demo.sh`
  - `scripts/record_three_links.sh`
  - `ros2_ws/README.md`
  - `CHANGELOG.md`
- **产出亮点**：
  - `test_three_links.py` 三关端到端用例 30/24、16/12、8/6 全 PASS；两个 launch 文件语法校验通过；`three_links_demo.sh` 一键串联感知→调度→规划 与 感知→封控。
  - 接口拍板 12 项决策 D-1..D-12（消息类型、frame、Qos、topic 名等）；联调中修复 `enclosure_node.py:87` C 风格 debug 调用与 `planner_stub_node` 误用 `DroneStateArray.header` 的问题。

---

## 汇总

| 负责人 | 组别 | 关键 commit | 提交摘要 |
| --- | --- | --- | --- |
| 马子越 | 调度组 | `830a941` | ROS2 fallback + parse helpers + node tests |
| 陈思睿 | 封控组 | `ae68129` | event-driven enclosure_node with dirty-flag throttling |
| 程维好 | 规划组 | `f652dc8` | add planning_pkg with A* and D* Lite |
| 杨诗钰 | 感知组 | `558d0c6` | stabilize tracker_node tests + launch + config |
| 何泓林 | 联调总集 | `7df2bbb` | three-link end-to-end integration |
