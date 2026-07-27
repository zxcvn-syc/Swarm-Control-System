# 三关联调手册 (Three-Link Integration Manual)

> 负责：何泓林（联调总指挥）
> 覆盖：三关链路 — 感知→调度→规划、感知+规划→封控
> 版本：v1.0（2026-07-27 联调整合首次落地）

---

## 1. 联调目标

把团队里分散在四个人手上的节点：

| 子代理 | 节点 | 包 |
|--------|------|---|
| 程维好 | `planner_node` | `planning_pkg` （**当前空**，用 `planner_stub_node` 占位） |
| 马子越 | `scheduler_node` | `scheduler_pkg` |
| 杨诗钰 | `tracker_node` | `perception_pkg` |
| 陈思睿 | `enclosure_node` | `containment_pkg` |

拼成下图所示的「三关」端到端链路，并保证 ROS2 Topic 接口命名、消息类型、QoS、坐标系全部对齐。

```text
┌──────────────────┐  /target_track       ┌──────────────────┐  /task_assignment  ┌──────────────────┐
│ tracker_node     │  TargetTrackArray    │ scheduler_node   │  TaskAssignment   │ planner_stub_node│
│ (perception_pkg) │ ────────────────────▶│ (scheduler_pkg)  │ ──────────────────▶│ (planner_stub)   │
└──────────────────┘                      └──────────────────┘                    └──────────────────┘
        │                                          ▲                                          │  /drone_states
        │ /enclosure_targets                       │ /drone_states                            │  DroneStateArray
        │ EnclosureTargetArray                     │ DroneStateArray                          ▼
        │                                          │                                  ┌────────────────┐
        │                                          └──────────────────────────────────│ enclosure_node │
        │                                                                             │ (containment)  │
        │                                                                             └────────────────┘
        │                                                                                      │
        └──────────────────────────────────────────────────────────────────────────────────────┘
                                                /enclosure_command
                                                EnclosureCommandArray
```

---

## 2. 三关定义与责任人

### 第一关：感知 → 调度（杨诗钰 → 我（中转校验）→ 马子越）

| 属性 | 值 |
|------|----|
| **上游发布者** | `tracker_node`（杨诗钰）`/target_track` `TargetTrackArray` |
| **下游订阅者** | `scheduler_node`（马子越） |
| **校验责任** | 我（何泓林）：保证 `target_id`/`x`/`y`/`confidence`/`is_confirmed` 透传到 `TaskAssignment` 时不丢失；详见 `interface_alignment.md` 决策 D-3 |
| **已验证场景** | 8 无人机、2 目标、greedy 策略，每 tick 输出 2 条 `TaskAssignment` |

### 第二关：调度 → 规划（马子越 → 程维好）

| 属性 | 值 |
|------|----|
| **上游发布者** | `scheduler_node` `/task_assignment` `TaskAssignment`（每对 (drone, target) 一条） |
| **下游订阅者** | `planner_stub_node`（占位，**程维好的真 planner_node 上线后替换**） |
| **校验责任** | 同一（drone, target）对的多次输出必须去重/收敛；目前实现按 tick 全量重发，重复同一 pair 是预期行为，C2 计划改为 diff-only |

### 第三关：感知 + 规划 → 封控（我 + 程维好 → 陈思睿 → 程维好）

| 属性 | 值 |
|------|----|
| **上游发布者** | `tracker_node`（我中转：发布 `/enclosure_targets` `EnclosureTargetArray`）以及 `planner_stub_node` 的 `/drone_states` `DroneStateArray` |
| **下游订阅者** | `enclosure_node`（陈思睿） |
| **反馈环节** | `enclosure_node` 发布 `/enclosure_command` `EnclosureCommandArray`，由规划/控制节点消费（程维好） |
| **校验责任** | `EnclosureCommandArray` 在没有目标时输出空 list 或 standby 命令（NaN 坐标）；有目标时所有（drone_id, target_x/y, enclosure_radius）非 0 |

---

## 3. 一键启动（推荐）

```bash
cd /home/hhh/Downloads/Swarm-Control-System
./scripts/three_links_demo.sh
```

这会按下列顺序执行：

1. `colcon build --packages-select swarm_interfaces perception_pkg scheduler_pkg containment_pkg planner_stub`
2. `source install/setup.bash`
3. `ros2 launch ros2_ws/launch/three_links.launch.py video_source:=...`

详细参数见 `./scripts/three_links_demo.sh --help`。

---

## 4. 端到端集成测试

### 4.1 不依赖 YOLO 权重的「自带数据」集成测试

```bash
cd /home/hhh/Downloads/Swarm-Control-System
source ros2_ws/install/setup.bash
cd ros2_ws
python3 test_three_links.py
```

脚本会：

* 启动 `scheduler_node` / `planner_stub_node` / `enclosure_node`（真实节点）
* 由于 `tracker_node` 默认要 YOLO 权重，脚本会自动降级 — `tracker_node` 启动失败时回退到内置的 `SyntheticTrackerPublisher`（2 个移动目标）
* 同时启动 image publisher 给 tracker_node 的 topic 输入端（如果用 cv_bridge + cv2 跑得起来）
* 订阅 4 个 Topic：`/target_track`、`/task_assignment`、`/drone_states`、`/enclosure_command`
* 跑 6 秒后输出每个链路的通过/失败状态到 JSON

输出示例：

```text
link1_perception_to_scheduler:              passed=True input=29 output=24
link2_scheduler_to_planner:                 passed=True input=24 output=12
link3_perception_planner_to_enclosure:      passed=True input=29 output=6
```

### 4.2 使用真实视频 + YOLO 的全链路联调

```bash
cd /home/hhh/Downloads/Swarm-Control-System
source ros2_ws/install/setup.bash
ros2 launch ros2_ws/launch/integration_test.launch.py \
    video_source:=/home/hhh/Downloads/Swarm-Control-System/videos/test_multi_target_tracking.mp4 \
    window_sec:=15
```

会同时启动 4 个节点 + 3 秒后调度 `test_three_links.py` 作为 watchdog；JSON 报告写到 `output/test_three_links_<时间戳>.json`。

---

## 5. 完整链路端到端用例

| 用例 | 视频 | 目标 | 验证 |
|------|------|------|------|
| UC-1：多目标 | `test_multi_target_tracking.mp4` | ≥3 | `TaskAssignment` 数量 == num_targets，`EnclosureCommand` 数量 == num_drones |
| UC-2：快速运动 | `test_fast_motion.mp4` | ≥2 | scheduler 切换不能丢失 task pair |
| UC-3：遮挡 | `test_occlusion_scenario.mp4` | ≥2 | enclosure_command 在遮挡期输出 standby（NaN） |
| UC-4：尺度变化 | `test_scale_variation.mp4` | ≥1 | pred_x/pred_y 持续更新 |
| UC-5：无人机俯视 | `test_drone_aerial.mp4` | ≥2 | 配合 VisDrone 权重时首次 ID 锁定时间 < 3s |

---

## 6. 每周联调签到（流程）

1. 拉最新代码 → `git pull`
2. `./scripts/three_links_demo.sh` 启动 → 不报错
3. 另开终端 → `python3 ros2_ws/test_three_links.py` → JSON 报告 `passed: true`
4. 在 `ros2_ws/test_three_links.py` 报告文件末尾追加本周日期与你的子代理姓名
5. 把 JSON 报告归档到 `output/weekly_YYYY-MM-DD.json`
6. 录屏：`./scripts/record_three_links.sh videos/three_links_$(date +%Y%m%d).mp4`

---

## 7. 与本次联调整合相关的脚本

| 脚本 | 用途 |
|------|------|
| `scripts/three_links_demo.sh` | 一键构建 + 启动 4 个节点（不录屏） |
| `scripts/record_three_links.sh` | 启动 + 录屏（ffmpeg 或 ROS2 bag） |
| `ros2_ws/test_three_links.py` | 集成测试（拉所有 4 个真实节点） |
| `ros2_ws/launch/three_links.launch.py` | 启动 4 个节点 |
| `ros2_ws/launch/integration_test.launch.py` | 启动 4 个节点 + watchdog |

---

## 8. 已知遗留（与联调相关）

| 项 | 影响 | 处理 |
|----|------|------|
| `planning_pkg` 仍空，依赖 `planner_stub` 占位 | 第二、三关走的是 stub 路径；C1 阶段足够联调，但缺少路径规划产物 | 程维好上线后替换 launch 单行 |
| `coord_transform_node` (`C3`) 是否纳入 `three_links` | 当前 `tracker_node` 默认发布像素坐标；如果世界坐标是硬需求，必须把 `coord_transform_node` 加进 launch | 决策 D-2 写在 `interface_alignment.md` |
| 真实 YOLO 权重依赖 `~/.local_lib/` 与 `weights/` 路径 | tracker_node 启动依赖外部资源，CI 不容易跑 | 集成测试有 `--no-real-nodes` 兜底 |

---

## 9. 与其他文档的关系

* 接口对齐：`interface_alignment.md`
* 故障排查：`troubleshooting.md`
* 日志规范：`logging.md`
* 全链路（环境依赖，YOLOv8 集成）：`全链路联调手册.md`（杨诗钰维护）
* 消息定义：`swarm_interfaces/README.md`

---

*生成于 2026-07-27；记录本次联调整合的入口与责任划分。*
