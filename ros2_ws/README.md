# `ros2_ws` — ROS2 工作空间

> 顶层 ROS2 工作空间。 联调总指挥：何泓林。

---

## 1. 包清单

| 包 | 状态 | 主要节点 / 内容 | 负责 |
|----|------|---------------|------|
| `swarm_interfaces` | ✅ | 共享 `.msg`：TargetTrack / TaskAssignment / DroneState / EnclosureCommand … | 团队共用 |
| `perception_pkg` | ✅ | `tracker_node`, `coord_transform_node`, `cvtrack/` | 杨诗钰 |
| `scheduler_pkg` | ✅ | `scheduler_node` + `assign.py`（greedy / hungarian） | 马子越 |
| `planning_pkg` | ⚠️ 占位 | 仅 `README.md`，**真 planner_node 待程维好上线** | 程维好 |
| `planner_stub` | 🆕 占位 | `planner_stub_node` — 联调总指挥放的占位，发布合成 `DroneStateArray` 闭合第二关与第三关 | 何泓林（联调） |
| `containment_pkg` | ✅ | `enclosure_node` + Voronoi 库 | 陈思睿 |

---

## 2. 一键构建

```bash
cd /home/hhh/Downloads/Swarm-Control-System/ros2_ws
colcon build --packages-select \
    swarm_interfaces perception_pkg scheduler_pkg containment_pkg planner_stub
source install/setup.bash
```

---

## 3. 一键启动三关（推荐）

```bash
# 仓库根目录
cd /home/hhh/Downloads/Swarm-Control-System
./scripts/three_links_demo.sh
```

这会启动：

* `tracker_node`（video → YOLO+DeepSORT → `/target_track` + `/enclosure_targets`）
* `scheduler_node`（consume `/target_track` + `/drone_states` → `/task_assignment`）
* `planner_stub_node`（consume `/task_assignment` + `/target_track` → `/drone_states`）
* `enclosure_node`（consume `/enclosure_targets` + `/drone_states` → `/enclosure_command`）

详见 `docs/integration/three_link_integration.md`。

---

## 4. 顶层 launch 文件

| 文件 | 用途 |
|------|------|
| `ros2_ws/launch/three_links.launch.py` | 启动四节点（不录制） |
| `ros2_ws/launch/integration_test.launch.py` | 启动四节点 + 集成 watchdog 脚本 |

各子包自带的 launch：

* `perception_pkg/launch/tracker_node.launch.py` — 单 tracker_node，模式 `video` 或 `topic`

---

## 5. 顶层集成测试

```bash
cd /home/hhh/Downloads/Swarm-Control-System/ros2_ws
python3 test_three_links.py         # 默认 6s 窗口
python3 test_three_links.py --window 30 --video /abs/path/to/clip.mp4
python3 test_three_links.py --no-real-nodes   # 仅 synthetic
```

输出：`output/test_three_links_<timestamp>.json`，含三关链路 pass/fail 与计数器。

---

## 6. 联调文档

`docs/integration/`：

* `three_link_integration.md` — 三关完整说明（最重要）
* `interface_alignment.md` — Topic / 字段 / QoS / 坐标系 拍板决议（citable truth）
* `troubleshooting.md` — 常见问题与排查
* `logging.md` — 联调日志规范

`docs/interface/`（对接指南）：

* `调度组接入指南.md` — scheduler_node 接口
* `封控组接入指南.md` — enclosure_node 接口
* `Topic接口设计V2.md` — Topic 设计 V2/V3
* …

`docs/integration/全链路联调手册.md`（杨诗钰维护）— 环境依赖 / YOLOv8 / 启动流程。

---

## 7. 接口对齐（简版 — 完整版见 interface_alignment.md）

```text
tracker_node  ─▶  /target_track      (TargetTrackArray)         ─▶  scheduler_node
tracker_node  ─▶  /enclosure_targets (EnclosureTargetArray)     ─▶  enclosure_node
scheduler_node ─▶ /task_assignment  (TaskAssignment[])         ─▶  planner_stub_node
planner_stub   ─▶ /drone_states      (DroneStateArray)          ─▶  scheduler_node + enclosure_node
enclosure_node ─▶ /enclosure_command (EnclosureCommandArray)    ─▶  控制 / 仿真器
```

**坐标系**：

* `/target_track` `/enclosure_targets`：像素，`frame_id=camera_optical_frame`
* `/drone_states` `/drone_state`：ENU 局部米，`frame_id=world`
* `/task_assignment` `/enclosure_command`：无 header，按 `world` (ENU) 解释坐标

**QoS**：RELIABLE depth=10（除 raw image 为 BEST_EFFORT depth=1）

---

## 8. 已知遗留

* `planning_pkg` 空 → 用 `planner_stub` 占位；程维好上线后替换 launch 中的 `planner_stub_node` 行为 `planner_node`
* YOLO 权重依赖外部路径 → tracker_node 在 CI 中降级到 synthetic publisher

详情见 `docs/integration/three_link_integration.md` § 8。
