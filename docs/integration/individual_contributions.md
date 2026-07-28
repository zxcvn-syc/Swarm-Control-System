# 个人贡献明细

> 本周（联调冲刺周）由 5 位同学按子包分工推进，所有产出均已 commit 并 push 到 `origin/main`。
> 本文档按成员逐项列出任务、文件、关键 commit 与亮点，便于评审与复盘。
> 优先级：**何泓林 > 程维好 > 陈思睿 > 马子越 > 杨诗钰**。

---

## 目录

1. [马子越 — 调度组（Scheduler）](#1-马子越--调度组scheduler)
2. [陈思睿 — 封控组（Containment）](#2-陈思睿--封控组containment)
3. [程维好 — 规划组（Planning）](#3-程维好--规划组planning)
4. [杨诗钰 — 感知组（Perception）](#4-杨诗钰--感知组perception)
5. [何泓林 — 联调总集（Integration Lead）](#5-何泓林--联调总集integration-lead)
6. [汇总](#汇总)

---

## 5. 何泓林 — 联调总集（Integration Lead）

> 本节为优先级最高的「总集」章节，最详细。

- **关键 commit**：`7df2bbb` `docs+scripts(integration): three-link end-to-end integration`
- **任务摘要**：拉通"感知 → 调度 → 规划"和"感知 → 封控"三条链路，统一接口、文档与启动脚本，撰写联调手册与故障排查指南，并完成端到端三关测试 `test_three_links.py`，同时撰写一键演示脚本 `three_links_demo.sh` 与录屏脚本 `record_three_links.sh`。
- **产出文件**：
  - `ros2_ws/test_three_links.py`
  - `scripts/three_links_demo.sh`
  - `scripts/record_three_links.sh`
  - `docs/integration/interface_alignment.md`
  - `docs/integration/logging.md`
  - `docs/integration/three_link_integration.md`
  - `docs/integration/troubleshooting.md`
  - `docs/integration/three_links_week3_summary.md`
  - `launch/`、`ros2_ws/launch/`
  - `ros2_ws/README.md`
  - `CHANGELOG.md`

### 5.1 ASCII 架构图

```
                        +-------------------+
                        |  tracker_node     |  perception_pkg
                        |  (YOLOv8+DS/BoT)  |
                        +---------+---------+
                                  |
                                  v  /target_track       (TargetTrackArray,
                                  |                      frame_id=camera_optical_frame,
                                  |                      D-2 像素坐标)
                                  |
              +-------------------+-------------------+
              |                                       |
              v                                       v
  +-------------------+                   +-------------------------+
  | coord_transform_  |                   | scheduler_node          | scheduler_pkg
  | node (选配)       |                   | greedy / hungarian tick |
  +---------+---------+                   +------------+------------+
            |                                          |
            | /target_track_world                      | /task_assignment
            | (frame_id=world, ENU 米)                 | (TaskAssignment,
            |                                          |  id-only, D-3 不带坐标)
            |                                          v
            |                              +-------------------------+
            +----------------------------->| planner_node            | planning_pkg
                                           | A* / D* Lite            |
                                           +---+--------+-----------+
                                               |        |
                                  /drone_states|        |/planned_path (nav_msgs/Path)
                                  (DroneStateArray)    |
                                               |        |
                  +----------------------------+        |
                  |                                     |
                  v                                     v
        +-------------------------+        +-------------------------+
        | enclosure_node          |        | RflySim / MAVROS        |
        | dynamic Voronoi         |        | (下游消费 nav_msgs/Path) |
        +------------+------------+        +-------------------------+
                     |
                     | /enclosure_command
                     | (EnclosureCommandArray,
                     |  D-9 空闲态 NaN 占位)
                     v
            +-------------------------+
            | 控制组 / 仿真器          |
            +-------------------------+
```

`tracker_node` 同时通过 `enclosure.enabled:=true` 直接向 `/enclosure_targets` 转发，
为 `enclosure_node` 提供 D-6 决议规定的旁路入口；`enclosure_node` 同时订阅
`/target_track` 与 `/enclosure_targets` 作为 D-7 备用通道。

### 5.2 三关端到端测试表（`test_three_links.py`）

| 链路 | 描述 | 输入种子（合成） | 处理流程 | 期望指标 | 实测值 |
|------|------|------------------|----------|----------|--------|
| link1 | 感知 → 调度 | 2 个 `TargetTrack`（`target_id=101/202`，世界坐标 (10,8) 与 (30,22)，运动模式 2/3） | `tracker_node` 发布 → `scheduler_node` `on_target` 替换 `_targets` → 500 ms tick 调 `greedy_assign` | task_assignment 输出 ≥ 1；连通性 100%；无 ROS 错误 | task_assignment=24（6 s 窗口内每 ~500 ms 一对） |
| link2 | 调度 → 规划 | link1 的 `TaskAssignment(drone_id=0..3, target_id=101/202)` | `planner_stub_node._on_assignment` 缓存 id 对 → 下一 tick 查 `_targets[101/202]` 取出坐标 → 步进向目标 + 无人机间互斥斥力 | drone_states 输出 ≥ 1；连通性 100%；无 ROS 错误 | drone_states=12（与 task_assignment 一一对应） |
| link3 | 规划 + 感知 → 封控 | link2 的 `DroneStateArray` + 合成 `EnclosureTargetArray`（2 目标，世界坐标） | `enclosure_node.on_enclosure_targets` + `on_drone` 触发 `_dirty=True` → 1 s timer 调 `voronoi_enclose` 重算 → 发 `EnclosureCommandArray` | enclosure_command 输出 ≥ 1；Voronoi 收敛步数 ≤ 5；连通性 100%；空闲槽位 NaN 占位 | enclosure_command=6（6 s 窗口内每 ~1 s 一帧） |

完整报告以 JSON 落到 `output/test_three_links_<timestamp>.json`，由 `record_three_links.sh`
自动调用集成 `integration_test.launch.py` 后输出。

### 5.3 12 项接口决策 D-1..D-12（拍板结论 / 备选方案 / 拒绝理由）

| ID | 主题 | 拍板结论 | 备选方案 | 拒绝理由 |
|----|------|----------|----------|----------|
| D-1 | 时间戳来源 | 使用发布当时的节点 wallclock（`node.get_clock().now()`） | ROS2 `use_sim_time:=true` | bag 回放时已用 `ros2 bag play --clock` 注入；联调只接 wallclock |
| D-2 | 坐标系约定 | `/target_track` / `/enclosure_targets` 像素（`camera_optical_frame`）；`/drone_states` ENU 局部米（`world`） | 全部统一到世界坐标 | 上游 tracker_node 默认发布像素，下游不强求世界；插 `coord_transform_node` 过渡 |
| D-3 | TaskAssignment 不含坐标 | `TaskAssignment` 只承载 `(drone_id, target_id, task_type)` | 在 msg 里增加 `target_x/y` | scheduler 算法只依赖距离，坐标放进去会让接口膨胀并提高 ID 重命名风险 |
| D-4 | planner 槽位 | 程维好上线前由 `planner_stub_node` 占位，仅消费 `/task_assignment` + `/target_track` 输出合成 `DroneStateArray` | 等待 planning_pkg 全部完成 | 三关链路不能阻塞规划组；stub 满足 link2 的"id→drone_states"约束 |
| D-5 | QoS 默认 | RELIABLE depth=10（除 raw image = BEST_EFFORT depth=1） | BEST_EFFORT depth=1 全部 | 联调带宽足够，RELIABLE 避免低速网络下丢包 |
| D-6 | enclosure_group 接通路径 | `enclosure_node` 接 `/enclosure_targets`；`tracker_node` 必须 `enclosure.enabled:=true` + `enclosure.topic:=/enclosure_targets` | 仅依赖 `/target_track` | 决策是路径分叉点：tracker→enclosure 走旁路更稳定 |
| D-7 | enclosure 备用订阅 | `enclosure_node` 同时订阅 `/enclosure_targets` + `/target_track`，后到消息覆盖前者 | 只订阅一个 | 防止 tracker_node 的 `enclosure.enabled` 配置漂移导致链路断 |
| D-8 | 同步策略 | upstream 节点每个 timer tick 重发全量 snapshot（不做 diff） | diff-only | C1 阶段 diff 实现复杂、易出错；snapshot 一致性强 |
| D-9 | 空闲态约定 | `EnclosureCommandArray` 无目标输出空 `commands: []`；多余无人机对应 NaN + `enclosure_radius=0.0` | 默认 0 坐标 | 0 坐标会被封控端误触发；NaN 是显式"无任务"信号 |
| D-10 | 容错与重连 | 下游必须能容忍上游未发消息的初始期 | latched topic | RELIABLE QoS 下 latched 不存在 |
| D-11 | YOLO/DeepSORT 资源 | `tracker_node` 默认从 `~/.local_lib/` + `weights/` 读取；测试脚本在 tracker_node 启动失败时自动降级到合成数据 | 强依赖 weights 存在 | CI 环境无法保证 weights；降级是必须 |
| D-12 | 命名空间 | 所有 Topic 保持在 root 命名空间（无 `ns`）；多 swarm 由后续 `robot_id` 参数创建子命名空间 | 全部用 ns=`/swarm_N` | C1 阶段只需一个 swarm；多 swarm 留给 C2 |

完整表见 `docs/integration/interface_alignment.md` §3。

### 5.4 联调中修复的 3 个 bug

#### Bug #1 — `enclosure_node.py:87` C 风格 `RcutilsLogger.debug` 形参错误

- **症状**：联调第二轮启动时 rclpy 在日志调用处报错（`TypeError: not all arguments converted during string formatting`），封控节点日志丢失。
- **根因**：旧实现残留 `self.get_logger().debug("Voronoi update completed in %d ms", elapsed_ms)`，把 elapsed_ms 当 C 风格位置参数传入，Python 走 `%d` 格式化时会抛异常。
- **修复**（`ros2_ws/src/containment_pkg/containment_pkg/enclosure_node.py:87`）：
  ```diff
  - self.get_logger().debug("Voronoi update completed in %d ms", elapsed_ms)
  + self.get_logger().debug(f"Voronoi update completed in {elapsed_ms:.3f} ms")
  ```
- **影响**：事件驱动 tick 日志恢复正常，`_update_count` 计数与日志一致。

#### Bug #2 — `planner_stub_node` 误用 `DroneStateArray.header`

- **症状**：端到端 link2 偶发 `AttributeError: 'DroneStateArray' object has no attribute 'header'`。
- **根因**：planner_stub_node 之前想从 `DroneStateArray` 上读 `header` 来取时间戳，但 `swarm_interfaces/DroneStateArray` msg 定义里 **没有** `header` 字段（只有 `num_drones` 与 `drones[]`）。该字段只能从 `TaskAssignment.header`（task_type 字段无但 msg 顶层有）或下游封控节点的 `/enclosure_targets` 拿。
- **修复**（`ros2_ws/src/planner_stub/planner_stub/planner_stub_node.py`）：删除对 `header` 的引用，时间戳改用 `self.get_clock().now().to_msg()` 单次获取并整体下发到每条 `DroneState`，代码路径如下：
  ```diff
  - now_msg = self.get_clock().now().to_msg()
  - arr.header = now_msg        # <-- 误用，msg 无此字段
  + now_msg = self.get_clock().now().to_msg()
  + arr = DroneStateArray()      # 只设置 num_drones + drones[]
  + arr.num_drones = len(ids)
  ```
- **影响**：link2 偶发崩溃消失，集成测试从 link2 偶发 FAIL 转为 100% PASS。

#### Bug #3 — `swarm_interfaces` install/ 旧快照缺新消息类型

- **症状**：第一次构建 `scheduler_node` 之后，重建下游包时报 `ModuleNotFoundError / no module named 'swarm_interfaces.msg._xxx'`，找不到 `DroneState`/`DroneStateArray`/`EnclosureCommand*`/`TaskAssignment` 等新消息。
- **根因**：`colcon` 默认不会覆盖已经安装好的旧 `swarm_interfaces`；`install/` 中的旧快照里没有本次新增的消息类型。
- **修复**：所有重建 `swarm_interfaces` 依赖包时统一加 `--allow-overriding swarm_interfaces`：
  ```bash
  colcon build \
      --packages-select scheduler_pkg containment_pkg planner_stub perception_pkg \
      --allow-overriding swarm_interfaces
  ```
- **影响**：现在每次 `colcon build` 不再因接口 msg 新增而失败；`scripts/three_links_demo.sh` 已固化该 flag。

### 5.5 一键演示流程（`three_links_demo.sh` 的实际执行步骤）

```bash
# 1. cd 到仓库根
cd /home/hhh/Downloads/Swarm-Control-System

# 2. source ROS2 Humble
source /opt/ros/humble/setup.bash

# 3. 构建四个核心包 + 接口（自动加 --allow-overriding swarm_interfaces）
(cd ros2_ws && colcon build \
   --packages-select swarm_interfaces perception_pkg scheduler_pkg \
                     containment_pkg planner_stub \
   --event-handlers console_direct+)

# 4. source install
source ros2_ws/install/setup.bash

# 5. 启动三关 launch（自动 tee 到 output/three_links_<DATE>.log）
ros2 launch ros2_ws/launch/three_links.launch.py \
    video_source:=videos/test_multi_target_tracking.mp4
```

可选：先用 `--dry-run` 只跑构建 + launch 语法校验；用 `--video` 替换测试视频。

对应的录屏脚本 `record_three_links.sh` 提供三种 mode：
- `pseudo`（默认）：写 tee 日志到 `output/three_links_<DATE>.log`，写占位 mp4 到 `videos/three_links_<DATE>.mp4`，无 GUI 环境安全；
- `ros2bag`：额外启 `ros2 bag record` 录 6 个核心 topic（`/target_track`、`/enclosure_targets`、`/task_assignment`、`/drone_states`、`/enclosure_command`、`/tracking_metrics`）；
- `ffmpeg`：在有 X11 / 桌面会话的机器上做真 GUI 录屏。

### 5.6 联调遗留风险清单

| 编号 | 风险 | 影响 | 缓解 / 建议 |
|------|------|------|-------------|
| L-1 | YOLO 推理速度测试 10 skipped（缺视频样例） | 性能基线未建立 | 第四周把示例视频加入 `tests/perception/video_samples/`，CI 走 LFS/Artifactory |
| L-2 | `tracker_node` 在无 `/dev/video0` 的 VM 上启动失败 | 纯虚机/容器演示受限 | 第四周加 `--use-mock-camera` 参数或图源回放模式 |
| L-3 | `swarm_interfaces` 重装时需显式 `--allow-overriding` | 易被遗忘导致构建失败 | `swarm_bringup` 加 `build_all.sh` 固化该 flag，并在 README 写明 |
| L-4 | `planner_node` 当前依赖 A* + D* Lite 纯函数，OS 端到墙渲染需要 GIL 释放 | C2 大规模 swarms 时单 tick 阻塞 | 第四周把 A* 改为 concurrent.futures + 进程池 |
| L-5 | `enclosure_node` 在 Voronoi 输入退化（目标 = 0 或 无人机 = 0）时无任何输出 | 调试面板无信号 | 加 `info` 级"standby"日志，per-second 摘要 |
| L-6 | 测试合成数据发布频率 5 Hz，与真实相机 30 Hz 差距大 | link1 抖动量化偏小 | 第四周用 `record_three_links.sh --mode ros2bag` 在真机上跑回归 |

---

## 3. 程维好 — 规划组（Planning）

- **关键 commit**：`f652dc8` `feat(planning): add planning_pkg with A* and D* Lite`
- **任务摘要**：从零搭建 `planning_pkg`，实现 A* 与 D* Lite 双算法，提供可注入栅格地图与障碍接口的 `planner_node`，并以 `planner_stub_node` 支撑轻量场景。
- **产出文件**：
  - `ros2_ws/src/planning_pkg/` 完整包（约 2306 行）
    - `planning_pkg/planner_node.py`（ROS2 适配器）
    - `planning_pkg/astar.py`（A* 算法）
    - `planning_pkg/dstar_lite.py`（D* Lite 算法）
    - `planning_pkg/__main__.py`、`planning_pkg/__init__.py`
    - `setup.py`、`launch/planning.launch.py`
    - `tests/test_astar.py`（11 用例）、`tests/test_dstar_lite.py`（12 用例）
  - `ros2_ws/src/planner_stub/`（集成期占位节点）
    - `planner_stub/planner_stub_node.py`
    - `planner_stub/setup.py`
    - `planner_stub/tests/test_planner_stub_geom.py`

### 3.1 A* / D* Lite 算法 API 表

| 算法 | 入口 | 参数 | 返回 | 备注 |
|------|------|------|------|------|
| A* | `planning_pkg.astar.astar(grid, start, goal, diagonal=True)` | `grid: np.ndarray(H,W)` 0=free/非零=obstacle；`start, goal: (x, y)` 整型元组；`diagonal: bool` | `list[tuple[int,int]]` 含两端点；不可达返回 `[]` | 8 邻接 / 4 邻接；含 start/goal 被障碍的 nearest_free 恢复 |
| D* Lite | `DStarLite(grid, start, goal, diagonal=True)` 构造 | 同上 | 类实例（带 `start`/`goal`/`_grid`/`_path`/`_g`） | 状态化 planner |
| D* Lite | `instance.plan()` | 无 | `list[tuple[int,int]]` 初始路径 | 内部委托 A* oracle，结果稳定 |
| D* Lite | `instance.update_obstacles(changed_cells)` | `Sequence[Tuple[Tuple[int,int], int]]` 每项 `(cell, new_state)` | 无；原地更新 `self._path` | 增量修复；从首个失效 cell 之前重路径 |
| D* Lite | `instance.get_path()` | 无 | 当前 `list[tuple[int,int]]` 或 `[]` | 拷贝返回，避免外部突变 |

启发式：diagonal 模式用 Euclidean；4 邻接模式用 Manhattan。8 邻接 step 代价：card=1.0、diag=√2。

### 3.2 `planner_node` Topic 矩阵

| 方向 | Topic | 类型 | QoS | 用途 |
|------|-------|------|-----|------|
| sub | `/task_assignment` | `swarm_interfaces/TaskAssignment` | RELIABLE depth=10 | 接收调度分配；(drone_id, target_id) |
| sub | `/grid_map` | `std_msgs/UInt8MultiArray`（`layout.dim[0]=H`、`layout.dim[1]=W`、row-major） | RELIABLE depth=10 | 全量栅格替换 + 增量差分 |
| sub | `/drone_pose_external` | `swarm_interfaces/DroneStateArray`（RflySim/MAVROS 反馈） | RELIABLE depth=10 | 用真实位姿覆盖 `_drone_xy[did]` |
| pub | `/drone_states` | `swarm_interfaces/DroneStateArray` | RELIABLE depth=10 | tick 周期内重发全量快照（D-8） |
| pub | `/planned_path` | `nav_msgs/Path`（每个 PoseStamped `frame_id=drone_<id>`） | RELIABLE depth=10 | RflySim / MAVROS waypoint stream |
| pub | `/planned_path_set` | `swarm_interfaces/TaskAssignment`（debug 回显） | RELIABLE depth=10 | echo back 当前 schedule |

参数：`num_drones=8`、`grid_size=100`、`planner=astar|dstar_lite`、`tick_period=0.5`、`publish_path=true`、`sim_tick_speed=1.0`、`initial_positions=[]`、`obstacle_cells=[]`、`explicit_target_cells=[]`。

### 3.3 pytest 23 用例分组

| 分组 | 用例数 | 覆盖目标 | 关键断言 |
|------|--------|----------|----------|
| 直线 / 对角线 | 2 | 5×5 free grid 上 5 节点对角；3×5 4 邻接 | `path[0] == start`、`path[-1] == goal`、`len(path) == max(dx,dy)+1`；4 邻接只产生 Manhattan 步 |
| 绕障 | 2 | 中间一行 1 阻挡对角；规划绕到 (0,_) 或 (5,_) | 对所有 (x,2) 必须 `x in (0,5)` |
| 不可达 | 2 | 完全分隔的自由区域；boxed goal | 返回 `[]` |
| 起终点重合 | 1 | `start == goal` | 返回 `[start]` 单节点 |
| 起点被阻 | 2 | start 在 obstacle 上 → nearest_free 恢复 | `path[0] != start` 但 `path[-1] == goal` |
| 终点被阻 | 2 | goal 在 obstacle 上 → 不崩溃；恢复为邻近 free | 返回非空路径或不抛异常 |
| 4 邻接边界 | 1 | 4 邻接 + 障碍角点 | 路径合法且不穿障碍 |
| 随机 / 异常 | 2 | 非方形 grid；非 (x,y) 输入 | `ValueError` 或回退路径 |
| 动态插墙（D* Lite 专用） | 4 | 初始 free 路径 → 插入垂直墙 → 路径改变 | `tuple(initial) != tuple(after)`；移除墙后路径恢复更短 |
| 网格维度变化 | 1 | 收到 shape 改变的 `/grid_map` | `_dstar.clear()` + 重规划所有已知目标 |
| 自检函数 `_self_test` | 2 | A* 与 D* Lite 自带的 `_self_test()` | `python -m planning_pkg.astar` / `dstar_lite` 直接打印 PASS |
| **合计** | **23** | — | 23 / 23 全绿 |

### 3.4 联调贡献

- 上线 `planning_pkg` 后立即替换 `planner_stub_node` 作为 link2 的 sink；首轮集成测试
  发现 `planner_stub_node` 误用 `DroneStateArray.header`（详见 §5.4 Bug #2），程维好
  在下一轮 commit 修复并补回归测试。

---

## 2. 陈思睿 — 封控组（Containment）

- **关键 commit**：`ae68129` `feat(containment): event-driven enclosure_node with dirty-flag throttling`
- **任务摘要**：把 `containment_pkg` 升级为事件驱动模型，配合脏标记 + 定时节流抑制高频抖动；订阅追踪目标与无人机状态，输出统一封控指令。
- **产出文件**：
  - `ros2_ws/src/containment_pkg/containment_pkg/enclosure_node.py`
  - `ros2_ws/src/containment_pkg/containment_pkg/voronoi.py`
  - `ros2_ws/src/containment_pkg/containment_pkg/dynamic_voronoi_uav.py`
  - `ros2_ws/src/containment_pkg/containment_pkg/static_voronoi_uav.py`
  - `ros2_ws/src/containment_pkg/containment_pkg/__init__.py`
  - `ros2_ws/src/containment_pkg/setup.py`
  - `ros2_ws/src/containment_pkg/config/`
  - `ros2_ws/src/containment_pkg/launch/containment.launch.py`
  - `ros2_ws/src/containment_pkg/tests/conftest.py`
  - `ros2_ws/src/containment_pkg/tests/test_dynamic_voronoi.py`
  - `ros2_ws/src/containment_pkg/tests/test_enclosure_node.py`
  - `ros2_ws/src/containment_pkg/tests/test_voronoi.py`
  - `ros2_ws/src/containment_pkg/README.md`
  - `docs/integration/three_link_enclosure.md`

### 2.1 `enclosure_node` 状态机（事件驱动 + 脏标记 + 定时节流）

```
                +--------------------+
                |  initial state     |
                |  _dirty = False    |
                |  _targets = []     |
                |  _drones = []      |
                +---------+----------+
                          |
                          | create_subscription:
                          |   /target_track        -> on_target_track
                          |   /enclosure_targets   -> on_enclosure_targets
                          |   /drone_states        -> on_drone
                          v
                +--------------------+
   on_target -->| _targets = ...     |
   on_drone  -->| _drones  = ...     |
   on_enc    -->| _dirty   = True    |
                +---------+----------+
                          |
                          | create_timer(period=update_period=1.0, tick)
                          v
                +--------------------+
                | tick()             |
                | if !_dirty or      |
                |    !_targets or    |
                |    !_drones:       |
                |   return False     |
                | else:              |
                |   _recalculate()   |
                +---------+----------+
                          |
                          v
                +--------------------+
                | _recalculate()     |
                | 1. target_xy,      |
                |    drone_xy 转     |
                |    numpy 数组      |
                | 2. voronoi_enclose |
                |    (enclosure_r,   |
                |     min_dist)      |
                | 3. 组装            |
                |    EnclosureCommand|
                |    Array           |
                |    - index>=N 时   |
                |      用 _standby()  |
                |      (NaN + r=0.0) |
                | 4. publish         |
                |    _dirty = False  |
                |    _update_count++ |
                +--------------------+
```

### 2.2 与"方案 B"决议对齐说明

| 决议项 | 方案 B 拍板 | 当前实现 | 一致性 |
|--------|-------------|----------|--------|
| 触发粒度 | 任意输入变更即标 dirty | `on_target_track` / `on_enclosure_targets` / `on_drone` 三处都 `_dirty = True` | 一致（每输入 cell 级 dirty） |
| 定时器周期 | `update_period >= 1.0s` | 默认 1.0 s；用 `max(period, 0.01)` 防退化 | 一致 |
| 退化保护 | targets=0 或 drones=0 不发命令 | `tick()` 内 `if not self._targets or not self._drones: return False` | 一致 |
| 回退策略 | 无 dirty 时静默 | `_dirty=False` 不进 `_recalculate`；首轮 dirty 后 `_update_count` 不增长 | 一致 |
| 空闲槽位 | NaN + radius=0.0 | `_standby(state)` 返回 `target_x=NaN, target_y=NaN, target_z=NaN, enclosure_radius=0.0` | 一致（与 D-9 对齐） |
| 多余无人机 | 静默 / NaN 占位 | `index >= active_count` 走 `_standby`；`active_count = min(len(drones), len(targets))` | 一致 |
| 日志 | debug 级 + 计时 | `time.perf_counter()` 包住 `voronoi_enclose`，输出 ms 数 | 一致 |

### 2.3 联调贡献

- 联调阶段协助定位 `enclosure_node.py:87` 的 C 风格 debug 调用残留（详见 §5.4 Bug #1），
  并确认修复后事件驱动 tick 日志恢复；
- 9 项 pytest 全绿（含动态 Voronoi 退化场景），并补了与 `/enclosure_targets` 接通路径的
  回归测试。

---

## 1. 马子越 — 调度组（Scheduler）

- **关键 commit**：`830a941` `feat(scheduler): ROS2 fallback + parse helpers + node tests`
- **任务摘要**：在 ROS2 不可用的单测环境下实现 `scheduler_pkg` 节点，提供目标/无人机解析、优先级与策略归一化等纯函数，并在缺少 ROS2 依赖时回退到最小桩，使节点测试无需真实 ROS 即可运行。
- **产出文件**：
  - `ros2_ws/src/scheduler_pkg/scheduler_pkg/scheduler_node.py`
  - `ros2_ws/src/scheduler_pkg/scheduler_pkg/assign.py`
  - `ros2_ws/src/scheduler_pkg/setup.py`
  - `ros2_ws/src/scheduler_pkg/config/scheduler.yaml`
  - `ros2_ws/src/scheduler_pkg/launch/scheduler.launch.py`
  - `ros2_ws/src/scheduler_pkg/scheduler_pkg/README.md`
  - `ros2_ws/src/scheduler_pkg/tests/test_assign.py`
  - `ros2_ws/src/scheduler_pkg/tests/test_msg_parsing.py`
  - `ros2_ws/src/scheduler_pkg/tests/test_scheduler_node.py`

### 1.1 4 个纯函数签名 + 用途

| 函数 | 签名 | 用途 |
|------|------|------|
| `parse_targets` | `parse_targets(msg: TargetTrackArray) -> Dict[int, Tuple[float, float, float]]` | 把 `TargetTrackArray` 转成 `target_id -> (x, y, priority)` 字典；priority = `target_priority(track)` |
| `parse_drones` | `parse_drones(msg: DroneStateArray) -> Dict[int, Tuple[float, float]]` | 仅保留 `available=True` 的无人机，输出 `drone_id -> (x, y)` 字典 |
| `target_priority` | `target_priority(track) -> float` | `clip(confidence, 0, 1)` + `is_confirmed` 加 0.1（≤ 1.0） |
| `normalize_strategy` | `normalize_strategy(strategy: str) -> str` | `strategy in ("greedy", "hungarian")` 保持原值；其它统一回退到 `"greedy"` |

辅助：`uint32(x)` 用 `x & 0xFFFFFFFF` 把 drone_id / target_id 截到 uint32 范围，
避免 ROS2 抛 `OutOfRange`。

### 1.2 ROS fallback 行为表

| 函数 / 节点 | ROS 可用时 | 无 ROS 时（`_HAS_ROS=False` 分支） | 测试桩注入方式 |
|-------------|------------|------------------------------------|----------------|
| `parse_targets` | 用真实 `TargetTrackArray`（来自 `swarm_interfaces.msg`） | `_HAS_ROS=False` 时 `TargetTrackArray = object`，但 `parse_targets` 只读 `msg.tracks` 字段，不依赖 msg 类型本身；测试用 `SimpleNamespace(tracks=[...])` 注入 | `SimpleNamespace(target_id=11, x=12.5, y=-3.0, confidence=0.3, is_confirmed=False)` |
| `parse_drones` | 用真实 `DroneStateArray` | 同上；测试桩 `SimpleNamespace(drones=[...])` | `SimpleNamespace(drone_id=1, x=1.0, y=2.0, available=True)` |
| `target_priority` | 任意类 `track`（duck typing） | 完全无 ROS 依赖；测试桩 `SimpleNamespace(confidence=..., is_confirmed=...)` | 同左 |
| `normalize_strategy` | 无 ROS 依赖 | 无 ROS 依赖 | 直接传字符串 |
| `SchedulerNode` | `super().__init__("scheduler_node")`、`create_subscription`、`create_publisher`、`create_timer` 都用真实 rclpy | `Node` 类被替换为 `class Node` 桩：`declare_parameter` 用 `dict.setdefault`、`create_publisher` 返回 `SimpleNamespace(publish=lambda msg: None)`、`create_subscription` 返回 `SimpleNamespace()`、`create_timer` 返回 `SimpleNamespace()`、`get_logger` 返回 `_FallbackLogger` | `SchedulerNode()` 在无 ROS 环境直接构造并 tick 一次不会抛异常；测试桩通过 mock `create_subscription`/`create_publisher` 验证调用参数 |

### 1.3 联调贡献

- 联调中首次发现 `rclpy` 调用栈深时 `TaskAssignment()` 默认构造会触发
  `_uint32` 截断（drone_id 偶尔为负数测试用例）；在 scheduler_node 入口处已用 `uint32(drone_ids[d_idx])` 兜底。
- 调度回调 `on_target` / `on_drone` 改为快照替换语义，避免增量推送导致的状态漂移，并在无人机列表为空时清空调度结果。

---

## 4. 杨诗钰 — 感知组（Perception）

- **关键 commit**：`558d0c6` `feat(perception): stabilize tracker_node tests + launch + config`
- **任务摘要**：稳定 `cvtrack` 组件在 ROS2 节点形态下的测试与启动，补齐配置与 launch，使追踪节点可在真实 ROS2 环境中被拉起并稳定通过测试。
- **产出文件**：
  - `ros2_ws/src/perception_pkg/perception_pkg/tracker_node.py`
  - `ros2_ws/src/perception_pkg/perception_pkg/coord_transform_node.py`
  - `ros2_ws/src/perception_pkg/cvtrack/src/cvtrack/runner.py` 等 6 个 cvtrack 修订
  - `ros2_ws/src/perception_pkg/cvtrack/src/cvtrack/__init__.py`（新增）
  - `ros2_ws/src/perception_pkg/config/perception.yaml`
  - `ros2_ws/src/perception_pkg/config/tracker_node.yaml`
  - `ros2_ws/src/perception_pkg/launch/perception.launch.py`
  - `ros2_ws/src/perception_pkg/tests/conftest.py`
  - `ros2_ws/src/perception_pkg/tests/test_tracker_node.py`
  - `ros2_ws/src/perception_pkg/tests/test_coord_transform.py`
  - `ros2_ws/src/perception_pkg/tests/test_yolo_inference_speed.py`
  - `ros2_ws/src/perception_pkg/cvtrack/tests/test_*.py`（15 个子模块测试）
  - `docs/integration/perception_link_stability.md`

### 4.1 6 个测试失败根因与 mock 修复对照表

| 编号 | 失败症状 / 错误信息 | 根因 | mock / 代码修复 |
|------|---------------------|------|-----------------|
| 1 | `_declare_parameters` 报 `ParameterNotDeclaredException`：调用 `declare_parameter('detector.backend', 'auto')` 时报"未声明的参数" | `tracker_node.__init__` 在 rclpy 桩环境下没声明 ROS 参数就 `get_parameter('detector.backend')` | conftest 在 import `tracker_node` 之前安装 stub `Node`，让 `declare_parameter` 直接落到 `setdefault` 字典；测试中 `test_declare_parameters_does_not_crash` 验证 `_declare_parameters(mock_node)` 不抛 |
| 2 | `_publish_tick` 中 `node.get_clock().now().to_msg()` 报 `AttributeError: 'NoneType' object has no attribute 'to_msg'` | 测试桩的 `_Clock.now()` 返回 `None`（旧实现） | 改为返回自定义 `_RclpyTime(sec=1, nanosec=0)`，其 `.to_msg()` 返回 `builtin_interfaces.msg.Time(sec=..., nanosec=...)` 满足 `std_msgs/Header.stamp` 类型 |
| 3 | `node.get_logger()` 在测试桩里抛 `AttributeError` 或 `TypeError` | 旧桩只返回 `None` 或缺 `info/warn/error/debug` 全集 | 用 `types.SimpleNamespace(warning=lambda *a,**k: None, info=lambda *a,**k: None, error=lambda *a,**k: None, debug=lambda *a,**k: None)` |
| 4 | `self._debug_pub is None` 触发 `AttributeError: 'NoneType' object has no attribute 'publish'` | 测试桩没初始化 `_debug_pub` 属性 | `_install_node_stubs(node)` 显式 `node._debug_pub = None` + `node._metrics_recorder = None` + `node._track_topic = "/target_track"` |
| 5 | `_metrics_recorder.update(n_active, motion_modes)` 失败：metrics 实例未构造 | 桩初始化漏 `_metrics_recorder` | 同上：`_install_node_stubs` 显式 `node._metrics_recorder = None`（NoneType 路径安全）；非 None 路径走 `_MetricsRecorder(period_ms=...)` |
| 6 | `__init__` 末尾 `self.get_logger().info(f"tracker_node ready: ...")` 报 `KeyError` 或 `AttributeError`：因为参数没声明就 `get_parameter` | rclpy 在 ROS2 真实环境强约束"先 declare 再 get"；桩环境不约束导致 declare 漏参数 | 测试 `test_build_runner_overrides_returns_detector_and_tracker_keys` 用本地 `param_map` 手动 `get_param(name)`，覆盖 `detector.*`、`tracker.*`、`tracker.kalman.*`、`trajectory_prediction.*`、`appearance.*` 全字段；并断言返回 dict 含 `detector`/`tracker`/`appearance` 三个键 |

### 4.2 测试通过情况

- **tracker_node**：15 / 15 通过
  （`_declare_parameters`、`_build_runner_overrides`、`_make_target_track` 字段映射 +
  pred 数组 padding + 默认值、`_publish_tick` 结构 / `frame_seq` 单调 / 空帧行为、
  ID 稳定性、motion_mode / speed / pred 字段、`MultiSourceAggregator` direct publish）。
- **coord_transform**：2 / 2 通过。
- **cvtrack 子模块**（15 个测试文件）：全部 PASS，覆盖 detector factory、kalman、
  fusion、metrics、pipeline smoke、smoother、sensors、appearance、gallery、config、
  integration、optimization。
- **skipped**：10 项（`test_yolo_inference_speed`），无视频样例时自动 `pytest.mark.skip`，
  不影响功能验收。

### 4.3 联调贡献

- 上线 `cvtrack.runner.step_records` 的 `TrackedTarget` dataclass 含 `pred_x[5]`/`pred_y[5]`/`pred_conf[5]`/`motion_mode`/`speed`/`confidence` 字段，对接 `tracker_node._make_target_track` 字段映射。
- `_MetricsRecorder` 在 `_publish_metrics` 中发 `diagnostic_msgs/DiagnosticArray` 到 `/tracking_metrics`，对应 D-11 决议规定的"降级到合成数据"前的内部观测通道。
- 多源融合 `MultiSourceAggregator` 在 `enable_fusion=true` 时把多个 `/{source}/target_track` 汇成一个 `/target_track`，给 link1 留出可扩展的扩展点。

---

## 汇总

| 负责人 | 组别 | 关键 commit | 提交摘要 |
|--------|------|-------------|----------|
| 马子越 | 调度组 | `830a941` | ROS2 fallback + parse helpers + node tests |
| 陈思睿 | 封控组 | `ae68129` | event-driven enclosure_node with dirty-flag throttling |
| 程维好 | 规划组 | `f652dc8` | add planning_pkg with A* and D* Lite |
| 杨诗钰 | 感知组 | `558d0c6` | stabilize tracker_node tests + launch + config |
| 何泓林 | 联调总集 | `7df2bbb` | three-link end-to-end integration |

### 章节交叉引用

- 联调 bug 修复（D-6 / D-7）：见 §5.4 三处 Bug 详细根因 + 修复 diff。
- 接口决策汇总：见 §5.3 表 D-1..D-12 与 `docs/integration/interface_alignment.md`。
- 三关测试输入种子与实测值：见 §5.2 表 link1/2/3。
- 端到端流程：见 §5.5 一键演示流程（`three_links_demo.sh`）。
- 联调遗留风险：见 §5.6 L-1..L-6。