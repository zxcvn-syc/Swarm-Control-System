# 接口对齐决议表（Interface Alignment Decisions）

> 负责：何泓林（联调总指挥）
> 制度：**任何分歧现场拍板，写入本表，不可拖延**。
> 本表是 **citable truth** — 任何人修改 Topic 名称、msg 字段、QoS、坐标系 都必须先更新本表，然后再改代码。

---

## 1. Topic 契约（The Topic Contract）

三关链路完整 Topic 表。所有 launch / 脚本都遵守这张表。

### 1.1 主表

| Topic | 类型 | 发布者 | 订阅者 | QoS | frame_id | 备注 |
|-------|------|--------|--------|-----|----------|------|
| `/target_track` | `swarm_interfaces/msg/TargetTrackArray` | `tracker_node` | `scheduler_node` / `planner_stub_node` | RELIABLE depth=10 | `camera_optical_frame`（像素） | 字段定义见 §2.1 |
| `/enclosure_targets` | `swarm_interfaces/msg/EnclosureTargetArray` | `tracker_node`（仅当 `enclosure.enabled:=true`） | `enclosure_node` | RELIABLE depth=5 | `camera_optical_frame` | 字段定义见 §2.2 |
| `/task_assignment` | `swarm_interfaces/msg/TaskAssignment` | `scheduler_node` | `planner_stub_node`（规划) / 仿真器 | RELIABLE depth=10 | 不带 header；坐标系约定为`world` (下游消费时按 ENU 解释) | 每对（drone, target）一条；scheduler tick 全量重发 |
| `/drone_states` | `swarm_interfaces/msg/DroneStateArray` | `planner_stub_node`（规划侧占位）| `scheduler_node` / `enclosure_node` | RELIABLE depth=10 | n/a (`DroneStateArray` 无 header) | ENU 局部坐标米 |
| `/drone_state` | `swarm_interfaces/msg/DroneState` | `planner_stub_node` | （预留订阅） | RELIABLE depth=10 | n/a | 单机版本；`DroneStateArray` 的 per-drone split |
| `/enclosure_command` | `swarm_interfaces/msg/EnclosureCommandArray` | `enclosure_node` | 控制组 / 仿真器 | RELIABLE depth=10 | n/a | 空闲态输出 `target_x=NaN`, `enclosure_radius=0.0` |
| `/target_track_debug` | `swarm_interfaces/msg/TargetTrackDebug` | `tracker_node` | 调试面板 | RELIABLE depth=10 | `camera_optical_frame` | KF 协方差、motion_mode 原因、appearance score |
| `/tracking_metrics` | `diagnostic_msgs/DiagnosticArray` | `tracker_node` | 调试面板 | RELIABLE depth=5 | n/a | 5 项指标 |
| `/target_track_world` | `swarm_interfaces/msg/TargetTrackArray` | `coord_transform_node` | 世界坐标系消费者 | RELIABLE depth=10 | `world` (ENU) | 选配；`coord_transform_node` 上线后再启用 |
| `/camera/image` | `sensor_msgs/Image` | Camera driver / synthetic | `tracker_node`（`input_mode:=topic`） | BEST_EFFORT depth=1 | `camera_optical_frame` | 仅感知用 |

> **QoS 约定**：所有 ROS2 Topic（除 raw image）使用 **RELIABLE depth=10**；raw image 用 BEST_EFFORT depth=1。修改需在这里更新，并同步对应 `create_publisher`/`create_subscription` 的 QoS 参数。

### 1.2 与 ROS2 官方内置消息的对照（不在 swarm_interfaces 里）

| Topic | 类型 | 备注 |
|-------|------|------|
| 内部测试聚合输出 | `std_msgs/msg/String` | 仅 `test_three_links.py` 内部使用，不发布 |

---

## 2. 字段约定

### 2.1 `TargetTrack` 字段（来自 `swarm_interfaces`）

| 字段 | 类型 | 含义 | 单位 / 约定 |
|------|------|------|----------|
| `target_id` | `uint32` | DeepSORT/BoT-SORT ID | 节点内唯一 |
| `x`, `y` | `float64` | 像素坐标（图像平面目标框中心） | 像素 |
| `vx`, `vy` | `float64` | 像素速度 | 像素/秒 |
| `confidence` | `float32` | 检测置信度 | [0.0, 1.0] |
| `cls` | `uint8` | 目标类别 | COCO label |
| `is_confirmed` | `bool` | DeepSORT 已确认 | - |
| `speed` | `float32` | 速度大小 | 像素/秒 |
| `motion_mode` | `uint8` | 0=unknown / 1=stationary / 2=slow / 3=fast | - |
| `pred_x[5]`, `pred_y[5]` | `float32[5]` | 未来 5 步预测位置 | 像素 |
| `pred_conf[5]` | `float32[5]` | 预测置信度 | [0, 1] |

### 2.2 `EnclosureTarget` 字段

继承 `TargetTrack` 的核心字段（target_id/x/y/speed/motion_mode/confidence），并补充：

| 字段 | 类型 | 含义 |
|------|------|------|
| `box_x1`..`box_y2` | `float32` | 包围盒（用于大小参考） |
| `pred_x[5]`, `pred_y[5]` | `float32[5]` | 未来 5 步 |
| `history_x[10]`, `history_y[10]` | `float32[10]` | 最近 10 步历史 |

### 2.3 `TaskAssignment` 字段

```text
uint32 drone_id
uint32 target_id
string task_type
```

> **不携带坐标**。详见决策 **D-3**。

### 2.4 `DroneState` 字段

| 字段 | 类型 | 含义 | 单位 |
|------|------|------|------|
| `drone_id` | `uint32` | 无人机 ID | - |
| `x`, `y`, `z` | `float64` | ENU 局部坐标 | **米** |
| `vx`, `vy`, `vz` | `float64` | 速度 | m/s |
| `available` | `bool` | 当前是否可分配 | - |

### 2.5 `EnclosureCommand` 字段

| 字段 | 类型 | 含义 | 单位 |
|------|------|------|------|
| `drone_id` | `uint32` | 无人机 ID | - |
| `target_x`, `target_y` | `float64` | 期望航点（ENU 局部） | **米** |
| `target_z` | `float32` | 高度 | **米** |
| `enclosure_radius` | `float32` | 有效封控半径 | **米** |

---

## 3. 决议清单（Decisions）

> 每条决议有 ID 与最终拍板时间。推翻一条决议需要新建一条带 `REPLACES D-N` 的新决议。

### D-1（时间戳）
**`header.stamp` 使用发布当时的节点 wallclock（rclpy `node.get_clock().now()`）。**

bag 回放测试时使用 `ros2 bag play --clock` 注入 sim time。

### D-2（坐标系）
| Topic | 坐标系 |
|-------|--------|
| `/target_track` | 像素（图像平面），`frame_id="camera_optical_frame"`（默认） |
| `/enclosure_targets` | 像素，`frame_id="camera_optical_frame"` |
| `/target_track_world`（如果启用）| ENU 局部，`frame_id="world"` |
| `/drone_states`, `/drone_state` | ENU 局部米，`frame_id="world"`（`DroneStateArray` 自身无 header，约定以"world"理解） |
| `/task_assignment`, `/enclosure_command` | 坐标以 `world` (ENU) 解释，没有 header.stamp |

**上游（`tracker_node`）默认发布像素坐标**；下游（`scheduler_node`、`enclosure_node`）当前也消费像素坐标。如果未来 scheduler/enclosure 切到世界坐标，**不修改 `tracker_node`**，而是在中间加 `coord_transform_node` 并把 launch / 配置改成订阅 `/target_track_world`。

### D-3（TaskAssignment 不含坐标）
**`TaskAssignment` 不携带 (x, y, z)**。

原因：scheduler_node 当前的算法只看 drone 和 target 之间的距离；把坐标放进 TaskAssignment 会让接口膨胀并提高 ID-重命名风险。

**planner_node 必须同时订阅 `/task_assignment`（拿 id 对）+ `/target_track`（拿坐标）**。这一点写进了本表，并在 `planner_stub_node.py` 的 `_on_target_track` / `_on_assignment` 注释里固化下来。

### D-4（planner 槽位）
**`planning_pkg` 仍是空的。** 程维好上线前，集成脚本使用 `planner_stub_node`（在 `planner_stub` 包内）作为占位，仅消费 `/task_assignment` + `/target_track` 后输出合成 `DroneStateArray`。

`planner_stub_node` 也发布 `/drone_state`（per-drone topic）以备未来 C2 control 节点使用。

### D-5（QoS 默认）
**RELIABLE depth=10**。所有 ROS2 Topic（除 raw image）遵守这一默认值。 raw image = **BEST_EFFORT depth=1**。

修改任何一条 Topic 的 QoS 必须更新本表 §1.1 + 对应 launch 节点参数。

### D-6（enclosure_group 接通路径）
**陈思睿的 enclosure_node 接收 `/enclosure_targets`**（不再依赖 `/target_track` 直接驱动）。`tracker_node` 必须开启 `enclosure.enabled:=true` + `enclosure.topic:=/enclosure_targets` 才能让第三关跑通。

`three_links.launch.py` 默认开启 `enclosure.enabled:=true`。

### D-7（enclosure_node 同时订阅 /target_track 作为备用）
**为方便过渡，enclosure_node 同时订阅 `/enclosure_targets` 和 `/target_track`**，后到消息覆盖前者。Coordinator 任选其一运行。

### D-8（同步策略）
**所有 upstream 节点在每个 timer tick 重发全量最新 snapshot**（不做 diff）。

* tracker_node：每 ~100 ms 重发 `TargetTrackArray`（已确认 tracks）
* scheduler_node：每 500 ms 重发当前全部 `TaskAssignment`
* planner_stub_node：每 500 ms 重发 `DroneStateArray`
* enclosure_node：仅在 dirty 时或 1 s tick 输出 `EnclosureCommandArray`

C2 / C3 可以改造为 diff-only，但本次联调按全量 snapshot 处理。

### D-9（空闲态 / standby 约定）
`EnclosureCommandArray` 在没有目标时输出空 `commands: []`；在目标少、无人机多的情况下，多余无人机对应 `EnclosureCommand{ target_x=NaN, target_y=NaN, target_z=NaN, enclosure_radius=0.0 }`（**已与陈思睿对齐，见 `docs/interface/封控组接入指南.md`**）。

### D-10（容错与重连）
**QoS RELIABLE 模式下，latched-state 不存在**。下游节点必须能容忍「上游未发消息」的初始期。

* scheduler_node：无 target 时静默一次 info 后保持静默
* planner_stub_node：无 drone 时用 5m 网格 seed
* enclosure_node：无 target 或 drone 时不发布

### D-11（YOLO/DeepSORT 资源）
**tracker_node 默认从 `~/.local_lib/` 与 `weights/` 读取**。 集成测试脚本 `test_three_links.py` 在 `tracker_node` 启动失败时**自动降级到合成数据**，不会阻塞 CI。

### D-12（命名空间 / 多实例）
**所有 Topic 保持在 root 命名空间**（无 `ns` 限定）。多个 swarm 部署时由 `robot_id` 参数（待 C2 引入）创建子命名空间。当前 C1 阶段不需要。

---

## 4. 字段覆盖度（每个下游节点消费上游哪个字段）

| 下游节点 | 上游 Topic | 关注的字段 | 不需要的字段（节约带宽） |
|----------|-----------|----------|------------------------|
| `scheduler_node` | `/target_track` | `target_id, x, y, confidence, is_confirmed` | `vx/vy, pred_*`（C2 用）|
| `scheduler_node` | `/drone_states` | `drone_id, x, y, available` | `z, vx/vy/vz` |
| `planner_stub_node` | `/task_assignment` | `drone_id, target_id, task_type` | - |
| `planner_stub_node` | `/target_track` | `target_id, x, y` | 其它 |
| `enclosure_node` | `/enclosure_targets` | `target_id, x, y, speed, motion_mode, confidence, pred_*` | `history_*` |
| `enclosure_node` | `/target_track` | `target_id, x, y` | 其它（备用） |
| `enclosure_node` | `/drone_states` | `drone_id, x, y, z` | `vx/vy/vz` |

---

## 5. 修改记录（Changelog）

| 日期 | 决议 | 变更 | 影响 |
|------|------|------|------|
| 2026-07-27 | D-1..D-12 | 首次拍板 | 三关链路第一次端到端跑通 |

---

*如对任何一条决议有异议，请在周报中提出，由何泓林现场拍板后再修改本表 + 对应代码。*
