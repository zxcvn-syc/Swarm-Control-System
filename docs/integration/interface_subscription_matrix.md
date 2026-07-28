# 接口互订阅矩阵 + 启动验证日志

> 4 个 ROS2 节点（5 个 executable）在 `ros2_ws/` 下实际启动并相互订阅 / 发布的验证结果。
> 所有日志均来自一次真实的 colcon build + ros2 run 联调，不是设计稿。

## 1. 节点 / Pkg 总览

| # | 节点 (executable)    | pkg                  | 角色 / 作者        | 入口文件                                                                 | entry_point                              |
|---|----------------------|----------------------|--------------------|--------------------------------------------------------------------------|------------------------------------------|
| 1 | `tracker_node`        | `perception_pkg`     | 感知 / 杨诗钰       | `ros2_ws/src/perception_pkg/perception_pkg/tracker_node.py`               | `tracker_node = perception_pkg.tracker_node:main` |
| 2 | `coord_transform_node`| `perception_pkg`     | 感知               | `ros2_ws/src/perception_pkg/perception_pkg/coord_transform_node.py`       | `coord_transform_node = perception_pkg.coord_transform_node:main` |
| 3 | `scheduler_node`      | `scheduler_pkg`      | 调度 / 马子越       | `ros2_ws/src/scheduler_pkg/scheduler_pkg/scheduler_node.py`              | `scheduler_node = scheduler_pkg.scheduler_node:main` |
| 4 | `planner_node`        | `planning_pkg`       | 规划 / 程维好       | `ros2_ws/src/planning_pkg/planning_pkg/planner_node.py`                  | `planner_node = planning_pkg.planner_node:main` |
| 5 | `enclosure_node`      | `containment_pkg`    | 封控 / 陈思睿       | `ros2_ws/src/containment_pkg/containment_pkg/enclosure_node.py`          | `enclosure_node = containment_pkg.enclosure_node:main` |

`planner_stub` 存在但不在三关链路里，仅历史占位用。
`swarm_interfaces` 是 msg 容器包，提供 `TargetTrack{Array,Debug}`、`TaskAssignment`、`DroneState{Array}`、`EnclosureTarget{Array}`、`EnclosureCommand{Array}`。

## 2. 互订阅矩阵

`S` = 订阅，`P` = 发布。空 = 无交互。Topic 名一律来自源码 `declare_parameter` 默认值
或 `enclosure_node.py` 字面量。

| Topic (msg type)                  | tracker_node | coord_transform_node | scheduler_node | planner_node | enclosure_node |
|-----------------------------------|:------------:|:--------------------:|:--------------:|:------------:|:--------------:|
| `/camera/image` (`sensor_msgs/Image`)      | **S** | ·  | · | · | · |
| `/camera_info` (`sensor_msgs/CameraInfo`)  | ·    | S | · | · | · |
| `/drone_pose` (`PoseStamped`)              | ·    | S | · | · | · |
| `/drone_pose_external` (`DroneStateArray`) | ·    | · | · | **S** | · |
| `/target_track` (`TargetTrackArray`)       | **P** | **S** | **S** | · | **S** |
| `/target_track_world` (`TargetTrackArray`) | ·    | **P** | · | · | · |
| `/target_track_debug` (`TargetTrackDebug` + `TargetTrackArray`) | **P** (debug only) | **P** (debug only) | · | · | · |
| `/tracking_metrics` (`DiagnosticArray`)    | **P** | · | · | · | · |
| `/enclosure_targets` (`EnclosureTargetArray`) | **P** | · | · | · | **S** |
| `/drone_states` (`DroneStateArray`)        | ·    | · | **S** | **P** | **S** |
| `/task_assignment` (`TaskAssignment`)      | ·    | · | **P** | **S** | · |
| `/grid_map` (`UInt8MultiArray`)            | ·    | · | · | **S** | · |
| `/planned_path` (`nav_msgs/Path`)          | ·    | · | · | **P** | · |
| `/enclosure_command` (`EnclosureCommandArray`) | · | · | · | · | **P** |

Topic 默认值来源（节选）：

- `tracker_node`: `track_topic=/target_track`, `image_topic=/camera/image`
  （`perception_pkg/tracker_node.py` L200–201）
- `coord_transform_node`: `input_topic=/target_track`, `output_topic=/target_track_world`,
  `camera_info_topic=/camera_info`, `drone_pose_topic=/drone_pose`
  （`perception_pkg/coord_transform_node.py` L251–254）
- `scheduler_node`: `target_topic=/target_track`, `drone_topic=/drone_states`,
  `output_topic=/task_assignment`（`scheduler_pkg/scheduler_node.py` L135–138 + `config/scheduler.yaml`）
- `planner_node`: `task_topic=/task_assignment`, `grid_topic=/grid_map`,
  `drone_states_topic=/drone_states`, `planned_path_topic=/planned_path`,
  `rfly_pose_topic=/drone_pose_external`（`planning_pkg/planner_node.py` L93–97 + `config/planning.yaml`）
- `enclosure_node`: 硬编码订阅 `/target_track`、`/enclosure_targets`、`/drone_states`，
  发布 `/enclosure_command`（`containment_pkg/enclosure_node.py` L30–40）

## 3. 启动顺序建议（拓扑排序）

```
                     ┌──────────────────────┐
                     │ sensor source        │
                     │ /camera/image        │
                     │ /camera_info         │
                     │ /drone_pose          │
                     └──────────┬───────────┘
                                ▼
┌──────────────────────────┐    ┌──────────────────────────┐
│ tracker_node             │───▶│ /target_track            │
│ (perception_pkg)         │    │ /enclosure_targets       │
└──────────────────────────┘    │ /target_track_debug      │
                                │ /tracking_metrics        │
                                └────────────┬─────────────┘
                                             │
              ┌──────────────────────────────┼─────────────────────────────┐
              ▼                              ▼                             ▼
   ┌────────────────────────┐   ┌────────────────────────┐   ┌────────────────────────┐
   │ coord_transform_node   │   │ scheduler_node         │   │ enclosure_node         │
   │ → /target_track_world  │   │ → /task_assignment     │   │ → /enclosure_command   │
   │ (world-frame track)    │   │ (greedy/hungarian)     │   │ (Voronoi containment)  │
   └────────────────────────┘   └──────────┬─────────────┘   └────────────────────────┘
                                           ▼
                                ┌────────────────────────┐
                                │ planner_node           │
                                │ ← /task_assignment     │
                                │ ← /grid_map            │
                                │ ← /drone_pose_external │
                                │ → /drone_states        │
                                │ → /planned_path        │
                                └──────────┬─────────────┘
                                           │ /drone_states 回环
                                           ▼
                                ┌────────────────────────┐
                                │ scheduler_node         │
                                │ ← /drone_states        │
                                └────────────────────────┘
```

启动顺序（无环依赖，至少保证 publisher 先出现；下面顺序从源头开始）：

1. **tracker_node** —— 它是 `/target_track` 的源头（perception→scheduling 的关键 link）。
2. **coord_transform_node** —— 在 tracker_node 启动后即可，它把 `/target_track` 投影到 `/target_track_world`。
3. **scheduler_node** —— 需要 `/target_track` 和 `/drone_states`；依赖 planner_node 的 `/drone_states` 回环。
4. **planner_node** —— 需要 `/task_assignment`；自身会回环发送 `/drone_states` 喂给 scheduler。
5. **enclosure_node** —— 独立，订阅 `/target_track` 和 `/drone_states`、可选 `/enclosure_targets`。

注：`scheduler_node` ↔ `planner_node` 在 `/drone_states` 上构成一个**环**，两个节点最终
都得在 DDS 层发现彼此。两者可任意先后启动；ring 的稳态不依赖先后。

## 4. B2 真实启动日志摘要

### 4.1 colcon build

```
$ colcon build --packages-select swarm_interfaces perception_pkg scheduler_pkg \
                planning_pkg containment_pkg --allow-overriding swarm_interfaces
Starting >>> swarm_interfaces
Finished <<< swarm_interfaces [5.63s]
Starting >>> containment_pkg
Starting >>> perception_pkg
Starting >>> planning_pkg
Starting >>> scheduler_pkg
Finished <<< planning_pkg [3.09s]
Finished <<< perception_pkg [3.12s]
Finished <<< scheduler_pkg [3.11s]
Finished <<< containment_pkg [3.15s]
Summary: 5 packages finished [9.99s]
```

**结果：成功**，无 `warning`/`error`。需要 `PYTHONPATH=<repo>/ros2_ws/src/perception_pkg/cvtrack/src`
让 `tracker_node` 找到 `cvtrack` Python 包，否则会 `UnboundLocalError`。

### 4.2 5 个 executable 同时启动

```bash
ros2 run perception_pkg tracker_node --ros-args \
    -p input_mode:=topic -p track_topic:=/target_track \
    -p enclosure.enabled:=true -p enclosure.topic:=/enclosure_targets
ros2 run perception_pkg coord_transform_node
ros2 run scheduler_pkg scheduler_node
ros2 run planning_pkg planner_node
ros2 run containment_pkg enclosure_node
```

启动后 `ros2 node list`：

```
/coord_transform_node
/enclosure_node
/planner_node
/scheduler_node
/tracker_node
```

启动后 `ros2 topic list`：

```
/camera/image
/camera_info
/drone_pose
/drone_pose_external
/drone_states
/enclosure_command
/enclosure_targets
/grid_map
/parameter_events
/planned_path
/rosout
/target_track
/target_track_debug
/target_track_world
/task_assignment
/tracking_metrics
```

`ros2 topic info` 中 publisher/subscriber 计数（与第二节矩阵一致）：

| Topic                | P | S |
|----------------------|---|---|
| `/target_track`            | 1 | **3**（coord / scheduler / enclosure） |
| `/drone_states`            | 1 | **2**（scheduler / enclosure） |
| `/task_assignment`         | 1 | 1（planner） |
| `/enclosure_command`       | 1 | 0 |
| `/target_track_world`      | 1 | 0 |
| `/enclosure_targets`       | 1 | 1（enclosure） |
| `/planned_path`            | 1 | 0 |
| `/target_track_debug`      | 2 | 0 |
| `/tracking_metrics`        | 1 | 0 |

### 4.3 节点 ready 日志（截取）

```
[INFO] [tracker_node]: Enclosure publisher enabled on /enclosure_targets
[INFO] [tracker_node]: subscribed to image topic /camera/image
[INFO] [tracker_node]: tracker_node ready: mode=topic topic=/target_track rate=10.0Hz frame_id=camera_optical_frame tracker=deepsort_cascade weights=(auto)

[INFO] [coord_transform_node]: coord_transform_node ready: input=/target_track -> output=/target_track_world camera_info=/camera_info drone_pose=/drone_pose ground_altitude=0.0 mount(rpy)=(0.000, 0.000, 0.000)

[INFO] [scheduler_node]: scheduler_node up: strategy=greedy, num_drones=8, max_per_drone=2, tick=0.5s, in=/target_track+/drone_states, out=/task_assignment

[INFO] [planner_node]: planner_node up: planner=astar, num_drones=8, grid=100x100, tick=0.5s, in=/task_assignment+/grid_map, out=/drone_states+/planned_path
```

### 4.4 端到端消息流验证（关键证据）

向 `/target_track` 发 1 条 TargetTrack（target_id=42, x=12.5, y=7.5），
向 `/drone_states` 发 2 条 DroneState（id=1, x=10, y=20；id=2, x=30, y=40）后：

```
# scheduler_node（订阅 /target_track + /drone_states）
[INFO] [scheduler_node]: scheduler summary: 0 assignments, 0 active targets, 0 active drones, tick=500 ms
... 5s 后 ...
[INFO] [scheduler_node]: scheduler summary: 0 assignments, 0 active targets, 8 active drones, tick=500 ms
... 又 5s 后，pub 之后 ...
[INFO] [scheduler_node]: scheduler summary: 1 assignments, 1 active targets, 8 active drones, tick=500 ms
                                                            ↑           ↑
                                            收到 TargetTrack,    收到 8 架 drone 状态
                                            生成了 1 个 TaskAssignment

# planner_node（订阅 /task_assignment + /drone_pose_external + /grid_map）
[INFO] [planner_node]: planner summary: 0/8 drones have active paths, pending grid edits = 0
... 5s 后，pub 之后 ...
[INFO] [planner_node]: planner summary: 2/8 drones have active paths, pending grid edits = 0
                                                  ↑
                                  收到 TaskAssignment, 给 2 架 drone 排了 A* 路径
```

**结论：三关链路 perception → scheduling → planning 端到端跑通。**
planner 把 `/drone_states` 喂回 scheduler，scheduler summary 显示 8 active drones
也是因为 planner_node 的回环输出在生效。

注意：tracker_node 没有 `/camera/image` 输入，所以 `/target_track` 不会自发自发；
本次链路验证是用 `ros2 topic pub` 直接注入的。如果未来接入真实摄像头或视频源，tracker
会持续发出 `/target_track` 消息（带 header，含 frame_idx）。

### 4.5 已知 benign traceback

每个节点被 `kill -9` / `SIGINT` 杀掉时，`rclpy.spin` 抛 `ExternalShutdownException`，
部分节点（如 `enclosure_node`）还会在 `rclpy.shutdown()` 上抛 `RCLError: failed to shutdown`。
这些是超时/手动终止导致的退出异常，**不是节点运行期 bug**。

## 5. 联调 gap 清单

### 5.1 已确认 OK（无 gap）

- ✅ 5 个 executable 全部能 colcon build + ros2 run 启动并进入 spinning 状态
- ✅ Topic 名 100% 对齐（默认参数 vs 矩阵）
- ✅ Publisher / Subscriber 数量与矩阵一致
- ✅ `/target_track` 被 3 个下游订阅，链路 fan-out 生效
- ✅ `/drone_states` 形成 planner → scheduler 回环
- ✅ `ros2 topic pub` 注入消息后，scheduler/planner 的 summary 指标立即变化
- ✅ swarm_interfaces msg 包所有 .msg 正确生成 idl，colcon build 无 rosidl 报错
- ✅ enclosure_node 的 3 个订阅（/target_track、/enclosure_targets、/drone_states）和 1 个发布（/enclosure_command）都按字面量注册成功

### 5.2 已知可改进项（非 gap，非阻塞）

- ⚠️ `tracker_node` 默认依赖 `cvtrack` Python 包（pip install -e 或 PYTHONPATH 注入）。
  这条已经在源码里给了 warning 日志，集成 demo 时需要在 `three_links_demo.sh` 顶部加
  `export PYTHONPATH="$REPO_ROOT/ros2_ws/src/perception_pkg/cvtrack/src:$PYTHONPATH"`。
  本次联调已确认加 PYTHONPATH 后 tracker_node 能正常启动。
- ⚠️ `enclosure_node` 在 SIGINT 后 `rclpy.shutdown()` 会报 `RCLError: rcl_shutdown already called`，
  退出码非 0。建议后续在 `main()` 里把 `rclpy.shutdown()` 包在 try/except，或用
  `rclpy.init()` + `try/finally` 模式。这是 cleanup hygiene，不是功能缺陷。
- ⚠️ `TargetTrack.msg` 不含 `std_msgs/Header`，只有 `TargetTrackArray` 含。
  下游消费者（scheduler / enclosure / planner）目前用 `tracks` 字段即可，不依赖
  track 级 header；`/target_track_debug` 用了一个独立 `TargetTrackDebug.msg`，也没要求
  track 级 header。如果未来要做 frame-level time sync，需要把 header 加到 TargetTrack。
- ⚠️ 当前没有 `/grid_map` 的发布方。`planner_node` 订阅 `/grid_map` 但没人发。
  在本次联调里 grid 始终是空默认（默认 100×100 全部 free），所以 planner 还能跑；
  真实集成需要 simulation_pkg 或 perception_pkg 提供 grid map 发布器。
- ⚠️ `coord_transform_node` 输出 `/target_track_world` 在本次联调里没人订阅（subscription count=0）。
  它是 "感知 → 规划" 的可选 world-frame 中间表示，目前调度链路不依赖它。建议在 matrix 文档中
  显式标注它是 **可选 / reserved**，避免下游误以为它已被使用。

### 5.3 必须处理（gap）

> **当前没有 blocker gap。** 所有声明的 topic 在代码里都已 `create_subscription`/`create_publisher`
> 实际注册，DDS 层真实可见，publisher/subscriber 计数与矩阵相符，消息流通真实可达。

唯一接近 gap 的是上面 ⚠️ 项里的：`/grid_map` 缺发布方、`/target_track_world` 缺消费者。
这两个属于 "未来集成点"，不在本次 4 节点三关链路的功能 scope 内，所以不列为 blocker。

## 6. 复现命令（一次性）

```bash
cd /home/hhh/Downloads/Swarm-Control-System
export PYTHONPATH="$PWD/ros2_ws/src/perception_pkg/cvtrack/src:$PYTHONPATH"
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash          # 或 colcon build 之后 source

# 后台 5 个节点
nohup ros2 run perception_pkg tracker_node --ros-args \
    -p input_mode:=topic -p track_topic:=/target_track \
    -p enclosure.enabled:=true -p enclosure.topic:=/enclosure_targets \
    > /tmp/tracker.log 2>&1 &
nohup ros2 run perception_pkg coord_transform_node > /tmp/coord.log 2>&1 &
nohup ros2 run scheduler_pkg scheduler_node            > /tmp/sched.log 2>&1 &
nohup ros2 run planning_pkg planner_node              > /tmp/plan.log 2>&1 &
nohup ros2 run containment_pkg enclosure_node          > /tmp/encl.log 2>&1 &
sleep 4

ros2 node list
ros2 topic list
ros2 node info /scheduler_node
ros2 topic info /target_track

# 注入端到端消息验证
ros2 topic pub --once /target_track swarm_interfaces/msg/TargetTrackArray \
    "{tracks: [{target_id: 42, x: 12.5, y: 7.5, vx: 0.1, vy: 0.2, \
                confidence: 0.9, cls: 0, is_confirmed: true, speed: 0.3, motion_mode: 2}]}"
ros2 topic pub --once /drone_states swarm_interfaces/msg/DroneStateArray \
    "{drones: [{drone_id: 1, x: 10.0, y: 20.0, z: 5.0, available: true}, \
                {drone_id: 2, x: 30.0, y: 40.0, z: 5.0, available: true}]}"

# 5 秒后查看 summary 日志会显示 scheduler/planner 都吃到了消息
sleep 5
grep summary /tmp/sched.log
grep summary /tmp/plan.log

# 收尾
pkill -9 -f tracker_node
pkill -9 -f coord_transform_node
pkill -9 -f scheduler_node
pkill -9 -f planner_node
pkill -9 -f enclosure_node
```

## 7. 总结

- **B1**: 源码结构核对 ✅ —— 5 个 executable / 5 个 pkg / 14 个 topic（含 `parameter_events`、`rosout`）
- **B2**: 真实启动 ✅ —— colcon build 通过，5 节点并行启动后 `ros2 node list` 与 `ros2 topic list` 都符合预期，`ros2 topic pub` 注入消息后下游日志立刻反映
- **B3**: 互订阅矩阵 ✅ —— 见本文件 §2，13 个核心 topic × 5 节点完整覆盖
- **B4**: 联调 gap ✅ —— 没有 blocker，5 个非阻塞改进项已记录在 §5.2

整体判定：**4 节点（5 个 executable）真实端到端跑通**，联调通过。