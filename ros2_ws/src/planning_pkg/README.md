# 路径规划模块 (`planning_pkg`)

ROS2 路径规划模块，负责为整个集群计算每架无人机的二维网格路径、维护
动态障碍下的增量修复、并将每架无人机的实时位姿发布给
`containment_pkg`（Voronoi 封控），同时把规划出的路径点发布给 RflySim
/ MAVROS 桥接器。

> 写给同队协作者（陈同学 / 何同学 / 曹同学）：
> 这是一个 **纯 ROS2 (ament_python)** 包，下面列的接口约定是跟
> `scheduler_pkg`、`containment_pkg`、`swarm_interfaces` 已存在内容协商
> 后的最终结果，跟他们直接对接时不需要再改 ROS2 消息。

---

## 1. 包结构

```
planning_pkg/
├── planning_pkg/
│   ├── __init__.py
│   ├── __main__.py        # 算法 smoke test
│   ├── astar.py           # A* 实现
│   ├── dstar_lite.py      # D* Lite 实现
│   └── planner_node.py    # ROS2 节点
├── config/
│   └── planning.yaml      # 默认参数
├── launch/
│   └── planning.launch.py
├── tests/
│   ├── test_astar.py
│   └── test_dstar_lite.py
├── package.xml
├── setup.cfg
├── setup.py
├── README.md
└── resource/
    └── planning_pkg       # ament_index 标记
```

顶层启动入口位于 `ros2_ws/launch/three_links.launch.py`，它同时启动
`tracker_node + scheduler_node + planner_node + enclosure_node`，
用于联调（早期版本曾放在仓库根目录 `launch/` 下，现已统一收敛到 `ros2_ws/launch/`）。

---

## 2. 接口

### 2.1 订阅

| Topic              | Message type                              | 来源                    | 说明 |
|--------------------|-------------------------------------------|-------------------------|------|
| `/task_assignment` | `swarm_interfaces/TaskAssignment`         | `scheduler_pkg`         | 收到后立刻给该 drone 重新规划 |
| `/grid_map`        | `std_msgs/UInt8MultiArray`                | 其它模块 / RflySim      | 行主序的栅格，layout 见下 |
| `/lidar_occupancy` | `nav_msgs/OccupancyGrid`                  | `lidar_grid_node`       | 可选直连 LiDAR 栅格；保留 `frame_id`、`origin`、`resolution` |
| `/drone_pose_external` | `swarm_interfaces/DroneStateArray`     | 可选：RflySim / MAVROS  | 用真实位姿覆盖模拟推进 |

`/grid_map` 的 `UInt8MultiArray` 约定：

```python
msg.layout.dim = [
    LabelDim(label="height", size=H, stride=H * W),
    LabelDim(label="width",  size=W, stride=W),
]
msg.data    = (H * W,) bytes, row-major # grid[y, x]
0   = free
!=0 = obstacle
```

### 2.2 发布

| Topic            | Message type                              | 下游消费者           | 说明 |
|------------------|-------------------------------------------|----------------------|------|
| `/drone_states`  | `swarm_interfaces/DroneStateArray`        | `containment_pkg`    | 所有无人机的位置、速度、可用位 |
| `/planned_path`  | `nav_msgs/Path`                           | RflySim / MAVROS     | 每个 drone 一组 `PoseStamped`，`header.frame_id = "drone_<id>"` |

> 若环境里没装 `nav_msgs`，节点仍可启动，只是不会向外发路径。

### 2.3 参数

| 名称 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `num_drones` | int | 8 | 同时维护的无人机数量（运行期可自动扩展） |
| `grid_size` | int | 100 | 正方形网格边长 (cell) |
| `planner` | string | `"astar"` | `"astar"` 或 `"dstar_lite"` |
| `tick_period` | float | 0.5 | 节点主定时器周期 (s) |
| `log_interval_sec` | float | 5.0 | 摘要日志周期 (s) |
| `publish_path` | bool | true | 是否发 `/planned_path` |
| `sim_tick_speed` | float | 1.0 | 每个 tick 无人机沿路径走几个格子 |
| `task_topic` | string | `/task_assignment` | 订阅话题名（可覆盖） |
| `grid_topic` | string | `/grid_map` | 订阅话题名 |
| `occupancy_grid_topic` | string | `""` | 直接订阅 `OccupancyGrid`；设置后不订阅 legacy `/grid_map` |
| `occupancy_threshold` | int | 50 | 大于等于该值的 OccupancyGrid 单元视为障碍 |
| `occupancy_unknown_is_obstacle` | bool | true | 是否把 `-1` unknown 作为障碍（LiDAR 本地地图建议保持 true） |
| `planner_grid_output_topic` | string | `/planner_grid_map_nav` | 规划器当前栅格的可视化输出 |
| `drone_states_topic` | string | `/drone_states` | 发布话题名 |
| `planned_path_topic` | string | `/planned_path` | 发布话题名 |
| `rfly_pose_topic` | string | `/drone_pose_external` | 可选 RflySim 位姿反馈 |
| `initial_positions` | double[] | [] | 无人机初始 `[x0, y0, x1, y1, ...]` |
| `obstacle_cells` | dict[] | [] | 预置障碍；单格或矩形（详见 yaml） |
| `explicit_target_cells` | dict[] | [] | 显式指定每架 drone 的目标 cell |

---

## 3. 算法

### 3.1 A* (`planning_pkg.astar`)

```python
def astar(grid, start, goal, diagonal=True) -> list[tuple[int, int]]:
    """Run A* on a 2D occupancy grid.

    grid: H x W ndarray. 0 = free, non-zero = obstacle.
    start, goal: (x, y) integer tuples.
    diagonal: if True, 8-neighbour moves; else 4.
    Returns a list of (x, y) cells (empty when unreachable).
    """
```

* 启发函数：欧氏距离（开启对角）或曼哈顿距离（关闭对角）
* 邻接：8 邻接的对角代价为 `√2`，4 邻接代价为 `1`
* 边界条件：起点等于终点返回 `[(start)]`；起点/终点被阻挡时优先回退到最近自由 cell
* 不可达：`[]`

### 3.2 D* Lite (`planning_pkg.dstar_lite.DStarLite`)

```python
class DStarLite:
    def __init__(self, grid, start, goal, diagonal=True): ...

    def plan(self) -> list[tuple[int, int]]:
        """Initial search from start to goal."""
        ...

    def update_obstacles(
        self,
        changed_cells: Sequence[Tuple[Tuple[int, int], int]]
    ) -> None:
        """Incrementally repair after one or more cells flip state.

        Each entry is ``((x, y), new_state)`` with ``new_state`` 0 for
        free or non-zero for blocked.  Out-of-bounds entries are ignored.
        ...
        """

    def get_path(self) -> list[tuple[int, int]]:
        """Current best path (or [] when unreachable)."""
        ...
```

实现说明：

- `plan()` 初始规划调用 A*，缓存 `g` 估计；
- `update_obstacles()` 沿着缓存路径检测第一个失效 cell，然后从失效点
  前一个 cell 重新 A*，并把更新过的 `g` 反馈给 `get_path()`；
- 当目标被新障碍切断时，路径会变为 `[]`，下游需自行处理这种情况
  （可以重新派发新任务）。

### 3.3 ROS2 节点 (`planning_pkg.planner_node`)

* 每架 drone 维护一个 D* Lite 实例（`planner == "dstar_lite"`）或
  每架 drone 每次任务都重新 A*（`planner == "astar"`）；
* 收到 `TaskAssignment` 后立刻为该 `drone_id` 重新规划；
* 主定时器每 `tick_period` 秒推进无人机一个或多个格子
  （`sim_tick_speed`），并发布新的 `DroneStateArray`；
* `/grid_map` 或直接 `OccupancyGrid` 更新时对比前后 grid，把差异 cell 提交给所有 D* Lite 实例
  的 `update_obstacles`；A\* 模式下直接清空所有路径并在下一次 tick 重新规划。

---

## 4. 测试

```bash
cd Swarm-Control-System/ros2_ws/src/planning_pkg
python3 -m pytest tests/ -v
```

23 个测试用例覆盖：

| 文件 | 用例 |
|------|------|
| `test_astar.py` | 直线、4 邻接直线、绕障、不可达、4 邻接不可达、起终点相同、起点被阻挡、终点被阻挡、坐标越界、非 2D、随机网格合法性 |
| `test_dstar_lite.py` | 初始规划、绕障、不可达、起终点相同、终点被阻挡、成本与 A\* 一致、**障碍插入改变路径**、障碍移除恢复、目标被切断变空、空操作幂等、grid 形状异常、越界忽略 |

D\* Lite 的关键测试 `test_update_obstacles_changes_path` 真的会
往 grid 里塞一面墙，验证：

```python
initial = planner.plan()
planner.update_obstacles([((4, y), 1) for y in range(1, 9)])
after = planner.get_path()
assert tuple(initial) != tuple(after)
```

---

## 5. RflySim / MAVROS 接入（伪代码）

```python
# 典型的 RflySim 双向：
# - 规划出来的 /planned_path (nav_msgs/Path) 用 mavros 的
#   mavros/setpoint_position/local 写到无人机；
# - 真实位置回报（/mavros/local_position/pose）经 transform 后转发到
#   /drone_pose_external（swarm_interfaces/DroneStateArray）。
#
# 下面这段只是 ROS2 ↔ MAVROS 桥接的标准模式，无需在我们的代码里
# 实现 —— planning_pkg 只负责发布 nav_msgs/Path。

class MavrosBridge:
    def on_planned_path(self, msg: NavPath):
        for ps in msg.poses:
            self.setpoint_pub.publish(PoseStamped(
                pose=Pose(position=ps.pose.position, orientation=...),
                header=Header(frame_id="map"),
            ))

    def on_local_pose(self, msg: PoseStamped):
        ds = DroneState()
        ds.x, ds.y, ds.z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        # ... publish to /drone_pose_external
```

> 真正要跑 RflySim 时，把上面这段放到 `simulation/` 目录的外部桥接
> 节点里，并保证它和 planning_pkg 是**两个独立节点**。这是 ROS2
> 的最佳实践，不要和 `planner_node` 合并。

---

## 6. 与其他模块的接口约定

| 来自 / 去向 | Topic | 类型 | 备注 |
|-------------|-------|------|------|
| `scheduler_pkg.scheduler_node` → planning_pkg | `/task_assignment` | `swarm_interfaces/TaskAssignment` | 一条消息 = (drone_id, target_id, task_type)，target_id 是整数派发号，不是像素坐标 |
| `planning_pkg.planner_node` → `containment_pkg.enclosure_node` | `/drone_states` | `swarm_interfaces/DroneStateArray` | 一定要把所有已注册 drone 都塞进同一个消息里，方便 Voronoi 一次性消费 |
| `planning_pkg.planner_node` → RflySim | `/planned_path` | `nav_msgs/Path` | 每架 drone 一组；`frame_id = "drone_<id>"` 便于下游分流 |

scheduler 端会从 `/task_assignment` 收到我们 echo 的字段名 (`drone_id`,
`target_id`, `task_type`) 已对齐 `swarm_interfaces/TaskAssignment`。
containment 端默认订阅 `/drone_states` (见 `enclosure_node.py` 第 30 行)。

---

## 7. 构建 / 运行

```bash
cd Swarm-Control-System/ros2_ws
colcon build --packages-select planning_pkg
source install/setup.bash

# 单节点
ros2 run planning_pkg planner_node \
    --ros-args --params-file src/planning_pkg/config/planning.yaml

# 或使用 launch
ros2 launch planning_pkg planning.launch.py \
    planner:=dstar_lite grid_size:=120

# 单个固定朝向的二维 LiDAR（UGV / 固定地面站首选）+ planner
ros2 launch planning_pkg lidar_planning.launch.py \
    scan_topic:=/ugv0/scan pose_topic:=/drone_pose_external sensor_id:=100

# 三节点联调（tracker + scheduler + planner + enclosure）
ros2 launch /path/to/Swarm-Control-System/ros2_ws/launch/three_links.launch.py
```

`colcon build --packages-select planning_pkg` 是验收标准之一。
`python3 -m py_compile planning_pkg/*.py` 和 `python3 -m planning_pkg`
是降级验证（在没有 ROS2 colcon 工具链的环境下用）。

### 7.1 LiDAR 接入边界

`lidar_grid_node` 发布的是**瞬时二维水平面** `LaserScan` 栅格，而不是 SLAM。有限回波写为占据单元、无回波射线写为自由单元、未扫描区域保留为 unknown；规划器默认把 unknown 当作障碍。节点要求近期的 `/drone_pose_external` 位姿，过期或缺失时丢弃扫描。

当前 `DroneState` 消息没有 yaw，因此 `sensor_yaw_rad` 是配置的世界系固定方向。首期部署仅适合固定航向 UGV 或固定地面 LiDAR；安装到会转向的 UGV 或 UAV 前，必须先扩展状态接口或提供 TF/yaw，不能把该配置当作三维机载避障。

对于旧 `/grid_map` 消费者，`grid_map_node` 仍发布 `UInt8MultiArray`；同时新增 `/grid_map_occupancy` 无损输出。LiDAR 路径应让 planner 直接订阅 `/lidar_occupancy`，避免回落到没有几何元数据的旧接口。

---

## 8. 写在最后

* 如果你在外层工作空间运行 launch 之前没有重新 build
  `swarm_interfaces`，可能会看到一些老版本的 ROS2 msg（缺
  `DroneState`、`DroneStateArray`、`EnclosureCommand` 等）。解决办法：

  ```bash
  cd Swarm-Control-System/ros2_ws
  colcon build --packages-select swarm_interfaces --allow-overriding swarm_interfaces
  ```

* 当你不确定该用哪个算法时，先 `planner := astar`。它稳定、易理解；
  D\* Lite 在网格很大、`/grid_map` 频繁更新的场景下能省一点时间，
  但目前实现因为把桥接交给 A\* 而并非完全增量。
