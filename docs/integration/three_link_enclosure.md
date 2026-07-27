# 第三关联调：动态 Voronoi 封控接口约定

## 1. 数据流

第三关的数据流固定为：

```text
何泓林：tracker_node
    └─ /target_track (TargetTrackArray)
程维好：无人机状态估计
    └─ /drone_states (DroneStateArray)
                ↓
陈思睿：enclosure_node
    └─ /enclosure_command (EnclosureCommandArray)
                ↓
程维好：飞控/编队控制器反向反馈执行
```

`enclosure_node` 以 `/target_track` 为主要目标来源，同时保留 `/enclosure_targets`（`EnclosureTargetArray`）兼容旧链路。若两个目标话题同时发布，最后收到的快照会覆盖前一个，正式联调建议只使用 `/target_track`。

## 2. 字段约定

### `/target_track`

- 消息数组字段：`tracks`。
- 每个 `TargetTrack` 必填：`target_id`、`x`、`y`。
- `x/y` 必须是同一世界坐标系下的米制坐标；`vx/vy`、置信度和预测字段可选，不参与当前 baseline 几何计算。
- 当前实现不使用 `frame_idx` 做去重，因此上游应按时间顺序发布。

### `/drone_states`

- 消息数组字段：`drones`。
- 每个 `DroneState` 必填：`drone_id`、`x`、`y`、`z`、`available`。
- 位置和目标必须使用同一坐标原点、轴向和单位。推荐 `map`/世界水平坐标，`z` 保持飞控高度。
- `available=false` 的状态目前仍按数组位置参与计算；若需要剔除不可用无人机，应由状态发布者先过滤，避免 command 数组索引与飞控编队索引不一致。

### `/enclosure_command`

- `commands` 与有效无人机输入按数组顺序对应，使用 `drone_id` 做最终匹配。
- `target_x/y` 是无人机应前往的封控点，`target_z` 默认沿用该无人机当前高度。
- `enclosure_radius` 是该点对应的有效封控半径。
- 无目标时不生成计算指令；无人机多于目标时，多出的无人机收到 `NaN` 位置和半径 `0` 的 standby 指令。

## 3. QoS 与更新策略

当前输入订阅和输出发布使用 ROS2 默认可靠 QoS、队列深度 10。需要实时性优先时，双方应统一改为 sensor-data QoS，不能只修改一端。

节点回调收到目标或无人机消息后只更新快照并设置 dirty 标记。定时器每 `update_period` 秒检查 dirty，默认 `1.0 s`；连续快速更新会被合并为一次 Voronoi 计算，计算结束后才允许下一次更新。该机制避免 tracker（例如 10 Hz）直接驱动封控算法高频重算。

## 4. 静态 Voronoi baseline（推荐第三关首轮流程）

1. 构建并加载接口与封控包：

   ```bash
   cd /home/hhh/Downloads/Swarm-Control-System/ros2_ws
   colcon build --packages-select swarm_interfaces containment_pkg
   source install/setup.bash
   ros2 launch containment_pkg containment.launch.py
   ```

2. 确认节点与话题：

   ```bash
   ros2 node list
   ros2 topic list | grep -E 'target_track|drone_states|enclosure_command'
   ros2 topic echo /enclosure_command
   ```

3. 何泓林发布固定的一帧 `TargetTrackArray`，例如目标为 `(50, 50)`；程维好发布固定的多架 `DroneStateArray`。
4. 等待一个 `update_period`，确认每架无人机收到一条有效 command，半径默认为 25 m。
5. 保持无人机位置不变，将目标从 `(50, 50)` 移动到 `(60, 50)`；下一次周期后 command 的封控点应整体向 x 正方向移动约 10 m。
6. 观察输出频率：即使 `/target_track` 高频发布，`/enclosure_command` 也不会超过 `1/update_period`。

## 5. 联调验收清单

- [ ] `TargetTrackArray.tracks[].x/y` 与 `DroneStateArray.drones[].x/y` 在同一世界坐标系。
- [ ] `/target_track` 和 `/drone_states` 均能持续发布。
- [ ] `enclosure_node` 能看到两个输入并产生 `/enclosure_command`。
- [ ] 固定输入时 command 稳定不抖动。
- [ ] 移动目标后，下一更新周期 command 位置改变。
- [ ] 快速连续输入被 `update_period` 节流。
- [ ] 程维好侧按 `drone_id` 接收 command，并将其作为反向控制目标。
