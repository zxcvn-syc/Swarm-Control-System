# containment_pkg：动态 Voronoi 封控

`containment_pkg` 将目标跟踪坐标和无人机位置转换为封控编队指令。当前实现以目标为中心、以无人机相对目标方向为外法向，生成确定性的 Voronoi-inspired 环形封控点；目标或无人机移动后会自动更新。

## ROS2 接口

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/target_track` | `swarm_interfaces/msg/TargetTrackArray` | 主目标输入，读取 `tracks[].x/y` |
| 输入 | `/enclosure_targets` | `swarm_interfaces/msg/EnclosureTargetArray` | 兼容输入，读取 `targets[].x/y` |
| 输入 | `/drone_states` | `swarm_interfaces/msg/DroneStateArray` | 读取 `drones[].drone_id/x/y/z` |
| 输出 | `/enclosure_command` | `swarm_interfaces/msg/EnclosureCommandArray` | 每架无人机的 `target_x/y/z` 和半径 |

`/target_track` 是主要来源；兼容话题适用于已有封控/调度节点。两者同时发布时，最后到达的消息会覆盖内部目标快照，联调时建议只启用一个目标输入。

## 动态更新机制

节点回调只更新内存快照并设置 dirty 标记，不直接计算。定时器以 `update_period` 触发检查：只有目标和无人机都已收到且 dirty 时才执行一次 `voronoi_enclose()` 并发布。计算完成后清除 dirty，因此同一周期内连续收到多条更新只产生一次输出；下一条输入会开启下一次更新。

## 参数

默认配置位于 `config/containment.yaml`：

- `enclosure_radius: 25.0`：封控点距目标的有效半径（米）。
- `min_dist: 5.0`：半径下限（米）。
- `update_period: 1.0`：动态重算周期（秒）。

## 启动

```bash
cd /home/hhh/Downloads/Swarm-Control-System/ros2_ws
colcon build --packages-select swarm_interfaces containment_pkg
source install/setup.bash
ros2 launch containment_pkg containment.launch.py
```

## 静态 baseline

先固定发布一帧目标和无人机状态，确认 `/enclosure_command` 有输出；再保持无人机位置不变，只移动 `TargetTrackArray.tracks[0].x/y`，每个 `update_period` 周期应看到封控点整体随目标移动。目标为空或无人机尚未到达时节点等待，不发布不完整的计算结果。

## 测试

在已 source ROS2 和 `swarm_interfaces` 的环境中执行：

```bash
pytest containment_pkg/tests/
```

其中动态算法测试不依赖 ROS2；节点测试在缺少 ROS2 接口时会自动跳过。
