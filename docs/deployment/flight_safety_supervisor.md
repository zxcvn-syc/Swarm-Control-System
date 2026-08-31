# 飞行安全监督器

## 适用范围

`planning_pkg/flight_safety_supervisor` 是封控链路上的 ROS 2 失效闭锁
指令门。它提供可观测的目标锁定与封控状态、人工或自动启用方式，以及
软件层的紧急保持联锁。

它不能替代 PX4 的电池、遥控器、数据链、地理围栏或 Offboard 丢失保护。
它绝不调用 MAVROS 的解锁/上锁、模式切换、返航或降落服务。每次真机飞行
仍必须具备物理急停流程，并正确配置 PX4 失效保护。

## 数据流

```text
enclosure_node
  /enclosure_command（带 header 和 sequence 的心跳）
        |
        v
flight_safety_supervisor
  /flight_safety/enclosure_command --> enclosure_command_bridge --> planner
  /flight_safety/hold_request ------> px4_offboard_bridge
  /flight_safety/status ------------> 操作员显示器 / 记录器
  /flight_safety/control <----------- 操作员服务客户端
```

监督器启动时处于 `LOCKED`，并持续发布 `hold_request=true`。只有收到
新的、已授权的启用请求和新的封控指令后，状态才会转为 `ACTIVE`。
Offboard 桥接器收到保持请求后会清空当前路径；本地位姿可用时，持续发送
被捕获的位置；本地位姿不可用时，不会伪造位置设定点，而是停止发送设定点，
使 PX4 已配置的 Offboard 丢失保护继续作为最终边界。解除保持也不会恢复旧
路径，规划器必须重新发布带正确平台归属的新路径。

封控侧可使用以下集成启动文件。它只启动安全监督器、封控计算节点和命令
桥接器；目标源和规划器必须已经运行，且该启动文件不会控制 PX4：

```bash
ros2 launch planning_pkg supervised_containment.launch.py \
  target_topic:=/target_track_world \
  require_mavros_connection:=true \
  mavros_state_topic:=/uav0/mavros/state
```

它是直接把 `enclosure_command_bridge` 接到 `/enclosure_command` 的受监督
替代方案。该启动文件会开启封控心跳，并且只允许桥接器订阅
`/flight_safety/enclosure_command`。

## 状态机

| 状态 | 指令门行为 | 退出条件 |
| --- | --- | --- |
| `LOCKED` | 阻断封控指令并请求保持。启动、停用和复位后的默认状态。 | 基础健康检查通过后，收到新的手动或自动启用请求。 |
| `MANUAL_READY` | 保持，等待一条在启用后产生的新封控指令。 | 新指令转为 `ACTIVE`；停用则回到 `LOCKED`。 |
| `AUTO_READY` | 保持，等待稳定目标锁定和新封控指令。 | 两项条件都满足时转为 `ACTIVE`。 |
| `ACTIVE` | 仅转发新鲜且序号递增的封控指令。 | 超时、所需目标锁定丢失、无效/重放指令、紧急保持或停用。 |
| `FAULT` | 锁存保持请求与故障位。 | 更新的显式复位请求且已确认地面安全后，回到 `LOCKED`。 |
| `EMERGENCY_HOLD` | 锁存操作员发起的保持请求。 | 更新的显式复位请求且已确认地面安全后，回到 `LOCKED`。 |

自动模式默认要求两个连续的新鲜 `TargetTrackArray` 样本确认同一个目标 ID。
将 `require_target_lock_in_manual:=true` 后，手动模式也要求满足同一锁定条件。

## 接口与操作

`swarm_interfaces/msg/FlightSafetyStatus` 会在 `/flight_safety/status`
上以可靠、瞬态本地 QoS 发布。消息包含状态、启用方式、锁定目标 ID、指令与
控制请求序号、指令新鲜度、MAVROS 新鲜度/连接状态、保持请求、故障掩码和
可读原因，可直接查看：

```bash
ros2 topic echo /flight_safety/status
```

日常查看和控制使用随包提供的终端控制台。它先从状态消息取得当前会话和已
消耗的最新请求号，再产生带有效期的新请求，因此无需手工拼接服务请求：

```bash
ros2 run planning_pkg flight_safety_console watch
ros2 run planning_pkg flight_safety_console enable-manual --operator-id safety_pilot
ros2 run planning_pkg flight_safety_console enable-auto --operator-id safety_pilot
ros2 run planning_pkg flight_safety_console emergency-hold --operator-id safety_pilot
ros2 run planning_pkg flight_safety_console reset-fault --operator-id safety_pilot --ground-confirmed
```

该控制台不提供 PX4 模式、解锁、返航或降落控制。`--ground-confirmed` 是一个
可审计的操作员联锁输入，不代表软件已经感知或验证了实际物理环境。

需要实时视频、目标锁定和封控状态的本机浏览器操作台时，请使用
[飞行安全图形操作台](flight_safety_dashboard.md)。它仍只控制本节描述的安全门，
不会增加任何 PX4 飞行控制能力。

`swarm_interfaces/srv/SafetyControl` 服务位于 `/flight_safety/control`，
支持 `ENABLE_MANUAL`、`ENABLE_AUTO`、`DISABLE`、`EMERGENCY_HOLD` 和
`RESET_FAULT`。

每个请求都必须携带状态消息中的当前 `session_id`、非空操作员 ID、严格递增
的 `request_id` 和未来的 `expires_at`。监督器每次启动都会随机生成新的
`session_id`，因此来自旧进程的已捕获请求会被拒绝。除非特意为仅 SITL 场景
关闭该保护，`RESET_FAULT` 还必须指定 `ground_confirmed:=true`。即使一个
格式合法的请求因当前状态无法执行，其请求 ID 仍会被消耗，不能通过修改请求
内容后用同一 ID 重放。

这些检查用于防止误操作与重放，并不能认证恶意 ROS 图参与者。真机部署必须
使用隔离的 `ROS_DOMAIN_ID`、受控网络和 SROS2/DDS 安全机制。

`EnclosureCommandArray` 现在包含 `header` 和单调递增的 `sequence`。
安全门会拒绝缺失、过期、未来时间戳、无效或重放的指令心跳。启用安全门时，
必须设置 `enclosure_node.publish_heartbeat:=true`；否则即便静态封控方案本身
有效，也会按设计因心跳超时而关闭。

## SITL 启动

必须先构建共享接口，再构建引用这些接口的包：

```bash
cd ~/Swarm-Control-System/ros2_ws
colcon build --packages-select swarm_interfaces containment_pkg planning_pkg
source install/setup.bash
export PX4_SITL_ROOT=/home/hhh/src/PX4-Autopilot
ros2 launch planning_pkg flight_safety_sitl.launch.py
```

该启动文件先启动监督器，并让 Offboard 桥接器以初始保持状态运行；它还显式
关闭了桥接器原有的仅 SITL 自动解锁行为。监督器不会代表操作员启用封控、
解锁飞行器或切换 PX4 模式。

如果不使用 `supervised_containment.launch.py`，必须显式把桥接器接到门控
话题：

```bash
ros2 run containment_pkg enclosure_command_bridge --ros-args \
  -p command_topic:=/flight_safety/enclosure_command \
  -p output_topic:=/task_assignment
```

同时以 `publish_heartbeat:=true` 启动 `enclosure_node`。应使用专用 ROS 2
服务客户端或操作员控制台发出请求，使过期时间相对于当前 ROS 时钟仍在未来。
不要复制旧时间戳或旧请求号的 shell 命令，因为监督器会将其视为过期或重放而
拒绝。

## 故障响应

| 观测到的条件 | 监督器动作 | 飞控边界 |
| --- | --- | --- |
| 平台状态超时或无可用平台 | 锁存 `FAULT`、阻断指令、请求保持。 | 飞行器失效保护仍由 PX4 负责。 |
| 必需的 MAVROS 状态超时或断开 | 锁存 `FAULT`、阻断指令、请求保持。 | 不发送模式或解锁请求。 |
| 自动模式下目标超时或锁定丢失 | 锁存 `FAULT`、阻断指令、请求保持。 | 手动模式只能在显式启用后使用。 |
| 指令超时、格式错误、未来时间戳或序号重放 | 锁存 `FAULT`、阻断指令、请求保持。 | 桥接器会清空既有路径。 |
| 操作员紧急保持 | 锁存 `EMERGENCY_HOLD`、请求保持、清空路径。 | 实际飞行紧急处置须使用 RC、PX4 和物理流程。 |
| 操作员复位 | 需要新请求、操作员 ID 和地面安全确认，随后回到 `LOCKED`。 | 不会恢复路径或激活飞行模式。 |

## 验收步骤

1. 启动后确认系统处于锁定状态，且 `/flight_safety/hold_request` 为真。
2. 验证旧会话 ID、重复请求 ID、过期请求和重复指令序号均被拒绝，且绝不转发。
3. 启用手动模式，发布新指令并确认进入 `ACTIVE`；停止心跳后，确认锁存
   `FAULT` 且保持请求为真。
4. 启用自动模式，对同一目标发布两个确认的新鲜样本，再确认目标丢失会关闭门。
5. 在 SITL Offboard 设定点流中发出 `EMERGENCY_HOLD`；确认桥接器清空路径、
   捕获当前本地位姿，且没有任何解锁或模式调用。
6. 在无本地位姿时重复上述操作；确认不会发布伪造位置设定点，并只在受控
   SITL 环境中观察 PX4 的 Offboard 丢失保护行为。

在完成 `real_uav_deployment.md` 中的标定、遥控器与失效保护测试、封闭场地
审批和物理紧急流程前，真机使用仍被禁止。
