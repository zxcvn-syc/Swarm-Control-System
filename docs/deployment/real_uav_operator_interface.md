# 真机飞手操作台与接口手册

> 适用范围：单架 PX4 飞行器（`uav0`）、ROS 2 Humble、MAVROS、机载计算机和本机浏览器。
> 本操作台服务于人工在环试验，不执行自动起飞、降落、返航、位置/速度手控、任务上传或 PX4 参数修改。姿态、方向和高度的人工控制只通过已验证的 RC 完成。

本文命令使用已验证的独立目录 `~/Swarm-Control-System-operator-console`，不修改
你的原 `~/Downloads/Swarm-Control-System` 工作区。

## 控制边界

浏览器操作台在默认配置下只显示视频和安全门状态。只有同时设置：

1. `enable_pilot_commands:=true`；
2. 非空的 `FLIGHT_SAFETY_TOKEN`（默认）或受控的 `operator_token` 参数；
3. 非空、可写的 `pilot_audit_log`；
4. `bind_address:=127.0.0.1`，或明确设置 `allow_remote_control:=true`；

它才会显示可用的飞手命令。每条命令必须填写操作员编号，按下按钮后输入精确确认短语，控制端会在发送 MAVROS 服务请求前把意图写入 JSONL 审计日志。

`/api/pilot-control` 不直接表示飞行器状态已经改变。PX4/MAVROS 仅会回复请求是否被接收或已发送，必须观察面板中的 `/uav0/mavros/state` 实际更新后再进入下一步。

## ROS 接口

| 方向 | 名称 | 类型 | 用途 |
| --- | --- | --- | --- |
| 输入 | `/uav0/mavros/state` | `mavros_msgs/msg/State` | 连接、解锁、飞行模式；超过 1 秒未更新则拒绝命令。 |
| 调用 | `/uav0/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | 仅由“请求解锁”或“地面确认后上锁”按钮调用。 |
| 调用 | `/uav0/mavros/set_mode` | `mavros_msgs/srv/SetMode` | 仅支持 `POSCTL`、`ALTCTL` 与受安全门约束的 `OFFBOARD`。 |
| 输入 | `/flight_safety/status` | `swarm_interfaces/msg/FlightSafetyStatus` | 监督器状态、保持、目标锁定和封控门。 |
| 调用 | `/flight_safety/control` | `swarm_interfaces/srv/SafetyControl` | 仅控制封控安全门，不直接控制 PX4。 |
| 输入 | `/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | 浏览器的实时视频。 |
| 输入 | `/planned_path` | `nav_msgs/msg/Path` | 规划路径；只由 `px4_offboard_bridge` 处理。 |

所有名称都可通过 launch 参数改为实际命名空间。一个面板只服务一架飞行器；多机必须每架单独启动面板、独立 token、审计文件和 MAVROS 命名空间。

## 浏览器操作

启动后通过浏览器打开 `http://127.0.0.1:8080`。远程查看建议使用 SSH 隧道，而不是开放 LAN 控制端口：

```bash
ssh -L 8080:127.0.0.1:8080 hhh@<机载计算机地址>
```

页面由三部分组成：

- **现场视频与运行记录**：确认相机、目标与每次控制结果。
- **封控状态与控制**：启用/停止封控、紧急保持、地面确认复位。它始终经过 `flight_safety_supervisor`。
- **PX4 状态与人工确认控制**：显示 FCU 链路、解锁状态、模式，并提供有限的 MAVROS 请求。

| 按钮 | 确认短语 | 服务请求 | 服务端放行条件 |
| --- | --- | --- | --- |
| 请求解锁 | `ARM` | `CommandBool(value=true)` | MAVROS 新鲜且已连接；飞行器当前未解锁；安全门新鲜、`LOCKED` 且处于保持。 |
| 地面确认后上锁 | `DISARM` | `CommandBool(value=false)` | MAVROS 新鲜且已连接；勾选“地面安全确认”；飞行器当前已解锁。 |
| 切换 POSCTL | `POSCTL` | `SetMode(custom_mode=POSCTL)` | MAVROS 新鲜且已连接。用于从 Offboard 回到飞手控制。 |
| 切换 ALTCTL | `ALTCTL` | `SetMode(custom_mode=ALTCTL)` | MAVROS 新鲜且已连接。仅在该机型已经验证此模式时使用。 |
| 请求 OFFBOARD | `OFFBOARD` | `SetMode(custom_mode=OFFBOARD)` | MAVROS 新鲜/连接/已解锁；安全门新鲜、`ACTIVE`、目标已锁定、封控已启用且保持已解除。 |

紧急情况优先使用 RC 模式开关、PX4 失效保护和现场物理流程。浏览器的 `POSCTL` 只是有审计的辅助请求，不能代替已演练的 RC 接管。

## 启动顺序

以下命令均为单机 `uav0` 示例。先完成[真机验证准备与放行手册](real_uav_flight_readiness.md)
的 `bench`、`perception` 与 `decision` 预检；所有命令均在机载计算机执行。

```bash
source /opt/ros/humble/setup.bash
source ~/Swarm-Control-System-operator-console/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=71
export FLIGHT_SAFETY_TOKEN="$(openssl rand -hex 32)"
mkdir -p ~/flight_evidence/$(date +%Y%m%d)
chmod 700 ~/flight_evidence/$(date +%Y%m%d)
```

使用单命令启动入口拉起默认锁定的监督器和本地操作台。`pilot_audit_log` 是飞手控制
的必填项；token 不写入 shell 历史、截图或报告：

```bash
ros2 launch planning_pkg real_uav_operator_console.launch.py \
  bind_address:=127.0.0.1 \
  port:=8080 \
  enable_pilot_commands:=true \
  mavros_state_topic:=/uav0/mavros/state \
  arm_service:=/uav0/mavros/cmd/arming \
  mode_service:=/uav0/mavros/set_mode \
  pilot_audit_log:="$HOME/flight_evidence/$(date +%Y%m%d)/pilot_commands.jsonl"
```

此时面板可查询状态和提出经确认的 MAVROS 请求，但尚未运行路径控制桥。先以**卸桨台架**验证 `POSCTL` 请求和状态更新，再进行系留验证。

仅在系留或防护网阶段、经飞行负责人放行后，才启动路径桥。它显式开启 setpoint 流，但保持 `auto_arm:=false`，并以安全门的初始保持状态启动：

```bash
ros2 run planning_pkg px4_offboard_bridge --ros-args \
  -p path_topic:=/planned_path \
  -p state_topic:=/uav0/mavros/state \
  -p local_pose_topic:=/uav0/mavros/local_position/pose \
  -p setpoint_topic:=/uav0/mavros/setpoint_raw/local \
  -p enable_setpoint_streaming:=true \
  -p auto_arm:=false \
  -p drone_id:=0 \
  -p safety_hold_enabled:=true \
  -p initial_safety_hold:=true \
  -p safety_hold_topic:=/flight_safety/hold_request
```

该桥不发 ARM 或模式请求。安全飞手先用 RC 确认处于可接管的 `POSCTL`；在面板确认
安全门锁定、保持为真时，使用“请求解锁”并在 `/uav0/mavros/state` 中确认
`armed=true`。只有目标锁定、路径归属正确、监督器进入 `ACTIVE` 且保持解除后，
“请求 OFFBOARD”才会放行。若没有连续 setpoint，PX4 应按已验证的 Offboard 丢失保护
处理；不要依赖本软件临时猜测安全位置。

## Web API

面板仅监听本机，HTTP 请求以 `X-Flight-Safety-Token` 传入 token：

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| `GET` | `/api/status` | 安全状态、视频状态、MAVROS 状态与命令可用性。 |
| `POST` | `/api/control` | 既有封控安全门请求。 |
| `POST` | `/api/pilot-control` | 受确认、审计的飞手 MAVROS 请求。 |

`/api/pilot-control` 请求体：

```json
{
  "action": "position",
  "confirmation": "POSCTL",
  "operator_id": "safety_pilot",
  "ground_confirmed": false
}
```

不要把 HTTP API 暴露到公网、无线访客网络或共享 ROS 域；不要用脚本循环调用该接口。它为人工单次操作设计，服务端会拒绝并发请求、过期 MAVROS 状态、无 token、错误确认短语和不满足安全门的 Offboard 请求。

## 结束与归档

1. 由安全飞手用 RC 切换回已验证的人工模式并确认飞行器状态更新。
2. 停止封控，确认安全门回到 `LOCKED`/保持状态。
3. 飞行器在地面后，勾选地面确认并使用“上锁”，再核对 `armed=false`。
4. 保存 `pilot_commands.jsonl`、预检 JSON、rosbag、PX4 参数/固件记录和异常视频。

任何不一致、抖振、路径 frame 错误、目标失锁、相机/位姿时间戳过期、MAVROS 断开或人为叫停，都应停止进入下一阶段并按 RC/PX4 的既定流程处置。
