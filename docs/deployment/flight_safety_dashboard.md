# 飞行安全图形操作台

`planning_pkg/flight_safety_dashboard` 是飞行安全监督器的本机浏览器操作台。
它在一个页面中显示真实相机视频、目标锁定、封控状态、保持状态和关键链路
新鲜度，并且只代理已有的 `/flight_safety/control` 封控安全门服务。

它**不是遥控器替代品**：不会发布 MAVROS 位置/速度设定点，也没有 PX4 解锁、
模式切换、返航或降落功能。真机的姿态和飞行应急处置仍必须由 RC、PX4
失效保护和已批准的物理流程负责。完整的安全门状态机与 SITL 流程见
[飞行安全监督器](flight_safety_supervisor.md)。

## 前置条件

1. 已构建并启动 `swarm_interfaces`、`containment_pkg`、`planning_pkg`，且
   `flight_safety_supervisor` 正在发布 `/flight_safety/status`。
2. 浏览器所在机器能访问仪表板绑定地址。默认仅绑定 `127.0.0.1:8080`。
3. 视频源发布 JPEG `sensor_msgs/CompressedImage`。默认订阅
   `/camera/image/compressed`；没有视频时页面会显示“未收到视频流”，不会伪造
   画面或缓存历史视频。

若相机当前只发布 `sensor_msgs/Image` 的 `/camera/image`，可在同一 ROS 2
环境中使用 `image_transport` 建立压缩转发，或把 dashboard 的 `video_topic`
参数改为现有 JPEG 压缩话题。不要把 PNG、H.264 或任意网络视频流直接接入该
订阅，因为此版本只接受有尺寸上限的 JPEG 帧。

## 构建与启动

```bash
cd ~/Swarm-Control-System/ros2_ws
colcon build --packages-select swarm_interfaces containment_pkg planning_pkg
source install/setup.bash

# 只读监控模式：不配置令牌时，任何网页控制请求都会被拒绝。
ros2 launch planning_pkg flight_safety_dashboard.launch.py
```

打开本机浏览器访问 `http://127.0.0.1:8080`。顶部“安全状态”会显示状态消息
的接收时效；状态超过 3 秒时，服务端拒绝控制请求，直到收到新的监督器状态。

### 开启本机控制

网页控制默认关闭。必须在启动时配置一个非空、专用于本次操作的
`operator_token`；浏览器中的“控制令牌”会随每次同源请求发送，但前端不会将它
写入 `localStorage`、Cookie 或事件日志。

```bash
ros2 launch planning_pkg flight_safety_dashboard.launch.py \
  operator_token:='replace-with-a-unique-local-token'
```

操作员还必须填写可审计的“操作员编号”。服务端从最新安全状态读取会话 ID 和
已消耗的请求 ID，再生成一次性、短有效期的服务请求，因此浏览器不会自行拼接
或复用控制请求。界面可发出的操作只有：开启手动封控、开启自动封控、停止封控、
紧急保持以及地面确认后复位。

“地面确认后复位”需要勾选地面安全确认，且服务端仍会按监督器状态机验证；该
复选框不是物理传感器，也不表示软件已经验证现场安全。紧急保持只向安全门发出
锁存保持请求，实际飞行中的紧急行为仍须按 RC/PX4/物理流程执行。

## 远程查看与控制

默认 `127.0.0.1` 只允许本机浏览器访问。需要在受控测试网远程查看时可以设置
监听地址，例如 `bind_address:=192.168.88.135`，但此时网页控制仍保持关闭。

远程控制需要同时满足以下条件：配置 `operator_token`，并显式设置
`allow_remote_control:=true`。仅在隔离网络、身份认证、访问控制和实机安全审查
都已完成后才允许这样做。令牌不能代替 DDS/SROS2 身份认证，也不应通过聊天、
Git、截图或共享命令历史传播。

```bash
ros2 launch planning_pkg flight_safety_dashboard.launch.py \
  bind_address:=192.168.88.135 \
  operator_token:='replace-with-a-unique-local-token' \
  allow_remote_control:=true
```

浏览器页面与 API 不添加 CORS 响应头，静态资源、安全状态、视频流和控制请求都
要求同源。HTTP 服务默认带有安全响应头和无缓存策略；它不是可公开暴露到互联网
的认证边界。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `bind_address` | `127.0.0.1` | HTTP 监听地址。除本机回环地址以外均视为远程。 |
| `port` | `8080` | HTTP 端口。 |
| `status_topic` | `/flight_safety/status` | 监督器状态话题。 |
| `control_service` | `/flight_safety/control` | 安全门控制服务。 |
| `video_topic` | `/camera/image/compressed` | JPEG `CompressedImage` 视频源。 |
| `operator_token` | 空 | 非空后才可能开启网页控制。 |
| `allow_remote_control` | `false` | 非回环监听时必须显式为真才能控制。 |
| `status_stale_timeout` | `3.0` | 状态超过该秒数时拒绝控制。 |

## 快速验收

1. 不传 `operator_token` 启动页面，确认能看到安全状态和视频状态，但所有控制
   按钮禁用；向 `/api/control` 请求会得到 `403`。
2. 配置令牌后，在本机打开页面，确认锁定目标、MAVROS、平台状态和封控指令会
   随 `/flight_safety/status` 更新。
3. 断开 JPEG 话题，确认视频区显示离线，而不是上一次的历史帧；恢复话题后确认
   帧序号继续增长。
4. 在 SITL 发出紧急保持，确认监督器进入 `EMERGENCY_HOLD`、保持请求为真，并
   且没有任何 PX4 解锁或模式切换调用。
5. 只有在飞行器已处于地面安全状态并由授权人员确认后，测试“地面确认后复位”。
