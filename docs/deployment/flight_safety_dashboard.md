# 飞行安全图形操作台

`planning_pkg/flight_safety_dashboard` 是飞行安全监督器的本机浏览器操作台。
它在一个页面中显示真实相机视频、目标锁定、封控状态、保持状态和关键链路
新鲜度，并且只代理已有的 `/flight_safety/control` 封控安全门服务。

它默认**不是遥控器替代品**：不会发布 MAVROS 位置/速度设定点，也不会在默认
配置下调用任何 PX4 服务。仅当显式开启 `enable_pilot_commands`、配置 token 和
可写审计日志后，才会暴露受确认的解锁/上锁、`POSCTL`、`ALTCTL` 和受安全门
约束的 `OFFBOARD` 请求；仍不提供返航、降落、起飞、位置/速度手控或参数修改。
真机姿态和飞行应急处置必须由 RC、PX4 失效保护和已批准的物理流程负责。完整的
飞手接口与启动顺序见[真机飞手操作台与接口手册](real_uav_operator_interface.md)，
完整安全门状态机见[飞行安全监督器](flight_safety_supervisor.md)。

## 前置条件

1. 已构建并启动 `swarm_interfaces`、`containment_pkg`、`planning_pkg`，且
   `flight_safety_supervisor` 正在发布 `/flight_safety/status`。
2. 浏览器所在机器能访问仪表板绑定地址。默认仅绑定 `127.0.0.1:8080`。
3. 视频源发布 JPEG `sensor_msgs/CompressedImage`。默认订阅
   `/camera/image/compressed`；没有视频时页面会显示“未收到视频流”，不会伪造
   画面或缓存历史视频。
4. 需要识别叠加时，构建并启动 `perception_pkg/tracker_node`，让它从与视频相同的
   相机帧发布 `/target_track`。检测框必须来自模型实际输出，禁止用固定尺寸框或
   模拟消息冒充真机识别结果。

若相机当前只发布 `sensor_msgs/Image` 的 `/camera/image`，可在同一 ROS 2
环境中使用 `image_transport` 建立压缩转发，或把 dashboard 的 `video_topic`
参数改为现有 JPEG 压缩话题。不要把 PNG、H.264 或任意网络视频流直接接入该
订阅，因为此版本只接受有尺寸上限的 JPEG 帧。

## 构建与启动

```bash
cd ~/Swarm-Control-System-operator-console/ros2_ws
colcon build --packages-select swarm_interfaces containment_pkg planning_pkg
source install/setup.bash

# 只读监控模式：不配置令牌时，任何网页控制请求都会被拒绝。
ros2 launch planning_pkg flight_safety_dashboard.launch.py
```

打开本机浏览器访问 `http://127.0.0.1:8080`。顶部“安全状态”会显示状态消息
的接收时效；状态超过 3 秒时，服务端拒绝控制请求，直到收到新的监督器状态。

### 启动真实识别叠加

以下命令以 `/camera/image` 为模型输入，使用项目权重发布真实像素框、类别、
置信度、跟踪 ID 和速度方向。权重路径和设备必须按现场环境核对，不能依赖空路径
触发的运动检测降级模式。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run perception_pkg tracker_node --ros-args \
  -p input_mode:=topic \
  -p image_topic:=/camera/image \
  -p track_topic:=/target_track \
  -p publish_rate_hz:=5.0 \
  -p detector.backend:=yolo \
  -p detector.weights:="$PWD/src/perception_pkg/best.pt" \
  -p detector.device:=cpu \
  -p detector.imgsz:=480 \
  -p detector.conf:=0.25 \
  -p tracker.stationary_prune:=false
```

操作台对感知结果采用三种互斥状态：

- **感知离线**：从未收到 `/target_track`，或者结果超过
  `perception_stale_timeout`；检测框会立即隐藏。
- **未检测到目标**：感知帧新鲜，但模型返回空数组。这是有效的零目标结果，
  不是链路故障。
- **检测到 N 个目标**：按相机原始图像尺寸将真实边界框映射到视频；标签同时显示
  类别、跟踪 ID 和置信度，已由监督器锁定的 ID 使用锁定样式。

若视频正常但持续为零目标，先确认相机确实对准人员或车辆，再检查权重类别、光照、
目标尺寸与阈值。不要为了“出现检测框”向真机页面注入测试目标。

### 开启本机控制

网页控制默认关闭。必须在启动时配置一个非空、专用于本次操作的 token；默认从
环境变量 `FLIGHT_SAFETY_TOKEN` 读取，避免把凭据写进 launch 命令行。浏览器中的
“控制令牌”会随每次同源请求发送，但前端不会将它写入 `localStorage`、Cookie 或
事件日志。

```bash
export FLIGHT_SAFETY_TOKEN="$(openssl rand -hex 32)"
ros2 launch planning_pkg flight_safety_dashboard.launch.py
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

远程控制需要同时满足以下条件：配置 `FLIGHT_SAFETY_TOKEN`（或受控的
`operator_token` 参数），并显式设置
`allow_remote_control:=true`。仅在隔离网络、身份认证、访问控制和实机安全审查
都已完成后才允许这样做。令牌不能代替 DDS/SROS2 身份认证，也不应通过聊天、
Git、截图或共享命令历史传播。

```bash
ros2 launch planning_pkg flight_safety_dashboard.launch.py \
  bind_address:=192.168.88.135 \
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
| `perception_topic` | `/target_track` | 与当前视频同源的像素级真实跟踪结果。 |
| `perception_stale_timeout` | `3.0` | 超过该秒数将感知标为过期并隐藏检测框。 |
| `operator_token` | 空 | 覆盖环境变量的直接 token；不建议在命令行使用。 |
| `operator_token_env` | `FLIGHT_SAFETY_TOKEN` | 从环境读取控制 token 的变量名。 |
| `allow_remote_control` | `false` | 非回环监听时必须显式为真才能控制。 |
| `status_stale_timeout` | `3.0` | 状态超过该秒数时拒绝控制。 |

## 快速验收

1. 不设置 `FLIGHT_SAFETY_TOKEN` 启动页面，确认能看到安全状态和视频状态，但所有控制
   按钮禁用；向 `/api/control` 请求会得到 `403`。
2. 配置令牌后，在本机打开页面，确认锁定目标、MAVROS、平台状态和封控指令会
   随 `/flight_safety/status` 更新。
3. 断开 JPEG 话题，确认视频区显示离线，而不是上一次的历史帧；恢复话题后确认
   帧序号继续增长。
4. 保持 `/target_track` 在线但让镜头对准无目标区域，确认页面显示“未检测到目标”；
   再把已知人员或车辆放入画面，确认检测框、类别、ID 和置信度与目标同步移动。
5. 停止 `tracker_node`，确认 3 秒后页面显示“感知结果已过期”并隐藏旧框，恢复节点
   后重新出现新鲜结果。
6. 在 SITL 发出紧急保持，确认监督器进入 `EMERGENCY_HOLD`、保持请求为真，并
   且没有任何 PX4 解锁或模式切换调用。
7. 只有在飞行器已处于地面安全状态并由授权人员确认后，测试“地面确认后复位”。
