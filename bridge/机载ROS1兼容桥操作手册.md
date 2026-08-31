# 机载 ROS1 兼容桥操作手册

## 目的与边界

本桥接组件用于把机载 Jetson TX2 的 ROS1 Melodic 观测数据转发给地面/虚拟机
中的 ROS2 Humble 系统，使现有的视觉、显示、记录和状态监看节点可复用。它只转发：

- 相机原始图像、压缩图像与标定信息；
- 本地位姿；
- 电池状态；
- MAVROS `State` 状态。

它不提供 ROS1/ROS2 双向 MAVROS 服务桥接，也不会发送解锁、上锁、模式切换、
Offboard、位置/速度设定点、航线、起飞、降落、返航或 PX4 参数。经桥接显示的
`/uav0/mavros/state` 只能用于观测，不能视为可被 ROS2 控制的飞控连接。

## 已完成准备

ROS2 虚拟机已部署接收端，路径为：

```bash
~/Swarm-Control-System-operator-console/bridge
```

其中 `config/ros2_receiver.env` 已生成随机共享 token，权限为 `0600`。接收端已经
在 ROS2 Humble 上完成协议、断线重连、干净停止及端到端话题映射验证。验证覆盖了
MAVROS 状态、位姿、电池、CameraInfo、原始 BGR 图像与 JPEG 转码话题，并确认不创建
`/uav0/mavros/cmd/arming` 或 `/uav0/mavros/set_mode` 服务。

机载电脑 `192.168.144.60` 的 SSH 与 TCP `19001` 已连通，新目录
`/home/amov/swarm-control-bridge` 已部署，未修改任何既有工作空间。普通 UVC 相机已在
Jetson 发布约 10 FPS 的 `1280x720` JPEG；ROS2 端压缩流约 10 FPS，C++ 解码后的
`/camera/image` 实测约 9.4 FPS。当前仍未发现 FCU 串口设备，MAVROS、真实相机标定与
飞控闭环均未完成，因此浏览器飞手命令继续保持禁用。

## 机载端连通后

机载设备恢复连通后，只在以下新目录部署发送端：

```bash
/home/amov/swarm-control-bridge
```

现有 `EGO_ws`、`fastlio_ws`、`realsense_ws`、`rplidar_ws`、`darknet` 等目录均不需要
移动、重建或修改。部署后依次确认：

```bash
source /opt/ros/melodic/setup.bash
python -m py_compile ~/swarm-control-bridge/protocol.py \
  ~/swarm-control-bridge/ros1_observation_sender.py
bash -n ~/swarm-control-bridge/start_jetson_ros1_sender.sh
```

在已经启动相机、ROS master 和 MAVROS 的终端中，先读取实际话题，不能按名称猜测：

```bash
rostopic list
rostopic info /实际相机话题
rostopic info /实际位姿话题
rostopic info /实际MAVROS状态话题
```

再把这些名称填写到机载端的
`~/swarm-control-bridge/config/jetson_ros1_sender.env`。若消息类型来自已有 catkin
overlay，可把 `ROS1_OVERLAY_SETUP` 填为相应的 `devel/setup.bash`；该操作只 source
环境，不写入 overlay。最后启动发送端：

```bash
cd ~/swarm-control-bridge
./start_jetson_ros1_sender.sh
```

发送端监听 TCP `19001`，只接受带有共享 token 的接收端。相机或 MAVROS 未运行时，
它保持连接但不伪造任何数据。

若机载相机是普通 UVC 设备、现有工作空间没有对应 ROS 驱动，可在发送端之前运行：

```bash
cd ~/swarm-control-bridge
./start_jetson_uvc_camera.sh
```

该脚本只读取真实 `/dev/video0` 图像并发布 JPEG 帧；ROS2 接收端会同时保留压缩图像
并还原 `/camera/image` 供原始图像消费者使用。它不生成虚假的 CameraInfo，涉及世界
坐标计算前仍必须完成真实相机标定。

## ROS2 监看启动

机载发送端已经运行后，在 ROS2 虚拟机中执行：

```bash
cd ~/Swarm-Control-System-operator-console/bridge
./start_ros2_observation_console.sh
```

该命令启动接收端、C++ JPEG 解码器与只读浏览器面板，默认地址为
`http://127.0.0.1:18080`。终端会输出本次会话日志目录；保持该终端运行，按 `Ctrl-C`
会同时停止接收端、解码器和面板。

在另一个已 source ROS2 环境的终端验证：

```bash
ros2 topic echo /uav0/mavros/state --once
ros2 topic echo /uav0/mavros/local_position/pose --once
ros2 topic echo /uav0/mavros/battery --once
ros2 topic echo /camera/camera_info --once
ros2 topic hz /camera/image
```

默认话题可通过两个受限配置文件修改：

- Jetson：`config/jetson_ros1_sender.env`
- ROS2 VM：`config/ros2_receiver.env`

两端的 `BRIDGE_TOKEN` 必须一致；不要把 token 放入 Git、截图、聊天或命令行参数。
`ROS_DOMAIN_ID` 必须使用真机专用域，不能和 SITL、录像回放共用。
保持 `DECODE_COMPRESSED_TO_RAW=false` 与 `USE_IMAGE_TRANSPORT_DECODER=true`，由 C++
`image_transport` 生成 `/camera/image`；不要同时打开 Python JPEG 解码，否则会重复发布
原始图像并显著降低帧率。

## 现场判断

| 现象 | 首先检查 | 不要做 |
| --- | --- | --- |
| 接收端持续提示等待 Jetson | 机载 TCP 22/19001、机载 sender 日志、两端 token | 修改 Jetson IP 或给飞控发送测试命令。 |
| 有连接但没有图像 | `rostopic list`、相机实际主题和消息类型 | 人工伪造图像或假设 RealSense 默认主题。 |
| 有图像但面板没有状态 | 机载 MAVROS 是否运行、`STATE_TOPIC` 和 ROS 域 | 在桥接状态下启用浏览器飞控按钮。 |
| 图像速率过低 | `MAX_IMAGE_HZ`、网络质量、Jetson CPU | 通过移除队列或无限提高码率造成延迟堆积。 |

## 飞控操作说明

本兼容桥完成的是观测兼容，不是飞控控制兼容。真实飞行控制仍应使用已经完成独立台架
验证的原生 ROS2 MAVROS-FCU 连接，或现场 RC/QGroundControl 的既定流程。没有原生 ROS2
MAVROS 服务时，浏览器面板的飞手控制必须保持禁用。首次真机只做卸桨台架观测验证：
确认视频、姿态、电池和时间戳连续稳定后再记录证据，不进入自动飞行或 Offboard 阶段。
