# 真机部署方案与安全门控

## 当前结论

当前工程可进入单机的机载感知、坐标转换和地面决策联调阶段；不能直接宣称具备多机自主飞行能力。2026-08-20 的 ROS 2 视频闭环验证只覆盖离线回放中的追踪、世界坐标转换、调度、规划和围控，不包含真机、解锁或 Offboard 飞行。

真机自主控制前仍需完成相机标定、实际 PX4 位姿链路校验、局部 ENU 网格尺度校验、失链保护验证和封闭场地审批。多机部署还需要完成多相机全局 ID、每机 MAVROS 连接与时间同步验证。

首次真机验证的现场角色、只读技术预检、证据归档和分阶段停止条件见[真机验证准备与放行手册](real_uav_flight_readiness.md)。该手册中的预检工具不会发布飞控命令；其 `GO` 不是飞行许可。

## 建议架构

第一阶段采用一架无人机、一台机载计算机和一个地面监控站：

- 机载计算机：Ubuntu 22.04 + ROS 2 Humble；推荐 Jetson Orin NX 16 GB 或性能相当的 x86 机载计算机。
- 机载节点：相机驱动、`tracker_node`（`input_mode:=topic`）、`coord_transform_node` 和 MAVROS。
- 地面节点：`scheduler_node`、`planner_node`、`grid_map_node`、`enclosure_node` 和只读监控。
- 飞控链路：PX4 通过稳定的 USB/UART 设备连接 MAVROS；使用 udev 规则固定设备名，不能依赖每次启动变化的 `/dev/ttyUSB*`。
- ROS 域：实机网络使用专用 `ROS_DOMAIN_ID`；开发回放、SITL 与真机不能共享同一域。

机载检测必须显式使用已验证的 YOLO 权重；本次无 GPU 虚拟机验证采用的 `MOG2 + DeepSORT` 只适合 CPU 回放吞吐验证，不应用作真机语义检测配置。

## 坐标与话题

真机不得启动 `video_replay_fixture`。`coord_transform_node` 必须使用实际相机的 `CameraInfo` 与 PX4 的本地位姿：

```text
/camera/image -> tracker_node -> /target_track
/camera/camera_info + /uav0/mavros/local_position/pose
    -> coord_transform_node -> /target_track_world
    -> scheduler_node -> /task_assignment
    -> planner_node -> /planned_path
```

当前规划器把网格单元按 1 m 处理，并由 MAVROS 将 ROS ENU 本地坐标转换为 MAVLink LOCAL_NED。上机前必须在地面基准点测量并确认：相机内参、相机到机体外参、PX4 local-origin、地面高度和网格原点一致。任何一项未经标定时，`/target_track_world` 只能用于显示，不能用于控制。

`/planned_path` 的每个 pose 以 `drone_<id>` 标记归属。`px4_offboard_bridge` 现在只接收其 `drone_id` 对应的 pose，防止多机 bridge 误跟随其他无人机的路径。

## 控制门控

`px4_offboard_bridge` 默认不发送 setpoint，也不会调用 ARM 或模式切换服务。必须同时满足以下条件才会发送控制数据：

1. 显式设置 `enable_setpoint_streaming:=true`。
2. 显式设置非负 `drone_id`，例如 `drone_id:=0`。

`auto_arm` 默认为 `false`，真机阶段必须保持为 `false`。只允许在封闭场地、经人工检查后手动解锁与切换模式；自动解锁仅保留给隔离 SITL。`three_links.launch.py` 仍默认 `enable_control_bridges:=false`，因此回放和桌面验证不会连接飞控控制面。

`flight_safety_supervisor` 是封控指令的软件安全门：默认锁定、仅在人工确认后允许手动或自动封控，并在状态、目标、指令或 MAVROS 链路异常时关闭上层指令通路、请求当前位置保持。它**不**调用 ARM、解锁、模式切换、RTL 或降落服务；PX4 参数、RC 和物理急停仍是飞行安全的最终边界。完整接口、验收和 SITL 流程见 [飞控安全监督器](flight_safety_supervisor.md)。

## 分阶段验收

1. 台架只读：启动 MAVROS、确认 heartbeat、姿态、相机图像和 `CameraInfo`；不启动控制 bridge。
2. 台架感知：运行 `tracker_node` 的 topic 模式与 `coord_transform_node`，记录 `/target_track` 和 `/target_track_world`；用已知地面标志物计算定位误差。
3. 台架决策：接入 scheduler/planner/enclosure，验证每个 `TargetTrackArray.header.frame_id` 为 `world`，检查每机路径与 `drone_<id>` 标签。
4. 系留或防护网内：只开启 setpoint streaming，`auto_arm:=false`，由飞手手动保持悬停；先验证路径的第一航点、坐标方向、失链后模式和物理急停。
5. 单机封闭场地：低高度、低速度、单目标、全程人工接管。
6. 多机前置条件：每机独立 MAVROS 命名空间、每机路径过滤、统一时钟、全局目标 ID、隔离与避碰策略验证完成后，才允许逐架增加。

每个阶段都应保存 MAVROS state、相机/位姿时间戳、关键 ROS topic 记录和操作人员确认。任何 heartbeat、相机、位姿或世界坐标有效性丢失时，停止进入下一阶段。

## 运行示例

以下只展示节点连接方式，不包含 ARM 或 Offboard 指令：

```bash
source /opt/ros/humble/setup.bash
source ~/Swarm-Control-System/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=71

ros2 run perception_pkg tracker_node --ros-args \
  -p input_mode:=topic \
  -p image_topic:=/camera/image \
  -p detector.backend:=yolo \
  -p detector.weights:=/opt/cvtrack/weights/visdrone_yolov8s.pt

ros2 run perception_pkg coord_transform_node --ros-args \
  -p camera_info_topic:=/camera/camera_info \
  -p drone_pose_topic:=/uav0/mavros/local_position/pose \
  -p input_topic:=/target_track \
  -p output_topic:=/target_track_world
```

在完成前述全部验收前，不要把控制 bridge 加入该启动序列。
