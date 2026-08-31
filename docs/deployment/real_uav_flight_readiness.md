# 真机验证准备与放行手册

> 日期：2026-08-30
> 适用对象：一架 PX4 飞行器、一台机载计算机、一个目标和一名安全飞手。
> 本手册不构成飞行许可。系统的 `GO` 只表示指定的**技术证据**通过；飞手、现场负责人和 PX4 的独立失效保护始终拥有最终权限。

## 明天的边界

明天只能按顺序完成台架、系留或防护网内、单机低高度低速度三个阶段。禁止自动解锁、禁止使用 `px4_sitl.launch.py` 或 `sitl_test.launch.py` 作为真机入口、禁止多机自主起飞、禁止把录像/SITL/固定种子 MATLAB 结果当作真机证据。

`px4_offboard_bridge` 默认关闭 setpoint 输出，`auto_arm` 默认关闭。未完成本手册的全部前置证据前，不启动控制 bridge；绝不以临时设置 `auto_arm:=true` 绕过人工解锁和模式切换。

## 分工与停止权

| 角色 | 必须负责的事项 | 停止权 |
| --- | --- | --- |
| 飞行负责人 | 空域、天气、场地、阶段放行和证据归档 | 可随时宣布 `NO-GO` |
| 安全飞手 | RC、模式、人工悬停、返航/降落和物理急停 | 最高优先级，任何异常立即接管 |
| 机载计算机操作员 | 相机、ROS 2、MAVROS、预检 JSON 与 rosbag | 不得 ARM 或切 Offboard |
| 观察员 | 人车隔离、目视姿态、电池和障碍物 | 可口头叫停 |

任意一人叫停，立即停止进入下一阶段。安全飞手按已演练的 RC 失效处置操作，地面操作员停止 ROS 节点并保存日志；不要依赖本项目软件完成返航或降落。

## 上电前清单

以下项目必须由现场人员完成，并填写到 `real_uav_operator_checklist.yaml`：

- 螺旋桨、机臂、紧固件、电池锁定和机载计算机供电均已检查；台架阶段默认卸桨。
- 空域/场地已授权，人员与障碍物隔离，风、雨、能见度和 GNSS/定位条件适合当前阶段。
- RC 链路、模式开关、PX4 电池/RC/数据链/Offboard 丢失保护及地理围栏已在 QGroundControl 按本机型和固件版本逐项复核。
- 已实际演练手动保持、返航或降落、断开机载计算机链路以及物理急停；不采用未验证的通用 PX4 参数值。
- 安全飞手保持 RC 的直接控制权，观察员和操作员已知晓口令与停止位置。
- 真机、机载计算机和地面站使用专用 `ROS_DOMAIN_ID` 与受控网络；开发回放、SITL 不能接入该域。

## 安装现场记录

飞控 USB/数传直通、稳定串口路径、MAVROS 命名空间和一次性启动入口见
[真机插机连接与一键会话](real_uav_connection.md)。该入口会在 MAVROS 未连接时失败，
不会替代下列标定和人工清单。

在机载计算机上创建只读预检所需的两份记录。仓库内的模板故意不能通过预检，不能直接拿来飞行。

```bash
sudo install -d -m 0750 /etc/swarm-control
sudo install -m 0600 \
  ~/Swarm-Control-System-operator-console/ros2_ws/src/planning_pkg/config/real_uav_calibration.template.yaml \
  /etc/swarm-control/real_uav_calibration.yaml
sudo install -m 0600 \
  ~/Swarm-Control-System-operator-console/ros2_ws/src/planning_pkg/config/real_uav_operator_checklist.template.yaml \
  /etc/swarm-control/real_uav_operator_checklist.yaml
```

填写 `real_uav_calibration.yaml` 时，必须提供实测相机内参文件、机体到相机外参文件、相机序列号、PX4 local-origin 与地面基准验证时间。内参重投影误差必须小于本次任务采用的阈值，默认是 `1.0 px`。文档存在不代表标定正确；每次相机重新安装、焦距/分辨率改变、机体改装或场地坐标原点变化后必须重做验证。

`real_uav_operator_checklist.yaml` 必须由当天的飞行负责人和安全飞手填写。全部检查为 `true` 才能通过，这是人工声明的审计记录，不是对物理环境的自动判断。

## 只读技术预检

先构建并加载真机 ROS 2 工作区。预检器只有订阅者，不会发布 MAVROS setpoint、调用 ARM/模式服务、请求 PX4 参数或触发行动。

```bash
source /opt/ros/humble/setup.bash
cd ~/Swarm-Control-System-operator-console/ros2_ws
colcon build --packages-select swarm_interfaces planning_pkg
source install/setup.bash
export ROS_DOMAIN_ID=71
```

启动 MAVROS、相机和位姿来源后，先运行台架检查。飞行器必须保持未解锁，建议卸桨：

```bash
ros2 run planning_pkg real_uav_preflight -- \
  --stage bench \
  --duration 20 \
  --mavros-state-topic /uav0/mavros/state \
  --local-pose-topic /uav0/mavros/local_position/pose \
  --image-topic /camera/image \
  --camera-info-topic /camera/camera_info \
  --battery-topic /uav0/mavros/battery \
  --output-dir ~/flight_evidence/$(date +%Y%m%d)/bench
```

预检会检查 MAVROS 连接、未解锁状态、未进入 Offboard、相机图像频率与时间戳、`CameraInfo` 内参、图像分辨率匹配、本地位姿的有限值/四元数/坐标系及电池阈值。任何 `NO_GO` 都不能通过加大时间阈值、关闭时间戳检查或修改输出 JSON 来绕过；先修复根因并重新运行。

完成相机到地面坐标转换后，检查世界坐标追踪。`/target_track_world` 必须是新鲜的 `world` 帧，并且至少有一个高于置信度阈值的确认目标：

```bash
ros2 run planning_pkg real_uav_preflight -- \
  --stage perception \
  --duration 20 \
  --calibration-manifest /etc/swarm-control/real_uav_calibration.yaml \
  --output-dir ~/flight_evidence/$(date +%Y%m%d)/perception
```

将调度和规划接入后，检查路径仅属于本机。例如 `drone_0` 的每个 Pose 必须带 `frame_id=drone_0`，坐标必须为有限数：

```bash
ros2 run planning_pkg real_uav_preflight -- \
  --stage decision \
  --duration 20 \
  --calibration-manifest /etc/swarm-control/real_uav_calibration.yaml \
  --expected-path-frame drone_0 \
  --output-dir ~/flight_evidence/$(date +%Y%m%d)/decision
```

最后的技术门只在安全监督器处于启动锁定/保持状态时检查所有上述输入和人工清单。此阶段仍然**不允许**把路径送入飞控：

```bash
ros2 launch planning_pkg flight_safety.launch.py \
  require_mavros_connection:=true \
  mavros_state_topic:=/uav0/mavros/state

ros2 run planning_pkg real_uav_preflight -- \
  --stage flight \
  --duration 20 \
  --calibration-manifest /etc/swarm-control/real_uav_calibration.yaml \
  --operator-checklist /etc/swarm-control/real_uav_operator_checklist.yaml \
  --expected-local-frame map \
  --expected-world-frame world \
  --expected-path-frame drone_0 \
  --output-dir ~/flight_evidence/$(date +%Y%m%d)/flight
```

每次运行都会保存 `real_uav_preflight_<time>.json`。只有退出码为 `0` 且 JSON 中同时包含 `technical_gate: "GO"` 与 `read_only: true` 时，才表示这一轮技术检查通过。它不会给出“可起飞”结论。

通过上述技术预检后，浏览器飞手操作台、MAVROS 接口、人工确认命令和路径桥的启动顺序见[真机飞手操作台与接口手册](real_uav_operator_interface.md)。

## 分阶段验证与回退

| 阶段 | 可验证目标 | 进入条件 | 立即停止条件 |
| --- | --- | --- | --- |
| 台架只读 | MAVROS、相机、CameraInfo、位姿、电池 | 卸桨、`bench` 通过 | 心跳/图像/位姿/时间戳缺失，已解锁或已进入 Offboard |
| 台架感知 | 检测、确认轨迹、世界坐标误差 | `perception` 通过，使用地面测量点核对误差 | 标定缺失、frame 不匹配、世界坐标跳变或误差超任务阈值 |
| 台架决策 | 调度、路径归属、监督器默认锁定 | `decision` 通过，`/flight_safety/hold_request=true` | 路径无 `drone_0` 归属、非有限坐标、监督器非锁定 |
| 系留/防护网 | 飞手人工悬停时观察设定点与失链回退 | 上述证据归档、飞行负责人放行、手动控制已演练 | 姿态/位置偏差、抖振、失去目视、任何人叫停 |
| 封闭场地单机 | 低高度、低速度、一个可见目标的人工接管验证 | 系留阶段通过且现场再次确认 | 目标丢失、风雨超限、控制异常、电量/链路异常 |

多机、自动搜索/接力、地面车辆围控、雨天/大风和障碍遮挡的飞行测试不属于明天的单机首次真机验证。它们需要独立风险评估、每机独立 MAVROS 命名空间与时钟同步、失链策略验证，以及逐项场地审批后再进行。

## 必须记录的证据

每阶段开始前再确认飞行器未解锁并启动独立 rosbag。以下记录不会控制飞行器：

```bash
mkdir -p ~/flight_evidence/$(date +%Y%m%d)
ros2 bag record --storage mcap \
  -o ~/flight_evidence/$(date +%Y%m%d)/stage_name \
  /uav0/mavros/state \
  /uav0/mavros/local_position/pose \
  /uav0/mavros/battery \
  /camera/image \
  /camera/camera_info \
  /target_track \
  /target_track_world \
  /planned_path \
  /flight_safety/status \
  /flight_safety/hold_request
```

归档每阶段的预检 JSON、rosbag 元数据、相机标定与地面点测量记录、PX4 参数/固件版本截图、飞行负责人签署的清单、异常视频和停止原因。报告中只能把这些材料称为对应阶段的真机证据，不能外推为多机自主飞行或全天候鲁棒性结论。

## 参考依据

- PX4 Offboard Mode: <https://docs.px4.io/main/en/flight_modes/offboard.html>
- ROS 2 Humble rosbag 记录: <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data.html>
- 工程内既有控制边界: [real_uav_deployment.md](real_uav_deployment.md)
- 工程内失效闭锁说明: [flight_safety_supervisor.md](flight_safety_supervisor.md)
