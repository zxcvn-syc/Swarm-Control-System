# UGV 真机部署与调试手册（树莓派 5）

适用包：`ros2_ws/src/ugv_base_driver`。

当前车辆硬件信息：树莓派 5 主板、二维激光雷达、深度相机、差速底盘。
本包完成标准 ROS 2 消息到车辆串口的控制链，不包含未知型号设备的厂商驱动。

## 1. 已实现链路

```text
/planned_path (nav_msgs/Path)
        |
        v
ugv_path_follower  <-- /ugv_100/odom
        |  /ugv_100/cmd_vel_nav
        v
ugv_obstacle_guard <-- /ugv_100/scan
        |            <-- /ugv_100/camera/depth/image_raw
        |  /ugv_100/cmd_vel
        v
ugv_base_driver
        |  厂商串口帧
        v
底盘驱动板

/ugv_100/odom --> ugv_odom_state_bridge --> /ground_vehicle_states
```

四个节点均采用失效停车策略：未使能、输入超时、传感器超时、急停、
非法数值、串口写入失败或坐标系不一致时，不继续发送运动速度。

## 2. 必须从硬件确认的信息

| 项目 | 当前代码接口 | 上车前必须确认 |
|---|---|---|
| 激光雷达 | `sensor_msgs/LaserScan` | 型号、驱动包、真实 topic、零度是否朝车头 |
| 深度相机 | `sensor_msgs/Image` | 型号、驱动包、真实 depth topic、编码 |
| 里程计 | `nav_msgs/Odometry` | 编码器、视觉里程计或 SLAM 的输出 topic 和 frame |
| 底盘串口 | `text` / `text_rpm` 示例协议 | 厂商帧格式、波特率、校验、左右轮单位 |
| 几何参数 | 轮距 0.4 m、轮半径 0.075 m | 实测轮距和有效滚动半径 |

深度门控支持 ROS 深度图常用的 `16UC1`（毫米）与 `32FC1`（米）。
其他编码会被拒绝并停车。激光扫描按 ROS 约定以零弧度为前方；若雷达安装
方向不同，应由静态 TF/驱动修正，不能靠错误扇区继续运行。

激光雷达和深度相机本身不等于里程计。路径跟随必须有稳定的 `/odom`，
并且其 `header.frame_id` 与路径顶层 frame 一致。当前规划器发布 `world`，
所以真车应提供已经变换到 `world` 的 odometry。

## 3. 树莓派 5 软件基线

为了与团队工作区一致，优先使用 64 位 Ubuntu 22.04 + ROS 2 Humble。
若树莓派当前是 Ubuntu 24.04 + ROS 2 Jazzy，需要在该系统上重新构建整个
工作区；本包只使用标准 ROS 2 API，但不能把未执行的 Jazzy 构建写成已验证。

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-serial

cd ~/Swarm-Control-System/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to ugv_base_driver
source install/setup.bash
```

若系统使用 Jazzy，将上面的 `humble` 换成 `jazzy`。

## 4. 先验证硬件 ROS topic

先单独启动雷达、深度相机和里程计驱动，再执行：

```bash
ros2 topic list | sort
ros2 topic type /scan
ros2 topic type /camera/depth/image_raw
ros2 topic type /odom
ros2 topic hz /scan
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /odom
ros2 topic echo --once /scan
ros2 topic echo --once /camera/depth/image_raw --field encoding
ros2 topic echo --once /odom --field header
```

预期类型依次为：

```text
sensor_msgs/msg/LaserScan
sensor_msgs/msg/Image
nav_msgs/msg/Odometry
```

如果真实 topic 不同，不要改节点源码，启动时用 `scan_topic`、`depth_topic`、
`odom_topic` 参数指向真实 topic。

## 5. 架空车轮做串口测试

先不启动自动路径链，只启动底盘驱动：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*

ros2 launch ugv_base_driver ugv_base_driver.launch.py \
  vehicle_namespace:=ugv_100 \
  serial_port:=/dev/ttyUSB0 \
  baudrate:=115200 \
  protocol:=text
```

新终端中使能并发送低速命令：

```bash
source ~/Swarm-Control-System/ros2_ws/install/setup.bash
ros2 topic pub --once /ugv_100/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub -r 10 /ugv_100/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

停止发布后 0.5 秒内看门狗必须停车。方向不对时修改
`left_wheel_sign`、`right_wheel_sign` 或 `swap_wheels`，不要交换运动学公式。

## 6. 启动完整自动链路

确认传感器和里程计持续发布后：

```bash
ros2 launch ugv_base_driver ugv_vehicle.launch.py \
  vehicle_namespace:=ugv_100 \
  vehicle_id:=100 \
  serial_port:=/dev/ttyUSB0 \
  baudrate:=115200 \
  protocol:=text \
  scan_topic:=/scan \
  depth_topic:=/camera/depth/image_raw \
  odom_topic:=/odom_world \
  output_frame:=world \
  path_resolution:=0.5
```

绝对 topic（以 `/` 开头）不会自动加车辆命名空间；相对 topic 会解析到
`/ugv_100/...`。按设备实际 topic 选择写法。

当前 `planning_pkg` 的路径点是网格 cell 坐标，默认地图分辨率为
0.5 m/cell，所以 `path_resolution:=0.5`。若规划端以后改为直接发布米制坐标，
应改为 `1.0`，不能重复缩放。

## 7. 启动前状态检查

```bash
ros2 topic echo /ugv_100/path_status
ros2 topic echo /ugv_100/obstacle_status
ros2 topic echo /ugv_100/driver_status
ros2 topic echo /ground_vehicle_states
```

在未使能状态下，状态应显示 disabled。遮住深度相机、停止雷达驱动或停止
里程计后，门控状态必须变成 missing/stale/no_valid_ranges，且车辆速度为零。

默认同时要求雷达和深度相机在线。初次只验证雷达时可以临时启动：

```bash
ros2 launch ugv_base_driver ugv_vehicle.launch.py \
  require_depth:=false require_lidar:=true \
  serial_port:=/dev/ttyUSB0 odom_topic:=/odom_world
```

这只是分阶段调试参数。正式运行应恢复两路传感器，或在实验记录中明确说明
关闭了哪一路安全输入。

## 8. 统一使能、停车与复位

四节点监听同一个使能和急停 topic：

```bash
ros2 topic pub --once /ugv_100/enable std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /ugv_100/estop std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /ugv_100/enable std_msgs/msg/Bool "{data: false}"
ros2 service call /ugv_100/ugv_path_follower/reset_estop std_srvs/srv/Trigger "{}"
ros2 service call /ugv_100/ugv_obstacle_guard/reset_fault std_srvs/srv/Trigger "{}"
ros2 service call /ugv_100/ugv_base_driver/reset_fault std_srvs/srv/Trigger "{}"
```

急停是锁存的，向 `/estop` 发布 false 不会自动恢复。复位后仍保持 disabled，
必须重新显式使能。

## 9. 障碍门控参数

关键参数位于 `config/ugv_base_driver.yaml`：

- `base_stop_distance`：静止基础停车距离，默认 0.6 m。
- `slowdown_distance`：开始线性减速的基础距离，默认 1.8 m。
- `reaction_time`、`max_deceleration`：按当前前进速度增加制动距离。
- `lidar_forward_half_angle`：雷达前向扇区半角，默认 0.7 rad。
- `depth_roi_*`：深度图中央检测区域。
- `depth_sample_stride`：深度采样步长，默认 4，适合树莓派 5 低负载运行。
- `allow_reverse_without_rear_sensor`：默认 false；没有后向传感器时禁止倒车。
- `allow_rotation_when_blocked`：默认 false；近障时连原地旋转也禁止。

停车距离必须用现场低速制动测试标定，默认值不是车辆的实测制动性能。

## 10. 尚需现场完成的硬件项

- 厂商底盘协议尚未知；当前 `text`/`text_rpm` 只是可测试的协议插件。
- 本包不生成编码器 odometry；必须接入底盘、视觉里程计或 SLAM 输出。
- 雷达和深度相机型号未知，因此各自厂商驱动、USB 权限与 udev 规则未写死。
- 尚未在这辆树莓派 5 实车上完成轮距、轮径、方向、制动距离和传感器外参标定。

这些项不影响节点和接口完整性，但在确认前不能宣称真车闭环已经通过。
