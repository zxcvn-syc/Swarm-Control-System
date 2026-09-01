# UGV 真机部署与调试手册（一机一车实验）

适用范围：`ugv_base_driver` 包（`ros2_ws/src/ugv_base_driver`），2026-09-01 新增。
用途：把 ROS2 的 `/cmd_vel` 速度指令转换成串口命令发给小车底盘驱动板，
作为 Swarm-Control-System 规划/封控结果落地到实车的最后一段链路。

## 1. 职责边界

本仓库提供的是"指令到串口"这一层：

```
planner_node（/planned_path，米）
        │
        ▼
ugv_path_follower          ← 本包：路径点 → /cmd_vel（纯追踪算法）
        │  /cmd_vel (geometry_msgs/Twist)
        ▼
ugv_base_driver            ← 本包
        │  串口文本命令（默认 "L<左轮rad/s> R<右轮rad/s>\n"）
        ▼
底盘驱动板（厂商提供，负责电机 PWM / 编码器闭环）
```

**驱动板固件协议由厂商/工程师定义。** 如果厂商协议不是默认文本格式，
只需修改 `ugv_base_driver/command_protocol.py` 中新增一个编码函数，
不要改节点和运动学。

## 2. 上车前信息核对清单（问工程师）

| 项目 | 为什么需要 | 默认假设 |
|---|---|---|
| 板载电脑型号与系统 | 决定安装命令 | Ubuntu 22.04 + ROS2 Humble |
| 底盘驱动板型号/固件协议 | 决定串口命令格式 | 文本 `L<左> R<右>\n` |
| 串口设备名与波特率 | 打开串口用 | `/dev/ttyUSB0` @ 115200 |
| 轮距（两轮中心距） | 运动学换算 | 0.4 m |
| 轮半径 | 运动学换算 | 0.075 m |
| 最大线/角速度 | 安全限速 | 1.0 m/s / 1.0 rad/s |

## 3. SSH 登录与代码上传

Windows 电脑网线直连小车（详见 2026-09-01 对话记录），登录后：

```bash
# 板载电脑上（示例用户名 ubuntu，按实际情况替换）
sudo apt install -y python3-serial        # pyserial
cd ~
git clone https://github.com/zxcvn-syc/Swarm-Control-System.git
# 或从电脑直接传：scp -r ros2_ws/src/ugv_base_driver ubuntu@<小车IP>:~/Swarm-Control-System/ros2_ws/src/

cd ~/Swarm-Control-System/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_base_driver
source install/setup.bash
```

## 4. 安全机制（写进代码，不要绕过）

1. **使能门控**：节点启动后处于 DISABLED 状态，不转发任何指令。
   必须显式发布使能后 `/cmd_vel` 才会下发到串口：
   ```bash
   ros2 topic pub --once /ugv_base_driver/enable std_msgs/msg/Bool "{data: true}"
   ```
2. **看门狗**：0.5 秒内没有新的 `/cmd_vel`，自动向串口写零速命令。
   上层节点崩溃/断链时车会自己停。
3. **限速**：线/角速度超过 `max_linear_speed`/`max_angular_speed` 时
   等比缩放（保持转弯弧线不突变）。
4. **Ctrl+C 退出时自动发零速**再关闭串口。

## 5. 首次调试流程（务必架空车轮）

```bash
# ① 确认串口设备（插上驱动板后）
ls /dev/ttyUSB* /dev/ttyACM*

# ② 起节点（先按实际参数改 serial_port / 轮距 / 轮半径）
ros2 launch ugv_base_driver ugv_base_driver.launch.py \
    serial_port:=/dev/ttyUSB0 baudrate:=115200 protocol:=text

# ③ 使能
ros2 topic pub --once /ugv_base_driver/enable std_msgs/msg/Bool "{data: true}"

# ④ 发一个低速直线指令（轮子架空！）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# ⑤ 观察：日志应持续打印 "cmd v=+0.200 ..."；驱动板有反应即链路通
#    停止：Ctrl+C 停掉 pub，0.5 秒后看门狗应打印 "STOP sent (cmd_vel timeout)"

# ⑥ 断使能（安全锁）
ros2 topic pub --once /ugv_base_driver/enable std_msgs/msg/Bool "{data: false}"
```

## 6. 常见问题

- **串口打不开**：`ls -l /dev/ttyUSB0` 看属主，非 dialout 组执行
  `sudo usermod -aG dialout $USER` 后重新登录。
- **驱动板不动但日志正常**：协议不匹配。抓厂商协议文档，
  在 `command_protocol.py` 加对应编码函数并在参数里切换 `protocol`。
- **方向反了**：单侧轮反转让轮速取反，或整体交换左右接线；
  运动学符号遵循 REP-103（+x 前进，+z 角速度左转）。
- **车一直发疯**：立即 `Ctrl+C`（退出发零速）并物理断电。

## 7. 与无人机联调（一机一车）

- 无人机侧参照 `docs/deployment/real_uav_deployment.md`。
- 实机网络使用专用 `ROS_DOMAIN_ID`（如 71），电脑/无人机/小车同网段同域。
- 联调顺序：无人机感知出 `/target_track_world` → scheduler 出
  `/task_assignment` → planner 出 `/planned_path` →
  `ugv_path_follower`（本包）把路径点转为 `/cmd_vel` →
  `ugv_base_driver` 下发串口。用法见第 9 节。

## 8. 已知缺口（诚实声明）

- [x] 路径跟随节点（`/planned_path` → `/cmd_vel`）：已实现
      （`ugv_path_follower`，纯追踪算法，43 项单元测试本地通过）。
- [ ] 里程计（`/odom`）未发布，当前车侧是开环速度控制；
      如需反馈闭环需驱动板回传编码器数据。
- [ ] 串口协议默认值是通用文本格式，需工程师确认厂商协议后定稿。

## 9. 路径跟随节点（ugv_path_follower）

### 9.1 接口

| 方向 | 话题/参数 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `path_topic`（默认 `/planned_path`） | `nav_msgs/Path` | planner_node 输出，米、world 系 |
| 订阅 | `pose_topic`（默认 `/ugv_pose`） | `geometry_msgs/PoseStamped` | 小车当前位姿，**必须与路径同坐标系** |
| 发布 | `cmd_vel_topic`（默认 `/cmd_vel`） | `geometry_msgs/Twist` | 给 ugv_base_driver |
| 参数 | `target_frame_id`（默认 `""`） | str | planner 把所有平台的路径混在同一条消息里，靠每个点的 `frame_id=drone_<id>` 区分。**多车实验必须设**（如 `drone_4`），单机实验留空即可 |

### 9.2 位姿从哪来（当前最大的现实缺口）

`ugv_base_driver` 是开环的，不发布 `/odom`，所以 `/ugv_pose` 需要
外部喂。现阶段可选：

- **台架/桌面测试**：手写一个固定/匀速假位姿发布器（见 9.4）；
- **真机联调**：任何外部定位（动捕、UWB、RTK）桥接成
  `PoseStamped` 发到 `/ugv_pose`；
- **后期**：驱动板回传编码器后，写轮式里程计节点发布 `/odom`，
  再由 `odometry → PoseStamped` 的桥接节点转发。

### 9.3 启动

```bash
ros2 launch ugv_base_driver ugv_path_follower.launch.py \
    target_frame_id:=drone_4 max_linear_speed:=0.3
```

安全行为（与 base_driver 叠加）：

- 位姿 0.5 s / 路径 2 s 没更新 → 自动发零速；
- 到达终点（默认 0.25 m 容差）→ 零速并保持；
- Ctrl+C 退出时发最后一帧零速。

### 9.4 台架测试流程（不需要真位姿真路径）

```bash
# 终端①：起跟随节点
ros2 launch ugv_base_driver ugv_path_follower.launch.py

# 终端②：喂一个静止位姿（原点、朝 +X）
ros2 topic pub -r 10 /ugv_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# 终端③：喂一条 5 米直线路径（frame_id 留空=不过滤）
ros2 topic pub --once /planned_path nav_msgs/msg/Path \
  "{header: {frame_id: world}, poses: [
     {header: {frame_id: ''}, pose: {position: {x: 1.0, y: 0.0}}},
     {header: {frame_id: ''}, pose: {position: {x: 3.0, y: 0.0}}},
     {header: {frame_id: ''}, pose: {position: {x: 5.0, y: 0.0}}}]}"
```

预期：终端①日志打印 `new path: 3 waypoints, goal=(5.00, 0.00)`；
`ros2 topic echo /cmd_vel` 应看到 `linear.x≈0.5`（或限速值）、
`angular.z≈0`。停止喂位姿 0.5 s 后 cmd_vel 归零（看门狗生效）。

### 9.5 调参建议

| 现象 | 调整 |
|---|---|
| 过弯甩尾/切内角 | 增大 `lookahead_distance`（0.6 → 0.8/1.0） |
| 弯道跟不上、走出路径 | 减小 `lookahead_distance` 或增大 `max_angular_speed` |
| 终点停不稳、来回蹭 | 增大 `goal_tolerance`（0.25 → 0.4） |
| 到终点冲过头 | 增大 `slowdown_radius`（1.0 → 1.5） |

### 9.6 重编译后 ros2 run 找不到入口（No executable found）

**现象**：`colcon build` 后 `ros2 run ugv_base_driver ugv_path_follower` 报
`No executable found`。

**原因**：部分环境（尤其容器内手动装的 colcon）把 `console_scripts`
生成的入口装到 `install/<pkg>/bin/`，而 `ros2 run` 只搜
`install/<pkg>/lib/<pkg>/`。包内 `setup.py` 已通过 `data_files` 把
`scripts/` 下的入口同时安装到 `lib/<pkg>/`，标准 ROS2 环境不会遇到此问题；
遇到时按下面命令修复（只改 install 产物，不碰源码）：

```bash
# 在容器内工作空间根目录执行
WS=/home/ubuntu/ros2_ws
mkdir -p $WS/install/ugv_base_driver/lib/ugv_base_driver
for f in $WS/install/ugv_base_driver/bin/*; do
  ln -sf "$f" "$WS/install/ugv_base_driver/lib/ugv_base_driver/$(basename "$f")"
done
# 验证
source $WS/install/setup.bash && ros2 run ugv_base_driver ugv_path_follower --ros-args -p target_frame_id:=drone_4
```

看到 `ugv_path_follower ready: ...` 即修复成功，Ctrl+C 退出。
