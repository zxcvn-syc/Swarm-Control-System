# UGV 真机部署与调试手册（一机一车实验）

适用范围：`ugv_base_driver` 包（`ros2_ws/src/ugv_base_driver`），2026-09-01 新增。
用途：把 ROS2 的 `/cmd_vel` 速度指令转换成串口命令发给小车底盘驱动板，
作为 Swarm-Control-System 规划/封控结果落地到实车的最后一段链路。

## 1. 职责边界

本仓库提供的是"指令到串口"这一层：

```
planner_node / enclosure_node / 手柄遥控
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
  `/task_assignment` → planner 出 `/planned_path` → 车侧需要一段
  "路径跟随"逻辑（将路径点转为 `/cmd_vel`，建议纯追踪算法，
  待实现，见下方 TODO）。

## 8. 已知缺口（诚实声明）

- [ ] 路径跟随节点（`/planned_path` → `/cmd_vel`）尚未实现。
- [ ] 里程计（`/odom`）未发布，当前车侧是开环速度控制；
      如需反馈闭环需驱动板回传编码器数据。
- [ ] 串口协议默认值是通用文本格式，需工程师确认厂商协议后定稿。
