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
      （`ugv_path_follower`，纯追踪算法，单元测试通过；
      2026-09-01 实车台架实测通过）。
- [x] 位姿来源：厂商 MentorPi 容器自带 `ekf_filter_node` 在 `/odom`
      发布 30 Hz 融合里程计（2026-09-01 实测确认），本包
      `ugv_odom_relay`（第 10 节）转发到 `/ugv_pose` 即可闭环。
- [ ] 串口协议默认值是通用文本格式，需工程师确认厂商协议后定稿。
      （注：HiWonder 真机上自带 `/ros_robot_controller` 已订阅
      `/cmd_vel` 直接驱动底盘，本包 base_driver 仅在自研底盘时使用。）

## 9. 路径跟随节点（ugv_path_follower）

### 9.1 接口

| 方向 | 话题/参数 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `path_topic`（默认 `/planned_path`） | `nav_msgs/Path` | planner_node 输出，米、world 系 |
| 订阅 | `pose_topic`（默认 `/ugv_pose`） | `geometry_msgs/PoseStamped` | 小车当前位姿，**必须与路径同坐标系** |
| 发布 | `cmd_vel_topic`（默认 `/cmd_vel`） | `geometry_msgs/Twist` | 给 ugv_base_driver |
| 参数 | `target_frame_id`（默认 `""`） | str | planner 把所有平台的路径混在同一条消息里，靠每个点的 `frame_id=drone_<id>` 区分。**多车实验必须设**（如 `drone_4`），单机实验留空即可 |

### 9.2 位姿从哪来

真机（HiWonder MentorPi）底盘自带 EKF 融合里程计：`ekf_filter_node`
在 `/odom` 上以约 30 Hz 发布 `nav_msgs/Odometry`（轮式里程计 +
激光雷达 `/odom_rf2o` 融合，漂移比纯轮式小）。本包的
`ugv_odom_relay` 节点（见第 10 节）把它转成 `PoseStamped` 发到
`/ugv_pose`，即插即用，无需外购定位硬件。

长航程/高精度需求时再考虑动捕、UWB、RTK 桥接成 `PoseStamped`
发到 `/ugv_pose`（topic 可参数化替换）。

### 9.3 启动

```bash
ros2 launch ugv_base_driver ugv_path_follower.launch.py \
    target_frame_id:=drone_4 max_linear_speed:=0.3
```

安全行为（与 base_driver 叠加）：

- 位姿 0.5 s / 路径 2 s 没更新 → 自动发零速；
- 到达终点（默认 0.25 m 容差）→ 零速并保持；
- Ctrl+C 退出时发最后一帧零速。

### 9.4 台架测试流程（2026-09-01 实车实测通过）

**务必架空车轮。** 实测发现三个坑，命令必须按下面来：

1. **路径不能 `--once` 单发**：DDS 发现需要约 1 秒，单发大概率
   被丢，必须持续发布（`-r 2` + `timeout` 限时）。
2. **路径点的 frame_id 逐点过滤**：`target_frame_id:=drone_4` 时，
   **每个 pose 的** `header.frame_id` 都必须是 `drone_4`；标成
   `map`/`world` 会被全部过滤并告警 `no usable waypoints`。
3. **路径必须持续刷新**：`path_timeout=2s`，超过 2 秒没新路径
   节点自动回零速（真实系统里 planner 持续重发，不受影响）。

```bash
# 终端①：起跟随节点
ros2 launch ugv_base_driver ugv_path_follower.launch.py target_frame_id:=drone_4

# 终端②：喂一个静止位姿（原点、朝 +X），保持运行不要 Ctrl+C
ros2 topic pub -r 10 /ugv_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# 终端③：持续喂一条前进路径，30 秒后自动停
timeout 30 ros2 topic pub -r 2 /planned_path nav_msgs/msg/Path \
  "{header: {frame_id: drone_4}, poses: [
     {header: {frame_id: drone_4}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
     {header: {frame_id: drone_4}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}]}"
```

预期（2026-09-01 实测输出）：

- 终端①日志打印 `new path: 2 waypoints, goal=(2.00, 0.00)`；
- `ros2 topic echo /cmd_vel` 看到 `linear.x=0.5`、`angular.z=0.0`，
  轮子悬空前转；
- 路径停止发布约 2 秒后 cmd_vel 自动归零（path_timeout 生效）；
- 位姿停止 0.5 秒后同样归零（pose_timeout 生效）。

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

## 10. 里程计中继节点（ugv_odom_relay）

真车完整启动顺序（都在 MentorPi 容器内）：

```bash
# ⓪ 厂商底盘执行节点（详见第 11 节，不带它动车指令无人执行）
export MACHINE_TYPE=MentorPi_Tank && ros2 run controller odom_publisher \
    --ros-args --params-file /home/ubuntu/ros2_ws/src/driver/controller/config/calibrate_params.yaml \
    -p base_frame_id:=base_footprint -p odom_frame_id:=odom -p pub_odom_topic:=true

# ① 里程计中继：/odom -> /ugv_pose
ros2 launch ugv_base_driver ugv_odom_relay.launch.py

# ② 路径跟随：/planned_path -> /cmd_vel
ros2 launch ugv_base_driver ugv_path_follower.launch.py target_frame_id:=drone_4

# ③ 验证 /ugv_pose 有数据（约 30 Hz）
ros2 topic hz /ugv_pose
```

### 10.1 接口与参数

| 方向 | 话题/参数 | 类型/默认 | 说明 |
|---|---|---|---|
| 订阅 | `odom_topic`（默认 `/odom`） | `nav_msgs/Odometry` | 厂商 EKF 融合里程计 |
| 发布 | `pose_topic`（默认 `/ugv_pose`） | `geometry_msgs/PoseStamped` | 供 ugv_path_follower 消费 |
| 参数 | `frame_id`（默认 `""`） | str | 输出 frame_id；空 = 透传来源头 |
| 参数 | `restamp`（默认 launch 里 `true`） | bool | 用本节点时钟重打时间戳，避免跨机时钟差导致 pose_timeout 误判 |

### 10.2 坐标系对齐（开局仪式）

`/odom` 从上电时刻起算，而无人机视觉的世界坐标是另一套原点。
演示级对齐做法：

1. 把小车摆在场地原点，**车头朝世界 +X 方向**；
2. 重置里程计。**注意（实测）**：`/set_odom`（Pose2D）只重置轮式原始
   里程计，对 EKF 融合输出 `/odom` 无效。可靠的归零方式是
   **重启 MentorPi 容器**（`docker restart MentorPi`），重启后 odom 从
   当前位置为原点重新起算；
3. 此后 odom 坐标 ≈ 世界坐标，`/ugv_pose` 可直接与 `/planned_path`
   配对使用。若不归零，也可以现读当前 `/ugv_pose` 真实位姿，按该
   位姿换算路径目标（联调脚本的做法）。

长航程或多人场景请改用 TF（`odom -> world` 变换），中继节点无需改动。

### 10.3 实测数据（2026-09-01，HiWonder MentorPi）

| 项目 | 值 |
|---|---|
| `/odom` 发布节点 | `ekf_filter_node` |
| `/odom` 频率 | 约 30 Hz（QoS: RELIABLE/VOLATILE） |
| 数据源 | 轮式里程计 + MS200 激光雷达 `/odom_rf2o` 融合 |
| 累计位置示例 | 上电后移动约 7 m，读数 x=6.65, y=2.43 |

## 11. 厂商底盘执行节点 odom_publisher（动车必读）

**2026-09-01 实测发现**：真正消费 `/cmd_vel` 的不是 `/ros_robot_controller`
（它只收 `/ros_robot_controller/set_motor` 电机指令），而是厂商
`controller` 包里的 **`odom_publisher`** 节点（源码
`src/driver/controller/controller/odom_publisher_node.py`）。它一肩三挑：

- 订阅 `/cmd_vel`（还同时收 `/controller/cmd_vel`、`/app/cmd_vel`）→
  发布 `set_motor` 电机指令；
- 发布轮式里程计 `/odom_raw`（实测约 56 Hz），供 EKF 融合；
- 订阅 `/set_odom` 做里程计重置。

### 11.1 为什么"没自启"（2026-09-01 晚已查明并更正）

~~容器开机自启的 bringup 不含 odom_publisher~~ **更正**：
`bringup.launch.py` 本身**包含** odom_publisher（9.1 晚重启整栈后实测它在进程列表里）。
昨天看不到它，是因为当时执行过 `docker restart MentorPi` 清零里程计——
容器入口只有 `tail -f /dev/null`，ROS 栈是主机侧脚本用 `docker exec` 拉起的
（`/home/pi/mentorpi/start_node.sh`），**docker restart 之后整套栈都是空的**。
完整机制与一键恢复脚本见第 12 节。

它不在的直接后果：

- `/cmd_vel` 订阅数为 0 —— 发指令车不动；
- `/odom_raw` 无发布者 —— 轮式里程计不更新，`/odom` 停滞。

### 11.2 启动命令（含必带的环境变量）

```bash
# 必须先 export，否则节点 KeyError 崩溃
export MACHINE_TYPE=MentorPi_Tank
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 run controller odom_publisher --ros-args \
    --params-file /home/ubuntu/ros2_ws/src/driver/controller/config/calibrate_params.yaml \
    -p base_frame_id:=base_footprint -p odom_frame_id:=odom -p pub_odom_topic:=true
```

`MACHINE_TYPE` 定义在 `/home/ubuntu/shared/.typerc`，只有交互式 shell 才
加载，`bash -c` / SSH 非交互执行必须手动 export。

### 11.3 启动后自查

```bash
ros2 topic info /cmd_vel | grep Subscription   # 应 > 0
ros2 topic hz /odom_raw                          # 应约 56 Hz
```

注意：odom_publisher 重启后轮式里程计从 0 重计，EKF 需要几秒重新收敛，
**发路径前先现读一次 `/ugv_pose` 确认位姿稳定**。

### 11.4 闭环实测记录（2026-09-01，台架空）

启动链路 ⓪odom_publisher + ①ugv_odom_relay + ②ugv_path_follower 后，
按当前真实位姿沿车头发 1.2 m 路径：

- cmd_vel 输出 x≈0.37（执行中）；
- 真实位姿实时前进（编码器计数，架空照样有效）；
- 进入目标 0.25 m 容差后 cmd_vel 归零、位姿稳定——**到点自停**，
  且路径仍在发布（非路径超时停车）。

结论：真实里程计反馈 → 纯追踪 → `/cmd_vel` → 底盘执行 → 到点自停，
车侧链路全通。落地测试前建议把 `max_linear_speed` 从 0.5 降到 0.3。

## 12. 整栈启动机制与一键脚本（2026-09-01 晚实测补充）

### 12.1 ROS 栈到底怎么起来的（重要）

- 容器 `MentorPi` 入口 = `tail -f /dev/null`（restart=always）。
  **容器自己不启动任何 ROS 节点。**
- ROS 栈由**主机侧**开机流程经 `docker exec` 拉起：
  `/home/pi/mentorpi/start_node.sh` → 先 `~/.stop_ros.sh` 杀残留，
  再 `ros2 launch bringup bringup.launch.py`（zsh 加载 `.zshrc` → `.typerc`
  → MACHINE_TYPE=MentorPi_Tank，所以它不需要手动 export）。
- **推论：`docker restart MentorPi`（清零里程计的标准操作）会杀掉整套栈，
  容器重启回来后是空壳，必须重新拉起整栈**——这就是 11.1 节昨天
  "odom_publisher 不见了"的真正原因。
- 整栈健康标准（UDP 探针实测）：/odom ≈ 30 Hz、/scan_raw ≈ 10 Hz、
  imu_raw ≈ 47 Hz、/ros_robot_controller/battery 可读。

### 12.2 车上一键脚本（主机 pi 用户家目录，已部署）

```bash
bash ~/scs_start.sh     # 明早开场仪式：docker restart(清零里程计) → 停残留
                        # → 拉 bringup(35s) → 拉 ugv_odom_relay → 自动体检
                        # 前提：车头朝正东摆好、电脑能 SSH 到车
bash ~/scs_status.sh    # 随时体检：电池mV / /odom / /ugv_pose 频率 / 关键节点
```

### 12.3 FastDDS 共享内存污染坑（诊断必读）

反复用 `timeout ros2 topic hz ...` 强杀 ros2cli 会泄漏 `/dev/shm` 段
（fastrtps_* 文件，一晚积累到 177 个）。之后**新起的**进程（探针、测试脚本）
可能收不到任何话题数据，表现为"节点全死了"，其实是探针瞎了。
今晚实测：SHM 探针收 0 条，同一环境 UDP 探针 30 Hz。

**规矩**：
- 诊断/测试命令一律带 `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`（绕开共享内存）；
- `ugv_odom_relay` 也用该环境变量启动（scs_start.sh 已带）；
- 别用 timeout 强杀 ros2 命令当作常态操作。

### 12.4 电池与网络备忘

- 电池：`/ros_robot_controller/battery`（单位 mV），2S 满电约 8400，
  **实测电机低压保护在 ~7300mV 就切力矩**（7200 拒跑线形同虚设），
  拒跑阈值已提高到 7500（详见 13.2）。整车开机静置也掉电
  （实测 25 分钟掉约 0.18 V），当天测完要么关机要么回充。
- WiFi（2026-09-02 更新）：车 wlan0 已改 STA 连场地网 B616_OP5G
  （DHCP 拿 192.168.1.163，开机自动连），`_car_ssh.py` 探测顺序为
  192.168.2.100(网线) → 192.168.1.163(B616) → 192.168.149.1(热点备用，
  配置保留，可 `nmcli connection up HW-6793FDED` 手动切回)。
- 车主机时钟与真实时间差约 2.5 小时（未做 NTP 对时），看日志时间戳时注意。

## 13. 四关自动测试与实测勘误（2026-09-02 晚）

### 13.1 四关自动测试体系（已上车）

主机安全壳 `~/scs_autotest.sh` + 容器驱动 `scs_autotest_driver.py`（本地副本
`_scs_autotest/`），默认死脚本：
- 四关：`straight`(1.5m 直线) / `turn`(L 形 0.8+右转 0.8) / `retarget`(正前 1.0m →
  右侧 0.9m 动态换目标) / `estop`(2.5m 行进急停 + 3s 漂移 ≤0.05m 检测)
- 安全设计：`--run` 前 5s 倒计时、电池 <7500mV 拒跑、follower 自动拉起；
  驱动被杀后 follower 2s path_timeout 自动零速（实测有效）
- 运行：`bash ~/scs_autotest.sh --run full`（单关 `--run straight` 等）

### 13.2 实测勘误（重要）

- **低压电机保护 ~7300mV 已切力矩**：第 2 关在 7315mV 时车完全静止、60s
  超时 FAIL（不是算法/话题问题）。`BAT_MIN_MV` 已提到 **7500**；开跑前请
  充到 **≥7700**（大电流下电压还会骤降）。
- **厂商 odom 是"假里程计"**：odom_publisher 的轮式里程计 = cmd_vel 数值积分
  （`delta = linear*dt*cos(yaw)`），不读编码器。**电机没转 odom 照样涨**——
  "里程计到点"不能证明车真走了，判关必须结合物理位移/录像。
- **follower 自拉起正确姿势**（scs_autotest.sh 已改）：
  `docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/bash -lc 'source
  /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash
  && export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && nohup ros2 run
  ugv_base_driver ugv_path_follower > /home/ubuntu/follower.log 2>&1 &'`
  不要用 zsh；不要 `docker exec -d`（SSH 断开进程即消失）；日志别写 `/tmp`
  （root 创建过同名文件 → ubuntu 重定向失败 → 命令静默不执行）。
- **launch 文件 import 修复**：Humble 里不存在 `LaunchArgument`（launch.substitutions
  和 launch.actions 都没有）；三个 launch 文件已改为标准
  `from launch.substitutions import LaunchConfiguration` + `LaunchConfiguration("x")`
  用法（车上 src 与 install 副本、仓库侧均已同步）。此前 `ros2 launch` 从未跑通，
  只有 `ros2 run` 路径可用。

### 13.3 电池充好后续跑流程

1. `bash ~/scs_status.sh` 确认 battery ≥ 7500（建议 7700+）；
2. 车头重新朝正东摆好 → 电脑连车（网线 .100 或 B616 .163）→
   `bash ~/scs_start.sh`（docker restart 清零里程计 + 整栈 + relay + 体检）；
3. `bash ~/scs_autotest.sh --run full` 或按关续跑；
4. 每关录像，判定以物理位移为准（勿信 odom 数字）。
