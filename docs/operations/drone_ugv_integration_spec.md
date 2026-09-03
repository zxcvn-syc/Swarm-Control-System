# 一机一车实物实验对接规格书（室外 GPS 方案）

> 本文档是车侧与无人机/集成侧的对接契约，自包含、可直接投喂给对接侧 AI 执行。
> 车侧链路已于 2026-09-01 实车验证通过（闭环：里程计反馈 → 纯追踪 → 底盘 → 到点自停）。
> 车侧不需要任何代码改动即可联调；本文档中的"无人机侧要求""集成侧要求"两节是待办。

## 1. 系统拓扑与职责

```
[无人机] PX4 EKF(融合GNSS) → MAVROS /mavros/local_position/pose (ROS1)
    → ROS1/ROS2 观测桥 → /uav0/mavros/local_position/pose
    → 位姿桥(补 stamp/frame_id) → /drone_pose_external (ROS2)

[集成电脑] ROS2: scheduler + enclosure_node + UGV planner_node
    输入: /drone_pose_external + /target_track_world(或 /enclosure_targets)
    输出: /planned_path (nav_msgs/Path)

    ─────── WiFi 路由器 (同一网段, 同一 ROS_DOMAIN_ID, DDS 组播) ───────

[小车 MentorPi 容器]
    odom_publisher(厂商, 手动启动) → 电机指令 + /odom_raw
    ekf_filter_node(厂商) → /odom (30Hz)
    ugv_odom_relay(我们) : /odom → /ugv_pose (30Hz PoseStamped)
    ugv_path_follower(我们) : /planned_path + /ugv_pose → /cmd_vel (20Hz)
```

职责一句话：无人机管看人和自己定位，集成电脑管算坐标发任务，小车管堵路。

## 2. 坐标系约定（联调成败的关键，先做这个）

### 2.1 世界系定义

- **world 系 = ENU**：原点 = **无人机 PX4 上电点**，+X = 东，+Y = 北，单位米。
- PX4 `local_position` 本身就是"相对上电点的 ENU"，与该定义天然一致，无人机侧零换算。
- 普通 GPS 水平精度 2~5 m，原点摆放误差在这个量级面前可忽略。

### 2.2 车的 odom 系对齐仪式（推荐方案，零代码）

车侧 `/ugv_pose`、`/planned_path` 的坐标都在车的 **odom 系**（原点=车里程计归零点，
+X=归零时车头朝向）。让两个系近似重合的摆位方法：

1. 把小车摆到场地中，**车头朝正东**（= world +X）。用手机指南针对准即可。
2. 把无人机放在**小车旁边**（0.5 m 内）上电开机——PX4 原点即在此处。
3. 在车上执行 `docker restart MentorPi`，等 2 分钟重启完，车的里程计归零。
4. 完成后：车 odom (0,0) ≈ world 原点，车 +X ≈ world +X。
   残余误差 = 无人机摆放偏移（≤0.5 m）+ 车头朝向误差（每偏 5°，每米多 9 cm）。

**联调前必须做一次这个仪式**，之后集成电脑发的 world 坐标路径车可以直接执行。

### 2.3 精确方案（可选，仅当 0.5 m 级原点误差不可接受时）

若无人机无法摆在车旁上电，需集成端做平移桥：量出车 odom 原点在 world 系的
`(dx, dy)`，路径下发前做 `world → odom` 平移（`x-dx, y-dy`）。
建议优先用 2.2 的摆位仪式，避免多一层变换代码。

## 3. 接口契约（车侧视角，全部已代码核实）

### 3.1 车侧消费

| 话题 | 类型 | 约束 |
|---|---|---|
| `/planned_path` | `nav_msgs/Path` | 见 3.3 路径硬性要求 |

### 3.2 车侧发布（集成端可订阅用于监控/可视化）

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/ugv_pose` | `geometry_msgs/PoseStamped` | 30 Hz | 车的真实位姿（EKF 融合里程计，odom 系） |
| `/cmd_vel` | `geometry_msgs/Twist` | 20 Hz | 车内部消费，集成端只需监控不需发布 |

### 3.3 `/planned_path` 硬性要求（不满足车不动或乱动）

1. **逐点 frame_id 必须是 `drone_<UGV_ID>`**。planner_node 按任务分配的 drone id
   给每个 PoseStamped 标 `frame_id="drone_<did>"`（planner_node.py L874）；
   车上 follower 启动参数 `target_frame_id` 做逐点过滤。
   **联调时两边必须一致**：本次实物实验约定 **UGV 的 drone id = 4**，
   车侧启动已固定 `target_frame_id:=drone_4`，集成端确保 UGV planner 的
   任务分配 id 也是 4。
2. **刷新周期 < 2 秒**。follower 的看门狗 `path_timeout=2.0`：超时即回零速。
   planner 周期重发即可满足；**禁止一次性发布**（DDS 发现未建立时单条消息会被丢弃）。
3. **单位米，z 忽略**（UGV planner 发的 z=0）。
4. QoS：RELIABLE（planner 端已是）。
5. 坐标在**车的 odom 系**（见 2.2 对齐仪式后 ≈ world ENU）。

### 3.4 车侧内部链路（对接方无需关心，仅供排障）

`/planned_path` + `/ugv_pose` → ugv_path_follower（纯追踪，限速
max_linear_speed=0.5，落地建议 0.3）→ `/cmd_vel` → 厂商 odom_publisher → 电机。
到点 0.25 m 容差自停；位姿断 0.5 s / 路径断 2 s 自动零速。

## 4. 无人机侧要求（何泓林侧待办）

1. **定位链路打通**：PX4 EKF（室外融合 GNSS）→ MAVROS `local_position/pose`
   → ROS1/ROS2 桥 → 位姿桥 → `/drone_pose_external`
   （`swarm_interfaces/DroneStateArray`）。
2. **消息规范补全**（现有 RflySim 桥的已知缺陷，真机必须修）：
   - `header.stamp` 必填（用当前时刻）；
   - `header.frame_id = "world"`；
   - 坐标为 world ENU 米制（= PX4 local_position 原样平移即可，无需换算）；
   - `drone_id` 与任务分配一致；建议频率 ≥ 10 Hz。
3. **实验场地**：开阔、头顶无遮挡（楼旁/树下多路径误差可达 10 m+）。
4. 起飞前确认 MAVROS 有 GNSS fix（`rostopic echo /mavros/global_position/raw/fix`
   status 非 No fix）再开始实验。

## 5. 集成侧要求（跑 planner/scheduler 电脑待办）

1. **网络**：集成电脑、无人机机载电脑、小车树莓派连**同一个 WiFi 路由器**，
   三方 `ROS_DOMAIN_ID` 一致（默认 0 即可）。小车容器是 `--network host`，
   无 NAT 问题。验证：在小车上 `ros2 topic list` 能看到 `/planned_path`。
2. **UGV planner 实例**：`platform_type=1`（CAR），参考 swarm_sim.launch.py 中
   UGV planner 的写法（z=0.0）。
3. **任务分配链路**：scheduler → `/task_assignment` → UGV planner（id=4）；
   封控链路 enclosure_command_bridge 如启用，注意它目前忽略 layer 字段、
   monitor/block 都发 `/task_assignment`（已知问题，一机一车场景无影响）。
4. **路径频率**：确认 UGV planner 的 tick 周期性重发 `/planned_path`，
   间隔 < 2 s（默认即满足）。

## 6. 车侧启动顺序（联调日操作，已验证）

```bash
# 小车侧 3 个节点，全部在 MentorPi 容器内（ssh pi@192.168.2.100 后 docker exec -it MentorPi bash）

# ⓪ 厂商底盘节点（不启动车不会动！开机不自启，必须手动）
export MACHINE_TYPE=MentorPi_Tank   # 必带，否则 KeyError 崩溃
source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash
ros2 run controller odom_publisher --ros-args \
  --params-file /home/ubuntu/ros2_ws/src/driver/controller/config/calibrate_params.yaml \
  -p base_frame_id:=base_footprint -p odom_frame_id:=odom -p pub_odom_topic:=true

# ① 里程计中继
ros2 run ugv_base_driver ugv_odom_relay --ros-args -p restamp:=true

# ② 路径跟随（落地测试建议 max_linear_speed:=0.3）
ros2 run ugv_base_driver ugv_path_follower --ros-args \
  -p target_frame_id:=drone_4 -p max_linear_speed:=0.3
```

自查（另开终端）：
`ros2 topic hz /ugv_pose` 应 ~30 Hz；`ros2 topic info /cmd_vel`
应有 1 个订阅者（odom_publisher）。

## 7. 联调验收清单（按序执行，每步过了再下一步）

| 步骤 | 操作 | 预期 |
|---|---|---|
| 1 | 三机上同一路由器，小车 `ros2 topic list` | 能看到 `/planned_path`（集成端 planner 在跑） |
| 2 | 无人机上电，桥启动后小车 `ros2 topic echo /drone_pose_external --once` | 有数据，frame_id=world，坐标随无人机移动变化 |
| 3 | 按第 2.2 节做坐标对齐仪式，重启容器后重新拉起 3 个节点 | `/ugv_pose` 输出 (0,0) 附近、车头朝东时 yaw≈0 |
| 4 | 手推小车 1 m | `/ugv_pose` 的 x 增加约 1（验证方向一致性；若 x 减小说明车头没朝东） |
| 5 | 集成端手动发一条测试路径（车前方 2 m，逐点 frame_id=drone_4，-r 2 持续发） | 车前进约 2 m 后自停 |
| 6 | 无人机起飞悬停，走完整封控流程 | 人在探测范围内移动时 `/planned_path` 刷新、车追点 |

## 8. 已知坑速查（前车之鉴，联调日别再踩）

1. `ros2 topic pub --times 1` 单发会被 DDS 发现机制丢弃——路径类消息一律持续发。
2. 路径点 frame_id 标错（如 map）→ follower 报 "no usable waypoints" 车不动。
3. odom_publisher 不自启且不带 MACHINE_TYPE 必崩——见第 6 节步骤 ⓪。
4. `/set_odom` 只重置轮式原始里程计，EKF 的 `/odom` 不受影响；
   可靠归零 = `docker restart MentorPi`。
5. EKF 重启后短时位姿可能跳变，发路径前现读 `/ugv_pose` 现算。
6. 路径停止发布 2 秒后车自动零速（安全设计，不是 bug）。
7. joystick_control 节点会持续发零速到 `/controller/cmd_vel`（厂商设计，
   与我们的 `/cmd_vel` 不冲突，不要去杀它）。

## 9. 版本

- 2026-09-01 初版。车侧依据：ugv_base_driver PR #10（commit 8fe59e0/bc74964），
  实车台架 + 闭环测试记录见 docs/operations/ugv_real_deployment.md 第 11 节。
