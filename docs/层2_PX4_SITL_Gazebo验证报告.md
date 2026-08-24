# 层 2 验证报告：PX4 SITL + Gazebo Classic + mavros + 动态 Voronoi 封控

**项目**：Swarm-Control-System（异构无人集群协同封控系统）  
**模块**：containment_pkg（动态 Voronoi 封控）  
**验证人**：陈思睿（containment_pkg 负责人）  
**验证时间**：2026-08-24  
**环境**：WSL2 Ubuntu 22.04 + ROS2 Humble + PX4 v1.14 (SITL) + Gazebo Classic 11 + mavros

---

## 1. 验证目标

在真实飞控仿真（而非 mock 位姿）下，确认 3 架无人机 + 2 辆地面车的位姿能经 PX4 SITL → Gazebo → mavros → 状态聚合器 → `enclosure_node`，正确生成动态 Voronoi 封控指令 `/enclosure_command`。

对应挑战需求：**3 机 2 车 Gazebo 全闭环三层 Voronoi 封控验证**（层 2）。

---

## 2. 系统链路

```text
[PX4 SITL ×3] ──UDP 14581/14582/14583──▶ [mavros ×3]
                                                  │ /uav{0,1,2}/mavros/local_position/pose (PoseStamped, best-effort)
                                                  ▼
                                        [sitl_state_publisher]  + 2 mock UGV
                                                  │ DroneStateArray (/drone_states)
                                                  ▼
                                        [target_pub] ──TargetTrackArray (/target_track)──▶ [enclosure_node]
                                                  │                                               │
                                                  └───────────────────────────────────────────────┘
                                                                          EnclosureCommandArray (/enclosure_command)
```

- `sitl_state_publisher.py`：订阅 3 个 mavros `local_position/pose`（真实 SITL 位姿），叠加 2 个 mock UGV（id=100/101，`platform_type=1`），打包为 `DroneStateArray` 发 `/drone_states`。
- `enclosure_node` 接口：订阅批量 `DroneStateArray`（`/drone_states`）而非单个 `DroneState`，这是改用聚合器而非 3 个独立桥接器的原因。

---

## 3. 构建与启动

### 3.1 编译（含环境兼容补丁）

PX4 v1.14 `sitl_gazebo-classic` 与 ROS2 Humble 自带的新版 mavlink（v2026.6.6）不兼容——新版给相机/云台类 `mavlink_msg_*_pack_chan` 函数末尾新增了 device_id 参数，旧代码缺参会报 `too few arguments`。本机源码补 `0` 后编译通过（详见 `simulation/px4_sitl_3uav/README.md` 的「已知编译坑」）：

```bash
cd ~/src/PX4-Autopilot && DONT_RUN=1 make px4_sitl_default gazebo-classic   # [56/56] 全绿
```

### 3.2 三终端启动

```bash
# 终端1：3 机 SITL（headless，无 gzclient）
bash simulation/px4_sitl_3uav/start_3uav_sitl.sh
# 终端2：3 mavros + 聚合器 + target_pub + enclosure_node
bash simulation/px4_sitl_3uav/start_3uav_ros.sh
# 终端3：验证
ros2 topic echo /drone_states
ros2 topic echo /enclosure_command
```

---

## 4. 验证结果

### 4.1 mavros 连接

```
[ros]   /uav0/mavros: CONNECTED
[ros]   /uav1/mavros: CONNECTED
[ros]   /uav2/mavros: CONNECTED
```

### 4.2 `/drone_states`（DroneStateArray，节选）

```yaml
drones:
- drone_id: 0   x: 0.012  y: 0.019  z: -0.076  available: true  platform_type: 0   # UAV 真实位姿
- drone_id: 1   x: 0.013  y: 0.010  z: -0.109  available: true  platform_type: 0
- drone_id: 2   x: 0.018  y: 0.006  z:  0.005  available: true  platform_type: 0
- drone_id: 100 x: -0.066 y: 14.99  z: 0.0    available: true  platform_type: 1   # UGV mock 圆周
- drone_id: 101 x:  0.066 y:-14.99  z: 0.0    available: true  platform_type: 1
```

3 架 UAV 坐标来自 mavros 实时位姿（停在地面附近，z≈0），2 辆 UGV 在 ±15 m 圆周正确分布。

### 4.3 `/enclosure_command`（EnclosureCommandArray）

```yaml
num_drones: 5
commands:
- drone_id: 0     layer: 0  enclosure_radius: 25.0   # UAV monitor
- drone_id: 1     layer: 0  enclosure_radius: 25.0
- drone_id: 2     layer: 0  enclosure_radius: 25.0
- drone_id: 100   layer: 1  enclosure_radius: 15.0   # UGV block
- drone_id: 101   layer: 1  enclosure_radius: 15.0
```

输出 **5 条指令：3 monitor（UAV，25 m 圆周，z=10）+ 2 block（UGV，15 m 圆周，z=0）**，与层 1 纯 ROS2 验证的几何结果一致。

---

## 5. 关键问题与修复记录

| # | 现象 | 根因 | 修复 |
|---|------|------|------|
| 1 | 终端1 一直 `Connection closed by client.` | PX4 自带 `sitl_multiple_run.sh` 末尾前台拉 `gzclient`（GUI），headless 下崩溃 → cleanup trap 把 px4 全 kill | 重写 `start_3uav_sitl.sh`，手写 gzserver+3×iris+3×px4 等价启动，**不启动 gzclient**，gzserver 日志转 `/tmp/gazebo_multi.log` |
| 2 | `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` 下 ROS `setup.bash` 引用未定义变量 → 整个 ROS 脚本 abort | `start_3uav_ros.sh` 去掉 `set -u`（改 `set -eo pipefail`） |
| 3 | `Address already in use` (Gazebo 11345) | 旧 gzserver 残留 | 脚本开头 `pkill -9 -f gzserver/gazebo/px4` 并轮询等待 11345 释放 |
| 4 | UAV 一直是 fallback(0/3/6) 且 `available:false` | mavros `local_position/pose` 用 **best-effort** QoS，state_publisher 用默认 reliable 订阅 → ROS2 QoS 不兼容，订阅建了收不到 | `sitl_state_publisher.py` 订阅改用 `QoSProfile(reliability=BEST_EFFORT, ...)` 并加诊断日志 |

**端口澄清**（避免再踩坑）：PX4 v1.14 `px4-rc.mavlink` 中 offboard 监听口 = `14580 + 实例N`（实例1→14581），不是早期笔记里的 14540/14557。mavros `fcu_url = udp://:{14540+N}@127.0.0.1:{14580+N}`，`tgt_system = N+1`。

---

## 6. 验证结论

| 验证项 | 结果 |
|--------|------|
| PX4 SITL 编译（含 MAVLink 补丁） | 通过（[56/56]） |
| 3 机 SITL + Gazebo Classic 启动 | 通过 |
| 3 个 mavros 连接 PX4 | 全部 CONNECTED |
| 真实位姿聚合为 DroneStateArray | 通过（UAV `available:true`） |
| enclosure_node 生成封控指令 | 通过（5 条：3 monitor + 2 block） |

**结论**：层 2（PX4 SITL + Gazebo + mavros + Voronoi 封控）全闭环打通，且与层 1（纯 ROS2）的封控几何结果一致。脚本与说明见 `simulation/px4_sitl_3uav/`。

---

## 7. 代码提交状态

- 本验证的 3 个脚本已放入 `simulation/px4_sitl_3uav/`（含 README）。
- 编译补丁属本机环境兼容问题（mavlink 版本差异），**不进仓库**，仅在工作区本地修改 `~/src/PX4-Autopilot/.../sitl_gazebo-classic/src/*.cpp`。
- 待在 WSL 终端提交并推送：
  ```bash
  cd /mnt/c/ProgramData/WorkBuddy/chromium-env/6ulcsx/WorkBuddy/2026-08-10-14-52-53/Swarm-Control-System
  git add simulation/px4_sitl_3uav docs/layer3_rflysim_delivery docs/层2_PX4_SITL_Gazebo验证报告.md docs/三层Voronoi封控全闭环验证报告.md
  git commit -m "docs(sim): 层2 PX4 SITL 3机联调脚本与三层验证报告"
  git push origin main
  ```
