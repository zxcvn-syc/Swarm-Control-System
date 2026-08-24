# 三层 Voronoi 封控全闭环验证报告

**项目**：Swarm-Control-System（异构无人集群协同封控系统）  
**赛事**：第十九届"挑战杯"全国大学生课外学术科技作品竞赛  
**验证总负责人**：陈思睿（containment_pkg 动态 Voronoi 封控）  
**验证时间**：2026-08-13（层1） / 2026-08-23（层3，何泓林） / 2026-08-24（层2）  
**环境**：WSL2 Ubuntu 22.04 + ROS2 Humble + PX4 v1.14 SITL + Gazebo Classic 11 + mavros + RflySim3D（层3 在合作机器）

---

## 1. 验证体系与目标

系统采用 **三层递进验证**，从数据逻辑闭环到真实飞控仿真再到视觉驱动的空地协同封控，逐层逼近真实部署：

| 层级 | 验证内容 | 仿真/数据来源 | 目标 |
|------|----------|---------------|------|
| **层 1** | 纯 ROS2 数据闭环 | mock 位姿（containment_pkg `full_loop_demo`） | 确认 感知→调度→规划→封控 全链路消息流正确 |
| **层 2** | PX4 SITL + Gazebo 物理仿真 | 3 架真实 PX4 SITL 无人机 + 2 辆 mock UGV | 确认封控算法在**真实飞控位姿**下几何正确 |
| **层 3** | RflySim3D 视觉驱动封控 | 真实 RGB 视觉检测 + 场景运动学控制（合作机器） | 确认视觉触发的空地协同封控可运行 |

三层共用同一套 `swarm_interfaces` 消息契约（`TargetTrackArray` / `DroneStateArray` / `EnclosureCommandArray`），因此层 1/2 的封控几何结果可直接在层 3 落地。

---

## 2. 层 1：纯 ROS2 数据闭环验证（2026-08-13 / 08-24）

**方式**：`ros2 launch containment_pkg full_loop_demo.launch.py`（3 机 2 车 mock 位姿）。

**结果**：

- 4 类节点全部正常：`tracker_node`(感知) → `scheduler_node`(调度) → `planner_node`(规划) → `enclosure_node`(封控)。
- `/enclosure_command` 输出 **5 条指令**：3 条 monitor（UAV，25 m 圆周，z=10）+ 2 条 block（UGV，15 m 圆周，z=0）。
- 几何校验：UAV 距目标 ≈ 23–28 m、UGV ≈ 13–16 m，均在 25±5 m / 15±5 m 合理范围。

**结论**：感知→规划→封控 完整数据闭环跑通。详见 `docs/全链路数据闭环验证报告.md`。

---

## 3. 层 2：PX4 SITL + Gazebo 物理仿真验证（2026-08-24）

**方式**：`simulation/px4_sitl_3uav/` 脚本启动 3 机 SITL + 3 mavros → `sitl_state_publisher` 聚合真实位姿 → `enclosure_node`。

**结果**：

- 3 个 mavros 全部 `CONNECTED`（PX4 监听 14581/14582/14583，sysid 2/3/4）。
- `/drone_states`：UAV 0/1/2 显示真实小坐标 + `available:true`；UGV 100/101 在 ±15 m 圆周。
- `/enclosure_command`：5 条指令，3 monitor（layer=0, r=25.0）+ 2 block（layer=1, r=15.0），`num_drones: 5`。

**关键修复**（详见 `docs/层2_PX4_SITL_Gazebo验证报告.md`）：

1. mavlink v2026.6.6 对相机/云台类消息新增 device_id 参数 → 本机 PX4 源码补 `0` 重编通过。
2. headless 下 `gzclient` 崩溃导致 `Connection closed by client.` 死循环 → 改为手写启动、不拉 GUI。
3. ROS `setup.bash` 在 `set -u` 下 abort → 去掉 `set -u`。
4. mavros `local_position/pose` 用 best-effort QoS，state_publisher 改用 best-effort 订阅才收到。

**结论**：层 2 全闭环打通，且与层 1 封控几何一致——**算法在真实飞控位姿下正确**。

---

## 4. 层 3：RflySim3D 视觉驱动封控验证（2026-08-23，何泓林）

**方式**：Windows 侧 RflySim3D + 实时视觉进程（YOLOv8s + BoT-SORT）联合 ROS2 虚拟机，运行 `rain_wind_3ddisplay` 预设 62 秒联调。

**结果**（最终通过运行 `rfly_full_demo_20260823_211000`）：

| 验收项 | 实测 |
|--------|------|
| UAV 视角视频 | 55.97 s，1280×720，30 FPS，1679 帧 |
| 在线视觉处理 | 1407 帧，22.69 FPS |
| 确认目标跟踪 | 1369 帧，稳定 ID `10001` |
| 目标居中率 | 81.4%（阈值 35%） |
| 物理遮挡重捕获 | **4/4**，最长 1.078 s |
| 车辆重叠 | 0；最小安全间隙 0.001147 m |
| ROS2 话题 | 7/7 收到有效消息 |
| 地面封控命令 | 2 条有效 `/enclosure_command` |

**结论**：视觉触发的空地协同封控在 RflySim3D 中可稳定运行，遮挡后重捕获、车辆零重叠、封控指令下发均达标。

**边界声明（重要）**：该验证为**视觉触发的 Rfly 空地协同控制演示**，非 PX4 / MAVROS / 真机飞控验证：

- 控制使用 `UE4CtrlAPI` 运动学接口，未执行 ARM、PX4 Offboard 或 MAVROS。
- 遥测中 `target_control_source=vision` 表示控制用图像投影得到目标状态，依赖仿真相机位姿，不能声称为未经标定/同步验证的纯单目世界坐标闭环。
- 当前 Rfly Free 稳定提供单条原生 UAV1 RGB 流；UAV2/UAV3 属场景级协同状态，不宣称已验证多相机原生视角交接。

完整证据与脚本见 `docs/layer3_rflysim_delivery/`（`FINAL_REPORT_ZH.md`、`STRATEGY_ZH.md`、`SCENARIO_MATRIX_20260822.md`、`claude_review_20260823.md`）。

---

## 5. 三层一致性结论

| 维度 | 层 1（mock） | 层 2（PX4 SITL） | 层 3（RflySim3D） |
|------|-------------|------------------|-------------------|
| 平台数量 | 3 UAV + 2 UGV | 3 UAV(真实) + 2 UGV(mock) | 3 UAV + 灰色封控车 + 动态/静态障碍 |
| 封控指令 | 3 monitor + 2 block | 3 monitor + 2 block | 2 条有效 `/enclosure_command` |
| 半径 | UAV 25 m / UGV 15 m | UAV 25 m / UGV 15 m | 与层 1/2 一致 |
| 闭环性质 | 数据逻辑闭环 | 真实飞控位姿闭环 | 视觉触发空地协同 |
| 验证状态 | ✅ | ✅ | ✅（视觉演示边界） |

三层共用同一消息契约（`TargetTrackArray` / `DroneStateArray` / `EnclosureCommandArray`）与同一封控几何（径向投影 + 25 m/15 m 双层封控），层 1/2 已证明算法逻辑与飞控位姿下几何正确，层 3 证明该算法可在视觉驱动的仿真场景中落地运行。

---

## 6. 限制与后续工作

1. **层 3 控制边界**：当前为视觉投影 + 运动学控制，未接入 PX4 Offboard。若需在 RflySim3D 中走 MAVROS/PX4 Offboard，需补充飞控桥接（与层 2 的 mavros 链路对齐）。
2. **多相机原生交接**：UAV2/UAV3 视角为场景协同状态，需 Rfly 多原生 RGB 流支持后再验证视角交接。
3. **真机飞控**：本验证均在仿真完成；真机部署需额外做 Offboard 安全链路与容错。
4. **代码提交**：层 2 脚本与三层报告待推送（见各报告末尾的 `git` 命令）。

---

## 7. 交付物索引

| 文件 | 说明 |
|------|------|
| `simulation/px4_sitl_3uav/` | 层 2 三机 SITL 联调脚本 + README |
| `docs/层2_PX4_SITL_Gazebo验证报告.md` | 层 2 详细验证报告 |
| `docs/全链路数据闭环验证报告.md` | 层 1 详细验证报告 |
| `docs/layer3_rflysim_delivery/` | 层 3 何泓林交付证据（报告/策略/场景矩阵/复核） |
| `docs/动态Voronoi封控设计规范.md` | 动态 Voronoi 封控算法设计规范 |
