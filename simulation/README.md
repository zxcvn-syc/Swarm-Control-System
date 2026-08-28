# 仿真与验证环境

本目录汇总 Swarm-Control-System 的三层仿真验证方案，从纯 ROS2 数据闭环到 PX4 SITL 物理仿真再到 RflySim3D 视觉封控。

## 目录结构

| 路径 | 内容 | 层级 |
|------|------|------|
| `px4_sitl_3uav/` | 3 机 PX4 SITL + Gazebo Classic（headless）+ 3 mavros + 封控联调脚本与 README | **层 2** |
| `lidar_swarm/` | 基础 MATLAB：3 UAV + 3 UGV LiDAR 量测、分类、锁定、鬼影剔除与测试 | 离线感知验证 |
| `maps/` | Gazebo 世界/地图文件 | 层 2 支撑 |
| `../docs/层2_PX4_SITL_Gazebo验证报告.md` | 层 2 详细验证报告 | 层 2 |
| `../docs/全链路数据闭环验证报告.md` | 层 1 纯 ROS2 数据闭环报告 | **层 1** |
| `../docs/三层Voronoi封控全闭环验证报告.md` | 三层统一验证报告（总入口） | 总览 |
| `../docs/layer3_rflysim_delivery/` | 层 3 何泓林 RflySim3D 交付证据 | **层 3** |

## 三层验证速览

- **层 1（纯 ROS2）**：`ros2 launch containment_pkg full_loop_demo.launch.py`（3 机 2 车 mock 位姿）→ `/enclosure_command` 出 5 条封控指令。
- **层 2（PX4 SITL）**：见 `px4_sitl_3uav/README.md`。3 架真实 PX4 SITL + mavros → 真实位姿聚合 → 封控，验证几何正确。
- **层 3（RflySim3D）**：视觉触发空地协同封控（在合作机器运行，证据见 `../docs/layer3_rflysim_delivery/`）。

> 仓库根目录另有 `scripts/bootstrap_px4_sitl_ubuntu.sh`（安装 Gazebo+PX4 v1.14+MAVROS）与 `ros2_ws/launch/` 下的纯 ROS2 启动文件。注意：原 `scripts/run_px4_sitl.sh` 仅支持 `NUM_UAV=1`，3 机扩展请用 `px4_sitl_3uav/`。

LiDAR 离线仿真运行方式和边界见 [`lidar_swarm/README.md`](lidar_swarm/README.md)。它是二维水平面量测级验证，不替代三维避障的 PX4/Gazebo 验收。
