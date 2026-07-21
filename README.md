# Swarm-Control-System

## 项目简介

**Swarm-Control-System** 是面向第十九届 **“挑战杯”全国大学生课外学术科技作品竞赛——揭榜挂帅专项赛** 的异构无人集群协同封控系统。

本项目围绕赛题要求，面向无人机、无人车等异构无人平台，构建统一通信、统一调度、协同控制的软件系统，实现目标感知、路径规划、任务分配、协同封控及仿真验证。

系统采用 **ROS2** 作为通信中间件，以 **PX4** 为无人机飞控平台，以 **RflySim** 为数字孪生仿真平台，按照模块化方式进行开发。

---

# 系统总体架构

系统主要由以下几个部分组成：

* 感知模块（目标检测）
* 跟踪模块（目标跟踪）
* 路径规划模块
* 任务调度模块
* 协同决策模块
* 控制执行模块
* PX4 飞控系统
* RflySim 仿真平台

各模块通过 ROS2 Topic 完成数据交互，实现空地协同控制。

---

# 技术路线

本项目主要采用以下技术：

* ROS2 Humble
* Ubuntu 22.04
* PX4 Autopilot
* RflySim
* YOLO（目标检测）
* DeepSORT（目标跟踪）
* A* / D* Lite（路径规划）
* Voronoi（协同封控）
* Python
* C++

---

# 项目目录

```text
Swarm-Control-System/

├── config/            系统配置文件
├── docs/              项目文档
│   ├── architecture/  系统架构设计
│   ├── interface/     ROS2接口设计
│   └── report/        项目文档
│
├── modules/           算法模块
├── ros2_ws/           ROS2工作空间
├── scripts/           自动化脚本
├── simulation/        仿真环境
├── videos/            演示视频
├── web/               Web可视化
└── README.md
```

---

# 当前开发进度

## 第一阶段（已完成）

* [x] GitHub 仓库建立
* [x] Ubuntu 22.04 开发环境部署
* [x] ROS2 Humble 环境搭建
* [x] ROS2 环境验证
* [x] 系统总体架构设计
* [x] ROS2 节点设计（V1）
* [x] Topic 接口设计（V1）

## 第二阶段（已完成 + 进行中）

* [x] 建立 ROS2 Workspace (`ros2_ws/`)
* [x] 创建 ROS2 Package（`swarm_interfaces`、`perception_pkg`）
* [x] 定义自定义 Message（`TargetTrack`、`TargetTrackArray`、`TaskAssignment`）
* [x] **`tracker_node` 上线**：YOLOv8 + DeepSORT / BoT-SORT 输出打包成
      `TargetTrackArray` 发布到 `/target_track`（详见
      [`perception_pkg`](../ros2_ws/src/perception_pkg/README.md) 与
      [`Topic接口设计V2`](../docs/interface/Topic接口设计V2.md)）
* [ ] 完成节点通信测试（`tracker_node` → `planner_node`）
* [ ] PX4 环境部署
* [ ] RflySim 联调

## 第三阶段（规划中）

* [ ] YOLO 目标检测接入 ✓（`perception_pkg/tracker_node` 已内嵌 YOLOv8）
* [ ] DeepSORT 目标跟踪 ✓（`perception_pkg/tracker_node` 支持
      `botsort` / `deepsort` / `deepsort_cascade` 三种跟踪器）
* [ ] A* / D* Lite 路径规划
* [ ] 多无人机协同调度
* [ ] 无人车协同控制
* [ ] 空地协同封控算法
* [ ] 系统联调与仿真验证

---

# 开发规范

* 所有代码统一提交至 GitHub 仓库。
* 文档统一维护于 `docs` 目录。
* ROS2 Package 统一放置于 `ros2_ws/src`。
* Topic 与 Message 接口统一管理，避免成员自行修改。
* 新功能建议通过独立分支开发，测试完成后再合并至主分支。

---

# 开发环境

| 软件                | 版本     |
| ----------------- | ------ |
| Ubuntu            | 22.04  |
| ROS2              | Humble |
| Python            | 3.10   |
| Git               | Latest |
| Oracle VirtualBox | Latest |

---

# 项目说明

目前项目处于系统设计与开发环境搭建阶段。

后续将逐步完成 ROS2 节点开发、PX4 飞控接入、RflySim 仿真验证以及异构无人集群协同封控算法实现。

---

# 参考资料

* ROS2 官方文档
* PX4 官方文档
* RflySim 官方资料
* 挑战杯揭榜挂帅专项赛赛题要求
# Swarm-Control-System
