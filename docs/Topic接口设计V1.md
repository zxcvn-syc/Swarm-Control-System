# Topic接口设计 V1.0

## 1. 设计说明

系统采用 ROS2 Topic 完成各节点之间的数据通信，实现感知、规划、调度和控制模块解耦。

---

## 2. Topic接口

| Topic名称          | 发布节点           | 订阅节点           | 数据内容    |
| ---------------- | -------------- | -------------- | ------- |
| /camera/image    | Camera         | detector_node  | 图像数据    |
| /target_info     | detector_node  | tracker_node   | 目标检测结果  |
| /target_track    | tracker_node   | planner_node   | 目标轨迹    |
| /environment_map | Map            | planner_node   | 环境地图    |
| /path_plan       | planner_node   | scheduler_node | 路径规划结果  |
| /mission_plan    | scheduler_node | decision_node  | 任务分配结果  |
| /decision_result | decision_node  | control_node   | 封控策略    |
| /uav_cmd         | control_node   | PX4            | 无人机控制命令 |
| /ugv_cmd         | control_node   | 无人车            | 无人车控制命令 |
| /status_feedback | PX4、无人车        | scheduler_node | 平台状态反馈  |

---

## 3. 数据流

相机图像

↓

目标检测

↓

目标跟踪

↓

路径规划

↓

任务调度

↓

封控决策

↓

控制执行

↓

无人机 / 无人车

---

## 4. 后续计划

第二周根据 ROS2 Workspace 建立实际消息类型（msg），完善 Topic 接口，并完成节点通信验证。
