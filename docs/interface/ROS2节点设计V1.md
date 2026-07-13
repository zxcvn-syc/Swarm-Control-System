# ROS2节点设计 V1.0

## 1. 设计目标

为了实现异构无人集群协同封控系统的模块化开发，系统采用 ROS2 作为通信中间件，将各功能模块设计为独立节点，通过 Topic 完成数据交互，提高系统的可扩展性、可维护性和协同效率。

---

## 2. 节点划分

### （1）detector_node（目标检测节点）

功能：

负责接收无人机相机图像，利用 YOLO 模型完成目标检测。

输入：

* /camera/image

输出：

* /target_info

---

### （2）tracker_node（目标跟踪节点）

功能：

根据检测结果进行目标关联和连续跟踪，输出目标轨迹。

输入：

* /target_info

输出：

* /target_track

---

### （3）planner_node（路径规划节点）

功能：

根据目标位置及环境地图进行路径规划，为无人机和无人车生成最优路径。

输入：

* /target_track
* /environment_map

输出：

* /path_plan

---

### （4）scheduler_node（任务调度节点）

功能：

根据无人平台状态完成任务分配，实现多机协同。

输入：

* /path_plan

输出：

* /mission_plan

---

### （5）decision_node（封控决策节点）

功能：

根据任务规划结果生成动态封控策略，实现空地协同决策。

输入：

* /mission_plan

输出：

* /decision_result

---

### （6）control_node（控制执行节点）

功能：

负责向 PX4 飞控和无人车控制器发送控制指令，完成任务执行。

输入：

* /decision_result

输出：

* /uav_cmd
* /ugv_cmd

---

## 3. 节点运行流程

Camera

↓

detector_node

↓

tracker_node

↓

planner_node

↓

scheduler_node

↓

decision_node

↓

control_node

↓

PX4 / 无人车

---

## 4. 后续工作

第二周将建立 ROS2 Workspace，创建各节点对应 Package，并完成节点通信开发。

