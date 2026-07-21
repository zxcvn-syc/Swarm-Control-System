# Topic 接口设计 V2.0

> V2.0 在 V1 的基础上引入 `TargetTrackArray`（每帧单条消息，循环 N 个目标）
> 用于把 `cv_tracking_demo` 的 YOLOv8 + DeepSORT / BoT-SORT 输出接入
> `tracker_node` 的 ROS2 Topic。

> **V3 (2026-07-20) 更新**：增强 `TargetTrack` 字段，新增 `EnclosureTargetArray`
> 接口用于封控组对接。详见 [感知组优化工作总结](./感知组优化工作总结.md)。

## 1. 设计说明

系统采用 ROS2 Topic 完成各节点之间的数据通信，实现感知、规划、调度和控制模块解耦。

`/target_track` 现在承载 `swarm_interfaces/TargetTrackArray`：每帧一条消息，
内部包含当帧所有确认目标（DeepSORT ID + 像素坐标 + 像素速度），下游节点按帧消费。

---

## 2. Topic 接口 (V3)

| Topic 名称 | 消息类型 | 发布节点 | 订阅节点 | 数据内容 |
| ---------------- | ------------------------------------------- | -------------- | -------------- | ------- |
| `/camera/image` | `sensor_msgs/Image` | Camera | tracker_node | 图像数据 |
| `/target_track` | `swarm_interfaces/TargetTrackArray` | tracker_node | 调度组 | 目标轨迹 + 预测 |
| `/enclosure_targets` | `swarm_interfaces/EnclosureTargetArray` | tracker_node | 封控组 | 目标坐标 + 轨迹预测 |
| `/environment_map` | nav_msgs/OccupancyGrid | Map | planner_node | 环境地图 |
| `/path_plan` | `swarm_interfaces/PathPlan`（规划中） | planner_node | scheduler_node | 路径规划结果 |
| `/mission_plan` | `swarm_interfaces/MissionPlan`（规划中） | scheduler_node | decision_node | 任务分配结果 |
| `/decision_result` | `swarm_interfaces/DecisionResult`（规划中） | decision_node | control_node | 封控策略 |
| `/uav_cmd` | `mavros_msgs/PositionTarget` 或自定义 | control_node | PX4 | 无人机控制命令 |
| `/ugv_cmd` | 自定义 | control_node | 无人车 | 无人车控制命令 |
| `/status_feedback` | `swarm_interfaces/PlatformStatus`（规划中） | PX4、无人车 | scheduler_node | 平台状态反馈 |

> **V3 新增**：`/enclosure_targets` 用于封控组（陈思睿的 Voronoi 围控算法）

---

## 3. TargetTrackArray 消息

```text
std_msgs/Header header
TargetTrack[]    tracks
uint32           frame_idx
```

| 字段 | 类型 | 说明 |
| -------------- | ----------------------------- | ------------------------------------------------------- |
| `header` | `std_msgs/Header` | 时间戳 + `frame_id`（相机坐标系，例如 `camera_optical_frame`） |
| `tracks` | `TargetTrack[]` | 当帧所有已确认目标的轨迹 |
| `frame_idx` | `uint32` | 单调递增的帧编号（自节点启动起） |

### TargetTrack 单目标字段 (V3 增强)

```text
# 基础字段
uint32   target_id
float64  x
float64  y
float64  vx
float64  vy

# V3 新增调度字段
float32  confidence
uint8    cls
bool     is_confirmed
float32  speed
uint8    motion_mode

# V3 预测轨迹 (5步)
float32[5] pred_x
float32[5] pred_y
float32[5] pred_conf
```

| 字段 | 类型 | 说明 |
| ----------- | ---------- | ---------------------------------------------------------- |
| `target_id` | `uint32` | DeepSORT / BoT-SORT 分配的目标 ID（节点内唯一） |
| `x`, `y` | `float64` | 像素坐标（图像平面内目标框中心），与 cvtrack 输出一致 |
| `vx`, `vy` | `float64` | 像素/秒（Kalman 估计速度） |
| `confidence` | `float32` | **V3新增** 检测置信度 (0.0-1.0) |
| `cls` | `uint8` | **V3新增** 目标类别 (COCO风格) |
| `is_confirmed` | `bool` | **V3新增** 是否已确认 |
| `speed` | `float32` | **V3新增** 速度大小 |
| `motion_mode` | `uint8` | **V3新增** 运动模式 (0=未知, 1=静止, 2=慢速, 3=快速) |
| `pred_x/y` | `float32[5]` | **V3新增** 未来5步预测位置 |
| `pred_conf` | `float32[5]` | **V3新增** 预测置信度 |

> 坐标约定：发布的是**像素坐标**。下游需要世界坐标时，由 planner / control 节点
> 自行完成相机标定 / IPM / 单应性投影。

---

## 4. EnclosureTargetArray 消息 (V3 新增)

```text
std_msgs/Header header
uint32           frame_idx
EnclosureTarget[] targets

# 无人机位置
float32[8] drone_x
float32[8] drone_y
uint8      num_drones

# 围控参数
float32    enclosure_radius
float32    min_enclosure_dist
```

### EnclosureTarget 单目标字段

```text
uint32     target_id
float64    x
float64    y
float32    speed
uint8      motion_mode
float32    confidence

# 包围盒
float32    box_x1, box_y1, box_x2, box_y2

# 预测轨迹
float32[5] pred_x, pred_y

# 历史轨迹
float32[10] history_x, history_y
```

---

## 5. 数据流

```
相机图像 ─► tracker_node (YOLOv8 + DeepSORT/BoT-SORT)
                       │
        ┌──────────────┴──────────────┐
        ▼                              ▼
/target_track                    /enclosure_targets
(TargetTrackArray)              (EnclosureTargetArray)
        │                              │
        ▼                              ▼
   调度组节点                    封控组节点
(马子越使用)                  (陈思睿使用)
```

---

## 6. 接入示例

订阅 `TargetTrackArray` 并打印前 3 个目标：

```python
from swarm_interfaces.msg import TargetTrackArray
import rclpy
from rclpy.node import Node


class PrintTargets(Node):
    def __init__(self):
        super().__init__('print_targets')
        self.sub = self.create_subscription(
            TargetTrackArray, '/target_track', self.cb, 10)

    def cb(self, msg: TargetTrackArray) -> None:
        self.get_logger().info(
            'frame_idx=%d n_tracks=%d frame_id=%s',
            msg.frame_idx, len(msg.tracks), msg.header.frame_id,
        )
        for t in msg.tracks[:3]:
            self.get_logger().info(
                '  id=%d pos=(%.1f, %.1f) vel=(%.2f, %.2f) speed=%.1f mode=%d',
                t.target_id, t.x, t.y, t.vx, t.vy, t.speed, t.motion_mode,
            )


rclpy.init()
rclpy.spin(PrintTargets())
```

发布 `tracker_node`：

```bash
# 方式 1：从本地视频读取
ros2 launch perception_pkg tracker_node.launch.py \
    mode:=video \
    video_source:=/data/pexels_aerial_2034115.mp4 \
    tracker.kind:=deepsort_cascade \
    detector.weights:=/data/visdrone_yolov8s.pt

# 方式 2：从 ROS2 图像话题读取
ros2 launch perception_pkg tracker_node.launch.py \
    mode:=topic \
    image_topic:=/uav/camera/image \
    frame_id:=uav_body

# 方式 3：启用封控组接口
ros2 run perception_pkg tracker_node --ros-args \
    -p enclosure.enabled:=true \
    -p enclosure.topic:=/enclosure_targets
```

---

## 7. V3 感知优化特性

### 自适应卡尔曼滤波
- `KalmanCV2DAdaptive`：4状态自适应过程/观测噪声
- `KalmanBoTAdaptive`：8状态带运动模式检测

### 轨迹预测
- `TrajectoryPredictor`：多步预测（默认10步）+ 置信度衰减
- `TrajectorySmoother`：RTS平滑算法
- `TrajectoryAnalyzer`：运动模式分类、曲率分析、出框预估

### 跟踪稳定性
- `IdentityManager`：ID管理 + 丢失重激活
- `OcclusionHandler`：遮挡检测 + 自适应门限
- `AppearanceMemory`：外观特征时间一致性

详见 [感知组优化工作总结](./感知组优化工作总结.md)

---

## 8. 后续计划

### ✅ 已完成 (V3)
- [x] `TargetTrack` 字段增强（speed, motion_mode, predictions）
- [x] `EnclosureTargetArray` 接口新增
- [x] 自适应卡尔曼滤波
- [x] 轨迹预测模块

### 📋 待完成
- [ ] `PathPlan` / `MissionPlan` / `DecisionResult` 等剩余消息定义
- [ ] 多相机 ID 编码 (`camera_id * 1000 + track_id`)
- [ ] 接 PX4 相机话题验证
- [ ] 单元测试完善
- [ ] 轨迹融合（多传感器）
