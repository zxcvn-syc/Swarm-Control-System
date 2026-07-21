# Topic 接口设计 V2.0

> V2.0 在 V1 的基础上引入 `TargetTrackArray`（每帧单条消息，循环 N 个目标）
> 用于把 `cv_tracking_demo` 的 YOLOv8 + DeepSORT / BoT-SORT 输出接入
> `tracker_node` 的 ROS2 Topic。其他 Topic 接口与 V1 完全一致。

## 1. 设计说明

系统采用 ROS2 Topic 完成各节点之间的数据通信，实现感知、规划、调度和控制模块解耦。

`/target_track` 现在承载 `swarm_interfaces/TargetTrackArray`：每帧一条消息，
内部包含当帧所有确认目标（DeepSORT ID + 像素坐标 + 像素速度），下游节点按帧消费。

---

## 2. Topic 接口

| Topic 名称          | 消息类型                                       | 发布节点           | 订阅节点           | 数据内容    |
| ---------------- | ------------------------------------------- | -------------- | -------------- | ------- |
| `/camera/image`  | `sensor_msgs/Image`                          | Camera         | tracker_node   | 图像数据    |
| `/target_track`  | `swarm_interfaces/TargetTrackArray`          | tracker_node   | planner_node   | 目标轨迹    |
| `/environment_map` | nav_msgs/OccupancyGrid                    | Map            | planner_node   | 环境地图    |
| `/path_plan`     | `swarm_interfaces/PathPlan`（规划中）            | planner_node   | scheduler_node | 路径规划结果  |
| `/mission_plan`  | `swarm_interfaces/MissionPlan`（规划中）         | scheduler_node | decision_node  | 任务分配结果  |
| `/decision_result` | `swarm_interfaces/DecisionResult`（规划中）   | decision_node  | control_node   | 封控策略    |
| `/uav_cmd`       | `mavros_msgs/PositionTarget` 或自定义           | control_node   | PX4            | 无人机控制命令 |
| `/ugv_cmd`       | 自定义                                       | control_node   | 无人车            | 无人车控制命令 |
| `/status_feedback` | `swarm_interfaces/PlatformStatus`（规划中）   | PX4、无人车        | scheduler_node | 平台状态反馈  |

> 字段以 `swarm_interfaces/TargetTrackArray` 为准，下游订阅节点不应再依赖旧版
> 单目标 `TargetTrack` 流式发布。

---

## 3. TargetTrackArray 消息

```text
std_msgs/Header header
TargetTrack[]    tracks
uint32           frame_idx
```

| 字段           | 类型                          | 说明                                                    |
| -------------- | ----------------------------- | ------------------------------------------------------- |
| `header`       | `std_msgs/Header`             | 时间戳 + `frame_id`（相机坐标系，例如 `camera_optical_frame`） |
| `tracks`       | `TargetTrack[]`               | 当帧所有已确认目标的轨迹                                  |
| `frame_idx`    | `uint32`                      | 单调递增的帧编号（自节点启动起）                            |

### TargetTrack 单目标字段

```text
uint32   target_id
float64  x
float64  y
float64  vx
float64  vy
```

| 字段        | 类型       | 说明                                                       |
| ----------- | ---------- | ---------------------------------------------------------- |
| `target_id` | `uint32`   | DeepSORT / BoT-SORT 分配的目标 ID（节点内唯一）             |
| `x`, `y`    | `float64`  | 像素坐标（图像平面内目标框中心），与 cvtrack 输出一致       |
| `vx`, `vy`  | `float64`  | 像素/秒（Kalman 估计速度）                                  |

> 坐标约定：发布的是**像素坐标**。下游需要世界坐标时，由 planner / control 节点
> 自行完成相机标定 / IPM / 单应性投影。

---

## 4. 数据流

```
相机图像 ─► tracker_node (YOLOv8 + DeepSORT/BoT-SORT)
                       │
                       ▼  /target_track  (TargetTrackArray)
                  planner_node ─► scheduler_node ─► decision_node ─► control_node
                                                                            │
                                                                  ┌─────────┴─────────┐
                                                                  ▼                   ▼
                                                              PX4                  无人车
```

---

## 5. 接入示例

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
                '  id=%d pos=(%.1f, %.1f) vel=(%.2f, %.2f)',
                t.target_id, t.x, t.y, t.vx, t.vy,
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

# 方式 3：直接用 ros2 run
ros2 run perception_pkg tracker_node --ros-args \
    -p input_mode:=video \
    -p video_source:=/data/clip.mp4
```

---

## 6. 后续计划

* 引入 `PathPlan` / `MissionPlan` / `DecisionResult` 等剩余消息定义
* 待规划：`TargetTrackArrayWithForecast` —— 在 `TargetTrackArray` 基础上
  追加未来 N 步 Kalman 投影（来自 cvtrack `predict_n_steps`）
