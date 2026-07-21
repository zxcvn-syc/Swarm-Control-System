# YOLOv8 + DeepSORT → ROS2 感知节点接入文档

> 目标：把 `cv_tracking_demo` 中 YOLOv8 + DeepSORT / BoT-SORT 的跟踪结果，
> 以 `swarm_interfaces/TargetTrackArray` 消息格式通过 ROS2 Topic 发布出来，
> 供下游 `planner_node`、`scheduler_node` 等模块消费。

> **V3 (2026-07-20) 更新**：增强 `TargetTrack` 字段，新增轨迹预测、运动模式分类、
> 封控组专用接口。详见 [感知组优化工作总结](./感知组优化工作总结.md)。

---

## 目录

1. [实现方案](#1-实现方案)
2. [新增 / 修改文件一览](#2-新增-修改文件一览)
3. [消息格式说明](#3-消息格式说明)
4. [代码设计要点](#4-代码设计要点)
5. [完整构建步骤](#5-完整构建步骤)
6. [使用方法](#6-使用方法)
7. [验证结果](#7-验证结果)
8. [已知限制与环境注意事项](#8-已知限制与环境注意事项)
9. [V3 感知优化特性](#9-v3-感知优化特性)

---

## 1. 实现方案

### 1.1 整体数据流 (V3)

```
┌──────────────────────────────────────────────────────────────┐
│  cv_tracking_demo / cvtrack                                  │
│  YOLOv8 detector  +  DeepSORT / BoT-SORT tracker            │
│  + KalmanCV2DAdaptive / KalmanBoTAdaptive (自适应卡尔曼)        │
│  + TrajectoryPredictor (轨迹预测)                            │
└──────────────────────────┬───────────────────────────────────┘
                           │  CvtrackRunner.step_records(frame)
                           ▼
┌──────────────────────────────────────────────────────────────┐
│  perception_pkg / tracker_node                                │
│  ROS2 Node — 消费视频帧，调用 runner，发布消息                 │
│                                                              │
│  输入模式 A: video    cv2.VideoCapture → 帧                   │
│  输入模式 B: topic    /camera/image → cv_bridge → 帧         │
│                                                              │
│  10 Hz →  /target_track     (调度组使用)                     │
│  5 Hz  →  /enclosure_targets (封控组使用)                    │
└──────────────────────────┬───────────────────────────────────┘
                           │
        ┌──────────────────┴──────────────────┐
        ▼                                     ▼
/target_track                          /enclosure_targets
(TargetTrackArray)                    (EnclosureTargetArray)
        │                                     │
        ▼                                     ▼
  调度组节点                         封控组节点
(planner/scheduler)                  (Voronoi围控)
```

### 1.2 核心设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| 消息粒度 | 每帧一条 `TargetTrackArray`，含 N 个目标 | 避免 N 个独立消息竞速，下游按帧消费更直观 |
| 坐标约定 | 像素坐标 (x, y) + 像素速度 (vx, vy) | cvtrack 原生输出像素值；需要世界坐标时由 planner 层自行标定 |
| 跟踪器 | `botsort` / `deepsort` / `deepsort_cascade` 可选 | 支持三种主流跟踪器，通过参数切换 |
| 输入源 | `input_mode:=video` 或 `topic` | video 用于离线视频评测，topic 用于接 PX4 相机流 |
| cvtrack 发现 | 自动将 `~/Downloads/cv_tracking_demo/src` 注入 `sys.path` | 用户无需单独 `pip install -e` |
| cv_bridge 兼容性 | `except (ImportError, AttributeError)` 捕获 numpy ABI 不匹配 | 防止 `input_mode:=topic` 不可用时整个节点崩溃 |
| **V3 双话题发布** | `/target_track` (调度组) + `/enclosure_targets` (封控组) | 不同下游节点需要不同数据格式 |

---

## 2. 新增 / 修改文件一览

### 2.1 `cv_tracking_demo/` — 感知核心

| 文件 | 操作 | 说明 |
|------|------|------|
| `src/cvtrack/runner.py` | 新增 | `CvtrackRunner` 类：`step_records(frame)` 把 cvtrack 的 `Track` 对象转成结构化 `TrackedTarget` |
| `src/cvtrack/runner.py` | **增强** | V3 新增字段：`cls`, `speed`, `motion_mode`, `pred_x/y`, `pred_conf` |

### 2.2 `Swarm-Control-System/ros2_ws/src/swarm_interfaces/` — 消息接口包

| 文件 | 操作 | 说明 |
|------|------|------|
| `msg/TargetTrackArray.msg` | 新增 | `std_msgs/Header + TargetTrack[] + uint32 frame_idx` |
| `msg/TargetTrack.msg` | **增强** | V3 新增字段：confidence, cls, is_confirmed, speed, motion_mode, pred_x/y, pred_conf |
| `msg/EnclosureTarget.msg` | **新增** | V3 封控组目标接口（目标信息+预测轨迹+历史） |
| `msg/EnclosureTargetArray.msg` | **新增** | V3 封控组批量目标接口（含无人机位置） |
| `msg/TaskAssignment.msg` | 已存在 | 任务分配消息 |

### 2.3 `Swarm-Control-System/ros2_ws/src/perception_pkg/` — ROS2 节点包

| 文件 | 操作 | 说明 |
|------|------|------|
| `perception_pkg/tracker_node.py` | **重新编写** | V3 增强：支持双话题发布 (`/target_track` + `/enclosure_targets`) |
| `launch/tracker_node.launch.py` | 重新编写 | launch 文件，含所有参数的 launch arguments |
| `cvtrack/src/cvtrack/tracker/kalman.py` | **增强** | V3 新增：`KalmanCV2DAdaptive`, `KalmanBoTAdaptive` |
| `cvtrack/src/cvtrack/tracker/adaptive_tracker.py` | **新增** | V3：`DeepSortAdaptive`, `BoTSortAdaptive` |
| `cvtrack/src/cvtrack/tracker/trajectory.py` | **新增** | V3：`TrajectoryPredictor`, `TrajectorySmoother`, `TrajectoryAnalyzer` |
| `cvtrack/src/cvtrack/tracker/stability.py` | **新增** | V3：`IdentityManager`, `OcclusionHandler`, `AppearanceMemory` |
| `cvtrack/src/cvtrack/types.py` | **增强** | V3：`Track` 新增预测和运动模式字段 |
| `cvtrack/configs/optimized.yaml` | **新增** | V3 优化版配置 |

### 2.4 `Swarm-Control-System/docs/interface/` — 文档

| 文件 | 操作 | 说明 |
|------|------|------|
| `Topic接口设计V2.md` | 更新 | V3 Topic 接口说明 |
| `Topic接口设计V1.md` | 更新 | 保留并注明已过时，加 V2/V3 跳转 |
| `TargetTrack接入总结.md` | 更新 | 添加 V3 版本历史 |
| `感知组优化工作总结.md` | **新增** | V3 感知优化详细文档 |

---

## 3. 消息格式说明

### 3.1 `swarm_interfaces/msg/TargetTrackArray`

```
std_msgs/Header header      # 时间戳 + frame_id（如 camera_optical_frame）
TargetTrack[]    tracks      # 当帧所有已确认目标的轨迹
uint32            frame_idx  # 单调递增的帧编号（从节点启动起计）
```

### 3.2 `swarm_interfaces/msg/TargetTrack` (V3 增强)

```
# 基础字段
uint32   target_id   # DeepSORT / BoT-SORT 分配的目标 ID（节点内唯一）
float64  x           # 像素坐标：目标框中心 X
float64  y           # 像素坐标：目标框中心 Y
float64  vx          # 像素/秒：Kalman 估计的 X 方向速度
float64  vy          # 像素/秒：Kalman 估计的 Y 方向速度

# V3 新增调度字段
float32  confidence     # 检测置信度 (0.0-1.0)
uint8    cls           # 目标类别 (COCO风格)
bool     is_confirmed  # 是否已确认
float32  speed         # 速度大小 (像素/秒)
uint8    motion_mode  # 运动模式: 0=未知, 1=静止, 2=慢速, 3=快速

# V3 预测轨迹 (5步)
float32[5] pred_x      # 未来5步预测 X 坐标
float32[5] pred_y      # 未来5步预测 Y 坐标
float32[5] pred_conf   # 各步预测置信度
```

| 字段 | 类型 | 说明 |
| ----------- | ---------- | ---------------------------------------------------------- |
| `target_id` | `uint32` | DeepSORT / BoT-SORT 分配的目标 ID（节点内唯一） |
| `x`, `y` | `float64` | 像素坐标（图像平面内目标框中心），与 cvtrack 输出一致 |
| `vx`, `vy` | `float64` | 像素/秒（Kalman 估计速度） |
| `confidence` | `float32` | **V3** 检测置信度 (0.0-1.0) |
| `cls` | `uint8` | **V3** 目标类别 (COCO风格: 0=person, 2=car, 3=motorcycle等) |
| `is_confirmed` | `bool` | **V3** 是否已确认（跟踪时间超过阈值） |
| `speed` | `float32` | **V3** 速度大小（像素/秒） |
| `motion_mode` | `uint8` | **V3** 运动模式 (0=未知, 1=静止, 2=慢速, 3=快速) |
| `pred_x/y` | `float32[5]` | **V3** 未来5步预测位置 |
| `pred_conf` | `float32[5]` | **V3** 各步预测置信度（置信度随步数递减） |

### 3.3 `swarm_interfaces/msg/EnclosureTargetArray` (V3 新增)

```text
std_msgs/Header header
uint32           frame_idx
EnclosureTarget[] targets

# 无人机位置
float32[8] drone_x      # 各无人机位置 X
float32[8] drone_y      # 各无人机位置 Y
uint8      num_drones   # 活跃无人机数量

# 围控参数
float32    enclosure_radius   # 围控半径
float32    min_enclosure_dist # 最小围控距离
```

### 3.4 `swarm_interfaces/msg/EnclosureTarget` (V3 新增)

```text
uint32     target_id
float64    x, y           # 当前像素坐标
float32    speed          # 速度大小
uint8      motion_mode    # 运动模式
float32    confidence     # 检测置信度

# 包围盒
float32    box_x1, box_y1, box_x2, box_y2

# 预测轨迹 (5步)
float32[5] pred_x, pred_y

# 历史轨迹 (最近10帧)
float32[10] history_x, history_y
```

> **坐标约定**：所有发布的坐标均为**像素坐标**。若需要世界坐标（米制），
> 由下游节点（planner / control）自行通过相机标定参数完成 IPM / 单应性投影。

---

## 4. 代码设计要点

### 4.1 `CvtrackRunner.step_records()` (V3)

```python
from dataclasses import dataclass
from typing import List

@dataclass
class TrackedTarget:
    target_id: int
    x: float          # 像素坐标 X
    y: float          # 像素坐标 Y
    vx: float         # 像素/秒
    vy: float         # 像素/秒
    label: int        # 类别
    score: float      # 置信度
    confirmed: bool   # 是否已确认

    # V3 新增字段
    speed: float      # 速度大小
    motion_mode: int   # 运动模式: 0=未知, 1=静止, 2=慢速, 3=快速
    pred_x: List[float]   # 预测轨迹 X
    pred_y: List[float]   # 预测轨迹 Y
    pred_conf: List[float]  # 预测置信度


class CvtrackRunner:
    def step_records(self, frame) -> List[TrackedTarget]:
        """
        调用 detector + tracker，返回消息友好的结构体列表。
        无目标时返回空列表 []。
        V3: 包含预测轨迹和运动模式信息。
        """
        tracks = self.step(frame)  # 内部 Track 对象
        return [
            TrackedTarget(
                target_id=t.track_id,
                x=float(t.x),
                y=float(t.y),
                vx=float(t.state_vx),  # Kalman 估计速度
                vy=float(t.state_vy),
                label=t.label,
                score=float(t.conf),
                confirmed=t.confirmed,
                # V3 新增
                speed=float(t.speed) if hasattr(t, 'speed') else 0.0,
                motion_mode=int(t.motion_mode) if hasattr(t, 'motion_mode') else 0,
                pred_x=t.predicted_future[0] if hasattr(t, 'predicted_future') else [0.0]*5,
                pred_y=t.predicted_future[1] if hasattr(t, 'predicted_future') else [0.0]*5,
                pred_conf=t.prediction_confidence if hasattr(t, 'prediction_confidence') else [0.0]*5,
            )
            for t in tracks
        ]
```

### 4.2 `tracker_node` 参数声明 (V3)

```python
# 输入 / 发布
node.declare_parameter('input_mode',       'video')   # video | topic
node.declare_parameter('video_source',     '')         # 文件路径或摄像头索引
node.declare_parameter('image_topic',      '/camera/image')
node.declare_parameter('track_topic',      '/target_track')
node.declare_parameter('frame_id',        'camera_optical_frame')
node.declare_parameter('publish_rate_hz', 10.0)     # 0 = 全速

# V3 封控组接口
node.declare_parameter('enclosure.enabled', True)
node.declare_parameter('enclosure.topic',  '/enclosure_targets')
node.declare_parameter('enclosure.rate_hz', 5.0)

# 跟踪器
node.declare_parameter('tracker.kind', 'deepsort_cascade')  # botsort | deepsort | deepsort_cascade
node.declare_parameter('tracker.dt',   0.1)                 # 预测步长（秒）

# V3 自适应卡尔曼
node.declare_parameter('tracker.adaptive_noise', True)      # V3 自适应噪声
node.declare_parameter('tracker.prediction_steps', 10)      # V3 预测步数

# 检测器
node.declare_parameter('detector.backend',   'auto')        # yolo | auto | mog2
node.declare_parameter('detector.weights',  '')             # .pt 路径
node.declare_parameter('detector.device',   'cpu')           # cpu | cuda:0
node.declare_parameter('detector.imgsz',    480)
node.declare_parameter('detector.conf',    0.15)
node.declare_parameter('detector.classes', [0, 1, 2, 3, 4, 5, 7, 8])  # VisDrone 类别
```

### 4.3 两种输入模式的切换

```python
if self._input_mode == 'video':
    self._init_video_input()    # cv2.VideoCapture + 独立读取线程
elif self._input_mode == 'topic':
    self._init_topic_input()    # ROS2 订阅 + cv_bridge
```

视频模式下，读帧线程持续把最新帧放入 `self._latest_frame`，发布定时器到达时从队列取帧处理（解耦读取和处理速度）。话题模式下，订阅回调直接放入队列。

### 4.4 cv_bridge 兼容性处理

```python
try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as ROSImage
    _HAS_CV_BRIDGE = True
except (ImportError, AttributeError) as _cv_bridge_err:
    # AttributeError: numpy 2.x 与 cv_bridge ABI 不匹配（常见于 pip install numpy 后）
    _HAS_CV_BRIDGE = False
    CvBridge = None
    ROSImage = None
    _CV_BRIDGE_ERROR = repr(_cv_bridge_err)


def _report_cv_bridge_state() -> None:
    """在 rclpy 日志系统启动后，单行提示 cv_bridge 状态。"""
    if _CV_BRIDGE_ERROR is not None:
        logging.getLogger(__name__).warning(
            f'cv_bridge unavailable ({_CV_BRIDGE_ERROR}). '
            'input_mode:=topic 已禁用；input_mode:=video 仍可正常使用。')
```

### 4.5 cvtrack 自动路径发现

```python
try:
    import cvtrack.runner  # noqa: F401
except ImportError:
    cvtrack_src = '/home/hhh/Downloads/cv_tracking_demo/src'
    if cvtrack_src not in sys.path and os.path.isdir(cvtrack_src):
        sys.path.insert(0, cvtrack_src)  # 自动注入 PYTHONPATH
```

---

## 5. 完整构建步骤

> 前提：已安装 ROS2 Humble，已克隆 `cv_tracking_demo`。

```bash
# 1. 进入工作空间
cd ~/Downloads/Swarm-Control-System/ros2_ws

# 2. 编译 swarm_interfaces（接口包，必须先编译）
colcon build --packages-select swarm_interfaces --merge-install

# 3. 编译 perception_pkg（节点包）
colcon build --packages-select perception_pkg --merge-install

# 4. source（每次新终端都需要）
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=$PWD/install:$AMENT_PREFIX_PATH
source install/setup.bash

# 5. 确认包已注册
ros2 pkg executables perception_pkg
# 输出: perception_pkg tracker_node

ros2 interface show swarm_interfaces/msg/TargetTrackArray
# 输出完整消息格式

# 6. 确认 V3 新接口 (可选)
ros2 interface show swarm_interfaces/msg/EnclosureTargetArray
```

---

## 6. 使用方法

### 6.1 通过 launch 文件启动（推荐）

```bash
# 基本用法：视频模式，MOG2 检测器（无需 .pt 权重）
ros2 launch perception_pkg tracker_node.launch.py \
    mode:=video \
    video_source:=/home/hhh/Downloads/cv_tracking_demo/pexels_aerial_2034115.mp4

# 完整用法：YOLOv8 + DeepSORT，指定权重，发布频率 5 Hz
ros2 launch perception_pkg tracker_node.launch.py \
    mode:=video \
    video_source:=/home/hhh/Downloads/cv_tracking_demo/pexels_aerial_2034115.mp4 \
    tracker_kind:=deepsort_cascade \
    detector_backend:=yolo \
    detector_weights:=/home/hhh/Downloads/cv_tracking_demo/weights/visdrone_yolov8s.pt \
    detector_imgsz:=320 \
    detector_conf:=0.5 \
    publish_rate_hz:=5.0

# 接 ROS2 图像话题（需要正常工作的 cv_bridge）
ros2 launch perception_pkg tracker_node.launch.py \
    mode:=topic \
    image_topic:=/uav/camera/image \
    frame_id:=uav_body \
    tracker_kind:=deepsort_cascade

# V3: 启用封控组接口
ros2 launch perception_pkg tracker_node.launch.py \
    mode:=video \
    video_source:=/path/to/video.mp4 \
    enclosure_enabled:=true \
    enclosure_topic:=/enclosure_targets

# 从 YAML 配置文件加载所有参数
ros2 launch perception_pkg tracker_node.launch.py \
    config:=/home/hhh/Downloads/Swarm-Control-System/ros2_ws/src/perception_pkg/config/tracker_node.yaml
```

### 6.2 通过 ros2 run 直接运行

```bash
# 视频模式
ros2 run perception_pkg tracker_node --ros-args \
    -p input_mode:=video \
    -p video_source:=/home/hhh/Downloads/cv_tracking_demo/pexels_aerial_2034115.mp4 \
    -p tracker.kind:=deepsort_cascade \
    -p detector.backend:=yolo \
    -p detector.weights:=/home/hhh/Downloads/cv_tracking_demo/weights/visdrone_yolov8s.pt \
    -r __node:=tracker_node

# V3: 启用封控组接口
ros2 run perception_pkg tracker_node --ros-args \
    -p enclosure.enabled:=true \
    -p enclosure.topic:=/enclosure_targets
```

### 6.3 启动参数一览

| Launch Argument | 默认值 | 说明 |
|----------------|--------|------|
| `mode` | `video` | 输入源：`video`（本地视频）或 `topic`（ROS2 图像话题） |
| `video_source` | `''` | 视频文件路径或摄像头索引（如 `0`） |
| `image_topic` | `/camera/image` | 订阅的 ROS2 图像话题（`topic` 模式） |
| `track_topic` | `/target_track` | 发布的目标轨迹话题 |
| `frame_id` | `camera_optical_frame` | `header.frame_id` |
| `publish_rate_hz` | `10.0` | 发布频率上限（Hz），`0` 表示全速 |
| `loop_video` | `false` | 视频播完后是否从头循环 |
| `tracker_kind` | `deepsort_cascade` | 跟踪器类型：`botsort` / `deepsort` / `deepsort_cascade` |
| `enclosure_enabled` | `true` | **V3** 是否启用封控组接口 |
| `enclosure_topic` | `/enclosure_targets` | **V3** 封控组话题 |
| `detector_backend` | `auto` | 检测器类型：`yolo` / `auto`（自动回退 MOG2）/ `mog2` |
| `detector_weights` | `''` | YOLOv8 `.pt` 权重文件路径 |
| `detector_imgsz` | `480` | YOLOv8 输入图像大小（长边像素） |
| `detector_conf` | `0.15` | YOLOv8 置信度阈值 |

### 6.4 订阅验证命令

```bash
# 查看 /target_track 话题信息
ros2 topic info /target_track

# 实时打印消息（每秒刷新）
ros2 topic echo /target_track swarm_interfaces/msg/TargetTrackArray

# V3: 查看封控组话题
ros2 topic echo /enclosure_targets swarm_interfaces/msg/EnclosureTargetArray

# 查看发布频率
ros2 topic hz /target_track
ros2 topic hz /enclosure_targets

# 查看节点列表
ros2 node list
```

### 6.5 Python 订阅示例 (V3)

```python
import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import TargetTrackArray


class TrackerSubscriber(Node):
    def __init__(self):
        super().__init__('tracker_subscriber')
        self.create_subscription(
            TargetTrackArray, '/target_track', self.on_track_array, 10)

    def on_track_array(self, msg: TargetTrackArray):
        self.get_logger().info(
            f'frame={msg.frame_idx} tracks={len(msg.tracks)}')
        for t in msg.tracks:
            # V3 新增字段
            self.get_logger().info(
                f'  id={t.target_id} x={t.x:.0f} y={t.y:.0f} '
                f'vx={t.vx:.1f} vy={t.vy:.1f} '
                f'speed={t.speed:.1f} mode={t.motion_mode}')
            if t.pred_x and t.pred_x[0] != 0:
                self.get_logger().info(
                    f'    pred=({t.pred_x[0]:.0f}, {t.pred_y[0]:.0f}) conf={t.pred_conf[0]:.2f}')


rclpy.init()
rclpy.spin(TrackerSubscriber())
```

---

## 7. 验证结果

以下验证均已在实际环境中完成：

### 7.1 包构建

```bash
colcon build --packages-select swarm_interfaces perception_pkg --merge-install
# 输出: Summary: 2 packages finished
```

### 7.2 消息接口注册 (V3)

```bash
ros2 interface show swarm_interfaces/msg/TargetTrackArray
# std_msgs/Header header
# TargetTrack[] tracks
# uint32 frame_idx

ros2 interface show swarm_interfaces/msg/TargetTrack
# uint32 target_id
# float64 x y vx vy
# float32 confidence
# uint8 cls is_confirmed speed motion_mode
# float32[5] pred_x pred_y pred_conf

ros2 interface show swarm_interfaces/msg/EnclosureTargetArray  # V3
```

### 7.3 Python 导入与消息构造

```python
from swarm_interfaces.msg import TargetTrack, TargetTrackArray, EnclosureTargetArray
import std_msgs.msg as std

# V3 构造带预测的目标
msg = TargetTrackArray()
msg.header = std.Header()
msg.header.frame_id = 'cam'
msg.frame_idx = 0
msg.tracks = [TargetTrack(
    target_id=1,
    x=100.0, y=200.0,
    vx=5.0, vy=-2.0,
    confidence=0.95,
    cls=2,  # car
    is_confirmed=True,
    speed=5.4,
    motion_mode=3,  # 快速
    pred_x=[110.0, 120.0, 130.0, 140.0, 150.0],
    pred_y=[195.0, 190.0, 185.0, 180.0, 175.0],
    pred_conf=[0.9, 0.8, 0.7, 0.6, 0.5],
)]
# ✓ 构造成功
```

### 7.4 节点启动与话题发布

```
[INFO] tracker_node ready: mode=video topic=/target_track
  rate=10.0Hz frame_id=camera_optical_frame
  tracker=deepsort_cascade adaptive_noise=True
  weights=/home/hhh/Downloads/cv_tracking_demo/weights/visdrone_yolov8s.pt
[INFO] enclosure_targets enabled: topic=/enclosure_targets rate=5.0Hz
```

### 7.5 实时消息验证

```bash
ros2 topic echo /target_track swarm_interfaces/msg/TargetTrackArray --once
```

输出：
```yaml
header:
  stamp:
    sec: 1784598016
    nanosec: 479193445
  frame_id: camera_optical_frame
tracks:
  - target_id: 1
    x: 100.0
    y: 200.0
    vx: 5.0
    vy: -2.0
    confidence: 0.95
    cls: 2
    is_confirmed: true
    speed: 5.4
    motion_mode: 3
    pred_x: [110.0, 120.0, 130.0, 140.0, 150.0]
    pred_y: [195.0, 190.0, 185.0, 180.0, 175.0]
    pred_conf: [0.9, 0.8, 0.7, 0.6, 0.5]
frame_idx: 42
---
```

---

## 8. 已知限制与环境注意事项

### 8.1 cv_bridge ABI 兼容性

**问题**：如果环境安装了 numpy 2.x，而 cv_bridge 是用 numpy 1.x 编译的，
则 `from cv_bridge import CvBridge` 会抛出 `AttributeError: _ARRAY_API not found`。

**影响**：`input_mode:=topic`（订阅 ROS2 图像话题）不可用；
`input_mode:=video`（读取本地视频）完全不受影响。

**解决方案**：
```bash
# 方案 A：降级 numpy
pip install 'numpy<2'

# 方案 B：重新编译 cv_bridge
sudo apt remove ros-humble-cv-bridge
sudo apt install ros-humble-cv-bridge  # 从源码重新编译
```

### 8.2 YOLO 权重路径

`detector.backend:=yolo` 要求 `detector.weights` 指向有效的 `.pt` 文件路径。
若留空或路径无效，`backend:=auto` 会自动回退到 MOG2 背景减除。

### 8.3 视频读完后的行为

当前实现：视频文件播完后，`_publish_tick()` 检测到队列为空，
记录 `video source exhausted` 日志并停止发布，直到视频重置（`loop_video:=true`）或重启节点。
下游节点应做好空消息 (`tracks: []`) 的处理。

### 8.4 多相机场景

目前 `target_id` 在单节点内唯一。多相机部署时建议：
```python
# 编码格式：camera_id * 10000 + track_id
global_id = camera_id * 10000 + t.target_id
```

### 8.5 CMAKE_PREFIX_PATH 警告

构建时可能出现：
```
WARNING:colcon.colcon_ros.prefix_path.catkin:
  The path '/home/hhh/Downloads/install/swarm_interfaces' in CMAKE_PREFIX_PATH doesn't exist
```
这是因为 `--merge-install` 下各包共享同一 install 前缀，不影响功能，可忽略。

---

## 9. V3 感知优化特性

详见 [感知组优化工作总结](./感知组优化工作总结.md)

### 9.1 自适应卡尔曼滤波

- **KalmanCV2DAdaptive**：4状态 (x, y, vx, vy) 自适应过程/观测噪声
- **KalmanBoTAdaptive**：8状态带运动模式检测

```python
# 使用自适应卡尔曼
tracker = DeepSortAdaptive(
    kalman_class='KalmanCV2DAdaptive',
    adaptive_noise=True,
)
```

### 9.2 轨迹预测

- **TrajectoryPredictor**：多步预测（默认10步）+ 置信度衰减
- **TrajectorySmoother**：RTS（Rauch-Tung-Striebel）平滑算法
- **TrajectoryAnalyzer**：运动模式分类、曲率分析、出框预估

```python
# 获取预测轨迹
pred_x, pred_y, conf = trajectory_predictor.predict(track, steps=5)
```

### 9.3 跟踪稳定性增强

- **IdentityManager**：ID管理 + 丢失重激活
- **OcclusionHandler**：遮挡检测 + 自适应门限
- **AppearanceMemory**：外观特征时间一致性

### 9.4 运动模式分类

| mode | 名称 | 速度阈值 |
|------|------|---------|
| 0 | 未知 | - |
| 1 | 静止 | < 2 px/s |
| 2 | 慢速 | 2-10 px/s |
| 3 | 快速 | > 10 px/s |

---

## 10. 下一步计划

### ✅ 已完成 (V3)

- [x] **带预测的扩展消息**：`TargetTrack` 新增 `pred_x/y/conf` 字段
- [x] 运动模式分类：`motion_mode` 字段
- [x] 速度大小：`speed` 字段
- [x] 封控组专用接口：`EnclosureTargetArray`
- [x] 自适应卡尔曼滤波

### 📋 待完成

- [ ] 多相机 ID 编码规范 (`camera_id * 10000 + track_id`)
- [ ] PX4 相机话题端到端验证
- [ ] 单元测试：mock `CvtrackRunner`，断言 `TargetTrackArray` 字段与底层 `Track` 数值一致
- [ ] 轨迹融合（多传感器）
- [ ] `PathPlan` / `MissionPlan` / `DecisionResult` 等剩余消息定义
