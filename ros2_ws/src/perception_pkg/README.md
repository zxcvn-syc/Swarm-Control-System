# perception_pkg

ROS2 感知与跟踪模块：YOLOv8 目标检测 + DeepSORT / BoT-SORT 跟踪，把
每帧的"目标实时 ID + 像素坐标 (X, Y)"打包成 `swarm_interfaces/TargetTrackArray`
消息发布到 `/target_track`。

---

## 接口对齐说明（何泓林要求）

`tracker_node` 发布的 `/target_track` 消息格式与文档完全一致：

| 字段 | 类型 | 说明 |
|------|------|------|
| `target_id` | `uint32` | DeepSORT / BoT-SORT 分配的轨迹 ID（节点内唯一） |
| `x`, `y` | `float64` | 像素坐标（检测框中心），与 cvtrack 输出一致 |
| `vx`, `vy` | `float64` | 像素/秒（Kalman 估计速度） |
| `confidence` | `float32` | 检测置信度 (0.0–1.0) |
| `cls` | `uint8` | COCO 目标类别 |
| `is_confirmed` | `bool` | 是否已确认（经 n_init 帧确认） |
| `speed` | `float32` | 速度大小（标量） |
| `motion_mode` | `uint8` | 运动模式：0=未知, 1=静止, 2=慢速, 3=快速 |
| `pred_x[5]` | `float32[5]` | 未来5步预测 X 坐标（仅 adaptive tracker 有意义） |
| `pred_y[5]` | `float32[5]` | 未来5步预测 Y 坐标 |
| `pred_conf[5]` | `float32[5]` | 预测置信度 |

### QoS 与帧率

- **QoS**: `depth=10, reliability=RELIABLE`
- **默认帧率**: `publish_rate_hz=10.0`（可调，0=无限制）
- **Topic**: `/target_track`

> **坐标约定**: `x/y` 为**像素坐标**，下游需要世界坐标时需自行完成相机标定 / IPM / 单应性投影。
> 可选启用 `coord_transform_node` 进行像素→地面世界坐标转换。

### 接口分歧记录

经逐字段对比 `TargetTrack.msg` 与 `tracker_node.py`，发现以下小差异并已处理：

| 分歧点 | 差异描述 | 处理方案 |
|--------|---------|---------|
| `bbox` 包围盒 | `TargetTrack.msg` 未定义 bbox 字段 | bbox 信息保留在 `cvtrack.types.Box`，通过 `coord_transform_node` 的地面交点还原 |
| `EnclosureTarget` | 封控组需 bbox + history_x/y | `tracker_node` 额外发布 `/enclosure_targets`（`EnclosureTargetArray`），`pred_x/y` 长度扩展至 10 步 |
| 融合场景协方差 | 融合路径用置信度估算协方差 | 融合模式下协方差为近似值，单源路径无此问题 |

---

## 节点

### `tracker_node`

| 输入 | 输出 |
|------|------|
| `video_source` (本地视频/摄像头) **或** `image_topic` (`/camera/image`) | `/target_track` (`TargetTrackArray`) |

### `coord_transform_node`

将像素坐标转换为地面 ENU 世界坐标（米），需要 `/camera_info` + `/drone_pose`。

---

## 启动方式

### 方式 1：统一 launch（推荐）

```bash
# 安装 vendored cvtrack 依赖（首次使用）
python3 -m pip install -e src/perception_pkg/cvtrack

# 编译
cd ros2_ws
colcon build --packages-select swarm_interfaces perception_pkg --merge-install
source install/setup.bash

# 默认：CPU，MOG2 备选，无 YOLO 权重时自动降级
ros2 launch perception_pkg perception.launch.py

# 完整 YOLOv8 + DeepSORT（需 YOLO 权重）
ros2 launch perception_pkg perception.launch.py \
    video_source:=videos/test_synthetic_multi_target.mp4 \
    detector_backend:=yolo \
    detector_weights:=/path/to/weights/visdrone_yolov8s.pt \
    tracker_kind:=deepsort_cascade

# 带世界坐标转换
ros2 launch perception_pkg perception.launch.py \
    input_mode:=topic \
    image_topic:=/uav/camera/image \
    coord_transform_enabled:=true \
    camera_info_topic:=/uav/camera/camera_info
```

### 方式 2：独立 tracker_node

```bash
ros2 run perception_pkg tracker_node --ros-args \
    -p input_mode:=video \
    -p video_source:=videos/test_synthetic_multi_target.mp4 \
    -p tracker.kind:=deepsort_cascade
```

---

## 参数说明

完整参数见 `config/perception.yaml`（推荐）或 `config/tracker_node.yaml`。

| 组 | 关键参数 |
|----|---------|
| `detector` | `backend`（yolo/auto），`weights`，`device`（cpu/cuda:0），`imgsz`，`conf`，`classes` |
| `tracker` | `kind`（botsort / deepsort / deepsort_cascade / *_adaptive），`dt`，`max_age`，`n_init` |
| `kalman` | 自适应卡尔曼参数（`botsort_adaptive` / `deepsort_adaptive` 生效） |
| `trajectory_prediction` | `prediction_steps`，`confidence_decay`（adaptive tracker 生效） |
| `appearance` | `enabled`（`deepsort_cascade` + OSNet ReID） |

---

## 性能指标（CPU 实测）

| 场景 | 检测器 | 分辨率 | 平均帧率 | 备注 |
|------|--------|--------|---------|------|
| 合成多目标 | MOG2 | 640×480 | ≥ 3 Hz | 无需 GPU |
| 合成遮挡 | MOG2 | 640×480 | ≥ 3 Hz | |
| 合成快速运动 | MOG2 | 640×480 | ≥ 3 Hz | |
| 真实视频（YOLOv8s） | YOLOv8s | 480px | ~3–5 Hz | CPU（GPU 可达 15+ Hz） |

> 实测方法：`pytest tests/test_yolo_inference_speed.py -v`（120 帧 warmup=5）

---

## 测试

```bash
# 运行所有测试
cd ros2_ws/src/perception_pkg
pytest tests/ -v

# 单独运行节点逻辑测试（无 ROS2 依赖）
pytest tests/test_tracker_node.py -v

# 单独运行 FPS 测试（需要测试视频）
pytest tests/test_yolo_inference_speed.py -v

# 坐标转换纯数学测试
pytest tests/test_coord_transform.py -v
```

---

## 文件结构

```
perception_pkg/
├── perception_pkg/
│   ├── tracker_node.py          # YOLOv8 + tracker → /target_track
│   └── coord_transform_node.py  # 像素 → 世界坐标（需要 /camera_info + /drone_pose）
├── cvtrack/                    # vendored cvtrack（YOLOv8 + tracker 算法）
├── config/
│   ├── perception.yaml          # 推荐：统一参数配置
│   └── tracker_node.yaml        # legacy 参数文件
├── launch/
│   ├── perception.launch.py     # 推荐：统一 launch
│   └── tracker_node.launch.py    # legacy launch
├── tests/
│   ├── test_tracker_node.py          # 节点逻辑测试（mock）
│   ├── test_yolo_inference_speed.py  # FPS + 检测质量测试
│   └── test_coord_transform.py       # 坐标转换数学测试
└── README.md
```

---

## 已知问题

| 问题 | 状态 | 说明 |
|------|------|------|
| Web 可视化暂缓 | ⚠️ 暂缓 | 优先保证数据链路稳定，Web 可视化后续迭代 |
| 多相机 ID 编码 | 📋 待办 | `camera_id * 1000 + track_id` 方案待实现 |
| 融合路径协方差近似 | ⚠️ 已知 | 融合场景协方差由置信度估算，非精确 KF 协方差 |
| homography IPM | 📋 待办 | `coord_transform_node` 使用射线-地面交点法，homography 法待实现 |
| YOLO 权重需手动下载 | ⚠️ 已知 | `detector.backend:=auto` 时自动降级为 MOG2，无需权重 |

---

## 前置依赖

- ROS2 Humble（`rclpy`、`cv_bridge`、`sensor_msgs`、`std_msgs`）
- `swarm_interfaces`（colcon build 同一 workspace）
- `cvtrack`（已在 `perception_pkg/cvtrack/`，通过 `sys.path` 自动发现）
- YOLOv8 权重（可选，`detector.backend:=yolo` 时需要）
