# perception_pkg

ROS2 感知与跟踪模块：YOLOv8 目标检测 + DeepSORT / BoT-SORT 跟踪，把
每帧的“目标实时 ID + 像素坐标 (X, Y)”打包成 `swarm_interfaces/TargetTrackArray`
消息发布到 `/target_track`。

## 节点

### `tracker_node`

| 输入（topic / video）         | 输出                                |
|-------------------------------|-------------------------------------|
| `video_source` (本地视频 / 摄像头) **或** `image_topic` (`/camera/image`) | `/target_track` (`TargetTrackArray`) |

启动：

```bash
# 方式 1：从本地视频读取
ros2 launch perception_pkg tracker_node.launch.py \
    mode:=video \
    video_source:=/home/hhh/Downloads/cv_tracking_demo/pexels_aerial_2034115.mp4 \
    tracker.kind:=deepsort_cascade \
    detector.weights:=/home/hhh/Downloads/weights/visdrone_yolov8s.pt

# 方式 2：从 ROS2 图像话题读取
ros2 launch perception_pkg tracker_node.launch.py \
    mode:=topic \
    image_topic:=/uav/camera/image \
    frame_id:=uav_body
```

也可以直接以节点方式启动，便于调试：

```bash
ros2 run perception_pkg tracker_node --ros-args \
    -p input_mode:=video \
    -p video_source:=/data/clip.mp4 \
    -p tracker.kind:=botsort
```

订阅 `/target_track` 验证：

```bash
ros2 topic echo /target_track swarm_interfaces/msg/TargetTrackArray
ros2 topic hz /target_track
```

## 参数

完整参数见 `config/tracker_node.yaml`。主要分组：

| 组           | 关键参数                                                  |
|--------------|-----------------------------------------------------------|
| 顶层         | `input_mode`、`video_source`、`image_topic`、`frame_id`  |
| `detector`   | `backend`、`weights`、`device`、`imgsz`、`conf`、`classes` |
| `tracker`    | `kind`、`dt`、`max_age`、`n_init`、`iou_thresh`、`high_conf`、`new_track_conf`、`lost_relink_frames` |
| `appearance` | `enabled`、`weights`（仅 `deepsort_cascade` 生效）         |

> 坐标约定：发布消息中 `x / y` 为**像素坐标**（图像平面内目标框中心），
> `vx / vy` 为像素/秒。下游若需世界坐标，需自行完成相机标定 / IPM / 单应性。

## 前置依赖

* ROS2 Humble（`rclpy`、`cv_bridge`、`sensor_msgs`、`std_msgs`）
* `swarm_interfaces`（colcon build 同一 workspace）
* `cvtrack` —— 来自同目录下的 `cv_tracking_demo`：

  ```bash
  pip install -e /home/hhh/Downloads/cv_tracking_demo
  ```

* YOLOv8 权重文件（如 `weights/visdrone_yolov8s.pt`）。可通过
  `detector.weights` 参数注入，或留空让 cvtrack 自动选择本地可用权重。

## 构建

```bash
cd Swarm-Control-System/ros2_ws
colcon build --packages-select swarm_interfaces perception_pkg
source install/setup.bash
ros2 launch perception_pkg tracker_node.launch.py mode:=video video_source:=/data/clip.mp4
```

## 后续

* 接入 RflySim 数字孪生相机话题
* 在 `TargetTrackArray` 之上引入“包含未来轨迹”的 `TargetTrackArrayWithForecast` 消息
* 将 `appearance.enabled` 默认值改为 `true`，让 cascade 匹配默认走 ReID