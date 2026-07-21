# YOLOv8 + DeepSORT → ROS2 `TargetTrackArray` 接入总结

> 把 YOLOv8 + DeepSORT / BoT-SORT 算出来的"目标实时 ID 与坐标 (X, Y)"打包成
> ROS2 `TargetTrackArray` 消息发出来。涵盖本仓库
> (`Swarm-Control-System/`) 与同目录的 `cv_tracking_demo/` 两个项目。

## 改动文件总览

### `cv_tracking_demo/` (感知实现)

| 文件 | 类型 | 说明 |
|------|------|------|
| `src/cvtrack/runner.py` | 新增 | `CvtrackRunner`：把 detector + tracker 封装成单帧接口 `step(frame) -> List[Track]`，外加 `step_records(frame) -> List[TrackedTarget]`（消息友好型）。提供 `from_yaml(preset)` / `from_overrides(...)` 工厂。 |

### `Swarm-Control-System/` (消息 + ROS2 节点)

| 文件 | 类型 | 说明 |
|------|------|------|
| `ros2_ws/src/swarm_interfaces/msg/TargetTrackArray.msg` | 新增 | `std_msgs/Header header`、`TargetTrack[] tracks`、`uint32 frame_idx` |
| `ros2_ws/src/swarm_interfaces/package.xml` | 新增 | ament_cmake 接口包元数据 |
| `ros2_ws/src/swarm_interfaces/CMakeLists.txt` | 新增 | 编译所有 msg / srv（包含 `TaskAssignment.msg`） |
| `ros2_ws/src/swarm_interfaces/README.md` | 新增 | 包说明 |
| `ros2_ws/src/perception_pkg/package.xml` | 新增 | ament_python 包元数据 |
| `ros2_ws/src/perception_pkg/setup.py` | 新增 | 安装脚本（包含 `console_scripts`: `tracker_node`） |
| `ros2_ws/src/perception_pkg/setup.cfg` | 新增 | `$base/lib/...` 脚本安装路径 |
| `ros2_ws/src/perception_pkg/perception_pkg/__init__.py` | 新增 | 包标记 |
| `ros2_ws/src/perception_pkg/perception_pkg/tracker_node.py` | 新增 | `TrackerNode`：`CvtrackRunner` + ROS2 发布器 |
| `ros2_ws/src/perception_pkg/launch/tracker_node.launch.py` | 新增 | launch 文件 |
| `ros2_ws/src/perception_pkg/config/tracker_node.yaml` | 新增 | 参数默认值 |
| `ros2_ws/src/perception_pkg/resource/perception_pkg` | 新增 | ament 资源标记文件 |
| `ros2_ws/src/perception_pkg/README.md` | 替换 | 节点说明 + 启动示例 |
| `docs/interface/Topic接口设计V2.md` | 新增 | V2 Topic 接口文档 |
| `docs/interface/Topic接口设计V1.md` | 更新 | 加 V2 跳转与说明 |
| `README.md` | 更新 | 第二阶段进度标记更新 |

## 数据流

```
                ┌──────────────────────────────┐
                │  cv_tracking_demo / cvtrack  │
                │  (YOLOv8 + DeepSORT/BoT-SORT)│
                └────────────┬─────────────────┘
                             │ step_records(frame)
                             ▼
   ┌────────────────────────────────────────────────┐
   │  perception_pkg / tracker_node                 │
   │  - input_mode=video  ─► cv2.VideoCapture       │
   │  - input_mode=topic  ─► sensor_msgs/Image      │
   │                       (cv_bridge → bgr8)      │
   │  - 10 Hz publish loop on /target_track         │
   └────────────────────────────┬───────────────────┘
                                │  TargetTrackArray
                                │  ┌─────────────────────────┐
                                │  │ std_msgs/Header         │
                                │  │ TargetTrack[] tracks    │
                                │  │   target_id / x / y     │
                                │  │   / vx / vy             │
                                │  │ uint32 frame_idx        │
                                │  └─────────────────────────┘
                                ▼
                       planner_node / scheduler_node ...
```

## 关键设计决策

1. **`TargetTrackArray` vs 单目标流式发布** — 选择前者：每帧一条消息，
   下游订阅者按帧消费，避免 N 个目标在 N 条独立消息里竞速。
2. **像素坐标 vs 世界坐标** — 选择前者：cvtrack 原生输出像素坐标，下游
   需要世界坐标时自行标定。`TargetTrack.x / y` 在文档里明确为像素。
3. **三种跟踪器都支持** — 通过 `tracker.kind` 参数切换：
   `botsort` / `deepsort` / `deepsort_cascade`。
4. **两种输入源都支持** — `input_mode=video`（本地视频/摄像头）、
   `input_mode=topic`（ROS2 `sensor_msgs/Image`，通常 `/camera/image`）。
5. **cvtrack 自动发现** — 若 `cvtrack` 没装到 site-packages，
   `tracker_node` 会自动把 `~/Downloads/cv_tracking_demo/src` 加进
   `sys.path`，省去 `pip install -e` 这一步。

## 构建与运行

```bash
# 1) 编译两个包
cd Swarm-Control-System/ros2_ws
colcon build --packages-select swarm_interfaces perception_pkg --merge-install

# 2) source
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=$PWD/install:$AMENT_PREFIX_PATH
source install/setup.bash

# 3) 启动 tracker_node（视频模式）
ros2 launch perception_pkg tracker_node.launch.py \
    mode:=video \
    video_source:=/home/hhh/Downloads/cv_tracking_demo/pexels_aerial_2034115.mp4 \
    tracker.kind:=deepsort_cascade \
    detector.weights:=/home/hhh/Downloads/cv_tracking_demo/weights/visdrone_yolov8s.pt

# 4) 订阅验证
ros2 topic echo /target_track swarm_interfaces/msg/TargetTrackArray
ros2 topic hz    /target_track
```

## 已通过验证

* `swarm_interfaces` 在 `--merge-install` 下 `colcon build` 成功
* `perception_pkg` 在 `--merge-install` 下 `colcon build` 成功
* `ros2 interface show swarm_interfaces/msg/TargetTrackArray` 输出正确
* `ros2 pkg executables perception_pkg` 列出 `tracker_node`
* `ros2 launch perception_pkg tracker_node.launch.py` 能解析所有 8 个 launch args
* `python3 -c "from swarm_interfaces.msg import TargetTrack, TargetTrackArray; ..."`
  能正常导入并构造消息
* `py_compile` 对所有新 Python 文件无语法错误
* 仓库级 lint (ReadLints) 无报错

## 后续

* 在 `TargetTrackArray` 之上引入 `TargetTrackArrayWithForecast`
  （含 Kalman 未来 N 步投影），让 planner 有更长的预测视野
* 把 cvtrack 的 ID 通过 `camera_id * 1000 + track_id` 编码，解决多相机场景
  下 ID 冲突
* 接 PX4 相机话题，验证 ROS2 image 端到端链路
* 单元测试：mock `cvtrack.runner.CvtrackRunner`，断言
  `TargetTrackArray` 字段（target_id / x / y / vx / vy / frame_idx）
  与底层 Track 数值一致
