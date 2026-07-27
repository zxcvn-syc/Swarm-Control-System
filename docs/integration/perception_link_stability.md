# 感知链路稳定性报告

> 本报告记录 `perception_pkg` 在真实测试视频上的帧率、检测数、轨迹数稳定性数据。
> 验收标准：**平均帧率 ≥ 3 Hz**，**数据链路持续输出**。

---

## 验收标准

| 指标 | 阈值 | 说明 |
|------|------|------|
| 平均帧率 | ≥ 3 Hz | 单帧推理时间 ≤ 333 ms（CPU） |
| 检测数 | > 0 | 每帧至少有一个检测（合成视频） |
| 轨迹 ID 稳定性 | ID Switch < 5% | DeepSORT / BoT-SORT 遮挡后 ID 保持 |
| 消息完整性 | 100% | `header`、`frame_idx`、`tracks` 字段不缺 |

---

## 测试环境

| 项目 | 值 |
|------|-----|
| 操作系统 | Linux (WSL/Ubuntu) |
| Python | 3.10+ |
| 检测器 | MOG2 (backend=auto fallback) 或 YOLOv8s |
| 跟踪器 | deepsort_cascade / botsort_adaptive |
| 设备 | CPU |
| 测试视频 | `videos/test_synthetic_multi_target.mp4` 等（见下表） |

---

## 测试视频清单

| 视频 | 分辨率 | 时长(s) | 场景类型 | 建议 tracker |
|------|--------|---------|---------|------------|
| `test_synthetic_multi_target.mp4` | 640×480 | ~15 | 多目标 | deepsort_cascade |
| `test_synthetic_occlusion.mp4` | 640×480 | ~15 | 遮挡 | deepsort_cascade |
| `test_synthetic_fast_motion.mp4` | 640×480 | ~15 | 快速运动 | botsort_adaptive |
| `test_synthetic_scale.mp4` | 640×480 | ~15 | 尺度变化 | deepsort_cascade |
| `test_drone_aerial.mp4` | 640×360 | ~28 | 航拍视角 | botsort_adaptive |
| `test_multi_target_tracking.mp4` | 1920×1080 | ~18 | 密集交通 | botsort_adaptive |

---

## 测试方法

### 1. FPS 测试（pytest）

```bash
cd /home/hhh/Downloads/Swarm-Control-System/ros2_ws/src/perception_pkg
pytest tests/test_yolo_inference_speed.py -v --tb=short 2>&1 | tee output/fps_test_log.txt
```

pytest 会在测试结束后打印：

```
  Processed 120 frames
  Average latency: 180.3 ms/frame
  Average FPS: 5.55 Hz
  Min latency: 95.2 ms
  Max latency: 420.1 ms
```

**通过条件**: `Average FPS: X.XX Hz` 中的 `X.XX ≥ 3.0`

### 2. 长时间稳定性测试（手动）

运行节点 30 秒，记录 `ros2 topic hz`：

```bash
# 启动节点（后台）
ros2 launch perception_pkg perception.launch.py \
    video_source:=videos/test_synthetic_multi_target.mp4 \
    tracker.kind:=deepsort_cascade &

sleep 2

# 记录帧率
ros2 topic hz /target_track --window 10 2>&1 | tee output/hz_log.txt

# 记录消息样例
ros2 topic echo /target_track --once > output/target_track_sample.txt

# 等待 30s 后终止
sleep 30
kill %1
```

### 3. ID Switch 测试（Python 脚本）

```python
# 录制脚本 save_id_log.py
import rclpy, csv
from swarm_interfaces.msg import TargetTrackArray

seen_ids = {}  # frame_idx -> set(target_ids)
csvfile = open("output/id_stability.csv", "w", newline="")
writer = csv.writer(csvfile)
writer.writerow(["frame_idx", "n_tracks", "ids", "id_switch"])

def cb(msg: TargetTrackArray):
    ids = {t.target_id for t in msg.tracks}
    prev = max(seen_ids.keys(), default=-1)
    prev_ids = seen_ids.get(prev, set())
    switch = len(prev_ids & ids) != len(prev_ids | ids)
    writer.writerow([msg.frame_idx, len(msg.tracks), sorted(ids), int(switch)])
    seen_ids[msg.frame_idx] = ids

rclpy.init()
node = rclpy.node.Node("id_logger")
node.create_subscription(TargetTrackArray, "/target_track", cb, 10)
rclpy.spin(node)
```

---

## 实测数据记录表

> ⚠️ **请在运行测试后填写以下表格**

### FPS 结果（pytest）

| 视频 | Tracker | 帧数 | 平均延迟(ms) | 平均帧率(Hz) | 通过 |
|------|---------|------|-----------|-------------|------|
| test_synthetic_multi_target.mp4 | deepsort_cascade | 120 | | | |
| test_synthetic_occlusion.mp4 | deepsort_cascade | 120 | | | |
| test_synthetic_fast_motion.mp4 | botsort_adaptive | 120 | | | |
| test_synthetic_scale.mp4 | deepsort_cascade | 120 | | | |
| test_drone_aerial.mp4 | botsort_adaptive | 120 | | | |
| test_multi_target_tracking.mp4 | botsort_adaptive | 120 | | | |

### ros2 topic hz 记录

```
# 命令: ros2 topic hz /target_track
average rate: X.XX Hz
min: X.XX ms max: XX.XX ms std dev: X.XX ms
```

### ID Switch 统计

```
总帧数: XXX
ID Switch 次数: XX
ID Switch 率: XX.X%
```

### 消息字段完整性

| 字段 | 缺失率 | 说明 |
|------|--------|------|
| `header` | 0% | |
| `frame_idx` | 0% | |
| `tracks` | 0% | 空帧（无目标）计为合法 |
| `target_id` | 0% | |
| `x`, `y` | 0% | |
| `vx`, `vy` | 0% | |
| `confidence` | 0% | |
| `cls` | 0% | |
| `is_confirmed` | 0% | |
| `speed` | 0% | |
| `motion_mode` | 0% | |
| `pred_x[5]` | 0% | |
| `pred_y[5]` | 0% | |
| `pred_conf[5]` | 0% | |

---

## 结论

- [ ] **FPS 验收**: 所有测试视频平均帧率 ≥ 3 Hz ✅ / ❌
- [ ] **数据完整性**: 所有消息字段无缺失 ✅ / ❌
- [ ] **ID 稳定性**: ID Switch 率 < 5% ✅ / ❌
- [ ] **ROS2 链路**: `ros2 topic hz` 持续输出，无断流 ✅ / ❌

---

## 日志存档

测试输出保存在 `output/` 目录：

```
output/
├── fps_test_log.txt           # pytest 完整输出
├── hz_log.txt                 # ros2 topic hz 记录
├── target_track_sample.txt    # 一帧消息样例
├── id_stability.csv           # ID Switch 统计
└── stability_report.md        # 本报告（测试后填写）
```

---

## 已知局限

1. **视频测试 ≠ 真机**：合成视频和真实 UAV 图像质量差异较大，帧率仅供参考。
2. **MOG2 vs YOLO**：无 YOLO 权重时使用 MOG2 背景分割，速度更快但检测质量稍低。
3. **合成视频无真实遮挡**：ID Switch 测试建议使用 `test_synthetic_occlusion.mp4` 或真实遮挡场景。
4. **ROS2 测试需要 colcon build**：纯 pytest 测试不需要 ROS2 环境。

---

## 更新记录

| 日期 | 更新内容 |
|------|---------|
| 2026-07-27 | 初始创建；定义验收标准、测试方法、数据记录表 |
