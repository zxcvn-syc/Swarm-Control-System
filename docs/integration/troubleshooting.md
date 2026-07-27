# 故障排查（Troubleshooting）

> 收集三关联调里遇到的问题与解决方案。每发现新情况即时追加。

---

## Q1: `tracker_node` 启动报 `cannot open video source '0'`

**症状**

```text
[WARN] [tracker_node]: input_mode=video but video_source is empty; falling back to /dev/video0.
[ WARN:0@0.323] global cap_v4l.cpp:999 open VIDEOIO(V4L2:/dev/video0): can't open camera by index
[ERROR:0@0.323] global obsensor_uvc_stream_channel.cpp:158 getStreamChannelGroup Camera index out out of range
WARNING [integration/test] TrackerNode() failed: cannot open video source '0'
```

**原因**：tracker_node 默认进入 `input_mode=video` 但 `video_source` 参数为空，会尝试 `/dev/video0`；本机没有摄像头，于是失败。

**解决 A — 给它视频**：

```bash
ros2 launch ros2_ws/launch/three_links.launch.py \
    video_source:=/abs/path/to/videos/test_multi_target_tracking.mp4
```

**解决 B — 关掉 cvtrack 依赖**：让 tracker_node 改成 `input_mode:=topic`，提供一个合成图像的 publisher（详见 `test_three_links.py` 里的 `SyntheticImagePublisher`）。

**解决 C — 让测试自动降级**：直接跑

```bash
python3 ros2_ws/test_three_links.py
```

脚本检测到 `TrackerNode()` 抛错时不会 crash，会回退到 `SyntheticTrackerPublisher` 直接喂数据。

---

## Q2: `EnclosureNode`: `RcutilsLogger.debug() takes 2 positional arguments but 3 were given`

**症状**

```text
TypeError: RcutilsLogger.debug() takes 2 positional arguments but 3 were given
```

**原因**：rclpy 的 `Logger.debug("pattern %s", value)` 在 ROS2 Humble 下不支持，**只能 `f-string` 或 `format=`**。`enclosure_node.py` 里写了 C 风格 format。

**修复**（已在本 PR 应用）：

```python
# enclosure_node.py line 87
self.get_logger().debug(f"Voronoi update completed in {elapsed_ms:.3f} ms")
```

**预防**：所有新增节点日志必须用 f-string 或 `.debug("text", extra={...})`。

---

## Q3: `rclpy.shutdown()` or `destroy_node()` 在测试结束时报错

**症状**

```text
rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown was already called
```

**原因**：测试脚本与 `ros2 launch` 同时关闭节点时，多次调用 `shutdown` 会触发。

**解决**：测试脚本用 `try/except` 包住 `rclpy.shutdown()`。本 PR `test_three_links.py` 的 `main()` 已经处理。

---

## Q4: `/task_assignment` 一直为空

**症状**

```text
scheduler_node summary: 0 assignments, 0 active targets, 0 active drones
```

**原因**：上游 `/target_track` 或 `/drone_states` 还没开始发消息。

**排查**：

```bash
# 看 /target_track 是否有数据
ros2 topic hz /target_track
ros2 topic echo /target_track --once

# 看 /drone_states 是否有数据
ros2 topic hz /drone_states
ros2 topic echo /drone_states --once
```

确认 `tracker_node` / `planner_stub_node` 都已启动并发布。

---

## Q5: `/enclosure_command` 不下发（但 `/enclosure_targets` 与 `/drone_states` 都有）

**症状**：enclosure_node 启动但 tick 不触发。

**原因 A**：v3 之前的 `enclosure_node` 用旧 callback 名 `_targets_callback` / `_drones_callback`，对不上 subscriber id，可能错过信号。当前源码已统一名。

**原因 B**：`enclosure_node` 的 `tick()` 在 `_dirty=False` 时直接 `return False`，不会发命令。**首条消息之后就会发**。

**排查**：开调试日志 `ros2 run containment_pkg enclosure_node --ros-args --log-level debug`。

---

## Q6: planner_stub_node 报 `AttributeError: 'DroneStateArray' object has no attribute 'header'`

**症状**：上一版本的占位节点错把 `header` 写到 `DroneStateArray`（实际该 msg 类型没有 header）。

**修复**：删掉该行。本 PR 已修正。

---

## Q7: cv_bridge 报 ABI mismatch

**症状**

```text
ImportError: cv_bridge/CvBridge.hpp: No such file or directory
```

**原因**：cv_bridge 是用 numpy 1.x 编译的，运行时 numpy 2.x 会失败。

**解决**：本 PR 中 `tracker_node.py` 已用 try/except 把 cv_bridge 降级为"可选"，不影响 `input_mode:=video` 路径；`input_mode:=topic` 在 cv_bridge 挂掉时不可用。

---

## Q8: 测试报告里 `task_assignments_received` 比 `tracks_received` 少

**原因**：正常情况。 scheduler 是 tick 驱动的（默认 500 ms 一次），而 tracker_node 是 publish_rate 驱动（默认 100 ms）。一轮可能收到 5 帧 tracks、但只有 1 次 scheduler tick 产生 2 条 assignment。

---

## Q9: 跑 `python3 test_three_links.py` 没有 real 节点起来（甚至 synthetic 也无输出）

**原因**：可能 `rclpy.init()` 已在外层被调用。 脚本对 `rclpy.ok()` 做了检查，应当不会重复 init；如仍然异常则 `rclpy.shutdown()` 后重试。

**解决**：手动 `pkill -f test_three_links` + 清理 `~/.ros/log/`。

---

## Q10: `DroneState` 没有 `header` 字段导致下游无法获取时间戳

**症状**：规划/控制想要 `/drone_state` 的时间戳，但 `DroneState.msg` 没有 `Header`。

**设计**：决策 D-1 + D-2：`DroneStateArray` 自身没有 header，发布节点 (`planner_stub_node`) 在 `_tick` 内使用 `node.get_clock().now()` 作为参考。 下游若需精确时间戳，使用 `message.header.stamp.sec`（**目前没有**）；退路是订阅时记 `rclpy.time.Time.now()`。

**未来**：C2 阶段考虑扩展 `DroneState.msg` 加入 `Header header`。

---

## Q11: `launch` 启动时报 `'NoneType' object has no attribute 'perform_substitution'`

**症状**：自定义 launch file 在静态导入阶段调用了 `LaunchConfiguration(x).perform(None)`。

**解决**：把 perform 调用放到 `OpaqueFunction` / `OpaqueFunction`-style 函数里，让 launch framework 提供 context。本 PR `integration_test.launch.py` 已用 `OpaqueFunction` 包装。

---

## Q12: 测试期间出现 stderr 噪音 `getStreamChannelGroup Camera index out of range`

**来源**：tracker_node 降级到 `/dev/video0` 的 OpenCV 提示。

**解决**：本环境没摄像头，直接 `pass`；不要通过 `OPENCV_VIDEOIO_PRIORITY_LIST` 隐藏，否则影响真实部署。

---

## Q13: `pendulum 节点未找到`

**症状**

```text
Package 'pendulum' not found
```

**误报**：本问题与本项目无关——Ros2 教程示例包。如果 ros2 launch 启动时打 "package not found" 是另外的包名，请确认 install/setup.bash 被 source。

---

## 当你遇到**新**问题时：

1. **先**用 `ros2 doctor` 与 `ros2 topic info /target_track -v` 看 transport 健康度。
2. **再**确认每个上游节点的 logger 输出（`/tmp/ros2_logs/<date>/<node>.log`）。
3. **最后**把根因+解决方案追加到这个文件，并在 CHANGELOG 中记录。
