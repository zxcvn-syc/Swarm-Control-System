# 联调日志规范（Logging Convention）

> 团队里每个节点在联调时打的日志必须遵守这份规范；联调总指挥（何泓林）每周抽检。

---

## 1. 总则

* **时间戳**：rclpy 默认会前缀 `YYYY-MM-DD HH:MM:SS,mmm` 到日志行。
* **节点名**：每个 ROS2 节点在 logger 自带前缀 `[<node_name>]`。
* **关卡**：联调期间开 INFO；问题排查时 `ros2 run ... --ros-args --log-level debug`。
* **必填标签**：每个节点的关键生命周期事件打一个 `[<tag>]` 标签，便于 grep。

---

## 2. 节点级日志标签

| 节点 | 必打标签 | 典型事件 |
|------|---------|---------|
| `tracker_node` | `[tracker]` | 启动 ready、收到 frame、发布 TargetTrack、YOLO/DeepSORT 失败 |
| `scheduler_node` | `[scheduler]` | ready、收到 TargetTrack、分配 TaskAssignment、空 target 警告 |
| `planner_stub_node` | `[planner_stub]` | ready、收到 TaskAssignment、drone_states 发布 |
| `enclosure_node` | `[enclosure]` | ready、Voronoi 区域更新、收到 enclosure_targets、收到 drone_states |
| `test_three_links.py` | `[integration/test]` | 心跳、链接 PASS/FAIL、报告路径 |
| `Synthetic*Publisher` | `[integration/test]` | ready |

---

## 3. 关键事件固定文案

下表是各节点关键事件的固定日志行格式（便于 grep / 周报自动统计）。

| 节点 | 事件 | 日志格式 |
|------|------|---------|
| `tracker_node` | 启动 ready | `<tag> ready: mode=<mode> topic=<topic> rate=<rate>Hz frame_id=<frame> tracker=<kind> weights=<weights or '(auto)'>` |
| `tracker_node` | 发布 TargetTrack（debug） | `<tag> published frame_idx=<idx> n_tracks=<n>` |
| `tracker_node` | Enclosure publisher 启用 | `<tag> Enclosure publisher enabled on <topic>` |
| `tracker_node` | YOLO 失败 | `<tag> cvtrack runner failed: <err>` (error level) |
| `scheduler_node` | 启动 ready | `<tag> up: strategy=<greedy/hungarian>, num_drones=<N>, max_per_drone=<K>, tick=<T>s, in=<topic1>+<topic2>, out=<topic>` |
| `scheduler_node` | 无 target 警告（**只打印一次**） | `<tag> no targets received yet, waiting...` |
| `scheduler_node` | 周期 summary | `<tag> summary: <pairs> assignments, <targets> active targets, <drones> active drones, tick=<ms> ms` |
| `scheduler_node` | Hungarian 回退 greedy | `hungarian_assign failed (<err>); falling back to greedy.` (warn) |
| `planner_stub_node` | 启动 ready | `<tag> ready: num_drones=<N> max_speed=<m>m/s tick=<t>s frame_id=<frame>` |
| `planner_stub_node` | 收到 TaskAssignment | `<tag> 收到 TaskAssignment: drone_id=<id> target_id=<id> task_type='<type>'` |
| `planner_stub_node` | drone_states 发布 | `<tag> drone_states: n=<n> n_assigned=<a> n_targets=<t>` |
| `enclosure_node` | 启动 ready | （构造体 + declare_parameter 完成即视为 ready；可用 ROS2 node list 验证） |
| `enclosure_node` | Voronoi 更新（debug） | `<tag> Voronoi update completed in <ms> ms` |
| `integration test` | 心跳 | `<tag> heartbeat: tracks=<t> tasks=<a> drones=<d> enc_tgt=<et> enc_cmd=<ec>` |
| `integration test` | 报告写入 | `<tag> report written to <path>` |
| `integration test` | PASS / FAIL | `<tag> PASS: link1=<n> link2=<n> link3=<n>` / `<tag> FAIL: link1=<p1> link2=<p2> link3=<p3>` |

---

## 4. 输出位置

* **运行期 / colcon build**：console → stdout / stderr → 终端
* **离线 / 录屏**：

  ```bash
  # ROS2 自带日志
  ros2 launch ros2_ws/launch/three_links.launch.py --log-dir /tmp/three_links_logs/...
  ```

  或者

  ```bash
  ./scripts/record_three_links.sh videos/three_links_$(date +%Y%m%d).mp4 2>&1 \
      | tee output/three_links_<DATE>.log
  ```

* **集成测试报告**：测试脚本自动写到 `output/test_three_links_<timestamp>.json`。

---

## 5. 字段命名规范

* 数字：`<key>=<value>` 中 `=` 两侧不要空格，浮点保留 1 位小数。
* 计时：ms（毫秒）；s（秒，浮点 2 位小数）。
* 坐标：`<x>=<v> <y>=<v>`，单位隐含在 `frame_id`。
* 物体：`<key>=<id>`，id 用 uint32 字面。

---

## 6. 示例：典型的联调一次运行（节选）

```text
2026-07-27 10:08:22,720 [INFO] [scheduler_node]: scheduler_node up: strategy=greedy, num_drones=8, max_per_drone=2, tick=0.5s, in=/target_track+/drone_states, out=/task_assignment
2026-07-27 10:08:22,727 [INFO] [planner_stub_node]: [planner_stub] ready: num_drones=8 max_speed=2.0m/s tick=0.50s frame_id=world
2026-07-27 10:08:23,737 [INFO] [integration/test]: heartbeat: tracks=4 tasks=4 drones=2 enc_tgt=4 enc_cmd=1
...
2026-07-27 10:08:28,738 [INFO] [integration/test]: report written to /home/hhh/Downloads/Swarm-Control-System/output/test_three_links_20260727_100828.json
2026-07-27 10:08:28,756 [INFO] [integration/test]: PASS: link1=24 link2=12 link3=6
```

---

## 7. 自查清单（每周抽）

- [ ] 4 个节点都在 INFO 级别启动成功
- [ ] 关键事件文案与本规范 §3 一致
- [ ] 报告 JSON 中 `passed: true`
- [ ] 没有 ERROR 级别日志
- [ ] WARN 级别日志只来自预期路径（hungarian 回退、YOLO 降级）

---

*最新更新：2026-07-27 — 联调首发版。*
