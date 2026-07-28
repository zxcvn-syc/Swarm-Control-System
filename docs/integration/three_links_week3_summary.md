# Swarm-Control-System 第三周工作总结

> 时间范围：第三周（三关端到端联调周）
> 仓库：`/home/hhh/Downloads/Swarm-Control-System`
> 文档路径：`docs/integration/three_links_week3_summary.md`

---

## 一、本周目标

围绕 **三关端到端跑通** 这一核心交付目标，把"目标感知 → 任务调度 → 路径规划 → 封控执行"的完整链路打通，验证 tracker / scheduler / planner / containment 四个核心节点在统一接口、统一定位、统一定时参数下的协同工作能力，并完成可重复运行的演示脚本与自动化测试。

- **三关定义**（参见 `docs/integration/three_link_enclosure.md`）：
  - `link1` — tracker → scheduler：把 `/target_track` 和 `/target_track_world` 测距信息交给 scheduler_node。
  - `link2` — scheduler → planner：scheduler_node 把 `TaskAssignment` 分发给 planning_pkg。
  - `link3` — planner → containment：planner 输出路径后封控节点发布 `/enclosure_targets`。
- **交付验收**：端到端三关全部 PASS、`pytest` 全量通过、`ros2 launch` 语法校验通过、一键演示脚本可重复运行。

---

## 二、团队分工与产出

本周由 5 人共同完成三关联调，分工与产出如下：

| # | 成员 | 主要产出 |
|---|------|----------|
| 1 | **马子越** | 负责 `scheduler_node`：订阅 `/target_track` + `/target_track_world` + `/drone_states`，对外发布 `TaskAssignment`，无坐标时一并向 `/target_track` 拿最新目标。在静态调度策略下输出稳定的无人机任务分配。 |
| 2 | **陈思睿** | 负责 `enclosure_node`：消费 `TaskAssignment` 与 `/target_track`，实时计算封控点并发布到 `/enclosure_targets`。空闲态用 NaN 占位避免误触发，发布适配 tracker 直通开关。 |
| 3 | **程维好** | 负责 `planning_pkg`：实现 planner_stub_node 与规划算法层；订阅 `TaskAssignment` + `/target_track`、输出 `TrajectoryPoint` 序列；同时维护话题 `/planned_trajectory` 的发布与时间戳一致性。 |
| 4 | **杨诗钰** | 负责 perception 端到端测试稳定化：补齐 tracker/perception 节点的单测与集成用例，统一 YOLO 测试夹具（缺视频时以 `pytest.mark.skip` 标记），并为端到端测试提供稳定的感知输入。 |
| 5 | **何泓林** | 联调总集：搭建三关端到端用例 `test_three_links.py`，撰写 `three_links_demo.sh` 一键演示脚本，维护 `docs/integration/three_link_integration.md` 联调手册，记录并修复联调中暴露的 bug。 |

---

## 三、关键决策（接口对齐）

本周通过多轮联调达成以下接口对齐决策，对应 `docs/integration/interface_alignment.md` 中的 **D-1 ~ D-12**：

- **D-1 — TaskAssignment 不含坐标**
  - `TaskAssignment` 消息只承载任务语义（目标 ID、动作类型、优先级等），**不**携带目标位置。
  - 因此 `planning_pkg`（planner_stub_node）必须同时订阅 `/target_track`，否则规划器拿不到实时目标点。

- **D-2 — `/target_track` 坐标系为像素**
  - 来自 perception（tracker_node）的原始测距输出，坐标系为图像平面像素坐标。

- **D-3 — `/target_track_world` 为世界坐标**
  - 由 tracker_node 同时发布的测距融合结果，坐标系为全局地图系（用于调度侧）。

- **D-4 — `/drone_states` 为 ENU 局部米**
  - 来自飞控/状态聚合节点，坐标系为 **ENU 局部**，单位米；下游消费方需自取朝向约定。

- **D-5 — 所有 Topic 为 RELIABLE，Depth=10**
  - 全链路（`/target_track`、`/target_track_world`、`/drone_states`、`/planned_trajectory`、`/enclosure_targets` 等）统一使用 `qos_profile(reliability=RELIABLE, depth=10)`，避免在低速网络中丢包触发频繁重连。

- **D-6 — tracker_node 默认 `enclosure.enabled:=true` 直通 `/enclosure_targets`**
  - 默认开启内置旁通模式：tracker_node 在收到 `/target_track` 的同时会把目标点直接转发给陈思睿的 `/enclosure_targets`，用于快速演示与单元测试。
  - 关闭时设置 `enclosure.enabled:=false`。

- **D-7 — 空闲态封控命令用 NaN 占位**
  - `enclosure_node` 在没有活跃任务时，对应字段（位置、航向等）发布 `NaN` 而不是历史值或零值，避免封控端误触发。

- **D-8 ~ D-12**：细化的 QoS 重试策略、坐标系转换符号约定、坐标系命名规范、错误码定义、版本兼容窗口（如 `swarm_interfaces` 共享）等，详见 `docs/integration/interface_alignment.md`。

---

## 四、测试与验收结果

### 4.1 `pytest` 总览

各子包 `pytest` 汇总：

| 子包 / 测试集 | 通过 | 跳过 | 备注 |
|---------------|------|------|------|
| `scheduler_node` | 17 | 0 | 调度分配与边界条件 |
| `planner_stub_node` + `planning_pkg` | 9 | 0 | TaskAssignment 路径生成 |
| `enclosure_node` | 23 | 0 | 封控点计算与 NaN 行为 |
| `perception` / tracker 稳定化 | 14 | 10 | YOLO 推理 10 skipped（缺视频源文件） |
| **合计** | **63** | **10** | — |

> 备注：10 个 skipped 全部为 YOLO 推理速度基准测试，需要真实视频文件作为输入，单元测试在没有视频文件的环境下被自动跳过，**不影响功能验收**。

### 4.2 端到端 `test_three_links.py` 结果

| 链路 | 计划节点数 | 实际通过 | 结果 |
|------|-----------|----------|------|
| `link1` — tracker → scheduler | 30 | 30 | **PASS** |
| `link2` — scheduler → planner | 16 | 16 | **PASS** |
| `link3` — planner → containment | 8 | 8 | **PASS** |

三关全部通过，满足端到端联调标准。

### 4.3 Launch 语法校验

两个关键 launch 文件均通过 `ros2 launch --show-args` 的语法校验：

- `swarm_bringup/launch/three_links.launch.py`
- `swarm_bringup/launch/three_links_demo.launch.py`

校验无参数解析错误，可正常拉起。

### 4.4 一键演示脚本

- `three_links_demo.sh` 一键运行通过：自动 source ROS 环境、构建必要包、启动 launch、调用演示用例、清理后台进程，可在干净环境下复现。

---

## 五、联调中修复的 Bug

联调过程中由何泓林（联调总集）汇总、定位并修复的关键问题如下：

### 5.1 `enclosure_node.py:87` C 风格 `RcutilsLogger.debug` 形参错误

- **症状**：`rclpy` 在日志调用处报错，日志丢失。
- **根因**：原写法为 C 风格 `RcutilsLogger.debug("... %d", x)`，传入位置参数而非格式化字符串。
- **修复**：改为 Python f-string，例如 `self.get_logger().debug(f"... {x}")`。

### 5.2 `planner_stub_node` 误用 `DroneStateArray.header`

- **症状**：端到端 link2 偶发 `AttributeError: 'DroneStateArray' object has no attribute 'header'`。
- **根因**：planner_stub_node 误从 `DroneStateArray` 上取 `header`，但该消息类型未定义该字段。
- **修复**：删除对 `header` 的多余引用，时间戳改从 `TaskAssignment.header.stamp` 或 `/drone_states` 内嵌字段取值。

### 5.3 `swarm_interfaces` install/ 旧快照缺失新消息类型

- **症状**：第一次构建 `scheduler_node` 之后，重建老包时报错 `ModuleNotFoundError / no module named 'swarm_interfaces.msg._xxx'`，找不到 `TaskAssignment` 等新消息。
- **根因**：`colcon` 不会覆盖已经安装好的旧 `swarm_interfaces`，`install/` 中的旧快照里没有本次新增的消息类型。
- **修复**：重建依赖 `swarm_interfaces` 的包时，统一加 `--allow-overriding swarm_interfaces`，例如：
  ```bash
  colcon build --packages-select <pkg> --allow-overriding swarm_interfaces
  ```

---

## 六、遗留事项

| 编号 | 遗留项 | 影响 | 建议做法 |
|------|--------|------|----------|
| L-1 | YOLO 推理速度测试 10 skipped（缺视频） | 性能基线尚未建立，但不影响功能 | 仓库新增 `tests/perception/video_samples/` 目录，加入示例视频，或在 CI 中通过 LFS/Artifactory 下载样例 |
| L-2 | tracker_node 在无 `/dev/video0` 的 VM 上无法启动 | 纯虚机/容器演示受限 | 计划在第四周增加 `--use-mock-camera` 参数或图源回放模式 |
| L-3 | `swarm_interfaces` 重装时需显式加 `--allow-overriding` | 易被遗忘导致构建失败 | 在 `swarm_bringup` 中提供 `build_all.sh`，固化该 flag，并在 README 写明 |

---

## 七、附录

- **接口对齐文档**：`docs/integration/interface_alignment.md`
- **三关链路细则**：`docs/integration/three_link_enclosure.md`
- **全链路联调手册**：`docs/integration/three_link_integration.md`
- **联调排错手册**：`docs/integration/troubleshooting.md`
- **日志规范**：`docs/integration/logging.md`
- **感知链路稳定化**：`docs/integration/perception_link_stability.md`

