# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

#### 感知组 (perception_pkg)
- **自适应跟踪器全链路**: `botsort_adaptive` / `deepsort_adaptive` 两种 tracker 类型接进 `CvtrackRunner`，优化 `optimized.yaml` 的 `tracker.kalman.*` 与 `trajectory_prediction.*` 段透传为 ROS 参数
- **motion_mode 透传**: 在 `_make_target_track` 中写入枚举值（0=unknown / 1=stationary / 2=slow / 3=fast），下游可直接消费
- **调试话题 `D1`**:
  - `/target_track_debug`（`swarm_interfaces/TargetTrackDebug`）— 含 KF 协方差、motion_mode 人类可读原因、Re-ID 外观分
  - `/tracking_metrics`（`diagnostic_msgs/DiagnosticArray`）— 5 项指标：id_switch_count / miss_rate / convergence_time_ms / active_tracks / motion_mode_distribution
  - 命令行参数 `enable_debug_topics`（默认 True）、`metrics_period_ms`（默认 1000）
- **轨迹融合 `B1/B2`**:
  - 新增 `cvtrack/tracker/fusion.py`（498 行）导出 `TrackFusion` / `weighted_fuse` / `TrajectoryGraph` / `ConsistencyGuard`
  - `tracker_node` 内置 `MultiSourceAggregator`：每 50 ms tick，按 `fusion_sources` 订阅 `/<source>/target_track` 后统一发布
  - 9 个 fusion 单元测试：加权位置、ID 一致性、离群点拒绝、时间平滑、跨源图关联、协方差下降、缺失源、新目标出现、目标消失
- **坐标变换节点 `C3`**: `coord_transform_node.py`（501 行）将 `/target_track`（像素）转 `/target_track_world`（ENU 米），支持相机内参 K + 无人机 pose + 地面平面
- **测试基础设施重写 `B3`**:
  - `test_integration.py` 从伪测试改为真 pytest（44+39 通过）
  - `test_fusion.py` 从 9 skipped 改为 9 passed
  - **整库回归：145 passed, 0 skipped**

#### 调度组 (scheduler_pkg) — 脚手架 `C1`
- `scheduler_node.py`（243 行）：订阅 `/target_track` + `/drone_states`，发布 `/task_assignment`
- 分配算法 `assign.py`：`greedy_assign` 与 `hungarian_assign`（scipy 不可用时降级 greedy）
- 关键参数：`num_drones=8`、`assignment_strategy=greedy|hungarian`、`max_per_drone=2`、`tick_period=0.5`、`default_task_type=track`
- 无 target 时只打一次 info 静默；无 drone 时自动播种默认网格用于 demo
- README: `docs/interface/调度组接入指南.md`

#### 封控组 (containment_pkg) — 脚手架 `C2`
- `enclosure_node.py`（111 行）：订阅 `/enclosure_targets` + `/drone_states`，发布 `/enclosure_command`
- 算法核心 `voronoi.py::voronoi_enclose`：Voronoi 分区封控
- 关键参数：`enclosure_radius=25.0`、`min_dist=5.0`、`update_period=1.0`
- 空闲态下发 `target_x/y/z=NaN, enclosure_radius=0.0` 的 standby 命令
- README: `docs/interface/封控组接入指南.md`

#### 文档 `D2`
- `docs/interface/感知组优化工作总结.md`（194 行，新结构）
- `docs/architecture/感知组架构图.md`（4 张 Mermaid 图：整体 / 内部 / 坐标变换 / 多源融合）
- `docs/integration/全链路联调手册.md`（构建、启动、bag 回放、ready 检查、FAQ 7 条、性能基准）
- `docs/interface/感知组融合接入指南.md`
- `docs/interface/目标转换设计.md`
- `docs/interface/目标转换接入指南.md`
- `docs/integration/`（联调相关文档目录）

### Fixed

- **【A4 真实 bug】motion_mode 在 8-state BoT-SORT KF 下错位**
  - `cvtrack/types.py::get_speed()` 与 `detect_motion_mode()` 原本用 `mean[2:4]` 当 vx,vy
  - 8-state KF mean 是 `[cx, cy, w, h, vx, vy, vw, vh]`，所以 `mean[2:4]` 永远是 w,h≈20
  - 直接后果：所有 BoT-SORT 轨迹 speed=28.28，无差别误判为 `fast`
  - 修复：抽出 `_velocity_xy()`，按 `len(mean)` 分支（4-state→[2,3]、8-state→[4,5]）。修复后回归 145 passed，0 skip、0 failed

### Notes / 遗留项

| 项 | 优先级 | 备注 |
|----|--------|------|
| 多源融合 ROS2 端到端回环验证 | 中 | 仅单元测试通过；上线前需两路真实 publisher |
| Re-ID 外观模型使能 | 低 | `appearance.enabled=false`，需提供权重文件 |
| tracker_node → enclosure_node 的 `/enclosure_targets` 通路 | 中 | 当前封控节点订阅 `/enclosure_targets`，tracker_node 暂未实现该发布路径，需协调实现或对接 |
| scheduler_node 当前订阅像素坐标 `/target_track` | 低 | 若调度想用世界坐标，需改 `target_topic` 参数为 `/target_track_world` |
| `kf_covariance` 真实填充 | 低 | `Track` 类型暂未暴露 KF 协方差到记录字段 |

### Verified

- `pytest -m "not slow and not reid"` → **145 passed, 0 skipped**
- `pytest tests/test_fusion.py -v` → **9 passed**
- `python3 -m py_compile` 所有改动节点/模块 → 无错
- ReadLints 改动文件 → 无错

---

*生成于 2026-07-26，覆盖 Phase 1 / 2 / 3 / 4 + A4 验证闭环。*