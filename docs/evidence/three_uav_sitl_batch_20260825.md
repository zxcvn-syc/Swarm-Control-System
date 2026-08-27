# 三机 PX4 SITL 批量稳定性证据（2026-08-25）

## 验收对象与边界

本记录对应三个 PX4 v1.14 iris 实例与一个 Gazebo Classic `gzserver` 的
**进程启动和存活稳定性**。每轮要求 Gazebo 和三个 PX4 子进程在给定稳定窗口内
持续存活，并保存每轮结果与日志。

该验收不启动 MAVROS，不发布 setpoint，不执行 ARM，也不请求 `OFFBOARD` 模式。
因此它不能替代 MAVROS 连接、Offboard 状态机或真机飞行的证据。

## 已有原始执行记录

历史实际批测在 Ubuntu 22.04 SITL 主机上完成，原始交付目录保存在本机：

```text
C:\Users\911MT\Desktop\cvtrack\CVTrack_何泓林_综合交付_20260826\02_三无人机SITL批测_历史原始证据\run\batch_summary.json
```

其 `batch_summary.json` 记录：

| 字段 | 数值 |
|---|---:|
| 运行轮数 | 20 |
| 每轮稳定窗口 | 60 s |
| 成功轮数 | 20 |
| 成功率 | 1.0 |
| 重试 | 0 |
| 结果 | `passed: true` |
| 当时源码版本 | `5851a4f63c140bda400ece30436d6c534c1557ec` |

历史记录的每轮原始日志和结果位于同一目录的 `trial_XX/attempt_XX/`。这份记录
代表当时主机与该源码版本的实际执行，不代表本次工作树已在另一台主机重新执行。

## 当前仓库复现入口

当前实现将批测器和受控启动器一并纳入版本控制：

- `simulation/scripts/run_3uav_sitl_batch.py`
- `simulation/px4_sitl_3uav/start_3uav_sitl.sh`

在装有 PX4 v1.14、Gazebo Classic 和构建产物的**隔离仿真主机**运行：

```bash
cd /path/to/Swarm-Control-System
python3 simulation/scripts/run_3uav_sitl_batch.py \
  --px4-sitl-root ~/src/PX4-Autopilot \
  --runs 20 \
  --duration 60 \
  --startup-timeout 60 \
  --output-dir simulation/results/three_uav_sitl_batch_$(date +%Y%m%d_%H%M%S)
```

输出目录包含 `batch_manifest.json`、`batch_summary.json`、每轮的
`launcher.log`、`result.json`、`gzserver.log` 和 PX4 `out.log`/`err.log`。
`batch_summary.json` 只有在所有轮次均通过时才写入 `passed: true`，且批测器此时
才返回零退出码。

若主机存在先前残留的 Gazebo/PX4 进程，可在确认这是隔离仿真主机后额外传入
`--cleanup-leftovers`。该选项会终止旧的 Gazebo/PX4 进程，不能在共享或真实飞控
环境中使用。

## 本次复核状态

截至 2026-08-27，原 Ubuntu 22.04 VM 的 SSH 入口不可达，无法诚实地声称已在
本次源码版本上重新完成上述 20 轮。仓库已具备同口径、可审计的复现工具；待可达
的 PX4/Gazebo 主机运行后，应将新的输出目录随提交或制品归档，并更新本记录。

## 当前源码复跑（2026-08-27）

VM 恢复可达后，在独立 Git worktree 中以提交
`825a072895c9f906bacfc041541d183519dd1769` 重新执行了完整批测：

| 项目 | 实际值 |
|---|---|
| 环境 | Ubuntu 22.04、Python 3.10.12、PX4 v1.14.0、Gazebo Classic 11.10.2 |
| world | `simulation/worlds/swarm_field.world` |
| 轮数 | 20 |
| 每轮稳定窗口 | 60 s |
| 重试 | 0 |
| 通过轮数 | 20 |
| 成功率 | 1.0 |
| 结果 | `passed: true` |

版本控制内的原始汇总和清单为：

- `docs/evidence/three_uav_sitl_batch_20260827/batch_summary.json`
- `docs/evidence/three_uav_sitl_batch_20260827/batch_manifest.json`

运行结束后确认没有该批测留下的 `gzserver` 或 PX4 进程。该结果仅证明每轮三个
PX4 实例和 Gazebo 在自定义场景中可重复启动并持续存活；不证明 MAVROS、ARM、
`OFFBOARD`、任务完成、碰撞规避或真机飞行。
