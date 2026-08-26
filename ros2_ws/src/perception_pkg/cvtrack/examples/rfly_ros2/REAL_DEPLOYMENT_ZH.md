# CVTrack 真机部署与 PX4 控制门控

本文档用于把 Rfly/ROS2 演示推进到真机前的审核流程。当前仓库已经能提供视觉触发、ROS2 调度、规划和地面封控的闭环证据；它仍然不是 PX4 Offboard 或真机飞控证明。

## 必须先完成的证据包

每次候选部署都要先跑完整 Rfly 联调，并保留同一输出目录中的原始证据：

- `validation.json`：必须为 `passed: true`。
- `capture_summary.json`：ROS2 证据采集不能有 `pending` 话题。
- `evidence_manifest.json`：每个关键话题都要有首条有效 payload 文件。
- `scene_telemetry.jsonl`、`tracks.csv`、`uav_live.mp4`、`decision_god_view.mp4`：用于人工复核视觉锁定、预测、重捕获和封控过程。

`capture_ros_evidence.py` 会为 `/target_track_world`、`/task_assignment`、`/planned_path`、`/enclosure_command`、`/drone_states`、`/ground_vehicle_states` 和 `/target_track_truth` 写出首条有效消息。PR 审核时只把这些文件当作仿真闭环证据，不能把它们描述成真机控制已经执行。

## PX4 控制前置门

任何 ARM、PX4 Offboard、MAVROS setpoint 或实车执行器控制入口前，都必须先调用：

```bash
CVTRACK_PX4_CONTROL_ALLOWED=YES_I_ACCEPT_REAL_VEHICLE_RISK \
python tools/px4_control_gate.py \
  --mode control \
  --allow-px4-control \
  --evidence-dir outputs/rfly_full_demo_YYYYMMDD_HHMMSS \
  --world-calibration calibrations/site_camera_01.yaml \
  --mavros-state-file /tmp/mavros_state.yaml \
  --operator-approval /tmp/operator_approval.yaml \
  --write-decision outputs/rfly_full_demo_YYYYMMDD_HHMMSS/px4_gate_decision.json
```

门控失败时退出码为 `3`，调用方必须停止控制链路。`--mode audit` 只生成审核报告，永不授权控制。

## 输入文件要求

`--world-calibration` 必须是人工标定的米制地面平面文件，至少包含 4 个地面对应点，且 `frame_id`/`world_frame` 为 `world`、`map` 或 `local_origin`，`units` 为 `m`。

`--mavros-state-file` 可以从新鲜的 `/mavros/state` 采样后整理为 YAML/JSON：

```yaml
connected: true
armed: false
mode: POSCTL
age_s: 0.4
```

`age_s` 必须小于默认 2 秒；默认不允许车辆已经处于 armed 状态。

`--operator-approval` 必须由现场负责人填写：

```yaml
mission_id: cvtrack-field-001
approved_by: operator_name
safety_pilot_present: true
kill_switch_tested: true
geofence_checked: true
battery_checked: true
propeller_area_clear: true
manual_takeover_ready: true
```

## 合并边界

- Rfly 演示代码可合入 `main`，作为仿真演示和证据生成工具。
- `px4_control_gate.py` 只提供控制前置拒绝/授权判定，不包含 ARM、Offboard 切换或 setpoint 发布。
- 真机分支在没有现场标定、MAVROS 新鲜状态和人工审批前，不能把任何视觉目标坐标发送到真实飞控。
