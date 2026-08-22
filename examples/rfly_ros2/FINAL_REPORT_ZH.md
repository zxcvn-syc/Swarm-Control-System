# 何泓林 CVTrack + RflySim3D 联调报告

## 本轮修复

- 原始航拍视频加入 `SEARCH / LOCK / HANDOFF` 状态和 `CAMERA HANDOFF Ux -> Uy` 横幅。
- 视觉主机默认每 8 秒在 UAV1、UAV2、UAV3 之间交接，保留 2.5 秒传感器稳定窗口。
- 决策回放增加蓝色目标真实轨迹、视觉投影轨迹、控制轨迹、预测箭头、三架 UAV 轨迹、三辆灰车轨迹、动态障碍物、世界运动小窗和全过程时间线。
- 遥测增加 `phase`、`host_changed`、目标速度和航向，便于把识别、预测、交接、恢复和封控对齐到视频。

## 雨天主验证

主视频：`rfly_multiview_motion_20260822_fresh.mp4`

决策视频：`rfly_multiview_decision_20260822.mp4`

- 地图：`3DDisplay`；天气：Rfly 雨天类型 5。
- 视觉压力：雾化 0.08、雨滴 120、模糊核 3、周期遮挡 19 秒/次、每次 0.7 秒。
- 场景障碍：3 辆动态遮挡车和 8 辆静态障碍车。
- 在线视觉：42 秒、1280x720、899 个处理帧、829 个确认跟踪行、899 个 UDP 视觉包。
- 多视角：5 次真实交接，时间为 8.0、16.0、24.0、32.0、40.0 秒。
- 同步窗口内目标真值从 `(171.2,149.8)` 移动到 `(8.8,118.1)` 米；决策视频右侧上帝视角和左下运动小窗持续显示这段运动。
- ROS2：`/target_track_world`、`/target_track_truth`、`/drone_states`、`/ground_vehicle_states`、`/planned_path`、`/task_assignment`、`/enclosure_command` 均收到有效消息。
- 封控：3 条有限坐标命令，`drone_id=0/1/2`，每条 `enclosure_radius=18.0 m`。

## 场景矩阵

短视频和统计文件位于 `scenario_matrix_20260822/`：

| 场景 | 地图 | 天气 | 动态障碍 | 遮挡级别 | 视觉压力 |
|---|---|---:|---:|---:|---|
| `clear_grasslands` | Grasslands | clear | 0 | 0.00 | 无 |
| `rain_3ddisplay` | 3DDisplay | rain | 3 | 0.35 | 雨、雾化、模糊、周期遮挡 |
| `fog_3ddisplay` | 3DDisplay | fog | 3 | 0.30 | 雾化 0.28、模糊、周期遮挡 |
| `snow_3ddisplay` | 3DDisplay | snow | 2 | 0.25 | 雪粒、雾化、模糊、周期遮挡 |
| `city_clear` | Changsha | clear | 4 | 0.45 | 轻雾、周期遮挡 |
| `mountain_clear` | MoutainRoad | clear | 3 | 0.38 | 轻雾、模糊、周期遮挡 |

## 复现

ROS VM：`192.168.88.135`；Windows/Rfly 主机：`192.168.88.1`；ROS Domain：`61`。

```bash
export RFLY_DEMO_ROOT=/home/hhh/Downloads/cvtrack-rfly-enhanced-20260821
export ROS2_WS_ROOT=/home/hhh/Downloads/Swarm-Control-System/ros2_ws
export ROS2_SETUP=$ROS2_WS_ROOT/install_validation/setup.bash
export RFLY_HOST_IP=192.168.88.1
bash $RFLY_DEMO_ROOT/scripts/run_ros_chain.sh 60 rain_3ddisplay
```

Windows 侧给 `rfly_live_cvtrack.py` 传入 `--view-cycle-s 8`，视频结束后用相同场景的 `scene_telemetry.jsonl` 运行 `make_decision_visualization.py`。

## 边界

这是“视觉触发的 Rfly 空地协同控制演示”。图像检测由蓝色目标语义分割式颜色候选加 BoT-SORT 完成，世界坐标控制在 Rfly Free 的动态相机换挂限制下保留 `truth_assist` 审计源；不能宣称为未经辅助的纯单目世界坐标闭环。此次没有 ARM、PX4 Offboard 或 MAVROS 飞控闭环。
