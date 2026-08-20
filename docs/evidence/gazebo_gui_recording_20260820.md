# Gazebo GUI 录制证据

## 录制结果

- 录制日期：2026-08-20
- 环境：Ubuntu 22.04 ROS 虚拟机 `192.168.88.135`
- 显示方式：`Xvfb :99`，软件 OpenGL，Gazebo Classic GUI
- 仿真：PX4 SITL v1.14 + Gazebo Classic，场景 `simulation/worlds/swarm_field.world`
- 输出：`videos/gazebo_gui_final_20260820.mp4`
- 规格：H.264 MP4，1280x720 原始桌面采集，90.000 秒
- 画面处理：最终剪辑裁掉 Xvfb 无窗口管理器造成的黑色右/下边界并缩放到 1920x1080

## 实际核验

- PX4 日志出现 `Ready for takeoff!`，且连接到 Gazebo simulator TCP 4560。
- 中段关键帧可见 Gazebo GUI、`swarm_field` 场景和 Iris 模型。
- 录制结束后未发现本次测试遗留的 `gazebo`、`gzserver`、`gzclient`、`Xvfb`、`ffmpeg` 或 `px4` 进程。
- 未执行 ARM、Offboard 激活、起飞或 setpoint follower；该文件是 GUI 仿真录制，不是真机飞行证据。

## 三场景精剪

- 输出：`videos/three_scene_system_demo_20260820.mp4`
- 规格：H.264 MP4，1920x1080，30 fps，90 秒
- 场景 1：实际 PX4/Gazebo GUI 仿真
- 场景 2：`data/demo_inputs/airport_tracked.mp4`，Ultralytics airport ground-vehicles 跟踪回放，原生 1280x720
- 场景 3：`data/demo_inputs/parking_tracked.mp4`，Ultralytics overhead parking 跟踪回放，原生 1280x720

场景 2 和场景 3 是公开样例上的感知输入回放，不是 VisDrone、UA-DETRAC、UAVDT 或 AU-AIR 的数据集评测，也不提供真实地理配准。成片顶部字幕已显式标注这一边界。

## 可复现命令

```bash
./scripts/record_gazebo_gui_xvfb.sh --out output/video_work/gazebo_gui_final.mp4 --duration 90
cd video-remotion
./prepare-assets.ps1
pnpm exec remotion render CVTrackThreeSceneDemo ../videos/three_scene_system_demo_20260820.mp4 --codec=h264 --crf=18
```

`video-remotion/` is the editable primary timeline, including the 90-second composition,
transition timing, subtitles, and source-boundary labels. Its `public/media/` directory is
generated locally by `prepare-assets.ps1` and deliberately excluded from Git.

When Node.js/Remotion is unavailable in the ROS VM, the following FFmpeg script remains a
non-editable fallback with the same three source inputs:

```bash
./scripts/render_three_scene_demo.sh --out videos/three_scene_system_demo_20260820.mp4
```
