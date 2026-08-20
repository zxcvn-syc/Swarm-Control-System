# 演示视频素材清单

## 已验证闭环数据

[video_closed_loop_v4_20260820.md](video_closed_loop_v4_20260820.md) 归档了同一次 ROS 回放中实际产生的原始追踪、世界坐标、任务、路径和围控 topic trace。当前不交付闭环演示视频；任何后续视频必须从该次或新的同链路 trace 生成，不能将独立回放画面与手工世界坐标拼接后表述为端到端结果。

| 段落 | 时长 | 录制方式 | 必须保留的原始证据 | 当前状态 |
|---|---:|---|---|---|
| 系统部署 | 30 秒 | Gazebo 桌面录制 | SITL 启动日志、`/uav0/mavros/state` 截图 | SITL smoke 已验证；桌面录制待采集 |
| 目标检测与 DeepSORT | 60 秒 | 真实视频叠加跟踪框 | 原始输入、`tracked.mp4`、轨迹 CSV | 已有历史感知样张；需复核后入片 |
| 坐标转换与多源融合 | 45 秒 | ROS topic / 可视化录制 | `/target_track_world`、fusion pytest 日志 | 测试已通过，演示画面待采集 |
| 调度与路径规划 | 60 秒 | ROS 图与路径叠加 | `/task_assignment`、`/planned_path` 录制 | headless 三链路已运行；桌面可视化待采集 |
| 空地协同封控 | 90 秒 | Gazebo 场景录制 | `/enclosure_command`、场景配置 | 待 Gazebo 完整运行 |
| 三场景剪辑 | 90 秒 | Gazebo 仿真 + 两段视频输入回放 | 每段原始视频、时间戳、场景参数 | 已完成；来源边界写入画面 |
| 指标结尾 | 30 秒 | PPT/可视化 | 七项指标原始 CSV/JSON | 两小时 headless 数据已归档；其余指标待采集 |

`record_three_links.sh --mode ffmpeg` 才生成真实 MP4；`pseudo` 与 `ros2bag` 模式只产生日志或 bag，不能作为演示视频交付。
