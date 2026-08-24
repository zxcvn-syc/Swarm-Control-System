# 场景矩阵 2026-08-22

本目录的场景预设可通过 `RFLY_SCENARIO` 选择。每个预设同时驱动 Rfly 地图/天气、动态障碍物和视觉压力层。

| 场景 | 地图 | 天气 | 动态障碍 | 视觉遮挡/压力 |
|---|---|---|---:|---|
| `clear_grasslands` | Grasslands | clear | 0 | baseline |
| `rain_3ddisplay` | 3DDisplay | rain | 3 | rain + fog + blur + periodic occlusion |
| `fog_3ddisplay` | 3DDisplay | fog | 3 | fog + blur + periodic occlusion |
| `snow_3ddisplay` | 3DDisplay | snow | 2 | snow + fog + blur + periodic occlusion |
| `city_clear` | Changsha | clear | 4 | light fog + periodic occlusion |
| `mountain_clear` | MoutainRoad | clear | 3 | light fog + blur + periodic occlusion |

## 实测短片统计

| 场景 | 处理帧 | 确认跟踪行 | 合成遮挡帧 | 视角交接 |
|---|---:|---:|---:|---:|
| `clear_grasslands` | 484 | 480 | 0 | 1 |
| `fog_3ddisplay` | 289 | 288 | 6 | 1 |
| `snow_3ddisplay` | 314 | 311 | 7 | 1 |
| `city_clear` | 337 | 334 | 17 | 1 |
| `mountain_clear` | 314 | 314 | 4 | 1 |

雨天主验证另外录制了 42 秒全过程成片，包含 5 次按时多视角交接和 ROS2 三车封控证据。场景矩阵短片用于比较天气、地图和遮挡压力，不替代主验证的完整 ROS2 证据。
