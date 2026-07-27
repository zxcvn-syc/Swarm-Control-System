# CVTrack 测试视频集

本目录包含用于测试 cvtrack 目标跟踪系统的测试视频。

## 视频来源

所有视频均来自 `/home/hhh/Downloads/cv_tracking_demo/` 目录，以符号链接形式存在。

## 测试视频清单

| 文件名 | 分辨率 | 时长(秒) | 适用场景 | 说明 |
|--------|--------|----------|----------|------|
| `test_urban_pedestrian.mp4` | 1920x1080 | 15.4 | 多目标跟踪、尺度变化 | 城市步行街视角，行人和骑行者 |
| `test_highway_traffic.mp4` | 1920x1080 | 18.0 | 快速运动、多目标跟踪 | 高速公路立交桥俯视，多车道车辆 |
| `test_multi_target_tracking.mp4` | 1920x1080 | 18.8 | **多目标跟踪** | 城市道路交叉口，密集交通场景 |
| `test_scale_variation.mp4` | 1920x1080 | 13.5 | 尺度变化、遮挡场景 | 有轨电车和行人，视角变化 |
| `test_drone_aerial.mp4` | 640x360 | 28.6 | **无人机视角、快速运动** | 低空无人机航拍，适合 VisDrone 检测器 |
| `test_drone_high_altitude.mp4` | 640x360 | 17.8 | 无人机视角、目标重识别 | 高空航拍，小目标场景 |
| `test_occlusion_scenario.mp4` | 640x360 | 15.8 | **遮挡场景** | 斑马线行人过街，存在相互遮挡 |
| `test_fast_motion.mp4` | 854x480 | 52.2 | **快速运动** | Sintel 电影预告片，动作场景 |

## 场景推荐

### 1. 多目标跟踪
- **推荐视频**: `test_multi_target_tracking.mp4` (coverr_road_traffic)
- **原因**: 密集交通场景，车辆众多，跟踪难度高
- **建议配置**: `--config drone --tracker deepsort_cascade`

### 2. 快速运动
- **推荐视频**: `test_fast_motion.mp4` (sintel_trailer)
- **原因**: 动作场景，物体移动速度快
- **建议配置**: `--tracker botsort --predict-horizon 20`

### 3. 遮挡场景
- **推荐视频**: `test_occlusion_scenario.mp4` (pexels_pedestrian_crossing)
- **原因**: 行人过街时存在相互遮挡
- **建议配置**: `--tracker deepsort_cascade --reid`

### 4. 尺度变化
- **推荐视频**: `test_scale_variation.mp4` (coverr_tram_city)
- **原因**: 有轨电车由远及近，尺度变化明显
- **建议配置**: `--config drone`

### 5. 无人机视角
- **推荐视频**: `test_drone_aerial.mp4` (pexels_aerial_2034115)
- **原因**: 专为 VisDrone 检测器优化
- **建议配置**: `--config drone --detector yolo --weights /path/to/visdrone_yolov8s.pt`

## 使用方法

### 快速测试（所有视频）
```bash
cd /home/hhh/Downloads/Swarm-Control-System

# 测试单个视频
python -m cvtrack --source videos/test_multi_target_tracking.mp4 \
    --out-dir output/test_multi_target \
    --config drone --tracker deepsort_cascade \
    --max-frames 200

# 使用 VisDrone 检测器（无人机场景）
python -m cvtrack --source videos/test_drone_aerial.mp4 \
    --out-dir output/test_drone \
    --config drone --detector yolo \
    --weights cv_tracking_demo/weights/visdrone_yolov8s.pt \
    --tracker deepsort_cascade --reid
```

### 批量测试
```bash
# 使用 cvtrack 自带的统计脚本
cd /home/hhh/Downloads/cv_tracking_demo
python3 scripts/collect_stats.py \
    --sources ../Swarm-Control-System/videos/*.mp4 \
    --out-dir weights/all_runs \
    --max-frames 200 \
    --config drone
```

### 运行 BoT-SORT（推荐用于快速运动）
```bash
python -m cvtrack --source videos/test_highway_traffic.mp4 \
    --out-dir output/botsort_highway \
    --tracker botsort \
    --predict-horizon 15 \
    --write-future-csv
```

### 运行 DeepSORT 级联（推荐用于遮挡场景）
```bash
python -m cvtrack --source videos/test_occlusion_scenario.mp4 \
    --out-dir output/deepsort_occlusion \
    --tracker deepsort_cascade \
    --reid
```

## 预期指标

根据 cv_tracking_demo 的基准测试：

| Tracker | MOTA | IDF1 | 特点 |
|---------|------|------|------|
| DeepSortCascade | 高 | 高 | 遮挡后重识别能力强 |
| BoT-SORT | 高 | 中 | 快速运动表现好 |
| DeepSort (旧版) | 中 | 中 | 基线对比 |

## 注意事项

1. **首次运行**: 可能需要安装依赖和下载模型权重
   ```bash
   cd /home/hhh/Downloads/cv_tracking_demo
   make install
   ```

2. **ReID 功能**: 需要安装 torchreid（可选）
   ```bash
   pip install torchreid
   ```

3. **GPU 加速**: 如有 NVIDIA GPU，建议安装 CUDA 版 PyTorch 以提升性能

4. **视频路径**: 所有视频使用符号链接，不会占用额外空间

## 更新日志

- 2026-07-27: 初始创建，整合 cv_tracking_demo 中的测试视频
