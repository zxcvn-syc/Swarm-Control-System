# cvtrack v6 —— 真正的 DeepSORT 级联匹配器 + BoT-SORT，带 Kalman 未来帧预测

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)]()
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-yellow.svg)]()
[![Trackers: deepsort | deepsort_cascade | botsort](https://img.shields.io/badge/trackers-deepsort%20%7C%20deepsort__cascade%20%7C%20botsort-blue.svg)()

> **关于本仓库。** 包名是 **cvtrack**，只提供计算机视觉跟踪流水线 —— **不包含 ROS / launch 文件 / ROS 话题**。
> 仓库目录原先叫 `<owner>/ROS`，现已改名为 `<owner>/cvtrack`；如果你从 "ROS" 搜索结果点到这里，那来对地方了。

> ⚠️ **`configs/drone.yaml` 默认的检测器是 MOG2，不是 YOLO**。
> 这是有意为之：为了保留 v4 的兼容性基线（`pexels_aerial_2034115` 上 798 个 ID / 平均长度 23.2），
> 级联匹配器的对比数字就是建立在这个基线之上的。如果你想在无人机跟踪阈值下使用 YOLO，
> 请显式传 `--detector yolo`（并传 `--weights /abs/path.pt`）——`scripts/visdrone_compare.py`
> 已经替你做了这件事。完整说明见 [Configs](#configs)。

`cvtrack` 是以 package 形式发布的跟踪流水线。v6 是**真正的 DeepSORT 发布版**：
它实现了忠实于原论文的匹配级联（Mahalanobis 门控 + 外观级联 + IoU 兜底），
带不确定度椭圆的 Kalman n 步预测，IDF1 指标，以及为下一版本铺路的传感器抽象层
（LiDAR / IMU / 多相机）。

`pexels_aerial_2034115.mp4`（200 帧，真实 YOLO 推理，权重 `weights/visdrone_yolov8s.pt`，
类别 `0,1,2,3,4,5,7,8`，CPU 设备，cascade 行使用 drone 预设）的关键数字：

| tracker              | IDs  | ID/frame | mean length | median | longest | total obs | avg FPS |
|----------------------|-----:|---------:|------------:|-------:|--------:|----------:|--------:|
| DeepSortCascade      |   56 |    0.280 |       60.3 |     54 |     174 |     3,379 |     0.9 |
| DeepSortCascade raw  |  113 |    0.565 |       19.5 |      8 |     104 |     2,199 |     2.6 |
| BoT-SORT             |   82 |    0.410 |       13.2 |      4 |      83 |     1,083 |    12.3 |
| DeepSort (legacy)    |  110 |    0.550 |       20.7 |     10 |     159 |     2,277 |    11.8 |

Drone 预设（低 min-conf，8 个 VisDrone 类别，IoU 门控 0.20，4 自由度 Mahalanobis 门控 9.4877）下的 DeepSortCascade
以巨大优势给出最长的轨迹 —— 中位数轨迹长度 54 vs BoT-SORT 的 4。
Cascade 行通过 torchreid 使用 OSNet 512 维嵌入（`weights/osnet_x0_25_msmt17.pth.tar`，
`loaded_pretrained=True`）；raw 行跑级联但不应用预设调整，可以看到 ID 爆炸警告如何触发
（比率 0.565 > 0.5）。保留的运行产物位于 `weights/run_deepsort_cascade_drone/`、
`weights/run_deepsort_cascade/`、`weights/run_botsort/` 和 `weights/run_deepsort_legacy/`。

## v6 新特性

* **真正的 DeepSORT 级联匹配器** —— `cvtrack.tracker.deepsort.DeepSortCascade`
  按原 DeepSORT 论文实现匹配级联：已确认的轨迹按 age 顺序匹配（刚丢失的最先），
  每一层内做 Mahalanobis 距离的卡方门控，门控内的代价是到该轨迹 ReID gallery 均值的余弦距离。
  级联未匹配的进入 IoU 兜底。新构造参数：`use_appearance=True`、
  `appearance_thresh=0.5`、`max_age=30`、`n_init=3`。

* **通过 torchreid 使用 OSNet** —— `cvtrack.appearance.osnet.OsNetExtractor` 是 v6 中唯一的外观后端。
  它通过 torchreid（或其内置的 OSNet 参考实现）加载 `osnet_x0_25`，并使用 MSMT17 微调的 checkpoint。
  如果两套网络栈都无法 import，工厂返回 `None`，流水线优雅降级为纯运动跟踪并打印明确警告，
  而不是悄悄使用颜色直方图兜底。

* **带协方差的 Kalman n 步预测** ——
  `cvtrack.tracker.kalman.predict_n_steps_with_covariance` 在每个投影步返回完整的状态协方差。
  渲染器现在会在每个未来步绘制 3σ 不确定度椭圆
  （`cvtrack.viz.renderer.draw_predicted_future_trail`），当 `--write-future-csv` 打开时，
  future CSV 会新增 `sigma_x` / `sigma_y` 列。

* **IDF1 指标** —— `cvtrack.tracker.metrics.idf1` 是一个独立的 IDF1 / IDP / IDR 实现，
  使用贪心的 1-to-1 最佳匹配。在 `tests/test_metrics.py` 中有单元测试。
  当接入了真实的 ground-truth 源（MOT17-mini、自定义标注等）后，它将作为头部 MOT 指标使用。

* **传感器抽象层** —— `cvtrack.sensors.Sensor` 是一个极简的抽象基类（read / 内参 / 外参），
  `cvtrack.sensors.VideoSensor` 是默认的视频后端。v6 中流水线仍直接消费 `VideoReader`，
  但所有下游调用方现在都可以接收一个 `Sensor`，为 v7 的 LiDAR / IMU / 多相机做好准备。

* **多传感器预设** —— `configs/multi_sensor.yaml` 是一个占位文件，
  文档化了即将到来的 `sensors:` 列表 schema，可以安全地通过 `--config multi_sensor` 传参
  （会回退到与 `default.yaml` 相同的默认）。

* **新的 CLI flag**：
  - `--tracker {deepsort, deepsort_cascade, botsort}`（默认未变）
  - `--predict-horizon <int>`（默认 15）控制 KF 未来帧水平
  - `--write-future-csv` 开启带 sigma 标注的 future CSV
  - `--reid` 现在在 `--tracker deepsort_cascade` 时自动启用

## 快速上手

```bash
git clone <repo>
cd cvtrack
python3 -m pip install -e .

# v6 默认：BoT-SORT，带 KF 未来帧投影 + 不确定度椭圆
python -m cvtrack --tracker botsort \
    --source pexels_aerial_2034115.mp4 \
    --out-dir weights/run_botsort --max-frames 200 --save-trail

# v6 主打：真正的 DeepSORT 级联匹配器（自动开启 appearance）
python -m cvtrack --tracker deepsort_cascade \
    --source pexels_aerial_2034115.mp4 \
    --out-dir weights/run_deepsort_cascade --max-frames 200 \
    --write-future-csv --predict-horizon 15

# 旧版 DeepSORT，用于向后兼容的数字
python -m cvtrack --tracker deepsort \
    --source pexels_aerial_2034115.mp4 \
    --out-dir weights/run_deepsort_legacy --max-frames 200

# 把所有 v6 运行汇总到一个 JSON
python3 scripts/summarise_v6_runs.py > weights/v6_runs_summary.json
```

安装完成后，package 以 editable 模式安装。**模型权重不在仓库里**（太大且与特定来源绑定），
先用下面命令拉取一次：

```bash
python3 scripts/download_weights.py
```

它会生成：

```text
weights/
├── visdrone_yolov8s.pt          ~22 MB, drone 微调的检测器
└── osnet_x0_25_msmt17.pth.tar   ~9 MB, 小型 ReID 头
```

再次运行流水线时，每次运行的输出（CSV、JSON 摘要、渲染视频）会写入
`weights/run_<tracker>/` 等子目录。这些都被 `.gitignore` 覆盖，所以重跑不会误提交。

## 架构

```
cvtrack/
├── pyproject.toml            # 构建 + 工具配置
├── requirements.txt
├── Makefile                  # install / lint / test / docker-build
├── Dockerfile
├── .github/workflows/ci.yml
├── configs/                  # YAML 预设（default, drone, street, multi_sensor）
├── src/cvtrack/              # package 主体
│   ├── pipeline.py           # 主 CLI
│   ├── config.py             # YAML 加载器、校验器
│   ├── types.py              # Box, Detection, Track
│   ├── detector/             # YOLO, MOG2, factory
│   ├── tracker/
│   │   ├── botsort.py        # 8 状态 KF + CMC + ReID 融合
│   │   ├── deepsort.py       # DeepSortLite (legacy) + DeepSortCascade (v6)
│   │   ├── kalman.py         # KalmanCV2D + KalmanBoT + n 步投影
│   │   ├── metrics.py        # IoU, gating, IDF1 (v6)
│   │   ├── smoother.py       # RTS
│   │   └── cmc.py            # sparse-OF + ECC
│   ├── appearance/
│   │   ├── osnet.py          # OSNet (torchreid, v6 唯一 appearance 后端)
│   │   ├── gallery.py        # per-track FIFO + EMA
│   │   └── factory.py        # 后端分发器
│   ├── sensors/              # v6 抽象层（骨架）
│   │   ├── base.py           # Sensor, Frame, Intrinsics, Extrinsics
│   │   └── video.py          # VideoSensor（包装 VideoReader）
│   ├── viz/                  # box/trail/不确定度渲染器
│   ├── io.py                 # 视频 + CSV 写入器 + 带 sigma 的 future-CSV
│   └── postprocess.py        # CLI: tracks.csv -> plots
├── tests/                    # pytest，仅 CPU
├── eval/mot17_mini/          # MOT17-mini + 合成评测
├── scripts/
│   ├── setup_visdrone_compare.sh    # 安装 + 下载权重
│   ├── visdrone_compare.py          # 4 路 head-to-head runner
│   ├── collect_stats.py             # 多视频汇总
│   ├── summarise_v6_runs.py         # v6 头部指标
│   ├── download_weights.py          # 独立权重下载器
│   └── run_cvtrack.sh
```

sensor 模块在 v6 中是**骨架**：它的存在是为了下一版流水线能组合多个 `Sensor` 实例
（LiDAR + IMU + 多相机）而无需修改 tracker 的契约。在此之前，主流水线仍直接消费 `VideoReader`。

## Configs

| Config          | ImgSz | Conf | HighConf | NewConf | IoU  | Relink | ReID    | Notes                  |
|-----------------|------:|-----:|---------:|--------:|-----:|-------:|---------|------------------------|
| `default`       |   320 | 0.15 |     0.35 |    0.20 | 0.30 |     30 | off     | 通用 / 网页            |
| `drone`         |   480 | 0.12 |     0.22 |    0.07 | 0.20 |     45 | opt-in  | 小目标移动框          |
| `street`        |   480 | 0.25 |     0.50 |    0.25 | 0.35 |     20 | off     | 1080p 街景            |
| `multi_sensor`  |   320 | 0.15 |     0.35 |    0.20 | 0.30 |     30 | off     | 多传感器占位（v7+）   |

YAML 与 CLI 同时给定时优先级：CLI > YAML > preset 默认值。

`--drone` 是 `--config configs/drone.yaml` 的语法糖。注意 `drone.yaml` 历史原因设置了
`detector.backend: mog2` 以保留 v6 兼容基线 —— 若要在无人机跟踪阈值下用 YOLO，
请显式传 `--detector yolo`（对比 runner 就是这么做的）。

## Head-to-head runner

`scripts/visdrone_compare.py` 仍可用于检测器 / ReID 的矩阵对比。
其输出目录是一次性的，并使用语义化子目录：

| 输出目录          | 配置                              |
|-------------------|-----------------------------------|
| `run_mog2`        | MOG2 兼容基线                    |
| `run_coco`        | COCO YOLOv8s, 无 ReID            |
| `run_visdrone`    | VisDrone YOLOv8s, 无 ReID        |
| `run_visdrone_reid` | VisDrone YOLOv8s + OSNet ReID  |

受维护的 v6 tracker 对比是独立的，位于 `weights/run_deepsort_legacy/`、
`weights/run_deepsort_cascade/` 和 `weights/run_botsort/`。每个目录包含渲染视频、
标准 track CSV、带 `sigma_x`/`sigma_y` 列的 15 步 future CSV、平滑 CSV 和 trail JSON。

输出：`weights/COMPARE_REPORT.md`（markdown 表格）加上 `weights/summary.json`（机器可读）。
脚本具备容错性：如果某个权重文件缺失或可选的 ReID import 失败，会记录失败并继续跑其他有效配置。

## 8 视频检测器基准

要看 COCO vs VisDrone 选择在其余样例片段上的表现（每个 200 帧，无人机跟踪预设）：

| video                      | COCO yolov8s IDs | COCO mean_len | VisDrone yolov8s IDs | VisDrone mean_len |
|----------------------------|-----------------:|--------------:|---------------------:|------------------:|
| coverr_city_walk           |               47 |          41.2 |                   27 |              22.5 |
| coverr_highway_overpass    |              277 |          10.7 |                  128 |              19.9 |
| coverr_road_traffic        |              773 |          11.2 |                3 872 |               7.4 |
| coverr_tram_city           |              331 |           6.5 |                  243 |              21.4 |
| pexels_aerial_2034115      |              481 |           6.4 |                  154 |              19.4 |
| pexels_aerial_2257013      |               49 |           5.6 |                    1 |              79.0 |
| pexels_pedestrian_crossing |               37 |          26.5 |                   13 |              48.5 |
| sintel_trailer             |               12 |          27.8 |                   43 |              23.5 |

复现命令：

```bash
python3 scripts/collect_stats.py \
    --sources *.mp4 \
    --out-dir weights/all_runs \
    --max-frames 200 \
    --config drone
```

…要拿单检测器的数字，把 YOLO 权重换成 `--weights` 指定的并加 `--detector yolo`。

## 评测方法

### 自测（无需数据）
```bash
python eval/mot17_mini/run_eval.py --synthetic
```

它会合成一段 100 帧的片段，里面有 3 个已知运动的矩形，跑跟踪器并报告 MOT 风格指标
（MOTA / IDF1 / MOTP）。`cvtrack.tracker.metrics.idf1` 也可以直接在 notebook 中调用，
用于自定义 MOT 风格分析（无外部依赖）。

### 对自定义 ground-truth 算 IDF1
```python
from cvtrack.tracker.metrics import idf1
from cvtrack.types import Box

gt_ids   = [1, 1, 1, 2, 2]
pred_ids = [7, 7, 8, 9, 9]   # your tracker's ids
gt_boxes = [Box(*b, score=1.0, cls=0, label="obj") for b in ...]
pr_boxes = [Box(*b, score=1.0, cls=0, label="obj") for b in ...]
print(idf1(gt_ids, pred_ids, gt_boxes, pr_boxes))
# {'idf1': ..., 'idp': ..., 'idr': ..., 'mapping': {1: 7, 2: 9}, ...}
```

### 真实 MOT17 mini
```bash
# 1. 下载 2 个序列（例如 MOT17-04-FRCNN、MOT17-13-FRCNN）
# 2. 整理成：
#    eval/mot17_mini/data/MOT17/train/MOT17-04-FRCNN/img1/...
#    eval/mot17_mini/data/MOT17/train/MOT17-04-FRCNN/gt/gt.txt
# 3. 跑：
python eval/mot17_mini/run_eval.py \
    --data-root eval/mot17_mini/data/MOT17/train \
    --sequences MOT17-04-FRCNN MOT17-13-FRCNN \
    --tracker botsort
```

## 如何添加自定义组件

* **Detector** —— 继承 `cvtrack.detector.base.DetectorProtocol`（或 `cvtrack.detector.base.Detector`），
  并在 `cvtrack.detector.factory.build_detector` 加一个分支。
* **Tracker** —— 实现 `step(dets, frame) -> tracks`，通过 `--tracker <name>` 传给流水线。
  新的 `DeepSortCascade` 是 appearance-aware 匹配器的参考实现；要自定义级联策略就继承它。
* **Appearance 模型** —— 继承 `cvtrack.appearance.osnet.OsNetExtractor`，并在
  `cvtrack.appearance.factory.make_extractor` 加一个分支。所有 appearance 模型必须是真实预训练网络
  （不允许直方图 / 合成 fallback）。
* **Sensor** —— 继承 `cvtrack.sensors.Sensor` 并把它传给流水线。契约有意做得极小，
  以便 LiDAR / IMU / 多相机适配器都能在 100 行内写完。

## 工程规范

* `requirements.txt`：固定到兼容范围（必要时使用 `>=` / `<`）。
* `pyproject.toml` 定义了 `[project]`，以及 `[tool.ruff]`、`[tool.mypy]`、`[tool.pytest.ini_options]`。
* `Dockerfile` 基于 `python:3.11-slim`，以 editable 方式拷贝源码安装，默认 `--help`。
* `Makefile` 暴露 `install`、`lint`、`typecheck`、`test`、`test-slow`、`run-drone`、`docker-build`、`clean`。
* CI：ruff check、mypy（尽力而为）、跨 Python 3.10/3.11 的快速 pytest。
* 测试套件在 Python 3.10 上 53 通过 + 1 跳过（torchreid-only）。

## 未来工作（下一版）

* **LiDAR + IMU + 多相机** —— `cvtrack.sensors` 模块已就绪；
  下一步是接入一个 `MultiSensorSource`，按时间戳同步多个 `Sensor` 实例并喂给单个 tracker。
* **MOT17 全量评测** —— `eval/mot17_mini/run_eval.py` 已经支持任意序列；
  加入完整的 MOT17 / KITTI 下载脚本是下一个交付物。
* **VisDrone 微调更大的 backbone**（yolov8m / yolov8l）。需要多小时的 GPU 训练；
  当前流水线使用 dronefreak 微调的 yolov8s。
* `torchreid` 在 Cython 不可用时的 **ONNX runtime 兜底**。
* GPU 上的 **TensorRT / DNN-based MOG2**。

## Troubleshooting

* **`No module named 'torchreid'`** 出现在 ReID 运行中。
  `torchreid` 1.0.6 带 Cython 扩展，无法在新版 setuptools 下编译。两个 workaround：
  1. `pip install --user "Cython<3"` 然后重跑 `pip install -e .[reid]`，或
  2. 跳过 ReID —— 级联匹配器会优雅降级到纯运动跟踪，head-to-head 仍能跑三种 tracker 配置。

* **`numpy.core.multiarray failed to import`** 在加载 ultralytics 时出现。
  本机 `~/.local` 是 `numpy 2.2.6`，但系统的 `matplotlib 3.5.1` 是针对 `numpy 1.x` 编译的。
  setup 脚本通过在 `/tmp/cvfix/` 下写一个迷你 `sitecustomize.py` 来绕过，
  让它在任何东西之前预先加载系统的 numpy 1.21.5 + scipy 1.8.0。
  source `scripts/setup_visdrone_compare.sh` 时会自动激活。
  要手动用这个修复跑 cvtrack：`PYTHONPATH=/tmp/cvfix:src python3 -m cvtrack …`。

* **`matplotlib cache permission denied`** —— matplotlib 尝试把字体缓存写到只读位置时的无害警告。

* **`yolov8s.pt` not found at the bare name** —— 用绝对路径传 `--weights /path/to/yolov8s.pt`。
  visdrone runner 始终传绝对路径，所以这只会影响手写 CLI 用法。

## License

Apache-2.0。详见 `LICENSE`。
