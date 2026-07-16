# cvtrack v6 — true DeepSORT cascade + BoT-SORT with Kalman future prediction

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)]()
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-yellow.svg)]()
[![Trackers: deepsort | deepsort_cascade | botsort](https://img.shields.io/badge/trackers-deepsort%20%7C%20deepsort__cascade%20%7C%20botsort-blue.svg)]()

`cvtrack` is the package-form tracking pipeline. v6 is the **true DeepSORT
release**: it ships a paper-faithful matching cascade (Mahalanobis gate +
appearance cascade + IoU fallback), Kalman n-step prediction with uncertainty
ellipses, an IDF1 metric, and a sensor abstraction layer that paves the way
for LiDAR / IMU / multi-camera rigs in the next iteration.

The headline numbers on `pexels_aerial_2034115.mp4` (200 frames, real YOLO inference
with `weights/visdrone_yolov8s.pt`, classes `0,1,2,3,4,5,7,8`, CPU device, drone preset
for the cascade row):

| tracker              | IDs  | ID/frame | mean length | median | longest | total obs | avg FPS |
|----------------------|-----:|---------:|------------:|-------:|--------:|----------:|--------:|
| DeepSortCascade      |   56 |    0.280 |       60.3 |     54 |     174 |     3,379 |     0.9 |
| DeepSortCascade raw  |  113 |    0.565 |       19.5 |      8 |     104 |     2,199 |     2.6 |
| BoT-SORT             |   82 |    0.410 |       13.2 |      4 |      83 |     1,083 |    12.3 |
| DeepSort (legacy)    |  110 |    0.550 |       20.7 |     10 |     159 |     2,277 |    11.8 |

DeepSortCascade with the drone preset (low min-conf, 8 VisDrone classes, IoU gate 0.20,
4-DOF Mahalanobis gate 9.4877) gives the longest tracks by a wide margin — the median
track length 54 vs BoT-SORT's 4. The cascade row uses OSNet 512-D embeddings via
torchreid (`weights/osnet_x0_25_msmt17.pth.tar`, `loaded_pretrained=True`); the raw
row runs cascade without preset tweaks so you can see how the ID-explosion warning
fires (ratio 0.565 > 0.5). The retained run outputs live in
`weights/run_deepsort_cascade_drone/`, `weights/run_deepsort_cascade/`,
`weights/run_botsort/`, and `weights/run_deepsort_legacy/`.

## What's new in v6

* **True DeepSORT cascade matcher** — `cvtrack.tracker.deepsort.DeepSortCascade`
  implements the matching cascade from the original DeepSORT paper: confirmed
  tracks are matched in age order (freshly-lost first), within each tier the
  gating is the chi-squared threshold on the Mahalanobis distance, and
  within the gate the cost is the cosine distance to the track's ReID
  gallery mean. Anything unmatched by the cascade falls through to an
  IoU fallback. New constructor parameters: `use_appearance=True`,
  `appearance_thresh=0.5`, `max_age=30`, `n_init=3`.

* **OSNet via torchreid** — `cvtrack.appearance.osnet.OsNetExtractor` is the
  only appearance backend in v6. It loads `osnet_x0_25` via torchreid (or
  its inline OSNet reference implementation) with the MSMT17
  fine-tuned checkpoint. If neither network stack can be imported the
  factory returns `None` and the pipeline degrades to motion-only tracking
  with a clear warning rather than a silent colour-histogram stub.

* **Kalman n-step prediction with covariance** —
  `cvtrack.tracker.kalman.predict_n_steps_with_covariance` returns the full
  state covariance at every projected step. The renderer now draws a
  3-sigma uncertainty ellipse at each future step (`cvtrack.viz.renderer
  .draw_predicted_future_trail`) and the future CSV gains `sigma_x` /
  `sigma_y` columns when `--write-future-csv` is on.

* **IDF1 metric** — `cvtrack.tracker.metrics.idf1` is a self-contained
  implementation of IDF1 / IDP / IDR with a greedy 1-to-1 best mapping.
  Unit-tested in `tests/test_metrics.py`. Used as the headline MOT-style
  metric once a real ground-truth source (MOT17-mini, custom labels, etc.)
  is plugged in.

* **Sensor abstraction layer** — `cvtrack.sensors.Sensor` is a tiny
  abstract base class (read / intrinsics / extrinsics) and
  `cvtrack.sensors.VideoSensor` is the default video backend. The pipeline
  still consumes a `VideoReader` directly in v6, but every downstream
  caller can now take a `Sensor` and be ready for LiDAR / IMU / multi-cam
  in v7.

* **Multi-sensor preset** — `configs/multi_sensor.yaml` is a placeholder
  that documents the upcoming `sensors:` list schema and is safe to pass
  via `--config multi_sensor` (it falls back to the same defaults as
  `default.yaml`).

* **New CLI flags**:
  - `--tracker {deepsort, deepsort_cascade, botsort}` (default unchanged)
  - `--predict-horizon <int>` (default 15) controls the KF future horizon
  - `--write-future-csv` switches on the sigma-annotated future CSV
  - `--reid` is now auto-enabled when `--tracker deepsort_cascade`

## Quick start

```bash
git clone <repo>
cd cv_tracking_demo
python3 -m pip install -e .

# v6 default: BoT-SORT with KF future projection + uncertainty ellipses
python -m cvtrack --tracker botsort \
    --source pexels_aerial_2034115.mp4 \
    --out-dir weights/run_botsort --max-frames 200 --save-trail

# v6 hero: true DeepSORT cascade matcher (auto-enables appearance)
python -m cvtrack --tracker deepsort_cascade \
    --source pexels_aerial_2034115.mp4 \
    --out-dir weights/run_deepsort_cascade --max-frames 200 \
    --write-future-csv --predict-horizon 15

# Legacy DeepSORT for backward-compat numbers
python -m cvtrack --tracker deepsort \
    --source pexels_aerial_2034115.mp4 \
    --out-dir weights/run_deepsort_legacy --max-frames 200

# Summarise all v6 runs into a JSON
python3 scripts/summarise_v6_runs.py > weights/v6_runs_summary.json
```

After setup, weights live in `weights/`:

```text
weights/
├── visdrone_yolov8s.pt          21.5 MB, drone-finetuned detector
├── osnet_x0_25_msmt17.pth.tar    8.9 MB, tiny ReID head
└── COMPARE_REPORT.md             generated comparison report
```

## Architecture

```
cv_tracking_demo/
├── pyproject.toml            # build + tool config
├── requirements.txt
├── Makefile                  # install / lint / test / docker-build
├── Dockerfile
├── .github/workflows/ci.yml
├── configs/                  # YAML presets (default, drone, street, multi_sensor)
├── src/cvtrack/              # the package
│   ├── pipeline.py           # main CLI
│   ├── config.py             # YAML loader, validator
│   ├── types.py              # Box, Detection, Track
│   ├── detector/             # YOLO, MOG2, factory
│   ├── tracker/
│   │   ├── botsort.py        # 8-state KF + CMC + ReID fusion
│   │   ├── deepsort.py       # DeepSortLite (legacy) + DeepSortCascade (v6)
│   │   ├── kalman.py         # KalmanCV2D + KalmanBoT + n-step projection
│   │   ├── metrics.py        # IoU, gating, IDF1 (v6)
│   │   ├── smoother.py       # RTS
│   │   └── cmc.py            # sparse-OF + ECC
│   ├── appearance/
│   │   ├── osnet.py          # OSNet (torchreid, v6 only appearance backend)
│   │   ├── gallery.py        # per-track FIFO + EMA
│   │   └── factory.py        # backend dispatcher
│   ├── sensors/              # v6 abstraction layer (skeleton)
│   │   ├── base.py           # Sensor, Frame, Intrinsics, Extrinsics
│   │   └── video.py          # VideoSensor (wraps VideoReader)
│   ├── viz/                  # box/trail/uncertainty renderer
│   ├── io.py                 # video + CSV writers + future-CSV with sigma
│   └── postprocess.py        # CLI: tracks.csv -> plots
├── tests/                    # pytest, CPU-only
├── eval/mot17_mini/          # MOT17-mini + synthetic eval
├── scripts/
│   ├── setup_visdrone_compare.sh    # install + download weights
│   ├── visdrone_compare.py          # 4-way head-to-head runner
│   ├── collect_stats.py             # multi-video summary
│   ├── summarise_v6_runs.py         # v6 headline metrics
│   ├── download_weights.py          # standalone weight downloader
│   └── run_cvtrack.sh
```

The sensor module is a **skeleton** in v6: it exists so that the next
iteration of the pipeline can compose multiple `Sensor` instances
(LiDAR + IMU + multi-camera) without changing the tracker contract.
Until that lands, the main pipeline still consumes a `VideoReader`
directly.

## Configs

| Config          | ImgSz | Conf | HighConf | NewConf | IoU  | Relink | ReID    | Notes                  |
|-----------------|------:|-----:|---------:|--------:|-----:|-------:|---------|------------------------|
| `default`       |   320 | 0.15 |     0.35 |    0.20 | 0.30 |     30 | off     | web / generic          |
| `drone`         |   480 | 0.12 |     0.22 |    0.07 | 0.20 |     45 | opt-in  | small moving boxes     |
| `street`        |   480 | 0.25 |     0.50 |    0.25 | 0.35 |     20 | off     | 1080p street footage   |
| `multi_sensor`  |   320 | 0.15 |     0.35 |    0.20 | 0.30 |     30 | off     | multi-sensor placeholder (v7+) |

YAML overrides CLI when both are passed: CLI > YAML > preset defaults.

`--drone` is sugar for `--config configs/drone.yaml`. Note that `drone.yaml`
historically sets `detector.backend: mog2` to preserve the v6 compatibility
baseline — to use YOLO on top of the drone tracking thresholds, pass
`--detector yolo` explicitly (the comparison runner does this).

## Head-to-head runner

`scripts/visdrone_compare.py` remains available for the detector/ReID matrix.
Its output directory is disposable and uses semantic subdirectories:

| Output directory | Configuration |
|---|---|
| `run_mog2` | MOG2 compatibility baseline |
| `run_coco` | COCO YOLOv8s, no ReID |
| `run_visdrone` | VisDrone YOLOv8s, no ReID |
| `run_visdrone_reid` | VisDrone YOLOv8s + OSNet ReID |

The maintained v6 tracker comparison is separate and lives in
`weights/run_deepsort_legacy/`, `weights/run_deepsort_cascade/`, and
`weights/run_botsort/`. Each contains the rendered video, standard track
CSV, 15-step future CSV with `sigma_x`/`sigma_y` columns, smoothed CSV,
and trail JSON.

Output: `weights/COMPARE_REPORT.md` (markdown table) plus
`weights/summary.json` (machine-readable). The script is fail-soft: if a
weight file is missing or the optional ReID import fails, it logs the
failure and continues with the valid configurations.

## 8-video detector benchmark

To see how the COCO vs VisDrone choice plays out across the rest of the
sample clips (200 frames each, drone tracking preset):

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

Reproduce with:

```bash
python3 scripts/collect_stats.py \
    --sources *.mp4 \
    --out-dir weights/all_runs \
    --max-frames 200 \
    --config drone
```

…then for the per-detector numbers, swap the YOLO weights with the
`--weights` flag and pass `--detector yolo`.

## How to evaluate

### Self-test (no data needed)
```bash
python eval/mot17_mini/run_eval.py --synthetic
```

This synthesises a 100-frame clip with 3 known moving rectangles, runs the
tracker, and reports MOT-style metrics (MOTA / IDF1 / MOTP). The
`cvtrack.tracker.metrics.idf1` function is also directly callable from
notebooks for custom MOT-style analysis (no external dependencies).

### IDF1 against your own ground truth
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

### Real MOT17 mini
```bash
# 1. Download 2 sequences (e.g. MOT17-04-FRCNN, MOT17-13-FRCNN)
# 2. Arrange:
#    eval/mot17_mini/data/MOT17/train/MOT17-04-FRCNN/img1/...
#    eval/mot17_mini/data/MOT17/train/MOT17-04-FRCNN/gt/gt.txt
# 3. Run:
python eval/mot17_mini/run_eval.py \
    --data-root eval/mot17_mini/data/MOT17/train \
    --sequences MOT17-04-FRCNN MOT17-13-FRCNN \
    --tracker botsort
```

## How to add a custom component

* **Detector** — subclass `cvtrack.detector.base.DetectorProtocol` (or
  inherit `cvtrack.detector.base.Detector`) and add a branch to
  `cvtrack.detector.factory.build_detector`.
* **Tracker** — implement `step(dets, frame) -> tracks` and pass it to the
  pipeline via `--tracker <name>`. The new `DeepSortCascade` is the
  reference for appearance-aware matchers; subclass it for custom cascade
  policies.
* **Appearance model** — inherit `cvtrack.appearance.osnet.OsNetExtractor` and
  add a branch to `cvtrack.appearance.factory.make_extractor`.  All appearance
  models must be real pretrained networks (no histogram/synthetic fallbacks).
* **Sensor** — subclass `cvtrack.sensors.Sensor` and pass it to the
  pipeline. The contract is intentionally tiny so LiDAR / IMU / multi-cam
  adapters fit in <100 lines.

## Engineering hygiene

* `requirements.txt`: pinned to compatible ranges (>= / < where needed).
* `pyproject.toml` defines `[project]`, plus `[tool.ruff]`, `[tool.mypy]`,
  `[tool.pytest.ini_options]`.
* `Dockerfile` based on `python:3.11-slim`, copies source as editable
  install, defaults to `--help`.
* `Makefile` exposes `install`, `lint`, `typecheck`, `test`, `test-slow`,
  `run-drone`, `docker-build`, `clean`.
* CI: ruff check, mypy (best-effort), fast pytest across Python 3.10/3.11.
* Test suite is 53 passed + 1 skipped (torchreid-only) on Python 3.10.

## Future work (next iteration)

* **LiDAR + IMU + multi-camera** — the `cvtrack.sensors` module is ready;
  the next step is to wire a `MultiSensorSource` that synchronises multiple
  `Sensor` instances by timestamp and feeds a single tracker.
* **MOT17 full eval** — `eval/mot17_mini/run_eval.py` already supports
  arbitrary sequences; adding the full MOT17 / KITTI download scripts is
  the next deliverable.
* **VisDrone fine-tune of larger backbones** (yolov8m / yolov8l).
  Multiple hours of GPU training; current pipeline uses yolov8s
  fine-tuned by dronefreak.
* **ONNX runtime fallback** for `torchreid` when Cython is unavailable.
* **TensorRT / DNN-based MOG2** when running on GPU.

## Troubleshooting

* **`No module named 'torchreid'`** during ReID run.  `torchreid` 1.0.6
  ships Cython extensions that don't build against modern setuptools.
  Two workarounds:
  1. `pip install --user "Cython<3"` then re-run `pip install -e .[reid]`,
     or
  2. Skip ReID — the cascade matcher gracefully degrades to motion-only
     tracking and the head-to-head still runs all three tracker
     configurations.

* **`numpy.core.multiarray failed to import`** when loading ultralytics.
  This box has `numpy 2.2.6` in `~/.local` but the system
  `matplotlib 3.5.1` was compiled against `numpy 1.x`. The setup script
  works around this by writing a tiny `sitecustomize.py` under
  `/tmp/cvfix/` that side-loads system numpy 1.21.5 + scipy 1.8.0 before
  anything else. The activation is automatic when you source
  `scripts/setup_visdrone_compare.sh`. To run cvtrack manually with the
  fix: `PYTHONPATH=/tmp/cvfix:src python3 -m cvtrack …`.

* **`matplotlib cache permission denied`** — harmless warning when
  matplotlib tries to write its font cache to a read-only location.

* **`yolov8s.pt` not found at the bare name.**  Pass an absolute
  `--weights /path/to/yolov8s.pt`. The visdrone runner always passes an
  absolute path so this is only a concern for hand-rolled CLI usage.

## License

Apache-2.0.  See `LICENSE`.