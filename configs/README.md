# Configs

YAML configuration presets for `cvtrack`.  Use with `--config <name>`:

```bash
python -m cvtrack --config drone --source clip.mp4 --out-dir out/
```

## Available presets

### `default.yaml`
Generic web/pedestrian scene.  COCO weights, `osnet_x1_0` ReID (off by
default), 320-px inference.

### `drone.yaml`
Aerial / drone footage.  Higher `imgsz`, lower confidence floor, smaller
box-area threshold, longer re-link window.  **NOTE**: this preset uses
the MOG2 background-subtraction detector to mirror the behaviour of v4
`cv_track_demo.py` (whose bare-weight-name resolution quirk silently
selected MOG2).  To switch to YOLOv8s, pass an absolute `--weights` or
override `detector.backend: yolo`.

### `street.yaml`
1080p street scene.  Higher confidence floor, larger box-area threshold,
faster turnover of tentative tracks.

## Schema

```yaml
detector:
  weights: <str>          # COCO or custom weights
  imgsz: <int>            # inference image side
  conf: <float>           # detection confidence threshold
  classes: [int, ...]     # COCO class IDs to keep
  min_box_area: <float>   # drop boxes below this area (px²)
  min_conf: <float>       # allow even lower-conf detections to enter 2nd-stage match

tracker:
  kind: botsort | deepsort
  cmc: <bool>
  cmc_method: sparse_of | ecc
  high_conf: <float>
  new_track_conf: <float>
  iou_thresh: <float>
  lost_relink_frames: <int>
  max_age: <int>          # frames before a lost track is removed
  n_init: <int>           # hits before a track is confirmed
  stationary_prune: <bool>

appearance:
  enabled: <bool>
  backend: osnet
  model: osnet_x0_25 | osnet_x1_0
  weights: <path>
  gallery_size: <int>
  ema_alpha: <float>
  match_weight: <float>   # weight of ReID cost in 2nd-stage fusion
  min_box_side: <int>     # below this side length, no embedding is computed

viz:
  save_trail: <bool>
  fps_overlay: <bool>

output:
  write_video: <bool>
  write_csv: <bool>
  write_smoothed_csv: <bool>
  write_reid_json: <bool>
  write_trails_json: <bool>

pipeline:
  max_frames: <int>       # 0 = full video
  max_seconds: <float>    # 0 = full video
  start_frame: <int>
  include_tentative: <bool>
  id_explosion_warn: <float>   # warn if ids/frame > this ratio
```

CLI flags take precedence over YAML keys.  The same key in `default.yaml`
can be overridden in a derived config by setting `extends: default` and
overriding just the desired fields.
