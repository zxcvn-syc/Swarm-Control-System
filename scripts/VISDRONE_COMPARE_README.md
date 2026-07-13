# VisDrone + ReID head-to-head comparison

This is a one-shot script that benchmarks four configurations of cvtrack on the
same drone video and reports how each affects track quality:

1. **MOG2** (legacy v4 fallback path) — baseline reference.
2. **COCO yolov8s** — generic pretraining, no ReID.
3. **VisDrone yolov8s** — fine-tuned on aerial imagery, no ReID.
4. **VisDrone yolov8s + ReID (OSNet MSMT17)** — full pipeline.

## Run it (3 commands)

```bash
cd /path/to/cv_tracking_demo

# Install ReID extra + download VisDrone & OSNet weights (~150 MB total)
bash scripts/setup_visdrone_compare.sh

# Run the comparison on the bundled drone clip
python3 scripts/visdrone_compare.py \
    --source pexels_aerial_2034115.mp4 \
    --frames 200 \
    --out-dir weights
# Read the report
cat weights/COMPARE_REPORT.md
```

> If `python` is missing on your system (you get "Command 'python' not found"),
> that's fine — every script in this repo uses `python3` explicitly.

## What to look at

- **IDs**: total confirmed track IDs.  Lower = better association, more stable tracks.
- **mean_len**: mean number of frames each ID survives.  Higher = better re-link after occlusion.
- **median dets/frame**: how selective the detector is.  Too high ⇒ noisy boxes; too low ⇒ missed objects.

For a 200-frame aerial clip the typical progression is:

| config | IDs | mean_len | reason |
|---|---:|---:|---|
| MOG2 baseline | ~180 | ~22 | every blob → new ID; no appearance model |
| COCO yolov8s | ~180 | ~22 | too weak on small vehicles at 480 px |
| VisDrone yolov8s | ~30–60 | ~80 | much better recall on small aerial objects |
| + ReID | ~25–50 | ~120 | second-stage appearance match re-links after gaps |

## Troubleshooting

- **VisDrone download fails**: your machine can't reach `huggingface.co`.  Manually
  download `best.pt` from https://huggingface.co/dronefreak/visdrone-yolov8s and
  place it at `weights/visdrone_yolov8s.pt`.
- **OSNet download fails**: get the file from
  https://kaiyangzhou.github.io/deep-person-reid/MODEL_ZOO.html and place it at
  `weights/osnet_x0_25_msmt17.pth.tar`.
- **torchreid install fails**: try `pip install torchreid` directly (the `reid`
  extra pulls from a git tag which may break).