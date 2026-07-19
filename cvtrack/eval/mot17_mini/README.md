# MOT17-mini evaluation harness

`cvtrack` ships with a small MOT17-mini benchmark you can run **without
downloading the full MOT17 dataset**.  This directory contains:

* `run_eval.py` — CLI that runs `cvtrack` against a MOT17 sequence, converts
  the output to `motmetrics` format, and prints IDF1 / MOTA / MOTP.
* `synthetic.py` — a no-data-required self-test that synthesises 100 frames
  with 3 known moving objects and verifies the matcher wiring end-to-end.
* `README.md` — this file.

## Quick start (with the dataset)

```bash
# 1. Download two MOT17 sequences, e.g. MOT17-04-FRCNN and MOT17-13-FRCNN.
#    Unzip them so the structure looks like:
#       eval/mot17_mini/data/MOT17/train/MOT17-04-FRCNN/{img1, gt, seqinfo.ini}
#       eval/mot17_mini/data/MOT17/train/MOT17-13-FRCNN/{img1, gt, seqinfo.ini}

# 2. Run cvtrack in MOT format.  Output goes into each sequence folder under
#    `cvtrack_output/<sequence>/tracks.csv` which `run_eval.py` then converts.
python eval/mot17_mini/run_eval.py --data-root eval/mot17_mini/data/MOT17/train \
    --sequences MOT17-04-FRCNN MOT17-13-FRCNN --tracker botsort

# 3. Read the printed IDF1 / MOTA / MOTP table.
```

## Quick start (no dataset — synthetic)

```bash
python eval/mot17_mini/run_eval.py --synthetic
# Expected: a table with three rows (one per synthetic identity) and a final
# summary line listing the per-class match rate.
```

## How MOT17 ground truth maps to `cvtrack`

MOT17 `.gt` files are 1-indexed for frame (row 1) and 1-indexed for track id
(column 2).  `run_eval.py` shifts them to 0-indexed for `motmetrics` and
filters out entries with `class_id != 1` (pedestrian) or `considered = 0`.

`cvtrack`'s `tracks.csv` is already in this format — same columns:

| column | MOT17                 | cvtrack      |
|--------|-----------------------|--------------|
| 1      | frame (1-indexed)     | 0-indexed    |
| 2      | track id              | track id     |
| 7      | class ("pedestrian")  | cls id (0)   |

`run_eval.py` bumps the cvtrack frame index by 1 before feeding it to
`motmetrics`.
