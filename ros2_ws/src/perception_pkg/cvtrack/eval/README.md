# Evaluation directory

This directory contains evaluation harnesses for `cvtrack`.  Currently
includes:

* `mot17_mini/` — MOT17-mini (2-sequence) evaluation using `motmetrics`.
  Run with the dataset, or with `--synthetic` to validate the harness without
  external data.

To add a new benchmark:

1. Create `eval/<benchmark>/` with a `run_eval.py` that:
   - Spawns cvtrack via `subprocess.run([sys.executable, "-m", "cvtrack", ...])`
   - Loads the resulting `tracks.csv`
   - Computes the metric (IDF1, MOTA, MOOS, IoU, …)
   - Prints a summary table
2. Document usage in `eval/<benchmark>/README.md`.
3. Reference it from the top-level `README.md`.

The harness is intentionally decoupled from the package — `cvtrack` is
treated as a black-box CLI, so behaviour measured here reflects exactly
what an external user sees when invoking `python -m cvtrack`.
