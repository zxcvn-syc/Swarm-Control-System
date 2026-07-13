#!/usr/bin/env bash
# Convenience wrapper to run cvtrack with the correct PYTHONPATH.
#
# Use this when the package is installed in a non-standard target directory
# (e.g. a user --target install).  For normal `pip install -e .` usage you
# can just run `python -m cvtrack ...`.
#
# Usage:
#   scripts/run_cvtrack.sh --config drone --source clip.mp4 --out-dir /tmp/x

set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
LOCAL_LIB="${CVTRACK_LOCAL_LIB:-/home/hhh/Downloads/.local_lib}"

export PYTHONPATH="${LOCAL_LIB}:${HERE}/src:${PYTHONPATH:-}"

exec python3 -m cvtrack "$@"
