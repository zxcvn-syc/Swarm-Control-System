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
# Allow the caller to point at a non-standard user-site install dir via env,
# otherwise we don't need any PYTHONPATH fiddling for the standard
# `pip install -e .` workflow.  See .local_lib/ in .gitignore.
if [ -n "${CVTRACK_LOCAL_LIB:-}" ]; then
    export PYTHONPATH="${CVTRACK_LOCAL_LIB}:${HERE}/src:${PYTHONPATH:-}"
else
    export PYTHONPATH="${HERE}/src:${PYTHONPATH:-}"
fi

exec python3 -m cvtrack "$@"
