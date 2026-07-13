#!/usr/bin/env bash
# Convenience: install the ReID extra and download VisDrone + OSNet weights
# into ./weights.  Safe to re-run; existing files are skipped.
#
# Usage:
#     bash scripts/setup_visdrone_compare.sh
set -eu

cd "$(dirname "$0")/.."
mkdir -p weights

# Confirm python3 exists
if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 not found.  Install with: sudo apt install python3 python3-pip"
    exit 1
fi

# 1) Install extras.  We install huggingface_hub unconditionally because the
#    weight downloader uses it; the torchreid git+https install is attempted
#    after so a torchreid failure doesn't block the headline comparison.
echo "[1/2] python3 -m pip install huggingface_hub (always)"
python3 -m pip install --user huggingface_hub

echo "[1/2] python3 -m pip install -e .[reid]   (best-effort, may take a few minutes)"
python3 -m pip install -e ".[reid]" || true

# Workaround for the numpy 2.x / scipy 1.15 ABI mismatch on this box.
# The user has numpy>=2 and scipy>=1.15 in ~/.local which require Python 3.10+
# typing/numpy 2.x features.  Loading scipy 1.15 against numpy 1.21.5 crashes
# with "'numpy._DTypeMeta' object is not subscriptable".  We side-load the
# system numpy 1.21.5 + scipy 1.8.0 + matplotlib 3.5.1 from /usr/lib/python3/
# dist-packages.  ultralytics/torch/opencv still load normally from user-site.
mkdir -p /tmp/cvfix
cat > /tmp/cvfix/sitecustomize.py <<'PYEOF'
"""Force-load numpy 1.21.5 + scipy 1.8.0 from system dist-packages.

The user has numpy>=2 and scipy>=1.15 in ~/.local which require numpy 2.x
typing APIs.  Without this hook, importing scipy 1.15 under numpy 1.x raises
"'numpy._DTypeMeta' object is not subscriptable".  matplotlib 3.5.1 in turn is
compiled against numpy 1.x and crashes under numpy 2.x with "numpy.core.
multiarray failed to import".  Loading the system trio (numpy 1.21.5, scipy
1.8.0, matplotlib 3.5.1) before anything else keeps everything compatible.
"""
import sys, os, importlib.util

def _force_load(name, root):
    if not os.path.isdir(root):
        return
    if name in sys.modules:
        return
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(root, "__init__.py"),
        submodule_search_locations=[root])
    if spec and spec.loader:
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        try:
            spec.loader.exec_module(mod)
        except Exception:
            sys.modules.pop(name, None)

_force_load("numpy", "/usr/lib/python3/dist-packages/numpy")
_force_load("numpy.core", "/usr/lib/python3/dist-packages/numpy/core")
_force_load("numpy.lib", "/usr/lib/python3/dist-packages/numpy/lib")
_force_load("numpy.linalg", "/usr/lib/python3/dist-packages/numpy/linalg")
_force_load("numpy.random", "/usr/lib/python3/dist-packages/numpy/random")
_force_load("numpy.fft", "/usr/lib/python3/dist-packages/numpy/fft")
_force_load("scipy", "/usr/lib/python3/dist-packages/scipy")
PYEOF
export PYTHONPATH="/tmp/cvfix${PYTHONPATH:+:$PYTHONPATH}"
echo "  patched PYTHONPATH for numpy/scipy/matplotlib ABI compatibility -> /tmp/cvfix"

# 2) Download weights
echo
echo "[2/2] python3 scripts/download_weights.py"
python3 scripts/download_weights.py