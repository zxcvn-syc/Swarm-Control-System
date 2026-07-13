#!/usr/bin/env python3
"""Download VisDrone-YOLOv8s and OSNet-MSMT17 weights into ./weights.

Tries mirrors in order; first one that works wins.  Idempotent.
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path
from urllib.request import Request, urlopen

WEIGHTS = Path(__file__).resolve().parents[1] / "weights"
WEIGHTS.mkdir(exist_ok=True)


# (filename, [mirror URLs tried in order])
VISDRONE_URLS = [
    "https://hf-mirror.com/dronefreak/visdrone-yolov8s/resolve/main/best.pt",
    "https://huggingface.co/dronefreak/visdrone-yolov8s/resolve/main/best.pt",
    "https://github.com/RuiYang-1010/dronefreak-visdrone-yolov8s/releases/download/v1/best.pt",
]
OSNET_URLS = [
    "https://hf-mirror.com/kaiyangzhou/osnet/resolve/main/osnet_x0_25_msmt17_combineall_256x128_amsgrad_ep150_stp60_lr0.0015_b64_fb10_softmax_labelsmooth_flip_jitter.pth",
    "https://huggingface.co/kaiyangzhou/osnet/resolve/main/osnet_x0_25_msmt17_combineall_256x128_amsgrad_ep150_stp60_lr0.0015_b64_fb10_softmax_labelsmooth_flip_jitter.pth",
]


def _http_download(url: str, dest: Path) -> None:
    """Stream `url` to `dest`.  Raises on HTTP error."""
    req = Request(url, headers={"User-Agent": "curl/8.0"})
    with urlopen(req, timeout=120) as r, open(dest, "wb") as f:
        shutil.copyfileobj(r, f, length=64 * 1024)


def download_with_fallback(target: Path, urls: list[str], label: str) -> Path | None:
    if target.exists() and target.stat().st_size > 100_000:
        print(f"  {label} weights already present: {target}")
        return target
    for url in urls:
        try:
            print(f"  try: {url}")
            _http_download(url, target)
            size = target.stat().st_size
            print(f"  OK  {label}: {size/1024/1024:.1f} MB -> {target}")
            return target
        except Exception as e:
            print(f"  fail: {type(e).__name__}: {e}")
            if target.exists():
                target.unlink()
    print(f"  ERROR: all mirrors failed for {label}")
    return None


def main() -> int:
    print("VisDrone...")
    v = download_with_fallback(WEIGHTS / "visdrone_yolov8s.pt", VISDRONE_URLS, "visdrone")
    print("\nOSNet...")
    o = download_with_fallback(
        WEIGHTS / "osnet_x0_25_msmt17.pth.tar", OSNET_URLS, "osnet"
    )
    if not v or not o:
        return 1
    print("\nAll weights ready in", WEIGHTS)
    return 0


if __name__ == "__main__":
    sys.exit(main())