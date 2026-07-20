# cvtrack Dockerfile
#
# Base image: python:3.11-slim (small, well-tested, CPU-friendly).  Build:
#
#     docker build -t cvtrack:latest .
#     docker run --rm -v $(pwd)/data:/data cvtrack:latest \
#         --config drone --source /data/clip.mp4 --out-dir /data/out
#
# Switch to `python:3.11-slim-cuda` (or an nvidia base) for GPU.  Then add
# `--index-url https://download.pytorch.org/whl/cu121` to the torch pip
# install line and remove `-e .[cpu]`.

FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# System libraries OpenCV needs at runtime (libgl1 for cv2.imshow headless,
# libglib for some contrib modules, ffmpeg for video decoding on Linux).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 ffmpeg \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install Python deps first so layer caching is effective on source-only edits.
COPY requirements.txt ./
RUN pip install --upgrade pip setuptools wheel \
 && pip install -r requirements.txt

# Copy the source.
COPY pyproject.toml ./
COPY src ./src
COPY configs ./configs
COPY README.md ./

# Editable install:  this finalises the import path.
RUN pip install -e .

# Default command: print help so a bare `docker run cvtrack:latest` is informative.
# Replace with `--config drone --source /data/clip.mp4 --out-dir /data/out` for
# your workload.
CMD ["python", "-m", "cvtrack", "--help"]
