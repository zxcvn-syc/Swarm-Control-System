#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_DIR="$REPO_ROOT/video-remotion"
OUTPUT_FILE="${1:-$REPO_ROOT/videos/closed_loop_evidence_demo_20260820.mp4}"

mkdir -p "$(dirname "$OUTPUT_FILE")"
cd "$PROJECT_DIR"
pnpm exec remotion render src/index.ts ClosedLoopEvidenceDemo "$OUTPUT_FILE" \
  --codec=h264 --crf=19 --overwrite
ffprobe -v error -show_entries format=duration,size -of default=noprint_wrappers=1 "$OUTPUT_FILE"
