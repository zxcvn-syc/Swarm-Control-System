#!/usr/bin/env bash

# Record a real PX4/Gazebo Classic GUI session from a headless VM through Xvfb.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DISPLAY_ID="${DISPLAY_ID:-:99}"
DURATION="${DURATION:-90}"
OUTFILE="${OUTFILE:-$REPO_ROOT/output/video_work/gazebo_gui_$(date +%Y%m%d_%H%M%S).mp4}"
PX4_SITL_ROOT="${PX4_SITL_ROOT:-$HOME/src/PX4-Autopilot}"
WORLD="${PX4_WORLD:-$REPO_ROOT/simulation/worlds/swarm_field.world}"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Options:
  --out FILE         H.264 MP4 output path.
  --duration SEC     Recording duration, default: 90.
  --display DISPLAY  Xvfb display, default: :99.
  --help             Show this help.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUTFILE="${2:?missing output path}"; shift 2 ;;
    --duration) DURATION="${2:?missing duration}"; shift 2 ;;
    --display) DISPLAY_ID="${2:?missing display}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[gazebo-record] unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$DURATION" =~ ^[1-9][0-9]*$ ]] || { echo "[gazebo-record] duration must be positive" >&2; exit 2; }
for program in Xvfb ffmpeg gzserver gzclient gz; do
  command -v "$program" >/dev/null || { echo "[gazebo-record] missing command: $program" >&2; exit 2; }
done

BUILD_DIR="$PX4_SITL_ROOT/build/px4_sitl_default"
SITL_BIN="$BUILD_DIR/bin/px4"
[[ -x "$SITL_BIN" ]] || SITL_BIN="$BUILD_DIR/px4_sitl_default"
SETUP_GAZEBO="$PX4_SITL_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash"
MODEL_SDF="$PX4_SITL_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf"
ROOTFS="$BUILD_DIR/rootfs"
[[ -x "$SITL_BIN" && -f "$SETUP_GAZEBO" && -f "$MODEL_SDF" && -f "$WORLD" ]] || {
  echo "[gazebo-record] PX4 SITL or world files are missing" >&2
  exit 2
}

mkdir -p "$(dirname "$OUTFILE")"

# setup_gazebo.bash reads unset variables, so initialize them before sourcing.
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
source "$SETUP_GAZEBO" "$PX4_SITL_ROOT" "$BUILD_DIR" >/dev/null
export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-/usr/share/gazebo-11}"
export PX4_SIM_MODEL="gazebo-classic_iris"
export PX4_SIM_WORLD="$WORLD"
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

XVFB_PID=""
GZSERVER_PID=""
GZSPAWN_PID=""
GZCLIENT_PID=""
PX4_PID=""
FFMPEG_PID=""

cleanup() {
  for pid in "$FFMPEG_PID" "$GZCLIENT_PID" "$PX4_PID" "$GZSPAWN_PID" "$GZSERVER_PID" "$XVFB_PID"; do
    [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "$FFMPEG_PID" "$GZCLIENT_PID" "$PX4_PID" "$GZSPAWN_PID" "$GZSERVER_PID" "$XVFB_PID"; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

Xvfb "$DISPLAY_ID" -screen 0 1280x720x24 -ac +extension GLX +render -noreset >/tmp/cvtrack_xvfb.log 2>&1 &
XVFB_PID=$!
sleep 2

gzserver "$WORLD" >/tmp/cvtrack_gzserver.log 2>&1 &
GZSERVER_PID=$!

# Gazebo 11 keeps the model CLI attached after it has inserted the model.
# Start it asynchronously and let the GUI connect while it maintains state.
gz model --spawn-file="$MODEL_SDF" --model-name=iris -x 1.01 -y 0.98 -z 0.83 \
  >/tmp/cvtrack_gzspawn.log 2>&1 &
GZSPAWN_PID=$!
sleep 5

(cd "$ROOTFS" && exec "$SITL_BIN" -d "$BUILD_DIR/etc") >/tmp/cvtrack_px4.log 2>&1 &
PX4_PID=$!

DISPLAY="$DISPLAY_ID" gzclient --verbose >/tmp/cvtrack_gzclient.log 2>&1 &
GZCLIENT_PID=$!
sleep 8

DISPLAY="$DISPLAY_ID" ffmpeg -hide_banner -loglevel warning -y \
  -video_size 1280x720 -framerate 15 -f x11grab -draw_mouse 0 -i "${DISPLAY_ID}.0" \
  -t "$DURATION" -c:v libx264 -preset ultrafast -crf 22 -pix_fmt yuv420p "$OUTFILE" &
FFMPEG_PID=$!
wait "$FFMPEG_PID"
FFMPEG_PID=""

ffprobe -v error -show_entries format=duration,size -of default=noprint_wrappers=1 "$OUTFILE"
echo "[gazebo-record] wrote $OUTFILE"
