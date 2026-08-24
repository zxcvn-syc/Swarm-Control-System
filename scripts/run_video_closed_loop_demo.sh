#!/usr/bin/env bash

# Run the actual video tracker through world projection, assignment, obstacle
# planning, and containment. This is a replay validation: no PX4, MAVROS, or
# vehicle-control process is launched.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"
INSTALL_BASE="${CVTRACK_INSTALL_BASE:-$WORKSPACE/install}"
OUTPUT_DIR="${OUTPUT_DIR:-$REPO_ROOT/output/video_closed_loop_demo}"
VIDEO="${VIDEO:-$HOME/Downloads/cv_tracking_demo/coverr_road_traffic.mp4}"
WEIGHTS="${WEIGHTS:-$WORKSPACE/src/perception_pkg/best.pt}"
DETECTOR_BACKEND="${DETECTOR_BACKEND:-mog2}"
DURATION=18
DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
TRACKER_RATE=10.0

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Options:
  --install-base DIR  ROS 2 install overlay to use.
  --output-dir DIR    Trace and node logs directory.
  --video FILE        Source video for tracker_node inference.
  --detector-backend  yolo | mog2. Default: mog2 for real-time CPU replay.
  --weights FILE      YOLO weights (used only with --detector-backend yolo).
  --duration SEC      Recorder duration, default: 18.
  --domain-id ID      Isolated ROS domain, default: 89.
  --help              Show this help.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-base) INSTALL_BASE="${2:?missing install base}"; shift 2 ;;
    --output-dir) OUTPUT_DIR="${2:?missing output directory}"; shift 2 ;;
    --video) VIDEO="${2:?missing video}"; shift 2 ;;
    --detector-backend) DETECTOR_BACKEND="${2:?missing detector backend}"; shift 2 ;;
    --weights) WEIGHTS="${2:?missing weights}"; shift 2 ;;
    --duration) DURATION="${2:?missing duration}"; shift 2 ;;
    --domain-id) DOMAIN_ID="${2:?missing domain id}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[video-closed-loop] unknown option: $1" >&2; exit 2 ;;
  esac
done

[[ "$DURATION" =~ ^[1-9][0-9]*([.][0-9]+)?$ ]] || { echo "[video-closed-loop] duration must be positive" >&2; exit 2; }
[[ -f "$VIDEO" ]] || { echo "[video-closed-loop] video not found: $VIDEO" >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "[video-closed-loop] ROS 2 Humble is required" >&2; exit 2; }
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "[video-closed-loop] missing install overlay: $INSTALL_BASE" >&2; exit 2; }
command -v ffprobe >/dev/null || { echo "[video-closed-loop] ffprobe is required" >&2; exit 2; }
[[ "$DETECTOR_BACKEND" == "mog2" || "$DETECTOR_BACKEND" == "yolo" ]] || {
  echo "[video-closed-loop] detector backend must be yolo or mog2" >&2
  exit 2
}
if [[ "$DETECTOR_BACKEND" == "yolo" && ! -f "$WEIGHTS" ]]; then
  echo "[video-closed-loop] YOLO weights not found: $WEIGHTS" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$INSTALL_BASE/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER

VIDEO_META="$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height,r_frame_rate -of csv=p=0 "$VIDEO")"
IFS=',' read -r VIDEO_WIDTH VIDEO_HEIGHT VIDEO_RATE <<< "$VIDEO_META"
VIDEO_FPS="$(python3 - "$VIDEO_RATE" <<'PY'
from fractions import Fraction
import sys
print(float(Fraction(sys.argv[1])))
PY
)"
FOCAL="$(python3 - "$VIDEO_WIDTH" <<'PY'
import sys
print(float(sys.argv[1]) / 2.0)
PY
)"
CX="$(python3 - "$VIDEO_WIDTH" <<'PY'
import sys
print(float(sys.argv[1]) / 2.0)
PY
)"
CY="$(python3 - "$VIDEO_HEIGHT" <<'PY'
import sys
print(float(sys.argv[1]) / 2.0)
PY
)"

mkdir -p "$OUTPUT_DIR"
PIDS=()

cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]:-}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]:-}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]:-}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

launch_node() {
  local logfile="$1"
  shift
  setsid env PYTHONUNBUFFERED=1 "$@" >"$logfile" 2>&1 & PIDS+=("$!")
}

launch_node "$OUTPUT_DIR/replay_fixture.log" ros2 run perception_pkg video_replay_fixture --ros-args \
  -p publish_rate_hz:="$TRACKER_RATE" \
  -p image_width:="$VIDEO_WIDTH" -p image_height:="$VIDEO_HEIGHT" \
  -p fx:="$FOCAL" -p fy:="$FOCAL" -p cx:="$CX" -p cy:="$CY" \
  -p camera_x:=35.0 -p camera_y:=20.0 -p camera_altitude:=10.0
launch_node "$OUTPUT_DIR/grid_map.log" ros2 run planning_pkg grid_map_node
launch_node "$OUTPUT_DIR/coord_transform.log" ros2 run perception_pkg coord_transform_node --ros-args \
  -p max_pose_age_s:=1.0 -p publish_debug:=false
launch_node "$OUTPUT_DIR/planner.log" ros2 run planning_pkg planner_node --ros-args \
  -p num_drones:=4 -p grid_size:=40 -p sim_tick_speed:=0.3 \
  -p initial_positions:="[3.0, 20.0, 3.0, 5.0, 39.0, 39.0, 39.0, 0.0]" \
  -p tick_period:=0.2 -p log_interval_sec:=2.0
launch_node "$OUTPUT_DIR/scheduler.log" ros2 run scheduler_pkg scheduler_node --ros-args \
  -p num_drones:=4 -p max_per_drone:=1 -p tick_period:=0.2 \
  -p log_interval_sec:=2.0 -p target_topic:=/target_track_world
launch_node "$OUTPUT_DIR/enclosure.log" ros2 run containment_pkg enclosure_node --ros-args \
  -p update_period:=0.2 -p target_track_topic:=/target_track_world \
  -p enclosure_target_topic:=/video_replay_unused_enclosure_targets

TRACKER_ARGS=(
  ros2 run perception_pkg tracker_node --ros-args
  -p input_mode:=video -p video_source:="$VIDEO" -p loop_video:=true
  -p publish_rate_hz:="$TRACKER_RATE" -p tracker.kind:=deepsort_cascade
  -p tracker.n_init:=1 -p tracker.include_tentative:=true
  -p detector.imgsz:=320 -p detector.conf:=0.2 -p enclosure.enabled:=false
  -p detector.backend:="$DETECTOR_BACKEND"
)
if [[ "$DETECTOR_BACKEND" == "yolo" ]]; then
  TRACKER_ARGS+=( -p detector.weights:="$WEIGHTS" )
fi
launch_node "$OUTPUT_DIR/tracker.log" "${TRACKER_ARGS[@]}"

sleep 4
python3 "$WORKSPACE/test_video_closed_loop.py" \
  --duration "$DURATION" --output "$OUTPUT_DIR/trace.json" \
  --source-video "$VIDEO" --source-width "$VIDEO_WIDTH" \
  --source-height "$VIDEO_HEIGHT" --source-fps "$VIDEO_FPS" \
  --tracker-rate "$TRACKER_RATE" --detector-backend "$DETECTOR_BACKEND"
echo "[video-closed-loop] trace: $OUTPUT_DIR/trace.json"
