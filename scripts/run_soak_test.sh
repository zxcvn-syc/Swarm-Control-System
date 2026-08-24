#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"
INSTALL_BASE="${CVTRACK_INSTALL_BASE:-$WORKSPACE/install}"
OUTPUT_DIR="$REPO_ROOT/output/soak"

DURATION=7200
SAMPLE_INTERVAL=30
STARTUP_GRACE=45
VIDEO="$REPO_ROOT/videos/test_multi_target_tracking.mp4"
if [[ ! -f "$VIDEO" ]]; then
  VIDEO="$WORKSPACE/src/perception_pkg/test_videos/pexels_pedestrian_crossing.mp4"
fi
DRY_RUN=0

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Options:
  --duration SEC       Unattended run duration; default: 7200.
  --sample-interval S  Memory/node sample interval; default: 30.
  --startup-grace SEC  Maximum startup time before a missing node fails the run; default: 45.
  --video FILE         Tracker video source.
  --dry-run            Validate prerequisites without launching nodes.
  --help               Show this help.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="${2:?missing duration}"; shift 2 ;;
    --sample-interval) SAMPLE_INTERVAL="${2:?missing interval}"; shift 2 ;;
    --startup-grace) STARTUP_GRACE="${2:?missing startup grace}"; shift 2 ;;
    --video) VIDEO="${2:?missing video}"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[soak] unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$DURATION" =~ ^[1-9][0-9]*$ ]] || { echo "[soak] duration must be a positive integer" >&2; exit 2; }
[[ "$SAMPLE_INTERVAL" =~ ^[1-9][0-9]*$ ]] || { echo "[soak] sample interval must be a positive integer" >&2; exit 2; }
[[ "$STARTUP_GRACE" =~ ^[1-9][0-9]*$ ]] || { echo "[soak] startup grace must be a positive integer" >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "[soak] ROS2 Humble is required" >&2; exit 2; }
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "[soak] build the workspace before running" >&2; exit 2; }
[[ -f "$VIDEO" ]] || { echo "[soak] video not found: $VIDEO" >&2; exit 2; }

source /opt/ros/humble/setup.bash
source "$INSTALL_BASE/setup.bash"

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[soak] prerequisites valid: duration=${DURATION}s video=$VIDEO"
  exit 0
fi

mkdir -p "$OUTPUT_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$OUTPUT_DIR/soak_${STAMP}.log"
METRICS_FILE="$OUTPUT_DIR/soak_${STAMP}_samples.csv"
REPORT_FILE="$OUTPUT_DIR/soak_${STAMP}_report.json"
LAUNCH_PID=""
START_EPOCH="$(date +%s)"
EXIT_CODE=0
CRASH_REASON=""
EXPECTED_NODES=(
  "/tracker_node"
  "/coord_transform_node"
  "/scheduler_node"
  "/planner_node"
  "/enclosure_node"
  "/ugv_state_publisher"
  "/px4_offboard_bridge"
  "/sitl_pose_bridge"
)

cleanup() {
  local signal
  local child_pids

  [[ -n "$LAUNCH_PID" ]] || return 0
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 5
  fi
  for signal in TERM KILL; do
    child_pids="$(pgrep -P "$LAUNCH_PID" 2>/dev/null || true)"
    if [[ -n "$child_pids" ]]; then
      kill -"$signal" $child_pids 2>/dev/null || true
    fi
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -"$signal" "$LAUNCH_PID" 2>/dev/null || true
      sleep 1
    else
      break
    fi
  done
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

missing_required_nodes() {
  local node_list
  local node
  local missing=()

  node_list="$(ros2 node list 2>/dev/null || true)"
  for node in "${EXPECTED_NODES[@]}"; do
    if [[ $'\n'"$node_list"$'\n' != *$'\n'"$node"$'\n'* ]]; then
      missing+=("$node")
    fi
  done

  local IFS='|'
  printf '%s' "${missing[*]}"
}

echo "timestamp,elapsed_s,launch_alive,rss_kb,node_count,missing_nodes" > "$METRICS_FILE"
echo "[soak] start duration=${DURATION}s sample=${SAMPLE_INTERVAL}s startup_grace=${STARTUP_GRACE}s" | tee "$LOG_FILE"
ros2 launch "$WORKSPACE/launch/three_links.launch.py" video_source:="$VIDEO" \
  >> "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

while (( $(date +%s) - START_EPOCH < DURATION )); do
  now="$(date +%s)"
  elapsed=$((now - START_EPOCH))
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    alive=1
    rss_kb="$(ps -o rss= -p "$LAUNCH_PID" | awk '{sum += $1} END {print sum + 0}')"
    node_count="$(ros2 node list 2>/dev/null | sed '/^$/d' | wc -l | tr -d ' ')"
    missing_nodes="$(missing_required_nodes)"
    if [[ -n "$missing_nodes" ]] && (( elapsed >= STARTUP_GRACE )); then
      EXIT_CODE=1
      CRASH_REASON="required ROS nodes missing: $missing_nodes"
    fi
  else
    alive=0
    rss_kb=0
    node_count=0
    missing_nodes="launch_process"
    EXIT_CODE=1
    CRASH_REASON="launch exited before requested duration"
  fi
  printf '%s,%s,%s,%s,%s,%s\n' "$(date -Is)" "$elapsed" "$alive" "$rss_kb" "$node_count" "${missing_nodes:-none}" >> "$METRICS_FILE"
  [[ "$EXIT_CODE" == "0" ]] || break
  sleep "$SAMPLE_INTERVAL"
done

if [[ "$EXIT_CODE" == "0" ]]; then
  echo "[soak] requested duration reached" | tee -a "$LOG_FILE"
else
  echo "[soak] failed: $CRASH_REASON" | tee -a "$LOG_FILE" >&2
fi

END_EPOCH="$(date +%s)"
ELAPSED=$((END_EPOCH - START_EPOCH))
cat > "$REPORT_FILE" <<JSON
{
  "status": "$( [[ "$EXIT_CODE" == "0" ]] && echo PASS || echo FAIL )",
  "requested_duration_s": $DURATION,
  "elapsed_duration_s": $ELAPSED,
  "sample_interval_s": $SAMPLE_INTERVAL,
  "video_source": "$VIDEO",
  "log_file": "$LOG_FILE",
  "memory_samples_csv": "$METRICS_FILE",
  "failure_reason": "$CRASH_REASON"
}
JSON

echo "[soak] report: $REPORT_FILE"
exit "$EXIT_CODE"
