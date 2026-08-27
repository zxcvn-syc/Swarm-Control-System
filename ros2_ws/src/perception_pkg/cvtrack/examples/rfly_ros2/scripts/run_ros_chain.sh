#!/usr/bin/env bash
set -eo pipefail

DEMO_ROOT="${RFLY_DEMO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-$DEMO_ROOT/ros2_ws}"
if [[ -z "${ROS2_SETUP:-}" ]]; then
  for candidate in \
    "$ROS2_WS_ROOT/install_cvtrack/setup.bash" \
    "$ROS2_WS_ROOT/install_validation/setup.bash" \
    "$ROS2_WS_ROOT/install/setup.bash"; do
    if [[ -f "$candidate" ]]; then
      ROS2_SETUP="$candidate"
      break
    fi
  done
fi
: "${ROS2_SETUP:?Set ROS2_SETUP or build install_cvtrack/install_validation/install under ROS2_WS_ROOT}"
LOG_ROOT="${RFLY_LOG_ROOT:-$DEMO_ROOT/logs}"
RUN_SECONDS="${1:-35}"
EVIDENCE_DURATION="${RFLY_EVIDENCE_DURATION:-$((RUN_SECONDS + 12))}"
SCENARIO="${2:-clear_grasslands}"
TARGET_TOPIC="${RFLY_TARGET_TOPIC:-/target_track_world}"
RUN_ID="${RFLY_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
PID_FILE="$LOG_ROOT/ros_chain_${RUN_ID}.pids"

mkdir -p "$LOG_ROOT"
source /opt/ros/humble/setup.bash
source "$ROS2_SETUP"
set -u
export ROS_DOMAIN_ID=61
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RFLY_SCENE_SEED=20260821
export RFLY_SCENARIO="$SCENARIO"
export RFLY_HOST_IP="${RFLY_HOST_IP:-127.0.0.1}"
export RFLY_SDK_ROOT="${RFLY_SDK_ROOT:-$DEMO_ROOT/rfly_sdk}"
export RFLY_LOG_ROOT="$LOG_ROOT"

printf 'scenario=%s run_seconds=%s target_topic=%s rfly_host_ip=%s rfly_sdk_root=%s ros2_setup=%s\n' \
  "$SCENARIO" "$RUN_SECONDS" "$TARGET_TOPIC" "$RFLY_HOST_IP" "$RFLY_SDK_ROOT" "$ROS2_SETUP" \
  >"$LOG_ROOT/scenario.txt"
rm -f "$PID_FILE"

start_grouped() {
  local output_file=$1
  shift
  setsid "$@" >"$output_file" 2>&1 &
  LAST_PID=$!
  printf '%s\n' "$LAST_PID" >>"$PID_FILE"
}

start_grouped "$LOG_ROOT/evidence_recorder.log" python3 "$DEMO_ROOT/scripts/capture_ros_evidence.py" \
  --duration "$EVIDENCE_DURATION" \
  --output-dir "$LOG_ROOT"
EVIDENCE_PID=$LAST_PID

start_grouped "$LOG_ROOT/scene.log" python3 "$DEMO_ROOT/scripts/rfly_ros_scene.py"
SCENE_PID=$LAST_PID
start_grouped "$LOG_ROOT/scheduler.log" ros2 run scheduler_pkg scheduler_node --ros-args \
  -p num_drones:=3 \
  -p target_topic:="$TARGET_TOPIC" \
  -p world_frame:=world \
  -r __node:=rfly_scheduler_${RUN_ID}
SCHEDULER_PID=$LAST_PID
start_grouped "$LOG_ROOT/planner.log" ros2 run planning_pkg planner_node --ros-args \
  -p num_drones:=3 \
  -p grid_size:=220 \
  -p world_frame:=world \
  -p grid_resolution_m:=1.0 \
  -p drone_z_default:=18.0 \
  -p sim_tick_speed:=3.0 \
  -p target_track_world_topic:="$TARGET_TOPIC" \
  -r __node:=rfly_planner_${RUN_ID}
PLANNER_PID=$LAST_PID
start_grouped "$LOG_ROOT/enclosure.log" ros2 run containment_pkg enclosure_node --ros-args \
  -p enclosure_radius:=18.0 \
  -p min_dist:=8.0 \
  -p update_period:=0.25 \
  -p target_topic:="$TARGET_TOPIC" \
  -p drone_topic:=/ground_vehicle_states \
  -p world_frame:=world \
  -r __node:=rfly_enclosure_${RUN_ID}
ENCLOSURE_PID=$LAST_PID

cleanup() {
  for pid in "$EVIDENCE_PID" "$ENCLOSURE_PID" "$PLANNER_PID" "$SCHEDULER_PID" "$SCENE_PID"; do
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -- -"$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$EVIDENCE_PID" "$ENCLOSURE_PID" "$PLANNER_PID" "$SCHEDULER_PID" "$SCENE_PID"; do
    wait "$pid" 2>/dev/null || true
  done
}
on_exit() {
  local exit_code=$?
  trap - EXIT
  cleanup
  rm -f "$PID_FILE"
  exit "$exit_code"
}
trap on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

sleep 9
ros2 topic list --no-daemon --spin-time 3 -t >"$LOG_ROOT/topics.txt"
ros2 node list --no-daemon --spin-time 3 >"$LOG_ROOT/nodes.txt"

sleep "$RUN_SECONDS"

# Let the recorder write its manifest before the EXIT trap stops the scene.
if ! wait "$EVIDENCE_PID"; then
  echo "[run_ros_chain] evidence recorder exited unsuccessfully" >&2
  exit 1
fi
if [[ ! -s "$LOG_ROOT/evidence_manifest.json" || ! -s "$LOG_ROOT/capture_summary.json" ]]; then
  echo "[run_ros_chain] required evidence files were not written" >&2
  exit 1
fi
printf '{"status":"ok","run_id":"%s"}\n' "$RUN_ID" >"$LOG_ROOT/run_complete.json"
