#!/usr/bin/env bash

# Run the real decision nodes against a dynamic two-target replay fixture.
# This intentionally excludes PX4, MAVROS, and every vehicle-control bridge.

# ROS 2 Humble setup scripts reference optional variables before defining them.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"
INSTALL_BASE="${CVTRACK_INSTALL_BASE:-$WORKSPACE/install}"
OUTPUT_DIR="${OUTPUT_DIR:-$REPO_ROOT/output/closed_loop_demo}"
DURATION=12
DOMAIN_ID="${ROS_DOMAIN_ID:-63}"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Options:
  --install-base DIR  ROS2 install overlay to use.
  --output-dir DIR    Report and node logs directory.
  --duration SEC      Replay duration, default: 12.
  --domain-id ID      Isolated ROS domain, default: 63.
  --help              Show this help.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-base) INSTALL_BASE="${2:?missing install base}"; shift 2 ;;
    --output-dir) OUTPUT_DIR="${2:?missing output directory}"; shift 2 ;;
    --duration) DURATION="${2:?missing duration}"; shift 2 ;;
    --domain-id) DOMAIN_ID="${2:?missing domain id}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[closed-loop-demo] unknown option: $1" >&2; exit 2 ;;
  esac
done

[[ "$DURATION" =~ ^[1-9][0-9]*([.][0-9]+)?$ ]] || { echo "[closed-loop-demo] duration must be positive" >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "[closed-loop-demo] ROS 2 Humble is required" >&2; exit 2; }
[[ -f "$INSTALL_BASE/setup.bash" ]] || { echo "[closed-loop-demo] missing install overlay: $INSTALL_BASE" >&2; exit 2; }

source /opt/ros/humble/setup.bash
source "$INSTALL_BASE/setup.bash"

# Isolated workspaces may be installed either package-by-package or through
# colcon's --merge-install layout.  The former needs the interface package
# prefix made explicit; a merged installation already exposes it at the root.
INTERFACE_PREFIX="$INSTALL_BASE/swarm_interfaces"
if [[ -d "$INTERFACE_PREFIX" ]]; then
  export AMENT_PREFIX_PATH="$INTERFACE_PREFIX:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$INTERFACE_PREFIX/lib:${LD_LIBRARY_PATH:-}"
fi
ros2 pkg prefix swarm_interfaces >/dev/null || {
  echo "[closed-loop-demo] swarm_interfaces is unavailable in $INSTALL_BASE" >&2
  exit 2
}
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER

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
  setsid "$@" >"$logfile" 2>&1 & PIDS+=("$!")
}

launch_node "$OUTPUT_DIR/scheduler.log" ros2 run scheduler_pkg scheduler_node --ros-args \
  -p num_drones:=4 -p max_per_drone:=1 -p tick_period:=0.2 -p log_interval_sec:=2.0
launch_node "$OUTPUT_DIR/grid_map.log" ros2 run planning_pkg grid_map_node
launch_node "$OUTPUT_DIR/planner.log" ros2 run planning_pkg planner_node --ros-args \
  -p num_drones:=4 -p grid_size:=40 -p sim_tick_speed:=0.3 \
  -p initial_positions:="[2.0, 20.0, 2.0, 5.0, 39.0, 39.0, 39.0, 0.0]" \
  -p tick_period:=0.2 -p log_interval_sec:=2.0
launch_node "$OUTPUT_DIR/enclosure.log" ros2 run containment_pkg enclosure_node --ros-args \
  -p update_period:=0.2

sleep 3
python3 "$WORKSPACE/test_process_closed_loop.py" \
  --duration "$DURATION" --output "$OUTPUT_DIR/report.json"
echo "[closed-loop-demo] report: $OUTPUT_DIR/report.json"
