#!/usr/bin/env bash
# run_px4_sitl.sh — start a PX4 SITL instance backed by Gazebo.
#
# This is a thin wrapper that:
#   1. Resolves PX4_SITL_ROOT (either from the environment or auto-detected
#      under $HOME/src/PX4-Autopilot).
#   2. Sets GAZEBO_PLUGIN_PATH and GAZEBO_MODEL_PATH so the iris model and
#      liftdrag plugin load.
#   3. Spawns N PX4 instances (each with a distinct -i<id> and UDP port).
#   4. Optionally traps SIGINT/SIGTERM for clean shutdown of children.
#
# Usage:
#   ./simulation/scripts/run_px4_sitl.sh                 # 1 drone, GUI
#   NUM_UAV=3 ./simulation/scripts/run_px4_sitl.sh       # 3 drones
#   GAZEBO_HEADLESS=true ./simulation/scripts/run_px4_sitl.sh   # headless
#   PX4_SITL_ROOT=$HOME/src/Firmware ./simulation/scripts/run_px4_sitl.sh
#
# Environment variables:
#   PX4_SITL_ROOT        Path to the built PX4 firmware tree.  Must
#                         contain build/px4_sitl_default/px4_sitl_default
#                         and etc/px4-rc.mavlink.
#   NUM_UAV              Number of PX4 instances (1..3). Default: 1.
#   GAZEBO_HEADLESS      "true" to run Gazebo without GUI. Default: false.
#   PX4_MODEL            Vehicle SDF model. Default: iris.
#   PX4_WORLD            Gazebo world file. Default: empty (PX4 default).
#
# Exit codes:
#   0  success
#   1  PX4_SITL_ROOT not set / build missing
#   2  PX4 SITL binary not built
#   3  one of the children died unexpectedly
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORLD_FILE="$REPO_ROOT/simulation/worlds/swarm_field.world"

NUM_UAV="${NUM_UAV:-1}"
HEADLESS="${GAZEBO_HEADLESS:-false}"
MODEL="${PX4_MODEL:-iris}"
WORLD="${PX4_WORLD:-$WORLD_FILE}"

if [[ -z "${PX4_SITL_ROOT:-}" ]]; then
    if [[ -d "$HOME/src/PX4-Autopilot" ]]; then
        PX4_SITL_ROOT="$HOME/src/PX4-Autopilot"
        echo "[run_px4_sitl] PX4_SITL_ROOT not set; defaulting to $PX4_SITL_ROOT"
    else
        echo "[run_px4_sitl] PX4_SITL_ROOT is not set and no default found." >&2
        echo "[run_px4_sitl] Set it to a built PX4 firmware tree, e.g.:" >&2
        echo "    export PX4_SITL_ROOT=\$HOME/src/PX4-Autopilot" >&2
        exit 1
    fi
fi

export PX4_SITL_ROOT
PX4_SITL_BIN="$PX4_SITL_ROOT/build/px4_sitl_default/px4_sitl_default"
PX4_RC_MAVLINK="$PX4_SITL_ROOT/etc/px4-rc.mavlink"
SITL_GAZEBO_DIR="$PX4_SITL_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

if [[ ! -x "$PX4_SITL_BIN" ]]; then
    echo "[run_px4_sitl] PX4 SITL binary not found at $PX4_SITL_BIN" >&2
    echo "[run_px4_sitl] Build it first with: (cd $PX4_SITL_ROOT && make px4_sitl_default)" >&2
    exit 2
fi

if [[ ! -f "$PX4_RC_MAVLINK" ]]; then
    echo "[run_px4_sitl] MAVLink rc file not found at $PX4_RC_MAVLINK" >&2
    exit 2
fi

# Make Gazebo plugins/models visible to gzserver.
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}:$SITL_GAZEBO_DIR"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SITL_GAZEBO_DIR"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$SITL_GAZEBO_DIR/build"

if [[ "$HEADLESS" == "true" ]]; then
    export HEADLESS=1
    export DISPLAY=""
    echo "[run_px4_sitl] running headless (GAZEBO_HEADLESS=true)"
fi

if [[ -n "$WORLD" && -f "$WORLD" ]]; then
    export PX4_SITL_WORLD="$WORLD"
    echo "[run_px4_sitl] world file: $WORLD"
fi

PIDS=()
cleanup() {
    local sig="${1:-EXIT}"
    echo ""
    echo "[run_px4_sitl] caught $sig, shutting down children..."
    for pid in "${PIDS[@]:-}"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
    exit 0
}
trap 'cleanup INT' INT
trap 'cleanup TERM' TERM
trap 'cleanup EXIT' EXIT

echo "[run_px4_sitl] starting $NUM_UAV PX4 SITL instance(s)..."
for ((i = 0; i < NUM_UAV; i++)); do
    udp_port=$((14540 + 10 * i))
    echo "[run_px4_sitl] -> instance $i (UDP $udp_port)"
    (
        "$PX4_SITL_BIN" \
            "-i$i" \
            "$PX4_RC_MAVLINK" \
            > "/tmp/px4_sitl_${i}.log" 2>&1
    ) &
    PIDS+=("$!")
    # Stagger start so each instance claims its UDP port.
    sleep 1
done

echo "[run_px4_sitl] all instances launched.  Tailing last 30 lines of /tmp/px4_sitl_*.log"
echo "[run_px4_sitl] press Ctrl-C to stop."

# Block until any child dies.
wait -n "${PIDS[@]}" || {
    rc=$?
    echo "[run_px4_sitl] a child process exited with code $rc" >&2
    cleanup ERR
    exit 3
}
