#!/usr/bin/env bash
# Launch three PX4 v1.14 Gazebo Classic SITL instances without a GUI.
#
# Manual mode keeps the processes in the foreground. Batch mode adds a bounded
# process-stability check and archives launcher/PX4 logs. This script never
# starts MAVROS, arms a vehicle, changes flight mode, or sends setpoints.

set -eo pipefail

PX4_SRC="${PX4_SITL_ROOT:-$HOME/src/PX4-Autopilot}"
TARGET="px4_sitl_default"
WORLD="empty"
NUM_UAV=3
DURATION_SECONDS=""
STARTUP_TIMEOUT_SECONDS=60
OUTPUT_DIR=""
RUN_ID=""
CLEANUP_LEFTOVERS=0
GZ_PID=""
PX4_PIDS=()
RESULT_STATUS="failed"
FAILURE_REASON=""
START_EPOCH="$(date +%s)"

usage() {
    cat <<'EOF'
Usage: bash start_3uav_sitl.sh [options]

Launch three PX4/Gazebo Classic SITL iris instances. With no --duration, the
script remains in the foreground for interactive diagnosis.

Options:
  --px4-sitl-root PATH   PX4-Autopilot root (default: $PX4_SITL_ROOT or ~/src/PX4-Autopilot)
  --world NAME_OR_PATH   Gazebo Classic world name or .world path (default: empty)
  --duration SECONDS     Verify all four simulator processes stay alive, then exit
  --startup-timeout SEC  Maximum wait for gzserver readiness (default: 60)
  --output-dir PATH      Write result.json and archive logs to PATH
  --run-id ID            Identifier written to result.json
  --cleanup-leftovers    Explicitly kill old PX4/Gazebo processes before launch
  -h, --help             Show this help

Safety boundary: this launcher is PX4/Gazebo SITL only. It does not start
MAVROS, arm vehicles, switch modes, or publish flight setpoints.
EOF
}

fail() {
    FAILURE_REASON="$1"
    echo "[sitl] ERROR: $FAILURE_REASON" >&2
    exit 1
}

is_positive_integer() {
    [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

cleanup_owned_processes() {
    local pid attempt remaining
    for pid in "${PX4_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    if [[ -n "$GZ_PID" ]]; then
        kill "$GZ_PID" 2>/dev/null || true
    fi

    for attempt in $(seq 1 5); do
        remaining=0
        for pid in "${PX4_PIDS[@]}" "$GZ_PID"; do
            if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
                remaining=1
            fi
        done
        [[ "$remaining" -eq 0 ]] && return
        sleep 1
    done

    for pid in "${PX4_PIDS[@]}" "$GZ_PID"; do
        [[ -n "$pid" ]] && kill -9 "$pid" 2>/dev/null || true
    done
}

archive_logs() {
    local instance working_dir log_name
    [[ -n "$OUTPUT_DIR" ]] || return

    mkdir -p "$OUTPUT_DIR/px4"
    for instance in $(seq 1 "$NUM_UAV"); do
        working_dir="$BUILD_PATH/rootfs/$instance"
        mkdir -p "$OUTPUT_DIR/px4/$instance"
        for log_name in out.log err.log; do
            if [[ -f "$working_dir/$log_name" ]]; then
                cp "$working_dir/$log_name" "$OUTPUT_DIR/px4/$instance/$log_name"
            fi
        done
    done
}

write_result() {
    local end_epoch actual_duration result_path
    [[ -n "$OUTPUT_DIR" ]] || return

    end_epoch="$(date +%s)"
    actual_duration=$((end_epoch - START_EPOCH))
    result_path="$OUTPUT_DIR/result.json"
    RUN_ID="$RUN_ID" RESULT_STATUS="$RESULT_STATUS" FAILURE_REASON="$FAILURE_REASON" \
        STARTUP_TIMEOUT_SECONDS="$STARTUP_TIMEOUT_SECONDS" DURATION_SECONDS="$DURATION_SECONDS" \
        ACTUAL_DURATION_SECONDS="$actual_duration" START_EPOCH="$START_EPOCH" END_EPOCH="$end_epoch" \
        GZ_PID="$GZ_PID" PX4_PID_COUNT="${#PX4_PIDS[@]}" python3 - "$result_path" <<'PY'
import json
import os
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
payload = {
    "run_id": os.environ["RUN_ID"],
    "status": os.environ["RESULT_STATUS"],
    "failure_reason": os.environ["FAILURE_REASON"] or None,
    "startup_timeout_seconds": int(os.environ["STARTUP_TIMEOUT_SECONDS"]),
    "stability_window_seconds": int(os.environ["DURATION_SECONDS"] or 0),
    "actual_duration_seconds": int(os.environ["ACTUAL_DURATION_SECONDS"]),
    "started_epoch": int(os.environ["START_EPOCH"]),
    "ended_epoch": int(os.environ["END_EPOCH"]),
    "gzserver_pid": int(os.environ["GZ_PID"]) if os.environ["GZ_PID"] else None,
    "px4_instance_count": int(os.environ["PX4_PID_COUNT"]),
    "scope": "PX4/Gazebo SITL process stability only; MAVROS/arming/mode/setpoints excluded",
}
path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
PY
}

on_exit() {
    local exit_code=$?
    trap - EXIT INT TERM
    if [[ "$exit_code" -ne 0 && -z "$FAILURE_REASON" ]]; then
        FAILURE_REASON="launcher exited with status $exit_code"
    fi
    archive_logs
    write_result
    if [[ -n "$GZ_PID" || "${#PX4_PIDS[@]}" -gt 0 ]]; then
        cleanup_owned_processes
    fi
    exit "$exit_code"
}

trap on_exit EXIT
trap 'FAILURE_REASON="interrupted"; exit 130' INT TERM

while [[ $# -gt 0 ]]; do
    case "$1" in
        --px4-sitl-root)
            [[ $# -ge 2 ]] || fail "--px4-sitl-root requires a path"
            PX4_SRC="$2"
            shift 2
            ;;
        --world)
            [[ $# -ge 2 ]] || fail "--world requires a name"
            WORLD="$2"
            shift 2
            ;;
        --duration)
            [[ $# -ge 2 ]] || fail "--duration requires seconds"
            DURATION_SECONDS="$2"
            shift 2
            ;;
        --startup-timeout)
            [[ $# -ge 2 ]] || fail "--startup-timeout requires seconds"
            STARTUP_TIMEOUT_SECONDS="$2"
            shift 2
            ;;
        --output-dir)
            [[ $# -ge 2 ]] || fail "--output-dir requires a path"
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --run-id)
            [[ $# -ge 2 ]] || fail "--run-id requires an identifier"
            RUN_ID="$2"
            shift 2
            ;;
        --cleanup-leftovers)
            CLEANUP_LEFTOVERS=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            fail "unknown option: $1"
            ;;
    esac
done

if [[ -n "$DURATION_SECONDS" ]] && ! is_positive_integer "$DURATION_SECONDS"; then
    fail "--duration must be a positive integer"
fi
if ! is_positive_integer "$STARTUP_TIMEOUT_SECONDS"; then
    fail "--startup-timeout must be a positive integer"
fi

if [[ -z "$RUN_ID" ]]; then
    RUN_ID="manual-$(date +%Y%m%d-%H%M%S)"
fi
if [[ -n "$DURATION_SECONDS" && -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="$PWD/sitl_runs/$RUN_ID"
fi
if [[ -n "$OUTPUT_DIR" ]]; then
    mkdir -p "$OUTPUT_DIR"
    OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"
fi

BUILD_PATH="$PX4_SRC/build/$TARGET"
if [[ "$WORLD" == *.world || "$WORLD" == */* ]]; then
    GAZEBO_WORLD="$WORLD"
else
    GAZEBO_WORLD="$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/${WORLD}.world"
fi
JINJA="$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py"
MODEL_DIR="$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris"
PX4_BIN="$BUILD_PATH/bin/px4"

command -v gzserver >/dev/null || fail "gzserver is not on PATH"
command -v gz >/dev/null || fail "gz is not on PATH"
command -v python3 >/dev/null || fail "python3 is not on PATH"
[[ -x "$PX4_BIN" ]] || fail "PX4 binary not found at $PX4_BIN; build: make px4_sitl_default gazebo-classic"
[[ -f "$GAZEBO_WORLD" ]] || fail "Gazebo world not found: $GAZEBO_WORLD"
[[ -f "$JINJA" ]] || fail "PX4 Jinja generator not found: $JINJA"

if [[ "$CLEANUP_LEFTOVERS" -eq 1 ]]; then
    echo "[sitl] explicitly cleaning existing PX4/Gazebo processes ..."
    pkill -9 -f gzserver 2>/dev/null || true
    pkill -9 -f gazebo 2>/dev/null || true
    pkill -9 -f px4 2>/dev/null || true
    sleep 2
fi

cd "$PX4_SRC"
source "$PX4_SRC/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_SRC" "$BUILD_PATH"
export HEADLESS=1
export DISPLAY=""
export PX4_SIM_MODEL=gazebo-classic_iris
unset ROS_VERSION 2>/dev/null || true

if [[ -n "$OUTPUT_DIR" ]]; then
    GZ_LOG="$OUTPUT_DIR/gzserver.log"
else
    GZ_LOG="/tmp/gazebo_multi.log"
fi
echo "[sitl] starting gzserver (log: $GZ_LOG) ..."
gzserver "$GAZEBO_WORLD" --verbose > "$GZ_LOG" 2>&1 &
GZ_PID=$!

for elapsed in $(seq 0 "$STARTUP_TIMEOUT_SECONDS"); do
    if ! kill -0 "$GZ_PID" 2>/dev/null; then
        tail -n 40 "$GZ_LOG" >&2 || true
        fail "gzserver exited during startup"
    fi
    if gz model -l >/dev/null 2>&1; then
        break
    fi
    if [[ "$elapsed" -eq "$STARTUP_TIMEOUT_SECONDS" ]]; then
        fail "gzserver did not become ready within ${STARTUP_TIMEOUT_SECONDS}s"
    fi
    sleep 1
done
echo "[sitl] gzserver is up (pid $GZ_PID)."

for instance in $(seq 1 "$NUM_UAV"); do
    working_dir="$BUILD_PATH/rootfs/$instance"
    mkdir -p "$working_dir"
    echo "[sitl] spawning iris_${instance} (px4 instance $instance) ..."
    (
        cd "$working_dir"
        exec "$PX4_BIN" -i "$instance" -d "$BUILD_PATH/etc" > out.log 2> err.log
    ) &
    PX4_PIDS+=("$!")

    python3 "$JINJA" "$MODEL_DIR/iris.sdf.jinja" \
        "$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
        --mavlink_tcp_port "$((4560 + instance))" \
        --mavlink_udp_port "$((14560 + instance))" \
        --mavlink_id "$((1 + instance))" \
        --gst_udp_port "$((5600 + instance))" \
        --video_uri "$((5600 + instance))" \
        --mavlink_cam_udp_port "$((14530 + instance))" \
        --output-file "/tmp/iris_${instance}.sdf"

    spawned=0
    for attempt in 1 2 3 4 5; do
        if gz model --spawn-file="/tmp/iris_${instance}.sdf" \
            --model-name="iris_${instance}" -x 0 -y "$((3 * instance))" -z 0.83; then
            spawned=1
            break
        fi
        echo "[sitl] model spawn attempt $attempt failed; retrying ..." >&2
        sleep 2
    done
    [[ "$spawned" -eq 1 ]] || fail "could not spawn iris_${instance}"
    sleep 2
done

echo "[sitl] 3x PX4 SITL + Gazebo Classic is running."
echo "[sitl] Offboard listen ports: 14581 / 14582 / 14583"
echo "[sitl] This launcher does not start MAVROS or command vehicles."

if [[ -z "$DURATION_SECONDS" ]]; then
    echo "[sitl] Manual mode: Ctrl-C stops only the simulator processes started here."
    wait -n || true
    fail "a simulator process exited in manual mode"
fi

echo "[sitl] checking process stability for ${DURATION_SECONDS}s ..."
for elapsed in $(seq 0 "$DURATION_SECONDS"); do
    if ! kill -0 "$GZ_PID" 2>/dev/null; then
        fail "gzserver exited during stability window"
    fi
    for index in "${!PX4_PIDS[@]}"; do
        if ! kill -0 "${PX4_PIDS[$index]}" 2>/dev/null; then
            fail "PX4 instance $((index + 1)) exited during stability window"
        fi
    done
    [[ "$elapsed" -eq "$DURATION_SECONDS" ]] && break
    sleep 1
done

RESULT_STATUS="passed"
echo "[sitl] PASS: all PX4/Gazebo simulator processes survived ${DURATION_SECONDS}s."
