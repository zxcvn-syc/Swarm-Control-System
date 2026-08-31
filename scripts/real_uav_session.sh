#!/usr/bin/env bash
#
# Start one physical PX4 connection and the local safety console.
#
# This script is intentionally not a flight-mode controller. It never sends
# ARM, DISARM, OFFBOARD, takeoff, landing, RTL, or setpoint commands itself.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="$REPO_DIR/ros2_ws"
DEFAULT_CONFIG="/etc/swarm-control/real_uav_connection.env"

MODE=""
CONFIG_PATH="${SWARM_CONTROL_CONNECTION_CONFIG:-$DEFAULT_CONFIG}"

usage() {
    cat <<'EOF'
Usage:
  scripts/real_uav_session.sh --discover
  scripts/real_uav_session.sh --monitor [--config PATH]
  scripts/real_uav_session.sh --controls [--config PATH]

--discover  Lists serial devices visible to this Linux host. It does not open
            a device or start ROS.
--monitor   Starts MAVROS and a locked, read-only local safety console.
--controls  Starts MAVROS and the explicitly enabled, audited pilot console.
            It does not auto-arm, change PX4 mode, or start the Offboard bridge.

The configuration defaults to /etc/swarm-control/real_uav_connection.env.
EOF
}

fail() {
    printf 'real_uav_session: %s\n' "$*" >&2
    exit 2
}

discover_devices() {
    local device
    local found=0

    printf 'Visible candidate FCU serial devices:\n'
    shopt -s nullglob
    for device in /dev/serial/by-id/* /dev/ttyACM* /dev/ttyUSB*; do
        [[ -c "$device" ]] || continue
        printf '  %s -> %s\n' "$device" "$(readlink -f "$device")"
        found=1
    done
    shopt -u nullglob

    if [[ "$found" -eq 0 ]]; then
        printf '  none\n'
        printf 'Attach the Pixhawk or telemetry USB adapter to the Linux host/VM, then run this command again.\n' >&2
        return 1
    fi
}

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --discover|--monitor|--controls)
            [[ -z "$MODE" ]] || fail 'select exactly one mode'
            MODE="${1#--}"
            ;;
        --config)
            [[ "$#" -ge 2 ]] || fail '--config requires a path'
            CONFIG_PATH="$2"
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            fail "unknown argument: $1"
            ;;
    esac
    shift
done

[[ -n "$MODE" ]] || {
    usage >&2
    exit 2
}

if [[ "$MODE" == 'discover' ]]; then
    discover_devices
    exit "$?"
fi

[[ -r "$CONFIG_PATH" ]] || fail "connection config is not readable: $CONFIG_PATH"
# The file is an administrator-controlled shell assignment file; never source
# a file received from an untrusted host.
# shellcheck source=/dev/null
source "$CONFIG_PATH"

: "${FCU_DEVICE:?FCU_DEVICE is required in $CONFIG_PATH}"
: "${FCU_BAUD:?FCU_BAUD is required in $CONFIG_PATH}"
: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID is required in $CONFIG_PATH}"
: "${MAVROS_NAMESPACE:?MAVROS_NAMESPACE is required in $CONFIG_PATH}"
: "${FCU_SYSTEM_ID:?FCU_SYSTEM_ID is required in $CONFIG_PATH}"
: "${FCU_COMPONENT_ID:?FCU_COMPONENT_ID is required in $CONFIG_PATH}"
: "${DASHBOARD_PORT:?DASHBOARD_PORT is required in $CONFIG_PATH}"
GCS_URL="${GCS_URL:-}"

[[ -c "$FCU_DEVICE" ]] || fail "FCU device is absent or not a character device: $FCU_DEVICE"
[[ "$FCU_BAUD" =~ ^[0-9]+$ ]] || fail 'FCU_BAUD must be an integer'
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ && "$ROS_DOMAIN_ID" -le 232 ]] || fail 'ROS_DOMAIN_ID must be 0 through 232'
[[ "$FCU_SYSTEM_ID" =~ ^[0-9]+$ ]] || fail 'FCU_SYSTEM_ID must be an integer'
[[ "$FCU_COMPONENT_ID" =~ ^[0-9]+$ ]] || fail 'FCU_COMPONENT_ID must be an integer'
[[ "$DASHBOARD_PORT" =~ ^[0-9]+$ && "$DASHBOARD_PORT" -ge 1024 && "$DASHBOARD_PORT" -le 65535 ]] || fail 'DASHBOARD_PORT must be 1024 through 65535'
[[ -d "$WORKSPACE_DIR/install" ]] || fail "ROS workspace is not built: $WORKSPACE_DIR/install"

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
export ROS_DOMAIN_ID

SESSION_DIR="$HOME/flight_evidence/$(date +%Y%m%d)/session-$(date +%H%M%S)"
mkdir -p "$SESSION_DIR"
chmod 700 "$SESSION_DIR"

MAVROS_NAMESPACE="/${MAVROS_NAMESPACE#/}"
MAVROS_NAMESPACE_ARGUMENT="${MAVROS_NAMESPACE#/}"
MAVROS_STATE_TOPIC="$MAVROS_NAMESPACE/state"
MAVROS_PID=""
CONSOLE_PID=""

cleanup() {
    local pid
    for pid in "$CONSOLE_PID" "$MAVROS_PID"; do
        [[ -n "$pid" ]] || continue
        kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "$CONSOLE_PID" "$MAVROS_PID"; do
        [[ -n "$pid" ]] || continue
        wait "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

printf 'Starting MAVROS on %s at %s baud.\n' "$FCU_DEVICE" "$FCU_BAUD"
ros2 launch mavros px4.launch \
    "fcu_url:=${FCU_DEVICE}:${FCU_BAUD}" \
    "gcs_url:=${GCS_URL}" \
    "tgt_system:=${FCU_SYSTEM_ID}" \
    "tgt_component:=${FCU_COMPONENT_ID}" \
    "namespace:=${MAVROS_NAMESPACE_ARGUMENT}" \
    >"$SESSION_DIR/mavros.log" 2>&1 &
MAVROS_PID=$!

connected=0
for _attempt in $(seq 1 20); do
    if ! kill -0 "$MAVROS_PID" 2>/dev/null; then
        tail -n 40 "$SESSION_DIR/mavros.log" >&2 || true
        fail 'MAVROS stopped before the FCU connected'
    fi
    state_snapshot="$(timeout 2 ros2 topic echo "$MAVROS_STATE_TOPIC" --once 2>/dev/null || true)"
    if grep -q 'connected: true' <<<"$state_snapshot"; then
        connected=1
        break
    fi
    sleep 1
done
[[ "$connected" -eq 1 ]] || fail "MAVROS did not report connected: true on $MAVROS_STATE_TOPIC; check cable, PX4 MAVLink port, and FCU_BAUD"

console_args=(
    "bind_address:=127.0.0.1"
    "port:=${DASHBOARD_PORT}"
    "mavros_state_topic:=${MAVROS_STATE_TOPIC}"
    "arm_service:=${MAVROS_NAMESPACE}/cmd/arming"
    "mode_service:=${MAVROS_NAMESPACE}/set_mode"
)

if [[ "$MODE" == 'controls' ]]; then
    if [[ -z "${FLIGHT_SAFETY_TOKEN:-}" ]]; then
        umask 077
        openssl rand -hex 32 >"$SESSION_DIR/control-token"
        export FLIGHT_SAFETY_TOKEN
        FLIGHT_SAFETY_TOKEN="$(<"$SESSION_DIR/control-token")"
    fi
    console_args+=(
        'enable_pilot_commands:=true'
        "pilot_audit_log:=${SESSION_DIR}/pilot_commands.jsonl"
    )
    printf 'Pilot commands are enabled with a session token stored at %s/control-token.\n' "$SESSION_DIR"
else
    console_args+=('enable_pilot_commands:=false')
    printf 'Read-only monitoring mode selected.\n'
fi

printf 'Starting locked safety console at http://127.0.0.1:%s\n' "$DASHBOARD_PORT"
ros2 launch planning_pkg real_uav_operator_console.launch.py "${console_args[@]}" \
    >"$SESSION_DIR/operator-console.log" 2>&1 &
CONSOLE_PID=$!

sleep 2
if ! kill -0 "$CONSOLE_PID" 2>/dev/null; then
    tail -n 40 "$SESSION_DIR/operator-console.log" >&2 || true
    fail 'operator console stopped during startup'
fi

printf 'Session directory: %s\n' "$SESSION_DIR"
printf 'MAVROS state: %s\n' "$MAVROS_STATE_TOPIC"
printf 'Press Ctrl-C here to stop MAVROS and the local console.\n'
wait "$CONSOLE_PID"
