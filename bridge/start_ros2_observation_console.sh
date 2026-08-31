#!/usr/bin/env bash
# Start the bridge receiver and local read-only browser monitor together.

set -eo pipefail

BRIDGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$BRIDGE_DIR/.." && pwd)"
WORKSPACE_DIR="$REPO_DIR/ros2_ws"
CONFIG_PATH="${ROS2_BRIDGE_CONFIG:-$BRIDGE_DIR/config/ros2_receiver.env}"

[[ -r "$CONFIG_PATH" ]] || {
    printf 'ROS 2 bridge configuration is not readable: %s\n' "$CONFIG_PATH" >&2
    exit 2
}
# shellcheck source=/dev/null
source "$CONFIG_PATH"

: "${BRIDGE_TOKEN:?BRIDGE_TOKEN is required}"
: "${ROS_DOMAIN_ID:=71}"
: "${DASHBOARD_PORT:=18080}"
[[ "$BRIDGE_TOKEN" != "REPLACE_WITH_A_RANDOM_SHARED_TOKEN" ]] || {
    printf 'Set a random BRIDGE_TOKEN before starting the observation console.\n' >&2
    exit 2
}
[[ "$DASHBOARD_PORT" =~ ^[0-9]+$ && "$DASHBOARD_PORT" -ge 1024 && "$DASHBOARD_PORT" -le 65535 ]] || {
    printf 'DASHBOARD_PORT must be between 1024 and 65535.\n' >&2
    exit 2
}
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ && "$ROS_DOMAIN_ID" -le 232 ]] || {
    printf 'ROS_DOMAIN_ID must be between 0 and 232.\n' >&2
    exit 2
}
[[ -r /opt/ros/humble/setup.bash ]] || {
    printf 'ROS 2 Humble is not available at /opt/ros/humble.\n' >&2
    exit 2
}
[[ -r "$WORKSPACE_DIR/install/setup.bash" ]] || {
    printf 'ROS workspace is not built: %s\n' "$WORKSPACE_DIR/install" >&2
    exit 2
}

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
export ROS_DOMAIN_ID

if ss -ltn "sport = :$DASHBOARD_PORT" | grep -q LISTEN; then
    printf 'Dashboard port is already in use: %s\n' "$DASHBOARD_PORT" >&2
    exit 2
fi

SESSION_DIR="${OBSERVATION_SESSION_LOG_ROOT:-$HOME/flight_evidence}/$(date +%Y%m%d)/observation-$(date +%H%M%S)"
mkdir -p "$SESSION_DIR"
chmod 700 "$SESSION_DIR"
RECEIVER_PID=""
IMAGE_DECODER_PID=""

cleanup() {
    if [[ -n "$IMAGE_DECODER_PID" ]]; then
        kill -INT "$IMAGE_DECODER_PID" 2>/dev/null || true
        wait "$IMAGE_DECODER_PID" 2>/dev/null || true
    fi
    if [[ -n "$RECEIVER_PID" ]]; then
        kill -INT "$RECEIVER_PID" 2>/dev/null || true
        wait "$RECEIVER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

"$BRIDGE_DIR/start_ros2_receiver.sh" >"$SESSION_DIR/bridge-receiver.log" 2>&1 &
RECEIVER_PID=$!
sleep 1
if ! kill -0 "$RECEIVER_PID" 2>/dev/null; then
    tail -n 40 "$SESSION_DIR/bridge-receiver.log" >&2 || true
    printf 'ROS 2 observation receiver stopped during startup.\n' >&2
    exit 2
fi

if [[ "${USE_IMAGE_TRANSPORT_DECODER:-false}" == "true" ]]; then
    if [[ "${DECODE_COMPRESSED_TO_RAW:-true}" != "false" ]]; then
        printf 'Set DECODE_COMPRESSED_TO_RAW=false when USE_IMAGE_TRANSPORT_DECODER=true.\n' >&2
        exit 2
    fi
    IMAGE_TRANSPORT_PREFIX="$(ros2 pkg prefix image_transport 2>/dev/null || true)"
    IMAGE_REPUBLISH="$IMAGE_TRANSPORT_PREFIX/lib/image_transport/republish"
    [[ -x "$IMAGE_REPUBLISH" ]] || {
        printf 'image_transport republish is required for the C++ JPEG decoder.\n' >&2
        exit 2
    }
    ros2 pkg prefix compressed_image_transport >/dev/null 2>&1 || {
        printf 'compressed_image_transport is required for the C++ JPEG decoder.\n' >&2
        exit 2
    }
    "$IMAGE_REPUBLISH" compressed raw --ros-args \
        -r "in/compressed:=${COMPRESSED_IMAGE_TOPIC:-/camera/image/compressed}" \
        -r "out:=${IMAGE_TOPIC:-/camera/image}" \
        >"$SESSION_DIR/image-decoder.log" 2>&1 &
    IMAGE_DECODER_PID=$!
    sleep 1
    if ! kill -0 "$IMAGE_DECODER_PID" 2>/dev/null; then
        tail -n 40 "$SESSION_DIR/image-decoder.log" >&2 || true
        printf 'image_transport JPEG decoder stopped during startup.\n' >&2
        exit 2
    fi
fi

printf 'Read-only bridge monitor: http://127.0.0.1:%s\n' "$DASHBOARD_PORT"
printf 'Session logs: %s\n' "$SESSION_DIR"
printf 'No MAVROS control services, setpoints, arming, or mode changes are exposed.\n'

ros2 run planning_pkg flight_safety_dashboard --ros-args \
    -p bind_address:=127.0.0.1 \
    -p port:="$DASHBOARD_PORT" \
    -p enable_pilot_commands:=false \
    -p video_topic:="${COMPRESSED_IMAGE_TOPIC:-/camera/image/compressed}" \
    -p mavros_state_topic:="${STATE_TOPIC:-/uav0/mavros/state}"
