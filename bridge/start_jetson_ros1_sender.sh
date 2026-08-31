#!/usr/bin/env bash
# Start the Jetson ROS 1 observation sender. It never sends control traffic.

set -eo pipefail

BRIDGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${JETSON_BRIDGE_CONFIG:-$BRIDGE_DIR/config/jetson_ros1_sender.env}"

[[ -r "$CONFIG_PATH" ]] || {
    printf 'Jetson bridge configuration is not readable: %s\n' "$CONFIG_PATH" >&2
    exit 2
}
# shellcheck source=/dev/null
source "$CONFIG_PATH"

: "${BRIDGE_TOKEN:?BRIDGE_TOKEN is required}"
[[ "$BRIDGE_TOKEN" != "REPLACE_WITH_A_RANDOM_SHARED_TOKEN" ]] || {
    printf 'Set a random BRIDGE_TOKEN before starting the Jetson sender.\n' >&2
    exit 2
}
export BRIDGE_TOKEN

ROS1_SETUP_PATH="${ROS1_SETUP:-/opt/ros/melodic/setup.bash}"
[[ -r "$ROS1_SETUP_PATH" ]] || {
    printf 'ROS 1 setup file is not readable: %s\n' "$ROS1_SETUP_PATH" >&2
    exit 2
}
source "$ROS1_SETUP_PATH"
if [[ -n "${ROS1_OVERLAY_SETUP:-}" ]]; then
    [[ -r "$ROS1_OVERLAY_SETUP" ]] || {
        printf 'ROS 1 overlay setup file is not readable: %s\n' "$ROS1_OVERLAY_SETUP" >&2
        exit 2
    }
    source "$ROS1_OVERLAY_SETUP"
fi
exec python "$BRIDGE_DIR/ros1_observation_sender.py" \
    _bind_host:="${BRIDGE_BIND_HOST:-0.0.0.0}" \
    _port:="${BRIDGE_PORT:-19001}" \
    _image_topic:="${IMAGE_TOPIC:-/camera/color/image_raw}" \
    _compressed_image_topic:="${COMPRESSED_IMAGE_TOPIC:-/camera/color/image_raw/compressed}" \
    _camera_info_topic:="${CAMERA_INFO_TOPIC:-/camera/color/camera_info}" \
    _pose_topic:="${POSE_TOPIC:-/mavros/local_position/pose}" \
    _battery_topic:="${BATTERY_TOPIC:-/mavros/battery}" \
    _state_topic:="${STATE_TOPIC:-/mavros/state}" \
    _max_image_hz:="${MAX_IMAGE_HZ:-12}"
