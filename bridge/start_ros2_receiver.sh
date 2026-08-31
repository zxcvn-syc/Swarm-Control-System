#!/usr/bin/env bash
# Start the ROS 2 observation receiver. It never exposes MAVROS services.

set -eo pipefail

BRIDGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${ROS2_BRIDGE_CONFIG:-$BRIDGE_DIR/config/ros2_receiver.env}"

[[ -r "$CONFIG_PATH" ]] || {
    printf 'ROS 2 bridge configuration is not readable: %s\n' "$CONFIG_PATH" >&2
    exit 2
}
# shellcheck source=/dev/null
source "$CONFIG_PATH"

: "${BRIDGE_TOKEN:?BRIDGE_TOKEN is required}"
[[ "$BRIDGE_TOKEN" != "REPLACE_WITH_A_RANDOM_SHARED_TOKEN" ]] || {
    printf 'Set a random BRIDGE_TOKEN before starting the ROS 2 receiver.\n' >&2
    exit 2
}
export BRIDGE_TOKEN

source /opt/ros/humble/setup.bash
exec python3 "$BRIDGE_DIR/ros2_observation_receiver.py" --ros-args \
    -p host:="${BRIDGE_HOST:-192.168.144.60}" \
    -p port:="${BRIDGE_PORT:-19001}" \
    -p image_topic:="${IMAGE_TOPIC:-/camera/image}" \
    -p compressed_image_topic:="${COMPRESSED_IMAGE_TOPIC:-/camera/image/compressed}" \
    -p camera_info_topic:="${CAMERA_INFO_TOPIC:-/camera/camera_info}" \
    -p pose_topic:="${POSE_TOPIC:-/uav0/mavros/local_position/pose}" \
    -p battery_topic:="${BATTERY_TOPIC:-/uav0/mavros/battery}" \
    -p state_topic:="${STATE_TOPIC:-/uav0/mavros/state}" \
    -p transcode_raw_to_jpeg:="${TRANSCODE_RAW_TO_JPEG:-true}" \
    -p decode_compressed_to_raw:="${DECODE_COMPRESSED_TO_RAW:-true}" \
    -p jpeg_quality:="${JPEG_QUALITY:-80}"
