#!/usr/bin/env bash
# Publish a detected UVC camera into ROS 1. It has no flight-control access.

set -eo pipefail

BRIDGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${JETSON_BRIDGE_CONFIG:-$BRIDGE_DIR/config/jetson_ros1_sender.env}"

[[ -r "$CONFIG_PATH" ]] || {
    printf 'Jetson bridge configuration is not readable: %s\n' "$CONFIG_PATH" >&2
    exit 2
}
# shellcheck source=/dev/null
source "$CONFIG_PATH"

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

exec python "$BRIDGE_DIR/ros1_uvc_camera.py" \
    _device_index:="${CAMERA_DEVICE_INDEX:-0}" \
    _width:="${CAMERA_WIDTH:-1280}" \
    _height:="${CAMERA_HEIGHT:-720}" \
    _fps:="${CAMERA_FPS:-12}" \
    _jpeg_quality:="${CAMERA_JPEG_QUALITY:-80}" \
    _frame_id:="${CAMERA_FRAME_ID:-camera_color_optical_frame}" \
    _compressed_image_topic:="${COMPRESSED_IMAGE_TOPIC:-/camera/color/image_raw/compressed}"
