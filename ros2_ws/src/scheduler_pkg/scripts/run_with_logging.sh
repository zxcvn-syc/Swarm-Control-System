#!/bin/bash
# Reallocation timing collector launcher (Ma Ziyue, 2026-08-26)
#
# Usage:
#   ./run_with_logging.sh [ROS2_WS_PATH]
#   Default: ~/ros2_ws
#   Example: ./run_with_logging.sh ~/Swarm-Control-System/ros2_ws
#
# Outputs (both under $ROS2_WS/logs/reallocation/):
#   reallocation_<ts>.log           full ros2 run stdout (tee)
#   reallocation_events_<ts>.csv    per-event timing rows
#     (wall_time, ros_time, frame_idx, num_targets, elapsed_ms, allocation)
#
# Run this BEFORE starting the enclosure test launch, keep it running
# during all trials, then stop with Ctrl+C and archive the log dir.

set -u

ROS2_WS=${1:-$HOME/ros2_ws}
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR="$ROS2_WS/logs/reallocation"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/reallocation_$TIMESTAMP.log"

# Source ROS2 distro first (harmless if already sourced via .bashrc)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

echo "========================================" | tee "$LOG_FILE"
echo "重分配采集节点启动时间: $(date)" | tee -a "$LOG_FILE"
echo "ROS2 工作空间: $ROS2_WS" | tee -a "$LOG_FILE"
echo "日志文件: $LOG_FILE" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"

cd "$ROS2_WS" || exit 1
source install/setup.bash
ros2 run scheduler_pkg reallocation_collector \
    --ros-args -p log_dir:="$LOG_DIR" 2>&1 | tee -a "$LOG_FILE"
