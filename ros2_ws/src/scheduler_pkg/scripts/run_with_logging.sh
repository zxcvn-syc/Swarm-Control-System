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

# Prefer the installed entry point; fall back to python3 -m when colcon did
# not regenerate it (ros2 run raises StopIteration on a stale install space).
# Reported by Ma Ziyue 2026-08-26; fallback keeps 8.27 collection unblocked
# regardless of install-space state on the operator's machine.
if ros2 pkg executables scheduler_pkg 2>/dev/null | grep -qw reallocation_collector; then
    LAUNCH_MODE="ros2run"
    LAUNCH_CMD="ros2 run scheduler_pkg reallocation_collector"
else
    LAUNCH_MODE="python -m (entry point not found in install space)"
    LAUNCH_CMD="python3 -m scheduler_pkg.reallocation_collector"
    echo "[WARN] reallocation_collector 入口未注册到 ros2run（install 空间可能未刷新），" | tee -a "$LOG_FILE"
    echo "[WARN] 自动降级为 python3 -m 启动，功能等价。" | tee -a "$LOG_FILE"
fi
echo "启动方式: $LAUNCH_MODE" | tee -a "$LOG_FILE"

$LAUNCH_CMD --ros-args -p log_dir:="$LOG_DIR" 2>&1 | tee -a "$LOG_FILE"
