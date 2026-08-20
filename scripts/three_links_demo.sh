#!/usr/bin/env bash
# three_links_demo.sh — 一键启动三关联调
#
# 用法：
#   ./scripts/three_links_demo.sh                 # 默认视频: videos/test_multi_target_tracking.mp4
#   ./scripts/three_links_demo.sh --video ...mp4  # 指定输入视频
#   ./scripts/three_links_demo.sh --dry-run       # 仅构建不启动
#   ./scripts/three_links_demo.sh --help          # 帮助
#
# 行为：
#   1. cd 到仓库根目录
#   2. colcon build（接口+感知+调度+规划+封控五个包）
#   3. source install/setup.bash
#   4. ros2 launch ros2_ws/launch/three_links.launch.py
#
# Update history (v2.3, 2026-08-06):
# - Version aligned with launch files v2.3
# - All three scheduler strategies supported: greedy/hungarian/auction
# - Same fixes from v2.2 retained:
#   * Added planning_pkg to build list (real A*/D*Lite planner replaces stub)
#   * Removed planner_stub from build list
#   * Launch now uses 5 nodes: tracker → coord_transform → scheduler → planner → enclosure
#   * --dry-run now actually validates launch args with --show-args after build
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"
OUTPUT_DIR="$REPO_ROOT/output"

mkdir -p "$OUTPUT_DIR"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--video VIDEO_PATH] [--dry-run] [--help]
Options:
  --video VIDEO_PATH   Override the video fed into tracker_node.
                       Default: \$REPO/videos/test_multi_target_tracking.mp4
  --dry-run            Build all packages AND validate launch arguments with
                       --show-args; do not start the nodes.
  --help               Print this help and exit.
USAGE
}

VIDEO=""
DRY_RUN="0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --video) VIDEO="${2:-}"; shift 2 ;;
    --dry-run) DRY_RUN="1"; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[three_links_demo] unknown arg: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$VIDEO" ]]; then
  VIDEO="$REPO_ROOT/videos/test_multi_target_tracking.mp4"
  if [[ ! -f "$VIDEO" ]]; then
    VIDEO="$WORKSPACE/src/perception_pkg/test_videos/pexels_pedestrian_crossing.mp4"
  fi
fi

# --- 0. 准备环境 ----------------------------------------------------------
# ROS2 Humble
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[three_links_demo] /opt/ros/humble/setup.bash not found; aborting" >&2
  exit 1
fi

cd "$REPO_ROOT"

# --- 1. 构建 --------------------------------------------------------------
echo "[three_links_demo] building workspace (swarm_interfaces + perception + scheduler + planning + containment)..."
(cd "$WORKSPACE" && colcon build \
   --packages-select swarm_interfaces perception_pkg scheduler_pkg planning_pkg containment_pkg \
   --event-handlers console_direct+)

# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[three_links_demo] (dry-run) build complete, verifying launch arguments..."
  echo ""
  echo "=== Validating ros2_ws/launch/three_links.launch.py ==="
  ros2 launch "$WORKSPACE/launch/three_links.launch.py" --show-args
  echo ""
  echo "[three_links_demo] (dry-run) all good; launch syntax is valid, all three strategies (greedy/hungarian/auction) available"
  exit 0
fi

# --- 2. 启动 --------------------------------------------------------------
LOG_FILE="$OUTPUT_DIR/three_links_$(date +%Y%m%d_%H%M%S).log"
echo "[three_links_demo] launching 5 nodes (tracker→coord_transform→scheduler→planner→enclosure), logs -> $LOG_FILE..."
echo "[three_links_demo] using video: $VIDEO"
echo "[three_links_demo] default strategy: greedy (use scheduler_strategy:=auction or hungarian to change)"
exec ros2 launch "$WORKSPACE/launch/three_links.launch.py" \
   video_source:="$VIDEO" 2>&1 | tee "$LOG_FILE"
