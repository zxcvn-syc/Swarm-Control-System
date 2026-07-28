#!/usr/bin/env bash
# record_three_links.sh — 启动三关 + 录屏 + 集成 watchdog
#
# 用法：
#   ./scripts/record_three_links.sh                                  # 默认 mp4 输出
#   ./scripts/record_three_links.sh --out videos/three_links_demo.mp4
#   ./scripts/record_three_links.sh --window 30                      # 30s 录屏窗口
#   ./scripts/record_three_links.sh --mode ffmpeg|ros2bag|pseudo     # 录屏方式
#       ffmpeg  : 用 ffmpeg 录 X11 / screen（需要桌面会话）
#       ros2bag : 用 ros2 bag record（推荐，C1 环境友好）
#       pseudo  : 伪录屏 → 仅写 tee 日志到 output/*.log（**默认**，无 ROS2 GUI 的环境）
#
# 默认是 pseudo：环境里通常没有 ROS2 GUI / X11 / ffmpeg 桌面录制，因此默认只产
# 出命令行日志与 ROS2 bag 到 video_output。同名 .mp4 是占位，并不真的是视频。
#
# 在有 GUI 的机器上请显式 --mode ffmpeg。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"
OUTPUT_DIR="$REPO_ROOT/output"
mkdir -p "$OUTPUT_DIR"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Options:
  --video FILE          Override tracker_node input video.
  --out FILE            Output recording path.  Default: videos/three_links_<DATE>.mp4
                        If --mode=pseudo, also writes the command log alongside.
  --window SEC          Test window seconds.  Default: 12.
  --mode MODE           ffmpeg | ros2bag | pseudo.  Default: pseudo.
  --help                Print this help and exit.
USAGE
}

VIDEO=""
OUTFILE=""
WINDOW="12"
MODE="pseudo"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --video) VIDEO="${2:-}"; shift 2 ;;
    --out) OUTFILE="${2:-}"; shift 2 ;;
    --window) WINDOW="${2:-}"; shift 2 ;;
    --mode) MODE="${2:-}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[record_three_links] unknown arg: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$VIDEO" ]]; then
  VIDEO="$REPO_ROOT/videos/test_multi_target_tracking.mp4"
fi
DATE_STR="$(date +%Y%m%d_%H%M%S)"
if [[ -z "$OUTFILE" ]]; then
  OUTFILE="$REPO_ROOT/videos/three_links_${DATE_STR}.mp4"
fi
LOG_FILE="$OUTPUT_DIR/three_links_${DATE_STR}.log"

# --- 准备 ----------------------------------------------------------------
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
cd "$REPO_ROOT"
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"

echo "[record_three_links] mode=$MODE out=$OUTFILE window=${WINDOW}s log=$LOG_FILE"

# --- 主录制 --------------------------------------------------------------
case "$MODE" in
  pseudo)
    # 伪录屏：tee 整个 launch + 集成测试 输出到 .log；同时运行
    # test_three_links.py 写出 JSON 报告作为成功/失败凭证。
    echo "[record_three_links] pseudo mode: writing $LOG_FILE"
    (
      ros2 launch "$WORKSPACE/launch/integration_test.launch.py" \
        video_source:="$VIDEO" \
        window_sec:="$WINDOW" \
        report_path:="$OUTPUT_DIR/integration_test_${DATE_STR}.json"
    ) 2>&1 | tee "$LOG_FILE"
    # 占位 mp4（仅文件存在性凭证）
    if [[ ! -f "$OUTFILE" ]]; then
      echo "[record_three_links] (pseudo) creating placeholder $OUTFILE"
      mkdir -p "$(dirname "$OUTFILE")"
      echo "# Placeholder; pseudo-mode recording.  See $LOG_FILE and integration_test_${DATE_STR}.json" > "$OUTFILE"
    fi
    echo "[record_three_links] DONE -> $LOG_FILE and $OUTFILE"
    ;;

  ros2bag)
    # ros2bag 录制 6 个核心 topic
    BAG_DIR="$OUTPUT_DIR/three_links_bag_${DATE_STR}"
    mkdir -p "$BAG_DIR"
    LOG_FILE="$LOG_FILE" \
      ros2 launch "$WORKSPACE/launch/integration_test.launch.py" \
        video_source:="$VIDEO" \
        window_sec:="$WINDOW" \
        report_path:="$OUTPUT_DIR/integration_test_${DATE_STR}.json" \
        &
    LAUNCH_PID=$!
    sleep 3   # give nodes time to register
    ros2 bag record -o "$BAG_DIR" \
      /target_track \
      /enclosure_targets \
      /task_assignment \
      /drone_states \
      /enclosure_command \
      /tracking_metrics &
    BAG_PID=$!
    wait $LAUNCH_PID
    kill -INT $BAG_PID 2>/dev/null || true
    # 写占位 mp4
    if [[ ! -f "$OUTFILE" ]]; then
      mkdir -p "$(dirname "$OUTFILE")"
      echo "# ros2 bag recording at $BAG_DIR" > "$OUTFILE"
    fi
    echo "[record_three_links] DONE -> bag: $BAG_DIR  log: $LOG_FILE"
    ;;

  ffmpeg)
    # 真 GUI 录制。需要 X11 / 桌面会话 + ffmpeg。
    if ! command -v ffmpeg >/dev/null; then
      echo "[record_three_links] ffmpeg not found; install or use --mode ros2bag/pseudo" >&2
      exit 1
    fi
    ffmpeg -video_size 1280x720 -framerate 25 -f x11grab -i :0.0 \
           "$OUTFILE" &
    FFMPEG_PID=$!
    ros2 launch "$WORKSPACE/launch/integration_test.launch.py" \
        video_source:="$VIDEO" \
        window_sec:="$WINDOW" \
        report_path:="$OUTPUT_DIR/integration_test_${DATE_STR}.json" \
        2>&1 | tee -a "$LOG_FILE"
    sleep 2
    kill -INT $FFMPEG_PID 2>/dev/null || true
    echo "[record_three_links] DONE -> $OUTFILE"
    ;;

  *)
    echo "[record_three_links] unknown mode: $MODE" >&2
    usage
    exit 1
    ;;
esac
