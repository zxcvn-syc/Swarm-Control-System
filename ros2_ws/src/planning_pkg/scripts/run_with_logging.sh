#!/bin/bash
# D*Lite 重规划时延采集启动脚本（8.27 三场景封控测试配套）
# 参照 scheduler_pkg/scripts/run_with_logging.sh（马子越，2026-08-26）模式。
#
# 用途：在跑 security/border 等封控场景时，把 planner_node 的 [REPLAN]
#       重规划事件日志实时落盘，并抽成 CSV，供 8.27"目标逃逸+重规划追击"
#       验证报告使用（补充任务单 L76 + L138 风险项）。
#
# 用法:
#   ./run_with_logging.sh [scene] [ROS2_WS_PATH]
#     scene   : park | security | border       （默认 security）
#     ROS2_WS : 工作空间根目录                  （默认 $HOME/ros2_ws）
#   示例:
#     ./run_with_logging.sh security ~/Swarm-Control-System/ros2_ws
#
# 输出（均在 $ROS2_WS/logs/planning/ 下）:
#   replan_<scene>_<ts>.log           完整 ros2 launch 输出（tee）
#   replan_events_<scene>_<ts>.csv    逐次重规划事件
#     (wall,ros,drone_id,reason,plan_ms,path_len,planner)
#
# 保持运行直到场景播完（escape_test_node 到 test_duration 后停止发布），
# 收集完成后 Ctrl+C 终止并归档 logs/planning 目录。
#
# 注意：不要用 `set -u` —— /opt/ros/humble/setup.bash 与 local_setup.sh
#       会读取未定义变量（如 AMENT_TRACE_SETUP_FILES / AMENT_PYTHON_EXECUTABLE），
#       在全新终端里会导致整个脚本中断（2026-08-26 于 Ubuntu 22.04 + Humble 复现）。

SCENE="${1:-security}"
ROS2_WS="${2:-$HOME/ros2_ws}"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR="$ROS2_WS/logs/planning"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/replan_${SCENE}_${TIMESTAMP}.log"
CSV_FILE="$LOG_DIR/replan_events_${SCENE}_${TIMESTAMP}.csv"

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

echo "========================================" | tee "$LOG_FILE"
echo "重规划采集启动时间: $(date)" | tee -a "$LOG_FILE"
echo "场景: $SCENE" | tee -a "$LOG_FILE"
echo "ROS2 工作空间: $ROS2_WS" | tee -a "$LOG_FILE"
echo "日志文件: $LOG_FILE" | tee -a "$LOG_FILE"
echo "事件 CSV: $CSV_FILE" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "wall,ros,drone_id,reason,plan_ms,path_len,planner" > "$CSV_FILE"

cd "$ROS2_WS" || exit 1
source install/setup.bash

# 启动重规划场景（mock_platform_pub + escape_test_node + planner_node），
# 完整输出 tee 到 LOG_FILE，并实时把 [REPLAN] 行抽成 CSV。
ros2 launch planning_pkg replan_eval.launch.py \
    scene:="$SCENE" 2>&1 | tee -a "$LOG_FILE" | \
    while IFS= read -r line; do
        case "$line" in
            *"[REPLAN]"*)
                wall=$(echo "$line" | grep -oE 'wall=[^ ]+' | cut -d= -f2)
                ros=$(echo "$line" | grep -oE 'ros=[^ ]+' | cut -d= -f2)
                did=$(echo "$line" | grep -oE 'drone_id=[^ ]+' | cut -d= -f2)
                reason=$(echo "$line" | grep -oE 'reason=[^ ]+' | cut -d= -f2)
                pms=$(echo "$line" | grep -oE 'plan_ms=[^ ]+' | cut -d= -f2)
                plen=$(echo "$line" | grep -oE 'path_len=[^ ]+' | cut -d= -f2)
                planner=$(echo "$line" | grep -oE 'planner=[^ ]+' | cut -d= -f2)
                echo "$wall,$ros,$did,$reason,$pms,$plen,$planner" | tee -a "$CSV_FILE"
                ;;
        esac
    done
