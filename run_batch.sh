#!/usr/bin/env bash
# 三场景封控批量测试 runner —— 不依赖 `timeout` 杀 ros2 launch（该环境 SIGTERM 不生效），
# 改为后台启动 + 固定等待判定完成 + 显式 pkill 按节点名清场，保证每轮必推进。
#
# 相对路径：用脚本自身所在目录定位仓库根，其他人克隆到任意位置都能跑。
# 注意：不要加 `set -u`，ROS 的 setup.bash 会引用未声明变量（AMENT_TRACE_SETUP_FILES 等）。
#
# 用法（在仓库根目录，或任意位置均可）：
#   bash run_batch.sh              # 默认：scripted 轨迹 + 响应证据门槛，60 次
#   CLOSED_LOOP=1 bash run_batch.sh   # 真闭环模式：目标被拦截才折返
#
# 改 intercept_radius： INTERCEPT_R=5.0 bash run_batch.sh

# 仓库根 = 本脚本所在目录（处理符号链接）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

INTERCEPT_R="${INTERCEPT_R:-5.0}"
CLOSED_LOOP="${CLOSED_LOOP:-false}"

CSV=$PWD/docs/evidence/eval_results_$(date +%F).csv
mkdir -p docs/evidence

cleanup() {
  pkill -9 -f 'mock_platform_pub'      2>/dev/null
  pkill -9 -f 'escape_test_node'       2>/dev/null
  pkill -9 -f 'enclosure_node'         2>/dev/null
  pkill -9 -f 'containment_evaluator'  2>/dev/null
  pkill -9 -f 'escape_eval.launch.py'  2>/dev/null
}

run_one() {
  local scene=$1
  echo "[$(date +%T)] >>> scene=$scene start (intercept=${INTERCEPT_R} closed_loop=${CLOSED_LOOP})"
  cleanup; sleep 1
  ros2 launch containment_pkg escape_eval.launch.py \
      scene:=$scene direction:=-1 result_csv:=$CSV \
      intercept_radius:=$INTERCEPT_R closed_loop:=$CLOSED_LOOP \
      >/tmp/escape_${scene}.log 2>&1 &
  local lp=$!
  sleep 40                       # 判定最晚在 ~30s 完成，留足余量
  kill -9 "$lp" 2>/dev/null      # 强行结束 launch 进程
  cleanup                        # 清掉常驻节点，避免下一轮名字/话题冲突
  sleep 2
  echo "[$(date +%T)] <<< scene=$scene done  (csv lines: $(wc -l < "$CSV" 2>/dev/null || echo 0))"
}

for s in park security border; do
  for i in $(seq 1 20); do
    run_one "$s"
  done
done

echo "DONE -> $CSV"
python3 ros2_ws/src/containment_pkg/analyze_results.py "$CSV" --markdown
