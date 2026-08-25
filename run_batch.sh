#!/usr/bin/env bash
# 三场景封控批量测试 runner —— 不依赖 `timeout` 杀 ros2 launch（该环境 SIGTERM 不生效），
# 改为后台启动 + 固定等待判定完成 + 显式 pkill 按节点名清场，保证每轮必推进。
# 注意：不要加 `set -u`，ROS 的 setup.bash 会引用未声明变量（AMENT_TRACE_SETUP_FILES 等）。

REPO=/mnt/c/ProgramData/WorkBuddy/chromium-env/6ulcsx/WorkBuddy/2026-08-10-14-52-53/Swarm-Control-System
cd "$REPO" || exit 1
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

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
  echo "[$(date +%T)] >>> scene=$scene start"
  cleanup; sleep 1
  ros2 launch containment_pkg escape_eval.launch.py \
      scene:=$scene direction:=-1 result_csv:=$CSV \
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
