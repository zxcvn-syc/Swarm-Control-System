#!/usr/bin/env bash
# 三场景封控 SITL 批量测试 runner（8.27 何泓林 30 次 / 可扩展）。
#
# 与 run_batch.sh 的差异：
#   1. 使用 escape_eval_sitl.launch.py（平台位姿来自外部 SITL 的 /drone_states，
#      不再用 mock_platform_pub，默认也不起 platform_state_merger）
#   2. 只清理测试节点，**不杀 SITL/PX4/Gazebo**（要求外部先起好 SITL 环境）
#
# 用法（在仓库根目录）：
#   bash run_batch_sitl.sh                    # 默认 10 次/场景，共 30 次
#   RUNS_PER_SCENE=20 bash run_batch_sitl.sh  # 60 次，与 mock 批量一致
#
# 前置条件（必须已在其他终端启动）：
#   ./simulation/px4_sitl_3uav/start_3uav_sitl.sh
#   bash simulation/px4_sitl_3uav/start_sitl_platform.sh
#   ros2 topic echo /drone_states --once  # 确认 5 个平台位姿已进来

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

INTERCEPT_R="${INTERCEPT_R:-5.0}"
CLOSED_LOOP="${CLOSED_LOOP:-false}"
RUNS_PER_SCENE="${RUNS_PER_SCENE:-10}"

CSV=$PWD/docs/evidence/eval_results_sitl_$(date +%F).csv
mkdir -p docs/evidence

# 只杀本次 launch 拉起的测试节点；**绝不**动 SITL/PX4/Gazebo。
cleanup() {
  pkill -9 -f 'platform_state_merger'  2>/dev/null
  pkill -9 -f 'escape_test_node'       2>/dev/null
  pkill -9 -f 'enclosure_node'         2>/dev/null
  pkill -9 -f 'containment_evaluator'  2>/dev/null
  pkill -9 -f 'escape_eval_sitl.launch.py' 2>/dev/null
}

run_one() {
  local scene=$1
  echo "[$(date +%T)] >>> scene=$scene start SITL (intercept=${INTERCEPT_R} closed_loop=${CLOSED_LOOP})"
  cleanup; sleep 1
  ros2 launch containment_pkg escape_eval_sitl.launch.py \
      scene:=$scene direction:=-1 result_csv:=$CSV \
      intercept_radius:=$INTERCEPT_R closed_loop:=$CLOSED_LOOP \
      >/tmp/escape_sitl_${scene}.log 2>&1 &
  local lp=$!
  sleep 40                       # 判定最晚在 ~30s 完成，留足余量
  kill -9 "$lp" 2>/dev/null      # 强行结束 launch 进程
  cleanup                        # 清掉常驻测试节点，避免下一轮冲突
  sleep 2
  echo "[$(date +%T)] <<< scene=$scene done  (csv lines: $(wc -l < "$CSV" 2>/dev/null || echo 0))"
}

for s in park security border; do
  for i in $(seq 1 "$RUNS_PER_SCENE"); do
    run_one "$s"
  done
done

echo "DONE -> $CSV"
python3 ros2_ws/src/containment_pkg/analyze_results.py "$CSV" --markdown
