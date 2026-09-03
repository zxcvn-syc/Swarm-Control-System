#!/bin/bash
# ============================================================================
# SCS 车侧四关自动测试 — 主机安全壳 (pi 用户家目录, 已部署: ~/scs_autotest.sh)
# ----------------------------------------------------------------------------
# 用法:
#   bash ~/scs_autotest.sh               # 只打印说明, 什么都不做(默认安全)
#   bash ~/scs_autotest.sh --dry-run     # 预览各关规划(不碰 ROS, 车不会动)
#   bash ~/scs_autotest.sh --run straight|turn|retarget|estop|full   # 正式执行
#
# 安全设计:
#   * 没有自启/cron/watchdog —— 只有人敲 --run 才会动, 传上去是"死"的。
#   * --run 前自动检查: 电池电压>=7500mV(否则拒跑) + follower 在运行 + 5秒倒计时(可 Ctrl+C)。
#   * 每关由驱动内置超时自动急停; Ctrl+C 杀掉驱动后, follower 的 path_timeout(2s)
#     会自动零速停车 —— 这是关键安全兜底, 无需额外命令。
# 前提: 已跑 ~/scs_start.sh 开场(或整车栈已就绪), 车头朝正东。
# ============================================================================
set -u
RUN=""
DRY=0

for arg in "$@"; do
  case "$arg" in
    --run) RUN_NEXT=1 ;;
    --dry-run) DRY=1 ;;
    straight|turn|retarget|estop|full)
      [ "${RUN_NEXT:-0}" = "1" ] && RUN="$arg" && RUN_NEXT=0 ;;
    *) ;;
  esac
done

# 实测(9.2): 电池 7315mV 时电机已被低压保护切断(车完全不动), 7200 拒跑线形同虚设。
# 提高到 7500mV 留足余量 —— 大电流下电压还会骤降。
BAT_MIN_MV=7500

# Ctrl+C: driver 被杀后不再发布路径, follower 的 path_timeout(2s)会自动零速停车,
# 无需额外命令 —— 这是本设计的关键安全兜底。
trap 'echo "[scs_autotest] 已取消(Ctrl+C)。follower 将在2秒路径超时后自动零速停车。"; exit 130' INT TERM

show_usage() {
  cat <<'EOF'
SCS 车侧四关自动测试 — 安全壳
--------------------------------
用法:
  bash ~/scs_autotest.sh                     # 只显示本说明, 不执行任何动作
  bash ~/scs_autotest.sh --dry-run           # 预览各关规划(车不会动)
  bash ~/scs_autotest.sh --run straight      # 第1关 直线1.5m
  bash ~/scs_autotest.sh --run turn          # 第2关 90度转弯(L形)
  bash ~/scs_autotest.sh --run retarget      # 第3关 动态换目标
  bash ~/scs_autotest.sh --run estop         # 第4关 急停
  bash ~/scs_autotest.sh --run full          # 连跑四关

前提(必须满足):
  1) 已跑 bash ~/scs_start.sh 开场(厂商栈+relay 就绪, /odom 与 /ugv_pose 约30Hz)
  2) ugv_path_follower 在运行(缺则本壳自动拉起)
  3) 车头朝正东摆好, 各关方向净空足够, 地面平整
  4) 电池 >= 7500mV

安全: 默认什么都不做; --run 前有电池检查+5秒倒计时, 可 Ctrl+C 取消;
      每关驱动内置超时自动急停。充电中/架空/轮子未离地时请勿 --run。
EOF
}

[ "$DRY" = "1" ] && [ -z "$RUN" ] && RUN="full"

if [ -z "$RUN" ]; then
  show_usage
  exit 0
fi

# ------------------------------------------------- dry-run: 直接预览, 零风险
if [ "$DRY" = "1" ]; then
  echo "[scs_autotest] dry-run 预览 (不初始化 ROS、不发布话题, 车不会动):"
  echo
  docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -lc \
    "source ~/.zshrc 2>/dev/null; python3 /home/ubuntu/scs_autotest_driver.py --leg $RUN --dry-run"
  echo
  echo "[scs_autotest] dry-run 结束。正式执行: bash ~/scs_autotest.sh --run $RUN"
  exit 0
fi

# ------------------------------------------------------------------ 电池检查
battery_mv() {
  docker exec MentorPi bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | grep -m1 "data:"' 2>/dev/null | awk '{print $2}' | tr -d '\r'
}

echo "[scs_autotest] 关卡: $RUN"
echo "[scs_autotest] 电池检查中..."
BAT=$(battery_mv)
if [ -z "$BAT" ]; then
  echo "[scs_autotest] ✗ 读不到电池电压(控制器/串口异常?) —— 中止。请先 bash ~/scs_status.sh 看体检。"
  exit 1
fi
if [ "$BAT" -lt "$BAT_MIN_MV" ] 2>/dev/null; then
  echo "[scs_autotest] ✗ 电池 ${BAT}mV < ${BAT_MIN_MV}mV, 电量不足拒跑 —— 先充电。"
  exit 1
fi
echo "[scs_autotest] 电池 ${BAT}mV OK"

# --------------------------------------------------------- follower 就绪检查
FOLLOWER=$(docker exec MentorPi bash -lc 'ps -eo cmd | grep -c "[u]gv_path_follower"' 2>/dev/null | tr -d '\r ')
if [ "${FOLLOWER:-0}" = "0" ]; then
  echo "[scs_autotest] follower 未运行, 自动拉起..."
  # 注意: 日志写 /home/ubuntu/ 下(root 的 /tmp/follower.log 会让 ubuntu 重定向失败, 命令静默不执行)
  # bash 显式 source(不能 source zsh 的 ~/.zshrc, bash 解析 zsh 语法会中断) + 容器内 nohup 后台(比 docker exec -d 可靠)
  docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && nohup ros2 run ugv_base_driver ugv_path_follower > /home/ubuntu/follower.log 2>&1 &'
  sleep 6
fi

# ------------------------------------------------------------------ 倒计时
echo "[scs_autotest] ⚠ 5 秒后开始执行 $RUN —— 车会动! 确认场地安全, Ctrl+C 可取消:"
for i in 5 4 3 2 1; do echo "  $i ..."; sleep 1; done

# ------------------------------------------------------------- 执行(驱动)
echo "[scs_autotest] ==== 开始 $RUN ===="
# 输出同步落盘 ~/scs_autotest_last.log, 便于回看/发群; RC 取驱动(管道左侧)退出码
docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -lc \
  "source ~/.zshrc 2>/dev/null; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; python3 -u /home/ubuntu/scs_autotest_driver.py --leg $RUN" 2>&1 | tee ~/scs_autotest_last.log
RC=${PIPESTATUS[0]}

echo "[scs_autotest] 驱动退出码: $RC (0=通过 2=存在失败)"
if [ "$RC" = "2" ]; then
  echo "[scs_autotest] ✗ 有关卡未通过 —— 看上面日志定位; 完整日志: ~/scs_autotest_last.log"
else
  echo "[scs_autotest] ✓ 完成"
fi
exit $RC
