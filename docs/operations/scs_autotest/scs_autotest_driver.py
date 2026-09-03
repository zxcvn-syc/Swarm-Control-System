#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SCS UGV 四关自动测试驱动（在车容器内执行，由主机 scs_autotest.sh 壳调用）。

链路: 本驱动发 /planned_path → ugv_path_follower(收路径) → /cmd_vel → 厂商底盘 → /odom → relay → /ugv_pose(本驱动监控)

用法:
    python3 scs_autotest_driver.py --leg straight|turn|retarget|estop|full [--dry-run]

安全约定:
    * --dry-run 只打印每关的规划(不初始化 ROS、不碰话题), 车绝对不动。
    * 真正执行必须由主机壳 scs_autotest.sh --run <leg> 触发(壳负责电池检查 + 倒计时)。
    * 每关内置最长时间限制(max_s), 超时自动发"当前位置点"路径让 follower 进 DONE 急停。
    * 运行中电压看门狗(9.3 加): 电池 <=7300mV(实测电机切断线)立即急停中止并报"低压";
      <7500mV(与壳层拒跑线一致)告警。低压下测试结果不可信, 不跑比跑错好。
    * 全程需要 /ugv_pose(odom_relay) 有数据; follower 在运行。缺任一, wait_pose 会报错退出, 不会动车。
    *
    * 判关注意(9.2 实测): /ugv_pose 源于厂商 odom_publisher 的 cmd_vel 数值积分(不读编码器,
    * 电机被切断时 odom 仍会"前进")—— 每关结束会打印 odom 位移/电池最低作证据,
    * 最终判关必须核对物理位移(录像/目测), 不能只信 odom。
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import UInt16

# ---------------------------------------------------------------------------
# 关卡说明(供 --dry-run / 帮助)
# ---------------------------------------------------------------------------
LEG_DESC = {
    "straight": "第1关 直线1.5m: 沿当前车头方向前进1.5m, follower 到点自停",
    "turn":     "第2关 90度转弯: 先直行约0.8m, 再向右转90度方向走0.8m (L形, 差速小弧转弯)",
    "retarget": "第3关 动态换目标: 先发前方1.0m目标启动, 车动起来后改发右转90度方向0.9m新目标, 验证切换",
    "estop":    "第4关 急停: 先发前方2.5m目标让车行进, 约2.5s后发当前位置点路径强制停, 检测3s内静止",
    "full":     "连跑四关 (straight -> turn -> retarget -> estop), 任一关失败即中止",
}
LEG_ORDER = ["straight", "turn", "retarget", "estop"]

FRAME = "drone_4"          # 与团队 planner 路径一致的 frame_id 标签(单机时 follower 不过滤)
ARRIVE_TOL = 0.35          # 到点判定(米): follower goal_tol=0.25, 加 0.1 余量
SETTLE_S = 0.6             # 到点后确认稳定的时长
STOP_SLOP = 0.05           # 急停后允许的最大漂移(米)

# 电压看门狗(9.3 加): 9.2 实测电池 7315mV 时电机已被低压保护切断力矩(车纹丝不动、
# odom 却照常"前进")。运行中 <=7300 立即急停中止; <7500(壳层拒跑线)告警一次/10s。
BAT_ABORT_MV = 7300
BAT_WARN_MV = 7500

FWD = lambda yaw: (math.cos(yaw), math.sin(yaw))
RIGHT = lambda yaw: (math.sin(yaw), -math.cos(yaw))   # 车体右侧(顺时针90度)方向


class LowVoltError(RuntimeError):
    """低压保护触发 —— 电机可能已无输出, 本关结果不可信, 由调用方中止后续。"""


def _now():
    return time.strftime("%H:%M:%S")


def _log(msg):
    print(f"{_now()} | {msg}", flush=True)


def _describe(leg):
    lines = [f"关卡: {leg}  —  {LEG_DESC[leg]}", ""]
    lines.append("运行前提(必须已满足, 否则请先跑 ~/scs_start.sh):")
    lines.append("  1) 厂商栈 bringup 在跑 (/odom 约30Hz)")
    lines.append("  2) odom_relay 在跑 (/ugv_pose 约30Hz)")
    lines.append("  3) ugv_path_follower 在跑 (订阅 /planned_path 与 /ugv_pose)")
    lines.append("  4) 车头朝正东摆好, 该关所需方向净空足够, 地面平整")
    lines.append("安全: 每关超时自动急停; 全程随时可 Ctrl+C (壳层).")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 路径生成
# ---------------------------------------------------------------------------
def _line_pts(x, y, yaw, dist, n=6, head=0.15):
    """沿车头方向直线到 dist, 从 head 起插 n 个点(不含起点)。"""
    fx, fy = FWD(yaw)
    ds = [head + (dist - head) * i / (n - 1) for i in range(n)]
    return [(x + fx * d, y + fy * d) for d in ds]


def _l_pts(x, y, yaw, d1=0.8, d2=0.8):
    """L形: 先直行 d1, 再向右转90度方向走 d2。返回顺序点(不含起点)。"""
    fx, fy = FWD(yaw)
    rx, ry = RIGHT(yaw)
    corner = (x + fx * d1, y + fy * d1)
    # 直段: 0.2/0.45/0.7/d1 ; 转后: corner+0.2r ... corner+d2*r
    pts = [(x + fx * d, y + fy * d) for d in (0.2, 0.45, 0.7, d1)]
    pts += [(corner[0] + rx * d, corner[1] + ry * d) for d in (0.2, 0.45, 0.7, d2)]
    return pts


# ---------------------------------------------------------------------------
# ROS 节点
# ---------------------------------------------------------------------------
class AutotestNode(Node):
    def __init__(self):
        super().__init__("scs_autotest_driver")
        self._pose = None
        self._bat = None            # 最近一次电池电压(mV)
        self.bat_min = None         # 本进程运行期间最低电压(mV)
        self._warned_at = 0.0
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pose_sub = self.create_subscription(
            PoseStamped, "/ugv_pose", self._cb_pose, q)
        self._bat_sub = self.create_subscription(
            UInt16, "/ros_robot_controller/battery", self._cb_battery, q)
        self._path_pub = self.create_publisher(
            Path, "/planned_path", QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

    # -- 基础 --
    def _cb_pose(self, m):
        p, qq = m.pose.position, m.pose.orientation
        yaw = math.atan2(2.0 * (qq.w * qq.z + qq.x * qq.y),
                         1.0 - 2.0 * (qq.y * qq.y + qq.z * qq.z))
        self._pose = (p.x, p.y, yaw)

    def _cb_battery(self, m):
        b = int(m.data)
        self._bat = b
        if self.bat_min is None or b < self.bat_min:
            self.bat_min = b

    def spin(self, dt):
        t0 = time.monotonic()
        while time.monotonic() - t0 < dt:
            rclpy.spin_once(self, timeout_sec=0.05)

    def read_pose(self):
        return self._pose

    def bat_guard(self):
        """低压看门狗(在 drive_to / wait_motion 循环里调用)。

        <=7300mV: 电机已被切断(9.2 实测 ~7315), 结果不可信 —— 先急停再抛
        LowVoltError, 让上层中止本关并明确报"低压", 而不是干耗到超时。
        <7500mV: 每 10s 告警一次, 提醒检查车是否还在动。
        """
        b = self._bat
        if b is None:
            return
        if b <= BAT_ABORT_MV:
            _log(f"✗ 电池 {b}mV <= {BAT_ABORT_MV}mV (实测电机切断线 ~7315) —— 低压保护触发, 急停中止")
            self.stop_now()
            raise LowVoltError(f"电池 {b}mV 触发低压看门狗")
        if b <= BAT_WARN_MV and time.monotonic() - self._warned_at > 10.0:
            self._warned_at = time.monotonic()
            _log(f"⚠ 电池 {b}mV < {BAT_WARN_MV}mV, 接近切断线 —— 若车已停止请检查电压")

    def wait_pose(self, timeout=20.0):
        t0 = time.monotonic()
        while self._pose is None and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._pose is None:
            raise RuntimeError(
                f"/ugv_pose {timeout:.0f}s 无数据 —— 请确认 odom_relay 已运行 (bash ~/scs_status.sh 体检)")
        return self._pose

    def pub_path(self, pts):
        msg = Path()
        msg.header.frame_id = FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in pts:
            ps = PoseStamped()
            ps.header.frame_id = FRAME
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._path_pub.publish(msg)

    def dist_xy(self, pose, pt):
        return math.hypot(pose[0] - pt[0], pose[1] - pt[1])

    # -- 高层动作 --
    def stop_now(self):
        """急停: 连续发布"最新当前位置点"路径 1 秒。

        follower 每个控制周期都会看到"目标就在脚下"(dist<goal_tol) -> 判定
        DONE -> 零速。点随车滑行实时刷新, 不会出现"去追旧点"的转圈。
        关键兜底: 本进程被杀(如 Ctrl+C)后不再发路径, follower 2s 路径超时零速。
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            p = self.read_pose()
            if p is None:
                self.spin(0.1)
                continue
            self.pub_path([(p[0], p[1])])
            self.spin(0.2)
        _log("急停: 已持续1s发当前位置点路径, follower 应进入 DONE 零速")

    def drive_to(self, pts, goal, max_s, name):
        """持续发布 pts 直到到达 goal(容差 ARRIVE_TOL), 超时自动急停。
        返回 True=到点; False=超时急停。"""
        t0 = time.monotonic()
        arrived_t = None
        while time.monotonic() - t0 < max_s:
            self.pub_path(pts)
            self.spin(0.2)
            self.bat_guard()
            p = self.read_pose()
            if p is None:
                continue
            d = self.dist_xy(p, goal)
            if d <= ARRIVE_TOL:
                if arrived_t is None:
                    arrived_t = time.monotonic()
                elif time.monotonic() - arrived_t >= SETTLE_S:
                    _log(f"[{name}] 到点 ✓  距目标 {d:.2f}m, 用时 {time.monotonic()-t0:.1f}s")
                    return True
            else:
                arrived_t = None
        _log(f"[{name}] 超时({max_s:.0f}s)未到点 -> 急停")
        self.stop_now()
        return False

    def wait_motion(self, pub_pts, dist, max_s, name):
        """持续发布 pub_pts, 等待车相对起点移动超过 dist 米。
        返回结束时实际位移。"""
        start = self.read_pose()
        if start is None:
            return 0.0
        t0 = time.monotonic()
        moved = 0.0
        while time.monotonic() - t0 < max_s:
            self.pub_path(pub_pts)
            self.spin(0.1)
            self.bat_guard()
            p = self.read_pose()
            if p is not None:
                moved = self.dist_xy(p, (start[0], start[1]))
                if moved >= dist:
                    return moved
        return moved


# ---------------------------------------------------------------------------
# 四关
# ---------------------------------------------------------------------------
def _leg_straight(n):
    _log("== 第1关 直线1.5m ==")
    x, y, yaw = n.wait_pose()
    pts = _line_pts(x, y, yaw, 1.5)
    goal = pts[-1]
    _log(f"起点 ({x:.2f},{y:.2f}) 车头 {math.degrees(yaw):.1f}° -> 目标 ({goal[0]:.2f},{goal[1]:.2f})")
    return n.drive_to(pts, goal, max_s=40, name="直线1.5m")


def _leg_turn(n):
    _log("== 第2关 90度转弯(L形) ==")
    x, y, yaw = n.wait_pose()
    pts = _l_pts(x, y, yaw, d1=0.8, d2=0.8)
    goal = pts[-1]
    _log(f"起点 ({x:.2f},{y:.2f}) 先直行0.8m再右转90度走0.8m -> 目标 ({goal[0]:.2f},{goal[1]:.2f})")
    return n.drive_to(pts, goal, max_s=60, name="转弯")


def _leg_retarget(n):
    _log("== 第3关 动态换目标 ==")
    x, y, yaw = n.wait_pose()
    fx, fy = FWD(yaw)
    rx, ry = RIGHT(yaw)
    goalA = (x + 1.0 * fx, y + 1.0 * fy)
    ptsA = _line_pts(x, y, yaw, 1.0, n=4)
    n.pub_path(ptsA)
    _log(f"目标A(正前1.0m): ({goalA[0]:.2f},{goalA[1]:.2f}) —— 持续发布, 等待车启动...")
    moved = n.wait_motion(ptsA, dist=0.20, max_s=15, name="换目标-启动")
    if moved < 0.20:
        _log("车未动起来 -> 中止本关")
        n.stop_now()
        return False
    _log(f"车已动 {moved:.2f}m, 现在切换目标...")
    # 新目标: 从"当前位姿"起沿车右侧方向 0.9m(可观测的大角度切换)
    cx, cy, cyaw = n.read_pose()
    bx, by = RIGHT(cyaw)
    goalB = (cx + 0.9 * bx, cy + 0.9 * by)
    # 构建通往 B 的平滑路径(当前点小幅前探 + B), 直接给 B 也可(follower 会 ROTATE 调头)
    ptsB = _line_pts(cx, cy, math.atan2(by, bx), 0.9, n=5)
    _log(f"目标B(右侧0.9m): ({goalB[0]:.2f},{goalB[1]:.2f}) —— 已切换")
    return n.drive_to(ptsB, goalB, max_s=40, name="换目标")


def _leg_estop(n):
    _log("== 第4关 急停 ==")
    x, y, yaw = n.wait_pose()
    fx, fy = FWD(yaw)
    goal = (x + 2.5 * fx, y + 2.5 * fy)
    pts = _line_pts(x, y, yaw, 2.5)
    n.pub_path(pts)
    _log(f"目标(正前2.5m): ({goal[0]:.2f},{goal[1]:.2f}) —— 持续发布, 车行进中...")
    moved = n.wait_motion(pts, dist=0.40, max_s=15, name="急停-行进")
    if moved < 0.40:
        _log("车未达到行进状态 -> 中止本关")
        n.stop_now()
        return False
    _log(f"车已行进 ~{moved:.2f}m, 触发急停...")
    n.stop_now()
    n.spin(0.8)
    p1 = n.read_pose()
    n.spin(3.0)                       # 静置 3s 观察漂移
    p2 = n.read_pose()
    drift = math.hypot(p2[0] - p1[0], p2[1] - p1[1]) if (p1 and p2) else 99.0
    if drift <= STOP_SLOP:
        _log(f"急停通过 ✓  3s 漂移 {drift:.3f}m (<=0.05)")
        return True
    _log(f"急停疑似未停死 ✗  3s 漂移 {drift:.3f}m (>0.05)")
    return False


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def _dry_run(leg):
    print(_describe(leg))
    print("\n[dry-run] 本轮不初始化 ROS、不发布任何话题, 车不会动。")
    print("[dry-run] 正式执行请用: bash ~/scs_autotest.sh --run <leg>")


def main():
    ap = argparse.ArgumentParser(description="SCS UGV 四关自动测试驱动")
    ap.add_argument("--leg", required=True, choices=sorted(LEG_DESC.keys()))
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    if a.dry_run:
        _dry_run(a.leg)
        return 0
    _log(f"驱动启动 leg={a.leg}")
    rclpy.init()
    node = AutotestNode()
    try:
        node.wait_pose(timeout=20)
        _log(f"/ugv_pose 在线, 当前位姿 ({node.read_pose()[0]:.2f}, "
             f"{node.read_pose()[1]:.2f}) 车头 {math.degrees(node.read_pose()[2]):.1f}°")
        legs = LEG_ORDER if a.leg == "full" else [a.leg]
        ok = True
        for leg in legs:
            if not ok:
                break
            _log(f"--- 开始 {LEG_DESC[leg]} ---")
            pose0 = node.read_pose()
            bat0 = node.bat_min
            t0 = time.monotonic()
            r = False
            try:
                fn = {"straight": _leg_straight, "turn": _leg_turn,
                      "retarget": _leg_retarget, "estop": _leg_estop}[leg]
                r = fn(node)
                ok = ok and r
                if not r:
                    _log(f"[{leg}] 未通过, 中止后续")
            except LowVoltError as e:                    # noqa: BLE001
                _log(f"[{leg}] 低压看门狗中止: {e} (急停已由看门狗执行)")
                ok = False
            except Exception as e:                       # noqa: BLE001
                _log(f"[{leg}] 异常: {e} -> 急停")
                node.stop_now()
                ok = False
            # 每关证据小结 —— odom 为 cmd_vel 数值积分(假里程计), 判关以物理位移/录像为准!
            pose1 = node.read_pose()
            dt = time.monotonic() - t0
            verdict = "PASS" if r else "FAIL"
            if pose0 and pose1:
                disp = math.hypot(pose1[0] - pose0[0], pose1[1] - pose0[1])
                _log(f"[{leg}] 小结: {verdict} | odom位移 {disp:.2f}m | 用时 {dt:.1f}s | "
                     f"电池 {bat0}->{node.bat_min}mV | 终点 ({pose1[0]:.2f},{pose1[1]:.2f})")
            else:
                _log(f"[{leg}] 小结: {verdict} | 用时 {dt:.1f}s | 电池最低 {node.bat_min}mV")
            _log(f"[{leg}] ※ odom 是 cmd_vel 数值积分假里程计, 请核对物理位移/录像后再判通过")
            node.spin(3.0)                               # 每关间隔
        _log("==== 四关自动测试 " + ("全部通过 PASS ====" if ok else "存在失败 FAIL ===="))
        return 0 if ok else 2
    finally:
        node.stop_now()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
