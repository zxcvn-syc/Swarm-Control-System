#!/usr/bin/env bash
#
# full_system_launch.sh — Verified ROS2 multi-node system startup
#
# Topology (based on three_links.launch.py):
#
#   tracker_node (perception_pkg)
#       → /target_track
#       → /enclosure_targets
#
#   scheduler_node (scheduler_pkg)
#       ← /target_track
#       ← /drone_states
#       → /task_assignment
#
#   planner_stub_node (planner_stub)
#       ← /task_assignment
#       ← /target_track
#       → /drone_states
#       → /drone_state
#
#   enclosure_node (containment_pkg)
#       ← /enclosure_targets
#       ← /drone_states
#       → /enclosure_command
#
# NOTE: tracker_node runs in topic-mode (no camera required).
#       A synthetic tracker publisher drives the pipeline.
#       Run with:  bash scripts/full_system_launch.sh
#

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

LOG_DIR="$REPO_DIR/output"
mkdir -p "$LOG_DIR"

echo "============================================"
echo "  Swarm Control System — Full System Launch"
echo "============================================"
echo "Repo:  $REPO_DIR"
echo "Log:   $LOG_DIR"
echo ""

# ---- 1. Kill any existing nodes from previous runs ----
echo "[launch] Stopping any previous nodes..."
for pkg in perception_pkg scheduler_pkg planner_stub containment_pkg; do
    pkill -f "ros2 run $pkg" 2>/dev/null || true
done
sleep 1

# ---- 2. Bring up nodes in dependency order ----
# Order: tracker → scheduler → planner_stub → enclosure
# (planner_stub must start before scheduler because scheduler subscribes to /drone_states)

echo "[launch] Starting tracker_node (perception_pkg, topic mode)..."
ros2 run perception_pkg tracker_node \
    --ros-args \
    -p input_mode:=topic \
    -p enclosure.enabled:=true \
    -p enclosure.topic:=/enclosure_targets \
    -p enclosure.publish_rate_hz:=5.0 \
    -r __node:=tracker_node \
    2>&1 | sed "s/^/[tracker] /" &
TRACKER_PID=$!
echo "  PID=$TRACKER_PID"

echo "[launch] Starting planner_stub_node..."
ros2 run planner_stub planner_stub_node \
    --ros-args \
    -p num_drones:=8 \
    -p tick_period:=0.5 \
    -p altitude:=5.0 \
    -p min_sep:=3.0 \
    -p frame_id:=world \
    -p seed_grid_spacing:=6.0 \
    -p assignment_topic:=/task_assignment \
    -p target_topic:=/target_track \
    -p drone_states_topic:=/drone_states \
    -p drone_state_topic:=/drone_state \
    -r __node:=planner_stub_node \
    2>&1 | sed "s/^/[planner_stub] /" &
STUB_PID=$!
echo "  PID=$STUB_PID"

echo "[launch] Starting scheduler_node..."
ros2 run scheduler_pkg scheduler_node \
    --ros-args \
    -p num_drones:=8 \
    -p assignment_strategy:=greedy \
    -p max_per_drone:=2 \
    -p tick_period:=0.5 \
    -p log_interval_sec:=5.0 \
    -p target_topic:=/target_track \
    -p drone_topic:=/drone_states \
    -p output_topic:=/task_assignment \
    -p default_task_type:=track \
    -r __node:=scheduler_node \
    2>&1 | sed "s/^/[scheduler] /" &
SCHEDULER_PID=$!
echo "  PID=$SCHEDULER_PID"

echo "[launch] Starting enclosure_node..."
ros2 run containment_pkg enclosure_node \
    --ros-args \
    -p enclosure_radius:=25.0 \
    -p min_dist:=5.0 \
    -p update_period:=1.0 \
    -r __node:=enclosure_node \
    2>&1 | sed "s/^/[enclosure] /" &
ENCLOSURE_PID=$!
echo "  PID=$ENCLOSURE_PID"

# ---- 3. Wait for nodes to initialise ----
echo ""
echo "[launch] Waiting 5s for nodes to initialise..."
sleep 5

# ---- 4. Report ROS graph ----
echo ""
echo "========== ros2 node list =========="
ros2 node list 2>&1

echo ""
echo "========== ros2 topic list =========="
ros2 topic list 2>&1

echo ""
echo "========== Key topic types =========="
for topic in /target_track /enclosure_targets /drone_states /task_assignment /enclosure_command /drone_state; do
    result=$(ros2 topic info "$topic" 2>&1 || echo "  (not found)")
    echo "--- $topic ---"
    echo "$result" | head -5
done

# ---- 5. Start synthetic data publisher (in-process) ----
echo ""
echo "[launch] Starting synthetic tracker publisher..."
SYNTH_PID=$(python3 -c "
import sys, time
sys.path.insert(0, '$WS_DIR/install/perception_pkg/lib/python3.10/site-packages')
sys.path.insert(0, '$WS_DIR/install/swarm_interfaces/lib/python3.10/site-packages')
sys.path.insert(0, '$WS_DIR/install/scheduler_pkg/lib/python3.10/site-packages')
sys.path.insert(0, '$WS_DIR/install/planner_stub/lib/python3.10/site-packages')
sys.path.insert(0, '$WS_DIR/install/containment_pkg/lib/python3.10/site-packages')

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from swarm_interfaces.msg import TargetTrackArray, TargetTrack, EnclosureTargetArray, EnclosureTarget

class Synth(Node):
    def __init__(self):
        super().__init__('synthetic_pub')
        self.track_pub = self.create_publisher(TargetTrackArray, '/target_track', 10)
        self.enc_pub = self.create_publisher(EnclosureTargetArray, '/enclosure_targets', 10)
        self.t0 = time.monotonic()
        self.create_timer(0.2, self.tick)

    def tick(self):
        t = time.monotonic() - self.t0
        ta = TargetTrackArray()
        ta.header = Header()
        ta.header.stamp = self.get_clock().now().to_msg()
        ta.header.frame_id = 'world'
        ta.frame_idx = int(t * 5)
        for tid, x, y in [(101, 10 + 2*t), (202, 30 - 1.5*t)]:
            tr = TargetTrack()
            tr.target_id = int(tid)
            tr.x = float(x); tr.y = float(y)
            tr.vx = 2.0; tr.vy = 1.5
            tr.confidence = 0.9; tr.cls = 0
            tr.is_confirmed = True
            tr.speed = 2.0; tr.motion_mode = 2
            ta.tracks.append(tr)
        self.track_pub.publish(ta)

        ea = EnclosureTargetArray()
        ea.header = ta.header
        ea.frame_idx = ta.frame_idx
        ea.num_drones = 8
        ea.drone_x = [0.0]*8; ea.drone_y = [0.0]*8
        ea.enclosure_radius = 25.0; ea.min_enclosure_dist = 5.0
        for tid, x, y in [(101, 10 + 2*t), (202, 30 - 1.5*t)]:
            et = EnclosureTarget()
            et.target_id = int(tid)
            et.x = float(x); et.y = float(y)
            et.speed = 2.0; et.motion_mode = 2
            et.confidence = 0.9
            ea.targets.append(et)
        self.enc_pub.publish(ea)

rclpy.init()
node = Synth()
try:
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.2)
finally:
    node.destroy_node()
    rclpy.shutdown()
" 2>&1)
echo "  Synthetic publisher done (pid=$SYNTH_PID)"

# ---- 6. Final report ----
echo ""
echo "========== Post-synthetic ros2 topic list =========="
ros2 topic list 2>&1

echo ""
echo "========== Message counts (10s echo) =========="
for topic in /target_track /enclosure_targets /drone_states /task_assignment /enclosure_command; do
    count=$(timeout 5 ros2 topic echo "$topic" --qos-reliability reliable --once 2>&1 | grep -c "target_id\|drone_id\|assignment_id\|command_id" 2>/dev/null || echo "0")
    echo "  $topic: $count messages in 5s"
done

echo ""
echo "============================================"
echo "  Nodes running (PIDs):"
echo "    tracker_node:     $TRACKER_PID"
echo "    planner_stub:     $STUB_PID"
echo "    scheduler_node:   $SCHEDULER_PID"
echo "    enclosure_node:   $ENCLOSURE_PID"
echo ""
echo "  To stop:  kill $TRACKER_PID $STUB_PID $SCHEDULER_PID $ENCLOSURE_PID"
echo "  To inspect:"
echo "    ros2 node list"
echo "    ros2 topic list"
echo "    ros2 topic echo /target_track"
echo "============================================"
