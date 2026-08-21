#!/usr/bin/env bash
set -eo pipefail

DEMO_ROOT="${RFLY_DEMO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-$DEMO_ROOT/ros2_ws}"
LOG_ROOT="$DEMO_ROOT/logs"
RUN_SECONDS="${1:-35}"

mkdir -p "$LOG_ROOT"
source /opt/ros/humble/setup.bash
source "$ROS2_WS_ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID=61
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RFLY_SCENE_SEED=20260821

python3 "$DEMO_ROOT/scripts/rfly_ros_scene.py" >"$LOG_ROOT/scene.log" 2>&1 &
SCENE_PID=$!
ros2 run scheduler_pkg scheduler_node --ros-args \
  -p num_drones:=3 \
  -p target_topic:=/target_track_world \
  -p world_frame:=world \
  >"$LOG_ROOT/scheduler.log" 2>&1 &
SCHEDULER_PID=$!
ros2 run planning_pkg planner_node --ros-args \
  -p num_drones:=3 \
  -p grid_size:=220 \
  -p world_frame:=world \
  -p grid_resolution_m:=1.0 \
  -p drone_z_default:=18.0 \
  -p sim_tick_speed:=3.0 \
  -p target_track_world_topic:=/target_track_world \
  >"$LOG_ROOT/planner.log" 2>&1 &
PLANNER_PID=$!
ros2 run containment_pkg enclosure_node --ros-args \
  -p enclosure_radius:=18.0 \
  -p min_dist:=8.0 \
  -p update_period:=0.25 \
  -p target_topic:=/target_track_world \
  -p drone_topic:=/ground_vehicle_states \
  -p world_frame:=world \
  >"$LOG_ROOT/enclosure.log" 2>&1 &
ENCLOSURE_PID=$!

cleanup() {
  kill "$ENCLOSURE_PID" "$PLANNER_PID" "$SCHEDULER_PID" "$SCENE_PID" 2>/dev/null || true
  wait "$ENCLOSURE_PID" "$PLANNER_PID" "$SCHEDULER_PID" "$SCENE_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 9
ros2 topic list --no-daemon --spin-time 3 -t >"$LOG_ROOT/topics.txt"
ros2 node list --no-daemon --spin-time 3 >"$LOG_ROOT/nodes.txt"

capture_topic() {
  local topic=$1
  local message_type=$2
  local output_file=$3
  if ! timeout 8 ros2 topic echo "$topic" "$message_type" --once >"$output_file" 2>&1; then
    printf 'capture_failed: %s\n' "$topic" >>"$output_file"
  fi
}

capture_event_topic() {
  local topic=$1
  local message_type=$2
  local output_file=$3
  timeout "$((RUN_SECONDS + 20))" ros2 topic echo \
    "$topic" "$message_type" --once >"$output_file" 2>&1 &
  EVENT_PIDS+=("$!")
}

EVENT_PIDS=()
capture_event_topic /task_assignment swarm_interfaces/msg/TaskAssignment "$LOG_ROOT/task_assignment.yaml"
capture_event_topic /planned_path nav_msgs/msg/Path "$LOG_ROOT/planned_path.yaml"
capture_event_topic /enclosure_command swarm_interfaces/msg/EnclosureCommandArray "$LOG_ROOT/enclosure_command.yaml"

capture_topic /target_track_world swarm_interfaces/msg/TargetTrackArray "$LOG_ROOT/target_track_world.yaml"
capture_topic /target_track_truth swarm_interfaces/msg/TargetTrackArray "$LOG_ROOT/target_track_truth.yaml"
capture_topic /drone_states swarm_interfaces/msg/DroneStateArray "$LOG_ROOT/drone_states.yaml"
capture_topic /ground_vehicle_states swarm_interfaces/msg/DroneStateArray "$LOG_ROOT/ground_vehicle_states.yaml"
sleep "$RUN_SECONDS"
for event_pid in "${EVENT_PIDS[@]}"; do
  kill "$event_pid" 2>/dev/null || true
  wait "$event_pid" 2>/dev/null || true
done
