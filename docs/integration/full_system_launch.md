# Full System Launch Manual

**Verified:** 2026-07-28
**ROS2 Version:** Humble
**Workspace:** `ros2_ws/`

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│                              Swarm Control System                                  │
│                                                                                  │
│  ┌──────────────┐    /target_track    ┌─────────────────┐                       │
│  │ tracker_node │ ──────────────────▶ │  scheduler_node  │                       │
│  │(perception_pkg)                   │  (scheduler_pkg)│                       │
│  └──────┬───────┘                     └────────┬────────┘                       │
│         │                                       │                                │
│         │ /enclosure_targets                    │ /task_assignment               │
│         ▼                                       ▼                                │
│  ┌────────────────┐                 ┌───────────────────┐                        │
│  │enclosure_node  │                 │  planner_node    │                        │
│  │(containment_pkg)                 │ (planning_pkg)   │                        │
│  └───────┬────────┘                 └─────────┬─────────┘                        │
│          │                                    │                                  │
│          │ /enclosure_command                  │ /drone_states                   │
│          ▼                                    ▼                                  │
│  ┌──────────────────┐        ┌────────────────────────────────┐                 │
│  │ coord_transform_ │        │  grid_map_node (planning_pkg)  │                 │
│  │ node (/grid_map_nav│◀──────│   /grid_map (UInt8MultiArray) │                 │
│  │  → /target_track_ │        └────────────────────────────────┘                 │
│  │  world)           │        subscribes /grid_map_nav (OccupancyGrid)           │
│  └──────────────────┘                                                            │
└──────────────────────────────────────────────────────────────────────────────────┘
```

### Topic Contract

| Topic                     | Type                      | Publisher              | Subscribers                        |
|---------------------------|---------------------------|------------------------|------------------------------------|
| `/target_track`           | `TargetTrackArray`        | tracker_node           | scheduler_node, coord_transform_node |
| `/target_track_world`     | `TargetTrackArray`       | coord_transform_node   | scheduler_node, **planner_node**    |
| `/enclosure_targets`      | `EnclosureTargetArray`   | tracker_node           | enclosure_node                     |
| `/task_assignment`        | `TaskAssignment`         | scheduler_node         | planner_node                       |
| `/drone_states`           | `DroneStateArray`        | planner_node           | scheduler_node, enclosure_node      |
| `/drone_state`            | `DroneState`             | planner_node           | —                                  |
| `/enclosure_command`      | `EnclosureCommandArray`  | enclosure_node         | —                                  |
| `/grid_map_nav`           | `nav_msgs/OccupancyGrid`| planner_node           | grid_map_node                      |
| `/grid_map`               | `std_msgs/UInt8MultiArray`| grid_map_node        | planner_node                       |

---

## Prerequisites

```bash
# ROS2 Humble must be installed and sourced
source /opt/ros/humble/setup.bash

# Workspace must be built (no --symlink-install due to setuptools 81 compatibility)
cd ros2_ws
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# cvtrack must be installed (for tracker_node video mode)
# pip install -e ros2_ws/src/perception_pkg/cvtrack
```

### Environment Notes

- **numpy**: Must be `<2` (numpy 2.x breaks opencv-python / cv_bridge ABI)
  ```bash
  pip install "numpy<2" "opencv-python<4.11"
  ```
- **setuptools**: Version 81.x causes `--symlink-install` to fail on ament_python packages. Use regular install.

---

## Quick Start (Recommended)

Run the verified launch script:

```bash
cd /home/hhh/Downloads/Swarm-Control-System
bash scripts/full_system_launch.sh
```

This brings up all 4 nodes in correct dependency order, then runs a synthetic data publisher to drive the pipeline.

### With Video Input

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch ros2_ws/launch/three_links.launch.py \
    video_source:=/home/hhh/Downloads/Swarm-Control-System/videos/test_multi_target_tracking.mp4
```

**Requirements:** Valid YOLO weights or `detector.backend:=auto` (MOG2 fallback).

### With Launch Files (No Video)

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# Terminal 1 — tracker (topic mode, no camera needed)
ros2 run perception_pkg tracker_node \
    --ros-args \
    -p input_mode:=topic \
    -p enclosure.enabled:=true \
    -p enclosure.topic:=/enclosure_targets

# Terminal 2 — planner node (A*/D* Lite path planner, self-publishes /grid_map_nav)
ros2 run planning_pkg planner_node \
    --ros-args \
    -p num_drones:=8 \
    -p tick_period:=0.5

# Terminal 3 — grid_map_node (bridges /grid_map_nav OccupancyGrid → /grid_map UInt8MultiArray)
ros2 run planning_pkg grid_map_node

# Terminal 4 — scheduler
ros2 run scheduler_pkg scheduler_node \
    --ros-args \
    -p num_drones:=8 \
    -p assignment_strategy:=greedy

# Terminal 5 — enclosure
ros2 run containment_pkg enclosure_node \
    --ros-args \
    -p enclosure_radius:=25.0
```

---

## Node Dependency Order

1. **tracker_node** — no input dependencies; publishes first
2. **planner_node** — publishes `/drone_states`; scheduler_node subscribes; also publishes `/grid_map_nav`
3. **grid_map_node** — subscribes `/grid_map_nav`, publishes `/grid_map`; planner_node subscribes `/grid_map`
4. **scheduler_node** — subscribes to `/target_track` + `/target_track_world` + `/drone_states`
5. **enclosure_node** — subscribes to `/enclosure_targets` + `/drone_states`

---

## Verification

### Check Node List

```bash
ros2 node list
# Expected:
#   /enclosure_node
#   /grid_map_node
#   /planner_node
#   /scheduler_node
#   /tracker_node
```

### Check Topic List

```bash
ros2 topic list
# Expected topics:
#   /camera/image
#   /drone_state
#   /drone_states
#   /enclosure_command
#   /enclosure_targets
#   /grid_map              (UInt8MultiArray, from grid_map_node)
#   /grid_map_nav          (OccupancyGrid, from planner_node)
#   /parameter_events
#   /rosout
#   /target_track
#   /target_track_debug
#   /target_track_world    (from coord_transform_node)
#   /task_assignment
#   /tracking_metrics
```

### Echo Topics

```bash
ros2 topic echo /target_track --once
ros2 topic echo /task_assignment --once
ros2 topic echo /drone_states --once
ros2 topic echo /enclosure_command --once
```

---

## End-to-End Integration Test

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
cd ros2_ws
python3 test_three_links.py
```

Expected output: `PASS: link1=N link2=M link3=K` (all links verified)

JSON report: `output/test_three_links_<timestamp>.json`

---

## Troubleshooting

### `error: option --editable not recognized`

Cause: `setuptools>=61` with `--symlink-install` flag.
Fix: Remove `--symlink-install` from colcon build command.

### `cannot open video source '0'`

Cause: No camera or video file available.
Fix: Use `input_mode:=topic` and drive with synthetic publisher or external image feed.

### `AttributeError: _ARRAY_API not found`

Cause: numpy 2.x incompatible with ROS2 cv_bridge / opencv-python.
Fix: `pip install "numpy<2" "opencv-python<4.11"`

### No messages on `/task_assignment`

Cause: scheduler_node needs `/drone_states` from planner_stub. Check planner_stub_node started first.
Fix: Restart nodes in correct order; ensure planner_stub_node starts before scheduler_node.

---

## Build Configuration

```bash
# Build command (tested on setuptools 81.x)
cd ros2_ws
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release

# NOT: colcon build --symlink-install (incompatible with setuptools>=61)
```

---

## Launch Script

`scripts/full_system_launch.sh` — Verified startup script that:
- Starts nodes in dependency order
- Uses topic mode for tracker (no camera required)
- Runs synthetic data publisher to drive pipeline
- Reports `ros2 node list` and `ros2 topic list`
- Echoes message counts on all key topics
