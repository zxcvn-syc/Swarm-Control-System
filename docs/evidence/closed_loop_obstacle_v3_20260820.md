# Closed-Loop Obstacle Validation

Run date: 2026-08-20

The isolated ROS 2 validation ran the real `scheduler_pkg`, `planning_pkg/grid_map_node`, `planning_pkg/planner_node`, and `containment_pkg/enclosure_node`. PX4, MAVROS, Offboard, and vehicle-control bridge processes were intentionally excluded.

## Result

- `passed`: `true`
- Published world-frame target arrays: `53`
- Received task assignments: `104`
- Received planned-path messages: `60`
- Received enclosure-command messages: `52`
- Injected obstacle cells: `102` in columns `21-26`, rows `12-28`

`drone_0` starts near `(3, 20)`, detours through row `29` to pass around the obstacle, and reaches `(30, 20)`. The captured longest path and the strict detour predicate are stored in [report.json](closed_loop_obstacle_v3_20260820/report.json).

## Reproduction

On the ROS VM, build the four packages in one `--merge-install` overlay, then run:

```bash
bash scripts/run_closed_loop_process_demo.sh \
  --install-base /path/to/ros2_ws/install_closed_loop_validation \
  --output-dir output/closed_loop_obstacle \
  --duration 12 --domain-id 88
```

The runner uses isolated process groups and cleans them up on exit. The rendered evidence video is generated with `scripts/render_closed_loop_evidence_demo.sh`; its left panel is a tracking-video replay and its right panel is this world-frame fixture, not a field-deployment claim.
