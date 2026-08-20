# Three-Link Headless Soak Evidence

Date: 2026-08-20

## Scope

- Host: Ubuntu 22.04.5 VM (`192.168.88.135`), ROS 2 Humble.
- ROS isolation: `ROS_DOMAIN_ID=60`.
- Input: `perception_pkg/test_videos/pexels_pedestrian_crossing.mp4`.
- Command:

```bash
ROS_DOMAIN_ID=60 \
CVTRACK_INSTALL_BASE=$HOME/codex-swarm-validation-v2/ros2_ws/install_validation \
./scripts/run_soak_test.sh --duration 7200 --sample-interval 60 --startup-grace 45
```

The command ran the headless `three_links.launch.py` integration stack. This is
a ROS2 integration soak; it does not arm a vehicle, request Offboard mode, or
claim a desktop Gazebo video or real flight.

## Result

| Check | Result |
|---|---|
| JSON status | `PASS` |
| Requested / actual duration | `7200 s` / `7256 s` |
| CSV samples | `116` total; `115` after the 45-second startup grace |
| Required nodes after grace | `8/8` in every sample; `0` missing-node samples |
| Required node set | tracker, coordinate transform, scheduler, planner, enclosure, UGV state publisher, PX4 offboard bridge, SITL pose bridge |
| RSS after grace | `36,712-36,720 KB` for the ROS launch process |
| Log failure patterns | `0` matches for traceback, module import failure, node death, ROS error, or test failure |
| Shutdown | launch process tree was absent after the script completed |

The runner sends SIGINT to ROS launch first and only escalates when a process
does not exit. This run completed without a recorded launch failure.

## Raw Artifacts

- `soak_20260820_004344_report.json`
  - SHA-256: `67AE12F3EC30C46DE3EE6EE759368007AA4B50D2C4CA36B1DB3D19849FD5DA3C`
- `soak_20260820_004344_samples.csv`
  - SHA-256: `F01E784739EB8A4409A117D04961AE1D6A70FF12A296AC42B9DBA1C54D692A36`
- `soak_20260820_004344.log`
  - SHA-256: `ED0C07910361FF2422168010510A287F44D90C923592B818C4FCC0374BC15066`
