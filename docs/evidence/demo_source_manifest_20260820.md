# Demo Source Manifest

| Local input | Source | Native format | Role |
|---|---|---|---|
| `data/demo_inputs/airport_tracked.mp4` | Ultralytics Assets `ground-vehicles-airport.mp4` | 1280x720, 30 fps, 5 s source loop | Airport ground-vehicle tracking replay |
| `data/demo_inputs/parking_tracked.mp4` | Ultralytics Assets `parking` sample, locally validated output | 1280x720, 30 fps, 5 s source loop | Overhead parking tracking replay |
| `videos/gazebo_gui_final_20260820.mp4` | This project, PX4 SITL + Gazebo Classic | 1280x720, 15 fps, 90 s | Real Gazebo GUI recording from Xvfb |

The two perception videos are already rendered tracking outputs, so the boxes, IDs, velocities and overlay text are part of the supplied input. They are demonstration inputs, not benchmark scores or georeferenced deployment footage.

## Source URLs

- https://github.com/ultralytics/assets/releases/download/v0.0.0/ground-vehicles-airport.mp4
- The parking sample was retained from the locally validated CVTrack candidate set; its source URL was not preserved in the earlier manifest. It is therefore described as a local validated sample rather than attributed to a named benchmark.
