# World-coordinate and ROS contract

`cvtrack` tracks in image pixels.  Metric output is enabled only by supplying
a verified ground-plane calibration.  It maps the bottom centre of every
detected bounding box, which is the best available image-space estimate of a
ground contact point for pedestrians and road vehicles.

## Calibration file

Copy `calibrations/ground_plane.example.yaml`, then survey at least four
non-collinear landmarks visible in a single fixed camera view.  Each
`image_points_px[i]` must correspond exactly to `world_points_m[i]`.  The
world frame is a local ENU-like two-dimensional frame, its name comes from
`frame_id`, and its unit must be `m`.

The pipeline rejects missing files, non-metre units, malformed arrays,
collinear points, singular transforms, projection at the homography horizon,
and optionally any calibration whose maximum landmark reprojection error is
greater than `max_reprojection_error_m`.

```bash
python -m cvtrack --config world_projection \
  --source /data/camera.mp4 --out-dir /data/run_001 \
  --world-calibration /data/calibration/site_camera_01.yaml
```

The calibration is valid only for the surveyed plane and the fixed camera
pose used to create it.  A moving airborne camera additionally needs calibrated
intrinsics, current camera pose and altitude, terrain handling, and a
transform into the flight controller's map frame.  This planar mode must not
be used as a substitute for those inputs.

## ROS input file

When projection is enabled, `tracks_world.csv` is written beside the legacy
pixel-space `tracks.csv`.  Its schema is:

```text
frame,timestamp_s,track_id,label,image_x_px,image_y_px,world_x_m,world_y_m,world_vx_mps,world_vy_mps,world_valid,frame_id,units
```

ROS adapters must publish or consume only rows where all of the following are
true: `world_valid == 1`, `units == "m"`, `frame_id` equals the planner's
declared frame (or has a known TF transform), and the timestamp is fresh.
Rows with `world_valid == 0` have blank world values and must be dropped.
Neither `cx/cy` from `tracks.csv` nor `image_x_px/image_y_px` are valid
planner coordinates.

For target-following or enclosure decisions, retain `track_id`, transform the
world position and velocity to the planner frame, reject stale tracks, then
run assignment and path planning.  The scheduler/planner code is not present
in this repository, so its topic/message adapter must be updated in its ROS
workspace to enforce this same gate.
