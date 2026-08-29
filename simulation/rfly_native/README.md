# RflySim Native RGB Demonstration

`rfly_native_demo.py` is a live RflySim demonstration, not a video replay. It
creates a blue target vehicle, three grey interception vehicles, three UAVs,
large static obstacles, and two moving large vehicles. The default `Grasslands`
run is the bright-RGB tracking scenario; use an Rfly weather-enabled map only
after its RGB calibration is verified for the installed Rfly build.
The controller receives candidate detections only from the RGB frames delivered
by `VisionCaptureApi`. The target's generated pose is retained in `trace.jsonl`
only for post-run validation and is not an input to tracking or control.

## Verified Local Evidence

The following native RflySim recording passed the built-in acceptance checks on
the local free-edition installation on 2026-08-29:

```powershell
python simulation/rfly_native/rfly_native_demo.py --duration 30 --no-yolo --output F:\RflyEvidence\native_gray_obstacle_reacquire_final_20260829
```

`report.json` in that directory records `accepted: true`, `0.966` active-camera
detection coverage, a maximum loss of `9` active RGB frames, `22.57` source RGB
FPS, `24.5` recorded FPS, and zero encoder queue drops. The two MP4 files show
the native down-looking RGB camera and a synchronized decision dashboard with
the Rfly god view.

The accompanying delivery video is
`F:\RflyEvidence\hybrid_explainer_20260829\Rfly_tracking_hybrid_explainer_20260829.mp4`.
Its 4--24 second section is the native recording above. Its weather, occlusion,
and multi-camera diagrams are generated strategy illustrations, each visibly
marked `策略示意 / 非本机 Rfly / 待验证`; they are not test evidence.

## Verified Scope And Limits

- The live three-camera probe with
  `config/vision_multi_downward.json` received `U1=372`, `U2=0`, and `U3=0`
  RGB frames. `handoff_events` is empty and
  `multi_camera_handoff_verified` is `false`. The installed free edition does
  not provide evidence of real multi-camera handoff. The controller now maps
  each stream from its `TargetCopter` value rather than relying on JSON order,
  but the handoff interface must be rerun on a multi-camera Rfly installation.
- The full Grasslands weather calibration cycled enum values 0 through 7
  (`CLEAR`, `CLOUDY`, `PARTLY_CLOUDY`, `OVERCAST`, `LIGHT_RAIN`, `RAIN`,
  `STORM`, and `FOG`) at
  `F:\RflyEvidence\native_weather_full_rgb_calibration_20260829_b`.
  None maintained usable RGB luminance and target detection. In particular,
  `LIGHT_RAIN`, `RAIN`, `STORM`, and `FOG` have 0.0 detection coverage.
  `FOG` correctly maps to enum `7`; the earlier enum `10` was Blizzard. Rain,
  storm, fog, and wind responses are not claimed as locally verified.
- `report.json` now exposes `camera_frames`, `camera_detections`,
  `handoff_events`, `all_requested_cameras_active`, and per-weather RGB
  luminance. A run cannot set `validation.accepted` when any requested camera
  has supplied zero frames.
- Static and moving grey vehicles are kinematically commanded Rfly scene
  objects. They provide visual obstruction and separation scenarios, but this
  demo does not claim Rfly contact-physics or traffic-engine validation.
- The blue vehicle pose in `trace.jsonl` is post-run validation data only. The
  detector, tracker, UAV commands, and ground-vehicle interception commands use
  Rfly `VisionCaptureApi` RGB detections.

## Run

1. Start `F:\RflySim3D\RflySim3D.exe` and leave its first viewport open.
2. From the repository root, run:

```powershell
python simulation/rfly_native/rfly_native_demo.py --duration 96 --output F:\RflyEvidence\native_20260829
```

The default configuration uses Rfly's validated free-edition camera binding on
UDP port `9999`. The controller can hand off only to a camera that has supplied
a real detection; the included control code keeps that boundary for a licensed
multi-camera configuration, while the default run is a single-RGB-camera
tracking run.

Do not use `--map-name 3DDisplay --weather` as evidence on this local build:
that map has not delivered a usable RGB feed in calibration. The generated
report lists requested weather states but weather coverage is valid only after a
successful RGB calibration and an accepted recording for that map.

## Calibration Commands

Use this command to reproduce the local multi-camera delivery failure and
inspect the per-camera counters in `report.json`:

```powershell
python simulation/rfly_native/rfly_native_demo.py --duration 16 --no-yolo `
  --vision-config simulation/rfly_native/config/vision_multi_downward.json `
  --output F:\RflyEvidence\native_multi_camera_downward_probe
```

Use this command only to calibrate whether the installed Rfly build sends
visible RGB for weather presets; a failing report is useful evidence:

```powershell
python simulation/rfly_native/rfly_native_demo.py --duration 44 --no-yolo --weather `
  --weather-interval 5 `
  --weather-profiles "CLEAR:0,CLOUDY:1,PARTLY_CLOUDY:2,OVERCAST:3,LIGHT_RAIN:4,RAIN:5,STORM:6,FOG:7" `
  --output F:\RflyEvidence\native_weather_calibration
```

`config/vision_handoff_probe.json` deliberately points U1 upward while U2 and
U3 point downward. It is a negative-control configuration for a real handoff:
only a report with nonzero frames for the candidate cameras and a recorded
`handoff U1->U2` or `handoff U1->U3` event may be presented as handoff evidence.

## Outputs

- `rfly_native_rgb.mp4`: raw Rfly VisionCapture RGB stream with a detection box.
- `rfly_native_dashboard.mp4`: raw RGB stream beside the scene diagnostic,
  prediction vector, active camera, weather, and control state.
- `trace.jsonl`: frame-level RGB detection, decision, command, and validation
  records.
- `report.json`: concise run metadata and the perception/control boundary.

The Rfly viewport itself is switched to an animated god view with vehicle IDs,
paths, the minimap, and weather. The dashboard is a diagnostic overlay; it does
not replace the native Rfly RGB stream.

## Test

```powershell
python -m pytest simulation/rfly_native/tests -q
```
