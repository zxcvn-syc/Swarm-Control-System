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

## Verified Scope And Limits

- The installed free edition has supplied RGB only for `TargetCopter=1`; the
  final recording is a single physical RGB camera run. The code retains a
  handoff interface, but live multi-camera handoff requires a licensed or
  otherwise calibrated multi-camera Rfly installation before it can be claimed.
- `Grasslands` in clear conditions is the locally verified visual scenario.
  The local `3DDisplay` weather attempt did not provide a usable RGB stream, so
  rain, storm, fog, and wind responses are not claimed as verified results.
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
