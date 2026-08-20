#!/usr/bin/env bash

# Render a traceable three-scene system demo from one Gazebo capture and two perception replays.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
GAZEBO_INPUT="$REPO_ROOT/output/video_work/gazebo_gui_final.mp4"
PEDESTRIAN_INPUT="$REPO_ROOT/data/demo_inputs/airport_tracked.mp4"
AERIAL_INPUT="$REPO_ROOT/data/demo_inputs/parking_tracked.mp4"
OUTFILE="$REPO_ROOT/videos/three_scene_system_demo_20260820.mp4"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Options:
  --gazebo FILE       PX4/Gazebo virtual-desktop capture.
  --pedestrian FILE   Ground-camera tracking replay input.
  --aerial FILE       Overhead tracking replay input.
  --out FILE          Output H.264 MP4 file.
  --help              Show this help.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gazebo) GAZEBO_INPUT="${2:?missing Gazebo input}"; shift 2 ;;
    --pedestrian) PEDESTRIAN_INPUT="${2:?missing pedestrian input}"; shift 2 ;;
    --aerial) AERIAL_INPUT="${2:?missing aerial input}"; shift 2 ;;
    --out) OUTFILE="${2:?missing output path}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[three-scene-demo] unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

for file in "$GAZEBO_INPUT" "$PEDESTRIAN_INPUT" "$AERIAL_INPUT"; do
  [[ -f "$file" ]] || { echo "[three-scene-demo] missing input: $file" >&2; exit 2; }
done
command -v ffmpeg >/dev/null || { echo "[three-scene-demo] missing command: ffmpeg" >&2; exit 2; }
command -v fc-match >/dev/null || { echo "[three-scene-demo] missing command: fc-match" >&2; exit 2; }

FONT_FILE="$(fc-match -f '%{file}' 'DejaVu Sans')"
[[ -f "$FONT_FILE" ]] || { echo "[three-scene-demo] unable to resolve DejaVu Sans font" >&2; exit 2; }
mkdir -p "$(dirname "$OUTFILE")"

ffmpeg -hide_banner -loglevel warning -y \
  -i "$GAZEBO_INPUT" \
  -stream_loop -1 -i "$PEDESTRIAN_INPUT" \
  -stream_loop -1 -i "$AERIAL_INPUT" \
  -filter_complex "\
[0:v]trim=duration=30,setpts=PTS-STARTPTS,crop=834:470:0:0,scale=1920:1080:flags=lanczos,fps=25,format=yuv420p,drawbox=x=0:y=0:w=iw:h=116:color=0x071827@0.86:t=fill,drawtext=fontfile='$FONT_FILE':text='SCENE 1 - PARK PATROL - PX4 GAZEBO SITL':fontcolor=white:fontsize=40:x=48:y=18,drawtext=fontfile='$FONT_FILE':text='VIRTUAL DESKTOP SIMULATION - NO FLIGHT COMMAND':fontcolor=0xBDE6FF:fontsize=24:x=48:y=74[s0];\
[1:v]trim=duration=30,setpts=PTS-STARTPTS,scale=1920:1080:flags=lanczos,fps=25,format=yuv420p,drawbox=x=0:y=0:w=iw:h=116:color=0x071827@0.86:t=fill,drawtext=fontfile='$FONT_FILE':text='SCENE 2 - EVENT SECURITY - AIRPORT TRACKING REPLAY':fontcolor=white:fontsize=40:x=48:y=18,drawtext=fontfile='$FONT_FILE':text='YOLOv8 TRACKING INPUT - NO DEPLOYED FLIGHT CLAIM':fontcolor=0xBDE6FF:fontsize=24:x=48:y=74[s1];\
[2:v]trim=duration=30,setpts=PTS-STARTPTS,scale=1920:1080:flags=lanczos,fps=25,format=yuv420p,drawbox=x=0:y=0:w=iw:h=116:color=0x071827@0.86:t=fill,drawtext=fontfile='$FONT_FILE':text='SCENE 3 - TRAFFIC CONTAINMENT - OVERHEAD TRACKING REPLAY':fontcolor=white:fontsize=40:x=48:y=18,drawtext=fontfile='$FONT_FILE':text='YOLOv8 TRACKING INPUT - NO DEPLOYED FLIGHT CLAIM':fontcolor=0xBDE6FF:fontsize=24:x=48:y=74[s2];\
[s0][s1][s2]concat=n=3:v=1:a=0[outv]" \
  -map '[outv]' -an -c:v libx264 -preset veryfast -crf 20 -movflags +faststart "$OUTFILE"

ffprobe -v error -show_entries stream=codec_name,width,height,avg_frame_rate -show_entries format=duration,size -of default=noprint_wrappers=1 "$OUTFILE"
echo "[three-scene-demo] wrote $OUTFILE"
