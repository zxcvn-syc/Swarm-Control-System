# CVTrack Three-Scene Video

Editable Remotion timeline for the CVTrack validation video. The video is intentionally silent: no music or voice source was approved for this deliverable.

## Prepare media

Run the sibling script from PowerShell before opening Studio or rendering:

```powershell
./prepare-assets.ps1
```

It copies the verified source footage into `public/media/`, which is intentionally ignored by Git.

## Preview and render

```powershell
pnpm exec remotion studio --no-open
pnpm exec remotion render CVTrackThreeSceneDemo ../videos/three_scene_system_demo_20260820.mp4
```

The composition is 1920x1080, 30 fps and exactly 90 seconds. Only the Gazebo segment is an actual PX4/Gazebo GUI simulation. The two tracking segments are high-resolution video replays with their source/claim boundary shown in frame.
