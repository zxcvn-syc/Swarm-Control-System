import {
  AbsoluteFill,
  Easing,
  interpolate,
  Loop,
  OffthreadVideo,
  staticFile,
  useCurrentFrame,
} from "remotion";

type Point = {
  x: number;
  y: number;
};

const obstacle = { x: 21, y: 12, width: 6, height: 17 };

const drone0Path: Point[] = [
  { x: 3, y: 20 }, { x: 4, y: 20 }, { x: 5, y: 20 }, { x: 6, y: 21 },
  { x: 7, y: 22 }, { x: 8, y: 23 }, { x: 9, y: 23 }, { x: 10, y: 23 },
  { x: 11, y: 23 }, { x: 12, y: 23 }, { x: 13, y: 23 }, { x: 14, y: 23 },
  { x: 15, y: 23 }, { x: 16, y: 24 }, { x: 17, y: 25 }, { x: 18, y: 26 },
  { x: 19, y: 27 }, { x: 20, y: 28 }, { x: 21, y: 29 }, { x: 22, y: 29 },
  { x: 23, y: 29 }, { x: 24, y: 29 }, { x: 25, y: 29 }, { x: 26, y: 29 },
  { x: 27, y: 28 }, { x: 27, y: 27 }, { x: 27, y: 26 }, { x: 27, y: 25 },
  { x: 28, y: 24 }, { x: 29, y: 23 }, { x: 29, y: 22 }, { x: 30, y: 21 },
  { x: 30, y: 20 },
];

const drone1Path: Point[] = Array.from({ length: 28 }, (_, index) => ({
  x: index + 3,
  y: 5,
}));

const pathString = (points: Point[]) => points.map((point) => `${point.x},${point.y}`).join(" ");

const countAtFrame = (frame: number, finalValue: number) => Math.round(
  interpolate(frame, [60, 540], [0, finalValue], {
    easing: Easing.out(Easing.cubic),
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  }),
);

const routeProgress = (frame: number, length: number) => Math.min(
  length - 1,
  Math.floor(interpolate(frame, [135, 540], [0, length - 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  })),
);

const Metric: React.FC<{ accent: string; frame: number; label: string; value: number }> = ({
  accent,
  frame,
  label,
  value,
}) => (
  <div style={{ borderLeft: `3px solid ${accent}`, minWidth: 156, paddingLeft: 13 }}>
    <div style={{ color: "#aab5ad", fontSize: 15, fontWeight: 700, letterSpacing: 1.6 }}>{label}</div>
    <div style={{ color: "#f5f7f2", fontSize: 37, fontWeight: 800, lineHeight: 1.1, marginTop: 3 }}>
      {countAtFrame(frame, value)}
    </div>
  </div>
);

export const ClosedLoopEvidenceDemo: React.FC = () => {
  const frame = useCurrentFrame();
  const drone0Index = routeProgress(frame, drone0Path.length);
  const drone1Index = routeProgress(frame, drone1Path.length);
  const drone0Visible = drone0Path.slice(0, drone0Index + 1);
  const drone1Visible = drone1Path.slice(0, drone1Index + 1);
  const drone0 = drone0Path[drone0Index];
  const drone1 = drone1Path[drone1Index];
  const gridOpacity = interpolate(frame, [24, 84], [0, 1], {
    easing: Easing.out(Easing.cubic),
    extrapolateRight: "clamp",
  });
  const formationOpacity = interpolate(frame, [450, 570], [0, 1], {
    easing: Easing.out(Easing.cubic),
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const videoOpacity = interpolate(frame, [510, 660], [1, 0.35], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ backgroundColor: "#101513", color: "#f4f6ef", fontFamily: "Arial, Helvetica, sans-serif" }}>
      <div style={{ position: "absolute", inset: 0, backgroundImage: "linear-gradient(rgba(164, 206, 185, 0.055) 1px, transparent 1px), linear-gradient(90deg, rgba(164, 206, 185, 0.055) 1px, transparent 1px)", backgroundSize: "40px 40px" }} />
      <div style={{ position: "absolute", top: 0, left: 0, right: 0, height: 126, padding: "25px 52px", display: "flex", alignItems: "center", justifyContent: "space-between", backgroundColor: "rgba(16, 21, 19, 0.96)", borderBottom: "2px solid #6ee7b7" }}>
        <div>
          <div style={{ color: "#6ee7b7", fontSize: 17, fontWeight: 700, letterSpacing: 2.2 }}>CVTRACK / CLOSED-LOOP EVIDENCE / 2026.08.20</div>
          <div style={{ fontSize: 42, fontWeight: 800, marginTop: 4 }}>Tracking to obstacle-aware containment</div>
        </div>
        <div style={{ color: "#101513", backgroundColor: "#d8f7b8", padding: "12px 17px", fontSize: 18, fontWeight: 800, letterSpacing: 1.2 }}>STRICT CHECK: PASS</div>
      </div>

      <div style={{ position: "absolute", top: 157, left: 52, width: 722, bottom: 166, border: "1px solid #39473f", backgroundColor: "#171f1b", opacity: videoOpacity }}>
        <Loop durationInFrames={150}>
          <OffthreadVideo src={staticFile("media/airport_tracked.mp4")} muted style={{ width: "100%", height: "100%", objectFit: "cover" }} />
        </Loop>
        <div style={{ position: "absolute", top: 0, left: 0, right: 0, padding: "16px 19px", backgroundColor: "rgba(14, 20, 17, 0.84)", borderBottom: "1px solid #6ee7b7" }}>
          <div style={{ color: "#6ee7b7", fontSize: 15, fontWeight: 800, letterSpacing: 1.8 }}>INPUT A / TRACKING REPLAY</div>
          <div style={{ marginTop: 5, fontSize: 22, fontWeight: 700 }}>Airport vehicle IDs and trajectories</div>
        </div>
        <div style={{ position: "absolute", left: 18, right: 18, bottom: 18, padding: "12px 14px", backgroundColor: "rgba(14, 20, 17, 0.88)", color: "#d1d9d0", fontSize: 16, lineHeight: 1.35 }}>
          Actual tracking-video output. This footage is input evidence only; it is not claimed as georeferenced flight data.
        </div>
      </div>

      <div style={{ position: "absolute", top: 157, left: 812, right: 52, bottom: 166, border: "1px solid #547260", backgroundColor: "rgba(20, 31, 25, 0.97)", padding: "22px 28px", opacity: gridOpacity }}>
        <div style={{ display: "flex", alignItems: "flex-start", justifyContent: "space-between" }}>
          <div>
            <div style={{ color: "#d8f7b8", fontSize: 16, fontWeight: 800, letterSpacing: 1.8 }}>INPUT B / WORLD-FRAME REPLAY FIXTURE</div>
            <div style={{ fontSize: 27, fontWeight: 800, marginTop: 4 }}>40 x 40 decision grid, ROS domain 88</div>
          </div>
          <div style={{ color: "#f8d477", fontSize: 15, fontWeight: 800, border: "1px solid #f8d477", padding: "8px 10px" }}>OBSTACLE 21-26 / 12-28</div>
        </div>

        <div style={{ position: "absolute", left: 35, top: 102, width: 550, height: 550, border: "1px solid #52645a", backgroundColor: "#0d1511" }}>
          <svg viewBox="0 0 40 40" style={{ display: "block", width: "100%", height: "100%" }}>
            <defs>
              <pattern id="grid" width="1" height="1" patternUnits="userSpaceOnUse">
                <path d="M 1 0 L 0 0 0 1" fill="none" stroke="#355044" strokeWidth="0.055" />
              </pattern>
            </defs>
            <rect x="0" y="0" width="40" height="40" fill="url(#grid)" />
            <rect x={obstacle.x} y={obstacle.y} width={obstacle.width} height={obstacle.height} fill="#d55d45" opacity="0.82" />
            <rect x="20.7" y="11.7" width="6.6" height="17.6" fill="none" stroke="#ffba70" strokeWidth="0.16" />
            <polyline points={pathString(drone0Path)} fill="none" stroke="#255b4a" strokeWidth="0.7" opacity="0.7" />
            <polyline points={pathString(drone1Path)} fill="none" stroke="#2f6252" strokeWidth="0.55" opacity="0.7" />
            <polyline points={pathString(drone0Visible)} fill="none" stroke="#6ee7b7" strokeWidth="0.85" strokeLinecap="round" strokeLinejoin="round" />
            <polyline points={pathString(drone1Visible)} fill="none" stroke="#f8d477" strokeWidth="0.65" strokeLinecap="round" />
            <circle cx="30" cy="20" r="1" fill="#ff9f7e" />
            <circle cx="30" cy="5" r="1" fill="#ffdc75" />
            <circle cx={drone0.x} cy={drone0.y} r="0.95" fill="#e6fff2" stroke="#183c2f" strokeWidth="0.25" />
            <circle cx={drone1.x} cy={drone1.y} r="0.8" fill="#fff1ad" stroke="#463d15" strokeWidth="0.2" />
            <circle cx="30" cy="20" r="25" fill="none" stroke="#b5eece" strokeWidth="0.18" strokeDasharray="0.7 0.55" opacity={formationOpacity * 0.82} />
            <circle cx="5.59" cy="20" r="0.58" fill="#b5eece" opacity={formationOpacity} />
            <circle cx="30" cy="-19.65" r="0.58" fill="#b5eece" opacity={formationOpacity} />
            <circle cx="40.71" cy="42.86" r="0.58" fill="#b5eece" opacity={formationOpacity} />
          </svg>
          <div style={{ position: "absolute", top: 13, left: 14, color: "#e6fff2", fontSize: 16, fontWeight: 800 }}>DRONE_0 / DETOUR ROUTE</div>
          <div style={{ position: "absolute", bottom: 12, left: 14, color: "#ffdc75", fontSize: 15, fontWeight: 800 }}>DRONE_1 / DIRECT ROUTE</div>
        </div>

        <div style={{ position: "absolute", top: 118, left: 618, right: 26, display: "grid", gap: 18 }}>
          <div style={{ borderLeft: "4px solid #6ee7b7", paddingLeft: 14 }}>
            <div style={{ color: "#6ee7b7", fontSize: 15, fontWeight: 800, letterSpacing: 1.6 }}>TASK ALLOCATION</div>
            <div style={{ fontSize: 25, fontWeight: 750, marginTop: 5 }}>UAV-0 -&gt; Target 101</div>
            <div style={{ color: "#c3d0c6", marginTop: 4, fontSize: 17 }}>UAV-1 -&gt; Target 202</div>
          </div>
          <div style={{ borderLeft: "4px solid #ff9f7e", paddingLeft: 14 }}>
            <div style={{ color: "#ffb39a", fontSize: 15, fontWeight: 800, letterSpacing: 1.6 }}>OBSTACLE-AWARE PATH</div>
            <div style={{ fontSize: 24, fontWeight: 750, marginTop: 5 }}>Route exits at y = 29</div>
            <div style={{ color: "#c3d0c6", marginTop: 4, fontSize: 17 }}>No waypoint crosses the blocked cells.</div>
          </div>
          <div style={{ borderLeft: "4px solid #d8f7b8", paddingLeft: 14, opacity: formationOpacity }}>
            <div style={{ color: "#d8f7b8", fontSize: 15, fontWeight: 800, letterSpacing: 1.6 }}>ENCLOSURE OUTPUT</div>
            <div style={{ fontSize: 24, fontWeight: 750, marginTop: 5 }}>4 formation targets</div>
            <div style={{ color: "#c3d0c6", marginTop: 4, fontSize: 17 }}>Voronoi command ring around the tracked area.</div>
          </div>
        </div>
      </div>

      <div style={{ position: "absolute", left: 52, right: 52, bottom: 42, height: 91, display: "flex", alignItems: "center", gap: 44, padding: "0 22px", backgroundColor: "#151d18", borderTop: "1px solid #536259" }}>
        <Metric accent="#6ee7b7" frame={frame} label="WORLD TRACK FRAMES" value={53} />
        <Metric accent="#92d8ff" frame={frame} label="TASK ASSIGNMENTS" value={104} />
        <Metric accent="#ffb39a" frame={frame} label="PLANNED PATHS" value={60} />
        <Metric accent="#d8f7b8" frame={frame} label="ENCLOSURE COMMANDS" value={52} />
        <div style={{ marginLeft: "auto", maxWidth: 432, color: "#c8d0c9", fontSize: 15, lineHeight: 1.36 }}>
          Recorded from real scheduler, grid-map, planner and enclosure processes. No PX4, MAVROS or vehicle-control bridge was started.
        </div>
      </div>
    </AbsoluteFill>
  );
};
