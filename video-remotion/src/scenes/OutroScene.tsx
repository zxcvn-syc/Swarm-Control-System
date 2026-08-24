import { AbsoluteFill, Easing, interpolate, useCurrentFrame, useVideoConfig } from "remotion";

export const OutroScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();
  const progress = interpolate(frame, [0, fps * 0.9], [0, 1], {
    easing: Easing.bezier(0.16, 1, 0.3, 1),
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ backgroundColor: "#06121f", color: "#f7fbff", padding: "116px 120px" }}>
      <div style={{ position: "absolute", inset: 0, backgroundImage: "linear-gradient(135deg, rgba(54, 209, 220, 0.12), transparent 46%), linear-gradient(315deg, rgba(251, 191, 36, 0.1), transparent 45%)" }} />
      <div style={{ position: "relative", width: "100%", height: "100%", borderTop: "3px solid #36d1dc", borderBottom: "1px solid #2d5b76", opacity: progress }}>
        <div style={{ paddingTop: 96, fontSize: 28, fontWeight: 700, color: "#73e3ff", letterSpacing: 3 }}>EVIDENCE PACKAGE</div>
        <div style={{ marginTop: 26, fontSize: 72, fontWeight: 800, lineHeight: 1.05 }}>Three scenes. One clear boundary.</div>
        <div style={{ marginTop: 36, fontSize: 31, color: "#c8dfed", lineHeight: 1.45, maxWidth: 1200 }}>
          One PX4/Gazebo visual simulation and two high-resolution tracking replays. The replay footage is not presented as field deployment or georeferenced flight evidence.
        </div>
        <div style={{ position: "absolute", bottom: 95, left: 0, display: "flex", gap: 48, fontSize: 24, color: "#d9ecf8" }}>
          <div><span style={{ color: "#36d1dc", fontWeight: 700 }}>01</span> GUI recording</div>
          <div><span style={{ color: "#6ee7b7", fontWeight: 700 }}>02</span> Airport tracking</div>
          <div><span style={{ color: "#fbbf24", fontWeight: 700 }}>03</span> Overhead tracking</div>
        </div>
      </div>
    </AbsoluteFill>
  );
};
