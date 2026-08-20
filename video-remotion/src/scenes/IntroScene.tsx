import { AbsoluteFill, Easing, interpolate, useCurrentFrame, useVideoConfig } from "remotion";

export const IntroScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();
  const titleProgress = interpolate(frame, [8, fps * 1.1], [0, 1], {
    easing: Easing.bezier(0.16, 1, 0.3, 1),
    extrapolateRight: "clamp",
  });
  const detailProgress = interpolate(frame, [fps * 0.65, fps * 1.5], [0, 1], {
    easing: Easing.bezier(0.16, 1, 0.3, 1),
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ backgroundColor: "#06121f", color: "#f7fbff", overflow: "hidden" }}>
      <div style={{ position: "absolute", inset: 0, backgroundImage: "linear-gradient(rgba(71, 198, 255, 0.11) 1px, transparent 1px), linear-gradient(90deg, rgba(71, 198, 255, 0.11) 1px, transparent 1px)", backgroundSize: "64px 64px", opacity: 0.55 }} />
      <div style={{ position: "absolute", top: 94, left: 90, width: 230, height: 8, backgroundColor: "#36d1dc" }} />
      <div style={{ position: "absolute", right: 0, top: 0, width: "36%", height: "100%", backgroundColor: "#0c2033", borderLeft: "1px solid #234763" }} />
      <div style={{ position: "absolute", left: 90, top: 170, width: 1120, opacity: titleProgress, translate: `${interpolate(titleProgress, [0, 1], [-70, 0])}px 0px` }}>
        <div style={{ color: "#73e3ff", fontSize: 28, fontWeight: 700, letterSpacing: 4 }}>VALIDATION RECORD / 2026.08.20</div>
        <div style={{ fontSize: 98, fontWeight: 800, lineHeight: 0.98, marginTop: 35 }}>CVTRACK</div>
        <div style={{ color: "#b7d0e3", fontSize: 49, fontWeight: 600, lineHeight: 1.2, marginTop: 24 }}>Three-Scene System Demonstration</div>
      </div>
      <div style={{ position: "absolute", left: 90, bottom: 110, width: 980, paddingLeft: 24, borderLeft: "5px solid #36d1dc", color: "#d9ecf8", fontSize: 28, lineHeight: 1.45, opacity: detailProgress, translate: `${interpolate(detailProgress, [0, 1], [0, 30])}px 0px` }}>
        PX4/Gazebo visual simulation and two high-resolution tracking replays. Source and claim boundaries remain visible throughout.
      </div>
      <div style={{ position: "absolute", top: 180, right: 105, width: 455, opacity: detailProgress }}>
        <div style={{ color: "#73e3ff", fontSize: 22, fontWeight: 700, letterSpacing: 2 }}>EVIDENCE SCOPE</div>
        <div style={{ marginTop: 24, display: "grid", gap: 18 }}>
          <div style={{ borderTop: "1px solid #2f6688", paddingTop: 15, fontSize: 25 }}>01 / PX4 + Gazebo GUI</div>
          <div style={{ borderTop: "1px solid #2f6688", paddingTop: 15, fontSize: 25 }}>02 / Airport target tracking</div>
          <div style={{ borderTop: "1px solid #2f6688", paddingTop: 15, fontSize: 25 }}>03 / Overhead vehicle tracking</div>
        </div>
      </div>
    </AbsoluteFill>
  );
};
