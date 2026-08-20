import { AbsoluteFill, Easing, interpolate, useCurrentFrame, useVideoConfig } from "remotion";

type SceneChromeProps = {
  accent: string;
  index: string;
  title: string;
  subtitle: string;
  footer: string;
  status: string;
};

export const SceneChrome: React.FC<SceneChromeProps> = ({ accent, footer, index, status, subtitle, title }) => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();
  const headerProgress = interpolate(frame, [0, fps * 0.55], [0, 1], {
    easing: Easing.bezier(0.16, 1, 0.3, 1),
    extrapolateRight: "clamp",
  });
  const footerProgress = interpolate(frame, [fps * 0.15, fps * 0.75], [0, 1], {
    easing: Easing.bezier(0.16, 1, 0.3, 1),
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ pointerEvents: "none" }}>
      <div
        style={{
          position: "absolute",
          top: 0,
          left: 0,
          width: "100%",
          height: 150,
          padding: "28px 64px",
          display: "flex",
          alignItems: "flex-start",
          justifyContent: "space-between",
          color: "#f7fbff",
          backgroundColor: "rgba(4, 18, 31, 0.9)",
          borderBottom: `2px solid ${accent}`,
          opacity: headerProgress,
          translate: `${interpolate(headerProgress, [0, 1], [-60, 0])}px 0px`,
        }}
      >
        <div>
          <div style={{ color: accent, fontSize: 20, fontWeight: 700, letterSpacing: 2.5 }}>{index}</div>
          <div style={{ fontSize: 42, fontWeight: 700, lineHeight: 1.1, marginTop: 5 }}>{title}</div>
          <div style={{ color: "#b8d2e6", fontSize: 23, marginTop: 8 }}>{subtitle}</div>
        </div>
        <div
          style={{
            color: accent,
            fontSize: 19,
            fontWeight: 700,
            letterSpacing: 1.8,
            padding: "9px 13px",
            border: `1px solid ${accent}`,
            marginTop: 4,
          }}
        >
          {status}
        </div>
      </div>
      <div
        style={{
          position: "absolute",
          right: 64,
          bottom: 46,
          color: "#e5f3ff",
          fontSize: 19,
          fontWeight: 600,
          letterSpacing: 1.1,
          padding: "12px 16px",
          backgroundColor: "rgba(4, 18, 31, 0.88)",
          borderLeft: `5px solid ${accent}`,
          opacity: footerProgress,
          translate: `${interpolate(footerProgress, [0, 1], [70, 0])}px 0px`,
        }}
      >
        {footer}
      </div>
    </AbsoluteFill>
  );
};
