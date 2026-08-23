import { AbsoluteFill, Loop, OffthreadVideo, staticFile } from "remotion";
import { SceneChrome } from "./SceneChrome";

type FootageSceneProps = {
  accent: string;
  cropGazebo?: boolean;
  footer: string;
  index: string;
  source: string;
  status: string;
  subtitle: string;
  title: string;
};

export const FootageScene: React.FC<FootageSceneProps> = ({
  accent,
  cropGazebo = false,
  footer,
  index,
  source,
  status,
  subtitle,
  title,
}) => {
  return (
    <AbsoluteFill style={{ backgroundColor: "#06121f", overflow: "hidden" }}>
      {cropGazebo ? (
        <OffthreadVideo
          src={staticFile(source)}
          muted
          style={{
            width: "153.48%",
            height: "153.19%",
            position: "absolute",
            top: 0,
            left: 0,
          }}
        />
      ) : (
        <Loop durationInFrames={150}>
          <OffthreadVideo
            src={staticFile(source)}
            muted
            style={{ width: "100%", height: "100%", objectFit: "cover" }}
          />
        </Loop>
      )}
      <AbsoluteFill style={{ background: "linear-gradient(180deg, rgba(3, 13, 23, 0.14), transparent 35%, rgba(3, 13, 23, 0.16))" }} />
      <SceneChrome accent={accent} index={index} title={title} subtitle={subtitle} status={status} footer={footer} />
    </AbsoluteFill>
  );
};
