import "./index.css";
import { Composition, Folder } from "remotion";
import { CVTrackThreeSceneDemo } from "./CVTrackThreeSceneDemo";
import { ClosedLoopEvidenceDemo } from "./ClosedLoopEvidenceDemo";
import { AirportReplayScene } from "./scenes/AirportReplayScene";
import { GazeboSimulationScene } from "./scenes/GazeboSimulationScene";
import { IntroScene } from "./scenes/IntroScene";
import { OutroScene } from "./scenes/OutroScene";
import { ParkingReplayScene } from "./scenes/ParkingReplayScene";

const width = 1920;
const height = 1080;
const fps = 30;

export const RemotionRoot: React.FC = () => {
  return (
    <>
      <Folder name="CVTrack-scenes">
        <Composition id="Intro" component={IntroScene} durationInFrames={150} fps={fps} width={width} height={height} />
        <Composition id="GazeboSimulation" component={GazeboSimulationScene} durationInFrames={840} fps={fps} width={width} height={height} />
        <Composition id="AirportReplay" component={AirportReplayScene} durationInFrames={840} fps={fps} width={width} height={height} />
        <Composition id="ParkingReplay" component={ParkingReplayScene} durationInFrames={840} fps={fps} width={width} height={height} />
        <Composition id="Outro" component={OutroScene} durationInFrames={150} fps={fps} width={width} height={height} />
      </Folder>
      <Composition
        id="CVTrackThreeSceneDemo"
        component={CVTrackThreeSceneDemo}
        durationInFrames={2700}
        fps={fps}
        width={width}
        height={height}
      />
      <Composition
        id="ClosedLoopEvidenceDemo"
        component={ClosedLoopEvidenceDemo}
        durationInFrames={720}
        fps={fps}
        width={width}
        height={height}
      />
    </>
  );
};
