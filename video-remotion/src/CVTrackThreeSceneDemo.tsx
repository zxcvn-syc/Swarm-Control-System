import { TransitionSeries, linearTiming } from "@remotion/transitions";
import { fade } from "@remotion/transitions/fade";
import { AirportReplayScene } from "./scenes/AirportReplayScene";
import { GazeboSimulationScene } from "./scenes/GazeboSimulationScene";
import { IntroScene } from "./scenes/IntroScene";
import { OutroScene } from "./scenes/OutroScene";
import { ParkingReplayScene } from "./scenes/ParkingReplayScene";

const transitionTiming = linearTiming({ durationInFrames: 30 });

export const CVTrackThreeSceneDemo: React.FC = () => {
  return (
    <TransitionSeries>
      <TransitionSeries.Sequence name="Opening" durationInFrames={150}>
        <IntroScene />
      </TransitionSeries.Sequence>
      <TransitionSeries.Transition presentation={fade()} timing={transitionTiming} />
      <TransitionSeries.Sequence name="PX4 Gazebo simulation" durationInFrames={840}>
        <GazeboSimulationScene />
      </TransitionSeries.Sequence>
      <TransitionSeries.Transition presentation={fade()} timing={transitionTiming} />
      <TransitionSeries.Sequence name="Airport tracking replay" durationInFrames={840}>
        <AirportReplayScene />
      </TransitionSeries.Sequence>
      <TransitionSeries.Transition presentation={fade()} timing={transitionTiming} />
      <TransitionSeries.Sequence name="Overhead parking replay" durationInFrames={840}>
        <ParkingReplayScene />
      </TransitionSeries.Sequence>
      <TransitionSeries.Transition presentation={fade()} timing={transitionTiming} />
      <TransitionSeries.Sequence name="Evidence summary" durationInFrames={150}>
        <OutroScene />
      </TransitionSeries.Sequence>
    </TransitionSeries>
  );
};
