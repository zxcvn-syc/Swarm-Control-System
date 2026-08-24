import { FootageScene } from "../components/FootageScene";

export const GazeboSimulationScene: React.FC = () => {
  return (
    <FootageScene
      accent="#36d1dc"
      cropGazebo
      footer="PX4 SITL + GAZEBO CLASSIC / VISUAL SIMULATION ONLY"
      index="SCENE 01"
      source="media/gazebo_gui_final_20260820.mp4"
      status="SIMULATION"
      subtitle="Headless VM desktop captured through Xvfb"
      title="PARK PATROL / PX4 GAZEBO"
    />
  );
};
