import { FootageScene } from "../components/FootageScene";

export const AirportReplayScene: React.FC = () => {
  return (
    <FootageScene
      accent="#6ee7b7"
      footer="YOLOv8 TRACKING OVERLAY / VIDEO REPLAY / NO FLIGHT CLAIM"
      index="SCENE 02"
      source="media/airport_tracked.mp4"
      status="TRACKING REPLAY"
      subtitle="Airport surface objects, trajectories and identifiers"
      title="AIRPORT SURFACE TRACKING"
    />
  );
};
