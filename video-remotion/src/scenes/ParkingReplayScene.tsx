import { FootageScene } from "../components/FootageScene";

export const ParkingReplayScene: React.FC = () => {
  return (
    <FootageScene
      accent="#fbbf24"
      footer="YOLOv8 TRACKING OVERLAY / VIDEO REPLAY / NO FLIGHT CLAIM"
      index="SCENE 03"
      source="media/parking_tracked.mp4"
      status="TRACKING REPLAY"
      subtitle="Overhead vehicle detections and parking occupancy visual"
      title="OVERHEAD VEHICLE TRACKING"
    />
  );
};
