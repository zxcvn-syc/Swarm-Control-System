"""Run a perception-driven multi-UAV tracking and interception scene in RflySim.

The control loop consumes only RGB detections from Rfly VisionCaptureApi. Scene
state is used to draw the diagnostic overview and to verify separation after a
run; it is never supplied to the tracking or interception controller.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from queue import Full, Queue
import sys
from threading import Thread
import time
import types

import cv2
import numpy as np

try:
    from .model import ClosedSplineRoute, GuidedGroundVehicle, PlanarState, RouteVehicle, SmoothUav, clamp, magnitude
except ImportError:
    from model import ClosedSplineRoute, GuidedGroundVehicle, PlanarState, RouteVehicle, SmoothUav, clamp, magnitude


TARGET_ID = 100
UAV_IDS = (1, 2, 3)
GROUND_IDS = (101, 102, 103)
MOVING_OBSTACLE_IDS = (401, 402)
WEATHER_CONTROLLER_ID = 1300
TARGET_VEHICLE_TYPE = 125000051
GROUND_VEHICLE_TYPE = 51
TARGET_VEHICLE_SCALE = 0.9
MOVING_OBSTACLE_TYPE = 51
CAMERA_FOV_DEG = 95.0
CAMERA_DEPRESSION_DEG = 84.0
CAMERA_FORWARD_BIAS_RATIO = 0.0
CAMERA_RIGHT_BIAS_RATIO = 0.0
SCENE_EXTENT = (0.0, 220.0, 0.0, 220.0)
SCENE_OBJECT_IDS = (TARGET_ID, *GROUND_IDS, *MOVING_OBSTACLE_IDS, 501, 502, 503, 104, 9000, 9001)
DEFAULT_WEATHER_PROFILES = (("CLEAR", 0), ("RAIN", 5), ("STORM", 6), ("FOG", 7))


@dataclass
class Detection:
    x1: int
    y1: int
    x2: int
    y2: int
    score: float
    source: str

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)

    @property
    def width(self) -> int:
        return max(0, self.x2 - self.x1)

    @property
    def height(self) -> int:
        return max(0, self.y2 - self.y1)


@dataclass
class VisualEstimate:
    x: float
    y: float
    vx: float
    vy: float
    confidence: float
    last_seen: float
    detection: Detection | None = None

    def prediction(self, lead_seconds: float) -> tuple[float, float]:
        return self.x + self.vx * lead_seconds, self.y + self.vy * lead_seconds


class VehicleDetector:
    """Associates the uniquely blue native target vehicle across RGB frames."""

    def __init__(self, weights: str | None, enable_yolo: bool, allow_blue_candidates: bool = True) -> None:
        self.previous_center: tuple[float, float] | None = None
        self.previous_size: tuple[int, int] | None = None
        self.allow_blue_candidates = allow_blue_candidates
        if enable_yolo:
            print("[detector] RGB blue-target association enabled; generic COCO YOLO is not reliable for this overhead scale")

    @staticmethod
    def _blue_candidates(frame: np.ndarray) -> list[Detection]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((85, 120, 55), dtype=np.uint8), np.array((125, 255, 255), dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image_height, image_width = frame.shape[:2]
        candidates: list[Detection] = []
        for contour in contours:
            x_value, y_value, width, height = cv2.boundingRect(contour)
            area = width * height
            if area < 70 or width < 8 or height < 7:
                continue
            aspect = width / max(height, 1)
            if aspect < 0.35 or aspect > 5.8:
                continue
            blue_ratio = float(np.count_nonzero(mask[y_value : y_value + height, x_value : x_value + width])) / float(area)
            if blue_ratio < 0.34:
                continue
            near_edge = x_value < 14 or y_value < 14 or x_value + width > image_width - 14 or y_value + height > image_height - 14
            if near_edge and (area < 160 or blue_ratio < 0.50):
                continue
            score = min(0.99, 0.34 + 0.070 * math.log1p(area) + 0.28 * blue_ratio)
            candidates.append(Detection(x_value, y_value, x_value + width, y_value + height, score, "blue-rgb"))
        return candidates

    @staticmethod
    def _blue_ratio(frame: np.ndarray, detection: Detection) -> float:
        height, width = frame.shape[:2]
        x1 = int(clamp(detection.x1, 0, width - 1))
        x2 = int(clamp(detection.x2, x1 + 1, width))
        y1 = int(clamp(detection.y1, 0, height - 1))
        y2 = int(clamp(detection.y2, y1 + 1, height))
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return 0.0
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((85, 120, 55), dtype=np.uint8), np.array((125, 255, 255), dtype=np.uint8))
        return float(np.count_nonzero(mask)) / float(mask.size)

    def reset_association(self) -> None:
        self.previous_center = None
        self.previous_size = None

    def detect(self, frame: np.ndarray) -> Detection | None:
        blue_candidates = self._blue_candidates(frame) if self.allow_blue_candidates else []
        if not blue_candidates:
            return None
        height, width = frame.shape[:2]
        center_default = (width * 0.5, height * 0.5)
        reference = self.previous_center or center_default
        scored: list[tuple[float, Detection]] = []
        for candidate in blue_candidates:
            center_x, center_y = candidate.center
            distance = magnitude(center_x - reference[0], center_y - reference[1]) / max(width, height)
            if self.previous_center is not None and distance > 0.75:
                continue
            if self.previous_size is not None:
                width_ratio = candidate.width / max(self.previous_size[0], 1)
                height_ratio = candidate.height / max(self.previous_size[1], 1)
                if width_ratio > 8.0 or height_ratio > 8.0 or width_ratio < 0.12 or height_ratio < 0.12:
                    continue
            blue_ratio = self._blue_ratio(frame, candidate)
            if blue_ratio < 0.34:
                continue
            area_bonus = min(candidate.width * candidate.height / float(width * height), 0.08)
            score = candidate.score + 0.72 * blue_ratio + area_bonus - 0.38 * distance
            scored.append((score, Detection(candidate.x1, candidate.y1, candidate.x2, candidate.y2, min(0.99, score), candidate.source)))
        if not scored:
            return None
        scored.sort(key=lambda item: item[0], reverse=True)
        selected = scored[0][1]
        self.previous_center = selected.center
        self.previous_size = (selected.width, selected.height)
        return selected


class VisualTrack:
    """World estimate reconstructed from one RGB detection and known camera pose."""

    _MAX_TARGET_SPEED = 9.0
    _MAX_MEASUREMENT_INNOVATION = 2.6

    def __init__(self) -> None:
        self.estimate: VisualEstimate | None = None

    @staticmethod
    def _project_to_ground(detection: Detection, frame_shape: tuple[int, int, int], uav: PlanarState) -> tuple[float, float]:
        height, width = frame_shape[:2]
        center_x, center_y = detection.center
        normalized_x = (center_x - width * 0.5) / (width * 0.5)
        normalized_y = (center_y - height * 0.5) / (height * 0.5)
        horizontal_half_fov = math.radians(CAMERA_FOV_DEG * 0.5)
        vertical_half_fov = math.atan(math.tan(horizontal_half_fov) * height / width)
        bearing = uav.yaw + math.atan(math.tan(horizontal_half_fov) * normalized_x)
        depression = math.radians(CAMERA_DEPRESSION_DEG) + math.atan(math.tan(vertical_half_fov) * normalized_y)
        depression = clamp(depression, math.radians(12.0), math.radians(88.0))
        ground_distance = uav.altitude / math.tan(depression)
        right_x = -math.sin(uav.yaw)
        right_y = math.cos(uav.yaw)
        optical_bias = uav.altitude * CAMERA_FORWARD_BIAS_RATIO
        lateral_bias = uav.altitude * CAMERA_RIGHT_BIAS_RATIO
        return (
            uav.x + ground_distance * math.cos(bearing) - optical_bias * math.cos(uav.yaw) - lateral_bias * right_x,
            uav.y + ground_distance * math.sin(bearing) - optical_bias * math.sin(uav.yaw) - lateral_bias * right_y,
        )

    def update(self, detection: Detection, frame_shape: tuple[int, int, int], uav: PlanarState, timestamp: float) -> VisualEstimate:
        x_value, y_value = self._project_to_ground(detection, frame_shape, uav)
        if self.estimate is None:
            self.estimate = VisualEstimate(x_value, y_value, 0.0, 0.0, detection.score, timestamp, detection)
            return self.estimate
        delta_time = max(timestamp - self.estimate.last_seen, 1.0 / 60.0)
        predicted_x = self.estimate.x + self.estimate.vx * delta_time
        predicted_y = self.estimate.y + self.estimate.vy * delta_time
        innovation_x = x_value - predicted_x
        innovation_y = y_value - predicted_y
        innovation_size = magnitude(innovation_x, innovation_y)
        if innovation_size > self._MAX_MEASUREMENT_INNOVATION:
            scale = self._MAX_MEASUREMENT_INNOVATION / innovation_size
            innovation_x *= scale
            innovation_y *= scale
        filtered_x = predicted_x + 0.32 * innovation_x
        filtered_y = predicted_y + 0.32 * innovation_y
        raw_vx = (filtered_x - self.estimate.x) / delta_time
        raw_vy = (filtered_y - self.estimate.y) / delta_time
        raw_speed = magnitude(raw_vx, raw_vy)
        if raw_speed > self._MAX_TARGET_SPEED:
            raw_vx *= self._MAX_TARGET_SPEED / raw_speed
            raw_vy *= self._MAX_TARGET_SPEED / raw_speed
        self.estimate = VisualEstimate(
            x=filtered_x,
            y=filtered_y,
            vx=0.24 * raw_vx + 0.76 * self.estimate.vx,
            vy=0.24 * raw_vy + 0.76 * self.estimate.vy,
            confidence=0.55 * detection.score + 0.45 * self.estimate.confidence,
            last_seen=timestamp,
            detection=detection,
        )
        return self.estimate

    def fresh(self, timestamp: float, timeout: float = 2.5) -> VisualEstimate | None:
        if self.estimate is None or timestamp - self.estimate.last_seen > timeout:
            return None
        return self.estimate


class RflyVision:
    def __init__(self, sdk_root: Path, config_path: Path) -> None:
        for component in (sdk_root, sdk_root / "ue", sdk_root / "vision", sdk_root / "ctrl", sdk_root / "comm", sdk_root / "swarm"):
            component_string = str(component)
            if component_string not in sys.path:
                sys.path.insert(0, component_string)
        sys.modules.setdefault("Open3DShow", types.ModuleType("Open3DShow"))
        from VisionCaptureApi import VisionCaptureApi

        self._vision = VisionCaptureApi()
        if not self._vision.jsonLoad(jsonPath=str(config_path)):
            raise RuntimeError(f"cannot load Rfly vision configuration: {config_path}")
        self.camera_uav_ids = tuple(int(sensor.TargetCopter) for sensor in self._vision.VisSensor)
        if not self.camera_uav_ids or len(set(self.camera_uav_ids)) != len(self.camera_uav_ids):
            raise RuntimeError("Rfly vision configuration must bind each camera to one distinct UAV")
        unknown_uavs = set(self.camera_uav_ids).difference(UAV_IDS)
        if unknown_uavs:
            raise RuntimeError(f"Rfly vision configuration contains unsupported UAV IDs: {sorted(unknown_uavs)}")
        self._vision.sendReqToUE4(0)
        self._vision.startImgCap()
        self._last_timestamps = [-1.0] * len(self._vision.Img)

    def read(self, camera_index: int) -> np.ndarray | None:
        if camera_index >= len(self._vision.Img) or not self._vision.hasData[camera_index]:
            return None
        timestamp = float(self._vision.timeStmp[camera_index])
        if timestamp <= self._last_timestamps[camera_index]:
            return None
        self._last_timestamps[camera_index] = timestamp
        lock = self._vision.Img_lock[camera_index]
        lock.acquire()
        try:
            frame = self._vision.Img[camera_index]
            if frame is None:
                return None
            # Rfly TypeID=1 payloads are RGB despite the legacy API advertising bgr8.
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        finally:
            lock.release()

    def close(self) -> None:
        try:
            self._vision.stopRun()
        except AttributeError:
            pass
        time.sleep(0.3)

    def camera_index_for_uav(self, uav_id: int) -> int:
        return self.camera_uav_ids.index(uav_id)


class AsyncVideoWriter:
    def __init__(self, path: Path, frame_size: tuple[int, int], queue_size: int = 90) -> None:
        self._writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 25.0, frame_size)
        if not self._writer.isOpened():
            raise RuntimeError(f"OpenCV could not open MP4 writer: {path}")
        self._queue: Queue[np.ndarray | None] = Queue(maxsize=queue_size)
        self.dropped_frames = 0
        self._thread = Thread(target=self._run, name=f"video-writer-{path.stem}", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while True:
            frame = self._queue.get()
            try:
                if frame is None:
                    return
                self._writer.write(frame)
            finally:
                self._queue.task_done()

    def write(self, frame: np.ndarray) -> None:
        try:
            self._queue.put_nowait(frame.copy())
        except Full:
            self.dropped_frames += 1

    def close(self) -> None:
        self._queue.put(None)
        self._thread.join()
        self._writer.release()


class RflyScene:
    def __init__(
        self,
        sdk_root: Path,
        window_id: int,
        weather_interval: float,
        map_name: str,
        weather_enabled: bool,
        with_obstacles: bool,
        weather_profiles: tuple[tuple[str, int], ...],
    ) -> None:
        for component in (sdk_root, sdk_root / "ue", sdk_root / "ctrl", sdk_root / "comm", sdk_root / "swarm"):
            component_string = str(component)
            if component_string not in sys.path:
                sys.path.insert(0, component_string)
        from UE4CtrlAPI import UE4CtrlAPI

        self._ue = UE4CtrlAPI()
        self.window_id = window_id
        self.weather_interval = weather_interval
        self.map_name = map_name
        self.weather_enabled = weather_enabled
        self.with_obstacles = with_obstacles
        self.weather_profiles = weather_profiles
        self.weather_index = -1
        self._last_god_view_update = -1.0
        self._last_scene_update = -1.0
        self._last_moving_obstacle_update = -1.0
        self._sized_uav_ids: set[int] = set()

    def _command(self, command: str) -> None:
        self._ue.sendUE4Cmd(command, self.window_id)

    def bootstrap(self) -> None:
        self._ue.sendUE4Destroy(WEATHER_CONTROLLER_ID, windowID=self.window_id)
        for object_id in SCENE_OBJECT_IDS:
            self._ue.sendUE4Destroy(object_id, windowID=self.window_id)
        time.sleep(0.5)
        self._command(f"RflyChangeMapbyName {self.map_name}")
        time.sleep(4.0)
        self._command("r.setres 1280x720w")
        self._command("t.MaxFPS 60")
        self._command("RflyChangeViewKeyCmd N 6")
        self._command("RflyChangeViewKeyCmd S -1")
        self._command("RflyChangeViewKeyCmd T 0")
        self._command("RflyChangeViewKeyCmd L 1")
        if self.weather_enabled:
            self._ue.sendUE4PosNew(WEATHER_CONTROLLER_ID, 804, [0.0, 0.0, -8.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0] * 8, windowID=self.window_id)
        if self.with_obstacles:
            self._ue.sendUE4PosScale2Ground(501, GROUND_VEHICLE_TYPE, 0.0, [96.0, 84.0, -100.0], [0.0, 0.0, 0.2], [3.2, 3.2, 3.2], windowID=self.window_id)
            self._ue.sendUE4PosScale2Ground(502, GROUND_VEHICLE_TYPE, 0.0, [145.0, 112.0, -100.0], [0.0, 0.0, 1.1], [3.2, 3.2, 3.2], windowID=self.window_id)
            self._ue.sendUE4PosScale2Ground(503, GROUND_VEHICLE_TYPE, 0.0, [62.0, 148.0, -100.0], [0.0, 0.0, 0.0], [3.2, 3.2, 3.2], windowID=self.window_id)
        for uav_id in UAV_IDS:
            self._command(f"RflyChange3DModel {uav_id} 3")
            self._command(f"RflyChangeVehicleSize {uav_id} 0.15")
            self._command(f"RflySetIDLabel {uav_id} U{uav_id} #FFFFFF 28")
        self._command(f"RflySetIDLabel {TARGET_ID} TARGET #0080FF 30")
        for ground_id in GROUND_IDS:
            self._command(f"RflySetIDLabel {ground_id} UGV #A0A0A0 20")

    def set_weather(self, elapsed: float) -> tuple[str, int]:
        if not self.weather_enabled:
            return "CLEAR", 0
        index = int(elapsed // self.weather_interval) % len(self.weather_profiles)
        if index != self.weather_index:
            self.weather_index = index
            weather_name, weather_type = self.weather_profiles[index]
            self._ue.sendUE4ExtAct(WEATHER_CONTROLLER_ID, [weather_type] + [0.0] * 15, windowID=self.window_id)
            print(f"[scene] weather={weather_name}")
        return self.weather_profiles[index]

    def push(
        self,
        target: PlanarState,
        uavs: dict[int, SmoothUav],
        ground_vehicles: dict[int, GuidedGroundVehicle],
        moving_obstacles: dict[int, RouteVehicle],
        elapsed: float,
        god_view_state: PlanarState,
    ) -> None:
        if elapsed - self._last_scene_update >= 1.0 / 15.0:
            self._last_scene_update = elapsed
            self._ue.sendUE4PosScale2Ground(
                TARGET_ID,
                TARGET_VEHICLE_TYPE,
                0.0,
                [target.x, target.y, -100.0],
                [0.0, 0.0, target.yaw],
                [TARGET_VEHICLE_SCALE] * 3,
                windowID=self.window_id,
            )
            for uav_id, uav in uavs.items():
                state = uav.state
                roll = clamp(-0.025 * state.speed * math.sin(state.yaw), -0.24, 0.24)
                pitch = clamp(0.025 * state.speed * math.cos(state.yaw), -0.24, 0.24)
                self._ue.sendUE4PosNew(
                    uav_id,
                    3,
                    [state.x, state.y, -state.altitude],
                    [roll, pitch, state.yaw],
                    [0.0, 0.0, 0.0],
                    [1100.0] * 4 + [0.0] * 4,
                    windowID=self.window_id,
                )
                if uav_id not in self._sized_uav_ids:
                    self._command(f"RflyChangeVehicleSize {uav_id} 0.15")
                    self._sized_uav_ids.add(uav_id)
            for ground_id, vehicle in ground_vehicles.items():
                state = vehicle.state
                self._ue.sendUE4PosScale2Ground(ground_id, GROUND_VEHICLE_TYPE, 0.0, [state.x, state.y, -100.0], [0.0, 0.0, state.yaw], [1.2, 1.2, 1.2], windowID=self.window_id)
            if self.with_obstacles and elapsed - self._last_moving_obstacle_update >= 1.0 / 8.0:
                self._last_moving_obstacle_update = elapsed
                for obstacle_id, obstacle in moving_obstacles.items():
                    state = obstacle.state
                    self._ue.sendUE4PosScale2Ground(obstacle_id, MOVING_OBSTACLE_TYPE, 0.0, [state.x, state.y, -100.0], [0.0, 0.0, state.yaw], [1.2, 1.2, 1.2], windowID=self.window_id)
        if elapsed - self._last_god_view_update >= 0.2:
            self._last_god_view_update = elapsed
            yaw_degrees = math.degrees(god_view_state.yaw)
            self._command(
                f"RflyCameraPosAng {god_view_state.x - 30.0:.2f} {god_view_state.y - 18.0:.2f} {-max(god_view_state.altitude + 58.0, 84.0):.2f} 0 -68 {yaw_degrees:.2f}"
            )


class MissionController:
    def __init__(self) -> None:
        self.uavs = {
            1: SmoothUav(PlanarState(22.0, 45.5, -0.24, altitude=54.0), maximum_speed=11.5, maximum_acceleration=3.8),
            2: SmoothUav(PlanarState(19.0, 62.0, -0.75, altitude=42.0), maximum_speed=11.5, maximum_acceleration=3.8),
            3: SmoothUav(PlanarState(48.0, 52.0, -0.75, altitude=42.0), maximum_speed=11.5, maximum_acceleration=3.8),
        }
        self.ground_vehicles = {
            101: GuidedGroundVehicle(PlanarState(184.0, 18.0)),
            102: GuidedGroundVehicle(PlanarState(188.0, 186.0)),
            103: GuidedGroundVehicle(PlanarState(16.0, 184.0)),
        }
        self.tracks = {uav_id: VisualTrack() for uav_id in UAV_IDS}
        self.active_uav_id = 1
        self.mode = "SEARCH"
        self.last_transition = "bootstrap"
        self.handoff_events: list[str] = []
        self._last_active_change = 0.0
        self._search_phase = 0.0
        self._started_at: float | None = None
        self._acquisition_hits = 0
        self._last_acquisition_hit: float | None = None
        self.target_released = False

    def observe(self, uav_id: int, detection: Detection | None, frame: np.ndarray, timestamp: float) -> None:
        if detection is not None:
            self.tracks[uav_id].update(detection, frame.shape, self.uavs[uav_id].state, timestamp)
            if not self.target_released:
                if self._last_acquisition_hit is None or timestamp - self._last_acquisition_hit <= 0.32:
                    self._acquisition_hits += 1
                else:
                    self._acquisition_hits = 1
                self._last_acquisition_hit = timestamp
                if self._acquisition_hits >= 5:
                    self.target_released = True
                    self.last_transition = "RGB lock confirmed; target released"
                    print(f"[mission] {self.last_transition}")

    def _select_active_uav(self, timestamp: float) -> VisualEstimate | None:
        current = self.tracks[self.active_uav_id].fresh(timestamp)
        candidates = [(uav_id, self.tracks[uav_id].fresh(timestamp)) for uav_id in UAV_IDS]
        candidates = [(uav_id, estimate) for uav_id, estimate in candidates if estimate is not None]
        if current is not None:
            return current
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[1].confidence, item[1].last_seen), reverse=True)
        new_active_id, estimate = candidates[0]
        if new_active_id != self.active_uav_id:
            self.last_transition = f"handoff U{self.active_uav_id}->U{new_active_id}"
            self.handoff_events.append(self.last_transition)
            self.active_uav_id = new_active_id
            self._last_active_change = timestamp
            print(f"[mission] {self.last_transition}")
        return estimate

    def requires_reassociation(self, timestamp: float) -> bool:
        estimate = self.tracks[self.active_uav_id].estimate
        return estimate is not None and timestamp - estimate.last_seen > 0.55

    def step(self, timestamp: float, delta_time: float, weather_type: int) -> VisualEstimate | None:
        if self._started_at is None:
            self._started_at = timestamp
        if not self.target_released:
            self.mode = "ACQUIRE"
            for uav in self.uavs.values():
                state = uav.state
                uav.step(state.x, state.y, state.altitude, state.yaw, delta_time)
            return self._select_active_uav(timestamp)
        estimate = self._select_active_uav(timestamp)
        wind_strength = 0.0 if weather_type <= 0 else 0.35 + 0.18 * weather_type
        if estimate is not None:
            previous_mode = self.mode
            self.mode = "TRACK" if timestamp - estimate.last_seen < 0.22 else "COAST"
            if previous_mode != self.mode:
                self.last_transition = f"{previous_mode}->{self.mode}"
            predicted_x, predicted_y = estimate.prediction(0.28)
            follow_yaw = self.uavs[self.active_uav_id].state.yaw
            detection = estimate.detection
            bbox_fraction = 0.0 if detection is None else detection.height / 360.0
            active_altitude = clamp(60.0 + 14.0 * (0.14 - bbox_fraction), 56.0, 68.0)
            for index, uav_id in enumerate(UAV_IDS):
                if uav_id == self.active_uav_id:
                    standoff = 16.0
                    lateral = 0.0
                    altitude = active_altitude
                else:
                    standoff = 38.0
                    lateral = (-1.0 if index == 1 else 1.0) * 26.0
                    altitude = clamp(active_altitude + 9.0, 28.0, 62.0)
                forward_x = math.cos(follow_yaw)
                forward_y = math.sin(follow_yaw)
                lateral_x = -forward_y
                lateral_y = forward_x
                desired_x = predicted_x - standoff * forward_x + lateral * lateral_x
                desired_y = predicted_y - standoff * forward_y + lateral * lateral_y
                wind_x = wind_strength * math.sin(0.73 * timestamp + uav_id)
                wind_y = wind_strength * math.cos(0.57 * timestamp + uav_id)
                self.uavs[uav_id].step(desired_x, desired_y, altitude, follow_yaw, delta_time, wind_x, wind_y)
            ring_radius = 19.0
            for index, (ground_id, vehicle) in enumerate(self.ground_vehicles.items()):
                angle = follow_yaw + index * 2.0 * math.pi / len(self.ground_vehicles)
                vehicle.step(predicted_x + ring_radius * math.cos(angle), predicted_y + ring_radius * math.sin(angle), delta_time)
            return estimate
        previous_mode = self.mode
        self.mode = "SEARCH"
        if previous_mode != self.mode:
            self.last_transition = f"{previous_mode}->SEARCH"
        self._search_phase += delta_time * 0.45
        for index, uav_id in enumerate(UAV_IDS):
            uav = self.uavs[uav_id]
            radius = 18.0 + index * 12.0
            angle = self._search_phase + index * 2.0 * math.pi / len(UAV_IDS)
            desired_x = 110.0 + radius * math.cos(angle)
            desired_y = 110.0 + radius * math.sin(angle)
            uav.step(desired_x, desired_y, min(uav.state.altitude + 0.45, 68.0), angle + math.pi * 0.5, delta_time)
        return None


def route_set() -> tuple[ClosedSplineRoute, ClosedSplineRoute, ClosedSplineRoute]:
    target_route = ClosedSplineRoute(((28, 44), (82, 31), (158, 48), (191, 92), (166, 161), (98, 188), (36, 155), (22, 93)))
    obstacle_route_one = ClosedSplineRoute(((70, 68), (112, 62), (150, 85), (142, 122), (98, 135), (62, 110)))
    obstacle_route_two = ClosedSplineRoute(((52, 120), (94, 102), (151, 126), (164, 159), (123, 174), (69, 157)))
    return target_route, obstacle_route_one, obstacle_route_two


def draw_dashboard(
    frame: np.ndarray,
    active_detection: Detection | None,
    mission: MissionController,
    estimate: VisualEstimate | None,
    target: PlanarState,
    moving_obstacles: dict[int, RouteVehicle],
    weather_name: str,
    elapsed: float,
    target_route: ClosedSplineRoute,
) -> np.ndarray:
    video_frame = frame.copy()
    if active_detection is not None:
        cv2.rectangle(video_frame, (active_detection.x1, active_detection.y1), (active_detection.x2, active_detection.y2), (255, 180, 0), 2)
        cv2.putText(video_frame, f"TARGET {active_detection.score:.2f} {active_detection.source}", (active_detection.x1, max(20, active_detection.y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 180, 0), 1, cv2.LINE_AA)
    cv2.putText(video_frame, f"RFLY RGB / MASTER U{mission.active_uav_id}", (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(video_frame, f"MODE {mission.mode} | WEATHER {weather_name}", (14, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
    height, width = video_frame.shape[:2]
    left = cv2.resize(video_frame, (820, 462), interpolation=cv2.INTER_AREA)
    panel = np.full((462, 460, 3), (24, 31, 38), dtype=np.uint8)
    min_x, max_x, min_y, max_y = SCENE_EXTENT
    def convert(x_value: float, y_value: float) -> tuple[int, int]:
        return (int(20 + (x_value - min_x) / (max_x - min_x) * 420), int(442 - (y_value - min_y) / (max_y - min_y) * 420))
    route_points = [convert(x_value, y_value) for x_value, y_value in target_route.polyline()]
    cv2.polylines(panel, [np.array(route_points, dtype=np.int32)], True, (80, 88, 96), 1, cv2.LINE_AA)
    for obstacle in moving_obstacles.values():
        point = convert(obstacle.state.x, obstacle.state.y)
        cv2.rectangle(panel, (point[0] - 9, point[1] - 7), (point[0] + 9, point[1] + 7), (55, 130, 180), -1)
    target_point = convert(target.x, target.y)
    cv2.circle(panel, target_point, 7, (255, 120, 0), -1)
    for uav_id, uav in mission.uavs.items():
        point = convert(uav.state.x, uav.state.y)
        colour = (80, 235, 255) if uav_id == mission.active_uav_id else (210, 210, 210)
        cv2.circle(panel, point, 6, colour, -1)
        cv2.putText(panel, f"U{uav_id}", (point[0] + 8, point[1] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.38, colour, 1, cv2.LINE_AA)
    for vehicle in mission.ground_vehicles.values():
        point = convert(vehicle.state.x, vehicle.state.y)
        cv2.rectangle(panel, (point[0] - 5, point[1] - 4), (point[0] + 5, point[1] + 4), (155, 155, 155), -1)
    if estimate is not None:
        estimate_point = convert(estimate.x, estimate.y)
        prediction = estimate.prediction(0.9)
        prediction_point = convert(*prediction)
        cv2.circle(panel, estimate_point, 10, (0, 225, 255), 1)
        cv2.arrowedLine(panel, estimate_point, prediction_point, (0, 225, 255), 2, cv2.LINE_AA, tipLength=0.2)
    cv2.putText(panel, "GOD VIEW / SCENE DIAGNOSTIC", (18, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (238, 238, 238), 1, cv2.LINE_AA)
    details = [
        f"t = {elapsed:05.1f}s",
        f"state = {mission.last_transition}",
        f"track = {'live' if estimate is not None else 'search'}",
        f"prediction = {'on' if estimate is not None else 'hold'}",
    ]
    if estimate is not None:
        details.append(f"speed = {magnitude(estimate.vx, estimate.vy):.1f} m/s")
        details.append(f"confidence = {estimate.confidence:.2f}")
    for index, detail in enumerate(details):
        cv2.putText(panel, detail, (18, 330 + index * 21), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (218, 218, 218), 1, cv2.LINE_AA)
    dashboard = np.concatenate((left, panel), axis=1)
    return cv2.copyMakeBorder(dashboard, 40, 38, 0, 0, cv2.BORDER_CONSTANT, value=(14, 18, 22))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Native Rfly RGB tracking, handoff, weather, occlusion and interception demonstration")
    parser.add_argument("--rfly-sdk", type=Path, default=Path(r"F:\RflySimAPIs\RflySimSDK"))
    parser.add_argument("--vision-config", type=Path, default=Path(__file__).with_name("config") / "vision_downward.json")
    parser.add_argument("--output", type=Path, default=Path("output") / "rfly_native")
    parser.add_argument("--duration", type=float, default=96.0)
    parser.add_argument("--weather-interval", type=float, default=24.0)
    parser.add_argument("--weather-profiles", default=",".join(f"{name}:{weather_type}" for name, weather_type in DEFAULT_WEATHER_PROFILES))
    parser.add_argument("--map-name", default="Grasslands")
    parser.add_argument("--weather", action="store_true")
    parser.add_argument("--window-id", type=int, default=0)
    parser.add_argument("--weights", type=Path)
    parser.add_argument("--no-yolo", action="store_true")
    parser.add_argument("--no-obstacles", action="store_true")
    parser.add_argument("--no-record", action="store_true")
    return parser


def parse_weather_profiles(raw_profiles: str) -> tuple[tuple[str, int], ...]:
    profiles: list[tuple[str, int]] = []
    for raw_profile in raw_profiles.split(","):
        name, separator, type_text = raw_profile.partition(":")
        if not separator or not name.strip():
            raise ValueError(f"invalid weather profile: {raw_profile!r}; expected NAME:TYPE")
        weather_type = int(type_text)
        if weather_type < 0 or weather_type > 10:
            raise ValueError(f"weather type must be in [0, 10], got {weather_type}")
        profiles.append((name.strip().upper(), weather_type))
    if not profiles:
        raise ValueError("at least one weather profile is required")
    return tuple(profiles)


def resolve_local_weights(requested: Path | None) -> str | None:
    candidates = [] if requested is None else [requested]
    repository_root = Path(__file__).resolve().parents[4]
    candidates.extend((Path.cwd() / "yolov8s.pt", repository_root / "yolov8s.pt"))
    for candidate in candidates:
        if candidate.exists() and candidate.is_file():
            return str(candidate.resolve())
    return None


def frame_change_score(previous: np.ndarray | None, current: np.ndarray) -> float:
    if previous is None:
        return 0.0
    previous_small = cv2.resize(previous, (160, 90), interpolation=cv2.INTER_AREA)
    current_small = cv2.resize(current, (160, 90), interpolation=cv2.INTER_AREA)
    return float(np.mean(cv2.absdiff(previous_small, current_small)))


def wait_for_first_frame(vision: RflyVision, timeout: float) -> tuple[int, np.ndarray]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for camera_index in range(len(vision._last_timestamps)):
            frame = vision.read(camera_index)
            if frame is not None:
                print(f"[vision] camera U{vision.camera_uav_ids[camera_index]} active: {frame.shape[1]}x{frame.shape[0]}")
                return camera_index, frame
        time.sleep(0.03)
    raise RuntimeError("Rfly did not provide an RGB frame within the timeout")


def main() -> int:
    arguments = build_parser().parse_args()
    if arguments.duration <= 1.0:
        raise ValueError("duration must exceed one second")
    if not arguments.rfly_sdk.exists():
        raise FileNotFoundError(f"Rfly SDK does not exist: {arguments.rfly_sdk}")
    arguments.vision_config = arguments.vision_config.resolve()
    weather_profiles = parse_weather_profiles(arguments.weather_profiles) if arguments.weather else (("CLEAR", 0),)
    arguments.output.mkdir(parents=True, exist_ok=True)
    target_route, obstacle_route_one, obstacle_route_two = route_set()
    target_vehicle = RouteVehicle(target_route, initial_distance=0.0, cruise_speed=5.2, speed_variation=1.6, phase=0.6)
    moving_obstacles = {
        MOVING_OBSTACLE_IDS[0]: RouteVehicle(obstacle_route_one, 43.0, 5.0, 0.8, phase=1.5),
        MOVING_OBSTACLE_IDS[1]: RouteVehicle(obstacle_route_two, 120.0, 4.4, 0.7, phase=4.1),
    }
    vision: RflyVision | None = None
    scene = RflyScene(
        arguments.rfly_sdk,
        arguments.window_id,
        arguments.weather_interval,
        arguments.map_name,
        arguments.weather,
        not arguments.no_obstacles,
        weather_profiles,
    )
    scene.bootstrap()
    mission = MissionController()
    initial_weather_name, initial_weather_type = scene.set_weather(0.0)
    scene.push(target_vehicle.state, mission.uavs, mission.ground_vehicles, moving_obstacles, 0.0, mission.uavs[1].state)
    time.sleep(1.0)
    vision = RflyVision(arguments.rfly_sdk, arguments.vision_config)
    try:
        first_camera_index, first_frame = wait_for_first_frame(vision, 12.0)
    except BaseException:
        vision.close()
        raise
    detector = VehicleDetector(resolve_local_weights(arguments.weights), not arguments.no_yolo)
    initial_detection = detector.detect(first_frame)
    mission.observe(vision.camera_uav_ids[first_camera_index], initial_detection, first_frame, time.monotonic())
    raw_writer: AsyncVideoWriter | None = None
    dashboard_writer: AsyncVideoWriter | None = None
    if not arguments.no_record:
        raw_path = arguments.output / "rfly_native_rgb.mp4"
        dashboard_path = arguments.output / "rfly_native_dashboard.mp4"
        raw_writer = AsyncVideoWriter(raw_path, (first_frame.shape[1], first_frame.shape[0]))
        dashboard_writer = AsyncVideoWriter(dashboard_path, (1280, 540))
    trace_path = arguments.output / "trace.jsonl"
    start_time = time.monotonic()
    previous_time = start_time
    target_motion_elapsed = 0.0
    latest_frames: dict[int, np.ndarray] = {first_camera_index: first_frame}
    latest_detections: dict[int, Detection | None] = {first_camera_index: initial_detection}
    frame_count = 0
    rgb_frame_count = 1
    camera_frame_counts = {uav_id: 0 for uav_id in vision.camera_uav_ids}
    camera_detection_counts = {uav_id: 0 for uav_id in vision.camera_uav_ids}
    weather_frame_counts = {name: 0 for name, _ in weather_profiles}
    weather_detection_counts = {name: 0 for name, _ in weather_profiles}
    weather_luminance_sums = {name: 0.0 for name, _ in weather_profiles}
    camera_frame_counts[vision.camera_uav_ids[first_camera_index]] += 1
    if initial_detection is not None:
        camera_detection_counts[vision.camera_uav_ids[first_camera_index]] += 1
    active_rgb_frame_count = 1
    detection_count = 0
    active_detection_count = 0
    active_history: list[int] = []
    previous_active_frame: np.ndarray | None = None
    frame_change_scores: list[float] = []
    maximum_detection_gap = 0
    current_detection_gap = 0
    diagnostic_times = iter((8.0, 24.0, 48.0, 72.0))
    next_diagnostic_time = next(diagnostic_times, None)
    try:
        with trace_path.open("w", encoding="utf-8") as trace_file:
            while True:
                now = time.monotonic()
                elapsed = now - start_time
                if elapsed >= arguments.duration:
                    break
                delta_time = clamp(now - previous_time, 1.0 / 60.0, 0.08)
                previous_time = now
                if mission.target_released:
                    target_motion_elapsed += delta_time
                    target_state = target_vehicle.step(target_motion_elapsed, delta_time)
                    for obstacle in moving_obstacles.values():
                        obstacle.step(target_motion_elapsed, delta_time)
                else:
                    target_state = target_vehicle.state
                weather_name, weather_type = scene.set_weather(elapsed)
                if mission.requires_reassociation(now):
                    detector.reset_association()
                active_frame_is_new = False
                for camera_index in range(len(vision._last_timestamps)):
                    frame = vision.read(camera_index)
                    if frame is None:
                        continue
                    uav_id = vision.camera_uav_ids[camera_index]
                    rgb_frame_count += 1
                    camera_frame_counts[uav_id] += 1
                    detection = detector.detect(frame)
                    mission.observe(uav_id, detection, frame, now)
                    latest_frames[camera_index] = frame
                    latest_detections[camera_index] = detection
                    if detection is not None:
                        detection_count += 1
                        camera_detection_counts[uav_id] += 1
                    if uav_id == mission.active_uav_id:
                        active_frame_is_new = True
                        active_rgb_frame_count += 1
                        if detection is not None:
                            active_detection_count += 1
                estimate = mission.step(now, delta_time, weather_type)
                active_history.append(mission.active_uav_id)
                active_index = vision.camera_index_for_uav(mission.active_uav_id)
                active_frame = latest_frames.get(active_index, first_frame)
                active_detection = latest_detections.get(active_index)
                scene.push(target_state, mission.uavs, mission.ground_vehicles, moving_obstacles, elapsed, mission.uavs[mission.active_uav_id].state)
                dashboard = draw_dashboard(active_frame, active_detection, mission, estimate, target_state, moving_obstacles if scene.with_obstacles else {}, weather_name, elapsed, target_route)
                change_score = frame_change_score(previous_active_frame, active_frame)
                if previous_active_frame is not None:
                    frame_change_scores.append(change_score)
                previous_active_frame = active_frame.copy()
                if active_frame_is_new and active_detection is None:
                    current_detection_gap += 1
                    maximum_detection_gap = max(maximum_detection_gap, current_detection_gap)
                elif active_frame_is_new:
                    current_detection_gap = 0
                if active_frame_is_new:
                    weather_frame_counts[weather_name] += 1
                    weather_luminance_sums[weather_name] += float(np.mean(active_frame))
                    if active_detection is not None:
                        weather_detection_counts[weather_name] += 1
                if raw_writer is not None:
                    raw_writer.write(active_frame)
                if dashboard_writer is not None:
                    dashboard_writer.write(dashboard)
                if next_diagnostic_time is not None and elapsed >= next_diagnostic_time:
                    cv2.imwrite(str(arguments.output / f"dashboard_t{int(next_diagnostic_time):02d}.png"), dashboard)
                    next_diagnostic_time = next(diagnostic_times, None)
                trace = {
                    "elapsed_s": round(elapsed, 3),
                    "mode": mission.mode,
                    "active_uav": mission.active_uav_id,
                    "weather": weather_name,
                    "target_released": mission.target_released,
                    "detected": active_detection is not None,
                    "detector": None if active_detection is None else active_detection.source,
                    "detection_box": None if active_detection is None else asdict(active_detection),
                    "frame_change_score": round(change_score, 4),
                    "rgb_luminance": round(float(np.mean(active_frame)), 2),
                    "estimate": None if estimate is None else asdict(estimate),
                    "uavs": {str(uav_id): asdict(uav.state) for uav_id, uav in mission.uavs.items()},
                    "ground_vehicles": {str(vehicle_id): asdict(vehicle.state) for vehicle_id, vehicle in mission.ground_vehicles.items()},
                    "target_truth_for_validation": asdict(target_state),
                }
                trace_file.write(json.dumps(trace, ensure_ascii=True) + "\n")
                frame_count += 1
                target_period = 1.0 / 25.0
                sleep_time = target_period - (time.monotonic() - now)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
    finally:
        if raw_writer is not None:
            raw_writer.close()
        if dashboard_writer is not None:
            dashboard_writer.close()
    detection_coverage = 0.0 if active_rgb_frame_count == 0 else active_detection_count / active_rgb_frame_count
    average_frame_change = 0.0 if not frame_change_scores else float(np.mean(frame_change_scores))
    frozen_frame_ratio = 0.0 if not frame_change_scores else sum(score < 0.18 for score in frame_change_scores) / len(frame_change_scores)
    achieved_fps = 0.0 if arguments.duration <= 0.0 else frame_count / arguments.duration
    all_requested_cameras_active = all(count > 0 for count in camera_frame_counts.values())
    multi_camera_handoff_verified = len(camera_frame_counts) > 1 and bool(mission.handoff_events)
    validation = {
        "target_released_after_rgb_lock": mission.target_released,
        "detection_coverage": round(detection_coverage, 3),
        "maximum_detection_gap_frames": maximum_detection_gap,
        "target_route_distance_m": round(target_motion_elapsed and target_vehicle.distance or 0.0, 2),
        "achieved_video_fps": round(achieved_fps, 2),
        "source_rgb_fps": round(rgb_frame_count / arguments.duration, 2),
        "active_rgb_frames": active_rgb_frame_count,
        "raw_recording_dropped_frames": 0 if raw_writer is None else raw_writer.dropped_frames,
        "dashboard_recording_dropped_frames": 0 if dashboard_writer is None else dashboard_writer.dropped_frames,
        "average_frame_change": round(average_frame_change, 4),
        "frozen_frame_ratio": round(frozen_frame_ratio, 3),
        "requested_camera_streams_active": all_requested_cameras_active,
    }
    validation["accepted"] = bool(
        validation["target_released_after_rgb_lock"]
        and detection_coverage >= 0.65
        and achieved_fps >= 18.0
        and validation["source_rgb_fps"] >= 10.0
        and validation["raw_recording_dropped_frames"] == 0
        and validation["dashboard_recording_dropped_frames"] == 0
        and frozen_frame_ratio < 0.85
        and target_motion_elapsed >= max(2.0, arguments.duration * 0.55)
        and all_requested_cameras_active
    )
    report = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "duration_requested_s": arguments.duration,
        "control_frames": frame_count,
        "rgb_detections": detection_count,
        "active_uavs_seen": sorted(set(active_history)),
        "camera_frames": {f"U{uav_id}": count for uav_id, count in camera_frame_counts.items()},
        "camera_detections": {f"U{uav_id}": count for uav_id, count in camera_detection_counts.items()},
        "handoff_events": mission.handoff_events,
        "all_requested_cameras_active": all_requested_cameras_active,
        "multi_camera_handoff_verified": multi_camera_handoff_verified,
        "weather_rgb_calibration": {
            name: {
                "active_rgb_frames": weather_frame_counts[name],
                "detection_coverage": round(weather_detection_counts[name] / weather_frame_counts[name], 3) if weather_frame_counts[name] else 0.0,
                "mean_luminance": round(weather_luminance_sums[name] / weather_frame_counts[name], 2) if weather_frame_counts[name] else 0.0,
            }
            for name, _ in weather_profiles
        },
        "weather_cycle": [name for name, _ in weather_profiles] if arguments.weather else ["CLEAR"],
        "obstacles_enabled": scene.with_obstacles,
        "target_identity": "The target is the only blue native Rfly vehicle in the scene. White ground vehicles and non-blue obstacles are excluded from RGB association.",
        "perception_boundary": "All control and interception inputs originate from VisionCaptureApi RGB detections. Target pose is recorded only for post-run validation and never enters the controller.",
        "validation": validation,
        "outputs": ["rfly_native_rgb.mp4", "rfly_native_dashboard.mp4", "trace.jsonl"],
    }
    (arguments.output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    sys.stdout.flush()
    if vision is not None:
        vision.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
