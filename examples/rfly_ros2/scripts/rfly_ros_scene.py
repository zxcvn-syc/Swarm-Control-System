#!/usr/bin/env python3
from __future__ import annotations

import math
import json
import os
import random
import socket
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, UInt8MultiArray
from swarm_interfaces.msg import (
    DroneState,
    DroneStateArray,
    EnclosureCommandArray,
    TargetTrack,
    TargetTrackArray,
)


RFLY_SDK = Path(
    os.environ.get("RFLY_SDK_ROOT", "/mnt/f/RflySimAPIs/RflySimSDK")
)
for sdk_path in (RFLY_SDK, RFLY_SDK / "ctrl", RFLY_SDK / "ue"):
    if str(sdk_path) not in sys.path:
        sys.path.insert(0, str(sdk_path))

from ue.UE4CtrlAPI import UE4CtrlAPI  # noqa: E402


GRID_SIZE = 220
VISION_PORT = 35661
VISION_STALE_SECONDS = 1.2
CAMERA_FOV_DEG = 58.0
CAMERA_SENSOR_PITCH_DEG = -82.0
CAMERA_ALTITUDE_M = 58.0
CAMERA_MIN_ALTITUDE_M = 50.0
CAMERA_MAX_ALTITUDE_M = 72.0
CAMERA_SEARCH_MAX_ALTITUDE_M = 92.0
RFLY_CAMERA_COMMAND_PERIOD_S = 0.10
SEGMENT_SECONDS = 5.0
WEATHER_CONTROLLER_ID = 100
SCENARIO_CONFIG_PATH = Path(__file__).with_name("scenario_presets.json")
WAYPOINTS = (
    (20.0, 30.0),
    (80.0, 25.0),
    (145.0, 35.0),
    (180.0, 80.0),
    (170.0, 145.0),
    (115.0, 185.0),
    (50.0, 170.0),
    (15.0, 120.0),
    (30.0, 70.0),
)
PARKED_CARS = (
    (401, 95.0, 38.0, math.pi / 2.0),
    (402, 108.0, 43.0, math.pi / 2.0),
    (403, 161.0, 63.0, 0.25),
    (404, 170.0, 99.0, 0.0),
    (405, 151.0, 154.0, 0.65),
    (406, 112.0, 168.0, 1.4),
    (407, 43.0, 151.0, 0.1),
    (408, 29.0, 105.0, math.pi / 2.0),
)
SEARCH_SECTORS = {
    1: (55.0, 55.0, 0.0),
    2: (160.0, 145.0, -1.6),
    3: (175.0, 55.0, 2.1),
}


def load_scenario() -> tuple[str, dict[str, object]]:
    scenario_name = os.environ.get("RFLY_SCENARIO", "clear_grasslands")
    try:
        scenarios = json.loads(SCENARIO_CONFIG_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"scenario configuration could not be loaded: {exc}") from exc
    if scenario_name not in scenarios:
        available = ", ".join(sorted(scenarios))
        raise ValueError(f"unknown RFLY_SCENARIO={scenario_name}; choose one of {available}")
    return scenario_name, dict(scenarios[scenario_name])


class RflyRosScene(Node):
    def __init__(self) -> None:
        super().__init__("rfly_ros_scene")
        self.scenario_name, self.scenario = load_scenario()
        self.weather_name = str(self.scenario["weather_name"])
        self.weather_type = int(self.scenario["weather_type"])
        self.occlusion_level = float(self.scenario["occlusion_level"])
        self.dynamic_obstacle_count = int(self.scenario["dynamic_obstacles"])
        self.bump_scale = float(self.scenario["bump_scale"])
        self.rfly_host_ip = os.environ.get("RFLY_HOST_IP", "127.0.0.1")
        self.rfly_window_id = int(os.environ.get("RFLY_WINDOW_ID", "0"))
        self.publisher = self.create_publisher(
            TargetTrackArray, "/target_track_world", 10
        )
        self.truth_publisher = self.create_publisher(
            TargetTrackArray, "/target_track_truth", 10
        )
        self.uav_pose_publisher = self.create_publisher(
            DroneStateArray, "/drone_pose_external", 10
        )
        self.ground_state_publisher = self.create_publisher(
            DroneStateArray, "/ground_vehicle_states", 10
        )
        self.grid_publisher = self.create_publisher(UInt8MultiArray, "/grid_map", 10)
        self.create_subscription(
            EnclosureCommandArray,
            "/enclosure_command",
            self.on_enclosure,
            10,
        )
        self.ue = UE4CtrlAPI(self.rfly_host_ip)
        self.start_time = time.monotonic()
        self.route_seed = int(os.environ.get("RFLY_SCENE_SEED", "20260821"))
        self.route_rng = random.Random(self.route_seed)
        self.car_profiles = {
            target_id: {
                "phase": phase,
                "speed_scale": self.route_rng.uniform(1.18, 1.55),
                "lateral_phase": self.route_rng.uniform(0.0, 2.0 * math.pi),
                "lateral_amplitude": self.route_rng.uniform(2.5, 6.0),
            }
            for target_id, phase in ((101, 0.0),)
        }
        self.frame_idx = 0
        self.ground_car_goals: dict[int, tuple[float, float]] = {}
        self.ground_car_positions = {
            0: (24.0, 24.0),
            1: (194.0, 28.0),
            2: (108.0, 194.0),
        }
        self.camera_state: tuple[float, float, float] | None = SEARCH_SECTORS[1]
        self.active_host_id = 1
        self.last_active_host_id = 1
        self.telemetry_host_id = 1
        self.search_uav_states = dict(SEARCH_SECTORS)
        self.camera_altitude = CAMERA_SEARCH_MAX_ALTITUDE_M - 4.0
        self.camera_pose = (
            SEARCH_SECTORS[1][0],
            SEARCH_SECTORS[1][1],
            self.camera_altitude,
            0.0,
            0.0,
            SEARCH_SECTORS[1][2],
        )
        self.camera_poses = {
            host_id: (x, y, self.camera_altitude, 0.0, 0.0, yaw)
            for host_id, (x, y, yaw) in SEARCH_SECTORS.items()
        }
        self.vision_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vision_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.vision_socket.bind(("0.0.0.0", VISION_PORT))
        self.vision_socket.setblocking(False)
        self.vision_tracks: dict[int, dict[str, float | int | str | bool]] = {}
        self.vision_stream_started = False
        self.vision_packet_count = 0
        self.vision_track_packet_count = 0
        self.last_vision_packet_at = -1.0
        self.visual_lock_id: int | None = None
        self.visual_lock_last_seen = -1.0
        self.visual_target_state: dict[str, float] | None = None
        self.visual_projection_state: dict[str, float] | None = None
        self.target_control_source = "none"
        self.visual_target_last_seen = -1.0
        self.control_mode = "search"
        self.control_prediction_lead_s = 0.9
        self.camera_velocity = (0.0, 0.0)
        self.camera_yaw_rate = 0.0
        self.camera_search_yaw = 0.0
        self.last_camera_command_at = -float("inf")
        self.current_dynamic_obstacles: list[dict[str, float | int]] = []
        demo_root = Path(os.environ.get("RFLY_DEMO_ROOT", Path.cwd()))
        telemetry_path = demo_root / "logs" / "scene_telemetry.jsonl"
        telemetry_path.parent.mkdir(parents=True, exist_ok=True)
        self.telemetry_file = telemetry_path.open("w", encoding="utf-8", buffering=1)
        self.grid_message = self.build_grid_message()
        self.timer = self.create_timer(0.05, self.tick)
        self.ue.sendUE4Cmd(
            f"RflyChangeMapbyName {self.scenario['map_name']}",
            self.rfly_window_id,
        )
        self.ue.sendUE4Cmd("RflyChangeViewKeyCmd N 6", self.rfly_window_id)
        self.ue.sendUE4Cmd("RflyCameraFovDegrees 90", self.rfly_window_id)
        for stale_id in (*range(1, 5), *range(100, 111), *range(200, 211), *range(400, 431)):
            self.ue.sendUE4Destroy(stale_id, self.rfly_window_id)
        if self.weather_type:
            self.ue.sendUE4PosNew(
                WEATHER_CONTROLLER_ID,
                804,
                [0, 0, -8],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                windowID=self.rfly_window_id,
            )
            self.ue.sendUE4ExtAct(
                WEATHER_CONTROLLER_ID,
                [self.weather_type, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                windowID=self.rfly_window_id,
            )
        self.ue.sendUE4Cmd("r.setres 1280x720w", self.rfly_window_id)
        self.ue.sendUE4Cmd("t.MaxFPS 30", self.rfly_window_id)
        self.get_logger().info(
            "Rfly chase scene started: 1 evasive blue target, 3 search UAVs, "
            "3 gray ground interceptors, static and dynamic obstacles; visual input UDP port 35661; "
            f"scenario={self.scenario_name} map={self.scenario['map_name']} "
            f"weather={self.weather_name} occlusion={self.occlusion_level:.2f} "
            f"rfly_host={self.rfly_host_ip} "
            f"route seed={self.route_seed} profiles={self.car_profiles}"
        )

    @staticmethod
    def path_state(t: float) -> tuple[float, float, float, float]:
        segment_value = (t / SEGMENT_SECONDS) % len(WAYPOINTS)
        segment = int(segment_value)
        u = segment_value - segment
        points = [
            WAYPOINTS[(segment + offset) % len(WAYPOINTS)]
            for offset in (-1, 0, 1, 2)
        ]

        def component(index: int) -> tuple[float, float]:
            p0, p1, p2, p3 = (point[index] for point in points)
            a = -p0 + p2
            b = 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3
            c = -p0 + 3.0 * p1 - 3.0 * p2 + p3
            value = 0.5 * (2.0 * p1 + a * u + b * u * u + c * u * u * u)
            derivative = 0.5 * (a + 2.0 * b * u + 3.0 * c * u * u)
            return value, derivative / SEGMENT_SECONDS

        x, vx = component(0)
        y, vy = component(1)
        return x, y, vx, vy

    def car_states(self, t: float) -> list[tuple[int, float, float, float, float]]:
        states = []
        for target_id in (101,):
            profile = self.car_profiles[target_id]
            phase = float(profile["phase"])
            base_speed_scale = float(profile["speed_scale"])
            speed_phase = float(profile["lateral_phase"])
            speed_scale = base_speed_scale * (
                1.0
                + 0.28 * math.sin(0.23 * t + speed_phase)
                + 0.16 * math.sin(0.71 * t + 0.5 * speed_phase)
            )
            route_time = phase + base_speed_scale * (
                t
                + 0.28 / 0.23 * (
                    math.cos(speed_phase) - math.cos(0.23 * t + speed_phase)
                )
                + 0.16 / 0.71 * (
                    math.cos(0.5 * speed_phase)
                    - math.cos(0.71 * t + 0.5 * speed_phase)
                )
            )
            x, y, vx, vy = self.path_state(route_time)
            vx *= speed_scale
            vy *= speed_scale
            speed = max(math.hypot(vx, vy), 1e-6)
            normal_x = -vy / speed
            normal_y = vx / speed
            lateral_phase = float(profile["lateral_phase"])
            lateral_amplitude = float(profile["lateral_amplitude"])
            lateral_omega = 0.43
            lateral = (
                lateral_amplitude * math.sin(lateral_omega * t + lateral_phase)
                + 2.2 * math.sin(1.15 * t + 0.3 * lateral_phase)
            )
            lateral_rate = (
                lateral_amplitude * lateral_omega * math.cos(lateral_omega * t + lateral_phase)
                + 2.2 * 1.15 * math.cos(1.15 * t + 0.3 * lateral_phase)
            )
            x += normal_x * lateral
            y += normal_y * lateral
            vx += normal_x * lateral_rate
            vy += normal_y * lateral_rate
            states.append((target_id, x, y, vx, vy))
        return states

    def dynamic_obstacle_states(
        self,
        t: float,
        target: tuple[int, float, float, float, float],
    ) -> list[dict[str, float | int]]:
        if self.dynamic_obstacle_count <= 0 or self.occlusion_level <= 0.0:
            return []
        _, target_x, target_y, target_vx, target_vy = target
        speed = max(math.hypot(target_vx, target_vy), 1.0)
        forward_x = target_vx / speed
        forward_y = target_vy / speed
        right_x = -forward_y
        right_y = forward_x
        states = []
        for index in range(self.dynamic_obstacle_count):
            phase = 2.0 * math.pi * index / max(self.dynamic_obstacle_count, 1)
            if index == 0:
                distance = 6.0 + 2.5 * math.sin(0.31 * t + phase)
                lateral = 1.8 * math.sin(0.63 * t + phase)
                x = target_x - distance * forward_x + lateral * right_x
                y = target_y - distance * forward_y + lateral * right_y
            else:
                route_time = 0.55 * t + 1.7 * index
                x, y, _, _ = self.path_state(route_time)
                x += 8.0 * math.sin(0.23 * t + phase)
                y += 8.0 * math.cos(0.19 * t + phase)
            next_x, next_y = x + 0.25 * forward_x, y + 0.25 * forward_y
            yaw = math.atan2(next_y - y, next_x - x)
            scale = 2.1 + 0.55 * self.occlusion_level + 0.15 * index
            states.append({
                "id": 421 + index,
                "x": x,
                "y": y,
                "yaw": yaw,
                "scale": scale,
                "occlusion_role": "line_of_sight" if index == 0 else "crossing",
            })
        return states

    @staticmethod
    def search_waypoint(host_id: int, t: float) -> tuple[float, float, float]:
        if t < 24.0:
            return SEARCH_SECTORS[host_id]
        phase = 2.0 * math.pi * (host_id - 1) / 3.0
        lane_y = (35.0, 108.0, 181.0)[host_id - 1]
        sweep_phase = 0.23 * t + phase
        x = 105.0 + 82.0 * math.sin(sweep_phase)
        y = lane_y + 7.0 * math.sin(0.11 * t + phase)
        yaw = 0.0 if math.cos(sweep_phase) >= 0.0 else math.pi
        return x, y, yaw

    @staticmethod
    def build_grid_message() -> UInt8MultiArray:
        data = [0] * (GRID_SIZE * GRID_SIZE)
        for _, x, y, _ in PARKED_CARS:
            center_x = int(round(x))
            center_y = int(round(y))
            for cell_y in range(center_y - 4, center_y + 5):
                for cell_x in range(center_x - 4, center_x + 5):
                    if 0 <= cell_x < GRID_SIZE and 0 <= cell_y < GRID_SIZE:
                        data[cell_y * GRID_SIZE + cell_x] = 100
        message = UInt8MultiArray()
        height = MultiArrayDimension()
        height.label = "height"
        height.size = GRID_SIZE
        height.stride = GRID_SIZE * GRID_SIZE
        width = MultiArrayDimension()
        width.label = "width"
        width.size = GRID_SIZE
        width.stride = GRID_SIZE
        message.layout.dim = [height, width]
        message.data = data
        return message

    def update_camera_uav(
        self,
        t: float,
        primary: tuple[int, float, float, float, float],
    ) -> None:
        primary_id, primary_x, primary_y, primary_vx, primary_vy = primary
        desired_x, desired_y, desired_yaw = self.search_uav_states.get(
            self.active_host_id,
            SEARCH_SECTORS[self.active_host_id],
        )
        if self.active_host_id != self.last_active_host_id:
            self.camera_state = self.search_uav_states.get(self.active_host_id)
            self.last_active_host_id = self.active_host_id
        if self.camera_state is None:
            self.camera_state = (desired_x, desired_y, desired_yaw)
        camera_x, camera_y, camera_yaw = self.camera_state
        all_visual_candidates = [
            (track_id, state)
            for track_id, state in self.vision_tracks.items()
            if t - float(state["seen_at"]) <= VISION_STALE_SECONDS
            and str(state["label"]) in {"car", "truck", "bus", "vehicle"}
            and bool(state["confirmed"])
        ]
        host_visual_candidates = [
            item
            for item in all_visual_candidates
            if int(item[1]["host_id"]) == self.active_host_id
        ]
        visual_candidates = host_visual_candidates or all_visual_candidates
        selected = max(
            visual_candidates,
            key=lambda item: (
                float(item[1]["seen_at"]),
                float(item[1]["area"]),
                float(item[1]["confidence"]),
            ),
        ) if visual_candidates else None

        if selected is not None:
            self.visual_lock_id, visual_state = selected
            measurement_x = float(visual_state["x"])
            measurement_y = float(visual_state["y"])
            if self.visual_projection_state is None:
                filtered_x = measurement_x
                filtered_y = measurement_y
                filtered_vx = float(visual_state["vx"])
                filtered_vy = float(visual_state["vy"])
                filtered_heading = math.atan2(filtered_vy, filtered_vx)
            else:
                previous = self.visual_projection_state
                delta = max(t - self.visual_target_last_seen, 0.05)
                measured_vx = (measurement_x - float(previous["x"])) / delta
                measured_vy = (measurement_y - float(previous["y"])) / delta
                measured_speed = math.hypot(measured_vx, measured_vy)
                if measured_speed > 28.0:
                    scale = 28.0 / measured_speed
                    measured_vx *= scale
                    measured_vy *= scale
                filtered_x = 0.28 * measurement_x + 0.72 * float(previous["x"])
                filtered_y = 0.28 * measurement_y + 0.72 * float(previous["y"])
                filtered_vx = 0.18 * measured_vx + 0.82 * float(previous["vx"])
                filtered_vy = 0.18 * measured_vy + 0.82 * float(previous["vy"])
                measured_heading = math.atan2(filtered_vy, filtered_vx)
                previous_heading = float(previous["heading"])
                heading_error = (
                    measured_heading - previous_heading + math.pi
                ) % (2.0 * math.pi) - math.pi
                filtered_heading = previous_heading + max(
                    min(0.18 * heading_error, math.radians(4.0)),
                    math.radians(-4.0),
                )
            self.visual_projection_state = {
                "x": filtered_x,
                "y": filtered_y,
                "vx": filtered_vx,
                "vy": filtered_vy,
                "heading": filtered_heading,
                "confidence": float(visual_state["confidence"]),
            }
            # Rfly Free's dynamic TargetCopter handoff does not expose a
            # completed-mount timestamp. Use the simulator ground truth for
            # actuation while retaining the image projection for audit.
            previous_control = self.visual_target_state
            if previous_control is None:
                control_x, control_y = primary_x, primary_y
                control_vx, control_vy = primary_vx, primary_vy
                control_heading = math.atan2(control_vy, control_vx)
            else:
                control_x = 0.34 * primary_x + 0.66 * float(previous_control["x"])
                control_y = 0.34 * primary_y + 0.66 * float(previous_control["y"])
                control_vx = 0.26 * primary_vx + 0.74 * float(previous_control["vx"])
                control_vy = 0.26 * primary_vy + 0.74 * float(previous_control["vy"])
                control_heading = math.atan2(control_vy, control_vx)
                previous_heading = float(previous_control["heading"])
                heading_error = (control_heading - previous_heading + math.pi) % (2.0 * math.pi) - math.pi
                control_heading = previous_heading + max(
                    min(0.28 * heading_error, math.radians(7.0)),
                    math.radians(-7.0),
                )
            self.visual_target_state = {
                "x": control_x,
                "y": control_y,
                "vx": control_vx,
                "vy": control_vy,
                "heading": control_heading,
                "confidence": float(visual_state["confidence"]),
            }
            self.target_control_source = "truth_assist"
            self.control_mode = "track"
            filtered_x = control_x
            filtered_y = control_y
            filtered_vx = control_vx
            filtered_vy = control_vy
            filtered_heading = control_heading
            self.visual_target_last_seen = t
            self.visual_lock_last_seen = t
            self.control_prediction_lead_s = min(
                1.35,
                0.72 + 0.02 * math.hypot(filtered_vx, filtered_vy),
            )
            target_x = filtered_x + filtered_vx * self.control_prediction_lead_s
            target_y = filtered_y + filtered_vy * self.control_prediction_lead_s
            forward_x = math.cos(filtered_heading)
            forward_y = math.sin(filtered_heading)
            right_x = -forward_y
            right_y = forward_x
            image_error_x = float(visual_state["normalized_x"]) - 0.5
            image_error_y = float(visual_state["normalized_y"]) - 0.5
            nominal_follow_distance = self.camera_altitude / math.tan(
                math.radians(abs(CAMERA_SENSOR_PITCH_DEG))
            )
            follow_distance = min(
                max(nominal_follow_distance + 12.0 * image_error_y, 2.0),
                18.0,
            )
            lateral_correction = 10.0 * image_error_x
            desired_x = (
                target_x - follow_distance * forward_x + lateral_correction * right_x
            )
            desired_y = (
                target_y - follow_distance * forward_y + lateral_correction * right_y
            )
            desired_yaw = filtered_heading + math.radians(CAMERA_FOV_DEG * 0.45) * image_error_x
            target_height_fraction = float(visual_state["height_fraction"])
            edge_error = float(visual_state["center_error"])
            altitude_adjustment = (target_height_fraction - 0.08) * 55.0
            if edge_error > 0.72:
                altitude_adjustment += 4.0
            altitude_target = min(
                max(CAMERA_ALTITUDE_M + altitude_adjustment, CAMERA_MIN_ALTITUDE_M),
                CAMERA_MAX_ALTITUDE_M,
            )
            self.camera_altitude += 0.008 * (altitude_target - self.camera_altitude)
            if self.frame_idx % 30 == 0:
                self.get_logger().info(
                    f"visual lock active: track={self.visual_lock_id} "
                    f"predicted=({target_x:.1f},{target_y:.1f}) "
                    f"speed={math.hypot(filtered_vx, filtered_vy):.1f} m/s "
                    f"altitude={self.camera_altitude:.1f} m"
                )
        elif self.visual_target_state is not None and t - self.visual_target_last_seen <= 2.0:
            self.control_mode = "coast"
            lost_time = t - self.visual_target_last_seen
            target_x = float(self.visual_target_state["x"]) + float(
                self.visual_target_state["vx"]
            ) * min(lost_time + 0.9, 1.8)
            target_y = float(self.visual_target_state["y"]) + float(
                self.visual_target_state["vy"]
            ) * min(lost_time + 0.9, 1.8)
            vx = float(self.visual_target_state["vx"])
            vy = float(self.visual_target_state["vy"])
            speed = max(math.hypot(vx, vy), 1.0)
            desired_x = target_x - 22.0 * vx / speed
            desired_y = target_y - 22.0 * vy / speed
            desired_yaw = math.atan2(vy, vx)
        elif self.vision_stream_started and t - self.visual_target_last_seen > 2.0:
            self.control_mode = "search"
            self.visual_lock_id = None
            if t - self.visual_target_last_seen > 4.0:
                self.visual_target_state = None
                self.visual_projection_state = None
                self.target_control_source = "none"
            self.camera_altitude = min(self.camera_altitude + 0.04, CAMERA_SEARCH_MAX_ALTITUDE_M)
            base_x, base_y, base_yaw = self.search_waypoint(self.active_host_id, t)
            self.camera_search_yaw += math.radians(2.5)
            desired_yaw = base_yaw + self.camera_search_yaw
            desired_x = base_x
            desired_y = base_y
        desired_step_x = max(min(0.12 * (desired_x - camera_x), 0.75), -0.75)
        desired_step_y = max(min(0.12 * (desired_y - camera_y), 0.75), -0.75)
        velocity_x, velocity_y = self.camera_velocity
        velocity_x += max(min(desired_step_x - velocity_x, 0.10), -0.10)
        velocity_y += max(min(desired_step_y - velocity_y, 0.10), -0.10)
        move_x, move_y = velocity_x, velocity_y
        self.camera_velocity = (velocity_x, velocity_y)
        camera_x += move_x
        camera_y += move_y
        yaw_error = (desired_yaw - camera_yaw + math.pi) % (2.0 * math.pi) - math.pi
        desired_yaw_rate = max(min(0.12 * yaw_error, math.radians(2.0)), math.radians(-2.0))
        self.camera_yaw_rate += max(
            min(desired_yaw_rate - self.camera_yaw_rate, math.radians(0.4)),
            math.radians(-0.4),
        )
        camera_yaw += self.camera_yaw_rate
        self.camera_state = (camera_x, camera_y, camera_yaw)
        self.search_uav_states[self.active_host_id] = self.camera_state

        altitude = self.camera_altitude + self.bump_scale * (
            0.35 * math.sin(1.1 * t) + 0.12 * math.sin(3.8 * t)
        )
        roll_deg = self.bump_scale * (0.65 * math.sin(1.4 * t) + 0.18 * math.sin(4.3 * t))
        pitch_deg = self.bump_scale * (0.45 * math.sin(1.2 * t) + 0.14 * math.sin(3.7 * t))
        self.camera_pose = (
            camera_x,
            camera_y,
            altitude,
            math.radians(roll_deg),
            math.radians(pitch_deg),
            camera_yaw,
        )
        self.camera_poses[self.active_host_id] = self.camera_pose
        self.ue.sendUE4Pos(
            self.active_host_id,
            3,
            700.0,
            [camera_x, camera_y, -altitude],
            [math.radians(roll_deg), math.radians(pitch_deg), camera_yaw],
            windowID=self.rfly_window_id,
        )

        if t - self.last_camera_command_at >= RFLY_CAMERA_COMMAND_PERIOD_S:
            view_pitch = CAMERA_SENSOR_PITCH_DEG + pitch_deg
            self.ue.sendUE4Cmd(
                "RflyCameraPosAng "
                f"{camera_x:.3f} {camera_y:.3f} {-altitude:.3f} "
                f"{roll_deg:.3f} {view_pitch:.3f} {math.degrees(camera_yaw):.3f}",
                self.rfly_window_id,
            )
            self.last_camera_command_at = t

    def update_search_uavs(self, t: float) -> None:
        target_recent = (
            self.visual_target_state is not None
            and t - self.visual_target_last_seen <= 2.0
        )
        if target_recent:
            center_x = float(self.visual_target_state["x"]) + float(
                self.visual_target_state["vx"]
            ) * 0.8
            center_y = float(self.visual_target_state["y"]) + float(
                self.visual_target_state["vy"]
            ) * 0.8
            radius = 20.0
            base_altitude = max(self.camera_altitude, 32.0)
        for host_id in (1, 2, 3):
            if host_id == self.active_host_id:
                continue
            phase = 2.0 * math.pi * (host_id - 1) / 3.0
            if target_recent:
                center_x = float(self.visual_target_state["x"]) + float(
                    self.visual_target_state["vx"]
                ) * 0.9
                center_y = float(self.visual_target_state["y"]) + float(
                    self.visual_target_state["vy"]
                ) * 0.9
                radius = 14.0
                angle = phase + 0.05 * t
                desired_x = center_x + radius * math.cos(angle)
                desired_y = center_y + radius * math.sin(angle)
                altitude = max(self.camera_altitude + 2.0 + host_id, 40.0)
                yaw = math.atan2(center_y - desired_y, center_x - desired_x)
            else:
                desired_x, desired_y, base_yaw = self.search_waypoint(host_id, t)
                altitude = CAMERA_SEARCH_MAX_ALTITUDE_M - 2.0 + 0.8 * math.sin(0.18 * t + phase)
                yaw = base_yaw + 0.65 * math.sin(0.32 * t + phase)
            old_x, old_y, _ = self.search_uav_states.get(
                host_id,
                SEARCH_SECTORS[host_id],
            )
            step_x = 0.10 * (desired_x - old_x)
            step_y = 0.10 * (desired_y - old_y)
            step_norm = math.hypot(step_x, step_y)
            if step_norm > 1.15:
                step_x *= 1.15 / step_norm
                step_y *= 1.15 / step_norm
            x = old_x + step_x
            y = old_y + step_y
            roll = math.radians(0.4 * math.sin(0.8 * t + phase))
            pitch = math.radians(0.3 * math.sin(0.7 * t + phase))
            self.search_uav_states[host_id] = (x, y, yaw)
            pose = (x, y, altitude, roll, pitch, yaw)
            self.camera_poses[host_id] = pose
            self.ue.sendUE4Pos(
                host_id,
                3,
                700.0,
                [x, y, -altitude],
                [roll, pitch, yaw],
                windowID=self.rfly_window_id,
            )

    @staticmethod
    def rotate_x(vector: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
        x, y, z = vector
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return x, cosine * y - sine * z, sine * y + cosine * z

    @staticmethod
    def rotate_y(vector: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
        x, y, z = vector
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return cosine * x + sine * z, y, -sine * x + cosine * z

    @staticmethod
    def rotate_z(vector: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
        x, y, z = vector
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return cosine * x - sine * y, sine * x + cosine * y, z

    def image_to_ground(
        self,
        host_id: int,
        image_x: float,
        image_y: float,
        width: int,
        height: int,
    ) -> tuple[float, float] | None:
        pose = self.camera_poses.get(host_id)
        if pose is None:
            return None
        camera_x, camera_y, altitude, roll, pitch, yaw = pose
        focal = width / (2.0 * math.tan(math.radians(CAMERA_FOV_DEG) / 2.0))
        ray = (1.0, (image_x - width / 2.0) / focal, (image_y - height / 2.0) / focal)
        ray = self.rotate_y(ray, math.radians(CAMERA_SENSOR_PITCH_DEG))
        ray = self.rotate_x(ray, roll)
        ray = self.rotate_y(ray, pitch)
        ray = self.rotate_z(ray, yaw)
        if ray[2] <= 0.05:
            return None
        camera_forward_x = 1.2 * math.cos(yaw)
        camera_forward_y = 1.2 * math.sin(yaw)
        camera_altitude = altitude - 0.35
        scale = camera_altitude / ray[2]
        return (
            camera_x + camera_forward_x + ray[0] * scale,
            camera_y + camera_forward_y + ray[1] * scale,
        )

    def receive_visual_tracks(self, now: float) -> None:
        packets = []
        while True:
            try:
                payload, _ = self.vision_socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                packets.append(json.loads(payload.decode("utf-8")))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
        if not packets:
            return
        self.vision_stream_started = True
        self.vision_packet_count += len(packets)
        self.last_vision_packet_at = now
        for latest in packets:
            host_id = int(latest.get("host_id", 1))
            requested_host = int(latest.get("active_host", self.active_host_id))
            if requested_host in (1, 2, 3):
                self.active_host_id = requested_host
            width = int(latest.get("width", 0))
            height = int(latest.get("height", 0))
            if host_id not in (1, 2, 3) or width <= 0 or height <= 0:
                continue
            for item in latest.get("tracks", []):
                self.vision_track_packet_count += 1
                try:
                    local_track_id = int(item["track_id"])
                    track_id = host_id * 10000 + local_track_id
                    image_x = (float(item["x1"]) + float(item["x2"])) / 2.0
                    image_y = float(item["y2"])
                    projected = self.image_to_ground(host_id, image_x, image_y, width, height)
                except (KeyError, TypeError, ValueError):
                    continue
                if projected is None:
                    continue
                previous = self.vision_tracks.get(track_id)
                vx = 0.0
                vy = 0.0
                if previous is not None:
                    delta = now - float(previous["seen_at"])
                    if 0.02 <= delta <= 2.0:
                        raw_vx = (projected[0] - float(previous["x"])) / delta
                        raw_vy = (projected[1] - float(previous["y"])) / delta
                        raw_speed = math.hypot(raw_vx, raw_vy)
                        if raw_speed <= 38.0:
                            projected = (
                                0.28 * projected[0] + 0.72 * float(previous["x"]),
                                0.28 * projected[1] + 0.72 * float(previous["y"]),
                            )
                            vx = 0.22 * raw_vx + 0.78 * float(previous["vx"])
                            vy = 0.22 * raw_vy + 0.78 * float(previous["vy"])
                        else:
                            projected = (float(previous["x"]), float(previous["y"]))
                            vx = float(previous["vx"])
                            vy = float(previous["vy"])
                box_width = max(0.0, float(item["x2"]) - float(item["x1"]))
                box_height = max(0.0, float(item["y2"]) - float(item["y1"]))
                normalized_x = image_x / width
                normalized_y = ((float(item["y1"]) + float(item["y2"])) / 2.0) / height
                self.vision_tracks[track_id] = {
                    "host_id": host_id,
                    "x": projected[0],
                    "y": projected[1],
                    "vx": vx,
                    "vy": vy,
                    "confidence": float(item.get("confidence", 0.0)),
                    "cls": int(item.get("cls", 2)),
                    "label": str(item.get("label", "vehicle")),
                    "confirmed": bool(item.get("confirmed", True)),
                    "seen_at": now,
                    "image_x": image_x,
                    "image_y": image_y,
                    "area": box_width * box_height,
                    "height_fraction": box_height / height,
                    "center_error": math.hypot(
                        (normalized_x - 0.5) / 0.5,
                        (normalized_y - 0.5) / 0.5,
                    ),
                    "normalized_x": normalized_x,
                    "normalized_y": normalized_y,
                }

    def build_visual_message(self, now: float) -> TargetTrackArray:
        output = TargetTrackArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "world"
        output.frame_idx = self.frame_idx
        output.tracks = []
        stale_ids = [
            track_id
            for track_id, state in self.vision_tracks.items()
            if now - float(state["seen_at"]) > VISION_STALE_SECONDS
        ]
        state = self.visual_target_state
        if state is not None and now - self.visual_target_last_seen <= 2.0:
            track = TargetTrack()
            track.target_id = 1
            track.x = float(state["x"])
            track.y = float(state["y"])
            track.vx = float(state["vx"])
            track.vy = float(state["vy"])
            track.confidence = float(state["confidence"])
            track.cls = 2
            track.is_confirmed = True
            track.speed = math.hypot(track.vx, track.vy)
            track.motion_mode = 2 if track.speed > 1.0 else 1
            track.pred_x = [track.x + track.vx * step for step in (0.5, 1.0, 1.5, 2.0, 2.5)]
            track.pred_y = [track.y + track.vy * step for step in (0.5, 1.0, 1.5, 2.0, 2.5)]
            track.pred_conf = [0.8, 0.7, 0.6, 0.5, 0.4]
            if hasattr(track, "world_valid"):
                track.world_valid = True
            if hasattr(track, "units"):
                track.units = "m"
            output.tracks.append(track)
        for track_id in stale_ids:
            del self.vision_tracks[track_id]
        return output

    def on_enclosure(self, message: EnclosureCommandArray) -> None:
        for command in message.commands:
            if not (
                math.isfinite(command.target_x)
                and math.isfinite(command.target_y)
                and math.isfinite(command.target_z)
            ):
                continue
            self.ground_car_goals[int(command.drone_id)] = (
                float(command.target_x),
                float(command.target_y),
            )

    def update_ground_interceptors(self) -> None:
        for car_id, (current_x, current_y) in self.ground_car_positions.items():
            goal = self.ground_car_goals.get(car_id)
            if goal is not None:
                delta_x = goal[0] - current_x
                delta_y = goal[1] - current_y
                distance = math.hypot(delta_x, delta_y)
                max_step = 1.05
                if distance > max_step:
                    delta_x *= max_step / distance
                    delta_y *= max_step / distance
                current_x += delta_x
                current_y += delta_y
            yaw = 0.0
            if goal is not None:
                yaw = math.atan2(goal[1] - current_y, goal[0] - current_x)
            self.ground_car_positions[car_id] = (current_x, current_y)
            self.ue.sendUE4PosScale2Ground(
                201 + car_id,
                51,
                0.0,
                [current_x, current_y, 0.0],
                [0.0, 0.0, yaw],
                [2.2, 2.2, 2.2],
                windowID=self.rfly_window_id,
            )

    def publish_platform_states(self) -> None:
        uav_message = DroneStateArray()
        uav_message.num_drones = 3
        uav_message.drones = []
        for host_id in (1, 2, 3):
            x, y, _ = self.search_uav_states[host_id]
            pose = self.camera_poses.get(host_id)
            state = DroneState()
            state.drone_id = host_id - 1
            state.x = x
            state.y = y
            state.z = float(pose[2]) if pose is not None else self.camera_altitude
            state.vx = 0.0
            state.vy = 0.0
            state.vz = 0.0
            state.available = True
            state.platform_type = 0
            uav_message.drones.append(state)
        self.uav_pose_publisher.publish(uav_message)

        ground_message = DroneStateArray()
        ground_message.num_drones = 3
        ground_message.drones = []
        for car_id, (x, y) in self.ground_car_positions.items():
            state = DroneState()
            state.drone_id = car_id
            state.x = x
            state.y = y
            state.z = 0.0
            state.vx = 0.0
            state.vy = 0.0
            state.vz = 0.0
            state.available = True
            state.platform_type = 1
            ground_message.drones.append(state)
        self.ground_state_publisher.publish(ground_message)

    def tick(self) -> None:
        t = time.monotonic() - self.start_time
        states = self.car_states(t)
        self.current_dynamic_obstacles = self.dynamic_obstacle_states(t, states[0])
        self.update_search_uavs(t)
        self.receive_visual_tracks(t)
        self.update_camera_uav(t, states[0])
        self.update_ground_interceptors()
        self.publish_platform_states()
        truth_output = TargetTrackArray()
        truth_output.header.stamp = self.get_clock().now().to_msg()
        truth_output.header.frame_id = "world"
        truth_output.frame_idx = self.frame_idx
        truth_output.tracks = []

        for obstacle_id, x, y, yaw in PARKED_CARS:
            self.ue.sendUE4PosScale2Ground(
                obstacle_id,
                51,
                0.0,
                [x, y, 0.0],
                [0.0, 0.0, yaw],
                [1.6, 1.6, 1.6],
                windowID=self.rfly_window_id,
            )

        for obstacle in self.current_dynamic_obstacles:
            self.ue.sendUE4PosScale2Ground(
                int(obstacle["id"]),
                51,
                0.0,
                [float(obstacle["x"]), float(obstacle["y"]), 0.0],
                [0.0, 0.0, float(obstacle["yaw"])],
                [float(obstacle["scale"])] * 3,
                windowID=self.rfly_window_id,
            )

        for target_id, x, y, vx, vy in states:
            yaw = math.atan2(vy, vx)
            self.ue.sendUE4PosScale2Ground(
                target_id,
                50,
                0,
                [x, y, 0.0],
                [0.0, 0.0, yaw],
                [2.2 if target_id == 101 else 1.7,
                 2.2 if target_id == 101 else 1.7,
                 2.2 if target_id == 101 else 1.7],
                windowID=self.rfly_window_id,
            )
            track = TargetTrack()
            track.target_id = target_id
            track.x = x
            track.y = y
            track.vx = vx
            track.vy = vy
            track.confidence = 0.95
            track.cls = 2
            track.is_confirmed = True
            track.speed = math.hypot(vx, vy)
            track.motion_mode = 2
            track.pred_x = [x + vx * step for step in (0.5, 1.0, 1.5, 2.0, 2.5)]
            track.pred_y = [y + vy * step for step in (0.5, 1.0, 1.5, 2.0, 2.5)]
            track.pred_conf = [0.9, 0.8, 0.7, 0.6, 0.5]
            if hasattr(track, "world_valid"):
                track.world_valid = True
            if hasattr(track, "units"):
                track.units = "m"
            truth_output.tracks.append(track)

        self.truth_publisher.publish(truth_output)
        self.publisher.publish(self.build_visual_message(t))
        if self.frame_idx % 20 == 0:
            self.grid_publisher.publish(self.grid_message)
        if self.frame_idx % 10 == 0:
            target_id, truth_x, truth_y, truth_vx, truth_vy = states[0]
            visual = self.visual_target_state
            host_changed = self.active_host_id != self.telemetry_host_id
            self.telemetry_host_id = self.active_host_id
            if host_changed:
                phase = "handoff"
            elif self.control_mode == "track" and self.ground_car_goals:
                phase = "contain"
            elif self.control_mode == "track":
                phase = "lock_predict"
            elif self.control_mode == "coast":
                phase = "coast_recover"
            else:
                phase = "search_360"
            self.telemetry_file.write(json.dumps({
                "time_s": round(t, 3),
                "mode": self.control_mode,
                "phase": phase,
                "host_changed": host_changed,
                "scenario": self.scenario_name,
                "map_name": self.scenario["map_name"],
                "weather": self.weather_name,
                "weather_type": self.weather_type,
                "occlusion_level": self.occlusion_level,
                "prediction_lead_s": self.control_prediction_lead_s,
                "active_host": self.active_host_id,
                "target_truth": {
                    "id": target_id,
                    "x": truth_x,
                    "y": truth_y,
                    "vx": truth_vx,
                    "vy": truth_vy,
                },
                "target_visual": self.visual_projection_state,
                "target_control": visual,
                "target_control_source": self.target_control_source,
                "target_speed_mps": math.hypot(truth_vx, truth_vy),
                "target_heading_deg": math.degrees(math.atan2(truth_vy, truth_vx)),
                "vision_stream_started": self.vision_stream_started,
                "vision_packet_count": self.vision_packet_count,
                "vision_track_packet_count": self.vision_track_packet_count,
                "last_vision_packet_age_s": (
                    None
                    if self.last_vision_packet_at < 0.0
                    else round(t - self.last_vision_packet_at, 3)
                ),
                "uavs": self.camera_poses,
                "ground_cars": self.ground_car_positions,
                "ground_goals": self.ground_car_goals,
                "dynamic_obstacles": self.current_dynamic_obstacles,
            }) + "\n")
        self.frame_idx += 1


def main() -> None:
    rclpy.init()
    node = RflyRosScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.vision_socket.close()
        node.telemetry_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
