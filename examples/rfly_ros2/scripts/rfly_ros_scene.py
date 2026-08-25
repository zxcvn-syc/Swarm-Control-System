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
CAMERA_SEARCH_FOV_DEG = 110.0
CAMERA_LOCK_FOV_DEG = 76.0
CAMERA_SENSOR_PITCH_DEG = -38.0
CAMERA_ALTITUDE_M = 76.0
CAMERA_MIN_ALTITUDE_M = 60.0
CAMERA_MAX_ALTITUDE_M = 104.0
CAMERA_SEARCH_MAX_ALTITUDE_M = 110.0
CAMERA_OCCLUSION_ALTITUDE_M = 76.0
RFLY_CAMERA_COMMAND_PERIOD_S = 0.14
RFLY_RENDER_UPDATE_PERIOD_S = 0.08
RFLY_STATUS_UPDATE_PERIOD_S = 0.10
MAX_VISUAL_TRANSPORT_LATENCY_S = 3.0
CAMERA_FOV_SLEW_DEG_PER_UPDATE = 1.8
CAMERA_MAX_STEP_M = 18.0
CAMERA_MAX_ACCELERATION_STEP_M = 6.0
CAMERA_MAX_SPEED_M = 18.0
CAMERA_SEARCH_YAW_STEP_DEG = 2.5
CAMERA_MAX_YAW_STEP_DEG = 55.0
CAMERA_MAX_YAW_ACCELERATION_DEG = 120.0
MAX_VISUAL_SPEED_MPS = 28.0
STATIC_SCENE_BOOTSTRAP_DELAY_S = 0.80
INITIAL_SEARCH_ASSIST_SECONDS = 30.0
CAMERA_CARRIER_ID = 1
TARGET_VEHICLE_TYPE = 50
GROUND_VEHICLE_TYPE = 51
GROUND_VEHICLE_CLEARANCE_M = 8.0
OBSTACLE_CLEARANCE_M = 10.0
TARGET_MAX_SPEED_MPS = 17.0
TARGET_ACCEL_MPS2 = 2.4
TARGET_BRAKE_MPS2 = 3.2
TARGET_MAX_YAW_RATE_RPS = 0.48
TARGET_MIN_TURN_RADIUS_M = 20.0
GROUND_MAX_SPEED_MPS = 10.0
GROUND_ACCEL_MPS2 = 2.0
GROUND_BRAKE_MPS2 = 3.0
GROUND_MAX_YAW_RATE_RPS = 0.52
GROUND_MIN_TURN_RADIUS_M = 12.0
TARGET_COLLISION_RADIUS_M = 5.8
GROUND_COLLISION_RADIUS_M = 4.2
DYNAMIC_COLLISION_RADIUS_M = 6.3
PHYSICAL_OCCLUDER_COLLISION_RADIUS_M = 10.0
PHYSICAL_OCCLUDER_VEHICLE_TYPE = 100000824
PHYSICAL_OCCLUDER_SCALE = 9.0
PHYSICAL_OCCLUDER_VERTICAL_SCALE = 18.0
VEHICLE_SEPARATION_MARGIN_M = 1.0
SEPARATION_SOLVER_ITERATIONS = 16
SEPARATION_CORRECTION_BUFFER_M = 0.12
PHYSICAL_OCCLUSION_DEFAULT_PERIOD_S = 12.0
PHYSICAL_OCCLUSION_DEFAULT_DURATION_S = 1.4
PHYSICAL_OCCLUSION_LOCK_STABILITY_S = 1.0
PHYSICAL_OCCLUSION_CENTER_ERROR_MAX = 0.78
CAMERA_FOV_ACQUIRE_CENTER_ERROR = 0.72
CAMERA_FOV_RELEASE_CENTER_ERROR = 0.98
PHYSICAL_OCCLUSION_APPROACH_S = 0.45
INITIAL_OCCLUDER_LATERAL_OFFSET_M = 22.0
OCCLUDER_PASS_HALF_WIDTH_M = 12.0
OCCLUDER_STAGING_LATERAL_OFFSET_M = 1.65 * OCCLUDER_PASS_HALF_WIDTH_M
OCCLUDER_LINE_TOLERANCE_M = 4.0
OCCLUDER_CROSS_TOLERANCE_M = 3.5
PHYSICAL_OCCLUDER_STANDBY = (50.0, 190.0)
WEATHER_CONTROLLER_ID = 100
SCENARIO_CONFIG_PATH = Path(__file__).with_name("scenario_presets.json")
WAYPOINTS = (
    (18.0, 24.0),
    (82.0, 18.0),
    (150.0, 22.0),
    (202.0, 62.0),
    (202.0, 128.0),
    (170.0, 190.0),
    (106.0, 205.0),
    (40.0, 190.0),
    (12.0, 140.0),
    (10.0, 74.0),
)
DYNAMIC_OBSTACLE_ROUTES = (
    (
        (26.0, 54.0),
        (108.0, 56.0),
        (192.0, 78.0),
        (190.0, 154.0),
        (116.0, 158.0),
        (28.0, 126.0),
    ),
    (
        (32.0, 92.0),
        (76.0, 48.0),
        (158.0, 66.0),
        (190.0, 142.0),
        (126.0, 194.0),
        (48.0, 166.0),
    ),
    (
        (22.0, 116.0),
        (88.0, 94.0),
        (158.0, 122.0),
        (196.0, 178.0),
        (112.0, 202.0),
        (34.0, 178.0),
    ),
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
LARGE_OBSTACLES = (
    (501, 125000051, 65.0, 36.0, 0.10, 2.8, "dump truck"),
    (502, 118000051, 138.0, 43.0, 1.35, 2.8, "soil roller"),
    (503, 100000750, 175.0, 113.0, 2.75, 2.4, "large crate"),
    (504, 100000824, 88.0, 176.0, -0.45, 3.6, "concrete pillar"),
)
SEARCH_SECTORS = {
    1: (24.0, 28.0, 0.0),
    2: (96.0, 30.0, 0.0),
    3: (178.0, 54.0, 1.1),
}


class UdpForwarder:
    def __init__(self, socket_obj: socket.socket, host: str, port: int) -> None:
        self.socket_obj = socket_obj
        self.host = host
        self.port = port

    def sendto(self, data: bytes, _address: tuple[str, int]) -> int:
        return self.socket_obj.sendto(data, (self.host, self.port))


class RateLimitedUE4:
    def __init__(self, api: UE4CtrlAPI, min_interval: float) -> None:
        self.api = api
        self.min_interval = max(min_interval, 0.02)
        self.last_sent: dict[tuple[str, int], float] = {}
        self.last_pos_state: dict[tuple[str, int], tuple[float, ...]] = {}

    def _allow(self, channel: str, copter_id: int) -> bool:
        now = time.monotonic()
        key = (channel, copter_id)
        last_sent = self.last_sent.get(key, -float("inf"))
        if now - last_sent < self.min_interval:
            return False
        self.last_sent[key] = now
        return True

    def sendUE4Pos(self, *args, **kwargs):
        copter_id = int(args[0] if args else kwargs.get("copterID", 0))
        if self._allow("pos", copter_id):
            if len(args) >= 5:
                state = tuple(
                    float(value)
                    for vector in args[3:5]
                    for value in vector
                )
                previous = self.last_pos_state.get(("pos", copter_id))
                if previous is not None and max(
                    abs(current - old) for current, old in zip(state, previous)
                ) < 0.035:
                    return None
                self.last_pos_state[("pos", copter_id)] = state
            return self.api.sendUE4Pos(*args, **kwargs)
        return None

    def sendUE4PosScale2Ground(self, *args, **kwargs):
        copter_id = int(args[0] if args else kwargs.get("copterID", 0))
        if self._allow("ground", copter_id):
            if len(args) >= 6:
                state = tuple(
                    float(value)
                    for vector in args[3:6]
                    for value in vector
                )
                previous = self.last_pos_state.get(("ground", copter_id))
                if previous is not None and max(
                    abs(current - old) for current, old in zip(state, previous)
                ) < 0.06:
                    return None
                self.last_pos_state[("ground", copter_id)] = state
            return self.api.sendUE4PosScale2Ground(*args, **kwargs)
        return None

    def __getattr__(self, name):
        return getattr(self.api, name)


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


def load_camera_ground_calibration() -> tuple[tuple[float, ...], tuple[float, ...]] | None:
    default_path = Path(__file__).with_name("rfly_world_sensor_calibration.json")
    calibration_path = Path(
        os.environ.get("RFLY_CAMERA_CALIBRATION_PATH", str(default_path))
    )
    if not calibration_path.is_file():
        return None
    try:
        calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
        if calibration.get("schema") != "rfly_world_sensor_affine_v1":
            raise ValueError("unexpected calibration schema")
        if not bool(calibration.get("acceptance", {}).get("passed", False)):
            raise ValueError("calibration acceptance has not passed")
        x_coefficients = tuple(
            float(value) for value in calibration["world_x_coefficients"]
        )
        y_coefficients = tuple(
            float(value) for value in calibration["world_y_coefficients"]
        )
        if (
            len(x_coefficients) != 5
            or len(y_coefficients) != 5
            or not all(math.isfinite(value) for value in (*x_coefficients, *y_coefficients))
        ):
            raise ValueError("expected five finite affine coefficients per axis")
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            f"invalid Rfly camera-ground calibration {calibration_path}: {exc}"
        ) from exc
    return x_coefficients, y_coefficients


class RflyRosScene(Node):
    def __init__(self) -> None:
        super().__init__("rfly_ros_scene")
        self.scenario_name, self.scenario = load_scenario()
        self.weather_name = str(self.scenario["weather_name"])
        self.weather_type = int(self.scenario["weather_type"])
        self.occlusion_level = float(self.scenario["occlusion_level"])
        self.dynamic_obstacle_count = int(self.scenario["dynamic_obstacles"])
        self.bump_scale = float(self.scenario["bump_scale"])
        self.wind_speed_mps = float(self.scenario.get("wind_speed_mps", 0.0))
        self.wind_direction_deg = float(self.scenario.get("wind_direction_deg", 0.0))
        self.wind_direction_rad = math.radians(self.wind_direction_deg)
        physical_occlusion = dict(self.scenario.get("physical_occlusion", {}))
        self.physical_occlusion_enabled = bool(physical_occlusion.get("enabled", False))
        self.physical_occlusion_period_s = float(
            physical_occlusion.get("period_s", PHYSICAL_OCCLUSION_DEFAULT_PERIOD_S)
        )
        self.physical_occlusion_duration_s = float(
            physical_occlusion.get("duration_s", PHYSICAL_OCCLUSION_DEFAULT_DURATION_S)
        )
        self.physical_occlusion_start_s = float(physical_occlusion.get("start_s", 18.0))
        self.physical_occluder_id = int(physical_occlusion.get("occluder_id", 421))
        self.vision_port = int(os.environ.get("RFLY_VISION_PORT", VISION_PORT))
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
        ue_api = UE4CtrlAPI(self.rfly_host_ip)
        bridge_host = os.environ.get("RFLY_UE4_BRIDGE_HOST", "").strip()
        bridge_port = int(os.environ.get("RFLY_UE4_BRIDGE_PORT", "0"))
        if bridge_host and bridge_port > 0:
            ue_api.udp_socket = UdpForwarder(
                ue_api.udp_socket,
                bridge_host,
                bridge_port,
            )
            self.get_logger().info(
                f"UE4 control forwarding enabled: {bridge_host}:{bridge_port}"
            )
        self.ue = RateLimitedUE4(
            ue_api,
            float(os.environ.get("RFLY_RENDER_UPDATE_PERIOD_S", RFLY_RENDER_UPDATE_PERIOD_S)),
        )
        self.start_time = time.monotonic()
        self.calibration_camera_assist = os.environ.get(
            "RFLY_CALIBRATION_CAMERA_ASSIST", "0"
        ).strip().lower() in {"1", "true", "yes"}
        self.camera_ground_calibration = load_camera_ground_calibration()
        self.route_seed = int(os.environ.get("RFLY_SCENE_SEED", "20260821"))
        self.route_rng = random.Random(self.route_seed)
        self.car_profiles = {
            target_id: {
                "phase": phase,
                "speed_phase": self.route_rng.uniform(0.0, 2.0 * math.pi),
            }
            for target_id, phase in ((101, 0.0),)
        }
        self.frame_idx = 0
        self.target_motion: dict[str, float | int] | None = None
        self.last_target_time = -1.0
        self.dynamic_obstacle_motion: dict[int, dict[str, float | int]] = {}
        self.last_dynamic_obstacle_time = -1.0
        self.static_scene_spawned = False
        self.ground_car_goals: dict[int, tuple[float, float]] = {}
        self.ground_goal_update_count = 0
        self.ground_car_positions = {
            0: (24.0, 184.0),
            1: (194.0, 28.0),
            2: (72.0, 208.0),
        }
        self.ground_car_velocities = {car_id: (0.0, 0.0) for car_id in self.ground_car_positions}
        self.ground_car_speeds = {car_id: 0.0 for car_id in self.ground_car_positions}
        self.ground_car_yaws = {
            0: -0.70,
            1: 2.45,
            2: -1.65,
        }
        self.ground_car_yaw_rates = {car_id: 0.0 for car_id in self.ground_car_positions}
        self.last_ground_update_time = -1.0
        self.camera_state: tuple[float, float, float] | None = SEARCH_SECTORS[1]
        self.active_host_id = 1
        self.last_active_host_id = 1
        self.telemetry_host_id = 1
        self.search_uav_states = dict(SEARCH_SECTORS)
        self.camera_altitude = CAMERA_SEARCH_MAX_ALTITUDE_M - 8.0
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
        self.vision_socket.bind(("0.0.0.0", self.vision_port))
        self.vision_socket.setblocking(False)
        self.vision_status_socket = self.vision_socket
        self.vision_status_address: tuple[str, int] | None = None
        status_bridge_host = os.environ.get("RFLY_STATUS_BRIDGE_HOST", "").strip()
        status_bridge_port = int(os.environ.get("RFLY_STATUS_BRIDGE_PORT", "0"))
        if status_bridge_host and status_bridge_port > 0:
            self.vision_status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.vision_status_address = (status_bridge_host, status_bridge_port)
            self.get_logger().info(
                "Vision status forwarding enabled: "
                f"{status_bridge_host}:{status_bridge_port}"
            )
        self.last_vision_status_sent_at = -float("inf")
        self.vision_tracks: dict[int, dict[str, float | int | str | bool]] = {}
        self.vision_stream_started = False
        self.vision_first_packet_at = -1.0
        self.vision_packet_count = 0
        self.vision_track_packet_count = 0
        self.last_vision_packet_at = -1.0
        self.visual_lock_id: int | None = None
        self.visual_lock_last_seen = -1.0
        self.visual_target_state: dict[str, float] | None = None
        self.visual_projection_state: dict[str, float] | None = None
        self.target_control_source = "none"
        self.visual_target_last_seen = -1.0
        self.last_visual_observation_at = -1.0
        self.reacquisition_count = 0
        self.last_reacquisition_latency_s: float | None = None
        self.control_mode = "search"
        self.search_bootstrap_active = False
        self.control_prediction_lead_s = 0.9
        self.camera_velocity = (0.0, 0.0)
        self.camera_yaw_rate = 0.0
        self.camera_search_yaw = 0.0
        self.last_camera_update_at = -1.0
        self.last_camera_command_at = -float("inf")
        self.current_camera_fov_deg = CAMERA_SEARCH_FOV_DEG
        self.locked_camera_fov = False
        self.current_dynamic_obstacles: list[dict[str, float | int]] = []
        self.physical_occlusion_requested = False
        self.physical_occlusion_engaged = False
        self.physical_occlusion_prepositioning = False
        self.physical_occlusion_lock_qualified = False
        self.physical_occlusion_approach_side_seen = False
        self.physical_occlusion_crossed_los = False
        self.physical_occlusion_epoch_at = -1.0
        self.physical_occlusion_lock_started_at = -1.0
        self.physical_occlusion_alignment_m = float("inf")
        self.physical_occlusion_line_error_m = float("inf")
        self.physical_occlusion_cross_error_m = float("inf")
        self.physical_occlusion_ray_height_m = float("inf")
        self.physical_occlusion_ray_height_m = float("inf")
        self.min_vehicle_distance_m = float("inf")
        self.min_vehicle_clearance_m = float("inf")
        self.vehicle_overlap_count = 0
        self.collision_resolutions = 0
        self.collision_resolution_brakes = 0
        demo_root = Path(os.environ.get("RFLY_DEMO_ROOT", Path.cwd()))
        log_root = Path(os.environ.get("RFLY_LOG_ROOT", demo_root / "logs"))
        telemetry_path = log_root / "scene_telemetry.jsonl"
        telemetry_path.parent.mkdir(parents=True, exist_ok=True)
        self.telemetry_file = telemetry_path.open("w", encoding="utf-8", buffering=1)
        self.grid_message = self.build_grid_message()
        for stale_id in (
            *range(1, 5),
            *range(100, 111),
            *range(200, 211),
            *range(400, 431),
            *range(500, 521),
            *range(880, 900),
        ):
            self.ue.sendUE4Destroy(stale_id, self.rfly_window_id)
        self.ue.sendUE4Cmd(
            f"RflyChangeMapbyName {self.scenario['map_name']}",
            self.rfly_window_id,
        )
        self.ue.sendUE4Cmd("RflyChangeViewKeyCmd N 6", self.rfly_window_id)
        self.ue.sendUE4Cmd(
            f"RflyCameraFovDegrees {self.current_camera_fov_deg:.1f}",
            self.rfly_window_id,
        )
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
        self.ue.sendUE4Cmd("t.MaxFPS 45", self.rfly_window_id)
        self.spawn_static_scene()
        self.static_scene_spawned = True
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            "Rfly chase scene started: 1 evasive blue target, 3 search UAVs, "
            "3 gray ground interceptors, static and dynamic obstacles; "
            f"visual input UDP port {self.vision_port}; "
            f"scenario={self.scenario_name} map={self.scenario['map_name']} "
            f"weather={self.weather_name} occlusion={self.occlusion_level:.2f} "
            f"wind={self.wind_speed_mps:.1f}m/s@{self.wind_direction_deg:.0f}deg "
            f"rfly_host={self.rfly_host_ip} "
            f"route seed={self.route_seed} profiles={self.car_profiles}"
        )

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @classmethod
    def advance_forward_vehicle(
        cls,
        state: dict[str, float | int],
        desired_heading: float,
        desired_speed: float,
        dt: float,
        *,
        max_speed: float,
        acceleration: float,
        braking: float,
        max_yaw_rate: float,
        max_yaw_acceleration: float,
        min_turn_radius: float,
        min_speed: float = 0.0,
    ) -> None:
        if dt <= 0.0:
            return
        yaw = float(state["yaw"])
        speed = float(state["speed"])
        yaw_rate = float(state.get("yaw_rate", 0.0))
        heading_error = cls.wrap_angle(desired_heading - yaw)
        turn_speed_scale = max(0.28, 1.0 - 0.85 * abs(heading_error))
        speed_target = min(max(desired_speed, 0.0), max_speed * turn_speed_scale)
        speed_delta_limit = (acceleration if speed_target >= speed else braking) * dt
        speed += max(min(speed_target - speed, speed_delta_limit), -speed_delta_limit)
        if speed_target > 0.0:
            speed = max(speed, min_speed)
        physical_yaw_limit = min(
            max_yaw_rate,
            max(speed, 0.8) / max(min_turn_radius, 1.0),
        )
        desired_yaw_rate = max(
            min(1.35 * heading_error, physical_yaw_limit),
            -physical_yaw_limit,
        )
        yaw_rate_delta = max_yaw_acceleration * dt
        yaw_rate += max(
            min(desired_yaw_rate - yaw_rate, yaw_rate_delta),
            -yaw_rate_delta,
        )
        yaw = cls.wrap_angle(yaw + yaw_rate * dt)
        velocity_x = speed * math.cos(yaw)
        velocity_y = speed * math.sin(yaw)
        state["x"] = float(state["x"]) + velocity_x * dt
        state["y"] = float(state["y"]) + velocity_y * dt
        state["yaw"] = yaw
        state["yaw_rate"] = yaw_rate
        state["speed"] = speed
        state["vx"] = velocity_x
        state["vy"] = velocity_y

    @staticmethod
    def waypoint_heading(
        state: dict[str, float | int],
        route: tuple[tuple[float, float], ...],
        capture_radius: float,
    ) -> float:
        waypoint_index = int(state["waypoint_index"])
        waypoint_x, waypoint_y = route[waypoint_index]
        delta_x = waypoint_x - float(state["x"])
        delta_y = waypoint_y - float(state["y"])
        if math.hypot(delta_x, delta_y) <= capture_radius:
            waypoint_index = (waypoint_index + 1) % len(route)
            state["waypoint_index"] = waypoint_index
            waypoint_x, waypoint_y = route[waypoint_index]
            delta_x = waypoint_x - float(state["x"])
            delta_y = waypoint_y - float(state["y"])
        return math.atan2(delta_y, delta_x)

    @classmethod
    def advance_guided_occluder(
        cls,
        state: dict[str, float | int],
        goal_x: float,
        goal_y: float,
        dt: float,
    ) -> None:
        if dt <= 0.0:
            return
        current_x = float(state["x"])
        current_y = float(state["y"])
        delta_x = goal_x - current_x
        delta_y = goal_y - current_y
        distance = math.hypot(delta_x, delta_y)
        if distance < 1e-6:
            return
        maximum_step = 20.0 * dt
        step = min(distance, maximum_step)
        desired_yaw = math.atan2(delta_y, delta_x)
        yaw = float(state["yaw"])
        maximum_yaw_step = 0.58 * dt
        yaw_error = cls.wrap_angle(desired_yaw - yaw)
        yaw_step = max(min(yaw_error, maximum_yaw_step), -maximum_yaw_step)
        yaw = cls.wrap_angle(yaw + yaw_step)
        heading_alignment = max(0.35, math.cos(cls.wrap_angle(desired_yaw - yaw)))
        step *= heading_alignment
        velocity_x = step * math.cos(yaw) / dt
        velocity_y = step * math.sin(yaw) / dt
        state["x"] = current_x + velocity_x * dt
        state["y"] = current_y + velocity_y * dt
        state["yaw"] = yaw
        state["yaw_rate"] = yaw_step / dt
        state["speed"] = math.hypot(velocity_x, velocity_y)
        state["vx"] = velocity_x
        state["vy"] = velocity_y

    @classmethod
    def avoidance_heading(
        cls,
        base_heading: float,
        x: float,
        y: float,
        avoidance_points: list[tuple[float, float, float]],
    ) -> float:
        steering_x = math.cos(base_heading)
        steering_y = math.sin(base_heading)
        for avoid_x, avoid_y, clearance in avoidance_points:
            separation_x = x - avoid_x
            separation_y = y - avoid_y
            separation = math.hypot(separation_x, separation_y)
            influence_radius = 2.25 * clearance
            if 1e-6 < separation < influence_radius:
                strength = 2.4 * (influence_radius - separation) / influence_radius
                steering_x += strength * separation_x / separation
                steering_y += strength * separation_y / separation
        boundary_margin = 18.0
        if x < boundary_margin:
            steering_x += 1.8 * (boundary_margin - x) / boundary_margin
        elif x > GRID_SIZE - boundary_margin:
            steering_x -= 1.8 * (x - GRID_SIZE + boundary_margin) / boundary_margin
        if y < boundary_margin:
            steering_y += 1.8 * (boundary_margin - y) / boundary_margin
        elif y > GRID_SIZE - boundary_margin:
            steering_y -= 1.8 * (y - GRID_SIZE + boundary_margin) / boundary_margin
        if math.hypot(steering_x, steering_y) < 1e-6:
            return base_heading
        return math.atan2(steering_y, steering_x)

    def car_states(self, t: float) -> list[tuple[int, float, float, float, float]]:
        states = []
        for target_id in (101,):
            profile = self.car_profiles[target_id]
            if self.target_motion is None:
                initial_yaw = math.atan2(
                    WAYPOINTS[1][1] - WAYPOINTS[0][1],
                    WAYPOINTS[1][0] - WAYPOINTS[0][0],
                )
                self.target_motion = {
                    "x": WAYPOINTS[0][0],
                    "y": WAYPOINTS[0][1],
                    "yaw": initial_yaw,
                    "yaw_rate": 0.0,
                    "speed": 7.5,
                    "vx": 7.5 * math.cos(initial_yaw),
                    "vy": 7.5 * math.sin(initial_yaw),
                    "waypoint_index": 1,
                }
            dt = (
                0.0
                if self.last_target_time < 0.0
                else min(max(t - self.last_target_time, 0.0), 0.15)
            )
            capture_radius = max(14.0, 1.15 * float(self.target_motion["speed"]))
            desired_heading = self.waypoint_heading(
                self.target_motion,
                WAYPOINTS,
                capture_radius,
            )
            wind_heading_bias = 0.006 * self.wind_speed_mps * (
                math.sin(0.29 * t + self.wind_direction_rad)
                + 0.25 * math.sin(0.71 * t)
            )
            desired_heading = self.wrap_angle(desired_heading + wind_heading_bias)
            avoidance_points = [
                (x, y, GROUND_VEHICLE_CLEARANCE_M)
                for _obstacle_id, x, y, _yaw in PARKED_CARS
            ]
            avoidance_points.extend(
                (x, y, OBSTACLE_CLEARANCE_M)
                for _obstacle_id, _vehicle_type, x, y, _yaw, _scale, _label in LARGE_OBSTACLES
            )
            avoidance_points.extend(
                (float(motion["x"]), float(motion["y"]), OBSTACLE_CLEARANCE_M)
                for motion in self.dynamic_obstacle_motion.values()
            )
            avoidance_points.extend(
                (x, y, GROUND_VEHICLE_CLEARANCE_M)
                for x, y in self.ground_car_positions.values()
            )
            desired_heading = self.avoidance_heading(
                desired_heading,
                float(self.target_motion["x"]),
                float(self.target_motion["y"]),
                avoidance_points,
            )
            speed_phase = float(profile["speed_phase"])
            desired_speed = min(
                16.7,
                max(
                    8.0,
                    13.2
                    + 2.1 * math.sin(0.18 * t + speed_phase)
                    + 1.2 * math.sin(0.49 * t + 0.4 * speed_phase),
                ),
            )
            self.advance_forward_vehicle(
                self.target_motion,
                desired_heading,
                desired_speed,
                dt,
                max_speed=TARGET_MAX_SPEED_MPS,
                acceleration=TARGET_ACCEL_MPS2,
                braking=TARGET_BRAKE_MPS2,
                max_yaw_rate=TARGET_MAX_YAW_RATE_RPS,
                max_yaw_acceleration=0.62,
                min_turn_radius=TARGET_MIN_TURN_RADIUS_M,
                min_speed=4.5,
            )
            self.last_target_time = t
            states.append(
                (
                    target_id,
                    float(self.target_motion["x"]),
                    float(self.target_motion["y"]),
                    float(self.target_motion["vx"]),
                    float(self.target_motion["vy"]),
                )
            )
        return states

    def dynamic_obstacle_states(
        self,
        t: float,
        target: tuple[int, float, float, float, float],
    ) -> list[dict[str, float | int]]:
        self.physical_occlusion_requested = False
        self.physical_occlusion_engaged = False
        self.physical_occlusion_alignment_m = float("inf")
        self.physical_occlusion_line_error_m = float("inf")
        self.physical_occlusion_cross_error_m = float("inf")
        if self.dynamic_obstacle_count <= 0 or self.occlusion_level <= 0.0:
            return []
        _, target_x, target_y, _, _ = target
        states = []
        dt = (
            0.0
            if self.last_dynamic_obstacle_time < 0.0
            else min(max(t - self.last_dynamic_obstacle_time, 0.0), 0.15)
        )
        self.last_dynamic_obstacle_time = t
        static_centers = [(entry[2], entry[3]) for entry in LARGE_OBSTACLES]
        for index in range(self.dynamic_obstacle_count):
            obstacle_id = 421 + index
            route = DYNAMIC_OBSTACLE_ROUTES[index % len(DYNAMIC_OBSTACLE_ROUTES)]
            motion = self.dynamic_obstacle_motion.get(obstacle_id)
            if motion is None:
                route_start = (0, 2, 1, 3)[index % 4] % len(route)
                route_next = (route_start + 1) % len(route)
                if index == 0 and self.physical_occlusion_enabled:
                    start_x, start_y = PHYSICAL_OCCLUDER_STANDBY
                    initial_yaw = 0.0
                    initial_speed = 0.0
                else:
                    initial_yaw = math.atan2(
                        route[route_next][1] - route[route_start][1],
                        route[route_next][0] - route[route_start][0],
                    )
                    start_x, start_y = route[route_start]
                    initial_speed = 4.5 + index
                motion = {
                    "x": start_x,
                    "y": start_y,
                    "yaw": initial_yaw,
                    "yaw_rate": 0.0,
                    "speed": initial_speed,
                    "vx": initial_speed * math.cos(initial_yaw),
                    "vy": initial_speed * math.sin(initial_yaw),
                    "waypoint_index": route_next,
                }
                self.dynamic_obstacle_motion[obstacle_id] = motion
            base_heading = self.waypoint_heading(
                motion,
                route,
                max(12.0, float(motion["speed"])),
            )
            occlusion_goal = None
            if (
                index == 0
                and self.physical_occlusion_enabled
                and self.physical_occlusion_prepositioning
            ):
                occlusion_goal = self.physical_occlusion_goal(
                    t, target_x, target_y
                )
            positions_to_avoid = [(target_x, target_y, 9.0)]
            positions_to_avoid.extend(
                (center_x, center_y, OBSTACLE_CLEARANCE_M)
                for center_x, center_y in static_centers
            )
            positions_to_avoid.extend(
                (float(previous["x"]), float(previous["y"]), OBSTACLE_CLEARANCE_M)
                for previous in states
            )
            if occlusion_goal is not None:
                goal_x, goal_y, goal_lateral_error = occlusion_goal
                direct_heading = math.atan2(
                    goal_y - float(motion["y"]),
                    goal_x - float(motion["x"]),
                )
                desired_heading = self.avoidance_heading(
                    direct_heading,
                    float(motion["x"]),
                    float(motion["y"]),
                    [
                        (center_x, center_y, OBSTACLE_CLEARANCE_M)
                        for center_x, center_y in static_centers
                    ] + [
                        (float(previous["x"]), float(previous["y"]), OBSTACLE_CLEARANCE_M)
                        for previous in states
                    ],
                )
                desired_speed = 15.5
                target_distance = math.hypot(
                    target_x - float(motion["x"]),
                    target_y - float(motion["y"]),
                )
                minimum_occluder_distance = (
                    TARGET_COLLISION_RADIUS_M
                    + PHYSICAL_OCCLUDER_COLLISION_RADIUS_M
                    + VEHICLE_SEPARATION_MARGIN_M
                )
            elif index == 0 and self.physical_occlusion_enabled:
                desired_heading = float(motion["yaw"])
                desired_speed = 0.0
            else:
                desired_heading = self.avoidance_heading(
                    base_heading,
                    float(motion["x"]),
                    float(motion["y"]),
                    positions_to_avoid,
                )
                desired_speed = 7.2 + 1.3 * math.sin(0.16 * t + 1.7 * index)
            if index == 0 and occlusion_goal is not None:
                self.advance_guided_occluder(
                    motion,
                    goal_x,
                    goal_y,
                    dt,
                )
            else:
                self.advance_forward_vehicle(
                    motion,
                    desired_heading,
                    desired_speed,
                    dt,
                    max_speed=9.0,
                    acceleration=1.6,
                    braking=2.4,
                    max_yaw_rate=0.42,
                    max_yaw_acceleration=0.58,
                    min_turn_radius=15.0,
                )
            x = float(motion["x"])
            y = float(motion["y"])
            yaw = float(motion["yaw"])
            if (
                index == 0
                and self.physical_occlusion_prepositioning
                and self.physical_occlusion_lock_qualified
                and self.has_centered_visual_lock(t)
                and self.physical_occlusion_epoch_at < 0.0
            ):
                geometry = self.physical_occlusion_geometry(target_x, target_y)
                if geometry is not None:
                    direction_x, direction_y, corridor_distance = geometry
                    lateral_x = -direction_y
                    lateral_y = direction_x
                    relative_x = x - target_x
                    relative_y = y - target_y
                    line_distance = relative_x * direction_x + relative_y * direction_y
                    cross_distance = relative_x * lateral_x + relative_y * lateral_y
                    stage_line_error = abs(line_distance - corridor_distance)
                    stage_cross_error = abs(
                        cross_distance + OCCLUDER_STAGING_LATERAL_OFFSET_M
                    )
                    minimum_occluder_distance = (
                        TARGET_COLLISION_RADIUS_M
                        + PHYSICAL_OCCLUDER_COLLISION_RADIUS_M
                        + VEHICLE_SEPARATION_MARGIN_M
                    )
                    if (
                        stage_line_error <= OCCLUDER_LINE_TOLERANCE_M
                        and stage_cross_error <= OCCLUDER_LINE_TOLERANCE_M
                        and math.hypot(relative_x, relative_y)
                        >= minimum_occluder_distance
                    ):
                        self.physical_occlusion_epoch_at = t
            if index == 0 and self.physical_occlusion_requested:
                geometry = self.physical_occlusion_geometry(target_x, target_y)
                if geometry is not None:
                    direction_x, direction_y, corridor_distance = geometry
                    lateral_x = -direction_y
                    lateral_y = direction_x
                    relative_x = x - target_x
                    relative_y = y - target_y
                    line_distance = (
                        relative_x * direction_x + relative_y * direction_y
                    )
                    cross_distance = (
                        relative_x * lateral_x + relative_y * lateral_y
                    )
                    self.physical_occlusion_line_error_m = abs(
                        line_distance - corridor_distance
                    )
                    self.physical_occlusion_cross_error_m = abs(cross_distance)
                    self.physical_occlusion_alignment_m = math.hypot(
                        self.physical_occlusion_line_error_m,
                        self.physical_occlusion_cross_error_m,
                    )
                    self.physical_occlusion_engaged = (
                        self.physical_occlusion_line_error_m
                        <= OCCLUDER_LINE_TOLERANCE_M
                        and self.physical_occlusion_cross_error_m
                        <= OCCLUDER_CROSS_TOLERANCE_M
                        and target_distance >= minimum_occluder_distance
                    )
            obstacle_types = (
                PHYSICAL_OCCLUDER_VEHICLE_TYPE,
                118000051,
                100000750,
                100000824,
            )
            scale = (
                PHYSICAL_OCCLUDER_SCALE
                if index == 0 and self.physical_occlusion_enabled
                else 2.7 + 0.35 * self.occlusion_level + 0.18 * index
            )
            scale_xyz = [scale, scale, scale]
            if index == 0 and self.physical_occlusion_enabled:
                scale_xyz[2] = PHYSICAL_OCCLUDER_VERTICAL_SCALE
            states.append({
                "id": obstacle_id,
                "vehicle_type": obstacle_types[index % len(obstacle_types)],
                "x": x,
                "y": y,
                "yaw": yaw,
                "yaw_rate": float(motion["yaw_rate"]),
                "speed": float(motion["speed"]),
                "vx": float(motion["vx"]),
                "vy": float(motion["vy"]),
                "scale": scale,
                "scale_xyz": scale_xyz,
                "occlusion_role": "large_line_of_sight_blocker" if index == 0 else "crossing",
                "label": (
                    "tall moving concrete pillar occluder"
                    if index == 0
                    else ("moving soil roller" if index == 1 else "moving large blocker")
                ),
            })
        return states

    def physical_occlusion_goal(
        self,
        t: float,
        target_x: float,
        target_y: float,
    ) -> tuple[float, float, float] | None:
        self.physical_occlusion_requested = False
        if (
            not self.physical_occlusion_enabled
            or self.physical_occlusion_period_s <= 0.0
            or self.physical_occlusion_duration_s <= 0.0
        ):
            return None
        if (
            self.physical_occlusion_epoch_at < 0.0
            and not self.has_centered_visual_lock(t)
        ):
            return None
        if not self.physical_occlusion_prepositioning:
            return None
        if self.camera_pose is None:
            return None
        geometry = self.physical_occlusion_geometry(target_x, target_y)
        if geometry is None:
            return None
        direction_x, direction_y, corridor_distance = geometry
        camera_target_distance = math.hypot(
            self.camera_pose[0] - target_x,
            self.camera_pose[1] - target_y,
        )
        required_distance = (
            TARGET_COLLISION_RADIUS_M
            + PHYSICAL_OCCLUDER_COLLISION_RADIUS_M
            + VEHICLE_SEPARATION_MARGIN_M
        )
        if camera_target_distance <= required_distance + 2.0:
            return None
        lateral_x = -direction_y
        lateral_y = direction_x
        center_x = target_x + direction_x * corridor_distance
        center_y = target_y + direction_y * corridor_distance
        if self.physical_occlusion_epoch_at < 0.0:
            return (
                center_x - lateral_x * OCCLUDER_STAGING_LATERAL_OFFSET_M,
                center_y - lateral_y * OCCLUDER_STAGING_LATERAL_OFFSET_M,
                -OCCLUDER_STAGING_LATERAL_OFFSET_M,
            )
        occlusion_elapsed = t - self.physical_occlusion_epoch_at
        if occlusion_elapsed < self.physical_occlusion_start_s:
            lateral_error = -OCCLUDER_STAGING_LATERAL_OFFSET_M
            return (
                center_x + lateral_x * lateral_error,
                center_y + lateral_y * lateral_error,
                lateral_error,
            )
        phase = (
            occlusion_elapsed - self.physical_occlusion_start_s
        ) % self.physical_occlusion_period_s
        approach_duration = min(
            PHYSICAL_OCCLUSION_APPROACH_S,
            max(self.physical_occlusion_period_s * 0.10, 1.0),
        )
        active_start = approach_duration
        active_end = active_start + self.physical_occlusion_duration_s
        if phase >= active_end:
            lateral_error = OCCLUDER_STAGING_LATERAL_OFFSET_M
            return (
                center_x + lateral_x * lateral_error,
                center_y + lateral_y * lateral_error,
                lateral_error,
            )
        self.physical_occlusion_requested = active_start <= phase < active_end
        if phase < active_start:
            self.physical_occlusion_approach_side_seen = False
            self.physical_occlusion_crossed_los = False
            progress = phase / max(active_start, 1e-6)
            lateral_error = -OCCLUDER_STAGING_LATERAL_OFFSET_M * (1.0 - progress)
        else:
            progress = (phase - active_start) / self.physical_occlusion_duration_s
            progress = min(max(progress, 0.0), 1.0)
            lateral_error = OCCLUDER_PASS_HALF_WIDTH_M * (2.0 * progress - 1.0)
        return (
            center_x + lateral_x * lateral_error,
            center_y + lateral_y * lateral_error,
            lateral_error,
        )

    def has_stable_visual_lock(self, now: float) -> bool:
        return (
            self.control_mode == "track"
            and self.visual_lock_id is not None
            and self.visual_target_last_seen >= 0.0
            and now - self.visual_target_last_seen <= VISION_STALE_SECONDS
            and self.physical_occlusion_lock_started_at >= 0.0
            and now - self.physical_occlusion_lock_started_at
            >= PHYSICAL_OCCLUSION_LOCK_STABILITY_S
        )

    def has_centered_visual_lock(self, now: float) -> bool:
        if not self.has_stable_visual_lock(now):
            return False
        state = self.vision_tracks.get(self.visual_lock_id)
        return (
            state is not None
            and now - float(state["seen_at"]) <= VISION_STALE_SECONDS
            and float(state["center_error"])
            <= PHYSICAL_OCCLUSION_CENTER_ERROR_MAX
        )

    def physical_occlusion_window_active(self, t: float) -> bool:
        if (
            not self.physical_occlusion_enabled
            or self.physical_occlusion_epoch_at < 0.0
            or self.physical_occlusion_period_s <= 0.0
        ):
            return False
        elapsed = t - self.physical_occlusion_epoch_at
        if elapsed < self.physical_occlusion_start_s:
            return False
        phase = (elapsed - self.physical_occlusion_start_s) % (
            self.physical_occlusion_period_s
        )
        approach_duration = min(
            PHYSICAL_OCCLUSION_APPROACH_S,
            max(self.physical_occlusion_period_s * 0.10, 1.0),
        )
        return phase < approach_duration + self.physical_occlusion_duration_s

    def clear_physical_occlusion(self) -> None:
        self.physical_occlusion_requested = False
        self.physical_occlusion_engaged = False
        self.physical_occlusion_prepositioning = False
        self.physical_occlusion_lock_qualified = False
        self.physical_occlusion_approach_side_seen = False
        self.physical_occlusion_crossed_los = False
        self.physical_occlusion_epoch_at = -1.0
        self.physical_occlusion_lock_started_at = -1.0

    def prepare_physical_occlusion_camera(self, t: float) -> bool:
        if (
            not self.physical_occlusion_enabled
            or self.physical_occlusion_epoch_at < 0.0
            or self.physical_occlusion_period_s <= 0.0
        ):
            return False
        elapsed = t - self.physical_occlusion_epoch_at
        if elapsed < self.physical_occlusion_start_s:
            return False
        phase = (elapsed - self.physical_occlusion_start_s) % (
            self.physical_occlusion_period_s
        )
        approach_duration = min(
            PHYSICAL_OCCLUSION_APPROACH_S,
            max(self.physical_occlusion_period_s * 0.10, 1.0),
        )
        active_end = approach_duration + self.physical_occlusion_duration_s
        return phase <= active_end + 0.6

    def physical_occlusion_geometry(
        self,
        target_x: float,
        target_y: float,
    ) -> tuple[float, float, float] | None:
        if self.camera_pose is None:
            return None
        camera_x, camera_y = self.camera_pose[0], self.camera_pose[1]
        delta_x = camera_x - target_x
        delta_y = camera_y - target_y
        camera_target_distance = math.hypot(delta_x, delta_y)
        required_distance = (
            TARGET_COLLISION_RADIUS_M
            + PHYSICAL_OCCLUDER_COLLISION_RADIUS_M
            + VEHICLE_SEPARATION_MARGIN_M
        )
        if camera_target_distance <= required_distance + 2.0:
            return None
        corridor_distance = min(
            max(0.16 * camera_target_distance, required_distance + 0.5),
            camera_target_distance - 1.5,
        )
        return (
            delta_x / camera_target_distance,
            delta_y / camera_target_distance,
            corridor_distance,
        )

    def refresh_physical_occlusion_alignment(
        self,
        target_x: float,
        target_y: float,
    ) -> None:
        self.physical_occlusion_engaged = False
        self.physical_occlusion_alignment_m = float("inf")
        self.physical_occlusion_line_error_m = float("inf")
        self.physical_occlusion_cross_error_m = float("inf")
        if (
            not self.physical_occlusion_requested
            or (
                not self.has_stable_visual_lock(time.monotonic() - self.start_time)
                and not self.physical_occlusion_window_active(
                    time.monotonic() - self.start_time
                )
            )
        ):
            return
        obstacle = next(
            (
                item
                for item in self.current_dynamic_obstacles
                if int(item["id"]) == self.physical_occluder_id
            ),
            None,
        )
        geometry = self.physical_occlusion_geometry(target_x, target_y)
        if obstacle is None or geometry is None:
            return
        direction_x, direction_y, corridor_distance = geometry
        lateral_x = -direction_y
        lateral_y = direction_x
        relative_x = float(obstacle["x"]) - target_x
        relative_y = float(obstacle["y"]) - target_y
        line_distance = relative_x * direction_x + relative_y * direction_y
        cross_distance = relative_x * lateral_x + relative_y * lateral_y
        self.physical_occlusion_line_error_m = abs(
            line_distance - corridor_distance
        )
        self.physical_occlusion_cross_error_m = abs(cross_distance)
        self.physical_occlusion_alignment_m = math.hypot(
            self.physical_occlusion_line_error_m,
            self.physical_occlusion_cross_error_m,
        )
        camera_target_distance = math.hypot(
            self.camera_pose[0] - target_x,
            self.camera_pose[1] - target_y,
        )
        if camera_target_distance > 1e-3:
            self.physical_occlusion_ray_height_m = abs(self.camera_pose[2]) * min(
                max(line_distance, 0.0), camera_target_distance
            ) / camera_target_distance
        if cross_distance <= -OCCLUDER_CROSS_TOLERANCE_M:
            self.physical_occlusion_approach_side_seen = True
        if self.physical_occlusion_approach_side_seen and cross_distance >= 0.0:
            self.physical_occlusion_crossed_los = True
        minimum_distance = (
            TARGET_COLLISION_RADIUS_M
            + PHYSICAL_OCCLUDER_COLLISION_RADIUS_M
            + VEHICLE_SEPARATION_MARGIN_M
        )
        self.physical_occlusion_engaged = (
            self.physical_occlusion_line_error_m <= OCCLUDER_LINE_TOLERANCE_M
            and self.physical_occlusion_cross_error_m <= OCCLUDER_CROSS_TOLERANCE_M
            and math.hypot(relative_x, relative_y) >= minimum_distance
            and self.physical_occlusion_crossed_los
        )

    @staticmethod
    def image_servo_goal(
        camera_x: float,
        camera_y: float,
        visual_state: dict[str, float | int | str | bool],
    ) -> tuple[float, float] | None:
        try:
            normalized_x = float(visual_state["normalized_x"])
            normalized_y = float(visual_state["normalized_y"])
            altitude = float(visual_state["sensor_altitude_m"])
            fov_deg = float(visual_state["sensor_fov_deg"])
            image_width = float(visual_state["image_width"])
            image_height = float(visual_state["image_height"])
        except (KeyError, TypeError, ValueError):
            return None
        if not (
            math.isfinite(normalized_x)
            and math.isfinite(normalized_y)
            and math.isfinite(altitude)
            and math.isfinite(fov_deg)
            and altitude > 1.0
            and 10.0 <= fov_deg <= 170.0
            and image_width > 0.0
            and image_height > 0.0
        ):
            return None
        focal = image_height / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
        image_x_offset = 0.85 * max(
            min(normalized_x - 0.5, 0.46), -0.46
        ) * image_width * altitude / focal
        image_y_offset = 0.85 * max(
            min(normalized_y - 0.5, 0.46), -0.46
        ) * image_height * altitude / focal
        return (
            camera_x + image_x_offset,
            camera_y - image_y_offset,
        )

    def enforce_vehicle_separation(self) -> None:
        entities: list[dict[str, object]] = []
        if self.target_motion is not None:
            entities.append({
                "id": 101,
                "kind": "target",
                "x": float(self.target_motion["x"]),
                "y": float(self.target_motion["y"]),
                "radius": TARGET_COLLISION_RADIUS_M,
                "priority": 0,
                "motion": self.target_motion,
            })
        for obstacle in self.current_dynamic_obstacles:
            entities.append({
                "id": int(obstacle["id"]),
                "kind": "dynamic",
                "x": float(obstacle["x"]),
                "y": float(obstacle["y"]),
                "radius": (
                    PHYSICAL_OCCLUDER_COLLISION_RADIUS_M
                    if int(obstacle["id"]) == self.physical_occluder_id
                    else DYNAMIC_COLLISION_RADIUS_M + 0.35 * float(obstacle["scale"])
                ),
                "priority": 1,
                "motion": self.dynamic_obstacle_motion.get(int(obstacle["id"])),
                "obstacle": obstacle,
            })
        for car_id, (x, y) in self.ground_car_positions.items():
            entities.append({
                "id": 201 + car_id,
                "kind": "ground",
                "x": float(x),
                "y": float(y),
                "radius": GROUND_COLLISION_RADIUS_M,
                "priority": 2,
                "car_id": car_id,
            })
        for obstacle_id, x, y, _yaw in PARKED_CARS:
            entities.append({
                "id": obstacle_id,
                "kind": "static",
                "x": x,
                "y": y,
                "radius": 4.4,
                "priority": 99,
            })
        for obstacle_id, _vehicle_type, x, y, _yaw, scale, _label in LARGE_OBSTACLES:
            entities.append({
                "id": obstacle_id,
                "kind": "static",
                "x": x,
                "y": y,
                "radius": max(5.5, 1.65 * scale),
                "priority": 99,
            })

        def apply_position(entity: dict[str, object], x: float, y: float) -> None:
            entity["x"], entity["y"] = x, y
            kind = str(entity["kind"])
            if kind == "ground":
                self.ground_car_positions[int(entity["car_id"])] = (x, y)
            elif kind == "target":
                motion = entity["motion"]
                if isinstance(motion, dict):
                    motion["x"], motion["y"] = x, y
            elif kind == "dynamic":
                motion = entity.get("motion")
                if isinstance(motion, dict):
                    motion["x"], motion["y"] = x, y
                obstacle = entity.get("obstacle")
                if isinstance(obstacle, dict):
                    obstacle["x"], obstacle["y"] = x, y

        self.collision_resolutions = 0
        self.collision_resolution_brakes = 0
        for _ in range(SEPARATION_SOLVER_ITERATIONS):
            changed = False
            for left_index, left in enumerate(entities):
                for right in entities[left_index + 1:]:
                    dx = float(right["x"]) - float(left["x"])
                    dy = float(right["y"]) - float(left["y"])
                    distance = math.hypot(dx, dy)
                    required = (
                        float(left["radius"])
                        + float(right["radius"])
                        + VEHICLE_SEPARATION_MARGIN_M
                    )
                    if distance >= required:
                        continue
                    if distance < 1e-6:
                        angle = 0.37 * (int(left["id"]) + int(right["id"]))
                        normal_x, normal_y = math.cos(angle), math.sin(angle)
                    else:
                        normal_x, normal_y = dx / distance, dy / distance
                    correction = (
                        required
                        + SEPARATION_CORRECTION_BUFFER_M
                        - max(distance, 0.0)
                    )
                    left_movable = str(left["kind"]) != "static"
                    right_movable = str(right["kind"]) != "static"
                    if not left_movable and not right_movable:
                        continue
                    if left_movable and right_movable:
                        left_share = 0.28 if int(left["priority"]) < int(right["priority"]) else 0.50
                    else:
                        left_share = 1.0 if left_movable else 0.0
                    right_share = 1.0 - left_share
                    left_x = float(left["x"]) - normal_x * correction * left_share
                    left_y = float(left["y"]) - normal_y * correction * left_share
                    right_x = float(right["x"]) + normal_x * correction * right_share
                    right_y = float(right["y"]) + normal_y * correction * right_share
                    if left_movable:
                        apply_position(left, left_x, left_y)
                    if right_movable:
                        apply_position(right, right_x, right_y)
                    self.collision_resolutions += 1
                    changed = True
                    for entity in (left, right):
                        if str(entity["kind"]) == "ground":
                            car_id = int(entity["car_id"])
                            self.ground_car_speeds[car_id] *= 0.35
                            ground_yaw = self.ground_car_yaws[car_id]
                            self.ground_car_velocities[car_id] = (
                                self.ground_car_speeds[car_id] * math.cos(ground_yaw),
                                self.ground_car_speeds[car_id] * math.sin(ground_yaw),
                            )
                            self.collision_resolution_brakes += 1
                        if str(entity["kind"]) == "target":
                            continue
                        motion = entity.get("motion")
                        if isinstance(motion, dict):
                            motion["speed"] = float(motion.get("speed", 0.0)) * 0.35
                            yaw = float(motion.get("yaw", 0.0))
                            motion["vx"] = float(motion["speed"]) * math.cos(yaw)
                            motion["vy"] = float(motion["speed"]) * math.sin(yaw)
                            obstacle = entity.get("obstacle")
                            if isinstance(obstacle, dict):
                                obstacle["speed"] = motion["speed"]
                                obstacle["vx"] = motion["vx"]
                                obstacle["vy"] = motion["vy"]
            if not changed:
                break

        self.vehicle_overlap_count = 0
        self.min_vehicle_distance_m = float("inf")
        self.min_vehicle_clearance_m = float("inf")
        for left_index, left in enumerate(entities):
            for right in entities[left_index + 1:]:
                distance = math.hypot(
                    float(right["x"]) - float(left["x"]),
                    float(right["y"]) - float(left["y"]),
                )
                required = (
                    float(left["radius"])
                    + float(right["radius"])
                    + VEHICLE_SEPARATION_MARGIN_M
                )
                self.min_vehicle_distance_m = min(self.min_vehicle_distance_m, distance)
                self.min_vehicle_clearance_m = min(
                    self.min_vehicle_clearance_m, distance - required
                )
                if distance < required:
                    self.vehicle_overlap_count += 1
        for car_id, (x, y) in self.ground_car_positions.items():
            self.ue.sendUE4PosScale2Ground(
                201 + car_id,
                GROUND_VEHICLE_TYPE,
                0.0,
                [x, y, 0.0],
                [0.0, 0.0, self.ground_car_yaws[car_id]],
                [1.8, 1.8, 1.8],
                windowID=self.rfly_window_id,
            )

    def spawn_static_scene(self) -> None:
        for obstacle_id, x, y, yaw in PARKED_CARS:
            self.ue.sendUE4PosScale2Ground(
                obstacle_id,
                GROUND_VEHICLE_TYPE,
                0.0,
                [x, y, 0.0],
                [0.0, 0.0, yaw],
                [1.6, 1.6, 1.6],
                windowID=self.rfly_window_id,
            )
        for obstacle_id, vehicle_type, x, y, yaw, scale, _label in LARGE_OBSTACLES:
            self.ue.sendUE4PosScale2Ground(
                obstacle_id,
                vehicle_type,
                0.0,
                [x, y, 0.0],
                [0.0, 0.0, yaw],
                [scale, scale, scale],
                windowID=self.rfly_window_id,
            )

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
        for _, _, x, y, _, _, _ in LARGE_OBSTACLES:
            center_x = int(round(x))
            center_y = int(round(y))
            for cell_y in range(center_y - 8, center_y + 9):
                for cell_x in range(center_x - 8, center_x + 9):
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

    def set_camera_fov(self, fov_deg: float) -> None:
        target_fov = min(
            max(float(fov_deg), CAMERA_LOCK_FOV_DEG), CAMERA_SEARCH_FOV_DEG
        )
        fov_error = target_fov - self.current_camera_fov_deg
        if abs(fov_error) < 0.1:
            return
        next_fov = self.current_camera_fov_deg + max(
            min(fov_error, CAMERA_FOV_SLEW_DEG_PER_UPDATE),
            -CAMERA_FOV_SLEW_DEG_PER_UPDATE,
        )
        self.ue.sendUE4Cmd(
            f"RflyCameraFovDegrees {next_fov:.1f}",
            self.rfly_window_id,
        )
        self.current_camera_fov_deg = next_fov
        self.locked_camera_fov = next_fov < CAMERA_SEARCH_FOV_DEG - 0.1

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
            self.camera_velocity = (0.0, 0.0)
            self.camera_yaw_rate = 0.0
            self.last_camera_update_at = t
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
        provisional_candidates = [
            (track_id, state)
            for track_id, state in self.vision_tracks.items()
            if t - float(state["seen_at"]) <= VISION_STALE_SECONDS
            and str(state["label"]) in {"car", "truck", "bus", "vehicle"}
            and not bool(state["confirmed"])
        ]
        host_provisional_candidates = [
            item
            for item in provisional_candidates
            if int(item[1]["host_id"]) == self.active_host_id
        ]
        provisional_selected = max(
            host_provisional_candidates or provisional_candidates,
            key=lambda item: (
                float(item[1]["seen_at"]),
                float(item[1]["area"]),
                float(item[1]["confidence"]),
            ),
        ) if (host_provisional_candidates or provisional_candidates) else None

        if self.calibration_camera_assist:
            # Calibration-only camera steering keeps the rendered target in view while
            # RGB measurements are recorded.  It never modifies visual world tracks.
            self.set_camera_fov(CAMERA_SEARCH_FOV_DEG)
            self.search_bootstrap_active = False
            self.control_mode = "calibration_camera_assist"
            self.target_control_source = "calibration_truth_camera_only"
            lead_seconds = 0.45
            calibration_phase = 0.11 * t
            desired_x = (
                primary_x + primary_vx * lead_seconds
                + 38.0 * math.sin(calibration_phase)
            )
            desired_y = (
                primary_y + primary_vy * lead_seconds
                + 34.0 * math.cos(calibration_phase)
            )
            desired_yaw = math.atan2(primary_vy, primary_vx)
            self.camera_altitude += 0.05 * (
                CAMERA_SEARCH_MAX_ALTITUDE_M - self.camera_altitude
            )
        elif selected is None and provisional_selected is not None:
            # An edge candidate can steer the camera into a better view but is
            # intentionally excluded from target publication and containment.
            _, candidate_state = provisional_selected
            self.set_camera_fov(CAMERA_SEARCH_FOV_DEG)
            self.control_mode = "candidate_search"
            self.search_bootstrap_active = False
            self.target_control_source = "candidate_image_only"
            if not self.physical_occlusion_window_active(t):
                self.clear_physical_occlusion()
            servo_goal = self.image_servo_goal(
                camera_x,
                camera_y,
                candidate_state,
            )
            if servo_goal is not None:
                desired_x = camera_x + 0.45 * (servo_goal[0] - camera_x)
                desired_y = camera_y + 0.45 * (servo_goal[1] - camera_y)
            desired_yaw = camera_yaw - 0.60 * math.radians(
                self.current_camera_fov_deg
            ) * (float(candidate_state["normalized_x"]) - 0.5)
            self.camera_altitude += 0.08 * (
                CAMERA_SEARCH_MAX_ALTITUDE_M - self.camera_altitude
            )
        elif selected is not None:
            selected_track_id, visual_state = selected
            if self.visual_lock_id != selected_track_id:
                self.camera_velocity = (0.0, 0.0)
                self.camera_yaw_rate = 0.0
                self.physical_occlusion_lock_started_at = t
                if not self.physical_occlusion_engaged:
                    self.physical_occlusion_epoch_at = -1.0
            elif self.physical_occlusion_lock_started_at < 0.0:
                self.physical_occlusion_lock_started_at = t
            lock_elapsed = t - self.physical_occlusion_lock_started_at
            if (
                self.physical_occlusion_enabled
                and lock_elapsed >= PHYSICAL_OCCLUSION_LOCK_STABILITY_S
            ):
                self.physical_occlusion_prepositioning = True
                self.physical_occlusion_lock_qualified = True
            image_center_error = float(visual_state["center_error"])
            fov_lock_limit = (
                CAMERA_FOV_RELEASE_CENTER_ERROR
                if self.locked_camera_fov
                else CAMERA_FOV_ACQUIRE_CENTER_ERROR
            )
            self.set_camera_fov(
                CAMERA_LOCK_FOV_DEG
                if (
                    image_center_error <= fov_lock_limit
                    and not self.physical_occlusion_engaged
                )
                else CAMERA_SEARCH_FOV_DEG
            )
            self.search_bootstrap_active = False
            self.visual_lock_id = selected_track_id
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
                predicted_x = float(previous["x"]) + float(previous["vx"]) * delta
                predicted_y = float(previous["y"]) + float(previous["vy"]) * delta
                measured_vx = float(visual_state["vx"])
                measured_vy = float(visual_state["vy"])
                measured_speed = math.hypot(measured_vx, measured_vy)
                if measured_speed > MAX_VISUAL_SPEED_MPS:
                    scale = MAX_VISUAL_SPEED_MPS / measured_speed
                    measured_vx *= scale
                    measured_vy *= scale
                filtered_x = 0.55 * measurement_x + 0.45 * predicted_x
                filtered_y = 0.55 * measurement_y + 0.45 * predicted_y
                filtered_vx = 0.48 * measured_vx + 0.52 * float(previous["vx"])
                filtered_vy = 0.48 * measured_vy + 0.52 * float(previous["vy"])
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
                "raw_x": float(visual_state.get("raw_x", measurement_x)),
                "raw_y": float(visual_state.get("raw_y", measurement_y)),
                "observation_age_s": float(visual_state.get("observation_age_s", 0.0)),
            }
            self.visual_target_state = {
                "x": filtered_x,
                "y": filtered_y,
                "vx": filtered_vx,
                "vy": filtered_vy,
                "heading": filtered_heading,
                "confidence": float(visual_state["confidence"]),
            }
            self.target_control_source = "vision"
            self.control_mode = "track"
            self.visual_target_last_seen = t
            self.visual_lock_last_seen = t
            self.control_prediction_lead_s = min(
                1.35,
                0.72 + 0.02 * math.hypot(filtered_vx, filtered_vy),
            )
            image_error_x = float(visual_state["normalized_x"]) - 0.5
            image_error_y = float(visual_state["normalized_y"]) - 0.5
            world_goal_x = filtered_x + filtered_vx * self.control_prediction_lead_s
            world_goal_y = filtered_y + filtered_vy * self.control_prediction_lead_s
            servo_goal = self.image_servo_goal(
                camera_x,
                camera_y,
                visual_state,
            )
            if lock_elapsed < 0.45:
                desired_x, desired_y = camera_x, camera_y
            elif servo_goal is None:
                desired_x, desired_y = world_goal_x, world_goal_y
            else:
                # Pixel feedback keeps the target centred even while a new camera
                # pose is settling or the calibrated world projection is noisy.
                servo_delta_x = servo_goal[0] - camera_x
                servo_delta_y = servo_goal[1] - camera_y
                pursuit_delta_x = world_goal_x - camera_x
                pursuit_delta_y = world_goal_y - camera_y
                if (
                    servo_delta_x * pursuit_delta_x
                    + servo_delta_y * pursuit_delta_y
                    < 0.0
                ):
                    desired_x, desired_y = world_goal_x, world_goal_y
                else:
                    desired_x, desired_y = servo_goal
            desired_yaw = camera_yaw - 0.60 * math.radians(
                self.current_camera_fov_deg
            ) * image_error_x
            target_height_fraction = float(visual_state["height_fraction"])
            edge_error = float(visual_state["center_error"])
            altitude_adjustment = (target_height_fraction - 0.08) * 55.0
            if edge_error > 0.72:
                altitude_adjustment += 4.0
            altitude_target = min(
                max(CAMERA_ALTITUDE_M + altitude_adjustment, CAMERA_MIN_ALTITUDE_M),
                CAMERA_MAX_ALTITUDE_M,
            )
            altitude_gain = 0.022
            if self.prepare_physical_occlusion_camera(t):
                altitude_target = max(
                    altitude_target,
                    CAMERA_OCCLUSION_ALTITUDE_M,
                )
                altitude_gain = 0.08
            self.camera_altitude += altitude_gain * (
                altitude_target - self.camera_altitude
            )
            if self.frame_idx % 30 == 0:
                self.get_logger().info(
                    f"visual lock active: track={self.visual_lock_id} "
                    f"projected=({filtered_x:.1f},{filtered_y:.1f}) "
                    f"speed={math.hypot(filtered_vx, filtered_vy):.1f} m/s "
                    f"altitude={self.camera_altitude:.1f} m"
                )
        elif self.visual_target_state is not None and t - self.visual_target_last_seen <= 2.0:
            if not self.physical_occlusion_window_active(t):
                self.clear_physical_occlusion()
            self.set_camera_fov(CAMERA_SEARCH_FOV_DEG)
            self.search_bootstrap_active = False
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
            follow_distance = min(
                max(
                    self.camera_altitude
                    / math.tan(math.radians(abs(CAMERA_SENSOR_PITCH_DEG))),
                    42.0,
                ),
                80.0,
            )
            desired_x = target_x - follow_distance * vx / speed
            desired_y = target_y - follow_distance * vy / speed
            desired_yaw = math.atan2(vy, vx)
        elif self.vision_stream_started and t - self.visual_target_last_seen > 2.0:
            if not self.physical_occlusion_window_active(t):
                self.clear_physical_occlusion()
            self.set_camera_fov(CAMERA_SEARCH_FOV_DEG)
            self.control_mode = "search"
            self.visual_lock_id = None
            if t - self.visual_target_last_seen > 4.0:
                self.visual_target_state = None
                self.visual_projection_state = None
                self.target_control_source = "none"
            bootstrap_age = (
                t - self.vision_first_packet_at
                if self.vision_first_packet_at >= 0.0
                else float("inf")
            )
            self.search_bootstrap_active = (
                self.vision_track_packet_count == 0
                and bootstrap_age <= INITIAL_SEARCH_ASSIST_SECONDS
            )
            self.search_bootstrap_active = False
            self.camera_altitude = min(
                self.camera_altitude + 0.04,
                CAMERA_SEARCH_MAX_ALTITUDE_M,
            )
            base_x, base_y, base_yaw = self.search_waypoint(self.active_host_id, t)
            self.camera_search_yaw += math.radians(CAMERA_SEARCH_YAW_STEP_DEG)
            desired_yaw = base_yaw + self.camera_search_yaw
            desired_x = base_x
            desired_y = base_y
        camera_dt = (
            0.05
            if self.last_camera_update_at < 0.0
            else min(max(t - self.last_camera_update_at, 0.01), 0.15)
        )
        self.last_camera_update_at = t
        desired_step_x = max(
            min(0.35 * (desired_x - camera_x), CAMERA_MAX_STEP_M),
            -CAMERA_MAX_STEP_M,
        )
        desired_step_y = max(
            min(0.35 * (desired_y - camera_y), CAMERA_MAX_STEP_M),
            -CAMERA_MAX_STEP_M,
        )
        velocity_x, velocity_y = self.camera_velocity
        velocity_x += max(
            min(
                desired_step_x - velocity_x,
                CAMERA_MAX_ACCELERATION_STEP_M * camera_dt,
            ),
            -CAMERA_MAX_ACCELERATION_STEP_M * camera_dt,
        )
        velocity_y += max(
            min(
                desired_step_y - velocity_y,
                CAMERA_MAX_ACCELERATION_STEP_M * camera_dt,
            ),
            -CAMERA_MAX_ACCELERATION_STEP_M * camera_dt,
        )
        velocity_norm = math.hypot(velocity_x, velocity_y)
        if velocity_norm > CAMERA_MAX_SPEED_M:
            velocity_x *= CAMERA_MAX_SPEED_M / velocity_norm
            velocity_y *= CAMERA_MAX_SPEED_M / velocity_norm
        move_x, move_y = velocity_x * camera_dt, velocity_y * camera_dt
        self.camera_velocity = (velocity_x, velocity_y)
        camera_x += move_x
        camera_y += move_y
        yaw_error = (desired_yaw - camera_yaw + math.pi) % (2.0 * math.pi) - math.pi
        desired_yaw_rate = max(
            min(1.15 * yaw_error, math.radians(CAMERA_MAX_YAW_STEP_DEG)),
            math.radians(-CAMERA_MAX_YAW_STEP_DEG),
        )
        self.camera_yaw_rate += max(
            min(
                desired_yaw_rate - self.camera_yaw_rate,
                math.radians(CAMERA_MAX_YAW_ACCELERATION_DEG) * camera_dt,
            ),
            math.radians(-CAMERA_MAX_YAW_ACCELERATION_DEG) * camera_dt,
        )
        camera_yaw += self.camera_yaw_rate * camera_dt
        self.camera_state = (camera_x, camera_y, camera_yaw)
        self.search_uav_states[self.active_host_id] = self.camera_state

        wind_x = 0.10 * self.wind_speed_mps * (
            math.sin(0.29 * t) + 0.28 * math.sin(0.83 * t + 0.4)
        ) * math.cos(self.wind_direction_rad)
        wind_y = 0.10 * self.wind_speed_mps * (
            math.sin(0.29 * t) + 0.28 * math.sin(0.83 * t + 0.4)
        ) * math.sin(self.wind_direction_rad)
        rendered_x = camera_x + wind_x
        rendered_y = camera_y + wind_y
        altitude = self.camera_altitude + self.bump_scale * (
            0.35 * math.sin(1.1 * t) + 0.12 * math.sin(3.8 * t)
        ) + 0.025 * self.wind_speed_mps * math.sin(0.41 * t + 0.8)
        roll_deg = (
            self.bump_scale * (0.65 * math.sin(1.4 * t) + 0.18 * math.sin(4.3 * t))
            + 0.06 * self.wind_speed_mps * math.sin(0.37 * t + 0.2)
        )
        pitch_deg = (
            self.bump_scale * (0.45 * math.sin(1.2 * t) + 0.14 * math.sin(3.7 * t))
            + 0.045 * self.wind_speed_mps * math.sin(0.33 * t + 1.1)
        )
        self.camera_pose = (
            rendered_x,
            rendered_y,
            altitude,
            math.radians(roll_deg),
            math.radians(pitch_deg),
            camera_yaw,
        )
        self.camera_poses[self.active_host_id] = self.camera_pose
        self.ue.sendUE4Pos(
            CAMERA_CARRIER_ID,
            3,
            700.0,
            [rendered_x, rendered_y, -altitude],
            [math.radians(roll_deg), math.radians(pitch_deg), camera_yaw],
            windowID=self.rfly_window_id,
        )

        if t - self.last_camera_command_at >= RFLY_CAMERA_COMMAND_PERIOD_S:
            view_pitch = CAMERA_SENSOR_PITCH_DEG + pitch_deg
            self.ue.sendUE4Cmd(
                "RflyCameraPosAng "
                f"{rendered_x:.3f} {rendered_y:.3f} {-altitude:.3f} "
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
            wind_sway = 0.05 * self.wind_speed_mps * math.sin(0.35 * t + phase)
            rendered_x = x + wind_sway * math.cos(self.wind_direction_rad)
            rendered_y = y + wind_sway * math.sin(self.wind_direction_rad)
            roll += math.radians(0.035 * self.wind_speed_mps * math.sin(0.31 * t + phase))
            pitch += math.radians(0.025 * self.wind_speed_mps * math.sin(0.27 * t + phase))
            pose = (rendered_x, rendered_y, altitude, roll, pitch, yaw)
            self.camera_poses[host_id] = pose
            self.ue.sendUE4Pos(
                host_id,
                3,
                700.0,
                [rendered_x, rendered_y, -altitude],
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
        camera_pose: tuple[float, float, float] | None = None,
        camera_fov_deg: float | None = None,
    ) -> tuple[float, float] | None:
        pose = camera_pose or self.camera_poses.get(host_id)
        if pose is None:
            return None
        camera_x, camera_y, altitude, _roll, _pitch, _yaw = pose
        altitude = abs(altitude)
        fov_deg = camera_fov_deg or self.current_camera_fov_deg
        focal = height / (
            2.0 * math.tan(math.radians(fov_deg) / 2.0)
        )
        image_ground_x = (image_x - width / 2.0) * altitude / focal
        image_ground_y = (image_y - height / 2.0) * altitude / focal
        if self.camera_ground_calibration is None:
            projected = (
                camera_x - image_ground_x,
                camera_y + image_ground_y,
            )
        else:
            x_coefficients, y_coefficients = self.camera_ground_calibration
            features = (1.0, camera_x, camera_y, image_ground_x, image_ground_y)
            projected = (
                sum(coefficient * feature for coefficient, feature in zip(x_coefficients, features)),
                sum(coefficient * feature for coefficient, feature in zip(y_coefficients, features)),
            )
        if not (
            -8.0 <= projected[0] <= GRID_SIZE + 8.0
            and -8.0 <= projected[1] <= GRID_SIZE + 8.0
        ):
            return None
        return projected

    def scene_status_payload(self, now: float) -> dict[str, object]:
        return {
            "type": "rfly_scene_status",
            "time_s": round(now, 3),
            "scenario": self.scenario_name,
            "weather": self.weather_name,
            "weather_type": self.weather_type,
            "camera_fov_deg": self.current_camera_fov_deg,
            "camera_fov_locked": self.locked_camera_fov,
            "calibration_camera_assist": self.calibration_camera_assist,
            "ground_projection": (
                "calibrated_affine_v1"
                if self.camera_ground_calibration is not None
                else "legacy_pinhole"
            ),
            "target_truth": (
                None
                if self.target_motion is None
                else {
                    "x": float(self.target_motion["x"]),
                    "y": float(self.target_motion["y"]),
                    "vx": float(self.target_motion["vx"]),
                    "vy": float(self.target_motion["vy"]),
                }
            ),
            "uavs": self.camera_poses,
            "active_host_id": self.active_host_id,
            "physical_occlusion_requested": self.physical_occlusion_requested,
            "physical_occlusion_engaged": self.physical_occlusion_engaged,
            "physical_occlusion_armed": self.physical_occlusion_epoch_at >= 0.0,
            "physical_occlusion_prepositioning": self.physical_occlusion_prepositioning,
            "physical_occlusion_lock_qualified": self.physical_occlusion_lock_qualified,
            "physical_occlusion_crossed_los": self.physical_occlusion_crossed_los,
            "physical_occluder_id": self.physical_occluder_id,
            "physical_occlusion_alignment_m": (
                None
                if not math.isfinite(self.physical_occlusion_alignment_m)
                else self.physical_occlusion_alignment_m
            ),
            "physical_occlusion_line_error_m": (
                None
                if not math.isfinite(self.physical_occlusion_line_error_m)
                else self.physical_occlusion_line_error_m
            ),
            "physical_occlusion_cross_error_m": (
                None
                if not math.isfinite(self.physical_occlusion_cross_error_m)
                else self.physical_occlusion_cross_error_m
            ),
            "physical_occlusion_ray_height_m": (
                None
                if not math.isfinite(self.physical_occlusion_ray_height_m)
                else self.physical_occlusion_ray_height_m
            ),
            "physical_occluder_vertical_scale": PHYSICAL_OCCLUDER_VERTICAL_SCALE,
            "min_vehicle_distance_m": self.min_vehicle_distance_m,
            "min_vehicle_clearance_m": self.min_vehicle_clearance_m,
            "vehicle_overlap_count": self.vehicle_overlap_count,
            "collision_resolutions": self.collision_resolutions,
        }

    def send_vision_status(
        self,
        now: float,
        fallback_address: tuple[str, int] | None = None,
    ) -> None:
        if now - self.last_vision_status_sent_at < RFLY_STATUS_UPDATE_PERIOD_S:
            return
        target_address = self.vision_status_address or fallback_address
        if target_address is None:
            return
        try:
            self.vision_status_socket.sendto(
                json.dumps(self.scene_status_payload(now)).encode("utf-8"),
                target_address,
            )
        except OSError:
            return
        self.last_vision_status_sent_at = now

    def receive_visual_tracks(self, now: float) -> None:
        packets = []
        while True:
            try:
                payload, source_address = self.vision_socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                packets.append((json.loads(payload.decode("utf-8")), source_address))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
        if not packets:
            return
        self.vision_stream_started = True
        if self.vision_first_packet_at < 0.0:
            self.vision_first_packet_at = now
        self.vision_packet_count += len(packets)
        self.last_vision_packet_at = now
        for latest, source_address in packets:
            host_id = int(latest.get("host_id", 1))
            requested_host = int(latest.get("active_host", self.active_host_id))
            if requested_host in (1, 2, 3):
                self.active_host_id = requested_host
            width = int(latest.get("width", 0))
            height = int(latest.get("height", 0))
            if host_id not in (1, 2, 3) or width <= 0 or height <= 0:
                continue
            sensor_pose = None
            raw_sensor_pose = latest.get("sensor_pose")
            if isinstance(raw_sensor_pose, (list, tuple)) and len(raw_sensor_pose) >= 3:
                try:
                    candidate_pose = tuple(float(value) for value in raw_sensor_pose[:3])
                except (TypeError, ValueError):
                    candidate_pose = ()
                if len(candidate_pose) == 3 and all(math.isfinite(value) for value in candidate_pose):
                    sensor_pose = (*candidate_pose, 0.0, 0.0, 0.0)
            sensor_altitude = (
                abs(float(sensor_pose[2]))
                if sensor_pose is not None
                else self.camera_altitude
            )
            try:
                sensor_fov_deg = float(latest.get("sensor_fov_deg", self.current_camera_fov_deg))
            except (TypeError, ValueError):
                sensor_fov_deg = self.current_camera_fov_deg
            if not math.isfinite(sensor_fov_deg) or not 10.0 <= sensor_fov_deg <= 170.0:
                sensor_fov_deg = self.current_camera_fov_deg
            try:
                observation_time = float(latest.get("sensor_scene_time_s", now))
            except (TypeError, ValueError):
                observation_time = now
            if not math.isfinite(observation_time) or observation_time > now + 0.25:
                observation_time = now
            observation_age = min(
                max(now - observation_time, 0.0),
                MAX_VISUAL_TRANSPORT_LATENCY_S,
            )
            for item in latest.get("tracks", []):
                self.vision_track_packet_count += 1
                try:
                    local_track_id = int(item["track_id"])
                    track_id = host_id * 10000 + local_track_id
                    image_x = (float(item["x1"]) + float(item["x2"])) / 2.0
                    image_y = (float(item["y1"]) + float(item["y2"])) / 2.0
                    raw_projected = self.image_to_ground(
                        host_id,
                        image_x,
                        image_y,
                        width,
                        height,
                        camera_pose=sensor_pose,
                        camera_fov_deg=sensor_fov_deg,
                    )
                except (KeyError, TypeError, ValueError):
                    continue
                if raw_projected is None:
                    continue
                previous = self.vision_tracks.get(track_id)
                vx = 0.0
                vy = 0.0
                projected = raw_projected
                if previous is not None:
                    delta = observation_time - float(
                        previous.get("observation_time", previous["seen_at"])
                    )
                    if 0.02 <= delta <= 2.0:
                        raw_vx = (
                            raw_projected[0] - float(previous.get("raw_x", previous["x"]))
                        ) / delta
                        raw_vy = (
                            raw_projected[1] - float(previous.get("raw_y", previous["y"]))
                        ) / delta
                        raw_speed = math.hypot(raw_vx, raw_vy)
                        if raw_speed <= MAX_VISUAL_SPEED_MPS:
                            predicted_raw_x = float(previous.get("raw_x", previous["x"])) + float(
                                previous["vx"]
                            ) * delta
                            predicted_raw_y = float(previous.get("raw_y", previous["y"])) + float(
                                previous["vy"]
                            ) * delta
                            projected = (
                                0.42 * raw_projected[0] + 0.58 * predicted_raw_x,
                                0.42 * raw_projected[1] + 0.58 * predicted_raw_y,
                            )
                            vx = 0.22 * raw_vx + 0.78 * float(previous["vx"])
                            vy = 0.22 * raw_vy + 0.78 * float(previous["vy"])
                        else:
                            projected = (
                                float(previous.get("raw_x", previous["x"])),
                                float(previous.get("raw_y", previous["y"])),
                            )
                            vx = float(previous["vx"])
                            vy = float(previous["vy"])
                current_projected = (
                    projected[0] + vx * observation_age,
                    projected[1] + vy * observation_age,
                )
                box_width = max(0.0, float(item["x2"]) - float(item["x1"]))
                box_height = max(0.0, float(item["y2"]) - float(item["y1"]))
                normalized_x = image_x / width
                normalized_y = ((float(item["y1"]) + float(item["y2"])) / 2.0) / height
                self.vision_tracks[track_id] = {
                    "host_id": host_id,
                    "x": current_projected[0],
                    "y": current_projected[1],
                    "raw_x": projected[0],
                    "raw_y": projected[1],
                    "vx": vx,
                    "vy": vy,
                    "confidence": float(item.get("confidence", 0.0)),
                    "cls": int(item.get("cls", 2)),
                    "label": str(item.get("label", "vehicle")),
                    "confirmed": bool(item.get("confirmed", True)),
                    "seen_at": now,
                    "observation_time": observation_time,
                    "observation_age_s": observation_age,
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
                    "sensor_altitude_m": sensor_altitude,
                    "sensor_fov_deg": sensor_fov_deg,
                    "image_width": width,
                    "image_height": height,
                }
                if bool(item.get("confirmed", True)):
                    if self.last_visual_observation_at >= 0.0:
                        gap = now - self.last_visual_observation_at
                        if gap >= 0.12:
                            self.reacquisition_count += 1
                            self.last_reacquisition_latency_s = round(gap, 3)
                    self.last_visual_observation_at = now
            self.send_vision_status(now, source_address)

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
        updated = 0
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
            updated += 1
        if updated:
            self.ground_goal_update_count += 1
            if self.ground_goal_update_count == 1 or self.ground_goal_update_count % 20 == 0:
                self.get_logger().info(
                    f"ground enclosure update {self.ground_goal_update_count}: "
                    f"{updated} finite command(s)"
                )

    def update_ground_interceptors(self, t: float) -> None:
        dt = (
            0.0
            if self.last_ground_update_time < 0.0
            else min(max(t - self.last_ground_update_time, 0.0), 0.15)
        )
        self.last_ground_update_time = t
        for car_id, (current_x, current_y) in self.ground_car_positions.items():
            goal = self.ground_car_goals.get(car_id)
            yaw = self.ground_car_yaws[car_id]
            speed = self.ground_car_speeds[car_id]
            yaw_rate = self.ground_car_yaw_rates[car_id]
            if goal is None:
                base_heading = yaw
                desired_speed = 0.0
            else:
                delta_x = goal[0] - current_x
                delta_y = goal[1] - current_y
                distance = math.hypot(delta_x, delta_y)
                base_heading = math.atan2(delta_y, delta_x)
                desired_speed = 0.0 if distance < 1.5 else min(9.2, distance * 0.55)
            avoidance_points = [
                (float(self.target_motion["x"]), float(self.target_motion["y"]), GROUND_VEHICLE_CLEARANCE_M)
            ] if self.target_motion is not None else []
            avoidance_points.extend(
                (other_x, other_y, GROUND_VEHICLE_CLEARANCE_M)
                for other_id, (other_x, other_y) in self.ground_car_positions.items()
                if other_id != car_id
            )
            avoidance_points.extend(
                (float(obstacle["x"]), float(obstacle["y"]), OBSTACLE_CLEARANCE_M)
                for obstacle in self.current_dynamic_obstacles
            )
            avoidance_points.extend((entry[2], entry[3], OBSTACLE_CLEARANCE_M) for entry in LARGE_OBSTACLES)
            desired_heading = self.avoidance_heading(
                base_heading,
                current_x,
                current_y,
                avoidance_points,
            )
            motion: dict[str, float | int] = {
                "x": current_x,
                "y": current_y,
                "yaw": yaw,
                "yaw_rate": yaw_rate,
                "speed": speed,
                "vx": speed * math.cos(yaw),
                "vy": speed * math.sin(yaw),
            }
            self.advance_forward_vehicle(
                motion,
                desired_heading,
                desired_speed,
                dt,
                max_speed=GROUND_MAX_SPEED_MPS,
                acceleration=GROUND_ACCEL_MPS2,
                braking=GROUND_BRAKE_MPS2,
                max_yaw_rate=GROUND_MAX_YAW_RATE_RPS,
                max_yaw_acceleration=0.72,
                min_turn_radius=GROUND_MIN_TURN_RADIUS_M,
            )
            current_x = float(motion["x"])
            current_y = float(motion["y"])
            yaw = float(motion["yaw"])
            self.ground_car_positions[car_id] = (current_x, current_y)
            self.ground_car_velocities[car_id] = (
                float(motion["vx"]),
                float(motion["vy"]),
            )
            self.ground_car_speeds[car_id] = float(motion["speed"])
            self.ground_car_yaws[car_id] = yaw
            self.ground_car_yaw_rates[car_id] = float(motion["yaw_rate"])

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
            velocity_x, velocity_y = self.ground_car_velocities[car_id]
            state = DroneState()
            state.drone_id = car_id
            state.x = x
            state.y = y
            state.z = 0.0
            state.vx = velocity_x
            state.vy = velocity_y
            state.vz = 0.0
            state.available = True
            state.platform_type = 1
            ground_message.drones.append(state)
        self.ground_state_publisher.publish(ground_message)

    def tick(self) -> None:
        t = time.monotonic() - self.start_time
        states = self.car_states(t)
        self.update_search_uavs(t)
        self.receive_visual_tracks(t)
        self.update_camera_uav(t, states[0])
        self.current_dynamic_obstacles = self.dynamic_obstacle_states(t, states[0])
        self.update_ground_interceptors(t)
        self.enforce_vehicle_separation()
        self.refresh_physical_occlusion_alignment(
            float(self.target_motion["x"]),
            float(self.target_motion["y"]),
        )
        self.send_vision_status(t)
        states = [(
            101,
            float(self.target_motion["x"]),
            float(self.target_motion["y"]),
            float(self.target_motion["vx"]),
            float(self.target_motion["vy"]),
        )]
        self.publish_platform_states()
        truth_output = TargetTrackArray()
        truth_output.header.stamp = self.get_clock().now().to_msg()
        truth_output.header.frame_id = "world"
        truth_output.frame_idx = self.frame_idx
        truth_output.tracks = []

        if not self.static_scene_spawned and t >= STATIC_SCENE_BOOTSTRAP_DELAY_S:
            self.spawn_static_scene()
            self.static_scene_spawned = True

        for obstacle in self.current_dynamic_obstacles:
            self.ue.sendUE4PosScale2Ground(
                int(obstacle["id"]),
                int(obstacle["vehicle_type"]),
                0.0,
                [float(obstacle["x"]), float(obstacle["y"]), 0.0],
                [0.0, 0.0, float(obstacle["yaw"])],
                [float(value) for value in obstacle["scale_xyz"]],
                windowID=self.rfly_window_id,
            )

        for target_id, x, y, vx, vy in states:
            yaw = math.atan2(vy, vx)
            self.ue.sendUE4PosScale2Ground(
                target_id,
                TARGET_VEHICLE_TYPE,
                0,
                [x, y, 0.0],
                [0.0, 0.0, yaw],
                [3.0 if target_id == 101 else 1.7,
                 3.0 if target_id == 101 else 1.7,
                 3.0 if target_id == 101 else 1.7],
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
                "wind_speed_mps": self.wind_speed_mps,
                "wind_direction_deg": self.wind_direction_deg,
                "occlusion_level": self.occlusion_level,
                "camera_fov_deg": self.current_camera_fov_deg,
                "camera_fov_locked": self.locked_camera_fov,
                "physical_occlusion_requested": self.physical_occlusion_requested,
                "physical_occlusion_engaged": self.physical_occlusion_engaged,
                "physical_occlusion_armed": self.physical_occlusion_epoch_at >= 0.0,
                "physical_occlusion_prepositioning": self.physical_occlusion_prepositioning,
                "physical_occlusion_lock_qualified": self.physical_occlusion_lock_qualified,
                "physical_occlusion_crossed_los": self.physical_occlusion_crossed_los,
                "physical_occluder_id": self.physical_occluder_id,
                "physical_occlusion_alignment_m": (
                    None
                    if not math.isfinite(self.physical_occlusion_alignment_m)
                    else self.physical_occlusion_alignment_m
                ),
                "physical_occlusion_line_error_m": (
                    None
                    if not math.isfinite(self.physical_occlusion_line_error_m)
                    else self.physical_occlusion_line_error_m
                ),
                "physical_occlusion_cross_error_m": (
                    None
                    if not math.isfinite(self.physical_occlusion_cross_error_m)
                    else self.physical_occlusion_cross_error_m
                ),
                "physical_occlusion_ray_height_m": (
                    None
                    if not math.isfinite(self.physical_occlusion_ray_height_m)
                    else self.physical_occlusion_ray_height_m
                ),
                "physical_occluder_vertical_scale": PHYSICAL_OCCLUDER_VERTICAL_SCALE,
                "min_vehicle_distance_m": self.min_vehicle_distance_m,
                "min_vehicle_clearance_m": self.min_vehicle_clearance_m,
                "vehicle_overlap_count": self.vehicle_overlap_count,
                "collision_resolutions": self.collision_resolutions,
                "collision_resolution_brakes": self.collision_resolution_brakes,
                "prediction_lead_s": self.control_prediction_lead_s,
                "active_host": self.active_host_id,
                "target_truth": {
                    "id": target_id,
                    "x": truth_x,
                    "y": truth_y,
                    "vx": truth_vx,
                    "vy": truth_vy,
                    "yaw": float(self.target_motion["yaw"]),
                    "yaw_rate": float(self.target_motion["yaw_rate"]),
                },
                "target_visual": self.visual_projection_state,
                "target_visual_error_m": (
                    None
                    if self.visual_projection_state is None
                    else math.hypot(
                        float(self.visual_projection_state["x"]) - truth_x,
                        float(self.visual_projection_state["y"]) - truth_y,
                    )
                ),
                "target_control": visual,
                "target_control_source": self.target_control_source,
                "search_bootstrap_active": self.search_bootstrap_active,
                "target_speed_mps": math.hypot(truth_vx, truth_vy),
                "target_heading_deg": math.degrees(math.atan2(truth_vy, truth_vx)),
                "target_vehicle_type": TARGET_VEHICLE_TYPE,
                "target_vehicle_asset": "Rfly Standard_Car_Blue",
                "ground_vehicle_type": GROUND_VEHICLE_TYPE,
                "target_waypoint_index": int(self.target_motion["waypoint_index"]),
                "vision_stream_started": self.vision_stream_started,
                "vision_port": self.vision_port,
                "vision_packet_count": self.vision_packet_count,
                "vision_track_packet_count": self.vision_track_packet_count,
                "reacquisition_active": (
                    self.last_visual_observation_at >= 0.0
                    and t - self.last_visual_observation_at >= 0.12
                    and t - self.last_visual_observation_at <= 2.0
                ),
                "reacquisition_count": self.reacquisition_count,
                "last_reacquisition_latency_s": self.last_reacquisition_latency_s,
                "last_vision_packet_age_s": (
                    None
                    if self.last_vision_packet_at < 0.0
                    else round(t - self.last_vision_packet_at, 3)
                ),
                "uavs": self.camera_poses,
                "ground_cars": self.ground_car_positions,
                "ground_goals": self.ground_car_goals,
                "ground_kinematics": {
                    car_id: {
                        "yaw": self.ground_car_yaws[car_id],
                        "yaw_rate": self.ground_car_yaw_rates[car_id],
                        "speed": self.ground_car_speeds[car_id],
                        "vx": self.ground_car_velocities[car_id][0],
                        "vy": self.ground_car_velocities[car_id][1],
                    }
                    for car_id in self.ground_car_positions
                },
                "ground_goal_update_count": self.ground_goal_update_count,
                "large_obstacles": [
                    {
                        "id": obstacle_id,
                        "vehicle_type": vehicle_type,
                        "x": x,
                        "y": y,
                        "yaw": yaw,
                        "scale": scale,
                        "label": label,
                    }
                    for obstacle_id, vehicle_type, x, y, yaw, scale, label in LARGE_OBSTACLES
                ],
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
