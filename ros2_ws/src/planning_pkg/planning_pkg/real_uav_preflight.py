"""Read-only technical preflight monitor for a single PX4/MAVROS vehicle.

The node has subscriptions only.  It never creates a publisher, service
client, action client, arm request, mode request, or MAVROS parameter request.
Its ``GO`` result means the selected technical evidence checks passed; flight
authority remains with the safety pilot and PX4's independently configured
failsafes.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Path as NavPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import BatteryState, CameraInfo, Image
from swarm_interfaces.msg import FlightSafetyStatus, TargetTrackArray

from .preflight_checks import (
    CheckResult,
    finite_number,
    fresh_topic_check,
    utc_now,
    validate_calibration_manifest,
    validate_operator_checklist,
)


_STAGES = ("bench", "perception", "decision", "flight")


def _header_stamp_seconds(message: object) -> float | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    seconds = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9
    return seconds if seconds > 0.0 else None


def _header_frame(message: object) -> str:
    header = getattr(message, "header", None)
    return str(getattr(header, "frame_id", "")).strip()


def _finite(values: list[object]) -> bool:
    return all(finite_number(value) is not None for value in values)


@dataclass
class TopicEvidence:
    """Timing and header evidence collected from one subscription."""

    count: int = 0
    first_received_monotonic: float | None = None
    last_received_monotonic: float | None = None
    last_source_stamp_seconds: float | None = None
    last_frame_id: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def observe(self, message: object, *, received_monotonic: float) -> None:
        self.count += 1
        if self.first_received_monotonic is None:
            self.first_received_monotonic = received_monotonic
        self.last_received_monotonic = received_monotonic
        self.last_source_stamp_seconds = _header_stamp_seconds(message)
        self.last_frame_id = _header_frame(message)

    def receive_age(self, now_monotonic: float) -> float | None:
        if self.last_received_monotonic is None:
            return None
        return max(0.0, now_monotonic - self.last_received_monotonic)

    def source_age(self, now_ros_seconds: float) -> float | None:
        if self.last_source_stamp_seconds is None or now_ros_seconds <= 0.0:
            return None
        return now_ros_seconds - self.last_source_stamp_seconds

    def rate_hz(self) -> float | None:
        if (
            self.count < 2
            or self.first_received_monotonic is None
            or self.last_received_monotonic is None
        ):
            return None
        elapsed = self.last_received_monotonic - self.first_received_monotonic
        return (self.count - 1) / elapsed if elapsed > 0.0 else None

    def observation(self) -> dict[str, Any]:
        return {
            "frame_id": self.last_frame_id,
            "last_source_stamp_seconds": self.last_source_stamp_seconds,
            **self.metadata,
        }


class RealUavPreflight(Node):
    """Collect bounded evidence for one selected readiness stage."""

    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__("real_uav_preflight")
        self.options = options
        self._evidence = {
            name: TopicEvidence()
            for name in (
                "mavros_state",
                "camera_image",
                "camera_info",
                "local_pose",
                "battery",
                "world_track",
                "planned_path",
                "safety_status",
            )
        }
        self._mavros_state: dict[str, Any] = {}
        self._battery: dict[str, Any] = {}
        self._image_shape: tuple[int, int] | None = None
        self._camera_info: dict[str, Any] = {}
        self._pose: dict[str, Any] = {}
        self._world_track: dict[str, Any] = {}
        self._planned_path: dict[str, Any] = {}
        self._safety_status: dict[str, Any] = {}

        self.create_subscription(State, options.mavros_state_topic, self._on_mavros_state, 10)
        self.create_subscription(Image, options.image_topic, self._on_image, 10)
        self.create_subscription(CameraInfo, options.camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(PoseStamped, options.local_pose_topic, self._on_local_pose, 10)
        self.create_subscription(BatteryState, options.battery_topic, self._on_battery, 10)
        if options.stage in {"perception", "decision", "flight"}:
            self.create_subscription(
                TargetTrackArray, options.world_track_topic, self._on_world_track, 10
            )
        if options.stage in {"decision", "flight"}:
            self.create_subscription(
                NavPath, options.planned_path_topic, self._on_planned_path, 10
            )
        if options.stage == "flight":
            self.create_subscription(
                FlightSafetyStatus, options.safety_status_topic, self._on_safety_status, 10
            )
        self.get_logger().info(
            f"read-only preflight started: stage={options.stage}, duration={options.duration:.1f}s"
        )

    def _record(self, key: str, message: object) -> TopicEvidence:
        evidence = self._evidence[key]
        evidence.observe(message, received_monotonic=time.monotonic())
        return evidence

    def _on_mavros_state(self, message: State) -> None:
        self._record("mavros_state", message)
        self._mavros_state = {
            "connected": bool(message.connected),
            "armed": bool(message.armed),
            "mode": str(message.mode),
            "system_status": int(message.system_status),
        }

    def _on_image(self, message: Image) -> None:
        evidence = self._record("camera_image", message)
        self._image_shape = (int(message.width), int(message.height))
        evidence.metadata = {
            "width": int(message.width),
            "height": int(message.height),
            "encoding": str(message.encoding),
        }

    def _on_camera_info(self, message: CameraInfo) -> None:
        evidence = self._record("camera_info", message)
        matrix = [float(value) for value in message.k]
        self._camera_info = {
            "width": int(message.width),
            "height": int(message.height),
            "k": matrix,
            "distortion_model": str(message.distortion_model),
        }
        evidence.metadata = {
            "width": int(message.width),
            "height": int(message.height),
            "fx": matrix[0] if matrix else None,
            "fy": matrix[4] if len(matrix) > 4 else None,
            "distortion_model": str(message.distortion_model),
        }

    def _on_local_pose(self, message: PoseStamped) -> None:
        evidence = self._record("local_pose", message)
        position = message.pose.position
        orientation = message.pose.orientation
        values = [
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]
        self._pose = {
            "position": [float(position.x), float(position.y), float(position.z)],
            "orientation": [
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ],
            "finite": _finite(values),
        }
        evidence.metadata = self._pose

    def _on_battery(self, message: BatteryState) -> None:
        evidence = self._record("battery", message)
        percentage = finite_number(message.percentage)
        voltage = finite_number(message.voltage)
        self._battery = {"percentage": percentage, "voltage": voltage}
        evidence.metadata = self._battery

    def _on_world_track(self, message: TargetTrackArray) -> None:
        evidence = self._record("world_track", message)
        tracks = list(message.tracks)
        confirmed = [
            track
            for track in tracks
            if bool(track.is_confirmed)
            and finite_number(track.confidence) is not None
            and float(track.confidence) >= self.options.min_track_confidence
            and _finite([track.x, track.y, track.vx, track.vy])
        ]
        self._world_track = {
            "track_count": len(tracks),
            "confirmed_track_ids": [int(track.target_id) for track in confirmed],
        }
        evidence.metadata = self._world_track

    def _on_planned_path(self, message: NavPath) -> None:
        evidence = self._record("planned_path", message)
        expected_frame = self.options.expected_path_frame
        poses = list(message.poses)
        scoped = bool(poses) and all(
            str(pose.header.frame_id).strip() == expected_frame for pose in poses
        )
        finite = all(
            _finite([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            for pose in poses
        )
        self._planned_path = {
            "pose_count": len(poses),
            "expected_frame": expected_frame,
            "scoped": scoped,
            "finite": finite,
            "pose_frames": sorted({str(pose.header.frame_id).strip() for pose in poses}),
        }
        evidence.metadata = self._planned_path

    def _on_safety_status(self, message: FlightSafetyStatus) -> None:
        evidence = self._record("safety_status", message)
        self._safety_status = {
            "state": int(message.state),
            "hold_requested": bool(message.hold_requested),
            "reason": str(message.reason),
            "mavros_connected": bool(message.mavros_connected),
        }
        evidence.metadata = self._safety_status

    def _topic_check(
        self,
        key: str,
        *,
        require_source_stamp: bool,
        min_rate_hz: float | None = None,
    ) -> CheckResult:
        evidence = self._evidence[key]
        now_monotonic = time.monotonic()
        now_ros_seconds = self.get_clock().now().nanoseconds / 1e9
        return fresh_topic_check(
            key,
            count=evidence.count,
            receive_age_seconds=evidence.receive_age(now_monotonic),
            source_age_seconds=evidence.source_age(now_ros_seconds),
            max_age_seconds=self.options.max_age_seconds,
            require_source_stamp=require_source_stamp,
            rate_hz=evidence.rate_hz(),
            min_rate_hz=min_rate_hz,
            observation=evidence.observation(),
        )

    def _mavros_checks(self) -> list[CheckResult]:
        checks = [self._topic_check("mavros_state", require_source_stamp=False)]
        state = self._mavros_state
        connected = state.get("connected") is True
        checks.append(
            CheckResult(
                "mavros_connected",
                connected,
                "connected" if connected else "not_connected",
                state,
            )
        )
        disarmed = state.get("armed") is False
        checks.append(
            CheckResult(
                "vehicle_disarmed",
                disarmed,
                "disarmed" if disarmed else "vehicle_armed_or_unknown",
                state,
            )
        )
        mode = str(state.get("mode", "")).strip().upper()
        not_offboard = mode != "OFFBOARD"
        checks.append(
            CheckResult(
                "offboard_inactive",
                not_offboard,
                "offboard_inactive" if not_offboard else "offboard_active",
                state,
            )
        )
        return checks

    def _camera_checks(self) -> list[CheckResult]:
        checks = [
            self._topic_check(
                "camera_image",
                require_source_stamp=self.options.require_source_stamps,
                min_rate_hz=self.options.min_image_hz,
            ),
            self._topic_check(
                "camera_info", require_source_stamp=self.options.require_source_stamps
            ),
        ]
        info = self._camera_info
        matrix = list(info.get("k", []))
        valid_intrinsics = (
            int(info.get("width", 0)) > 0
            and int(info.get("height", 0)) > 0
            and len(matrix) == 9
            and _finite(matrix)
            and matrix[0] > 0.0
            and matrix[4] > 0.0
        )
        checks.append(
            CheckResult(
                "camera_info_valid",
                valid_intrinsics,
                "valid" if valid_intrinsics else "camera_info_invalid",
                info,
            )
        )
        same_dimensions = self._image_shape is not None and self._image_shape == (
            int(info.get("width", 0)),
            int(info.get("height", 0)),
        )
        checks.append(
            CheckResult(
                "image_camera_info_match",
                same_dimensions,
                "dimensions_match" if same_dimensions else "image_camera_info_mismatch",
                {
                    "image_shape": self._image_shape,
                    "camera_info_shape": [info.get("width"), info.get("height")],
                },
            )
        )
        return checks

    def _pose_checks(self) -> list[CheckResult]:
        checks = [
            self._topic_check(
                "local_pose", require_source_stamp=self.options.require_source_stamps
            )
        ]
        pose = self._pose
        orientation = list(pose.get("orientation", []))
        norm = (
            math.sqrt(sum(value * value for value in orientation))
            if len(orientation) == 4 and _finite(orientation)
            else None
        )
        valid = pose.get("finite") is True and norm is not None and 0.95 <= norm <= 1.05
        checks.append(
            CheckResult(
                "local_pose_valid",
                valid,
                "valid" if valid else "position_or_quaternion_invalid",
                {**pose, "quaternion_norm": norm},
            )
        )
        expected_frame = self.options.expected_local_frame
        observed_frame = self._evidence["local_pose"].last_frame_id
        frame_valid = bool(observed_frame) and (
            not expected_frame or observed_frame == expected_frame
        )
        checks.append(
            CheckResult(
                "local_pose_frame",
                frame_valid,
                "frame_matches" if frame_valid else "local_pose_frame_unexpected",
                {"expected": expected_frame, "observed": observed_frame},
            )
        )
        return checks

    def _battery_checks(self) -> list[CheckResult]:
        checks = [
            self._topic_check("battery", require_source_stamp=self.options.require_source_stamps)
        ]
        percentage = self._battery.get("percentage")
        valid = percentage is not None and self.options.min_battery_percent <= percentage <= 1.0
        checks.append(
            CheckResult(
                "battery_threshold",
                valid,
                "threshold_met" if valid else "battery_percentage_missing_or_below_threshold",
                {**self._battery, "minimum": self.options.min_battery_percent},
            )
        )
        return checks

    def _perception_checks(self) -> list[CheckResult]:
        checks = [
            self._topic_check(
                "world_track", require_source_stamp=self.options.require_source_stamps
            )
        ]
        observed_frame = self._evidence["world_track"].last_frame_id
        frame_valid = observed_frame == self.options.expected_world_frame
        checks.append(
            CheckResult(
                "world_track_frame",
                frame_valid,
                "frame_matches" if frame_valid else "world_track_frame_unexpected",
                {"expected": self.options.expected_world_frame, "observed": observed_frame},
            )
        )
        has_confirmed_track = bool(self._world_track.get("confirmed_track_ids"))
        checks.append(
            CheckResult(
                "confirmed_world_track",
                has_confirmed_track,
                "confirmed_track_present" if has_confirmed_track else "no_confirmed_world_track",
                self._world_track,
            )
        )
        return checks

    def _decision_checks(self) -> list[CheckResult]:
        checks = [
            self._topic_check(
                "planned_path", require_source_stamp=self.options.require_source_stamps
            )
        ]
        valid = (
            self._planned_path.get("scoped") is True
            and self._planned_path.get("finite") is True
        )
        checks.append(
            CheckResult(
                "planned_path_scope",
                valid,
                "scoped_and_finite" if valid else "path_unscoped_or_invalid",
                self._planned_path,
            )
        )
        return checks

    def _flight_gate_checks(self) -> list[CheckResult]:
        calibration = validate_calibration_manifest(
            self.options.calibration_manifest,
            max_reprojection_error_px=self.options.max_reprojection_error_px,
        )
        checklist = validate_operator_checklist(self.options.operator_checklist)
        safety_topic = self._topic_check(
            "safety_status", require_source_stamp=self.options.require_source_stamps
        )
        locked = (
            self._safety_status.get("state") == 0
            and self._safety_status.get("hold_requested") is True
        )
        safety_locked = CheckResult(
            "safety_gate_locked",
            locked,
            "locked_with_hold" if locked else "safety_gate_not_locked",
            self._safety_status,
        )
        return [calibration, checklist, safety_topic, safety_locked]

    def evaluate(self) -> dict[str, Any]:
        """Return the complete evidence report without affecting the vehicle."""

        checks = (
            self._mavros_checks()
            + self._camera_checks()
            + self._pose_checks()
            + self._battery_checks()
        )
        if self.options.stage in {"perception", "decision", "flight"}:
            checks += self._perception_checks()
        if self.options.stage in {"decision", "flight"}:
            checks += self._decision_checks()
        if self.options.stage == "flight":
            checks += self._flight_gate_checks()
        failed = [check.name for check in checks if not check.passed]
        return {
            "schema_version": 1,
            "generated_at": utc_now(),
            "stage": self.options.stage,
            "read_only": True,
            "technical_gate": "GO" if not failed else "NO_GO",
            "flight_authority": "operator_and_px4_only",
            "failed_checks": failed,
            "checks": [check.as_dict() for check in checks],
        }


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def _percentage(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or not 0.0 <= parsed <= 1.0:
        raise argparse.ArgumentTypeError("must be between 0.0 and 1.0")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    """Parse non-ROS arguments for the read-only monitor."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=_STAGES, default="bench")
    parser.add_argument("--duration", type=_positive_float, default=15.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("~/real_uav_preflight").expanduser(),
    )
    parser.add_argument("--max-age-seconds", type=_positive_float, default=1.0)
    parser.add_argument("--min-image-hz", type=_positive_float, default=10.0)
    parser.add_argument("--min-battery-percent", type=_percentage, default=0.70)
    parser.add_argument("--max-reprojection-error-px", type=_positive_float, default=1.0)
    parser.add_argument("--min-track-confidence", type=_percentage, default=0.50)
    parser.add_argument(
        "--require-source-stamps",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--mavros-state-topic", default="/uav0/mavros/state")
    parser.add_argument("--image-topic", default="/camera/image")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--local-pose-topic", default="/uav0/mavros/local_position/pose")
    parser.add_argument("--battery-topic", default="/uav0/mavros/battery")
    parser.add_argument("--world-track-topic", default="/target_track_world")
    parser.add_argument("--planned-path-topic", default="/planned_path")
    parser.add_argument("--safety-status-topic", default="/flight_safety/status")
    parser.add_argument("--expected-local-frame", default="map")
    parser.add_argument("--expected-world-frame", default="world")
    parser.add_argument("--expected-path-frame", default="drone_0")
    parser.add_argument(
        "--calibration-manifest",
        type=Path,
        default=Path("/etc/swarm-control/real_uav_calibration.yaml"),
    )
    parser.add_argument(
        "--operator-checklist",
        type=Path,
        default=Path("/etc/swarm-control/real_uav_operator_checklist.yaml"),
    )
    return parser.parse_args(argv)


def _report_path(output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return output_dir / f"real_uav_preflight_{timestamp}.json"


def main(args: list[str] | None = None) -> int:
    """Run the bounded monitor and persist its report before exiting."""

    all_args = list(args) if args is not None else sys.argv
    options = parse_args(remove_ros_args(args=all_args)[1:])
    rclpy.init(args=all_args)
    node = RealUavPreflight(options)
    interrupted = False
    try:
        deadline = time.monotonic() + options.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.25, deadline - time.monotonic()))
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
    finally:
        report = node.evaluate()
        report["interrupted"] = interrupted
        if interrupted:
            report["technical_gate"] = "NO_GO"
            report["failed_checks"].append("preflight_interrupted")
        path = _report_path(options.output_dir)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        node.get_logger().info(
            f"technical_gate={report['technical_gate']} report={path} read_only=true"
        )
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if report["technical_gate"] == "GO" else 2


if __name__ == "__main__":
    raise SystemExit(main())
