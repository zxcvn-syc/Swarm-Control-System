"""Local web ground station for the containment safety supervisor.

It exposes only the existing containment safety-gate commands.  It never
publishes MAVROS setpoints or calls vehicle arm, mode, RTL, or landing APIs.
The HTTP server binds to loopback by default and requires an explicit operator
token before it will proxy any control request.
"""

from __future__ import annotations

import hmac
import json
import mimetypes
import os
import secrets
import threading
import time
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from swarm_interfaces.msg import FlightSafetyStatus
from swarm_interfaces.srv import SafetyControl

from .flight_safety_dashboard_state import (
    ACTIVATION_MODE_NAMES,
    SAFETY_STATE_NAMES,
    DashboardState,
    is_loopback_host,
)
from .pilot_control_policy import (
    MavrosSnapshot,
    PilotDecision,
    SafetySnapshot,
    decide_pilot_action,
)


CONTROL_COMMANDS = {
    "enable_manual": SafetyControl.Request.COMMAND_ENABLE_MANUAL,
    "enable_auto": SafetyControl.Request.COMMAND_ENABLE_AUTO,
    "disable": SafetyControl.Request.COMMAND_DISABLE,
    "emergency_hold": SafetyControl.Request.COMMAND_EMERGENCY_HOLD,
    "reset_fault": SafetyControl.Request.COMMAND_RESET_FAULT,
}
STATIC_FILES = {
    "/": "flight_safety_dashboard.html",
    "/dashboard.css": "flight_safety_dashboard.css",
    "/dashboard.js": "flight_safety_dashboard.js",
    "/flight_safety_dashboard.css": "flight_safety_dashboard.css",
    "/flight_safety_dashboard.js": "flight_safety_dashboard.js",
}
MAX_CONTROL_BODY_BYTES = 4096


def _stamp_seconds(stamp: Any) -> float:
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9


def _set_stamp(stamp: Any, seconds: float) -> None:
    value = max(float(seconds), 0.0)
    sec = int(value)
    nanosec = int(round((value - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp.sec = sec
    stamp.nanosec = nanosec


class _DashboardHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class _DashboardRequestHandler(BaseHTTPRequestHandler):
    server: _DashboardHTTPServer

    def log_message(self, format: str, *args: Any) -> None:
        self.server.dashboard.get_logger().debug("dashboard http: " + format % args)

    def end_headers(self) -> None:
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; img-src 'self' data:; style-src 'self'; "
            "script-src 'self'; connect-src 'self'; base-uri 'none'; "
            "form-action 'self'; frame-ancestors 'none'",
        )
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/status":
            self._write_json(HTTPStatus.OK, self.server.dashboard.status_payload())
            return
        if path == "/stream.mjpg":
            self._stream_mjpeg()
            return
        filename = STATIC_FILES.get(path)
        if filename is not None:
            self._serve_static(filename)
            return
        self._write_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path not in {"/api/control", "/api/pilot-control"}:
            self._write_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
            return
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": "invalid_content_length"})
            return
        if content_length <= 0 or content_length > MAX_CONTROL_BODY_BYTES:
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": "invalid_request_size"})
            return
        try:
            body = json.loads(self.rfile.read(content_length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": "invalid_json"})
            return
        if not isinstance(body, dict):
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": "invalid_request"})
            return
        token = self.headers.get("X-Flight-Safety-Token", "")
        if path == "/api/control":
            status, response = self.server.dashboard.handle_control(body, token=token)
        else:
            status, response = self.server.dashboard.handle_pilot_control(body, token=token)
        self._write_json(status, response)

    def do_OPTIONS(self) -> None:
        self.send_error(HTTPStatus.METHOD_NOT_ALLOWED)

    def _serve_static(self, filename: str) -> None:
        path = self.server.asset_directory / filename
        if not path.is_file():
            self._write_json(HTTPStatus.NOT_FOUND, {"error": "static_asset_missing"})
            return
        payload = path.read_bytes()
        content_type, _ = mimetypes.guess_type(path.name)
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _stream_mjpeg(self) -> None:
        boundary = b"flight-safety-frame"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary.decode()}")
        self.send_header("Connection", "close")
        self.end_headers()
        sequence = 0
        try:
            while self.server.dashboard.http_running:
                sequence, frame = self.server.dashboard.state.wait_for_frame(sequence, 1.0)
                if frame is None:
                    continue
                self.wfile.write(b"--" + boundary + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def _write_json(self, status: int | HTTPStatus, body: dict[str, Any]) -> None:
        payload = json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


class FlightSafetyDashboard(Node):
    """Bridge the safety status, compressed video, and control service to HTTP."""

    def __init__(self) -> None:
        super().__init__("flight_safety_dashboard")
        self._declare_parameters()
        self.state = DashboardState(self._int_param("max_video_bytes"))
        self._operator_token = self._str_param("operator_token") or os.environ.get(
            self._str_param("operator_token_env"), ""
        ).strip()
        self._bind_address = self._str_param("bind_address")
        self._allow_remote_control = self._bool_param("allow_remote_control")
        self._control_timeout = max(self._float_param("control_timeout"), 0.1)
        self._request_ttl = max(self._float_param("request_ttl"), 1.0)
        self._status_stale_timeout = max(self._float_param("status_stale_timeout"), 0.1)
        self._enable_pilot_commands = self._bool_param("enable_pilot_commands")
        self._pilot_state_stale_timeout = max(
            self._float_param("pilot_state_stale_timeout"), 0.1
        )
        self._pilot_command_timeout = max(
            self._float_param("pilot_command_timeout"), 0.1
        )
        self._pilot_audit_log = Path(self._str_param("pilot_audit_log")).expanduser()
        self._mavros_lock = threading.RLock()
        self._mavros_state: dict[str, Any] = {"available": False}
        self._pilot_command_lock = threading.Lock()
        self._http_running = True

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            FlightSafetyStatus,
            self._str_param("status_topic"),
            self.on_status,
            status_qos,
        )
        self.create_subscription(
            CompressedImage,
            self._str_param("video_topic"),
            self.on_video,
            image_qos,
        )
        self.create_subscription(
            State,
            self._str_param("mavros_state_topic"),
            self.on_mavros_state,
            10,
        )
        self._control_client = self.create_client(
            SafetyControl,
            self._str_param("control_service"),
        )
        self._arm_client = None
        self._mode_client = None
        if self._enable_pilot_commands:
            self._arm_client = self.create_client(
                CommandBool, self._str_param("arm_service")
            )
            self._mode_client = self.create_client(SetMode, self._str_param("mode_service"))

        asset_directory = self._asset_directory()
        self._http_server = _DashboardHTTPServer(
            (self._bind_address, self._int_param("port")),
            _DashboardRequestHandler,
        )
        self._http_server.dashboard = self
        self._http_server.asset_directory = asset_directory
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="flight-safety-dashboard-http",
            daemon=True,
        )
        self._http_thread.start()
        visibility = "本机" if is_loopback_host(self._bind_address) else "远程"
        self.get_logger().info(
            "flight safety dashboard started at "
            f"http://{self._bind_address}:{self._int_param('port')} ({visibility}访问)"
        )
        if not self.control_available:
            self.get_logger().warning(
                "dashboard control is read-only; configure operator_token and keep a loopback bind "
                "or explicitly allow remote control"
            )
        elif not self.pilot_control_available:
            self.get_logger().info(
                "pilot commands are disabled; the dashboard controls only the containment gate"
            )

    @property
    def http_running(self) -> bool:
        return self._http_running

    @property
    def control_available(self) -> bool:
        return bool(self._operator_token) and (
            is_loopback_host(self._bind_address) or self._allow_remote_control
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "bind_address": "127.0.0.1",
            "port": 8080,
            "status_topic": "/flight_safety/status",
            "control_service": "/flight_safety/control",
            "video_topic": "/camera/image/compressed",
            "operator_token": "",
            "operator_token_env": "FLIGHT_SAFETY_TOKEN",
            "allow_remote_control": False,
            "max_video_bytes": 4 * 1024 * 1024,
            "control_timeout": 3.0,
            "request_ttl": 10.0,
            "status_stale_timeout": 3.0,
            "enable_pilot_commands": False,
            "mavros_state_topic": "/uav0/mavros/state",
            "arm_service": "/uav0/mavros/cmd/arming",
            "mode_service": "/uav0/mavros/set_mode",
            "pilot_state_stale_timeout": 1.0,
            "pilot_safety_stale_timeout": 1.0,
            "pilot_command_timeout": 3.0,
            "pilot_audit_log": "",
            "position_mode": "POSCTL",
            "altitude_mode": "ALTCTL",
            "offboard_mode": "OFFBOARD",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _asset_directory(self) -> Path:
        try:
            installed = Path(get_package_share_directory("planning_pkg")) / "web"
            if installed.is_dir():
                return installed
        except (LookupError, ValueError):
            pass
        return Path(__file__).resolve().parents[1] / "web"

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value).strip()

    def _int_param(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _bool_param(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def on_status(self, message: FlightSafetyStatus) -> None:
        state_value = int(message.state)
        mode_value = int(message.activation_mode)
        self.state.update_status(
            {
                "state": state_value,
                "state_name": SAFETY_STATE_NAMES.get(state_value, "UNKNOWN"),
                "activation_mode": mode_value,
                "activation_mode_name": ACTIVATION_MODE_NAMES.get(mode_value, "UNKNOWN"),
                "containment_enabled": bool(message.containment_enabled),
                "hold_requested": bool(message.hold_requested),
                "target_locked": bool(message.target_locked),
                "locked_target_id": int(message.locked_target_id),
                "drone_states_fresh": bool(message.drone_states_fresh),
                "command_fresh": bool(message.command_fresh),
                "mavros_fresh": bool(message.mavros_fresh),
                "mavros_connected": bool(message.mavros_connected),
                "session_id": int(message.session_id),
                "last_control_request_id": int(message.last_control_request_id),
                "last_command_sequence": int(message.last_command_sequence),
                "fault_mask": int(message.fault_mask),
                "reason": str(message.reason),
                "source_stamp": _stamp_seconds(message.header.stamp),
            }
        )

    def on_video(self, message: CompressedImage) -> None:
        if not self.state.update_frame(bytes(message.data)):
            self.get_logger().warning("discarded invalid or oversized dashboard JPEG frame")

    def on_mavros_state(self, message: State) -> None:
        with self._mavros_lock:
            self._mavros_state = {
                "available": True,
                "connected": bool(message.connected),
                "armed": bool(message.armed),
                "mode": str(message.mode),
                "system_status": int(message.system_status),
                "updated_at": time.monotonic(),
            }

    def status_payload(self) -> dict[str, Any]:
        payload = self.state.status_snapshot()
        payload["control_available"] = self.control_available
        payload["control_reason"] = self._control_reason()
        payload["pilot_control_available"] = self.pilot_control_available
        payload["pilot_control_reason"] = self._pilot_control_reason()
        payload["mavros"] = self._mavros_snapshot_payload()
        payload["pilot_actions"] = {
            "arm": "ARM",
            "disarm": "DISARM",
            "position": "POSCTL",
            "altitude": "ALTCTL",
            "offboard": "OFFBOARD",
        }
        payload["video_topic"] = self._str_param("video_topic")
        return payload

    def handle_control(self, body: dict[str, Any], *, token: str) -> tuple[int, dict[str, Any]]:
        if not self.control_available:
            return HTTPStatus.FORBIDDEN, {"accepted": False, "reason": self._control_reason()}
        if not hmac.compare_digest(token, self._operator_token):
            return HTTPStatus.UNAUTHORIZED, {"accepted": False, "reason": "operator_token_invalid"}

        command_name = str(body.get("command", "")).strip()
        command = CONTROL_COMMANDS.get(command_name)
        if command is None:
            return HTTPStatus.BAD_REQUEST, {"accepted": False, "reason": "unsupported_command"}
        raw_operator_id = body.get("operator_id")
        if not isinstance(raw_operator_id, str):
            return HTTPStatus.BAD_REQUEST, {"accepted": False, "reason": "operator_id_required"}
        operator_id = raw_operator_id.strip()
        if not operator_id or len(operator_id) > 64:
            return HTTPStatus.BAD_REQUEST, {"accepted": False, "reason": "operator_id_required"}
        ground_confirmed = body.get("ground_confirmed", False)
        if not isinstance(ground_confirmed, bool):
            return HTTPStatus.BAD_REQUEST, {"accepted": False, "reason": "invalid_ground_confirmation"}
        if command_name == "reset_fault" and not ground_confirmed:
            return HTTPStatus.BAD_REQUEST, {"accepted": False, "reason": "ground_confirmation_required"}

        status = self.state.status_snapshot()
        if not status.get("available") or not int(status.get("session_id", 0)):
            return HTTPStatus.SERVICE_UNAVAILABLE, {"accepted": False, "reason": "safety_status_unavailable"}
        status_age = status.get("status_age_seconds")
        if status_age is None or float(status_age) > self._status_stale_timeout:
            return HTTPStatus.SERVICE_UNAVAILABLE, {"accepted": False, "reason": "safety_status_stale"}
        if not self._control_client.service_is_ready():
            return HTTPStatus.SERVICE_UNAVAILABLE, {"accepted": False, "reason": "control_service_unavailable"}

        request = SafetyControl.Request()
        request.command = int(command)
        request.session_id = int(status["session_id"])
        last_request_id = int(status.get("last_control_request_id", 0))
        if last_request_id >= (1 << 64) - 1:
            return HTTPStatus.CONFLICT, {"accepted": False, "reason": "request_id_space_exhausted"}
        request.request_id = max(last_request_id + 1, time.time_ns())
        base_time = float(status.get("source_stamp", 0.0))
        if base_time <= 0.0:
            base_time = self.get_clock().now().nanoseconds / 1e9
        _set_stamp(request.expires_at, base_time + self._request_ttl)
        request.operator_id = operator_id
        request.ground_confirmed = ground_confirmed

        completion = threading.Event()
        result: dict[str, Any] = {}
        future = self._control_client.call_async(request)

        def _done(done_future: Any) -> None:
            try:
                response = done_future.result()
                result["response"] = response
            except Exception as exc:  # The result must be rendered as a safe API error.
                result["error"] = str(exc)
            finally:
                completion.set()

        future.add_done_callback(_done)
        if not completion.wait(self._control_timeout):
            return HTTPStatus.GATEWAY_TIMEOUT, {"accepted": False, "reason": "control_service_timeout"}
        if "error" in result:
            return HTTPStatus.BAD_GATEWAY, {"accepted": False, "reason": "control_service_error"}
        response = result["response"]
        payload = {
            "accepted": bool(response.accepted),
            "state": int(response.state),
            "state_name": SAFETY_STATE_NAMES.get(int(response.state), "UNKNOWN"),
            "activation_mode": int(response.activation_mode),
            "activation_mode_name": ACTIVATION_MODE_NAMES.get(
                int(response.activation_mode), "UNKNOWN"
            ),
            "accepted_request_id": int(response.accepted_request_id),
            "reason": str(response.reason),
        }
        return (HTTPStatus.OK if response.accepted else HTTPStatus.CONFLICT), payload

    def _mavros_snapshot_payload(self) -> dict[str, Any]:
        with self._mavros_lock:
            payload = dict(self._mavros_state)
        updated_at = payload.pop("updated_at", None)
        payload["age_seconds"] = (
            None if updated_at is None else max(0.0, time.monotonic() - float(updated_at))
        )
        return payload

    def _mavros_snapshot(self) -> MavrosSnapshot:
        payload = self._mavros_snapshot_payload()
        return MavrosSnapshot(
            available=bool(payload.get("available")),
            connected=bool(payload.get("connected")),
            armed=bool(payload.get("armed")),
            mode=str(payload.get("mode", "")),
            age_seconds=payload.get("age_seconds"),
        )

    def _safety_snapshot(self) -> SafetySnapshot:
        payload = self.state.status_snapshot()
        return SafetySnapshot(
            available=bool(payload.get("available")),
            state_name=str(payload.get("state_name", "")),
            containment_enabled=bool(payload.get("containment_enabled")),
            hold_requested=bool(payload.get("hold_requested")),
            target_locked=bool(payload.get("target_locked")),
            age_seconds=payload.get("status_age_seconds"),
        )

    def handle_pilot_control(
        self, body: dict[str, Any], *, token: str
    ) -> tuple[int, dict[str, Any]]:
        """Issue one audited MAVROS request after policy and token checks."""

        if not self.pilot_control_available:
            return HTTPStatus.FORBIDDEN, {
                "accepted": False,
                "reason": self._pilot_control_reason(),
            }
        if not hmac.compare_digest(token, self._operator_token):
            return HTTPStatus.UNAUTHORIZED, {
                "accepted": False,
                "reason": "operator_token_invalid",
            }
        action = body.get("action")
        confirmation = body.get("confirmation")
        operator_id = body.get("operator_id")
        ground_confirmed = body.get("ground_confirmed", False)
        if not isinstance(action, str) or len(action.strip()) > 32:
            return HTTPStatus.BAD_REQUEST, {
                "accepted": False,
                "reason": "pilot_action_required",
            }
        if not isinstance(confirmation, str) or len(confirmation) > 32:
            return HTTPStatus.BAD_REQUEST, {
                "accepted": False,
                "reason": "pilot_confirmation_required",
            }
        if not isinstance(operator_id, str) or not operator_id.strip() or len(operator_id) > 64:
            return HTTPStatus.BAD_REQUEST, {
                "accepted": False,
                "reason": "operator_id_required",
            }
        if not isinstance(ground_confirmed, bool):
            return HTTPStatus.BAD_REQUEST, {
                "accepted": False,
                "reason": "invalid_ground_confirmation",
            }
        if not self._pilot_command_lock.acquire(blocking=False):
            return HTTPStatus.CONFLICT, {
                "accepted": False,
                "reason": "pilot_command_in_progress",
            }
        try:
            mavros = self._mavros_snapshot()
            safety = self._safety_snapshot()
            decision = decide_pilot_action(
                action.strip(),
                confirmation=confirmation,
                ground_confirmed=ground_confirmed,
                mavros=mavros,
                safety=safety,
                mavros_max_age_seconds=self._pilot_state_stale_timeout,
                safety_max_age_seconds=max(
                    self._float_param("pilot_safety_stale_timeout"), 0.1
                ),
                position_mode=self._str_param("position_mode"),
                altitude_mode=self._str_param("altitude_mode"),
                offboard_mode=self._str_param("offboard_mode"),
            )
            request_record = self._pilot_audit_record(
                event="request",
                operator_id=operator_id.strip(),
                action=action.strip(),
                ground_confirmed=ground_confirmed,
                decision=decision,
                mavros=mavros,
                safety=safety,
            )
            if not self._write_pilot_audit(request_record):
                return HTTPStatus.SERVICE_UNAVAILABLE, {
                    "accepted": False,
                    "reason": "pilot_audit_log_unavailable",
                }
            if not decision.allowed:
                return HTTPStatus.CONFLICT, self._pilot_payload(False, decision.reason, decision)
            status, payload = self._send_pilot_request(decision)
            result_record = self._pilot_audit_record(
                event="result",
                operator_id=operator_id.strip(),
                action=action.strip(),
                ground_confirmed=ground_confirmed,
                decision=decision,
                mavros=mavros,
                safety=safety,
                result=payload,
            )
            self._write_pilot_audit(result_record)
            return status, payload
        finally:
            self._pilot_command_lock.release()

    def _send_pilot_request(
        self, decision: PilotDecision
    ) -> tuple[int, dict[str, Any]]:
        if decision.service == "arm":
            if self._arm_client is None or not self._arm_client.service_is_ready():
                return HTTPStatus.SERVICE_UNAVAILABLE, self._pilot_payload(
                    False, "pilot_arm_service_unavailable", decision
                )
            request = CommandBool.Request()
            request.value = bool(decision.arm_value)
            future = self._arm_client.call_async(request)
            service_name = "arm"
        elif decision.service == "mode":
            if self._mode_client is None or not self._mode_client.service_is_ready():
                return HTTPStatus.SERVICE_UNAVAILABLE, self._pilot_payload(
                    False, "pilot_mode_service_unavailable", decision
                )
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = str(decision.custom_mode)
            future = self._mode_client.call_async(request)
            service_name = "mode"
        else:
            return HTTPStatus.BAD_REQUEST, self._pilot_payload(
                False, "pilot_action_unsupported", decision
            )

        completion = threading.Event()
        result: dict[str, Any] = {}

        def _done(done_future: Any) -> None:
            try:
                result["response"] = done_future.result()
            except Exception as error:
                result["error"] = str(error)
            finally:
                completion.set()

        future.add_done_callback(_done)
        if not completion.wait(self._pilot_command_timeout):
            return HTTPStatus.GATEWAY_TIMEOUT, self._pilot_payload(
                False, "pilot_command_timeout", decision
            )
        if "error" in result:
            return HTTPStatus.BAD_GATEWAY, self._pilot_payload(
                False, "pilot_command_service_error", decision
            )
        response = result["response"]
        accepted = bool(response.success) if service_name == "arm" else bool(response.mode_sent)
        reason = "pilot_command_sent" if accepted else "pilot_command_rejected"
        payload = self._pilot_payload(accepted, reason, decision)
        payload["service"] = service_name
        if service_name == "arm":
            payload["mavros_result"] = int(response.result)
        return (HTTPStatus.OK if accepted else HTTPStatus.CONFLICT), payload

    @staticmethod
    def _pilot_payload(
        accepted: bool, reason: str, decision: PilotDecision
    ) -> dict[str, Any]:
        return {
            "accepted": bool(accepted),
            "reason": reason,
            "action": decision.action,
            "requested_mode": decision.custom_mode,
            "requested_arm": decision.arm_value,
        }

    def _pilot_audit_record(
        self,
        *,
        event: str,
        operator_id: str,
        action: str,
        ground_confirmed: bool,
        decision: PilotDecision,
        mavros: MavrosSnapshot,
        safety: SafetySnapshot,
        result: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        record: dict[str, Any] = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "event": event,
            "operator_id": operator_id,
            "action": action,
            "ground_confirmed": ground_confirmed,
            "decision": decision.audit_fields(),
            "mavros": {
                "available": mavros.available,
                "connected": mavros.connected,
                "armed": mavros.armed,
                "mode": mavros.mode,
                "age_seconds": mavros.age_seconds,
            },
            "safety": {
                "available": safety.available,
                "state_name": safety.state_name,
                "containment_enabled": safety.containment_enabled,
                "hold_requested": safety.hold_requested,
                "target_locked": safety.target_locked,
                "age_seconds": safety.age_seconds,
            },
        }
        if result is not None:
            record["result"] = result
        return record

    def _write_pilot_audit(self, record: dict[str, Any]) -> bool:
        try:
            self._pilot_audit_log.parent.mkdir(parents=True, exist_ok=True)
            with self._pilot_audit_log.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
                handle.flush()
                os.fsync(handle.fileno())
            return True
        except OSError as error:
            self.get_logger().error(f"pilot audit log write failed: {error}")
            return False

    def _control_reason(self) -> str:
        if not self._operator_token:
            return "operator_token_not_configured"
        if not is_loopback_host(self._bind_address) and not self._allow_remote_control:
            return "remote_control_not_explicitly_allowed"
        return "control_ready"

    @property
    def pilot_control_available(self) -> bool:
        return self._enable_pilot_commands and self.control_available and bool(
            self._str_param("pilot_audit_log")
        )

    def _pilot_control_reason(self) -> str:
        if not self._enable_pilot_commands:
            return "pilot_commands_disabled"
        if not self.control_available:
            return self._control_reason()
        if not self._str_param("pilot_audit_log"):
            return "pilot_audit_log_not_configured"
        return "pilot_control_ready"

    def destroy_node(self) -> bool:
        self._http_running = False
        self.state.notify_waiters()
        self._http_server.shutdown()
        self._http_server.server_close()
        self._http_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FlightSafetyDashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
