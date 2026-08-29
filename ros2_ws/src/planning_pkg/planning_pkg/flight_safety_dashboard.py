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
import secrets
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
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
        if urlparse(self.path).path != "/api/control":
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
        status, response = self.server.dashboard.handle_control(
            body,
            token=self.headers.get("X-Flight-Safety-Token", ""),
        )
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
        self._operator_token = self._str_param("operator_token")
        self._bind_address = self._str_param("bind_address")
        self._allow_remote_control = self._bool_param("allow_remote_control")
        self._control_timeout = max(self._float_param("control_timeout"), 0.1)
        self._request_ttl = max(self._float_param("request_ttl"), 1.0)
        self._status_stale_timeout = max(self._float_param("status_stale_timeout"), 0.1)
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
        self._control_client = self.create_client(
            SafetyControl,
            self._str_param("control_service"),
        )

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
            "allow_remote_control": False,
            "max_video_bytes": 4 * 1024 * 1024,
            "control_timeout": 3.0,
            "request_ttl": 10.0,
            "status_stale_timeout": 3.0,
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

    def status_payload(self) -> dict[str, Any]:
        payload = self.state.status_snapshot()
        payload["control_available"] = self.control_available
        payload["control_reason"] = self._control_reason()
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

    def _control_reason(self) -> str:
        if not self._operator_token:
            return "operator_token_not_configured"
        if not is_loopback_host(self._bind_address) and not self._allow_remote_control:
            return "remote_control_not_explicitly_allowed"
        return "control_ready"

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
