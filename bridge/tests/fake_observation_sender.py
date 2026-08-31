#!/usr/bin/env python3
"""Serve deterministic observations for a local ROS 2 receiver smoke test."""

import argparse
import hmac
import socket
import sys
import time
from pathlib import Path


BRIDGE_ROOT = Path(__file__).resolve().parents[1]
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

import protocol


def stamped_header(frame_id):
    now = time.time()
    seconds = int(now)
    return {
        "stamp": {"sec": seconds, "nanosec": int((now - seconds) * 1_000_000_000)},
        "frame_id": frame_id,
    }


def records():
    return (
        {
            "kind": "mavros_state",
            "header": stamped_header("map"),
            "connected": True,
            "armed": False,
            "guided": False,
            "manual_input": True,
            "mode": "POSCTL",
            "system_status": 3,
        },
        {
            "kind": "pose",
            "header": stamped_header("map"),
            "position": {"x": 1.25, "y": -0.5, "z": 3.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        {
            "kind": "battery",
            "header": stamped_header("base_link"),
            "voltage": 15.2,
            "temperature": 22.0,
            "current": 1.1,
            "charge": 3.8,
            "capacity": 5.0,
            "design_capacity": 5.0,
            "percentage": 0.76,
            "power_supply_status": 2,
            "power_supply_health": 1,
            "power_supply_technology": 3,
            "present": True,
            "cell_voltage": [3.8, 3.8, 3.8, 3.8],
            "cell_temperature": [],
            "location": "synthetic",
            "serial_number": "bridge-smoke-test",
        },
        {
            "kind": "camera_info",
            "header": stamped_header("camera_color_optical_frame"),
            "height": 480,
            "width": 640,
            "distortion_model": "plumb_bob",
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
            "k": [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            "binning_x": 0,
            "binning_y": 0,
        },
        {
            "kind": "image_raw",
            "header": stamped_header("camera_color_optical_frame"),
            "height": 1,
            "width": 1,
            "encoding": "bgr8",
            "is_bigendian": 0,
            "step": 3,
        },
    )


def run(host, port, token, duration):
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((host, port))
    listener.listen(1)
    listener.settimeout(10.0)
    try:
        connection, _ = listener.accept()
        with connection:
            hello, payload = protocol.recv_record(connection)
            if payload or hello.get("kind") != "hello":
                raise protocol.ProtocolError("receiver did not send hello")
            if not hmac.compare_digest(str(hello.get("token", "")), token):
                raise protocol.ProtocolError("receiver token did not match")
            protocol.send_record(
                connection,
                {"kind": "hello_ack", "protocol": protocol.PROTOCOL_VERSION},
            )
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                for record in records():
                    payload = b"\x10\x20\x30" if record["kind"] == "image_raw" else b""
                    try:
                        protocol.send_record(connection, record, payload)
                    except OSError:
                        return
                time.sleep(0.25)
    finally:
        listener.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=19002)
    parser.add_argument("--token", required=True)
    parser.add_argument("--duration", type=float, default=12.0)
    arguments = parser.parse_args()
    run(arguments.host, arguments.port, arguments.token, arguments.duration)


if __name__ == "__main__":
    main()
