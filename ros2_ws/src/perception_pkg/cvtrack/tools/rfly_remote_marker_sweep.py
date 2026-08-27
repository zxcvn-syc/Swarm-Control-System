#!/usr/bin/env python3
"""Send deterministic blue-car calibration markers through the VM Rfly bridge."""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from pathlib import Path


RFLY_SDK = Path(os.environ.get("RFLY_SDK_ROOT", "/mnt/f/RflySimAPIs/RflySimSDK"))
for sdk_path in (RFLY_SDK, RFLY_SDK / "ctrl", RFLY_SDK / "ue"):
    if str(sdk_path) not in sys.path:
        sys.path.insert(0, str(sdk_path))

from ue.UE4CtrlAPI import UE4CtrlAPI  # noqa: E402


TARGET_ID = 101
TARGET_VEHICLE_TYPE = 50
CAMERA_POSES = (
    (42.0, 42.0, -76.0),
    (112.0, 42.0, -76.0),
    (42.0, 112.0, -62.0),
    (112.0, 112.0, -62.0),
)
MARKER_OFFSETS = (
    (-30.0, -24.0),
    (0.0, -24.0),
    (30.0, -24.0),
    (-30.0, 22.0),
    (0.0, 22.0),
    (30.0, 22.0),
)


class UdpForwarder:
    def __init__(self, socket_obj: socket.socket, host: str, port: int) -> None:
        self.socket_obj = socket_obj
        self.host = host
        self.port = port

    def sendto(self, data: bytes, _address: tuple[str, int]) -> int:
        return self.socket_obj.sendto(data, (self.host, self.port))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge-host", default=os.environ.get("RFLY_UE4_BRIDGE_HOST", "192.168.88.1"))
    parser.add_argument("--bridge-port", type=int, default=int(os.environ.get("RFLY_UE4_BRIDGE_PORT", "30010")))
    parser.add_argument("--initial-delay-seconds", type=float, default=8.0)
    parser.add_argument("--marker-hold-seconds", type=float, default=1.8)
    parser.add_argument("--update-hz", type=float, default=15.0)
    args = parser.parse_args()

    ue = UE4CtrlAPI(os.environ.get("RFLY_HOST_IP", "192.168.88.1"))
    ue.udp_socket = UdpForwarder(ue.udp_socket, args.bridge_host, args.bridge_port)
    ue.sendUE4Destroy(TARGET_ID, 0)
    time.sleep(max(args.initial_delay_seconds, 0.0))
    try:
        for camera_index, camera_pose in enumerate(CAMERA_POSES):
            for marker_index, (offset_x, offset_y) in enumerate(MARKER_OFFSETS):
                target_x = camera_pose[0] + offset_x
                target_y = camera_pose[1] + offset_y
                print(
                    f"camera={camera_index} marker={marker_index} target=({target_x:.1f},{target_y:.1f})",
                    flush=True,
                )
                marker_deadline = time.monotonic() + max(args.marker_hold_seconds, 0.1)
                while time.monotonic() < marker_deadline:
                    ue.sendUE4PosScale2Ground(
                        TARGET_ID,
                        TARGET_VEHICLE_TYPE,
                        0.0,
                        [target_x, target_y, 0.0],
                        [0.0, 0.0, 0.0],
                        [3.0, 3.0, 3.0],
                        windowID=0,
                    )
                    time.sleep(1.0 / max(args.update_hz, 1.0))
    finally:
        ue.sendUE4Destroy(TARGET_ID, 0)


if __name__ == "__main__":
    main()
