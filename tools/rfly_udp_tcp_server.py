#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import struct
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=30010)
    parser.add_argument("--tcp-host", default="0.0.0.0")
    parser.add_argument("--tcp-port", type=int, default=31000)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((args.udp_host, args.udp_port))

    tcp_listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_listener.bind((args.tcp_host, args.tcp_port))
    tcp_listener.listen(1)

    forwarded_packets = 0
    forwarded_bytes = 0
    last_reported_at = time.monotonic()
    while True:
        client, _address = tcp_listener.accept()
        print(f"client connected: {_address[0]}:{_address[1]}", flush=True)
        with client:
            while True:
                payload, _source = udp_socket.recvfrom(65535)
                try:
                    client.sendall(struct.pack("!I", len(payload)) + payload)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    print("client disconnected", flush=True)
                    break
                forwarded_packets += 1
                forwarded_bytes += len(payload)
                now = time.monotonic()
                if now - last_reported_at >= 5.0:
                    print(
                        f"forwarded packets={forwarded_packets} bytes={forwarded_bytes}",
                        flush=True,
                    )
                    last_reported_at = now


if __name__ == "__main__":
    main()
