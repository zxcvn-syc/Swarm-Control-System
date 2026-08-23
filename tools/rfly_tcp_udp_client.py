#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import struct
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tcp-host", required=True)
    parser.add_argument("--tcp-port", type=int, default=31000)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=20010)
    parser.add_argument("--reconnect-delay", type=float, default=0.5)
    return parser.parse_args()


def receive_exact(stream: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("Rfly TCP relay closed")
        chunks.extend(chunk)
    return bytes(chunks)


def main() -> None:
    args = parse_args()
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    forwarded_packets = 0
    forwarded_bytes = 0
    last_reported_at = time.monotonic()
    while True:
        try:
            with socket.create_connection(
                (args.tcp_host, args.tcp_port), timeout=5.0
            ) as stream:
                stream.settimeout(None)
                while True:
                    payload_size = struct.unpack("!I", receive_exact(stream, 4))[0]
                    if payload_size > 65535:
                        raise ValueError(f"invalid UDP payload size: {payload_size}")
                    payload = receive_exact(stream, payload_size)
                    udp_socket.sendto(payload, (args.udp_host, args.udp_port))
                    forwarded_packets += 1
                    forwarded_bytes += len(payload)
                    now = time.monotonic()
                    if now - last_reported_at >= 5.0:
                        print(
                            f"forwarded packets={forwarded_packets} bytes={forwarded_bytes}",
                            flush=True,
                        )
                        last_reported_at = now
        except (ConnectionError, OSError, ValueError):
            print("relay reconnecting", flush=True)
            time.sleep(max(args.reconnect_delay, 0.1))


if __name__ == "__main__":
    main()
