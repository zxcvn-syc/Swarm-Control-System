#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=30010)
    parser.add_argument("--target-host", default="127.0.0.1")
    parser.add_argument("--target-port", type=int, default=20010)
    args = parser.parse_args()

    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.listen_host, args.listen_port))
    target = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        data, _address = listener.recvfrom(65535)
        target.sendto(data, (args.target_host, args.target_port))


if __name__ == "__main__":
    main()
