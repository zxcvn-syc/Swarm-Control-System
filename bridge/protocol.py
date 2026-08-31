"""Small length-prefixed transport shared by the ROS 1 and ROS 2 bridge ends.

The format is intentionally simple: a JSON header and an optional binary payload.
It carries observations only. No control request kinds exist in this protocol.
"""

from __future__ import print_function

import json
import struct


PROTOCOL_VERSION = 1
MAX_HEADER_BYTES = 64 * 1024
MAX_PAYLOAD_BYTES = 32 * 1024 * 1024


class ProtocolError(Exception):
    """Raised when a peer sends an invalid or incomplete record."""


def _to_bytes(value):
    if value is None:
        return b""
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, bytes):
        return value
    return value.encode("utf-8")


def encode_record(header, payload=b""):
    """Return a validated wire record for one observation."""

    if not isinstance(header, dict):
        raise ProtocolError("header must be an object")
    raw_header = _to_bytes(json.dumps(header, separators=(",", ":"), sort_keys=True))
    raw_payload = _to_bytes(payload)
    if not raw_header or len(raw_header) > MAX_HEADER_BYTES:
        raise ProtocolError("invalid header length")
    if len(raw_payload) > MAX_PAYLOAD_BYTES:
        raise ProtocolError("payload exceeds limit")
    return (
        struct.pack("!I", len(raw_header))
        + raw_header
        + struct.pack("!I", len(raw_payload))
        + raw_payload
    )


def send_record(sock, header, payload=b""):
    """Send one complete record to an already-connected socket."""

    sock.sendall(encode_record(header, payload))


def recv_exact(sock, length):
    """Receive exactly ``length`` bytes or raise a protocol error."""

    chunks = []
    remaining = length
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ProtocolError("peer closed connection")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_record(sock):
    """Receive and validate one record from the socket."""

    header_length = struct.unpack("!I", recv_exact(sock, 4))[0]
    if header_length == 0 or header_length > MAX_HEADER_BYTES:
        raise ProtocolError("invalid header length")
    try:
        header = json.loads(recv_exact(sock, header_length).decode("utf-8"))
    except (TypeError, ValueError, UnicodeDecodeError) as exc:
        raise ProtocolError("invalid JSON header: {0}".format(exc))
    if not isinstance(header, dict):
        raise ProtocolError("header must be an object")

    payload_length = struct.unpack("!I", recv_exact(sock, 4))[0]
    if payload_length > MAX_PAYLOAD_BYTES:
        raise ProtocolError("payload exceeds limit")
    return header, recv_exact(sock, payload_length)
