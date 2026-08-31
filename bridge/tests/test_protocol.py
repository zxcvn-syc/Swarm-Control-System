import socket
import sys
import unittest
from pathlib import Path


BRIDGE_ROOT = Path(__file__).resolve().parents[1]
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

import protocol


class ProtocolTests(unittest.TestCase):
    def test_round_trip_binary_payload(self):
        left, right = socket.socketpair()
        try:
            protocol.send_record(left, {"kind": "image_raw", "sequence": 7}, b"\x00\x01jpeg")
            header, payload = protocol.recv_record(right)
        finally:
            left.close()
            right.close()
        self.assertEqual({"kind": "image_raw", "sequence": 7}, header)
        self.assertEqual(b"\x00\x01jpeg", payload)

    def test_rejects_oversized_payload(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.encode_record({"kind": "image_raw"}, b"x" * (protocol.MAX_PAYLOAD_BYTES + 1))

    def test_receives_fragmented_record(self):
        left, right = socket.socketpair()
        try:
            record = protocol.encode_record({"kind": "heartbeat", "sequence": 3}, b"ok")
            for index in range(0, len(record), 3):
                left.sendall(record[index : index + 3])
            header, payload = protocol.recv_record(right)
        finally:
            left.close()
            right.close()
        self.assertEqual({"kind": "heartbeat", "sequence": 3}, header)
        self.assertEqual(b"ok", payload)


if __name__ == "__main__":
    unittest.main()
