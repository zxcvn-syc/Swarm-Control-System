#!/usr/bin/env python3
"""Receive Jetson ROS 1 observations and republish them into ROS 2.

The receiver deliberately exposes no MAVROS service proxy and no publisher for
setpoints. It is a one-way observation adapter, not a flight-control bridge.
"""

import hmac
import os
import queue
import socket
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image

import protocol


OBSERVATION_KINDS = frozenset(
    ("image_raw", "image_compressed", "camera_info", "pose", "battery", "mavros_state", "heartbeat")
)


def apply_header(target_header, source):
    source = source or {}
    stamp = source.get("stamp") or {}
    target_header.stamp.sec = int(stamp.get("sec", 0))
    target_header.stamp.nanosec = int(stamp.get("nanosec", 0))
    target_header.frame_id = str(source.get("frame_id", ""))


class ObservationClient:
    """Reconnect to the Jetson sender and queue validated observations."""

    def __init__(self, host, port, token, queue_size, logger):
        self._host = host
        self._port = port
        self._token = token
        self._queue = queue.Queue(maxsize=queue_size)
        self._logger = logger
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="ros2-observation-client", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(3.0)

    def get_nowait(self):
        return self._queue.get_nowait()

    def _put(self, record):
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(record)
            except queue.Full:
                self._logger.warning("observation bridge receive queue full; dropping record")

    def _run(self):
        while not self._stop.is_set():
            try:
                self._receive_connection()
            except (OSError, protocol.ProtocolError) as exc:
                if not self._stop.is_set():
                    self._logger.warning("waiting for Jetson observation bridge: %s" % exc)
                    self._stop.wait(2.0)

    def _receive_connection(self):
        connection = socket.create_connection((self._host, self._port), timeout=5.0)
        try:
            protocol.send_record(
                connection,
                {"kind": "hello", "protocol": protocol.PROTOCOL_VERSION, "token": self._token},
            )
            acknowledgment, payload = protocol.recv_record(connection)
            if payload or acknowledgment.get("kind") != "hello_ack":
                raise protocol.ProtocolError("sender rejected hello")
            if int(acknowledgment.get("protocol", 0)) != protocol.PROTOCOL_VERSION:
                raise protocol.ProtocolError("protocol version mismatch")
            self._logger.info("connected to Jetson observation bridge at %s:%s" % (self._host, self._port))
            connection.settimeout(5.0)
            while not self._stop.is_set():
                header, payload = protocol.recv_record(connection)
                kind = header.get("kind")
                if kind not in OBSERVATION_KINDS:
                    raise protocol.ProtocolError("unsupported observation kind: %s" % kind)
                self._put((header, payload))
        finally:
            connection.close()


class Ros2ObservationReceiver(Node):
    """Map bridge records to the topic names used by the ROS 2 deployment."""

    def __init__(self):
        super().__init__("ros2_observation_receiver")
        self.declare_parameter("host", "192.168.144.60")
        self.declare_parameter("port", 19001)
        self.declare_parameter("token", "")
        self.declare_parameter("queue_size", 32)
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("compressed_image_topic", "/camera/image/compressed")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("pose_topic", "/uav0/mavros/local_position/pose")
        self.declare_parameter("battery_topic", "/uav0/mavros/battery")
        self.declare_parameter("state_topic", "/uav0/mavros/state")
        self.declare_parameter("transcode_raw_to_jpeg", True)
        self.declare_parameter("decode_compressed_to_raw", True)
        self.declare_parameter("jpeg_quality", 80)

        token = self._string_parameter("token") or os.environ.get("BRIDGE_TOKEN", "").strip()
        if not token:
            raise RuntimeError("token parameter is required")
        self._image_publisher = self.create_publisher(Image, self._string_parameter("image_topic"), 5)
        self._compressed_publisher = self.create_publisher(
            CompressedImage, self._string_parameter("compressed_image_topic"), 5
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, self._string_parameter("camera_info_topic"), 5
        )
        self._pose_publisher = self.create_publisher(PoseStamped, self._string_parameter("pose_topic"), 10)
        self._battery_publisher = self.create_publisher(BatteryState, self._string_parameter("battery_topic"), 10)
        self._state_publisher = self.create_publisher(State, self._string_parameter("state_topic"), 10)
        self._transcode_raw = bool(self.get_parameter("transcode_raw_to_jpeg").value)
        self._decode_compressed = bool(
            self.get_parameter("decode_compressed_to_raw").value
        )
        self._jpeg_quality = max(1, min(int(self.get_parameter("jpeg_quality").value), 100))
        self._last_native_compressed = 0.0
        self._transport = ObservationClient(
            self._string_parameter("host"),
            int(self.get_parameter("port").value),
            token,
            max(int(self.get_parameter("queue_size").value), 2),
            self.get_logger(),
        )
        self._transport.start()
        self.create_timer(0.01, self._drain_records)

    def destroy_node(self):
        self._transport.stop()
        return super().destroy_node()

    def _string_parameter(self, name):
        return str(self.get_parameter(name).value).strip()

    def _drain_records(self):
        for _ in range(8):
            try:
                header, payload = self._transport.get_nowait()
            except queue.Empty:
                return
            try:
                self._publish_record(header, payload)
            except (TypeError, ValueError, cv2.error) as exc:
                self.get_logger().warning("discarded invalid bridge record: %s" % exc)

    def _publish_record(self, record, payload):
        kind = record["kind"]
        if kind == "image_raw":
            self._publish_raw_image(record, payload)
        elif kind == "image_compressed":
            self._publish_compressed_image(record, payload)
        elif kind == "camera_info":
            self._publish_camera_info(record)
        elif kind == "pose":
            self._publish_pose(record)
        elif kind == "battery":
            self._publish_battery(record)
        elif kind == "mavros_state":
            self._publish_mavros_state(record)

    def _publish_raw_image(self, record, payload):
        message = Image()
        apply_header(message.header, record.get("header"))
        message.height = int(record["height"])
        message.width = int(record["width"])
        message.encoding = str(record["encoding"])
        message.is_bigendian = int(record.get("is_bigendian", 0))
        message.step = int(record["step"])
        expected = message.height * message.step
        if expected <= 0 or expected != len(payload):
            raise ValueError("raw image payload does not match height and step")
        message.data = payload
        self._image_publisher.publish(message)
        if self._transcode_raw and time.monotonic() - self._last_native_compressed > 1.0:
            compressed = self._transcode_to_jpeg(message)
            if compressed is not None:
                self._compressed_publisher.publish(compressed)

    def _publish_compressed_image(self, record, payload):
        message = CompressedImage()
        apply_header(message.header, record.get("header"))
        message.format = str(record.get("format", "jpeg"))
        message.data = payload
        self._compressed_publisher.publish(message)
        self._last_native_compressed = time.monotonic()
        if self._decode_compressed:
            raw = self._decode_to_raw(message)
            if raw is not None:
                self._image_publisher.publish(raw)

    def _publish_camera_info(self, record):
        message = CameraInfo()
        apply_header(message.header, record.get("header"))
        message.height = int(record["height"])
        message.width = int(record["width"])
        message.distortion_model = str(record.get("distortion_model", ""))
        message.d = [float(value) for value in record.get("d", [])]
        message.k = [float(value) for value in record.get("k", [])]
        message.r = [float(value) for value in record.get("r", [])]
        message.p = [float(value) for value in record.get("p", [])]
        message.binning_x = int(record.get("binning_x", 0))
        message.binning_y = int(record.get("binning_y", 0))
        self._camera_info_publisher.publish(message)

    def _publish_pose(self, record):
        message = PoseStamped()
        apply_header(message.header, record.get("header"))
        position = record["position"]
        orientation = record["orientation"]
        message.pose.position.x = float(position["x"])
        message.pose.position.y = float(position["y"])
        message.pose.position.z = float(position["z"])
        message.pose.orientation.x = float(orientation["x"])
        message.pose.orientation.y = float(orientation["y"])
        message.pose.orientation.z = float(orientation["z"])
        message.pose.orientation.w = float(orientation["w"])
        self._pose_publisher.publish(message)

    def _publish_battery(self, record):
        message = BatteryState()
        apply_header(message.header, record.get("header"))
        for name in ("voltage", "temperature", "current", "charge", "capacity", "design_capacity", "percentage"):
            setattr(message, name, float(record.get(name, float("nan"))))
        for name in ("power_supply_status", "power_supply_health", "power_supply_technology"):
            setattr(message, name, int(record.get(name, 0)))
        message.present = bool(record.get("present", False))
        message.cell_voltage = [float(value) for value in record.get("cell_voltage", [])]
        message.cell_temperature = [float(value) for value in record.get("cell_temperature", [])]
        message.location = str(record.get("location", ""))
        message.serial_number = str(record.get("serial_number", ""))
        self._battery_publisher.publish(message)

    def _publish_mavros_state(self, record):
        message = State()
        apply_header(message.header, record.get("header"))
        message.connected = bool(record.get("connected", False))
        message.armed = bool(record.get("armed", False))
        message.guided = bool(record.get("guided", False))
        message.manual_input = bool(record.get("manual_input", False))
        message.mode = str(record.get("mode", ""))
        message.system_status = int(record.get("system_status", 0))
        self._state_publisher.publish(message)

    def _transcode_to_jpeg(self, message):
        channels = {"mono8": 1, "bgr8": 3, "rgb8": 3}.get(message.encoding.lower())
        if channels is None:
            self.get_logger().warning("cannot transcode unsupported image encoding %s" % message.encoding)
            return None
        row = np.frombuffer(message.data, dtype=np.uint8).reshape((message.height, message.step))
        pixels = row[:, : message.width * channels]
        image = pixels.reshape((message.height, message.width, channels)) if channels > 1 else pixels.reshape((message.height, message.width))
        if message.encoding.lower() == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        success, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if not success:
            return None
        compressed = CompressedImage()
        compressed.header = message.header
        compressed.format = "jpeg"
        compressed.data = encoded.tobytes()
        return compressed

    def _decode_to_raw(self, message):
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("compressed image payload is not a BGR JPEG")
        if not image.flags["C_CONTIGUOUS"]:
            image = np.ascontiguousarray(image)
        raw = Image()
        raw.header = message.header
        raw.height = int(image.shape[0])
        raw.width = int(image.shape[1])
        raw.encoding = "bgr8"
        raw.is_bigendian = 0
        raw.step = int(image.shape[1] * image.shape[2])
        raw.data = image.tobytes()
        return raw


def main(args=None):
    rclpy.init(args=args)
    node = Ros2ObservationReceiver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
