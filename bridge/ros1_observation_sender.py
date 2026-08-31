#!/usr/bin/env python
"""ROS 1 observation sender for the isolated Jetson compatibility bridge.

This process only subscribes to ROS 1 topics and forwards observations to an
authenticated TCP peer. It has no publishers, service clients, MAVLink, or
control-message handling.
"""

from __future__ import print_function

import hmac
import os
import socket
import threading
import time

try:
    import queue
except ImportError:
    import Queue as queue

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image

import protocol


OBSERVATION_KINDS = frozenset(
    (
        "image_raw",
        "image_compressed",
        "camera_info",
        "pose",
        "battery",
        "mavros_state",
        "heartbeat",
    )
)


def stamp_to_dict(stamp):
    return {
        "sec": int(getattr(stamp, "secs", 0)),
        "nanosec": int(getattr(stamp, "nsecs", 0)),
    }


def header_to_dict(header):
    return {
        "stamp": stamp_to_dict(header.stamp),
        "frame_id": str(header.frame_id),
    }


def float_list(values):
    return [float(value) for value in values]


class ObservationServer(object):
    """Own a local TCP listener and serialize the latest ROS 1 observations."""

    def __init__(self, bind_host, port, token, queue_size):
        self._bind_host = bind_host
        self._port = port
        self._token = token
        self._queue = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, name="ros1-observation-server")
        self._thread.daemon = True

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(2.0)

    def publish(self, header, payload=b""):
        kind = header.get("kind")
        if kind not in OBSERVATION_KINDS:
            raise ValueError("unsupported observation kind: {0}".format(kind))
        record = (header, payload)
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
                rospy.logwarn("observation bridge queue is full; dropping %s", kind)

    def _serve(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self._bind_host, self._port))
        listener.listen(1)
        listener.settimeout(1.0)
        rospy.loginfo("ROS1 observation bridge listening on %s:%s", self._bind_host, self._port)
        try:
            while not self._stop.is_set() and not rospy.is_shutdown():
                try:
                    connection, peer = listener.accept()
                except socket.timeout:
                    continue
                try:
                    self._serve_peer(connection, peer)
                except (socket.error, protocol.ProtocolError) as exc:
                    rospy.logwarn("observation bridge peer disconnected: %s", exc)
                finally:
                    try:
                        connection.close()
                    except socket.error:
                        pass
        finally:
            listener.close()

    def _serve_peer(self, connection, peer):
        connection.settimeout(5.0)
        hello, payload = protocol.recv_record(connection)
        if payload or hello.get("kind") != "hello":
            raise protocol.ProtocolError("missing hello record")
        received_token = str(hello.get("token", ""))
        if not hmac.compare_digest(received_token, self._token):
            raise protocol.ProtocolError("bridge token rejected")
        if int(hello.get("protocol", 0)) != protocol.PROTOCOL_VERSION:
            raise protocol.ProtocolError("protocol version mismatch")
        protocol.send_record(
            connection,
            {"kind": "hello_ack", "protocol": protocol.PROTOCOL_VERSION},
        )
        connection.settimeout(None)
        rospy.loginfo("ROS2 observation receiver connected from %s:%s", peer[0], peer[1])
        last_heartbeat = time.time()
        while not self._stop.is_set() and not rospy.is_shutdown():
            try:
                header, payload = self._queue.get(timeout=0.5)
                protocol.send_record(connection, header, payload)
            except queue.Empty:
                if time.time() - last_heartbeat >= 1.0:
                    protocol.send_record(
                        connection,
                        {"kind": "heartbeat", "sent_unix": time.time()},
                    )
                    last_heartbeat = time.time()


class Ros1ObservationSender(object):
    """Convert selected ROS 1 observations into bridge records."""

    def __init__(self):
        bind_host = rospy.get_param("~bind_host", "0.0.0.0")
        port = int(rospy.get_param("~port", 19001))
        token = str(rospy.get_param("~token", os.environ.get("BRIDGE_TOKEN", ""))).strip()
        queue_size = max(int(rospy.get_param("~queue_size", 32)), 2)
        self._max_image_hz = max(float(rospy.get_param("~max_image_hz", 15.0)), 0.0)
        self._last_image_sent = 0.0
        if not token:
            raise RuntimeError("~token is required")
        self._transport = ObservationServer(bind_host, port, token, queue_size)
        self._transport.start()

        self._subscribe("~image_topic", "/camera/color/image_raw", Image, self._on_raw_image)
        self._subscribe(
            "~compressed_image_topic",
            "/camera/color/image_raw/compressed",
            CompressedImage,
            self._on_compressed_image,
        )
        self._subscribe("~camera_info_topic", "/camera/color/camera_info", CameraInfo, self._on_camera_info)
        self._subscribe("~pose_topic", "/mavros/local_position/pose", PoseStamped, self._on_pose)
        self._subscribe("~battery_topic", "/mavros/battery", BatteryState, self._on_battery)
        self._subscribe("~state_topic", "/mavros/state", State, self._on_mavros_state)

    def close(self):
        self._transport.stop()

    def _subscribe(self, parameter, default_topic, message_type, callback):
        topic = str(rospy.get_param(parameter, default_topic)).strip()
        if topic:
            rospy.Subscriber(topic, message_type, callback, queue_size=1)
            rospy.loginfo("bridging ROS1 topic %s", topic)

    def _allow_image(self):
        if self._max_image_hz == 0.0:
            return True
        now = time.time()
        if now - self._last_image_sent < 1.0 / self._max_image_hz:
            return False
        self._last_image_sent = now
        return True

    def _on_raw_image(self, message):
        if not self._allow_image():
            return
        self._transport.publish(
            {
                "kind": "image_raw",
                "header": header_to_dict(message.header),
                "height": int(message.height),
                "width": int(message.width),
                "encoding": str(message.encoding),
                "is_bigendian": int(message.is_bigendian),
                "step": int(message.step),
            },
            message.data,
        )

    def _on_compressed_image(self, message):
        if not self._allow_image():
            return
        self._transport.publish(
            {
                "kind": "image_compressed",
                "header": header_to_dict(message.header),
                "format": str(message.format),
            },
            message.data,
        )

    def _on_camera_info(self, message):
        self._transport.publish(
            {
                "kind": "camera_info",
                "header": header_to_dict(message.header),
                "height": int(message.height),
                "width": int(message.width),
                "distortion_model": str(message.distortion_model),
                "d": float_list(message.D),
                "k": float_list(message.K),
                "r": float_list(message.R),
                "p": float_list(message.P),
                "binning_x": int(message.binning_x),
                "binning_y": int(message.binning_y),
            }
        )

    def _on_pose(self, message):
        pose = message.pose
        self._transport.publish(
            {
                "kind": "pose",
                "header": header_to_dict(message.header),
                "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
                "orientation": {
                    "x": pose.orientation.x,
                    "y": pose.orientation.y,
                    "z": pose.orientation.z,
                    "w": pose.orientation.w,
                },
            }
        )

    def _on_battery(self, message):
        self._transport.publish(
            {
                "kind": "battery",
                "header": header_to_dict(message.header),
                "voltage": message.voltage,
                "temperature": message.temperature,
                "current": message.current,
                "charge": message.charge,
                "capacity": message.capacity,
                "design_capacity": message.design_capacity,
                "percentage": message.percentage,
                "power_supply_status": int(message.power_supply_status),
                "power_supply_health": int(message.power_supply_health),
                "power_supply_technology": int(message.power_supply_technology),
                "present": bool(message.present),
                "cell_voltage": float_list(message.cell_voltage),
                "cell_temperature": float_list(message.cell_temperature),
                "location": str(message.location),
                "serial_number": str(message.serial_number),
            }
        )

    def _on_mavros_state(self, message):
        self._transport.publish(
            {
                "kind": "mavros_state",
                "header": header_to_dict(message.header),
                "connected": bool(message.connected),
                "armed": bool(message.armed),
                "guided": bool(message.guided),
                "manual_input": bool(message.manual_input),
                "mode": str(message.mode),
                "system_status": int(message.system_status),
            }
        )


def main():
    rospy.init_node("ros1_observation_sender", anonymous=False)
    sender = Ros1ObservationSender()
    rospy.on_shutdown(sender.close)
    rospy.spin()


if __name__ == "__main__":
    main()
