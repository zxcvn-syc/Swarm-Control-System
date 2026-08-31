#!/usr/bin/env python
"""Publish JPEG frames from a local UVC camera into ROS 1."""

from __future__ import print_function

import cv2
import rospy
from sensor_msgs.msg import CompressedImage


class UvcCameraPublisher(object):
    def __init__(self):
        device_index = int(rospy.get_param("~device_index", 0))
        self._width = max(int(rospy.get_param("~width", 1280)), 1)
        self._height = max(int(rospy.get_param("~height", 720)), 1)
        self._fps = max(float(rospy.get_param("~fps", 12.0)), 1.0)
        self._jpeg_quality = max(
            1,
            min(int(rospy.get_param("~jpeg_quality", 80)), 100),
        )
        self._frame_id = str(
            rospy.get_param("~frame_id", "camera_color_optical_frame")
        )
        image_topic = str(
            rospy.get_param(
                "~compressed_image_topic",
                "/camera/color/image_raw/compressed",
            )
        ).strip()
        if not image_topic:
            raise RuntimeError("~compressed_image_topic is required")

        self._capture = cv2.VideoCapture(device_index)
        if not self._capture.isOpened():
            raise RuntimeError("cannot open UVC camera index {0}".format(device_index))

        if hasattr(cv2, "VideoWriter_fourcc"):
            self._capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*"MJPG")
            )
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._capture.set(cv2.CAP_PROP_FPS, self._fps)
        self._publisher = rospy.Publisher(image_topic, CompressedImage, queue_size=1)
        rospy.on_shutdown(self.close)
        rospy.loginfo(
            "publishing UVC JPEG camera %s at %sx%s %.1f FPS on %s",
            device_index,
            self._width,
            self._height,
            self._fps,
            image_topic,
        )

    def close(self):
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def run(self):
        rate = rospy.Rate(self._fps)
        while not rospy.is_shutdown():
            success, frame = self._capture.read()
            if not success or frame is None:
                rospy.logwarn_throttle(5.0, "UVC camera frame read failed")
                rate.sleep()
                continue
            if frame.ndim != 3 or frame.shape[2] != 3:
                rospy.logwarn_throttle(5.0, "UVC camera returned a non-BGR frame")
                rate.sleep()
                continue
            encoded_ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
            )
            if not encoded_ok:
                rospy.logwarn_throttle(5.0, "UVC camera JPEG encoding failed")
                rate.sleep()
                continue

            message = CompressedImage()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = self._frame_id
            message.format = "jpeg"
            message.data = encoded.tostring()
            self._publisher.publish(message)
            rate.sleep()


def main():
    rospy.init_node("ros1_uvc_camera", anonymous=False)
    publisher = UvcCameraPublisher()
    publisher.run()


if __name__ == "__main__":
    main()
