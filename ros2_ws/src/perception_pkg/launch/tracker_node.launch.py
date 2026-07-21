"""Launch file for the swarm tracker_node (YOLOv8 + DeepSORT/BoT-SORT).

Two configuration profiles are bundled:

* ``mode:=video`` (default) — reads from a local video file or webcam
  and publishes per-frame ``TargetTrackArray`` messages on
  ``/target_track``.
* ``mode:=topic`` — subscribes to ``/camera/image`` and republishes the
  tracking output on ``/target_track``.

The launch file forwards both the input-source / publish knobs (as
launch arguments) and the detector / tracker knobs (as inline
parameters, since they're typed booleans / numbers and don't fit the
single-value ``LaunchConfiguration`` pattern).  Override either by
editing the inline dict below or by passing a YAML config file via
``config:=tracker_node.yaml``.

Example invocations::

    ros2 launch perception_pkg tracker_node.launch.py \\
        mode:=video video_source:=/data/pexels_aerial_2034115.mp4

    ros2 launch perception_pkg tracker_node.launch.py \\
        mode:=topic image_topic:=/uav/camera/image frame_id:=uav_body

    ros2 launch perception_pkg tracker_node.launch.py \\
        mode:=video tracker.kind:=botsort detector.weights:=/data/yolov8s.pt
"""

from __future__ import annotations

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            'mode', default_value='video',
            choices=['video', 'topic'],
            description='Input source: local video file / webcam, or ROS2 image topic.',
        ),
        DeclareLaunchArgument(
            'video_source', default_value='',
            description='Path to a video file or webcam index (video mode).',
        ),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/image',
            description='sensor_msgs/Image input topic (topic mode).',
        ),
        DeclareLaunchArgument(
            'track_topic', default_value='/target_track',
            description='Output topic for swarm_interfaces/TargetTrackArray.',
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='camera_optical_frame',
            description='frame_id stamped on the published TargetTrackArray.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz', default_value='10.0',
            description='Cap on publishing rate (Hz). 0 means as fast as frames arrive.',
        ),
        DeclareLaunchArgument(
            'loop_video', default_value='false',
            description='When in video mode, restart the file from the beginning at EOF.',
        ),
        DeclareLaunchArgument(
            'config', default_value='',
            description='Optional path to a tracker_node.yaml override file.',
        ),
        # The detector / tracker knobs can also be set as launch args
        # for convenience so users don't have to maintain a YAML file.
        DeclareLaunchArgument(
            'tracker_kind', default_value='deepsort_cascade',
            description='Tracker kind: botsort | deepsort | deepsort_cascade.',
        ),
        DeclareLaunchArgument(
            'detector_backend', default_value='auto',
            description='Detector backend: yolo | auto | mog2.',
        ),
        DeclareLaunchArgument(
            'detector_weights', default_value='',
            description='Absolute path to YOLOv8 .pt weights file.',
        ),
        DeclareLaunchArgument(
            'detector_imgsz', default_value='480',
            description='YOLOv8 input image size (longest side).',
        ),
        DeclareLaunchArgument(
            'detector_conf', default_value='0.15',
            description='YOLOv8 confidence threshold.',
        ),
    ]

    inline_overrides = {
        # Input-source / publish knobs (from launch args).
        'input_mode': LaunchConfiguration('mode'),
        'video_source': LaunchConfiguration('video_source'),
        'image_topic': LaunchConfiguration('image_topic'),
        'track_topic': LaunchConfiguration('track_topic'),
        'frame_id': LaunchConfiguration('frame_id'),
        'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        'loop_video': LaunchConfiguration('loop_video'),
        # Detector / tracker knobs (from launch args; can be overridden
        # by the YAML file when ``config:=`` is provided).
        'detector.backend': LaunchConfiguration('detector_backend'),
        'detector.weights': LaunchConfiguration('detector_weights'),
        'detector.imgsz': LaunchConfiguration('detector_imgsz'),
        'detector.conf': LaunchConfiguration('detector_conf'),
        'tracker.kind': LaunchConfiguration('tracker_kind'),
    }

    tracker_node = Node(
        package='perception_pkg',
        executable='tracker_node',
        name='tracker_node',
        output='screen',
        parameters=[
            # Inline launch args (lowest priority).
            inline_overrides,
            # Optional YAML — overrides the launch args when supplied.
            LaunchConfiguration('config'),
        ],
    )

    return LaunchDescription(args + [tracker_node])