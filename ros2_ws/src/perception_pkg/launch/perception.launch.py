"""Launch file for the full perception pipeline.

This launch file starts both the ``tracker_node`` (YOLOv8 + DeepSORT / BoT-SORT
→ TargetTrackArray) and optionally the ``coord_transform_node`` (pixel → world
coordinate transform).

Example invocations::

    # Default: video input, CPU, MOG2 fallback
    ros2 launch perception_pkg perception.launch.py

    # YOLO on a synthetic multi-target video
    ros2 launch perception_pkg perception.launch.py \\
        video_source:=videos/test_synthetic_multi_target.mp4 \\
        detector.backend:=yolo \\
        detector.weights:=/path/to/visdrone_yolov8s.pt

    # Topic input with world-frame transform
    ros2 launch perception_pkg perception.launch.py \\
        input_mode:=topic \\
        image_topic:=/uav/camera/image \\
        coord_transform.enabled:=true \\
        camera_info_topic:=/uav/camera/camera_info

    # Two-source fusion (each source must publish /<name>/target_track)
    ros2 launch perception_pkg perception.launch.py \\
        enable_fusion:=true \\
        fusion_sources:=[sensor_0, sensor_1] \\
        track_topic:=/target_track
"""

from __future__ import annotations

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_tracker_node() -> Node:
    """Build the tracker_node Node with all launch-arg-driven parameters."""
    return Node(
        package="perception_pkg",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
        parameters=[
            {
                # Input / output
                "input_mode": LaunchConfiguration("input_mode"),
                "video_source": LaunchConfiguration("video_source"),
                "image_topic": LaunchConfiguration("image_topic"),
                "track_topic": LaunchConfiguration("track_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "loop_video": LaunchConfiguration("loop_video"),
                "enable_debug_topics": LaunchConfiguration("enable_debug_topics"),
                # Fusion
                "enable_fusion": LaunchConfiguration("enable_fusion"),
                "fusion_sources": LaunchConfiguration("fusion_sources"),
                "sources": LaunchConfiguration("sources"),
                # Detector
                "detector.backend": LaunchConfiguration("detector_backend"),
                "detector.weights": LaunchConfiguration("detector_weights"),
                "detector.device": LaunchConfiguration("detector_device"),
                "detector.imgsz": LaunchConfiguration("detector_imgsz"),
                "detector.conf": LaunchConfiguration("detector_conf"),
                "detector.classes": LaunchConfiguration("detector_classes"),
                "detector.min_box_area": LaunchConfiguration("detector_min_box_area"),
                "detector.min_conf": LaunchConfiguration("detector_min_conf"),
                "detector.nms_iou": LaunchConfiguration("detector_nms_iou"),
                # Tracker
                "tracker.kind": LaunchConfiguration("tracker_kind"),
                "tracker.dt": LaunchConfiguration("tracker_dt"),
                "tracker.max_age": LaunchConfiguration("tracker_max_age"),
                "tracker.n_init": LaunchConfiguration("tracker_n_init"),
                "tracker.iou_thresh": LaunchConfiguration("tracker_iou_thresh"),
                "tracker.high_conf": LaunchConfiguration("tracker_high_conf"),
                "tracker.new_track_conf": LaunchConfiguration("tracker_new_track_conf"),
                "tracker.lost_relink_frames": LaunchConfiguration("tracker_lost_relink_frames"),
                "tracker.stationary_prune": LaunchConfiguration("tracker_stationary_prune"),
                "tracker.include_tentative": LaunchConfiguration("tracker_include_tentative"),
                # Kalman (adaptive trackers)
                "tracker.kalman.dt": LaunchConfiguration("kalman_dt"),
                "tracker.kalman.sigma_p": LaunchConfiguration("kalman_sigma_p"),
                "tracker.kalman.sigma_v": LaunchConfiguration("kalman_sigma_v"),
                "tracker.kalman.sigma_m": LaunchConfiguration("kalman_sigma_m"),
                "tracker.kalman.acceleration_gain": LaunchConfiguration("kalman_acceleration_gain"),
                "tracker.kalman.motion_threshold_slow": LaunchConfiguration("kalman_motion_threshold_slow"),
                "tracker.kalman.motion_threshold_fast": LaunchConfiguration("kalman_motion_threshold_fast"),
                "tracker.kalman.base_std_pos": LaunchConfiguration("kalman_base_std_pos"),
                "tracker.kalman.base_std_vel": LaunchConfiguration("kalman_base_std_vel"),
                "tracker.kalman.base_std_meas": LaunchConfiguration("kalman_base_std_meas"),
                "tracker.kalman.motion_adapt_gain": LaunchConfiguration("kalman_motion_adapt_gain"),
                "tracker.kalman.velocity_limit": LaunchConfiguration("kalman_velocity_limit"),
                "tracker.kalman.innovation_gate": LaunchConfiguration("kalman_innovation_gate"),
                # Trajectory prediction
                "trajectory_prediction.enabled": LaunchConfiguration("tp_enabled"),
                "trajectory_prediction.prediction_steps": LaunchConfiguration("tp_prediction_steps"),
                "trajectory_prediction.confidence_decay": LaunchConfiguration("tp_confidence_decay"),
                "trajectory_prediction.min_confidence": LaunchConfiguration("tp_min_confidence"),
                # Appearance
                "appearance.enabled": LaunchConfiguration("appearance_enabled"),
                "appearance.weights": LaunchConfiguration("appearance_weights"),
                # Enclosure
                "enclosure.enabled": LaunchConfiguration("enclosure_enabled"),
                "enclosure.topic": LaunchConfiguration("enclosure_topic"),
                "enclosure.publish_rate_hz": LaunchConfiguration("enclosure_publish_rate_hz"),
                "enclosure.drone_positions": LaunchConfiguration("enclosure_drone_positions"),
                # Diagnostics
                "metrics_period_ms": LaunchConfiguration("metrics_period_ms"),
            },
            # Optional YAML override file (lowest priority)
            LaunchConfiguration("config"),
        ],
    )


def _make_coord_transform_node() -> Node | None:
    """Build the coord_transform_node if coord_transform.enabled:=true."""
    enabled = LaunchConfiguration("coord_transform_enabled")
    return Node(
        package="perception_pkg",
        executable="coord_transform_node",
        name="coord_transform_node",
        output="screen",
        parameters=[
            {
                "enabled": enabled,
                "input_topic": LaunchConfiguration("track_topic"),
                "output_topic": LaunchConfiguration("coord_transform_output_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "drone_pose_topic": LaunchConfiguration("drone_pose_topic"),
                "world_frame": LaunchConfiguration("world_frame"),
                "ground_altitude": LaunchConfiguration("ground_altitude"),
                "max_pose_age_s": LaunchConfiguration("max_pose_age_s"),
                "camera_mount_roll": LaunchConfiguration("camera_mount_roll"),
                "camera_mount_pitch": LaunchConfiguration("camera_mount_pitch"),
                "camera_mount_yaw": LaunchConfiguration("camera_mount_yaw"),
                "frame_id": LaunchConfiguration("world_frame"),
                "publish_debug": LaunchConfiguration("publish_debug"),
            },
        ],
    )


def generate_launch_description() -> LaunchDescription:
    # ── Input / output ──────────────────────────────────────────────────────
    args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument("input_mode", default_value="video",
                             choices=["video", "topic"],
                             description="video (local file/webcam) or topic (ROS2 image)"),
        DeclareLaunchArgument("video_source", default_value="",
                             description="Path or webcam index (video mode)"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image",
                             description="sensor_msgs/Image topic (topic mode)"),
        DeclareLaunchArgument("track_topic", default_value="/target_track",
                             description="Output TargetTrackArray topic"),
        DeclareLaunchArgument("frame_id", default_value="camera_optical_frame",
                             description="frame_id on published messages"),
        DeclareLaunchArgument("publish_rate_hz", default_value="10.0",
                             description="Cap on publish rate (0 = as-fast-as-frames)"),
        DeclareLaunchArgument("loop_video", default_value="false",
                             description="Loop video at EOF (video mode)"),
        DeclareLaunchArgument("enable_debug_topics", default_value="true",
                             description="Publish /target_track_debug and /tracking_metrics"),
        # Fusion
        DeclareLaunchArgument("enable_fusion", default_value="false",
                             description="Enable multi-source track fusion"),
        DeclareLaunchArgument("fusion_sources", default_value="[]",
                             description="Source names for fusion, e.g. [sensor_0,sensor_1]"),
        DeclareLaunchArgument("sources", default_value="[]",
                             description="Fallback alias for fusion_sources"),
        # Detector
        DeclareLaunchArgument("detector_backend", default_value="yolo",
                             description="Detector backend: yolo | auto"),
        DeclareLaunchArgument("detector_weights", default_value="",
                             description="Absolute path to YOLOv8 .pt weights"),
        DeclareLaunchArgument("detector_device", default_value="cpu",
                             description="Inference device: cpu | cuda:0"),
        DeclareLaunchArgument("detector_imgsz", default_value="480",
                             description="YOLO input size (longest side, pixels)"),
        DeclareLaunchArgument("detector_conf", default_value="0.15",
                             description="Detection confidence threshold"),
        DeclareLaunchArgument(
            "detector_classes", default_value="[0, 1, 2, 3, 4, 5, 7, 8]",
            description="COCO class IDs to detect"),
        DeclareLaunchArgument("detector_min_box_area", default_value="200.0",
                             description="Minimum bbox area in pixels"),
        DeclareLaunchArgument("detector_min_conf", default_value="0.0",
                             description="Minimum detection confidence"),
        DeclareLaunchArgument("detector_nms_iou", default_value="0.5",
                             description="NMS IoU threshold"),
        # Tracker
        DeclareLaunchArgument("tracker_kind", default_value="deepsort_cascade",
                             choices=["botsort", "deepsort", "deepsort_cascade",
                                      "botsort_adaptive", "deepsort_adaptive"],
                             description="Tracker algorithm"),
        DeclareLaunchArgument("tracker_dt", default_value="0.05",
                             description="Time step in seconds"),
        DeclareLaunchArgument("tracker_max_age", default_value="30",
                             description="Max missed frames before track deletion"),
        DeclareLaunchArgument("tracker_n_init", default_value="3",
                             description="Frames to confirm a new track"),
        DeclareLaunchArgument("tracker_iou_thresh", default_value="0.30",
                             description="IoU threshold for track-update matching"),
        DeclareLaunchArgument("tracker_high_conf", default_value="0.35",
                             description="High-confidence detection threshold"),
        DeclareLaunchArgument("tracker_new_track_conf", default_value="0.20",
                             description="Minimum confidence for new track"),
        DeclareLaunchArgument("tracker_lost_relink_frames", default_value="30",
                             description="Frames to attempt track re-linking"),
        DeclareLaunchArgument("tracker_stationary_prune", default_value="true",
                             description="Prune stationary tracks"),
        DeclareLaunchArgument("tracker_include_tentative", default_value="false",
                             description="Include unconfirmed tracks in output"),
        # Kalman
        DeclareLaunchArgument("kalman_dt", default_value="0.05"),
        DeclareLaunchArgument("kalman_sigma_p", default_value="0.05"),
        DeclareLaunchArgument("kalman_sigma_v", default_value="0.00625"),
        DeclareLaunchArgument("kalman_sigma_m", default_value="0.05"),
        DeclareLaunchArgument("kalman_acceleration_gain", default_value="0.5"),
        DeclareLaunchArgument("kalman_motion_threshold_slow", default_value="2.0"),
        DeclareLaunchArgument("kalman_motion_threshold_fast", default_value="20.0"),
        DeclareLaunchArgument("kalman_base_std_pos", default_value="0.05"),
        DeclareLaunchArgument("kalman_base_std_vel", default_value="0.00625"),
        DeclareLaunchArgument("kalman_base_std_meas", default_value="0.05"),
        DeclareLaunchArgument("kalman_motion_adapt_gain", default_value="0.3"),
        DeclareLaunchArgument("kalman_velocity_limit", default_value="100.0"),
        DeclareLaunchArgument("kalman_innovation_gate", default_value="9.4877"),
        # Trajectory prediction
        DeclareLaunchArgument("tp_enabled", default_value="true"),
        DeclareLaunchArgument("tp_prediction_steps", default_value="10"),
        DeclareLaunchArgument("tp_confidence_decay", default_value="0.9"),
        DeclareLaunchArgument("tp_min_confidence", default_value="0.1"),
        # Appearance
        DeclareLaunchArgument("appearance_enabled", default_value="false"),
        DeclareLaunchArgument("appearance_weights", default_value=""),
        # Enclosure
        DeclareLaunchArgument("enclosure_enabled", default_value="false"),
        DeclareLaunchArgument("enclosure_topic", default_value="/enclosure_targets"),
        DeclareLaunchArgument("enclosure_publish_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("enclosure_drone_positions", default_value="[]"),
        # Diagnostics
        DeclareLaunchArgument("metrics_period_ms", default_value="1000"),
        # Coordinate transform
        DeclareLaunchArgument("coord_transform_enabled", default_value="false"),
        DeclareLaunchArgument("coord_transform_output_topic", default_value="/target_track_world"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("drone_pose_topic", default_value="/drone_pose"),
        DeclareLaunchArgument("world_frame", default_value="world"),
        DeclareLaunchArgument("ground_altitude", default_value="0.0"),
        DeclareLaunchArgument("max_pose_age_s", default_value="0.5"),
        DeclareLaunchArgument("camera_mount_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_mount_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_mount_yaw", default_value="0.0"),
        DeclareLaunchArgument("publish_debug", default_value="true"),
        # Config override
        DeclareLaunchArgument("config", default_value="",
                             description="Optional path to a perception.yaml override file"),
    ]

    return LaunchDescription(args + [
        OpaqueFunction(function=lambda ctx: [_make_tracker_node()]),
        OpaqueFunction(
            function=lambda ctx: [_make_coord_transform_node()]
            if _ctx_coord_enabled(ctx) else []
        ),
    ])


def _ctx_coord_enabled(ctx) -> bool:
    """Return True when coord_transform.enabled is set to True."""
    return ctx.launch_configurations.get("coord_transform_enabled", "false").lower() == "true"
