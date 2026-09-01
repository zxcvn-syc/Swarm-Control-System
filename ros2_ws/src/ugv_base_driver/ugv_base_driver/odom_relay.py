"""Pure helpers for the odometry relay (importable without rclpy).

``odom_relay_node`` keeps only ROS wiring; validation and field
extraction live here so they stay unit-testable on machines without
ROS installed — same split as ``pure_pursuit`` / ``diff_kinematics``.
"""

from __future__ import annotations

import math


def as_bool(value: object, default: bool = False) -> bool:
    """Coerce ROS parameter values, including launch-substitution strings."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off", ""}:
            return False
    return default


def odom_pose_fields(msg: object) -> tuple:
    """Validate an odometry message and pull out the fields we relay.

    Accepts any duck-typed object exposing ``header.stamp.sec/.nanosec``,
    ``header.frame_id`` and ``pose.pose.{position,orientation}`` (i.e.
    ``nav_msgs/Odometry``), returns
    ``(stamp_sec, stamp_nanosec, frame_id, position, orientation)`` and
    raises ``ValueError`` on non-finite pose values.
    """
    p = msg.pose.pose.position  # type: ignore[attr-defined]
    q = msg.pose.pose.orientation  # type: ignore[attr-defined]
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]  # type: ignore[attr-defined]
    if not all(math.isfinite(v) for v in values):  # type: ignore[arg-type]
        raise ValueError("non-finite odometry pose")
    return (
        msg.header.stamp.sec,  # type: ignore[attr-defined]
        msg.header.stamp.nanosec,  # type: ignore[attr-defined]
        msg.header.frame_id,  # type: ignore[attr-defined]
        p,
        q,
    )
