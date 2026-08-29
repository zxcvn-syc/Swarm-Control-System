from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("mavros_msgs")

from planning_pkg.px4_offboard_bridge import PX4OffboardBridge


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def _pose(drone_id, x, y, z):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=f"drone_{drone_id}"),
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z)),
    )


def _bridge(drone_id):
    node = PX4OffboardBridge.__new__(PX4OffboardBridge)
    node.drone_id = drone_id
    node._waypoints = []
    node._index = 7
    node._warned_path_scope = False
    logger = _Logger()
    node.get_logger = lambda: logger
    return node, logger


def test_path_callback_keeps_only_the_configured_drone_path():
    node, logger = _bridge(1)
    message = SimpleNamespace(
        poses=[_pose(0, 1.0, 2.0, 3.0), _pose(1, 4.0, 5.0, 6.0)],
    )

    PX4OffboardBridge.on_path(node, message)

    assert node._waypoints == [(4.0, 5.0, 6.0)]
    assert node._index == 0
    assert logger.warnings == []


def test_path_callback_rejects_unscoped_drone_id():
    node, logger = _bridge(-1)
    node._waypoints = [(9.0, 9.0, 9.0)]
    message = SimpleNamespace(poses=[_pose(0, 1.0, 2.0, 3.0)])

    PX4OffboardBridge.on_path(node, message)

    assert node._waypoints == []
    assert node._index == 0
    assert logger.warnings == [
        "ignoring /planned_path until a non-negative drone_id is set"
    ]


def test_disabled_streaming_does_not_publish_setpoints():
    node = PX4OffboardBridge.__new__(PX4OffboardBridge)
    node.enable_setpoint_streaming = False
    published = []
    node._publish_setpoint = lambda: published.append(True)

    PX4OffboardBridge.tick(node)

    assert published == []


def test_safety_hold_captures_pose_and_discards_stale_path():
    node = PX4OffboardBridge.__new__(PX4OffboardBridge)
    node._safety_hold = False
    node._safety_hold_position = None
    node._local_position = (4.0, 5.0, 6.0)
    node._waypoints = [(9.0, 9.0, 9.0)]
    node._index = 1
    logger = _Logger()
    node.get_logger = lambda: logger

    PX4OffboardBridge.on_safety_hold(node, SimpleNamespace(data=True))

    assert node._safety_hold
    assert node._safety_hold_position == (4.0, 5.0, 6.0)
    assert node._waypoints == []
    assert node._index == 0
    assert logger.warnings == ["safety hold requested; path cleared and position captured"]


def test_safety_hold_blocks_auto_arm_and_offboard_requests():
    node = PX4OffboardBridge.__new__(PX4OffboardBridge)
    node.enable_setpoint_streaming = True
    node._safety_hold = True
    node.auto_arm = True
    published = []
    node._publish_setpoint = lambda: published.append(True) or True

    PX4OffboardBridge.tick(node)

    assert published == [True]
