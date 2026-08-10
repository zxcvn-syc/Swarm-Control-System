import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("swarm_interfaces")
pytest.importorskip("geometry_msgs")

from containment_pkg.enclosure_node import EnclosureNode
from geometry_msgs.msg import PoseStamped
from swarm_interfaces.msg import DroneState, DroneStateArray, TargetTrack, TargetTrackArray


def make_track(x, y, target_id=1):
    item = TargetTrack()
    item.target_id = target_id
    item.x = x
    item.y = y
    return item


def make_drone(x, y, drone_id=1):
    item = DroneState()
    item.drone_id = drone_id
    item.x = x
    item.y = y
    item.z = 10.0
    item.available = True
    return item


def make_pose(x, y, z=0.0):
    msg = PoseStamped()
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    return msg


@pytest.fixture
def node():
    rclpy.init()
    instance = EnclosureNode()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_target_track_and_drone_callbacks_update_state(node):
    tracks = TargetTrackArray()
    tracks.tracks = [make_track(3.0, 4.0)]
    drones = DroneStateArray()
    drones.drones = [make_drone(20.0, 0.0)]
    node.on_target_track(tracks)
    node.on_drone(drones)
    assert node._targets[0].x == 3.0
    assert node._batch_drones[0].drone_id == 1
    assert node._dirty


def test_tick_publishes_once_until_next_input(node):
    tracks = TargetTrackArray()
    tracks.tracks = [make_track(0.0, 0.0)]
    drones = DroneStateArray()
    drones.drones = [make_drone(20.0, 0.0)]
    published = []
    node._publisher = type("Publisher", (), {"publish": published.append})()
    node.on_target_track(tracks)
    node.on_drone(drones)
    assert node.tick()
    assert len(published) == 1
    assert not node._dirty
    assert not node.tick()
    assert len(published) == 1
    tracks.tracks[0].x = 8.0
    node.on_target_track(tracks)
    assert node.tick()
    assert len(published) == 2
    assert published[-1].commands[0].target_x == 8.0 + 25.0


def test_multiple_targets_and_drones_publish_standby_for_extra_drone(node):
    tracks = TargetTrackArray()
    tracks.tracks = [make_track(0.0, 0.0, 1), make_track(50.0, 0.0, 2)]
    drones = DroneStateArray()
    drones.drones = [make_drone(20.0, 0.0, 1), make_drone(30.0, 0.0, 2), make_drone(40.0, 0.0, 3)]
    published = []
    node._publisher = type("Publisher", (), {"publish": published.append})()
    node.on_target_track(tracks)
    node.on_drone(drones)
    assert node.tick()
    assert len(published[-1].commands) == 3
    assert published[-1].commands[-1].enclosure_radius == 0.0


# ---------------------------------------------------------------------------
# PoseStamped subscription tests (on_pose / _merged_drones)
# ---------------------------------------------------------------------------

def test_on_pose_converts_to_dronestate(node):
    """on_pose should convert PoseStamped to a DroneState in _pose_drones."""
    pose = make_pose(15.0, 25.0, 5.0)
    node.on_pose(pose)
    assert 1 in node._pose_drones
    state = node._pose_drones[1]
    assert state.drone_id == 1
    assert state.x == 15.0
    assert state.y == 25.0
    assert state.z == 5.0
    assert state.available is True
    assert node._dirty


def test_pose_only_publishes_without_batch(node):
    """PoseStamped alone (no DroneStateArray) should still drive enclosure."""
    tracks = TargetTrackArray()
    tracks.tracks = [make_track(0.0, 0.0)]
    published = []
    node._publisher = type("Publisher", (), {"publish": published.append})()
    node.on_target_track(tracks)
    node.on_pose(make_pose(20.0, 0.0))
    assert node.tick()
    assert len(published) == 1
    assert published[-1].commands[0].target_x == 0.0 + 25.0


def test_pose_overrides_batch_for_same_drone(node):
    """When both batch and pose update the same drone_id, pose wins."""
    drones = DroneStateArray()
    drones.drones = [make_drone(20.0, 0.0, 1)]
    node.on_drone(drones)
    node.on_pose(make_pose(50.0, 60.0))
    merged = node._merged_drones()
    assert len(merged) == 1
    assert merged[0].x == 50.0
    assert merged[0].y == 60.0


def test_pose_and_batch_coexist_for_different_drones(node):
    """Batch drones and pose drone coexist; only the pose drone is overridden."""
    drones = DroneStateArray()
    drones.drones = [make_drone(20.0, 0.0, 1), make_drone(30.0, 0.0, 2)]
    node.on_drone(drones)
    # pose_drone_id defaults to 1
    node.on_pose(make_pose(50.0, 60.0))
    merged = node._merged_drones()
    assert len(merged) == 2
    by_id = {int(s.drone_id): s for s in merged}
    assert by_id[1].x == 50.0   # overridden by pose
    assert by_id[2].x == 30.0   # unchanged from batch


def test_pose_dynamic_update_triggers_recalculate(node):
    """Successive pose updates should each produce a new enclosure command."""
    tracks = TargetTrackArray()
    tracks.tracks = [make_track(0.0, 0.0)]
    published = []
    node._publisher = type("Publisher", (), {"publish": published.append})()
    node.on_target_track(tracks)

    # First pose update
    node.on_pose(make_pose(10.0, 0.0))
    assert node.tick()
    assert len(published) == 1

    # Second pose update — drone moved
    node.on_pose(make_pose(15.0, 0.0))
    assert node.tick()
    assert len(published) == 2

    # No new input — should not publish again
    assert not node.tick()
    assert len(published) == 2
