import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("swarm_interfaces")

from containment_pkg.enclosure_node import EnclosureNode
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
    assert node._drones[0].drone_id == 1
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
