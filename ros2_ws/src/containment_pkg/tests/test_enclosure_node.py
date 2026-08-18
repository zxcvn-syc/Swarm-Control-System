import math

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("swarm_interfaces")
pytest.importorskip("geometry_msgs")

from containment_pkg.enclosure_node import (
    LAYER_BLOCK,
    LAYER_COMMAND,
    LAYER_MONITOR,
    EnclosureNode,
)
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
    item.platform_type = 0  # PLATFORM_DRONE
    return item


def make_car(x, y, car_id=100):
    item = DroneState()
    item.drone_id = car_id
    item.x = x
    item.y = y
    item.z = 0.0
    item.available = True
    item.platform_type = 1  # PLATFORM_CAR
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


def test_multiple_targets_and_drones_all_active(node):
    """All platforms in a containment layer should be assigned a ring point."""
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
    for cmd in published[-1].commands:
        assert cmd.enclosure_radius > 0.0
        assert math.isfinite(cmd.target_x)
        assert math.isfinite(cmd.target_y)


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


# ---------------------------------------------------------------------------
# Layered enclosure tests (three-layer containment)
# ---------------------------------------------------------------------------

def test_layer_of_maps_platform_type(node):
    """UAVs go to the monitor layer, UGVs to the block layer."""
    assert node._layer_of(make_drone(0.0, 0.0)) == LAYER_MONITOR
    assert node._layer_of(make_car(0.0, 0.0)) == LAYER_BLOCK


def test_layer_of_defaults_to_monitor_for_missing_field(node):
    """Legacy states without platform_type fall back to the monitor layer."""
    fake = type("FakeState", (), {"drone_id": 9, "x": 0.0, "y": 0.0, "z": 1.0})()
    assert node._layer_of(fake) == LAYER_MONITOR


def test_drone_and_car_get_different_layer_radii(node):
    """Monitor layer uses 25.0, block layer uses 15.0, layer field is set."""
    tracks = TargetTrackArray()
    tracks.tracks = [make_track(0.0, 0.0)]
    drones = DroneStateArray()
    drones.drones = [make_drone(20.0, 0.0, 1), make_car(18.0, 0.0, 101)]
    published = []
    node._publisher = type("Publisher", (), {"publish": published.append})()
    node.on_target_track(tracks)
    node.on_drone(drones)
    assert node.tick()
    cmds = {int(c.drone_id): c for c in published[-1].commands}
    assert cmds[1].layer == LAYER_MONITOR
    assert cmds[101].layer == LAYER_BLOCK
    # Target at origin; drone at (20,0) -> monitor point (25,0); car -> (15,0)
    assert cmds[1].target_x == pytest.approx(25.0)
    assert cmds[101].target_x == pytest.approx(15.0)
    assert cmds[1].enclosure_radius == pytest.approx(25.0)
    assert cmds[101].enclosure_radius == pytest.approx(15.0)


def test_layered_standby_is_per_layer(node):
    """Extra platforms standby within their own layer only."""
    tracks = TargetTrackArray()
    tracks.tracks = [make_track(0.0, 0.0)]
    drones = DroneStateArray()
    # 2 UAVs + 2 UGVs vs 1 target -> one standby in each layer
    drones.drones = [
        make_drone(20.0, 0.0, 1),
        make_drone(30.0, 0.0, 2),
        make_car(18.0, 0.0, 101),
        make_car(16.0, 0.0, 102),
    ]
    published = []
    node._publisher = type("Publisher", (), {"publish": published.append})()
    node.on_target_track(tracks)
    node.on_drone(drones)
    assert node.tick()
    cmds = {int(c.drone_id): c for c in published[-1].commands}
    # first drone of each layer active
    assert cmds[1].enclosure_radius == pytest.approx(25.0)
    assert cmds[101].enclosure_radius == pytest.approx(15.0)
    # second of each layer standby (NaN target, radius 0)
    assert cmds[2].enclosure_radius == 0.0
    assert cmds[102].enclosure_radius == 0.0


def test_three_uav_two_car_scene_publishes_all_layers(node):
    """Full 3-UAV + 2-car mock scene: all platforms get a layer command."""
    tracks = TargetTrackArray()
    tracks.tracks = [make_track(0.0, 0.0), make_track(40.0, 0.0, 2)]
    drones = DroneStateArray()
    drones.drones = [
        make_drone(20.0, 0.0, 0),
        make_drone(30.0, 0.0, 1),
        make_drone(60.0, 0.0, 2),
        make_car(18.0, 0.0, 100),
        make_car(50.0, 0.0, 101),
    ]
    published = []
    node._publisher = type("Publisher", (), {"publish": published.append})()
    node.on_target_track(tracks)
    node.on_drone(drones)
    assert node.tick()
    cmds = {int(c.drone_id): c for c in published[-1].commands}
    assert len(cmds) == 5
    assert cmds[0].layer == LAYER_MONITOR
    assert cmds[100].layer == LAYER_BLOCK
    assert cmds[101].layer == LAYER_BLOCK


def test_command_layer_reserved_standby(node):
    """A platform marked for the command layer waits on standby (reserved)."""
    tracks = TargetTrackArray()
    tracks.tracks = [make_track(0.0, 0.0)]
    drones = DroneStateArray()
    cmd_state = DroneState()
    cmd_state.drone_id = 200
    cmd_state.x = 10.0
    cmd_state.y = 0.0
    cmd_state.z = 2.0
    cmd_state.available = True
    cmd_state.platform_type = 2  # command-layer platform (reserved)
    drones.drones = [make_drone(20.0, 0.0, 1), cmd_state]
    published = []
    node._publisher = type("Publisher", (), {"publish": published.append})()
    node.on_target_track(tracks)
    node.on_drone(drones)
    assert node.tick()
    cmds = {int(c.drone_id): c for c in published[-1].commands}
    assert cmds[1].layer == LAYER_MONITOR
    assert cmds[1].enclosure_radius == pytest.approx(25.0)
    assert cmds[200].layer == LAYER_COMMAND
    assert cmds[200].enclosure_radius == 0.0


# ---------------------------------------------------------------------------
# UGV block-layer dynamic containment (target moves -> block recomputes)
# ---------------------------------------------------------------------------

def test_ugv_block_layer_tracks_moving_target(node):
    """When the target moves, the UGV block-layer point follows it."""
    car = make_car(15.0, 0.0, 101)  # UGV on the block layer
    drones = DroneStateArray()
    drones.drones = [car]
    published = []
    node._publisher = type("Publisher", (), {"publish": published.append})()

    # Target at origin -> UGV block point at (15, 0)
    t1 = TargetTrackArray()
    t1.tracks = [make_track(0.0, 0.0, 1)]
    node.on_target_track(t1)
    node.on_drone(drones)
    assert node.tick()
    first = {int(c.drone_id): c for c in published[-1].commands}[101]
    assert first.layer == LAYER_BLOCK
    assert first.target_x == pytest.approx(15.0)
    assert first.enclosure_radius == pytest.approx(15.0)

    # Target moves to (10, 0) -> block point should follow to (25, 0)
    t2 = TargetTrackArray()
    t2.tracks = [make_track(10.0, 0.0, 1)]
    node.on_target_track(t2)
    assert node.tick()
    second = {int(c.drone_id): c for c in published[-1].commands}[101]
    assert second.layer == LAYER_BLOCK
    assert second.target_x == pytest.approx(25.0)
    assert second.enclosure_radius == pytest.approx(15.0)
