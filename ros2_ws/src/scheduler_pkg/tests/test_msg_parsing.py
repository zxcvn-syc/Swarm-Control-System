"""Message parsing tests using ROS-shaped lightweight message objects."""

from types import SimpleNamespace

from scheduler_pkg.scheduler_node import parse_drones, parse_targets, target_priority


def test_target_track_array_parsing_preserves_ids_coordinates_and_priority():
    tracks = [
        SimpleNamespace(target_id=11, x=12.5, y=-3.0, confidence=0.3, is_confirmed=False),
        SimpleNamespace(target_id=42, x=8.0, y=9.5, confidence=0.95, is_confirmed=True),
    ]
    parsed = parse_targets(SimpleNamespace(tracks=tracks))

    assert parsed[11] == (12.5, -3.0, 0.3)
    assert parsed[42] == (8.0, 9.5, 1.0)
    assert target_priority(tracks[1]) > target_priority(tracks[0])


def test_drone_state_array_parsing_drops_unavailable_drones():
    drones = [
        SimpleNamespace(drone_id=1, x=1.0, y=2.0, available=True, platform_type=0),
        SimpleNamespace(drone_id=2, x=3.0, y=4.0, available=False, platform_type=1),
        SimpleNamespace(drone_id=9, x=-1.0, y=0.0, available=True, platform_type=1),
    ]
    assert parse_drones(SimpleNamespace(drones=drones)) == {
        1: (1.0, 2.0, 0),
        9: (-1.0, 0.0, 1),
    }


def test_drone_state_array_parsing_defaults_to_uav_without_platform_type():
    """Legacy DroneState stubs without ``platform_type`` fall back to 0 (UAV)."""
    drones = [
        SimpleNamespace(drone_id=5, x=1.5, y=-2.5, available=True),
    ]
    assert parse_drones(SimpleNamespace(drones=drones)) == {
        5: (1.5, -2.5, 0),
    }
