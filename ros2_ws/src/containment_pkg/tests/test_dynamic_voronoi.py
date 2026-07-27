import numpy as np

from containment_pkg.voronoi import voronoi_enclose


def test_static_voronoi_is_deterministic():
    target_xy = np.array([[0.0, 0.0]])
    drone_xy = np.array([[10.0, 0.0], [0.0, 10.0]])
    first, first_radii = voronoi_enclose(target_xy, drone_xy, 25.0, min_dist=5.0)
    second, second_radii = voronoi_enclose(target_xy, drone_xy, 25.0, min_dist=5.0)
    assert np.allclose(first, second)
    assert np.allclose(first_radii, second_radii)


def test_target_motion_changes_enclosure_region():
    drones = np.array([[20.0, 0.0], [-20.0, 0.0]])
    before, _ = voronoi_enclose(np.array([[0.0, 0.0]]), drones, 25.0)
    after, _ = voronoi_enclose(np.array([[10.0, 0.0]]), drones, 25.0)
    assert not np.allclose(before, after)
    assert np.isclose(np.mean(after[:, 0]) - np.mean(before[:, 0]), 10.0)


def test_min_dist_is_enforced():
    _, radii = voronoi_enclose(np.array([[0.0, 0.0]]), np.array([[1.0, 0.0]]), 2.0, min_dist=5.0)
    assert np.allclose(radii, 5.0)
