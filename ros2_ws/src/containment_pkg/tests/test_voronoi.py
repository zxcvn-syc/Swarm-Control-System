import numpy as np

from containment_pkg.voronoi import voronoi_enclose


def test_voronoi_enclose_basic():
    targets, radii = voronoi_enclose(
        np.array([[0.0, 0.0]]),
        np.array([[10.0, 0.0], [-10.0, 0.0], [0.0, 10.0], [0.0, -10.0]]),
        20.0,
    )
    assert targets.shape == (4, 2)
    assert radii.shape == (4,)
    assert np.allclose(np.linalg.norm(targets, axis=1), 20.0)


def test_voronoi_enclose_radius():
    _, radii = voronoi_enclose(np.array([[1.0, 2.0]]), np.array([[0.0, 0.0]]), 12.5, min_dist=2.0)
    assert radii[0] == 12.5


def test_voronoi_more_drones_than_targets():
    drone_targets, radii = voronoi_enclose(
        np.array([[0.0, 0.0], [20.0, 0.0]]),
        np.zeros((5, 2)),
        10.0,
    )
    assert drone_targets.shape == (5, 2)
    assert radii.shape == (5,)
