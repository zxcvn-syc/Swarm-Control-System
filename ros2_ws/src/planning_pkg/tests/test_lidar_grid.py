import numpy as np
import pytest

from planning_pkg.lidar_grid import GridGeometry, inflate_occupied, rasterize_scan


def test_geometry_uses_occupancy_grid_cell_centers():
    geometry = GridGeometry(10, 8, 0.5, -2.0, 1.0, "world")

    assert geometry.world_to_cell(-2.0, 1.0) == (0, 0)
    assert geometry.world_to_cell(2.999, 4.999) == (9, 7)
    assert geometry.world_to_cell(3.0, 5.0) is None
    assert geometry.cell_to_world(3, 2) == pytest.approx((-0.25, 2.25))
    assert geometry.clamp_world_to_cell(99.0, -99.0) == (9, 0)


def test_scan_rasterization_marks_hits_free_space_and_unknown_cells():
    geometry = GridGeometry(20, 20, 1.0, -10.0, -10.0)
    grid = rasterize_scan(
        [3.0, np.inf, np.nan],
        angle_min=0.0,
        angle_increment=np.pi / 2.0,
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_yaw=0.0,
        min_range=0.1,
        max_range=5.0,
        geometry=geometry,
    )

    assert grid[10, 10] == 0  # sensor cell
    assert grid[10, 11] == 0  # free ray before the return
    assert grid[10, 13] == 100  # finite endpoint
    assert grid[15, 10] == 0  # positive infinity clears to max range
    assert grid[0, 0] == -1  # no beam observed this cell


def test_inflation_preserves_unknown_and_expands_in_metric_radius():
    geometry = GridGeometry(9, 9, 0.5, 0.0, 0.0)
    source = np.full((9, 9), -1, dtype=np.int8)
    source[4, 4] = 100

    inflated = inflate_occupied(source, geometry, 0.75)

    assert inflated[4, 4] == 100
    assert inflated[4, 5] == 100
    assert inflated[5, 5] == 100
    assert inflated[4, 6] == -1
