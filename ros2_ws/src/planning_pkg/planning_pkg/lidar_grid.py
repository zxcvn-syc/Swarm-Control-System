"""Pure geometry and scan-rasterization helpers for 2D LiDAR maps.

The ROS2 adapter in :mod:`planning_pkg.lidar_grid_node` uses this module, but
the transforms and occupancy construction deliberately have no ROS imports so
they can be unit-tested on a development machine without a ROS installation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


Cell = Tuple[int, int]


@dataclass(frozen=True)
class GridGeometry:
    """Axis-aligned OccupancyGrid geometry in a world-coordinate frame."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = "world"

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid width and height must be positive")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("grid resolution must be finite and positive")
        if not math.isfinite(self.origin_x) or not math.isfinite(self.origin_y):
            raise ValueError("grid origin must be finite")
        if not self.frame_id:
            raise ValueError("grid frame_id must not be empty")

    def contains_cell(self, cell: Cell) -> bool:
        x, y = int(cell[0]), int(cell[1])
        return 0 <= x < self.width and 0 <= y < self.height

    def world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        """Return the cell containing a world point, or ``None`` if outside."""
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        cell_x = int(math.floor((x - self.origin_x) / self.resolution))
        cell_y = int(math.floor((y - self.origin_y) / self.resolution))
        cell = (cell_x, cell_y)
        return cell if self.contains_cell(cell) else None

    def clamp_world_to_cell(self, x: float, y: float) -> Cell:
        """Return the nearest in-map cell for a finite world coordinate."""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("world coordinate must be finite")
        raw_x = int(math.floor((x - self.origin_x) / self.resolution))
        raw_y = int(math.floor((y - self.origin_y) / self.resolution))
        return (
            min(max(raw_x, 0), self.width - 1),
            min(max(raw_y, 0), self.height - 1),
        )

    def cell_to_world(self, cell_x: int, cell_y: int) -> Tuple[float, float]:
        """Return the world coordinate at the center of an in-map cell."""
        cell = (int(cell_x), int(cell_y))
        if not self.contains_cell(cell):
            raise ValueError(f"cell outside grid: {cell}")
        return (
            self.origin_x + (cell[0] + 0.5) * self.resolution,
            self.origin_y + (cell[1] + 0.5) * self.resolution,
        )


def rasterize_scan(
    ranges: Sequence[float] | np.ndarray,
    *,
    angle_min: float,
    angle_increment: float,
    sensor_x: float,
    sensor_y: float,
    sensor_yaw: float,
    min_range: float,
    max_range: float,
    geometry: GridGeometry,
) -> np.ndarray:
    """Build a local occupancy grid from one horizontal-plane laser scan.

    Cells not traversed by any beam are ``-1`` (unknown), beam paths are
    ``0`` (free), and finite returns are ``100`` (occupied).  Positive infinity
    is interpreted as a no-return beam and clears cells through ``max_range``.
    Obstacle endpoints always win over free-space samples from another beam.
    """
    if not all(
        math.isfinite(value)
        for value in (angle_min, angle_increment, sensor_x, sensor_y, sensor_yaw)
    ):
        raise ValueError("scan angles and sensor pose must be finite")
    if not math.isfinite(min_range) or min_range < 0.0:
        raise ValueError("min_range must be finite and non-negative")
    if not math.isfinite(max_range) or max_range <= min_range:
        raise ValueError("max_range must be finite and greater than min_range")

    occupancy = np.full((geometry.height, geometry.width), -1, dtype=np.int8)
    hit_cells: set[Cell] = set()
    values = np.asarray(ranges, dtype=float).reshape(-1)
    sample_step = max(geometry.resolution * 0.5, 0.01)

    for index, measured_range in enumerate(values):
        if math.isnan(measured_range) or measured_range <= 0.0:
            continue
        hit = math.isfinite(measured_range) and measured_range <= max_range
        distance = min(float(measured_range), max_range)
        if distance < min_range:
            continue

        bearing = sensor_yaw + angle_min + index * angle_increment
        end_x = sensor_x + distance * math.cos(bearing)
        end_y = sensor_y + distance * math.sin(bearing)
        _mark_free_ray(
            occupancy,
            geometry,
            sensor_x,
            sensor_y,
            end_x,
            end_y,
            sample_step,
        )
        if hit:
            endpoint = geometry.world_to_cell(end_x, end_y)
            if endpoint is not None:
                hit_cells.add(endpoint)

    for cell_x, cell_y in hit_cells:
        occupancy[cell_y, cell_x] = 100
    return occupancy


def inflate_occupied(
    occupancy: np.ndarray,
    geometry: GridGeometry,
    inflation_radius: float,
) -> np.ndarray:
    """Inflate occupied cells while preserving free/unknown cell semantics."""
    source = np.asarray(occupancy, dtype=np.int8)
    if source.shape != (geometry.height, geometry.width):
        raise ValueError(
            "occupancy shape does not match geometry: "
            f"{source.shape} != {(geometry.height, geometry.width)}"
        )
    if not math.isfinite(inflation_radius) or inflation_radius < 0.0:
        raise ValueError("inflation_radius must be finite and non-negative")

    inflated = source.copy()
    radius_cells = int(math.ceil(inflation_radius / geometry.resolution))
    if radius_cells == 0:
        return inflated

    offsets: Iterable[Tuple[int, int]] = (
        (dx, dy)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if math.hypot(dx, dy) * geometry.resolution <= inflation_radius + 1e-12
    )
    offsets = tuple(offsets)
    for cell_y, cell_x in np.argwhere(source >= 100):
        for dx, dy in offsets:
            nx, ny = int(cell_x) + dx, int(cell_y) + dy
            if geometry.contains_cell((nx, ny)):
                inflated[ny, nx] = 100
    return inflated


def _mark_free_ray(
    occupancy: np.ndarray,
    geometry: GridGeometry,
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    sample_step: float,
) -> None:
    """Sample an axis-aligned map ray densely enough to cover all cells."""
    distance = math.hypot(end_x - start_x, end_y - start_y)
    steps = max(1, int(math.ceil(distance / sample_step)))
    for step in range(steps + 1):
        ratio = step / steps
        cell = geometry.world_to_cell(
            start_x + ratio * (end_x - start_x),
            start_y + ratio * (end_y - start_y),
        )
        if cell is not None:
            occupancy[cell[1], cell[0]] = 0

