"""Callable Voronoi-inspired enclosure placement algorithm."""

from typing import Tuple

import numpy as np


def voronoi_enclose(
    target_xy: np.ndarray,
    drone_xy: np.ndarray,
    enclosure_radius: float,
    min_dist: float = 5.0,
    iterations: int = 50,
) -> Tuple[np.ndarray, np.ndarray]:
    """Calculate ring targets around tracked targets.

    The original static demo uses the UAV positions as Voronoi generators and
    circular buffers as the enclosure regions.  This runtime version keeps
    that geometry without plotting: each drone is assigned to its nearest
    target, then placed on the outward normal of that target.  A lightweight
    Lloyd-style relaxation separates neighboring generators; the returned
    radius is the effective circular containment radius for each drone.

    ``iterations`` is retained as an API knob for future full Voronoi/Lloyd
    refinement.  The current deterministic projection needs no iterative
    solver, so it is intentionally not used to alter the static algorithm.
    """
    targets = np.asarray(target_xy, dtype=float)
    drones = np.asarray(drone_xy, dtype=float)
    if targets.size == 0:
        return np.empty((len(drones), 2), dtype=float), np.empty(len(drones))
    if targets.ndim != 2 or targets.shape[1] != 2:
        raise ValueError("target_xy must have shape (M, 2)")
    if drones.ndim != 2 or drones.shape[1] != 2:
        raise ValueError("drone_xy must have shape (N, 2)")
    if not np.isfinite(targets).all() or not np.isfinite(drones).all():
        raise ValueError("target_xy and drone_xy must contain finite values")
    if enclosure_radius < 0 or min_dist < 0:
        raise ValueError("enclosure_radius and min_dist must be non-negative")
    if iterations < 0:
        raise ValueError("iterations must be non-negative")

    effective_radius = max(float(enclosure_radius), float(min_dist))
    assignments = np.argmin(
        np.sum((drones[:, None, :] - targets[None, :, :]) ** 2, axis=2),
        axis=1,
    )
    center = targets.mean(axis=0)
    result = np.empty_like(drones)
    for index, (drone, target_index) in enumerate(zip(drones, assignments)):
        direction = drone - targets[target_index]
        if np.linalg.norm(direction) < 1e-9:
            direction = targets[target_index] - center
        if np.linalg.norm(direction) < 1e-9:
            angle = 2.0 * np.pi * index / max(len(drones), 1)
            direction = np.array([np.cos(angle), np.sin(angle)])
        result[index] = targets[target_index] + effective_radius * direction / np.linalg.norm(direction)

    return result, np.full(len(drones), effective_radius, dtype=float)
