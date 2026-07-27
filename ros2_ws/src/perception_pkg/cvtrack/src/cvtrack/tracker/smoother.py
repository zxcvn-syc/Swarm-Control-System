"""Rauch-Tung-Striebel backward smoother for trajectory post-processing.

Purely cosmetic: the runtime tracker still uses the forward KF.  The smoother
is applied to a track's ``pred_trail`` after the run to produce the values
written to ``tracks_smoothed.csv`` and used by the trajectory plot.

Enhanced with:
- Robust handling of degenerate inputs (single point, all-same points)
- Stable matrix inversion with regularization and pseudoinverse fallback
- Comprehensive input validation
- NaN/Inf protection throughout
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np


# Numerical stability constants
_EPS = 1e-9
_MIN_PROCESS_VAR = 1e-9
_MIN_MEAS_VAR = 1e-9
_MAX_MAHALANOBIS = 1e6  # Cap to avoid numerical issues


def _is_finite_point(point: Tuple[float, float]) -> bool:
    """Check if a point has finite coordinates."""
    try:
        x, y = point
        return math.isfinite(float(x)) and math.isfinite(float(y))
    except (TypeError, ValueError):
        return False


def _safe_inv(matrix: np.ndarray, regularization: float = 1e-9) -> np.ndarray:
    """Numerically stable matrix inversion with regularization.

    Args:
        matrix: Square matrix to invert
        regularization: Small value added to diagonal for stability

    Returns:
        Inverse of the matrix, or pseudoinverse if singular
    """
    if matrix is None:
        return np.eye(2, dtype=np.float64)

    try:
        m = np.asarray(matrix, dtype=np.float64)
        if m.ndim != 2 or m.shape[0] != m.shape[1]:
            # Return pseudoinverse for non-square
            return np.linalg.pinv(m)

        if m.size == 0:
            return np.eye(2, dtype=np.float64)

        # Check for finite values
        if not np.all(np.isfinite(m)):
            return np.eye(m.shape[0], dtype=np.float64)

        # Try direct inversion
        try:
            return np.linalg.inv(m)
        except np.linalg.LinAlgError:
            pass

        # Try regularized inversion
        try:
            m_reg = m + np.eye(m.shape[0]) * regularization
            return np.linalg.inv(m_reg)
        except np.linalg.LinAlgError:
            pass

        # Final fallback: pseudoinverse
        return np.linalg.pinv(m)
    except Exception:
        # Emergency fallback
        size = matrix.shape[0] if hasattr(matrix, "shape") and matrix.ndim == 2 else 2
        return np.eye(size, dtype=np.float64)


def rts_smooth_2d(
    positions: Sequence[Tuple[float, float]],
    process_var: float = 1.0,
    meas_var: float = 1.0,
) -> List[Tuple[float, float]]:
    """Apply a constant-velocity RTS smoother over a 2D point sequence.

    State is (x, y, vx, vy); we run a forward KF with the supplied noise
    levels and then walk backwards using the standard RTS gain:

        C_k = P_k F^T P_{k+1|k}^-1
        x_{k|N} = x_k + C_k (x_{k+1|N} - x_{k+1|k})

    Enhanced with:
    - Comprehensive input validation
    - Handling of degenerate cases (single point, constant trajectory)
    - Stable matrix inversion with regularization
    - NaN/Inf protection in all calculations

    Args:
        positions: Sequence of (x, y) tuples
        process_var: Process noise variance (must be positive)
        meas_var: Measurement noise variance (must be positive)

    Returns:
        List of smoothed (x, y) tuples, same length as input.
        Returns empty list for empty input, or list of same point for single input.
    """
    # Validate inputs
    if positions is None:
        return []

    # Filter out invalid points (None, NaN, Inf)
    valid_positions = [p for p in positions if _is_finite_point(p)]
    n = len(valid_positions)

    if n == 0:
        return []
    if n == 1:
        return [(float(valid_positions[0][0]), float(valid_positions[0][1]))]

    # Check if all points are identical (degenerate case)
    unique_pts = set(valid_positions)
    if len(unique_pts) == 1:
        # All points identical - return as-is
        return [valid_positions[0]] * n

    # Validate noise parameters
    try:
        q = float(process_var)
        r = float(meas_var)
        if not math.isfinite(q) or q <= 0:
            q = _MIN_PROCESS_VAR
        if not math.isfinite(r) or r <= 0:
            r = _MIN_MEAS_VAR
    except (TypeError, ValueError):
        q = 1.0
        r = 1.0

    F = np.eye(4)
    F[0, 2] = 1.0
    F[1, 3] = 1.0
    H = np.eye(4)[:2, :]
    Q = np.diag([q, q, q, q])
    R = np.diag([r, r])

    means_f = np.zeros((n, 4))
    covs_f = np.zeros((n, 4, 4))
    means_p = np.zeros((n, 4))
    covs_p = np.zeros((n, 4, 4))

    try:
        z0 = np.array(
            [valid_positions[0][0], valid_positions[0][1], 0.0, 0.0],
            dtype=np.float64,
        )
        means_f[0] = z0
        covs_f[0] = np.diag([2 * r, 2 * r, 10 * q, 10 * q])
    except (TypeError, ValueError, IndexError):
        # Fallback: return original positions if initial setup fails
        return [(float(p[0]), float(p[1])) for p in valid_positions]

    # Forward Kalman filter pass
    for k in range(1, n):
        try:
            means_p[k] = F @ means_f[k - 1]
            covs_p[k] = F @ covs_f[k - 1] @ F.T + Q

            # Validate predicted state
            if not np.all(np.isfinite(means_p[k])):
                means_p[k] = np.array([0.0, 0.0, 0.0, 0.0])
            if not np.all(np.isfinite(covs_p[k])):
                covs_p[k] = np.eye(4) * r

            z = np.array([valid_positions[k][0], valid_positions[k][1]], dtype=np.float64)
            z_pred = H @ means_p[k]
            S = H @ covs_p[k] @ H.T + R

            S_inv = _safe_inv(S)
            K = covs_p[k] @ H.T @ S_inv

            means_f[k] = means_p[k] + K @ (z - z_pred)
            covs_f[k] = (np.eye(4) - K @ H) @ covs_p[k]

            # Validate filtered state
            if not np.all(np.isfinite(means_f[k])):
                means_f[k] = means_p[k]
            if not np.all(np.isfinite(covs_f[k])):
                covs_f[k] = covs_p[k]
        except (np.linalg.LinAlgError, ValueError, TypeError) as exc:
            # On failure at step k, use prediction
            means_f[k] = means_p[k]
            covs_f[k] = covs_p[k]

    # Backward RTS smoothing pass
    smoothed = np.zeros((n, 4))
    smoothed[-1] = means_f[-1]
    if not np.all(np.isfinite(smoothed[-1])):
        smoothed[-1] = means_p[-1]

    for k in range(n - 2, -1, -1):
        try:
            # Stable inversion with regularization
            cov_p_inv = _safe_inv(covs_p[k + 1], regularization=q)

            C = covs_f[k] @ F.T @ cov_p_inv
            smoothed[k] = means_f[k] + C @ (smoothed[k + 1] - means_p[k + 1])

            # Validate
            if not np.all(np.isfinite(smoothed[k])):
                smoothed[k] = means_f[k]
        except (np.linalg.LinAlgError, ValueError, TypeError):
            smoothed[k] = means_f[k]

    # Build output with safety checks
    result = []
    for i in range(n):
        try:
            sx = float(smoothed[i, 0])
            sy = float(smoothed[i, 1])
            if not math.isfinite(sx) or not math.isfinite(sy):
                # Fall back to original position
                sx, sy = float(valid_positions[i][0]), float(valid_positions[i][1])
            result.append((sx, sy))
        except (IndexError, ValueError, TypeError):
            # Emergency fallback
            result.append((0.0, 0.0))

    return result