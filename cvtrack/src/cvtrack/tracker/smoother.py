"""Rauch-Tung-Striebel backward smoother for trajectory post-processing.

Purely cosmetic: the runtime tracker still uses the forward KF.  The smoother
is applied to a track's ``pred_trail`` after the run to produce the values
written to ``tracks_smoothed.csv`` and used by the trajectory plot.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np


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
    """
    n = len(positions)
    if n == 0:
        return []
    if n == 1:
        return [(float(positions[0][0]), float(positions[0][1]))]

    F = np.eye(4)
    F[0, 2] = 1.0
    F[1, 3] = 1.0
    H = np.eye(4)[:2, :]
    q = float(process_var)
    r = float(meas_var)
    Q = np.diag([q, q, q, q])
    R = np.diag([r, r])

    means_f = np.zeros((n, 4))
    covs_f = np.zeros((n, 4, 4))
    means_p = np.zeros((n, 4))
    covs_p = np.zeros((n, 4, 4))

    z0 = np.array([positions[0][0], positions[0][1], 0.0, 0.0], dtype=np.float64)
    means_f[0] = z0
    covs_f[0] = np.diag([2 * r, 2 * r, 10 * q, 10 * q])

    for k in range(1, n):
        means_p[k] = F @ means_f[k - 1]
        covs_p[k] = F @ covs_f[k - 1] @ F.T + Q
        z = np.array([positions[k][0], positions[k][1]], dtype=np.float64)
        z_pred = H @ means_p[k]
        S = H @ covs_p[k] @ H.T + R
        K = covs_p[k] @ H.T @ np.linalg.inv(S)
        means_f[k] = means_p[k] + K @ (z - z_pred)
        covs_f[k] = (np.eye(4) - K @ H) @ covs_p[k]

    smoothed = np.zeros((n, 4))
    smoothed[-1] = means_f[-1]
    for k in range(n - 2, -1, -1):
        try:
            cov_p_inv = np.linalg.inv(covs_p[k + 1])
        except np.linalg.LinAlgError:
            smoothed[k] = means_f[k]
            continue
        C = covs_f[k] @ F.T @ cov_p_inv
        smoothed[k] = means_f[k] + C @ (smoothed[k + 1] - means_p[k + 1])

    return [(float(smoothed[i, 0]), float(smoothed[i, 1])) for i in range(n)]