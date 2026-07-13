"""Kalman filters used by the tracker (extracted from the v4 monolith).

Two implementations live here:

* KalmanCV2D  -- 4-state constant-velocity KF (cx, cy, vx, vy).  This is the
                 "DeepSortLite" tracker (kept for backward compatibility).
* KalmanBoT   -- 8-state BoT-SORT KF (cx, cy, w, h, vx, vy, vw, vh).

Both filters are intentionally *behaviour-preserving* with respect to the v4
script: every magic number in Q / R and the BoT-SORT sigma constants is
identical to the original.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np


CHI2_THRESHOLD = 5.991  # p=0.95, 2 dof (legacy DeepSORT gate)

# BoT-SORT noise factors (paper sec. 3.1, tuned for ~30 FPS).
_BOTSORT_SIGMA_P = 0.05
_BOTSORT_SIGMA_V = 0.00625
_BOTSORT_SIGMA_M = 0.05


class KalmanCV2D:
    """Constant-velocity 2D Kalman filter (cx, cy, vx, vy)."""

    STATE_DIM = 4
    MEAS_DIM = 2

    def __init__(self, dt: float = 0.05) -> None:
        self.dt = dt
        self.F = np.eye(4)
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.H = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float64
        )
        self._std_pos = 1.0 / 20.0
        self._std_vel = 1.0 / 160.0
        self._std_meas = 1.0 / 20.0

    def _Q(self, mean: np.ndarray) -> np.ndarray:
        std = np.array(
            [
                self._std_pos * max(abs(mean[0]), 1.0),
                self._std_pos * max(abs(mean[1]), 1.0),
                self._std_vel * max(abs(mean[2]), 0.1),
                self._std_vel * max(abs(mean[3]), 0.1),
            ],
            dtype=np.float64,
        )
        return np.diag(std ** 2)

    def _R(self) -> np.ndarray:
        return np.diag([self._std_meas ** 2, self._std_meas ** 2])

    def initiate(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        mean = np.array([z[0], z[1], 0.0, 0.0], dtype=np.float64)
        std = np.array(
            [
                2 * self._std_pos * max(abs(z[0]), 1.0),
                2 * self._std_pos * max(abs(z[1]), 1.0),
                10 * self._std_vel,
                10 * self._std_vel,
            ],
            dtype=np.float64,
        )
        return mean, np.diag(std ** 2)

    def predict(
        self, mean: np.ndarray, cov: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        return self.F @ mean, self.F @ cov @ self.F.T + self._Q(mean)

    def update(
        self, mean: np.ndarray, cov: np.ndarray, z: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        z_pred = self.H @ mean
        S = self.H @ cov @ self.H.T + self._R()
        K = cov @ self.H.T @ np.linalg.inv(S)
        innov = z - z_pred
        return mean + K @ innov, (np.eye(4) - K @ self.H) @ cov

    def mahalanobis(
        self, mean: np.ndarray, cov: np.ndarray, z: np.ndarray
    ) -> float:
        z_pred = self.H @ mean
        S = self.H @ cov @ self.H.T + self._R()
        d = z - z_pred
        return float(d @ np.linalg.inv(S) @ d)


class KalmanBoT:
    """BoT-SORT 8-state KF (cx, cy, w, h, vx, vy, vw, vh)."""

    STATE_DIM = 8
    MEAS_DIM = 4

    def __init__(self, dt: float = 1.0 / 30.0) -> None:
        self.dt = dt
        self.F = np.eye(8)
        for i in range(4):
            self.F[i, i + 4] = dt
        self.H = np.eye(4, 8)

    def _q(self, mean: np.ndarray) -> np.ndarray:
        w_prev = max(abs(float(mean[2])), 1.0)
        h_prev = max(abs(float(mean[3])), 1.0)
        std = [
            _BOTSORT_SIGMA_P * w_prev,
            _BOTSORT_SIGMA_P * h_prev,
            _BOTSORT_SIGMA_P * w_prev,
            _BOTSORT_SIGMA_P * h_prev,
            _BOTSORT_SIGMA_V * w_prev,
            _BOTSORT_SIGMA_V * h_prev,
            _BOTSORT_SIGMA_V * w_prev,
            _BOTSORT_SIGMA_V * h_prev,
        ]
        return np.diag(np.array(std, dtype=np.float64) ** 2)

    def _r(self, mean: np.ndarray) -> np.ndarray:
        w_pred = max(abs(float(mean[2])), 1.0)
        h_pred = max(abs(float(mean[3])), 1.0)
        std = [
            _BOTSORT_SIGMA_M * w_pred,
            _BOTSORT_SIGMA_M * h_pred,
            _BOTSORT_SIGMA_M * w_pred,
            _BOTSORT_SIGMA_M * h_pred,
        ]
        return np.diag(np.array(std, dtype=np.float64) ** 2)

    def initiate(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        mean = np.zeros(8, dtype=np.float64)
        mean[:4] = z
        cov = np.diag(
            np.array(
                [
                    2 * _BOTSORT_SIGMA_P * max(abs(z[0]), 1.0),
                    2 * _BOTSORT_SIGMA_P * max(abs(z[1]), 1.0),
                    2 * _BOTSORT_SIGMA_P * max(abs(z[2]), 1.0),
                    2 * _BOTSORT_SIGMA_P * max(abs(z[3]), 1.0),
                    10 * _BOTSORT_SIGMA_V * max(abs(z[2]), 1.0),
                    10 * _BOTSORT_SIGMA_V * max(abs(z[3]), 1.0),
                    10 * _BOTSORT_SIGMA_V * max(abs(z[2]), 1.0),
                    10 * _BOTSORT_SIGMA_V * max(abs(z[3]), 1.0),
                ],
                dtype=np.float64,
            )
            ** 2
        )
        return mean, cov

    def predict(
        self, mean: np.ndarray, cov: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        return self.F @ mean, self.F @ cov @ self.F.T + self._q(mean)

    def update(
        self, mean: np.ndarray, cov: np.ndarray, z: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        z_pred = self.H @ mean
        S = self.H @ cov @ self.H.T + self._r(mean)
        K = cov @ self.H.T @ np.linalg.inv(S)
        innov = z - z_pred
        return mean + K @ innov, (np.eye(8) - K @ self.H) @ cov

    def apply_affine(
        self, mean: np.ndarray, cov: np.ndarray, A: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Camera-motion compensation (BoT-SORT paper Eq. 7, 8)."""
        if A is None:
            return mean, cov
        M = A[:, :2]
        t = A[:, 2]
        block = np.zeros((8, 8), dtype=np.float64)
        for i in range(4):
            block[2 * i:2 * i + 2, 2 * i:2 * i + 2] = M
        T = np.zeros(8, dtype=np.float64)
        T[0:2] = t
        return block @ mean + T, block @ cov @ block.T


# ----------------------------------------------------------------------------
# Default BoT-SORT hyperparameters (paper-style with v4 calibrated values)
# ----------------------------------------------------------------------------
BOTSORT_IOU_THRESH = 0.30
BOTSORT_HIGH_CONF = 0.35
BOTSORT_NEW_TRACK_CONF = 0.20
BOTSORT_LOST_RELINK_FRAMES = 30