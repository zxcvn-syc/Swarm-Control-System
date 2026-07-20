"""Kalman filters used by the tracker (extracted from the v4 monolith).

Two implementations live here:

* KalmanCV2D  -- 4-state constant-velocity KF (cx, cy, vx, vy).  This is the
                 "DeepSortLite" tracker (kept for backward compatibility).
* KalmanBoT   -- 8-state BoT-SORT KF (cx, cy, w, h, vx, vy, vw, vh).

Both filters are intentionally *behaviour-preserving* with respect to the v4
script: every magic number in Q / R and the BoT-SORT sigma constants is
identical to the original.

The module also exposes two trajectory-projection helpers used by the
renderer:

* ``predict_n_steps`` -- returns the predicted ``(x, y)`` centres only.
* ``predict_n_steps_with_covariance`` -- returns ``(mean, cov)`` at every
  step so callers can render uncertainty ellipses.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, List, Tuple

import numpy as np


if TYPE_CHECKING:  # pragma: no cover
    from cvtrack.types import Track


CHI2_THRESHOLD = 5.991  # p=0.95, 2 dof (legacy DeepSORT gate)
# DeepSORT paper uses chi2_95 with 4 dof as the gating threshold (it
# parameterises the state with cx, cy, a, h so 4-D).  The 4-state KF here
# is only 2-D in the position space, so we expose both constants.
CHI2_INV_95_4DOF = 9.4877

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
# Trajectory projection helpers (used by the renderer for future trails).
# ----------------------------------------------------------------------------
def predict_n_steps(kf: Any, track: "Track", n: int) -> List[Tuple[float, float]]:
    """Project a track forward without mutating its live Kalman state.

    Walks the KF forward ``n`` times and returns the predicted ``(x, y)``
    centres.  Pure function over the supplied state.
    """
    steps = max(0, int(n))
    mean = np.array(track.mean, dtype=np.float64, copy=True)
    cov = np.array(track.cov, dtype=np.float64, copy=True)
    points: List[Tuple[float, float]] = []
    for _ in range(steps):
        mean, cov = kf.predict(mean, cov)
        points.append((float(mean[0]), float(mean[1])))
    return points


def predict_n_steps_with_covariance(
    kf: Any,
    track_mean: np.ndarray,
    track_cov: np.ndarray,
    n: int,
) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Like :func:`predict_n_steps` but also returns the full state covariance.

    Each element of the returned list is ``(mean_step, cov_step)``; the
    position sub-block is ``mean_step[:2]`` and the position-position
    sub-block is ``cov_step[:2, :2]`` (true for both the 4-state KF and
    the 8-state BoT-SORT KF -- the first two state entries are always
    ``cx, cy``).

    The ``kf`` object only needs to expose ``predict(mean, cov) ->
    (mean, cov)``; no mutation of any track state is performed.
    """
    steps = max(0, int(n))
    mean = np.array(track_mean, dtype=np.float64, copy=True)
    cov = np.array(track_cov, dtype=np.float64, copy=True)
    out: List[Tuple[np.ndarray, np.ndarray]] = []
    for _ in range(steps):
        mean, cov = kf.predict(mean, cov)
        out.append((mean.copy(), cov.copy()))
    return out


# ----------------------------------------------------------------------------
# Default BoT-SORT hyperparameters (paper-style with v4 calibrated values)
# ----------------------------------------------------------------------------
BOTSORT_IOU_THRESH = 0.30
BOTSORT_HIGH_CONF = 0.35
BOTSORT_NEW_TRACK_CONF = 0.20
BOTSORT_LOST_RELINK_FRAMES = 30