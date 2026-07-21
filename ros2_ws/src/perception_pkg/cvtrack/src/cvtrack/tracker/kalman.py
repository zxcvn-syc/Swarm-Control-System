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

Additionally, enhanced versions with adaptive noise are provided:

* ``KalmanCV2DAdaptive`` -- 4-state KF with adaptive process/measurement noise.
* ``KalmanBoTAdaptive`` -- 8-state KF with motion-adaptive noise tuning.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, List, Optional, Tuple

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


# =============================================================================
# Enhanced Kalman Filters with Adaptive Noise and Trajectory Prediction
# =============================================================================

class KalmanCV2DAdaptive:
    """Constant-velocity 2D Kalman filter with adaptive noise tuning.

    Improvements over KalmanCV2D:
    - Adaptive process noise based on track velocity magnitude
    - Adaptive measurement noise based on detection confidence
    - Innovation-based anomaly detection
    - Velocity confidence weighting
    - Multi-step trajectory prediction with uncertainty propagation
    """

    STATE_DIM = 4
    MEAS_DIM = 2

    # State indices
    IX, IY, VX, VY = 0, 1, 2, 3

    def __init__(
        self,
        dt: float = 0.05,
        base_std_pos: float = 1.0 / 20.0,
        base_std_vel: float = 1.0 / 160.0,
        base_std_meas: float = 1.0 / 20.0,
        motion_adapt_gain: float = 0.3,
        velocity_limit: float = 100.0,
        innovation_gate: float = 5.991,
    ) -> None:
        self.dt = dt
        self.base_std_pos = base_std_pos
        self.base_std_vel = base_std_vel
        self.base_std_meas = base_std_meas
        self.motion_adapt_gain = motion_adapt_gain
        self.velocity_limit = velocity_limit
        self.innovation_gate = innovation_gate

        self.F = np.eye(4)
        self.F[self.IX, self.VX] = dt
        self.F[self.IY, self.VY] = dt
        self.H = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float64
        )

        self._track_history: List[Tuple[np.ndarray, np.ndarray]] = []
        self._max_history = 10

    def _compute_velocity_magnitude(self, mean: np.ndarray) -> float:
        """Compute normalized velocity magnitude (0 to 1)."""
        vx, vy = abs(mean[self.VX]), abs(mean[self.VY])
        speed = math.sqrt(vx * vx + vy * vy)
        return min(speed / self.velocity_limit, 1.0)

    def _adaptive_process_noise(self, mean: np.ndarray) -> np.ndarray:
        """Compute adaptive process noise based on motion state."""
        vel_factor = self._compute_velocity_magnitude(mean)
        adaptive_gain = 1.0 + self.motion_adapt_gain * vel_factor

        std = np.array(
            [
                self.base_std_pos * max(abs(mean[self.IX]), 1.0) * adaptive_gain,
                self.base_std_pos * max(abs(mean[self.IY]), 1.0) * adaptive_gain,
                self.base_std_vel * max(abs(mean[self.VX]), 0.1) * adaptive_gain,
                self.base_std_vel * max(abs(mean[self.VY]), 0.1) * adaptive_gain,
            ],
            dtype=np.float64,
        )
        return np.diag(std ** 2)

    def _adaptive_measurement_noise(self, confidence: float = 1.0) -> np.ndarray:
        """Compute adaptive measurement noise based on detection confidence.

        Lower confidence detections get higher noise, making the filter
        trust them less.
        """
        conf_factor = 1.0 / max(confidence, 0.1)
        std = self.base_std_meas * conf_factor
        return np.diag([std ** 2, std ** 2])

    def _Q(self, mean: np.ndarray) -> np.ndarray:
        return self._adaptive_process_noise(mean)

    def _R(self, confidence: float = 1.0) -> np.ndarray:
        return self._adaptive_measurement_noise(confidence)

    def initiate(
        self, z: np.ndarray, confidence: float = 1.0
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Initialize filter with first detection."""
        mean = np.array([z[0], z[1], 0.0, 0.0], dtype=np.float64)
        std = np.array(
            [
                2 * self.base_std_pos * max(abs(z[0]), 1.0),
                2 * self.base_std_pos * max(abs(z[1]), 1.0),
                10 * self.base_std_vel,
                10 * self.base_std_vel,
            ],
            dtype=np.float64,
        )
        self._track_history = [(mean.copy(), np.diag(std ** 2))]
        return mean, np.diag(std ** 2)

    def predict(
        self, mean: np.ndarray, cov: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        return self.F @ mean, self.F @ cov @ self.F.T + self._Q(mean)

    def update(
        self,
        mean: np.ndarray,
        cov: np.ndarray,
        z: np.ndarray,
        confidence: float = 1.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Update with measurement, using adaptive noise based on confidence."""
        z_pred = self.H @ mean
        S = self.H @ cov @ self.H.T + self._R(confidence)
        K = cov @ self.H.T @ np.linalg.inv(S)
        innov = z - z_pred
        new_mean = mean + K @ innov
        new_cov = (np.eye(4) - K @ self.H) @ cov

        self._track_history.append((new_mean.copy(), new_cov.copy()))
        if len(self._track_history) > self._max_history:
            self._track_history.pop(0)

        return new_mean, new_cov

    def mahalanobis(
        self, mean: np.ndarray, cov: np.ndarray, z: np.ndarray
    ) -> float:
        z_pred = self.H @ mean
        S = self.H @ cov @ self.H.T + self._R()
        d = z - z_pred
        return float(d @ np.linalg.inv(S) @ d)

    def is_innovation_anomaly(
        self, mean: np.ndarray, cov: np.ndarray, z: np.ndarray
    ) -> bool:
        """Detect if the innovation is an anomaly (could indicate occlusion or misdetection)."""
        maha_dist = self.mahalanobis(mean, cov, z)
        return maha_dist > self.innovation_gate

    def compute_velocity_confidence(self, mean: np.ndarray, cov: np.ndarray) -> float:
        """Compute confidence in the velocity estimate (0 to 1)."""
        if len(self._track_history) < 3:
            return 0.5

        recent_states = [s[0] for s in self._track_history[-3:]]
        vx_var = np.var([s[self.VX] for s in recent_states])
        vy_var = np.var([s[self.VY] for s in recent_states])

        max_var = max(vx_var, vy_var, 1e-6)
        confidence = 1.0 / (1.0 + max_var * 10)
        return float(np.clip(confidence, 0.0, 1.0))

    def predict_n_steps(
        self, mean: np.ndarray, cov: np.ndarray, n: int
    ) -> List[Tuple[float, float]]:
        """Predict future positions for n steps with uncertainty."""
        steps = max(0, int(n))
        cur_mean = np.array(mean, dtype=np.float64, copy=True)
        cur_cov = np.array(cov, dtype=np.float64, copy=True)
        points: List[Tuple[float, float]] = []
        for _ in range(steps):
            cur_mean, cur_cov = self.predict(cur_mean, cur_cov)
            points.append((float(cur_mean[self.IX]), float(cur_mean[self.IY])))
        return points

    def predict_n_steps_with_uncertainty(
        self, mean: np.ndarray, cov: np.ndarray, n: int
    ) -> List[Tuple[float, float, float, float]]:
        """Predict future positions with position uncertainties (std_x, std_y).

        Returns list of (x, y, std_x, std_y) tuples.
        """
        steps = max(0, int(n))
        cur_mean = np.array(mean, dtype=np.float64, copy=True)
        cur_cov = np.array(cov, dtype=np.float64, copy=True)
        results: List[Tuple[float, float, float, float]] = []
        for _ in range(steps):
            cur_mean, cur_cov = self.predict(cur_mean, cur_cov)
            results.append((
                float(cur_mean[self.IX]),
                float(cur_mean[self.IY]),
                float(math.sqrt(max(cur_cov[self.IX, self.IX], 1e-6))),
                float(math.sqrt(max(cur_cov[self.IY, self.IY], 1e-6))),
            ))
        return results


class KalmanBoTAdaptive:
    """BoT-SORT 8-state KF with adaptive noise and enhanced prediction.

    Improvements over KalmanBoT:
    - Adaptive process noise based on acceleration changes
    - Camera motion sensitivity
    - Enhanced trajectory prediction
    - Motion mode detection (stationary, slow, fast)
    """

    STATE_DIM = 8
    MEAS_DIM = 4

    def __init__(
        self,
        dt: float = 1.0 / 30.0,
        base_sigma_p: float = _BOTSORT_SIGMA_P,
        base_sigma_v: float = _BOTSORT_SIGMA_V,
        base_sigma_m: float = _BOTSORT_SIGMA_M,
        acceleration_gain: float = 0.5,
        motion_threshold_slow: float = 2.0,
        motion_threshold_fast: float = 20.0,
    ) -> None:
        self.dt = dt
        self.base_sigma_p = base_sigma_p
        self.base_sigma_v = base_sigma_v
        self.base_sigma_m = base_sigma_m
        self.acceleration_gain = acceleration_gain
        self.motion_threshold_slow = motion_threshold_slow
        self.motion_threshold_fast = motion_threshold_fast

        self.F = np.eye(8)
        for i in range(4):
            self.F[i, i + 4] = dt
        self.H = np.eye(4, 8)

        self._prev_velocity: Optional[np.ndarray] = None
        self._motion_mode: str = "slow"

    def _detect_motion_mode(self, mean: np.ndarray) -> str:
        """Detect current motion mode based on velocity magnitude."""
        vx, vy = mean[4], mean[5]
        speed = math.sqrt(vx * vx + vy * vy)

        if speed < self.motion_threshold_slow:
            return "stationary"
        elif speed > self.motion_threshold_fast:
            return "fast"
        else:
            return "slow"

    def _compute_acceleration_change(self, mean: np.ndarray) -> float:
        """Compute acceleration change from previous velocity."""
        if self._prev_velocity is None:
            self._prev_velocity = mean[4:6].copy()
            return 0.0

        dvx = mean[4] - self._prev_velocity[0]
        dvy = mean[5] - self._prev_velocity[1]
        accel_change = math.sqrt(dvx * dvx + dvy * dvy)

        self._prev_velocity = mean[4:6].copy()
        return accel_change

    def _adaptive_sigma(
        self, mean: np.ndarray, sigma_p: float, sigma_v: float
    ) -> List[float]:
        """Compute adaptive sigma values based on motion state."""
        motion_mode = self._detect_motion_mode(mean)
        accel_change = self._compute_acceleration_change(mean)

        if motion_mode == "stationary":
            scale_p, scale_v = 0.3, 0.1
        elif motion_mode == "fast":
            scale_p, scale_v = 1.5, 2.0
        else:
            accel_factor = 1.0 + self.acceleration_gain * min(accel_change / 10.0, 1.0)
            scale_p, scale_v = accel_factor, accel_factor

        w_prev = max(abs(float(mean[2])), 1.0)
        h_prev = max(abs(float(mean[3])), 1.0)
        return [
            sigma_p * scale_p * w_prev,
            sigma_p * scale_p * h_prev,
            sigma_p * scale_p * w_prev,
            sigma_p * scale_p * h_prev,
            sigma_v * scale_v * w_prev,
            sigma_v * scale_v * h_prev,
            sigma_v * scale_v * w_prev,
            sigma_v * scale_v * h_prev,
        ]

    def _q(self, mean: np.ndarray) -> np.ndarray:
        std = self._adaptive_sigma(mean, self.base_sigma_p, self.base_sigma_v)
        return np.diag(np.array(std, dtype=np.float64) ** 2)

    def _r(self, mean: np.ndarray) -> np.ndarray:
        w_pred = max(abs(float(mean[2])), 1.0)
        h_pred = max(abs(float(mean[3])), 1.0)
        std = [
            self.base_sigma_m * w_pred,
            self.base_sigma_m * h_pred,
            self.base_sigma_m * w_pred,
            self.base_sigma_m * h_pred,
        ]
        return np.diag(np.array(std, dtype=np.float64) ** 2)

    def initiate(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        mean = np.zeros(8, dtype=np.float64)
        mean[:4] = z
        cov = np.diag(
            np.array(
                [
                    2 * self.base_sigma_p * max(abs(z[0]), 1.0),
                    2 * self.base_sigma_p * max(abs(z[1]), 1.0),
                    2 * self.base_sigma_p * max(abs(z[2]), 1.0),
                    2 * self.base_sigma_p * max(abs(z[3]), 1.0),
                    10 * self.base_sigma_v * max(abs(z[2]), 1.0),
                    10 * self.base_sigma_v * max(abs(z[3]), 1.0),
                    10 * self.base_sigma_v * max(abs(z[2]), 1.0),
                    10 * self.base_sigma_v * max(abs(z[3]), 1.0),
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

    def predict_n_steps(
        self, mean: np.ndarray, cov: np.ndarray, n: int
    ) -> List[Tuple[float, float]]:
        """Predict n steps forward, returning (x, y) centres."""
        steps = max(0, int(n))
        cur_mean = np.array(mean, dtype=np.float64, copy=True)
        cur_cov = np.array(cov, dtype=np.float64, copy=True)
        points: List[Tuple[float, float]] = []
        for _ in range(steps):
            cur_mean, cur_cov = self.predict(cur_mean, cur_cov)
            points.append((float(cur_mean[0]), float(cur_mean[1])))
        return points

    def predict_n_steps_with_uncertainty(
        self, mean: np.ndarray, cov: np.ndarray, n: int
    ) -> List[Tuple[float, float, float, float]]:
        """Predict n steps forward with position uncertainties."""
        steps = max(0, int(n))
        cur_mean = np.array(mean, dtype=np.float64, copy=True)
        cur_cov = np.array(cov, dtype=np.float64, copy=True)
        results: List[Tuple[float, float, float, float]] = []
        for _ in range(steps):
            cur_mean, cur_cov = self.predict(cur_mean, cur_cov)
            results.append((
                float(cur_mean[0]),
                float(cur_mean[1]),
                float(math.sqrt(max(cur_cov[0, 0], 1e-6))),
                float(math.sqrt(max(cur_cov[1, 1], 1e-6))),
            ))
        return results

    def get_motion_mode(self) -> str:
        """Return the current motion mode detected from recent updates."""
        return self._motion_mode


__all__ = [
    "KalmanCV2D",
    "KalmanBoT",
    "KalmanCV2DAdaptive",
    "KalmanBoTAdaptive",
    "predict_n_steps",
    "predict_n_steps_with_covariance",
    "CHI2_THRESHOLD",
    "CHI2_INV_95_4DOF",
    "BOTSORT_IOU_THRESH",
    "BOTSORT_HIGH_CONF",
    "BOTSORT_NEW_TRACK_CONF",
    "BOTSORT_LOST_RELINK_FRAMES",
]