"""Lightweight cvtrack runner for live consumers (e.g. ROS2 tracker_node).

This module wraps the detector + tracker factories from the :mod:`cvtrack`
package and exposes a single :func:`process_frame` function that returns
the list of currently-confirmed :class:`~cvtrack.types.Track` objects for
the given BGR frame.

It deliberately does **not** touch any of cvtrack's writers (CSV, video,
trail JSON) so callers can repurpose the live tracking output for whatever
downstream protocol they want — ROS2 topics in the swarm project,
zero-mq sockets in a flight-stack integration, etc.

Usage::

    from cvtrack.runner import CvtrackRunner

    runner = CvtrackRunner.from_yaml("configs/drone.yaml")
    cap = cv2.VideoCapture(0)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        tracks = runner.step(frame)
        for t in tracks:
            print(t.track_id, t.pos, t.mean[2:4])

Vendored source-of-truth policy
-------------------------------
This file is the **only** runtime copy of ``cvtrack.runner`` consumed by
the Swarm-Control-System ROS2 perception node (``tracker_node.py``).
The independent reference checkout at
``/home/hhh/Downloads/cv_tracking_demo/`` is kept for documentation /
benchmarking only — ``tracker_node.py`` does **not** fall back to it
anymore.  The reference tree predates the adaptive-tracker integration
and the optimized YAML schema (``tracker.kalman`` + ``trajectory_prediction``)
that this vendored copy now handles.  See ``MIGRATION.md`` for context.

Supported tracker kinds (set via ``tracker.kind`` in YAML or
``overrides['tracker']['kind']`` in code)::

    botsort              - vanilla BoT-SORT (8-state KF, CMC)
    deepsort             - legacy DeepSORT-lite (4-state KF)
    deepsort_cascade     - true DeepSORT cascade matcher (4-state KF + ReID)
    botsort_adaptive     - BoT-SORT with KalmanBoTAdaptive + trajectory prediction
    deepsort_adaptive    - DeepSORT with KalmanCV2DAdaptive + trajectory prediction

The two ``*_adaptive`` variants are backed by
:mod:`cvtrack.tracker.adaptive_tracker` and consume the extra
``tracker.kalman`` and ``trajectory_prediction`` sections of the
optimized YAML.  They are the trackers the ROS2 node actually uses in
the swarm's optimized configuration.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from cvtrack.appearance.gallery import Gallery
from cvtrack.config import Config, load_config, merge_cli
from cvtrack.detector.factory import make_detector
from cvtrack.tracker.adaptive_tracker import BoTSortAdaptive, DeepSortAdaptive
from cvtrack.tracker.botsort import BoTSortTracker
from cvtrack.tracker.deepsort import DeepSortCascade, DeepSortLite
from cvtrack.types import Box, Track


log = logging.getLogger(__name__)


# Adaptive tracker kinds route through cvtrack.tracker.adaptive_tracker.
# Listed here so other modules (and the runner settings dataclass) can
# validate against the same set without re-typing the strings.
ADAPTIVE_KINDS = frozenset({"botsort_adaptive", "deepsort_adaptive"})
LEGACY_KINDS = frozenset({"botsort", "deepsort", "deepsort_cascade"})
ALL_KINDS = LEGACY_KINDS | ADAPTIVE_KINDS


@dataclass
class RunnerSettings:
    """Resolved settings for the runner.

    All fields have safe defaults so the runner can also be constructed
    by passing only a YAML preset.  See :meth:`CvtrackRunner.from_yaml`
    and :meth:`CvtrackRunner.from_overrides` for the typical entry
    points.
    """

    tracker_kind: str = "deepsort_cascade"
    detector_backend: str = "yolo"
    weights: str = ""
    device: str = "cpu"
    imgsz: int = 320
    conf: float = 0.15
    classes: List[int] = field(default_factory=lambda: [0, 2, 5, 7, 16])
    min_box_area: float = 200.0
    min_conf: float = 0.0
    nms_iou: float = 0.50
    dt: float = 0.05
    max_age: int = 30
    n_init: int = 3
    iou_thresh: float = 0.30
    high_conf: float = 0.35
    new_track_conf: float = 0.20
    lost_relink_frames: int = 30
    stationary_prune: bool = True
    use_appearance: bool = False
    appearance_weights: Optional[str] = None
    appearance_thresh: float = 0.5
    include_tentative: bool = False

    # Adaptive-tracker knobs (forwarded to KalmanBoTAdaptive /
    # KalmanCV2DAdaptive).  All default to None so the trackers fall back
    # to their own built-in defaults when the optimized YAML doesn't
    # override them.
    kalman_dt: Optional[float] = None
    sigma_p: Optional[float] = None
    sigma_v: Optional[float] = None
    sigma_m: Optional[float] = None
    acceleration_gain: Optional[float] = None
    motion_threshold_slow: Optional[float] = None
    motion_threshold_fast: Optional[float] = None
    base_std_pos: Optional[float] = None
    base_std_vel: Optional[float] = None
    base_std_meas: Optional[float] = None
    motion_adapt_gain: Optional[float] = None
    velocity_limit: Optional[float] = None
    innovation_gate: Optional[float] = None

    # Trajectory prediction knobs (forwarded to the adaptive trackers).
    enable_prediction: bool = True
    prediction_steps: int = 10
    prediction_confidence_decay: float = 0.9
    min_prediction_confidence: float = 0.1


class CvtrackRunner:
    """Runs detection + tracking on individual frames.

    The runner is single-threaded and not thread-safe.  Create one
    instance per input stream and call :meth:`step` sequentially.
    """

    def __init__(self, settings: RunnerSettings) -> None:
        self.settings = settings

        # Detector ----------------------------------------------------
        self.detector = make_detector(
            backend=settings.detector_backend,
            weights=settings.weights or None,
            device=settings.device,
            conf=settings.conf,
            classes=list(settings.classes),
            imgsz=settings.imgsz,
            min_box_area=settings.min_box_area,
            min_conf=settings.min_conf,
            nms_iou=settings.nms_iou,
        )

        # Tracker -----------------------------------------------------
        kind = settings.tracker_kind.lower()
        if kind == "botsort":
            self.tracker: Any = BoTSortTracker(
                dt=settings.dt,
                max_age=settings.max_age,
                n_init=settings.n_init,
                stationary_prune=settings.stationary_prune,
                use_cmc=True,
                iou_thresh=settings.iou_thresh,
                high_conf=settings.high_conf,
                new_track_conf=settings.new_track_conf,
                lost_relink_frames=settings.lost_relink_frames,
            )
        elif kind == "deepsort_cascade":
            self.tracker = DeepSortCascade(
                dt=settings.dt,
                max_age=settings.max_age,
                n_init=settings.n_init,
                stationary_prune=settings.stationary_prune,
                use_appearance=settings.use_appearance,
                appearance_thresh=settings.appearance_thresh,
                iou_thresh=settings.iou_thresh,
            )
        elif kind == "deepsort":
            self.tracker = DeepSortLite(
                dt=settings.dt,
                max_age=settings.max_age,
                n_init=settings.n_init,
                stationary_prune=settings.stationary_prune,
            )
        elif kind == "botsort_adaptive":
            self.tracker = BoTSortAdaptive(
                dt=settings.dt,
                max_age=settings.max_age,
                n_init=settings.n_init,
                stationary_prune=settings.stationary_prune,
                use_cmc=True,
                iou_thresh=settings.iou_thresh,
                high_conf=settings.high_conf,
                new_track_conf=settings.new_track_conf,
                lost_relink_frames=settings.lost_relink_frames,
                sigma_p=settings.sigma_p if settings.sigma_p is not None else 0.05,
                sigma_v=settings.sigma_v if settings.sigma_v is not None else 0.00625,
                sigma_m=settings.sigma_m if settings.sigma_m is not None else 0.05,
                acceleration_gain=settings.acceleration_gain if settings.acceleration_gain is not None else 0.5,
                motion_threshold_slow=settings.motion_threshold_slow if settings.motion_threshold_slow is not None else 2.0,
                motion_threshold_fast=settings.motion_threshold_fast if settings.motion_threshold_fast is not None else 20.0,
                enable_prediction=settings.enable_prediction,
                prediction_steps=settings.prediction_steps,
                prediction_confidence_decay=settings.prediction_confidence_decay,
                min_prediction_confidence=settings.min_prediction_confidence,
            )
        elif kind == "deepsort_adaptive":
            self.tracker = DeepSortAdaptive(
                dt=settings.dt,
                max_age=settings.max_age,
                n_init=settings.n_init,
                stationary_prune=settings.stationary_prune,
                use_appearance=settings.use_appearance,
                appearance_thresh=settings.appearance_thresh,
                iou_thresh=settings.iou_thresh,
                kalman_dt=settings.kalman_dt,
                base_std_pos=settings.base_std_pos if settings.base_std_pos is not None else 0.05,
                base_std_vel=settings.base_std_vel if settings.base_std_vel is not None else 0.00625,
                base_std_meas=settings.base_std_meas if settings.base_std_meas is not None else 0.05,
                motion_adapt_gain=settings.motion_adapt_gain if settings.motion_adapt_gain is not None else 0.3,
                velocity_limit=settings.velocity_limit if settings.velocity_limit is not None else 100.0,
                enable_prediction=settings.enable_prediction,
                prediction_steps=settings.prediction_steps,
                prediction_confidence_decay=settings.prediction_confidence_decay,
                min_prediction_confidence=settings.min_prediction_confidence,
            )
        else:
            raise ValueError(
                f"unknown tracker kind: {settings.tracker_kind!r}; "
                f"expected one of {sorted(ALL_KINDS)}"
            )

        # ReID (only used by cascade / adaptive matchers) -----------
        self.reid_extractor = None
        needs_reid = kind in {"deepsort_cascade", "deepsort_adaptive"} and settings.use_appearance
        if needs_reid:
            try:
                from cvtrack.appearance.factory import make_extractor
                self.reid_extractor = make_extractor(
                    "osnet",
                    weights=settings.appearance_weights,
                    device=settings.device,
                )
            except Exception as exc:
                log.warning("ReID extractor failed to build: %s; falling back to motion-only", exc)
                self.reid_extractor = None
                self.tracker.use_appearance = False

        self._reid_galleries: Dict[int, Gallery] = {}
        self._reid_gallery_size = 50
        self._reid_ema_alpha = 0.05
        self._reid_min_side = 8

        # Velocity index in the KF mean vector depends on the tracker.
        if isinstance(self.tracker, BoTSortTracker):
            self._vx_idx, self._vy_idx = 4, 5
        else:
            # BoTSortAdaptive also uses 8-state KF -> velocity is at 4,5.
            # DeepSortAdaptive / DeepSortCascade / DeepSortLite use 4-state.
            kind_lower = settings.tracker_kind.lower()
            if kind_lower == "botsort_adaptive":
                self._vx_idx, self._vy_idx = 4, 5
            else:
                self._vx_idx, self._vy_idx = 2, 3

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------
    @classmethod
    def from_yaml(cls, preset: str) -> "CvtrackRunner":
        """Build a runner from a cvtrack YAML preset (e.g. ``drone``)."""
        cfg = load_config(preset)
        settings = _settings_from_config(cfg.raw)
        return cls(settings)

    @classmethod
    def from_overrides(
        cls,
        preset: Optional[str] = None,
        overrides: Optional[Dict[str, Any]] = None,
        fps: float = 20.0,
    ) -> "CvtrackRunner":
        """Build a runner from a preset + flat overrides dict.

        ``overrides`` accepts the same top-level keys as the cvtrack CLI:
        ``tracker``, ``detector``, ``appearance``, plus the
        ``tracker.kalman`` and ``trajectory_prediction`` sub-dicts that
        drive the adaptive tracker variants.  ``fps`` is used to
        derive the Kalman ``dt`` if not explicitly provided.
        """
        if preset:
            cfg = load_config(preset)
        else:
            cfg = Config(raw={})
        merged = merge_cli(cfg.raw, overrides or {})
        settings = _settings_from_config(merged)
        # If the user didn't pass an explicit dt, derive from fps.
        if not overrides or "dt" not in overrides:
            settings.dt = 1.0 / max(float(fps), 1.0)
        return cls(settings)

    # ------------------------------------------------------------------
    # Per-frame interface
    # ------------------------------------------------------------------
    def step(self, frame: np.ndarray) -> List[Track]:
        """Process one BGR frame and return the list of confirmed tracks.

        Tentative tracks (fewer than ``n_init`` hits) are excluded unless
        ``include_tentative`` was set to True in the settings.  The
        returned list is the same list the underlying tracker keeps
        internally; do not mutate it in place.
        """
        if frame is None or frame.size == 0:
            return []
        detections = self.detector(frame)
        embeddings: List[Optional[np.ndarray]] = [None] * len(detections)
        if self.reid_extractor is not None:
            for j, d in enumerate(detections):
                if min(d.w, d.h) >= self._reid_min_side:
                    embeddings[j] = self.reid_extractor(
                        frame, (d.x1, d.y1, d.x2, d.y2)
                    )

        # Dispatch on tracker class so each variant gets the exact
        # signature it expects.  BoT-SORT (legacy + adaptive) needs the
        # frame for CMC; the 4-state DeepSORT variants do not.
        if isinstance(self.tracker, (BoTSortTracker, BoTSortAdaptive)):
            tracks = self.tracker.step(frame, detections, det_embeddings=embeddings)
        elif isinstance(self.tracker, (DeepSortCascade, DeepSortAdaptive)):
            tracks = self.tracker.step(
                detections,
                det_embeddings=embeddings,
                galleries=self._reid_galleries,
            )
        else:
            # Defensive fallback: DeepSortLite (no frame, no embeddings).
            tracks = self.tracker.step(detections)

        # Refresh galleries + per-track embedding means so the cascade
        # matcher has an updated appearance prototype next step.
        if self.reid_extractor is not None:
            for t in tracks:
                best_idx, best_d2 = -1, float("inf")
                for j, d in enumerate(detections):
                    if embeddings[j] is None:
                        continue
                    dd = (d.cx - t.pos[0]) ** 2 + (d.cy - t.pos[1]) ** 2
                    if dd < best_d2:
                        best_d2 = dd
                        best_idx = j
                if best_idx < 0:
                    continue
                max_d2 = (max(t.box.w, t.box.h) ** 2) * 4.0
                if best_d2 > max_d2:
                    continue
                g = self._reid_galleries.get(t.track_id)
                if g is None:
                    g = Gallery(
                        size=self._reid_gallery_size,
                        ema_alpha=self._reid_ema_alpha,
                    )
                    self._reid_galleries[t.track_id] = g
                g.add(embeddings[best_idx])
                t.embedding_mean = g.mean

        if self.settings.include_tentative:
            return tracks
        return [t for t in tracks if t.confirmed]

    # ------------------------------------------------------------------
    # Convenience: structured output for downstream protocols
    # ------------------------------------------------------------------
    def step_records(self, frame: np.ndarray) -> List["TrackedTarget"]:
        """Return a list of :class:`TrackedTarget` records for the frame.

        This is the canonical shape the swarm ROS2 node consumes: it
        exposes only the fields that round-trip cleanly through a
        ``swarm_interfaces/TargetTrack`` message (target_id, x, y,
        vx, vy) plus the detection score and label for logging /
        debugging.

        Enhanced version includes scheduling fields:
        - cls: Object class ID
        - speed: Speed magnitude
        - motion_mode: Motion classification
        - pred_x, pred_y, pred_conf: Future trajectory predictions
        """
        return [
            self._make_record(t)
            for t in self.step(frame)
        ]

    def _make_record(self, t: Track) -> "TrackedTarget":
        """Create a TrackedTarget record from a Track object.

        Production path: the adaptive tracker classes (BoTSortAdaptive,
        DeepSortAdaptive) call :meth:`Track.update_trajectory_prediction`
        and :meth:`Track.detect_motion_mode` on every step, so
        ``t.predicted_future`` and ``t.motion_mode`` are guaranteed to
        be populated.  The ``hasattr`` guards below are kept as
        belt-and-braces for the legacy trackers (BoTSortTracker /
        DeepSortCascade / DeepSortLite), which do **not** populate
        those fields — so for those trackers the prediction arrays
        collapse to zero / one and the motion_mode falls back to
        ``0`` (unknown).  Downstream consumers should treat those as
        "not available" rather than as a real classification.
        """
        # Extract predicted trajectory
        pred_x = [0.0] * 5
        pred_y = [0.0] * 5
        pred_conf = [1.0] * 5

        if hasattr(t, 'predicted_future') and t.predicted_future:
            for i, (px, py, std_x, std_y) in enumerate(t.predicted_future[:5]):
                pred_x[i] = px
                pred_y[i] = py
                pred_conf[i] = t.prediction_confidence if i == 0 else max(0.1, t.prediction_confidence - i * 0.15)

        # Get motion mode
        motion_mode = 0
        if hasattr(t, 'motion_mode'):
            mode_str = getattr(t, 'motion_mode', 'unknown')
            if mode_str == 'stationary':
                motion_mode = 1
            elif mode_str == 'slow':
                motion_mode = 2
            elif mode_str == 'fast':
                motion_mode = 3

        # Calculate speed
        speed = 0.0
        if hasattr(t, 'get_speed'):
            speed = t.get_speed()
        else:
            speed = float(np.sqrt(t.mean[self._vx_idx]**2 + t.mean[self._vy_idx]**2))

        return TrackedTarget(
            target_id=int(t.track_id),
            x=float(t.pos[0]),
            y=float(t.pos[1]),
            vx=float(t.mean[self._vx_idx]),
            vy=float(t.mean[self._vy_idx]),
            label=str(t.label),
            score=float(t.box.score),
            confirmed=bool(t.confirmed),
            cls=int(t.box.cls),
            speed=speed,
            motion_mode=motion_mode,
            confidence=float(t.box.score),
            pred_x=pred_x,
            pred_y=pred_y,
            pred_conf=pred_conf,
        )


@dataclass
class TrackedTarget:
    """Lightweight, message-friendly view of a confirmed track.

    The fields here are the ones the swarm ``tracker_node`` publishes
    in a ``swarm_interfaces/TargetTrack`` (target_id, x, y, vx, vy);
    ``label`` / ``score`` / ``confirmed`` are diagnostic only.

    Enhanced with fields for scheduling module integration:
    - cls: Object class ID
    - speed: Speed magnitude
    - motion_mode: Motion classification
    - pred_x, pred_y, pred_conf: Future trajectory predictions
    """

    target_id: int
    x: float
    y: float
    vx: float
    vy: float
    label: str = ""
    score: float = 0.0
    confirmed: bool = True

    # Enhanced scheduling fields
    cls: int = 0
    speed: float = 0.0
    motion_mode: int = 0  # 0=unknown, 1=stationary, 2=slow, 3=fast
    confidence: float = 1.0
    pred_x: List[float] = field(default_factory=lambda: [0.0] * 5)
    pred_y: List[float] = field(default_factory=lambda: [0.0] * 5)
    pred_conf: List[float] = field(default_factory=lambda: [1.0] * 5)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _settings_from_config(raw: Dict[str, Any]) -> RunnerSettings:
    det = raw.get("detector", {}) or {}
    tr = raw.get("tracker", {}) or {}
    ap = raw.get("appearance", {}) or {}
    kal = (tr.get("kalman") or {}) if isinstance(tr, dict) else {}
    # deepsort_kalman section in the optimized YAML is the variant for
    # the 4-state cascade matcher; alias it to the same fields the
    # deepsort_adaptive constructor reads.
    if not kal and isinstance(tr, dict) and isinstance(tr.get("deepsort_kalman"), dict):
        kal = tr.get("deepsort_kalman", {}) or {}
    tp = raw.get("trajectory_prediction", {}) or {}

    classes_raw = det.get("classes")
    if isinstance(classes_raw, str):
        classes = [int(c) for c in classes_raw.split(",") if c.strip()]
    elif isinstance(classes_raw, list):
        classes = [int(c) for c in classes_raw]
    else:
        classes = [0, 2, 5, 7, 16]

    use_app = bool(ap.get("enabled", False))
    kind = str(tr.get("kind", "deepsort_cascade"))

    return RunnerSettings(
        tracker_kind=kind,
        detector_backend=str(det.get("backend", "yolo")),
        weights=str(det.get("weights", "") or ""),
        device=str(det.get("device", "cpu")),
        imgsz=int(det.get("imgsz", 320)),
        conf=float(det.get("conf", 0.15)),
        classes=classes,
        min_box_area=float(det.get("min_box_area", 200.0)),
        min_conf=float(det.get("min_conf", 0.0)),
        nms_iou=float(det.get("nms_iou", 0.50)),
        dt=1.0 / max(float(raw.get("fps", 20.0) or 20.0), 1.0),
        max_age=int(tr.get("max_age", 30)),
        n_init=int(tr.get("n_init", 3)),
        iou_thresh=float(tr.get("iou_thresh", 0.30)),
        high_conf=float(tr.get("high_conf", 0.35)),
        new_track_conf=float(tr.get("new_track_conf", 0.20)),
        lost_relink_frames=int(tr.get("lost_relink_frames", 30)),
        stationary_prune=bool(tr.get("stationary_prune", True)),
        use_appearance=use_app,
        appearance_weights=(str(ap["weights"]) if ap.get("weights") else None),
        appearance_thresh=float(ap.get("match_threshold", tr.get("appearance_thresh", 0.5))),
        include_tentative=bool(raw.get("pipeline", {}).get("include_tentative", False)),
        # Adaptive tracker knobs (only effective for the *_adaptive kinds).
        kalman_dt=_opt_float(kal.get("dt")),
        sigma_p=_opt_float(kal.get("sigma_p")),
        sigma_v=_opt_float(kal.get("sigma_v")),
        sigma_m=_opt_float(kal.get("sigma_m")),
        acceleration_gain=_opt_float(kal.get("acceleration_gain")),
        motion_threshold_slow=_opt_float(kal.get("motion_threshold_slow")),
        motion_threshold_fast=_opt_float(kal.get("motion_threshold_fast")),
        base_std_pos=_opt_float(kal.get("base_std_pos")),
        base_std_vel=_opt_float(kal.get("base_std_vel")),
        base_std_meas=_opt_float(kal.get("base_std_meas")),
        motion_adapt_gain=_opt_float(kal.get("motion_adapt_gain")),
        velocity_limit=_opt_float(kal.get("velocity_limit")),
        innovation_gate=_opt_float(kal.get("innovation_gate")),
        # Trajectory prediction knobs.
        enable_prediction=bool(tp.get("enabled", True)),
        prediction_steps=int(tp.get("prediction_steps", 10)),
        prediction_confidence_decay=float(tp.get("confidence_decay", 0.9)),
        min_prediction_confidence=float(tp.get("min_confidence", 0.1)),
    )


def _opt_float(value: Any) -> Optional[float]:
    """Coerce to float, returning None for missing or non-numeric values."""
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


__all__ = [
    "ADAPTIVE_KINDS",
    "ALL_KINDS",
    "CvtrackRunner",
    "LEGACY_KINDS",
    "RunnerSettings",
    "TrackedTarget",
]
