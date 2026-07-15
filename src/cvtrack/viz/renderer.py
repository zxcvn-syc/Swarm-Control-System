"""Renderer: bounding boxes, info panels, trails, overlays, future projection."""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional

import cv2
import numpy as np

from cvtrack.viz.styles import RELINK_COLOUR, colour_for


if TYPE_CHECKING:  # pragma: no cover
    from cvtrack.types import Track


def draw_box(frame: np.ndarray, track: "Track") -> None:
    """Draw a single track's bounding box + compact info panel."""
    b = track.box
    relinking = getattr(track, "relink_remaining", 0) > 0
    c = RELINK_COLOUR if relinking else colour_for(track.track_id)
    x1, y1, x2, y2 = int(b.x1), int(b.y1), int(b.x2), int(b.y2)
    thickness = 3 if relinking else 2
    cv2.rectangle(frame, (x1, y1), (x2, y2), c, thickness)
    label = f"id {track.track_id} {b.label}"
    if not track.confirmed:
        label = f"id? {b.label}"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(frame, (x1, max(0, y1 - th - 8)), (x1 + tw + 8, y1), c, -1)
    cv2.putText(
        frame, label, (x1 + 4, max(th + 2, y1 - 4)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
    )

    mean = getattr(track, "mean", np.zeros(1))
    vx = float(mean[4]) if mean.shape[0] >= 6 else float(mean[2])
    vy = float(mean[5]) if mean.shape[0] >= 6 else float(mean[3])
    speed = (vx * vx + vy * vy) ** 0.5
    info = f"v={speed:4.1f}px/f  c={b.score:.2f}"
    (tw2, th2), _ = cv2.getTextSize(info, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
    cv2.rectangle(frame, (x1, y1), (x1 + tw2 + 8, y1 + th2 + 6), (0, 0, 0), -1)
    cv2.putText(
        frame, info, (x1 + 4, y1 + th2 + 2),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA,
    )


def draw_trail(frame: np.ndarray, track: "Track", length: int = 35) -> None:
    """Anti-aliased fading polyline of the KF-predicted trail + velocity crosshair."""
    c = colour_for(track.track_id)
    pts = list(track.pred_trail[-length:])
    n = len(pts)
    if n < 2:
        if n == 1:
            cv2.circle(frame, (int(pts[0][0]), int(pts[0][1])), 4, c, -1, cv2.LINE_AA)
        return
    overlay = frame.copy()
    for i in range(n - 1):
        age = (n - 2) - i
        t = 1.0 - (age / max(n - 1, 1))
        thickness = max(1, int(round(2 + 3 * t)))
        faded = (int(c[0] * t), int(c[1] * t), int(c[2] * t))
        a = (int(pts[i][0]), int(pts[i][1]))
        b = (int(pts[i + 1][0]), int(pts[i + 1][1]))
        cv2.line(overlay, a, b, faded, thickness, cv2.LINE_AA)
    cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)
    cx, cy = int(pts[-1][0]), int(pts[-1][1])
    cv2.circle(frame, (cx, cy), 4, c, -1, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 5, (255, 255, 255), 1, cv2.LINE_AA)
    mean = getattr(track, "mean", None)
    if mean is not None and mean.shape[0] >= 6:
        vx, vy = float(mean[4]), float(mean[5])
    else:
        vx = vy = 0.0
    speed = (vx * vx + vy * vy) ** 0.5
    if speed > 0.5:
        nx, ny = int(cx + vx), int(cy + vy)
        cl = (255, 255, 255)
        cv2.line(frame, (nx - 6, ny), (nx + 6, ny), cl, 1, cv2.LINE_AA)
        cv2.line(frame, (nx, ny - 6), (nx, ny + 6), cl, 1, cv2.LINE_AA)
        cv2.line(frame, (nx - 6, ny), (nx + 6, ny), c, 1, cv2.LINE_AA)
        cv2.line(frame, (nx, ny - 6), (nx, ny + 6), c, 1, cv2.LINE_AA)


def add_overlay(frame: np.ndarray, fps: float, n_tracks: int, model_name: str) -> None:
    text = f"{model_name} | tracks {n_tracks} | FPS {fps:4.1f}"
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(frame, (8, 8), (12 + tw, 8 + th + 12), (0, 0, 0), -1)
    cv2.putText(
        frame, text, (12, 8 + th),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA,
    )


def draw_predicted_future_trail(
    frame: np.ndarray,
    track: "Track",
    n: int = 15,
    cov_steps: Optional[List[np.ndarray]] = None,
    sigma_scale: float = 3.0,
) -> None:
    """Draw the next KF positions as a dashed trail with uncertainty ellipses.

    When ``cov_steps`` is supplied (a list of ``(2, 2)`` covariance blocks
    for each future step, the position-position slice of the KF state
    covariance), each future position is annotated with a ``sigma_scale``
    (default 3-sigma) error ellipse whose transparency fades with the
    step index -- a visual cue for "the further we project, the less
    certain we are".
    """
    future = list(getattr(track, "future_trail", []))[:max(0, int(n))]
    if not future:
        return

    points = [track.pos, *future]
    base_colour = (255, 210, 40)

    # Optional: draw uncertainty ellipses for each future step.
    if cov_steps is not None:
        for i, cov in enumerate(cov_steps[:len(future)], start=1):
            try:
                # Eigen-decompose the 2x2 position covariance.
                eigvals, eigvecs = np.linalg.eigh(np.asarray(cov, dtype=np.float64))
                if eigvals is None or np.any(eigvals <= 0.0):
                    continue
                step_alpha = max(0.05, 0.55 - 0.04 * i)
                radii = sigma_scale * np.sqrt(eigvals)
                angle_deg = float(np.degrees(np.arctan2(eigvecs[1, 1], eigvecs[0, 1])))
                cx_f, cy_f = future[i - 1]
                overlay = frame.copy()
                cv2.ellipse(
                    overlay,
                    (int(round(cx_f)), int(round(cy_f))),
                    (int(round(radii[1])), int(round(radii[0]))),
                    angle_deg,
                    0.0, 360.0,
                    base_colour,
                    1,
                    cv2.LINE_AA,
                )
                cv2.addWeighted(overlay, step_alpha, frame, 1.0 - step_alpha, 0, frame)
            except (np.linalg.LinAlgError, ValueError):
                continue

    # Dashed future trail.
    for i in range(len(points) - 1):
        start = np.asarray(points[i], dtype=np.float64)
        end = np.asarray(points[i + 1], dtype=np.float64)
        distance = float(np.linalg.norm(end - start))
        dashes = max(1, int(distance // 5.0) + 1)
        for dash in range(dashes):
            if dash % 2:
                continue
            a_t = dash / dashes
            b_t = min((dash + 1) / dashes, 1.0)
            a = start + (end - start) * a_t
            b = start + (end - start) * b_t
            cv2.line(
                frame,
                (int(round(a[0])), int(round(a[1]))),
                (int(round(b[0])), int(round(b[1]))),
                base_colour,
                2,
                cv2.LINE_AA,
            )
    end_x, end_y = future[-1]
    cv2.circle(
        frame,
        (int(round(end_x)), int(round(end_y))),
        3,
        base_colour,
        1,
        cv2.LINE_AA,
    )