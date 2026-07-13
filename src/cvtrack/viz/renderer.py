"""Renderer: bounding boxes, info panels, trails, overlays."""

from __future__ import annotations

from typing import TYPE_CHECKING

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