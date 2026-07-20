"""Drawing styles + colour palette."""

from __future__ import annotations

from typing import Tuple


_COLOURS = [
    (66, 133, 244), (219, 68, 55), (15, 157, 88), (244, 180, 0),
    (171, 71, 188), (255, 112, 67), (38, 198, 218), (124, 77, 255),
]

RELINK_COLOUR = (0, 255, 255)


def colour_for(track_id: int) -> Tuple[int, int, int]:
    """Deterministic colour for a given track id."""
    return _COLOURS[int(track_id) % len(_COLOURS)]