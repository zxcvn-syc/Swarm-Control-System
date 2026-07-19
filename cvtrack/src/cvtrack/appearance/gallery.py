"""Per-track embedding gallery with FIFO buffer and EMA mean."""

from __future__ import annotations

import logging
from collections import deque
from typing import Deque, Optional

import numpy as np

from cvtrack.appearance.base import l2_normalize


log = logging.getLogger(__name__)


class Gallery:
    """FIFO buffer of recent embeddings + a running mean for a single track.

    Parameters
    ----------
    size:
        Maximum number of embeddings to retain (deque length).
    ema_alpha:
        Smoothing factor for the running mean update (0 = no update, 1 = full
        overwrite by latest embedding).
    """

    def __init__(self, size: int = 50, ema_alpha: float = 0.05) -> None:
        self.size = int(size)
        self.ema_alpha = float(ema_alpha)
        self._buf: Deque[np.ndarray] = deque(maxlen=self.size)
        self._mean: Optional[np.ndarray] = None

    def __len__(self) -> int:
        return len(self._buf)

    @property
    def mean(self) -> Optional[np.ndarray]:
        return None if self._mean is None else self._mean.copy()

    def add(self, embedding: np.ndarray) -> None:
        if embedding is None:
            return
        emb = l2_normalize(embedding.astype(np.float64).reshape(-1))
        self._buf.append(emb)
        if self._mean is None:
            self._mean = emb.copy()
        else:
            self._mean = (1.0 - self.ema_alpha) * self._mean + self.ema_alpha * emb
            self._mean = l2_normalize(self._mean)

    def cosine_to(self, embedding: np.ndarray) -> float:
        """Return cosine *similarity* to the running mean (in [-1, 1])."""
        if self._mean is None or embedding is None:
            return 0.0
        e = l2_normalize(embedding.astype(np.float64).reshape(-1))
        return float(np.dot(e, self._mean))

    def cosine_distance_to(self, embedding: np.ndarray) -> float:
        """Return 1 - cosine_similarity, in [0, 2] (typically [0, 1])."""
        return 1.0 - self.cosine_to(embedding)


def make_galleries(
    size: int = 50, ema_alpha: float = 0.05
) -> "dict[int, Gallery]":
    """Factory for a per-track-id dict of galleries."""
    return {}