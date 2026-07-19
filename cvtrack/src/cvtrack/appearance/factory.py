"""Factory + protocol for appearance extractors.

Only ships one backend:

* ``osnet`` -- OSNet via torch.hub (pytorch/vision) with optional torchreid path
  for custom ReID checkpoints.  Returns ``None`` if the network stack is
  unavailable so the pipeline degrades to pure geometric tracking.

The ``HistogramExtractor`` fallback has been removed per user requirement:
all ReID runs must use a real pretrained model.
"""

from __future__ import annotations

import logging
from typing import Optional

from cvtrack.appearance.base import AppearanceExtractor
from cvtrack.appearance.osnet import OsNetExtractor


log = logging.getLogger(__name__)


def make_extractor(
    backend: str = "osnet",
    *,
    weights: Optional[str] = None,
    model_name: str = "osnet_x1_0",
    device: str = "cpu",
) -> Optional[AppearanceExtractor]:
    """Create an appearance extractor.

    Returns ``None`` if the backend cannot be loaded (no torch, no network,
    no weights).  The caller should handle this gracefully by disabling ReID.
    """
    backend = backend.lower()
    if backend == "osnet":
        ext = OsNetExtractor(model_name=model_name, weights=weights, device=device)
        if ext.is_available:
            log.info(
                "OSNet extractor ready (loaded_pretrained=%s, embedding_dim=%d)",
                ext.loaded_pretrained,
                ext.embedding_dim,
            )
            return ext
        log.warning("OSNet unavailable; ReID is disabled for this run")
        return None
    raise ValueError(f"unknown appearance backend: {backend!r}")
