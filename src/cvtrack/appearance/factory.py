"""Factory + protocol for appearance extractors.

Currently only ``OsNetExtractor`` is shipped; the factory exists so users can
register alternative backends without modifying the pipeline.
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
    """Create an appearance extractor.  Returns None if the backend is unavailable."""
    backend = backend.lower()
    if backend == "osnet":
        ext = OsNetExtractor(model_name=model_name, weights=weights, device=device)
        if not ext.is_available:
            log.warning("OSNet extractor unavailable; ReID disabled")
            return None
        return ext
    raise ValueError(f"unknown appearance backend: {backend!r}")