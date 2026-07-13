"""Re-identification (ReID) appearance module."""

from cvtrack.appearance.base import AppearanceExtractor, crop_with_margin, l2_normalize
from cvtrack.appearance.gallery import Gallery
from cvtrack.appearance.osnet import OsNetExtractor

__all__ = [
    "AppearanceExtractor",
    "Gallery",
    "OsNetExtractor",
    "crop_with_margin",
    "l2_normalize",
]