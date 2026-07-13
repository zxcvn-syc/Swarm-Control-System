"""Legacy cv_track_demo.py shim.

.. deprecated::
    This file is preserved for backwards compatibility only.  The code that
    used to live here is now the `cvtrack` package under ``src/cvtrack``.
    New users should run ``python -m cvtrack`` instead of
    ``python cv_track_demo.py``.

The CLI flags exposed below all match the original v4 script — they are
forwarded to ``cvtrack.pipeline.main``.  Output paths are unchanged, so
existing pipelines that consume ``tracked.mp4`` / ``tracks.csv`` etc. will
keep working.
"""

from __future__ import annotations

import sys
import warnings
from typing import List, Optional, Sequence

# Emit a single deprecation warning on first import (not on every call).
warnings.warn(
    "cv_track_demo.py is a compatibility shim.  Use `python -m cvtrack` instead.",
    DeprecationWarning,
    stacklevel=2,
)


def _forward_argv(extra: Optional[Sequence[str]] = None) -> List[str]:
    return ["-m", "cvtrack", *(extra or sys.argv[1:])]


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Forward to ``cvtrack.pipeline.main`` after re-argving."""
    from cvtrack.pipeline import build_argparser, main as cv_main

    # Re-parse arguments using the cvtrack parser so existing flags like
    # --drone / --tracker / --save-trail keep working.
    ap = build_argparser()
    ns = ap.parse_args(list(argv) if argv is not None else sys.argv[1:])
    return cv_main(ns)


if __name__ == "__main__":
    sys.exit(main())
