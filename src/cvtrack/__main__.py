"""python -m cvtrack ... — CLI entry point."""

from __future__ import annotations

from cvtrack.pipeline import main

if __name__ == "__main__":
    raise SystemExit(main())