"""Configuration loading, preset merging, validation.

YAML schema (top-level keys):

    detector:   dict
    tracker:    dict
    appearance: dict
    viz:        dict
    output:     dict
    pipeline:   dict  (max_frames, max_seconds, start_frame, etc.)

Preset merging rules:

* Each config can declare `extends: <name>` which is resolved relative to the
  configs dir (no path traversal allowed).
* The chain of extends is resolved depth-first (first parent wins for ties),
  but the *child* overrides the parent on conflicting scalar/list keys.
* List values are *replaced* by the child, not concatenated (predictable for
  `classes`).
* Unknown top-level keys raise an error so typos surface early.
"""

from __future__ import annotations

import copy
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml


ALLOWED_TOP_LEVEL = {
    "detector",
    "tracker",
    "appearance",
    "viz",
    "output",
    "pipeline",
    "extends",
    "sensors",  # multi-sensor preset placeholder; unused by the v6 pipeline
}


@dataclass
class Config:
    """Resolved, fully-merged configuration."""

    raw: Dict[str, Any] = field(default_factory=dict)

    def get(self, *path: str, default: Any = None) -> Any:
        """Walk dotted path. Returns default if any segment is missing."""
        cur: Any = self.raw
        for p in path:
            if not isinstance(cur, dict) or p not in cur:
                return default
            cur = cur[p]
        return cur

    def section(self, name: str) -> Dict[str, Any]:
        return dict(self.raw.get(name, {}))


def _configs_dir() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "configs")


def _resolve_extends(value: str, configs_dir: str, _seen: Optional[set] = None) -> str:
    """Resolve a relative config name (e.g. 'default' or 'drone') to a path.

    Rejects path traversal: the resolved path must remain inside `configs_dir`.
    """
    seen = _seen if _seen is not None else set()
    if value in seen:
        raise ValueError(f"cyclic config extends chain: {value}")
    seen.add(value)
    if value.endswith((".yaml", ".yml")):
        candidate = os.path.join(configs_dir, os.path.basename(value))
    else:
        candidate = os.path.join(configs_dir, f"{value}.yaml")
    abs_dir = os.path.abspath(configs_dir)
    abs_cand = os.path.abspath(candidate)
    if not abs_cand.startswith(abs_dir + os.sep) and abs_cand != abs_dir:
        raise ValueError(f"config extends path escapes configs dir: {value}")
    return abs_cand


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data or {}


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge `override` into `base`.

    Lists and scalars in `override` replace the base value. Only dicts are
    merged recursively. The base is *not* mutated.
    """
    out = copy.deepcopy(base)
    for k, v in override.items():
        if k == "extends":
            continue  # handled by caller
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_config(path_or_name: str, configs_dir: Optional[str] = None) -> Config:
    """Load a YAML config, resolving `extends:` chains.

    Parameters
    ----------
    path_or_name:
        Either a bare name (resolved under `configs_dir`) or a direct path.
    configs_dir:
        Directory holding named presets. Defaults to the bundled `configs/`.

    Returns
    -------
    Config
        Fully-merged config with the resolved raw dict.
    """
    cdir = configs_dir or _configs_dir()
    if os.path.isfile(path_or_name):
        path = path_or_name
    else:
        path = _resolve_extends(path_or_name, cdir)
    data = _load_yaml(path)
    parent_name = data.get("extends")
    if parent_name:
        if not os.path.isfile(parent_name):
            parent_path = _resolve_extends(parent_name, cdir)
        else:
            parent_path = parent_name
        parent_cfg = load_config(parent_path, configs_dir=cdir)
        merged = _deep_merge(parent_cfg.raw, data)
    else:
        merged = data
    _validate(merged)
    return Config(raw=merged)


def _validate(data: Dict[str, Any]) -> None:
    unknown = set(data.keys()) - ALLOWED_TOP_LEVEL
    if unknown:
        raise ValueError(f"unknown top-level config keys: {sorted(unknown)}")
    det = data.get("detector", {})
    if "imgsz" in det and not isinstance(det["imgsz"], int):
        raise ValueError(f"detector.imgsz must be int, got {type(det['imgsz']).__name__}")
    if "classes" in det and det["classes"] is not None and not isinstance(det["classes"], list):
        raise ValueError("detector.classes must be a list of ints")
    tr = data.get("tracker", {})
    if "kind" in tr and tr["kind"] not in {"botsort", "deepsort", "deepsort_cascade"}:
        raise ValueError(
            f"tracker.kind must be 'botsort', 'deepsort' or 'deepsort_cascade', "
            f"got {tr['kind']!r}"
        )
    ap = data.get("appearance", {})
    if ap.get("enabled"):
        weights = ap.get("weights")
        # ``enabled=True`` without usable weights is allowed: the pipeline
        # auto-tries OSNet (via torch.hub) and gracefully disables ReID
        # when the network stack is unavailable.  The tracker continues
        # with pure geometric matching.
        if weights is not None and not isinstance(weights, str):
            raise ValueError("appearance.weights must be a string path")
    if "min_box_side" in ap and not isinstance(ap["min_box_side"], (int, float)):
        raise ValueError("appearance.min_box_side must be a number")


def merge_cli(raw: Dict[str, Any], cli: Dict[str, Any]) -> Dict[str, Any]:
    """Overlay CLI-supplied values onto the loaded YAML.

    Only keys that are explicitly set (non-None) in `cli` override the YAML.
    Nested dicts are merged one level deep so e.g. `tracker.max_age=30` only
    touches the tracker section.
    """
    out = copy.deepcopy(raw)
    for k, v in cli.items():
        if v is None:
            continue
        if isinstance(v, dict):
            out.setdefault(k, {})
            if not isinstance(out[k], dict):
                out[k] = {}
            out[k].update(v)
        else:
            out[k] = v
    return out