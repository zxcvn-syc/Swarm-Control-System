from __future__ import annotations

import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = REPO_ROOT / "examples" / "rfly_ros2" / "scripts" / "validate_rfly_run.py"


def load_validator_module():
    spec = importlib.util.spec_from_file_location("rfly_validator", VALIDATOR_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_manifest(path: Path, topics: set[str]) -> None:
    manifest_topics = {}
    for topic in topics:
        evidence = f"{topic}.yaml"
        (path.parent / evidence).write_text("data: present\n", encoding="utf-8")
        manifest_topics[topic] = {
            "received": 1,
            "first_payload_file": evidence,
            "has_payload_evidence": True,
        }
    path.write_text(
        json.dumps({"topics": manifest_topics, "pending": []}),
        encoding="utf-8",
    )


def test_manifest_requires_every_expected_topic(tmp_path: Path) -> None:
    validator = load_validator_module()
    manifest = tmp_path / "evidence_manifest.json"
    write_manifest(manifest, set(validator.EXPECTED_EVIDENCE_TOPICS))
    assert validator.evidence_manifest_is_complete(manifest)

    write_manifest(manifest, {"task_assignment"})
    assert not validator.evidence_manifest_is_complete(manifest)
