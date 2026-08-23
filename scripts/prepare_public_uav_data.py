#!/usr/bin/env python3
"""Download and inspect public UAV/traffic datasets outside the Git repo."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import urllib.request
import zipfile
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_ROOT = Path(
    os.environ.get(
        "CVTRACK_DATASETS_ROOT",
        "F:/codex-cursor-plugins/vision-recognition/datasets",
    )
)


@dataclass(frozen=True)
class Dataset:
    key: str
    name: str
    task: str
    source: str
    size: str
    url: str = ""
    manual: str = ""
    archive_ext: str = ""


DATASETS: dict[str, Dataset] = {
    "visdrone_det_val": Dataset(
        key="visdrone_det_val",
        name="VisDrone2019-DET-val",
        task="drone object detection validation",
        source="https://github.com/VisDrone/VisDrone-Dataset",
        size="0.07 GB",
        url="https://github.com/ultralytics/assets/releases/download/v0.0.0/VisDrone2019-DET-val.zip",
    ),
    "visdrone_det_train": Dataset(
        key="visdrone_det_train",
        name="VisDrone2019-DET-train",
        task="drone object detection training",
        source="https://github.com/VisDrone/VisDrone-Dataset",
        size="1.44 GB",
        url="https://github.com/ultralytics/assets/releases/download/v0.0.0/VisDrone2019-DET-train.zip",
    ),
    "visdrone_det_test_dev": Dataset(
        key="visdrone_det_test_dev",
        name="VisDrone2019-DET-test-dev",
        task="drone object detection test-dev with GT",
        source="https://github.com/VisDrone/VisDrone-Dataset",
        size="0.28 GB",
        url="https://github.com/ultralytics/assets/releases/download/v0.0.0/VisDrone2019-DET-test-dev.zip",
    ),
    "visdrone_mot_val": Dataset(
        key="visdrone_mot_val",
        name="VisDrone2019-MOT-val",
        task="drone multi-object tracking validation",
        source="https://github.com/VisDrone/VisDrone-Dataset",
        size="1.48 GB",
        manual="Google Drive file id: 1rqnKe9IgU_crMaxRoel9_nuUsMEBBVQu",
    ),
    "uavdt_benchmark_m": Dataset(
        key="uavdt_benchmark_m",
        name="UAVDT-Benchmark-M",
        task="UAV vehicle detection and multi-object tracking",
        source="https://sites.google.com/view/grli-uavdt",
        size="about 80k annotated frames",
        manual="Use the official UAVDT-Benchmark-M Google Drive dataset link; research purpose only.",
    ),
    "auair_annotations": Dataset(
        key="auair_annotations",
        name="AU-AIR annotations",
        task="UAV traffic labels plus flight metadata",
        source="https://github.com/sunw71/auairdataset",
        size="3.9 MB",
        manual="Google Drive file id: 1boGF0L6olGe_Nu7rd1R8N7YmQErCb0xA",
    ),
    "auair_images": Dataset(
        key="auair_images",
        name="AU-AIR images",
        task="low-altitude UAV traffic frames",
        source="https://github.com/sunw71/auairdataset",
        size="2.2 GB",
        manual="Google Drive file id: 1pJ3xfKtHiTdysX5G3dxqKTdGESOBYCxJ",
    ),
    "mitra_data_t1": Dataset(
        key="mitra_data_t1",
        name="MiTra Data_T1",
        task="freeway trajectory and traffic-state training",
        source="https://doi.org/10.25532/OPARA-881",
        size="80.71 MB",
        url="https://opara.zih.tu-dresden.de/bitstreams/83c9ec15-7438-4d45-8b5d-f0d57b3f49a4/download",
        archive_ext=".zip",
    ),
    "drift": Dataset(
        key="drift",
        name="DRIFT",
        task="4K drone OBB detection, ByteTrack, trajectory analysis",
        source="https://github.com/AIxMobility/The-DRIFT",
        size="HuggingFace dataset",
        manual='Python: from datasets import load_dataset; load_dataset("Hj-Lee/The-DRIFT")',
    ),
}


def dataset_dir(root: Path, dataset: Dataset) -> Path:
    return root / dataset.key


def archive_path(root: Path, dataset: Dataset) -> Path:
    suffix = dataset.archive_ext or (".zip" if dataset.url.endswith(".zip") else ".bin")
    return dataset_dir(root, dataset) / "archive" / f"{dataset.key}{suffix}"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download(dataset: Dataset, root: Path) -> Path:
    if not dataset.url:
        raise SystemExit(f"{dataset.key} has no direct URL; run the manual command.")
    path = archive_path(root, dataset)
    legacy_path = dataset_dir(root, dataset) / "archive" / f"{dataset.key}.bin"
    if path.suffix == ".zip" and not path.exists() and legacy_path.exists():
        legacy_path.rename(path)
        print(f"[data] renamed zip payload: {legacy_path} -> {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > 0:
        print(f"[data] exists: {path}")
        return path
    print(f"[data] downloading {dataset.name} -> {path}")
    with urllib.request.urlopen(dataset.url) as response, path.open("wb") as output:
        shutil.copyfileobj(response, output)
    return path


def extract(path: Path, root: Path, dataset: Dataset) -> Path:
    target = dataset_dir(root, dataset) / "extracted"
    if target.exists() and any(target.iterdir()):
        print(f"[data] extracted exists: {target}")
        return target
    if not zipfile.is_zipfile(path):
        raise SystemExit(f"{path} is not a zip archive; skip extraction.")
    target.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path) as archive:
        archive.extractall(target)
    print(f"[data] extracted: {target}")
    return target


def inspect_visdrone_det(extracted: Path) -> dict[str, object]:
    images = sorted(extracted.rglob("images/*.jpg"))
    annotations = sorted(extracted.rglob("annotations/*.txt"))
    sample_annotation = None
    boxes = 0
    class_counts: dict[str, int] = {}
    if annotations:
        sample_annotation = str(annotations[0])
        for line in annotations[0].read_text(encoding="utf-8").splitlines():
            parts = line.split(",")
            if len(parts) >= 6:
                boxes += 1
                class_counts[parts[5]] = class_counts.get(parts[5], 0) + 1
    return {
        "image_count": len(images),
        "annotation_count": len(annotations),
        "sample_image": str(images[0]) if images else None,
        "sample_annotation": sample_annotation,
        "sample_box_count": boxes,
        "sample_class_counts": class_counts,
    }


def inspect_mitra_trajectory(extracted: Path) -> dict[str, object]:
    csv_files = sorted(extracted.rglob("*.csv"))
    files: list[dict[str, object]] = []
    total_rows = 0
    for csv_file in csv_files:
        with csv_file.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
            header = handle.readline().strip()
            sample = handle.readline().strip()
            rows = 1 if sample else 0
            rows += sum(1 for _ in handle)
        total_rows += rows
        files.append(
            {
                "file": str(csv_file),
                "size_bytes": csv_file.stat().st_size,
                "data_rows": rows,
                "columns": header.split(",") if header else [],
                "sample_values": sample.split(",")[:12] if sample else [],
            }
        )
    return {
        "csv_count": len(csv_files),
        "total_data_rows": total_rows,
        "files": files,
    }


def inspect_dataset(dataset: Dataset, extracted: Path | None) -> dict[str, object] | None:
    if not extracted:
        return None
    if dataset.key.startswith("visdrone_det"):
        return inspect_visdrone_det(extracted)
    if dataset.key.startswith("mitra_data"):
        return inspect_mitra_trajectory(extracted)
    return None


def write_manifest(root: Path, dataset: Dataset, archive: Path, extracted: Path | None) -> Path:
    manifest = {
        "dataset": asdict(dataset),
        "downloaded_at": datetime.now(timezone.utc).isoformat(),
        "archive": str(archive),
        "archive_size_bytes": archive.stat().st_size,
        "archive_sha256": sha256_file(archive),
        "extracted": str(extracted) if extracted else None,
        "inspection": inspect_dataset(dataset, extracted),
    }
    path = dataset_dir(root, dataset) / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[data] manifest: {path}")
    return path


def cmd_list(_: argparse.Namespace) -> None:
    for dataset in DATASETS.values():
        method = "direct" if dataset.url else "manual"
        print(f"{dataset.key:22} {method:7} {dataset.size:12} {dataset.task}")


def cmd_manual(args: argparse.Namespace) -> None:
    dataset = DATASETS[args.dataset]
    print(f"{dataset.name}: {dataset.manual or dataset.url}")


def cmd_download(args: argparse.Namespace) -> None:
    root = Path(args.root)
    dataset = DATASETS[args.dataset]
    archive = download(dataset, root)
    extracted = extract(archive, root, dataset) if args.extract else None
    manifest = write_manifest(root, dataset, archive, extracted)
    if args.inspect:
        data = json.loads(manifest.read_text(encoding="utf-8"))
        print(json.dumps(data["inspection"], indent=2, ensure_ascii=False))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=str(DEFAULT_ROOT), help="dataset root directory")
    subparsers = parser.add_subparsers(dest="command", required=True)
    list_parser = subparsers.add_parser("list")
    list_parser.set_defaults(func=cmd_list)
    manual_parser = subparsers.add_parser("manual")
    manual_parser.add_argument("dataset", choices=sorted(DATASETS))
    manual_parser.set_defaults(func=cmd_manual)
    download_parser = subparsers.add_parser("download")
    download_parser.add_argument("dataset", choices=sorted(DATASETS))
    download_parser.add_argument("--extract", action="store_true")
    download_parser.add_argument("--inspect", action="store_true")
    download_parser.set_defaults(func=cmd_download)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
