"""Main pipeline: detection + tracking + ReID + writing.

This module glues together:

* config (YAML) loading and CLI overlay
* detector (YOLO / MOG2)
* appearance extractor (optional)
* tracker (BoT-SORT / DeepSORT-lite)
* I/O writers (mp4, csv, smoothed csv, trails json, ReID json)
* postprocess hook

The CLI surface is backward-compatible with the v4 script: every flag the old
script understood has a corresponding argument or YAML key here, and the
default behaviour reproduces the v4 baseline on pexels_aerial_2034115
(798 confirmed IDs, mean length 23.15, ~47 fps on CPU).
"""

from __future__ import annotations

import argparse
import base64
import csv
import json
import logging
import os
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from cvtrack.appearance.gallery import Gallery
from cvtrack.config import Config, load_config, merge_cli
from cvtrack.detector.factory import make_detector
from cvtrack.io import FutureTrailCsvWriter, TrackCsvWriter, VideoReader, VideoWriter
from cvtrack.tracker.botsort import BoTSortTracker
from cvtrack.tracker.cmc import make_cmc
from cvtrack.tracker.deepsort import DeepSortCascade, DeepSortLite
from cvtrack.tracker.kalman import (
    predict_n_steps,
    predict_n_steps_with_covariance,
)
from cvtrack.tracker.smoother import rts_smooth_2d
from cvtrack.types import Box, Track
from cvtrack.viz.renderer import (
    add_overlay,
    draw_box,
    draw_predicted_future_trail,
    draw_trail,
)


log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Resolution-adaptive rescale (matches v4's "drone on a 1080p input" guard).
# ---------------------------------------------------------------------------
def detect_resolution_class(width: int, height: int) -> str:
    return "large" if max(width, height) >= 1280 else "small"


def resolve_weights(weights: str) -> str:
    """Resolve bare weight names against the conventional models dir."""
    if not weights:
        return weights
    if os.path.isabs(weights) or os.path.exists(weights):
        return weights
    # Conventional local location written by scripts/download_weights.py.
    local_candidate = os.path.join("weights", weights)
    if os.path.exists(local_candidate):
        return local_candidate
    return weights


def _build_parser() -> argparse.ArgumentParser:
    """CLI argument parser.  Every flag is overrideable via YAML."""
    ap = argparse.ArgumentParser(
        prog="cvtrack",
        description="YOLOv8 + BoT-SORT tracking demo with optional ReID.",
    )
    ap.add_argument("--config", default=None,
                    help="path or name of a YAML preset (e.g. configs/drone.yaml)")
    ap.add_argument("--source", default="",
                    help="path to input video (default: synthesise sample.mp4)")
    ap.add_argument("--out-dir", default="output",
                    help="output directory for tracked.mp4 / tracks.csv / etc. "
                         "(default: ./output)")
    ap.add_argument("--weights", default="")
    ap.add_argument("--imgsz", type=int, default=None)
    ap.add_argument("--conf", type=float, default=None)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--classes", default=None,
                    help="comma-separated COCO IDs to keep")
    ap.add_argument("--min-conf", type=float, default=None)
    ap.add_argument("--min-box-area", type=float, default=None)
    ap.add_argument("--nms-iou", type=float, default=None)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--max-seconds", type=float, default=0.0)
    ap.add_argument("--detector", choices=["yolo", "mog2", "auto"], default="auto")
    ap.add_argument("--include-tentative", action="store_true")
    ap.add_argument("--max-age", type=int, default=None)
    ap.add_argument("--n-init", type=int, default=None)
    ap.add_argument("--no-stationary-prune", action="store_true")
    ap.add_argument("--start-frame", type=int, default=0)
    ap.add_argument("--tracker", choices=["deepsort", "deepsort_cascade", "botsort"], default=None,
                    help="tracker kind (v6 adds deepsort_cascade with appearance cascade + IoU fallback)")
    ap.add_argument("--predict-horizon", type=int, default=15,
                    help="number of frames to project each track's KF into the future")
    ap.add_argument("--write-future-csv", action="store_true",
                    help="enable per-step future projection CSV with sigma_x/sigma_y columns")
    ap.add_argument("--no-cmc", action="store_true")
    ap.add_argument("--high-conf", type=float, default=None)
    ap.add_argument("--new-track-conf", type=float, default=None)
    ap.add_argument("--iou-thresh", type=float, default=None)
    ap.add_argument("--lost-relink-frames", type=int, default=None)
    ap.add_argument("--drone", action="store_true",
                    help="sugar for --config configs/drone.yaml")
    ap.add_argument("--save-trail", action="store_true")
    ap.add_argument("--id-explosion-warn", type=float, default=0.5)
    ap.add_argument("--reid", action="store_true",
                    help="enable ReID second-stage matching")
    ap.add_argument("--reid-weights", default=None,
                    help="path to OSNet pretrained weights (.pth)")
    ap.add_argument("--reid-model", default="osnet_x1_0",
                    help="OSNet variant (osnet_x0_25 / osnet_x0_5 / osnet_x1_0)")
    ap.add_argument("--reid-weight", type=float, default=None,
                    help="weight of ReID cost in second-stage fusion (0..1)")
    ap.add_argument("--cmc-method", choices=["sparse_of", "ecc"], default="sparse_of")
    ap.add_argument("--save-reid", action="store_true",
                    help="write tracks_reid.json with per-track embeddings")
    ap.add_argument("--no-video", action="store_true",
                    help="don't write tracked.mp4")
    ap.add_argument("--log-level", default="INFO")
    return ap


def _apply_drone_sugar(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    """When --drone is set, override CLI defaults that the user didn't explicitly set."""
    if not args.drone:
        return
    drone_overrides = {
        "imgsz": 480,
        "conf": 0.12,
        "min_box_area": 60.0,
        "high_conf": 0.22,
        "new_track_conf": 0.07,
        "iou_thresh": 0.20,
        "lost_relink_frames": 45,
        "classes": "0,1,2,3,4,5,7,8",
        "weights": "yolov8s.pt",
    }
    parser_defaults = {
        action.dest: action.default
        for action in parser._actions
        if action.dest not in ("help",) and action.default is not None
    }
    for k, v in drone_overrides.items():
        if getattr(args, k, None) == parser_defaults.get(k):
            setattr(args, k, v)
    print(f"[demo] drone preset applied: weights={os.path.basename(args.weights)} "
          f"imgsz={args.imgsz} conf={args.conf} "
          f"min_box_area={args.min_box_area} high>={args.high_conf} "
          f"new>={args.new_track_conf} iou>={args.iou_thresh} "
          f"relink<={args.lost_relink_frames} classes={args.classes}")


def _args_to_overrides(args: argparse.Namespace) -> Dict[str, Any]:
    """Translate CLI flags to a config-overlay dict matching the YAML schema."""
    out: Dict[str, Any] = {}
    detector: Dict[str, Any] = {}
    if args.imgsz is not None:
        detector["imgsz"] = args.imgsz
    if args.conf is not None:
        detector["conf"] = args.conf
    if args.classes is not None:
        try:
            detector["classes"] = [int(c) for c in args.classes.split(",") if c.strip()]
        except ValueError as exc:
            raise SystemExit(f"bad --classes value {args.classes!r}: {exc}")
    if args.min_box_area is not None:
        detector["min_box_area"] = args.min_box_area
    if args.min_conf is not None:
        detector["min_conf"] = args.min_conf
    if args.nms_iou is not None:
        detector["nms_iou"] = args.nms_iou
    if detector:
        out["detector"] = detector

    tracker: Dict[str, Any] = {}
    if args.tracker is not None:
        tracker["kind"] = args.tracker
    if args.high_conf is not None:
        tracker["high_conf"] = args.high_conf
    if args.new_track_conf is not None:
        tracker["new_track_conf"] = args.new_track_conf
    if args.iou_thresh is not None:
        tracker["iou_thresh"] = args.iou_thresh
    if args.lost_relink_frames is not None:
        tracker["lost_relink_frames"] = args.lost_relink_frames
    if args.max_age is not None:
        tracker["max_age"] = args.max_age
    if args.n_init is not None:
        tracker["n_init"] = args.n_init
    if args.no_cmc:
        tracker["cmc"] = False
    tracker["stationary_prune"] = not args.no_stationary_prune
    tracker["cmc_method"] = args.cmc_method
    if tracker:
        out["tracker"] = tracker

    appearance: Dict[str, Any] = {}
    if args.reid or (args.tracker == "deepsort_cascade"):
        # The cascade matcher benefits from any appearance signal; auto-enable
        # when the user asked for cascade even if they didn't pass --reid.
        appearance["enabled"] = True
    if args.reid_weights:
        appearance["weights"] = args.reid_weights
    if args.reid_model:
        appearance["model"] = args.reid_model
    if args.reid_weight is not None:
        appearance["match_weight"] = args.reid_weight
    if appearance:
        out["appearance"] = appearance

    viz: Dict[str, Any] = {}
    if args.save_trail:
        viz["save_trail"] = True
        viz["save_trails_json"] = True
    if viz:
        out["viz"] = viz

    output: Dict[str, Any] = {}
    if args.no_video:
        output["write_video"] = False
    if args.save_reid:
        output["write_reid_json"] = True
    if output:
        out["output"] = output

    pipeline: Dict[str, Any] = {}
    if args.max_frames:
        pipeline["max_frames"] = args.max_frames
    if args.max_seconds:
        pipeline["max_seconds"] = args.max_seconds
    if args.start_frame:
        pipeline["start_frame"] = args.start_frame
    if args.include_tentative:
        pipeline["include_tentative"] = True
    if args.id_explosion_warn != 0.5:
        pipeline["id_explosion_warn"] = args.id_explosion_warn
    if pipeline:
        out["pipeline"] = out.get("pipeline", {})
        out["pipeline"].update(pipeline)
    return out


def _resolve_config(args: argparse.Namespace) -> Config:
    """Pick config file from --config / --drone / bundled default."""
    if args.drone and not args.config:
        # Resolve to bundled drone.yaml.  Use the package's configs/ dir.
        from cvtrack.config import _configs_dir
        drone_path = os.path.join(_configs_dir(), "drone.yaml")
        if os.path.isfile(drone_path):
            args.config = drone_path
    if args.config:
        cfg = load_config(args.config)
    else:
        cfg = load_config("default")
    return cfg


def _classes_from_config(raw_classes: Any) -> List[int]:
    if raw_classes is None:
        return [0, 2, 5, 7, 16]
    if isinstance(raw_classes, str):
        return [int(c) for c in raw_classes.split(",") if c.strip()]
    return [int(c) for c in raw_classes]


def run(args: argparse.Namespace) -> int:
    """Execute the tracking pipeline.  Returns process exit code (0)."""
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = _build_parser()
    _apply_drone_sugar(args, parser)

    cfg = _resolve_config(args)
    overlay = _args_to_overrides(args)
    merged = merge_cli(cfg.raw, overlay)
    # Backfill None-valued CLI overrides back into the raw dict so the rest
    # of the code can read them as authoritative.
    for k, v in overlay.items():
        if isinstance(v, dict):
            merged.setdefault(k, {}).update(v)
        else:
            merged[k] = v

    # Top-level scalar overrides.
    if args.device:
        merged.setdefault("detector", {})["device"] = args.device
    if args.detector and args.detector != "auto":
        merged.setdefault("detector", {})["backend"] = args.detector
    if args.weights:
        merged["detector"]["weights"] = args.weights
    merged.setdefault("detector", {})["weights"] = resolve_weights(
        merged["detector"].get("weights", "yolov8s.pt")
    )

    # Re-validate after merge (cheap).
    from cvtrack.config import _validate
    _validate(merged)

    os.makedirs(args.out_dir, exist_ok=True)

    # -------- source ---------------------------------------------------
    source = args.source
    if not source:
        source = os.path.join(args.out_dir, "sample.mp4")
        if not os.path.exists(source):
            sys.path.insert(0, args.out_dir)
            try:
                from make_sample_video import generate
                log.info("no source given, synthesising %s", source)
                generate(args.out_dir)
            except Exception as exc:
                raise SystemExit(f"no --source and sample synthesis failed: {exc}")

    reader = VideoReader(source)
    info = reader.info()
    w, h = info.width, info.height
    fps = info.fps
    total = info.total_frames

    dt = 1.0 / max(float(fps), 1.0)
    pipe_cfg = merged.get("pipeline", {})
    max_frames_cap = int(pipe_cfg.get("max_frames", args.max_frames) or 0)
    if max_frames_cap <= 0:
        max_frames_cap = 10 ** 9
    if pipe_cfg.get("max_seconds"):
        max_frames_cap = min(max_frames_cap, int(pipe_cfg["max_seconds"] * fps))
    start_frame = int(pipe_cfg.get("start_frame", args.start_frame) or 0)
    log.info(
        "source=%s fps=%.1f size=%dx%d dt=%.3fs total=%d cap=%d",
        source, fps, w, h, dt, total, max_frames_cap,
    )

    # -------- detector -------------------------------------------------
    det_cfg = merged["detector"]
    classes = _classes_from_config(det_cfg.get("classes"))
    res_class = detect_resolution_class(w, h)
    min_area = float(det_cfg.get("min_box_area", 300.0))
    conf_eff = float(det_cfg.get("conf", 0.15))
    drone_active = bool(args.drone)
    if drone_active and res_class == "large":
        # Same adaptive rescale as v4.
        baseline_px = 640 * 360
        actual_px = w * h
        scale = max(actual_px / baseline_px, 1.0)
        min_area = max(min_area, 60.0 * scale)
        conf_eff = max(conf_eff, 0.20)
        log.info("drone-rescale: input=%dx%d -> min_box_area=%.0f conf>=%.2f",
                 w, h, min_area, conf_eff)

    backend = det_cfg.get("backend", args.detector)
    if backend in (None, "", "auto"):
        backend = "auto"
    detector = make_detector(
        backend=backend,
        weights=det_cfg.get("weights"),
        device=det_cfg.get("device", "cpu"),
        conf=conf_eff,
        classes=classes,
        imgsz=int(det_cfg.get("imgsz", 320)),
        min_box_area=min_area,
        min_conf=float(det_cfg.get("min_conf", 0.0)),
        nms_iou=float(det_cfg.get("nms_iou", 0.5)),
    )
    model_name = getattr(detector, "model_name", "detector")

    # -------- appearance (optional) ------------------------------------
    appearance_cfg = merged.get("appearance", {})
    reid_enabled = bool(appearance_cfg.get("enabled", False))
    reid_extractor = None
    if reid_enabled:
        try:
            from cvtrack.appearance.factory import make_extractor
            reid_extractor = make_extractor(
                "osnet",
                weights=appearance_cfg.get("weights"),
                model_name=appearance_cfg.get("model", "osnet_x1_0"),
                device=det_cfg.get("device", "cpu"),
            )
        except Exception as exc:
            log.warning("ReID extractor failed to build: %s -- disabling", exc)
            reid_extractor = None
    reid_min_side = int(appearance_cfg.get("min_box_side", 8))
    reid_match_weight = float(appearance_cfg.get("match_weight", 0.7))
    reid_galleries: Dict[int, Gallery] = {}
    reid_gallery_size = int(appearance_cfg.get("gallery_size", 50))
    reid_ema_alpha = float(appearance_cfg.get("ema_alpha", 0.05))

    # -------- tracker --------------------------------------------------
    tr_cfg = merged["tracker"]
    use_cmc = bool(tr_cfg.get("cmc", True)) and not args.no_cmc
    kind = tr_cfg.get("kind", "botsort")
    if kind == "botsort":
        tracker: Any = BoTSortTracker(
            dt=dt,
            max_age=int(tr_cfg.get("max_age", 30)),
            n_init=int(tr_cfg.get("n_init", 3)),
            stationary_prune=bool(tr_cfg.get("stationary_prune", True)),
            use_cmc=use_cmc,
            iou_thresh=float(tr_cfg.get("iou_thresh", 0.30)),
            high_conf=float(tr_cfg.get("high_conf", 0.35)),
            new_track_conf=float(tr_cfg.get("new_track_conf", 0.20)),
            lost_relink_frames=int(tr_cfg.get("lost_relink_frames", 30)),
            cmc_method=str(tr_cfg.get("cmc_method", args.cmc_method)),
            appearance_reid_weight=(reid_match_weight if (reid_enabled and reid_extractor)
                                    else 0.0),
        )
        log.info("tracker=BoT-SORT cmc=%s high>=%.2f new>=%.2f iou>=%.2f relink<=%df",
                 "on" if use_cmc else "off",
                 tracker.high_conf, tracker.new_track_conf,
                 tracker.iou_thresh, tracker.lost_relink_frames)
    elif kind == "deepsort_cascade":
        # The v6 true DeepSORT matcher.  Falls back to IoU-only if no
        # appearance extractor is available (the gallery remains empty
        # but the cascade still runs on Mahalanobis + IoU).
        use_appearance = bool(reid_enabled and reid_extractor)
        tracker = DeepSortCascade(
            dt=dt,
            max_age=int(tr_cfg.get("max_age", 30)),
            n_init=int(tr_cfg.get("n_init", 3)),
            stationary_prune=bool(tr_cfg.get("stationary_prune", True)),
            use_appearance=use_appearance,
            appearance_thresh=float(tr_cfg.get("appearance_thresh", 0.5)),
            iou_thresh=float(tr_cfg.get("iou_thresh", 0.30)),
        )
        log.info(
            "tracker=DeepSortCascade use_appearance=%s appearance_thresh=%.2f iou>=%.2f",
            use_appearance, tracker.appearance_thresh, tracker.iou_thresh,
        )
    else:
        tracker = DeepSortLite(
            dt=dt,
            max_age=int(tr_cfg.get("max_age", 20)),
            n_init=int(tr_cfg.get("n_init", 3)),
            stationary_prune=bool(tr_cfg.get("stationary_prune", True)),
        )
        log.info("tracker=DeepSortLite (legacy Mahalanobis + KF4)")
    model_name = getattr(tracker, "model_name", model_name)

    # If the user explicitly disabled CMC, drop the GMC instance.
    if isinstance(tracker, BoTSortTracker) and not use_cmc:
        tracker.gmc = None

    # -------- writers --------------------------------------------------
    out_cfg = merged.get("output", {})
    csv_path = os.path.join(args.out_dir, "tracks.csv")
    csv_w = TrackCsvWriter(csv_path)
    future_csv_path = os.path.join(args.out_dir, "tracks_future.csv")
    write_future_csv = bool(args.write_future_csv or out_cfg.get("write_future_csv", False))
    future_csv_w = FutureTrailCsvWriter(future_csv_path) if write_future_csv else None
    future_steps = max(0, int(getattr(args, "predict_horizon", 15) or 0))

    writer: Optional[VideoWriter] = None
    if out_cfg.get("write_video", True) and not args.no_video:
        writer = VideoWriter(
            os.path.join(args.out_dir, "tracked.mp4"), fps=fps, size=(w, h)
        )

    track_birth: Dict[int, int] = {}
    seen_track_ids: set = set()
    frame_idx = 0
    fps_avg = 0.0
    n_frames = 0
    vx_idx = 4 if isinstance(tracker, BoTSortTracker) else 2
    vy_idx = 5 if isinstance(tracker, BoTSortTracker) else 3

    # -------- main loop ------------------------------------------------
    if start_frame > 0:
        reader.set_pos(start_frame)
    try:
        while True:
            ok, frame = reader.read()
            if not ok:
                break
            t0 = time.time()
            detections = detector(frame)

            # ReID embeddings (per detection).
            det_embeddings: List[Optional[np.ndarray]] = [None] * len(detections)
            if reid_extractor is not None:
                for j, d in enumerate(detections):
                    if min(d.w, d.h) >= reid_min_side:
                        emb = reid_extractor(frame, (d.x1, d.y1, d.x2, d.y2))
                        det_embeddings[j] = emb

            if isinstance(tracker, BoTSortTracker):
                tracks = tracker.step(frame, detections, det_embeddings=det_embeddings)
            elif isinstance(tracker, DeepSortCascade):
                # Cascade accepts embeddings + galleries and uses both for the
                # matching cascade; the per-track embedding_mean on each
                # returned Track is updated in-place when an embedding matches.
                tracks = tracker.step(
                    detections,
                    det_embeddings=det_embeddings,
                    galleries=reid_galleries,
                )
            else:
                tracks = tracker.step(detections)

            # Refresh galleries and per-track embedding means.
            # The matching between tracks and detections happens inside the
            # tracker; here we approximate by associating the new embeddings
            # to tracks by nearest-centre distance (cheap O(N*M) but N is
            # the number of detections, typically tens per frame).
            if reid_extractor is not None:
                for t in tracks:
                    if not det_embeddings:
                        continue
                    best_idx = -1
                    best_d = float("inf")
                    for j, d in enumerate(detections):
                        if det_embeddings[j] is None:
                            continue
                        dd = (d.cx - t.pos[0]) ** 2 + (d.cy - t.pos[1]) ** 2
                        if dd < best_d:
                            best_d = dd
                            best_idx = j
                    if best_idx >= 0:
                        # Only update if the match is plausibly close
                        # (inside ~1 box width).
                        max_d2 = (max(t.box.w, t.box.h) ** 2) * 4.0
                        if best_d <= max_d2:
                            g = reid_galleries.get(t.track_id)
                            if g is None:
                                g = Gallery(size=reid_gallery_size, ema_alpha=reid_ema_alpha)
                                reid_galleries[t.track_id] = g
                            g.add(det_embeddings[best_idx])
                            t.embedding_mean = g.mean

            fps_inst = 1.0 / max(time.time() - t0, 1e-6)
            fps_avg = (fps_avg * n_frames + fps_inst) / (n_frames + 1)
            n_frames += 1

            include_tent = bool(pipe_cfg.get("include_tentative", args.include_tentative))
            visible = tracks if include_tent else [t for t in tracks if t.confirmed]
            for t in visible:
                if t.track_id not in track_birth:
                    track_birth[t.track_id] = frame_idx
                if t.confirmed:
                    seen_track_ids.add(t.track_id)

                # Future projection (with covariance when --write-future-csv).
                if write_future_csv:
                    steps = predict_n_steps_with_covariance(
                        tracker.kf, t.mean, t.cov, future_steps
                    )
                    cov_blocks = [step_cov[:2, :2] for _, step_cov in steps]
                    t.future_trail = [(float(m[0]), float(m[1])) for m, _ in steps]
                    csv_points = []
                    for (m, _), cov in zip(steps, cov_blocks):
                        sx = float(np.sqrt(max(cov[0, 0], 0.0)))
                        sy = float(np.sqrt(max(cov[1, 1], 0.0)))
                        csv_points.append((float(m[0]), float(m[1]), sx, sy))
                    if future_csv_w is not None:
                        future_csv_w.write_trail_with_cov(
                            frame_idx, t.track_id, csv_points
                        )
                    draw_predicted_future_trail(
                        frame, t, n=future_steps, cov_steps=cov_blocks,
                    )
                else:
                    t.future_trail = predict_n_steps(tracker.kf, t, future_steps)
                    if future_csv_w is not None:
                        future_csv_w.write_trail(
                            frame_idx, t.track_id, t.future_trail
                        )
                    draw_predicted_future_trail(frame, t, n=future_steps)

                draw_box(frame, t)
                if out_cfg.get("save_trail", False) or args.save_trail:
                    draw_trail(frame, t)
                csv_w.write_row(
                    frame_idx, t.track_id, t.label,
                    t.pos[0], t.pos[1],
                    float(t.mean[vx_idx]), float(t.mean[vy_idx]),
                    t.confirmed,
                )

            if out_cfg.get("fps_overlay", True):
                add_overlay(frame, fps_avg, len(visible), model_name)
            if writer is not None:
                writer.write(frame)

            frame_idx += 1
            if frame_idx >= max_frames_cap:
                break
    finally:
        reader.close()
        csv_w.close()
        if future_csv_w is not None:
            future_csv_w.close()
        if writer is not None:
            writer.close()

    log.info("wrote %s (%d frames, avg fps %.1f)", os.path.join(args.out_dir, "tracked.mp4"),
             frame_idx, fps_avg)
    log.info("tracks -> %s", csv_path)
    log.info("future tracks -> %s", future_csv_path)

    # -------- warning / optional exports ------------------------------
    n_ids = len(seen_track_ids)
    if frame_idx > 0:
        ratio = n_ids / frame_idx
        warn_threshold = float(pipe_cfg.get("id_explosion_warn", args.id_explosion_warn))
        if ratio > warn_threshold:
            log.warning(
                "track-id explosion: %d confirmed IDs in %d frames (ratio=%.2f > %.2f). "
                "Detection is too noisy OR association is too strict. "
                "Try --imgsz 480, --min-box-area 60, or --classes to widen the set; "
                "or --drone.", n_ids, frame_idx, ratio, warn_threshold,
            )

    if args.save_trail or out_cfg.get("save_trail", False):
        _export_smoothed(tracker.tracks, track_birth,
                         os.path.join(args.out_dir, "tracks_smoothed.csv"))
        _export_trails(tracker.tracks, track_birth,
                       os.path.join(args.out_dir, "tracks_trails.json"))

    if (args.save_reid or out_cfg.get("write_reid_json", False)) and reid_extractor is not None:
        _export_reid(reid_galleries,
                     os.path.join(args.out_dir, "tracks_reid.json"))

    return 0


# ---------------------------------------------------------------------------
# Optional exports
# ---------------------------------------------------------------------------
def _export_smoothed(tracks: List[Track], birth: Dict[int, int], out_path: str) -> None:
    rows = []
    for tid, tr in sorted({t.track_id: t for t in tracks}.items()):
        smooth = rts_smooth_2d(tr.pred_trail)
        bf = birth.get(tid, 0)
        for i, ((cx, cy), (sx, sy)) in enumerate(zip(tr.pred_trail, smooth)):
            vx = float(tr.mean[4]) if tr.mean.shape[0] >= 6 else float(tr.mean[2])
            vy = float(tr.mean[5]) if tr.mean.shape[0] >= 6 else float(tr.mean[3])
            rows.append((tid, bf + i, tr.label, cx, cy, sx, sy, vx, vy,
                         int(tr.confirmed)))
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["track_id", "frame", "label", "cx", "cy",
                    "cx_smooth", "cy_smooth", "vx", "vy", "confirmed"])
        w.writerows(rows)
    log.info("smoothed -> %s", out_path)


def _export_trails(tracks: List[Track], birth: Dict[int, int], out_path: str) -> None:
    out = []
    for tid, tr in sorted({t.track_id: t for t in tracks}.items()):
        out.append({
            "track_id": int(tid),
            "label": tr.label,
            "birth_frame": int(birth.get(tid, 0)),
            "total_frames": len(tr.pred_trail),
            "confirmed": bool(tr.confirmed),
            "predicted": [[float(x), float(y)] for x, y in tr.pred_trail],
        })
    with open(out_path, "w") as f:
        json.dump(out, f)
    log.info("trails -> %s", out_path)


def _export_reid(galleries: Dict[int, Gallery], out_path: str) -> None:
    out = []
    for tid, g in sorted(galleries.items()):
        embs = list(g._buf)  # FIFO buffer of L2-normalised embeddings
        b64_list = [base64.b64encode(np.ascontiguousarray(e, dtype=np.float32).tobytes()).decode()
                    for e in embs]
        mean = g.mean
        mean_b64 = (
            base64.b64encode(np.ascontiguousarray(mean, dtype=np.float32).tobytes()).decode()
            if mean is not None else None
        )
        out.append({
            "track_id": int(tid),
            "count": len(embs),
            "mean_embedding_b32": mean_b64,
            "embeddings_b32": b64_list,
        })
    with open(out_path, "w") as f:
        json.dump(out, f)
    log.info("reid -> %s", out_path)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def main(argv: Optional[List[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))