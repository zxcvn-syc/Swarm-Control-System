import argparse
import csv
import os
import sys
from collections import defaultdict
from pathlib import Path
from typing import Sequence


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


# Frozen evidence is read by default, but output is always written under the
# code directory so a normal invocation cannot overwrite the delivery CSVs.
DELIVERY_DIR = Path(__file__).resolve().parent.parent
GT_DIR = DELIVERY_DIR / "03_tracking_gt"
TRACK_DIR = DELIVERY_DIR / "02_tracking"

SCENES = [
    "park",
    "security",
    "border"
]

IOU_THRESHOLD = 0.30

OUTPUT_FILE = DELIVERY_DIR / "04_code" / "outputs" / "evaluation" / "tracking_success_result.csv"


def parse_args(argv: Sequence[str] | None = None):

    parser = argparse.ArgumentParser(
        description="真实 YOLO + Tracker 跟踪成功率评估"
    )

    parser.add_argument(
        "--gt-dir",
        type=Path,
        default=GT_DIR,
        help="Tracking GT 目录（含 tracking_gt_<scene>.csv）"
    )

    parser.add_argument(
        "--track-dir",
        type=Path,
        default=TRACK_DIR,
        help="Tracker 输出目录（含 tracking_eval_<scene>.csv）"
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=OUTPUT_FILE,
        help="结果输出 CSV 路径（默认写入 04_code/outputs/evaluation）"
    )

    parser.add_argument(
        "--scenes",
        nargs="+",
        choices=SCENES,
        default=SCENES,
        help="要评估的场景（默认全部）"
    )

    return parser.parse_args(argv)


# ============================================================
# VisDrone类别 -> Tracking GT统一类别
# ============================================================

YOLO_CLASS_MAP = {
    "pedestrian": "person",
    "people": "person",

    "bicycle": "bicycle",

    "car": "car",
    "van": "car",

    "truck": "truck",

    "bus": "bus",

    "motor": "motorcycle",

    # 如果GT里把三轮车也标成motorcycle，可保留
    "tricycle": "motorcycle",
    "awning-tricycle": "motorcycle",
}


GT_CLASS_MAP = {
    "person": "person",
    "bicycle": "bicycle",
    "car": "car",
    "motorcycle": "motorcycle",
    "bus": "bus",
    "truck": "truck"
}


# ============================================================
# 类别名称标准化
# ============================================================

def normalize_class_name(name, source):

    name = str(name).strip().lower()

    if source == "yolo":
        return YOLO_CLASS_MAP.get(
            name,
            name
        )

    return GT_CLASS_MAP.get(
        name,
        name
    )


# ============================================================
# IoU
# ============================================================

def calculate_iou(box1, box2):

    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])

    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter_w = max(
        0.0,
        x2 - x1
    )

    inter_h = max(
        0.0,
        y2 - y1
    )

    intersection = (
        inter_w * inter_h
    )

    area1 = (
        max(0.0, box1[2] - box1[0])
        *
        max(0.0, box1[3] - box1[1])
    )

    area2 = (
        max(0.0, box2[2] - box2[0])
        *
        max(0.0, box2[3] - box2[1])
    )

    union = (
        area1
        + area2
        - intersection
    )

    if union <= 0:
        return 0.0

    return intersection / union


# ============================================================
# 读取GT
# ============================================================

def load_gt(path):

    data = defaultdict(list)

    with open(
        path,
        "r",
        encoding="utf-8-sig"
    ) as f:

        reader = csv.DictReader(f)

        for row in reader:

            frame = int(
                row["frame_index"]
            )

            original_name = (
                row["class_name"]
            )

            canonical_name = (
                normalize_class_name(
                    original_name,
                    "gt"
                )
            )

            data[frame].append({
                "gt_id": int(
                    row["gt_id"]
                ),

                "class_name":
                    original_name,

                "canonical_class":
                    canonical_name,

                "bbox": [
                    float(row["x1"]),
                    float(row["y1"]),
                    float(row["x2"]),
                    float(row["y2"])
                ]
            })

    return data


# ============================================================
# 读取Tracker
# ============================================================

def load_tracker(path):

    data = defaultdict(list)

    with open(
        path,
        "r",
        encoding="utf-8-sig"
    ) as f:

        reader = csv.DictReader(f)

        for row in reader:

            frame = int(
                row["frame_index"]
            )

            original_name = (
                row["class_name"]
            )

            canonical_name = (
                normalize_class_name(
                    original_name,
                    "yolo"
                )
            )

            data[frame].append({
                "track_id": int(
                    row["track_id"]
                ),

                "class_name":
                    original_name,

                "canonical_class":
                    canonical_name,

                "confidence": float(
                    row["confidence"]
                ),

                "bbox": [
                    float(row["x1"]),
                    float(row["y1"]),
                    float(row["x2"]),
                    float(row["y2"])
                ]
            })

    return data


# ============================================================
# 评估单个场景
# ============================================================

def evaluate_scene(scene, gt_dir, track_dir):

    gt_path = os.path.join(
        gt_dir,
        f"tracking_gt_{scene}.csv"
    )

    tracker_path = os.path.join(
        track_dir,
        f"tracking_eval_{scene}.csv"
    )

    if not os.path.exists(gt_path):

        print(
            "找不到GT:",
            gt_path
        )

        return None

    if not os.path.exists(
        tracker_path
    ):

        print(
            "找不到Tracker:",
            tracker_path
        )

        return None


    gt_data = load_gt(
        gt_path
    )

    tracker_data = load_tracker(
        tracker_path
    )


    total_gt = 0
    matched = 0
    missed = 0

    gt_history = defaultdict(list)


    # ========================================================
    # 逐帧匹配
    # ========================================================

    for frame in sorted(
        gt_data.keys()
    ):

        gt_objects = (
            gt_data[frame]
        )

        tracker_objects = (
            tracker_data.get(
                frame,
                []
            )
        )

        used_track_ids = set()


        for gt in gt_objects:

            total_gt += 1

            best_tracker = None
            best_iou = 0.0


            for tr in tracker_objects:

                if (
                    tr["track_id"]
                    in used_track_ids
                ):
                    continue


                # ============================================
                # 关键修改：
                # 不再比较原始class_id
                # 而比较映射后的统一类别名称
                # ============================================

                if (
                    tr["canonical_class"]
                    !=
                    gt["canonical_class"]
                ):
                    continue


                score = calculate_iou(
                    gt["bbox"],
                    tr["bbox"]
                )


                if score > best_iou:

                    best_iou = score
                    best_tracker = tr


            # ================================================
            # 匹配成功
            # ================================================

            if (
                best_tracker is not None
                and
                best_iou >= IOU_THRESHOLD
            ):

                matched += 1

                track_id = (
                    best_tracker[
                        "track_id"
                    ]
                )

                used_track_ids.add(
                    track_id
                )

                gt_history[
                    gt["gt_id"]
                ].append({
                    "frame": frame,
                    "track_id":
                        track_id,
                    "iou":
                        best_iou
                })


            # ================================================
            # 漏跟
            # ================================================

            else:

                missed += 1

                gt_history[
                    gt["gt_id"]
                ].append({
                    "frame": frame,
                    "track_id":
                        None,
                    "iou":
                        0
                })


    # ========================================================
    # ID Switch
    # ========================================================

    id_switches = 0

    target_results = []


    for gt_id, history in (
        gt_history.items()
    ):

        history = sorted(
            history,
            key=lambda x: x["frame"]
        )

        total_frames = len(
            history
        )

        matched_frames = sum(
            1
            for item in history
            if item["track_id"]
            is not None
        )

        target_success = (
            matched_frames
            / total_frames
            * 100
            if total_frames > 0
            else 0
        )


        target_switches = 0

        last_track_id = None


        for item in history:

            current_id = (
                item["track_id"]
            )

            if current_id is None:
                continue


            if last_track_id is None:

                last_track_id = (
                    current_id
                )

                continue


            if current_id != last_track_id:

                target_switches += 1
                id_switches += 1

                last_track_id = (
                    current_id
                )


        matched_ious = [
            item["iou"]
            for item in history
            if item["track_id"]
            is not None
        ]


        avg_iou = (
            sum(matched_ious)
            /
            len(matched_ious)
            if matched_ious
            else 0
        )


        target_results.append({
            "gt_id":
                gt_id,

            "total_frames":
                total_frames,

            "matched_frames":
                matched_frames,

            "success_rate":
                target_success,

            "id_switches":
                target_switches,

            "avg_iou":
                avg_iou
        })


    # ========================================================
    # 场景指标
    # ========================================================

    success_rate = (
        matched
        / total_gt
        * 100
        if total_gt > 0
        else 0
    )

    miss_rate = (
        missed
        / total_gt
        * 100
        if total_gt > 0
        else 0
    )


    return {
        "scene":
            scene,

        "gt_targets":
            len(gt_history),

        "gt_instances":
            total_gt,

        "matched_instances":
            matched,

        "missed_instances":
            missed,

        "tracking_success_rate_percent":
            round(
                success_rate,
                2
            ),

        "miss_rate_percent":
            round(
                miss_rate,
                2
            ),

        "id_switches":
            id_switches,

        "targets":
            target_results
    }


# ============================================================
# 主程序
# ============================================================

def main(argv: Sequence[str] | None = None):

    args = parse_args(argv)

    gt_dir = args.gt_dir
    track_dir = args.track_dir

    output_file = args.output

    print("=" * 70)
    print(
        "真实 YOLO + Tracker 跟踪成功率评估"
    )
    print(
        "已启用 VisDrone -> GT 类别映射"
    )
    print("=" * 70)

    print(
        "IoU阈值:",
        IOU_THRESHOLD
    )


    results = []


    for scene in args.scenes:

        result = evaluate_scene(
            scene,
            gt_dir,
            track_dir
        )

        if result is None:
            continue

        results.append(
            result
        )


        print()
        print("=" * 60)
        print("场景:", scene)

        print(
            "GT目标数:",
            result["gt_targets"]
        )

        print(
            "GT实例数:",
            result["gt_instances"]
        )

        print(
            "成功匹配:",
            result[
                "matched_instances"
            ]
        )

        print(
            "漏跟:",
            result[
                "missed_instances"
            ]
        )

        print(
            f"跟踪成功率: "
            f"{result['tracking_success_rate_percent']:.2f}%"
        )

        print(
            f"漏跟率: "
            f"{result['miss_rate_percent']:.2f}%"
        )

        print(
            "ID Switch:",
            result["id_switches"]
        )


        print()
        print("各GT目标结果:")

        for t in result["targets"]:

            print(
                f"GT_ID={t['gt_id']} | "
                f"{t['matched_frames']}/"
                f"{t['total_frames']} | "
                f"成功率="
                f"{t['success_rate']:.2f}% | "
                f"ID Switch="
                f"{t['id_switches']} | "
                f"Avg IoU="
                f"{t['avg_iou']:.3f}"
            )


    # ========================================================
    # 总体
    # ========================================================

    total_gt = sum(
        r["gt_instances"]
        for r in results
    )

    total_matched = sum(
        r["matched_instances"]
        for r in results
    )

    total_missed = sum(
        r["missed_instances"]
        for r in results
    )

    total_switches = sum(
        r["id_switches"]
        for r in results
    )


    overall_success = (
        total_matched
        / total_gt
        * 100
        if total_gt > 0
        else 0
    )


    overall_miss = (
        total_missed
        / total_gt
        * 100
        if total_gt > 0
        else 0
    )


    print()
    print("=" * 70)
    print("总体结果")
    print("=" * 70)

    print(
        "GT实例总数:",
        total_gt
    )

    print(
        "成功跟踪实例:",
        total_matched
    )

    print(
        "漏跟实例:",
        total_missed
    )

    print(
        f"总体跟踪成功率: "
        f"{overall_success:.2f}%"
    )

    print(
        f"总体漏跟率: "
        f"{overall_miss:.2f}%"
    )

    print(
        "总 ID Switch:",
        total_switches
    )


    # ========================================================
    # 保存
    # ========================================================

    fieldnames = [
        "scene",
        "gt_targets",
        "gt_instances",
        "matched_instances",
        "missed_instances",
        "tracking_success_rate_percent",
        "miss_rate_percent",
        "id_switches"
    ]


    Path(output_file).parent.mkdir(
        parents=True,
        exist_ok=True
    )

    with open(
        output_file,
        "w",
        newline="",
        encoding="utf-8-sig"
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames
        )

        writer.writeheader()


        for result in results:

            writer.writerow({
                key:
                    result[key]
                for key
                in fieldnames
            })


        writer.writerow({
            "scene":
                "overall",

            "gt_targets":
                sum(
                    r["gt_targets"]
                    for r in results
                ),

            "gt_instances":
                total_gt,

            "matched_instances":
                total_matched,

            "missed_instances":
                total_missed,

            "tracking_success_rate_percent":
                round(
                    overall_success,
                    2
                ),

            "miss_rate_percent":
                round(
                    overall_miss,
                    2
                ),

            "id_switches":
                total_switches
        })


    print()
    print("新结果保存到：")
    print(output_file)


if __name__ == "__main__":
    main()
