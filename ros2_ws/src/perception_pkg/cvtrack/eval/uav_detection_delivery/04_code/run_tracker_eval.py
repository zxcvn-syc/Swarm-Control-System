import csv
import os
from collections import defaultdict


# ============================================================
# 输入 / 输出
# ============================================================

BASE_DIR = r"D:\UAV_detection\real_tracker"

INPUT_FILES = {
    "park": os.path.join(BASE_DIR, "detections_eval_park.csv"),
    "security": os.path.join(BASE_DIR, "detections_eval_security.csv"),
    "border": os.path.join(BASE_DIR, "detections_eval_border.csv"),
}

OUTPUT_FILES = {
    "park": os.path.join(BASE_DIR, "tracking_eval_park.csv"),
    "security": os.path.join(BASE_DIR, "tracking_eval_security.csv"),
    "border": os.path.join(BASE_DIR, "tracking_eval_border.csv"),
}


# ============================================================
# Tracker参数
# ============================================================

IOU_THRESHOLD = 0.30

# 允许轨迹最多中断多少个“原视频帧”
# 注意：这里是按 frame_index 判断，不是按CSV行数
MAX_AGE = 10

# 类别必须一致才能匹配
MATCH_SAME_CLASS = True


# ============================================================
# IoU
# ============================================================

def calculate_iou(box1, box2):

    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])

    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)

    intersection = inter_w * inter_h

    area1 = max(
        0.0,
        box1[2] - box1[0]
    ) * max(
        0.0,
        box1[3] - box1[1]
    )

    area2 = max(
        0.0,
        box2[2] - box2[0]
    ) * max(
        0.0,
        box2[3] - box2[1]
    )

    union = area1 + area2 - intersection

    if union <= 0:
        return 0.0

    return intersection / union


# ============================================================
# 读取检测结果
# ============================================================

def load_detections(file_path):

    frames = defaultdict(list)

    with open(
        file_path,
        "r",
        encoding="utf-8-sig"
    ) as f:

        reader = csv.DictReader(f)

        required_fields = {
            "frame_index",
            "video_time",
            "class_id",
            "class_name",
            "confidence",
            "x1",
            "y1",
            "x2",
            "y2",
        }

        actual_fields = set(
            reader.fieldnames or []
        )

        missing = required_fields - actual_fields

        if missing:
            raise ValueError(
                f"\n{file_path}\n"
                f"缺少字段: {missing}\n"
                f"实际字段: {reader.fieldnames}"
            )

        for row in reader:

            frame_index = int(
                row["frame_index"]
            )

            frames[
                frame_index
            ].append({
                "frame_index": frame_index,
                "video_time": float(
                    row["video_time"]
                ),
                "class_id": int(
                    row["class_id"]
                ),
                "class_name": row[
                    "class_name"
                ],
                "confidence": float(
                    row["confidence"]
                ),
                "bbox": [
                    float(row["x1"]),
                    float(row["y1"]),
                    float(row["x2"]),
                    float(row["y2"]),
                ]
            })

    return frames


# ============================================================
# 单场景Tracker
# ============================================================

def run_scene(scene, input_path, output_path):

    print()
    print("=" * 70)
    print(f"场景：{scene}")
    print("=" * 70)

    if not os.path.exists(input_path):
        print("❌ 找不到输入文件：")
        print(input_path)
        return

    frames = load_detections(
        input_path
    )

    if not frames:
        print("❌ 检测结果为空")
        return

    # --------------------------------------------------------
    # 活跃轨迹
    # --------------------------------------------------------

    tracks = {}

    next_track_id = 1

    output_rows = []

    total_detections = 0

    # ========================================================
    # 按 frame_index 顺序处理
    # ========================================================

    for frame_index in sorted(
        frames.keys()
    ):

        detections = frames[
            frame_index
        ]

        total_detections += len(
            detections
        )

        # ----------------------------------------------------
        # 删除过期轨迹
        # ----------------------------------------------------

        expired_ids = []

        for track_id, track in tracks.items():

            frame_gap = (
                frame_index
                - track["last_frame"]
            )

            if frame_gap > MAX_AGE:
                expired_ids.append(
                    track_id
                )

        for track_id in expired_ids:
            del tracks[track_id]

        # 当前帧一个track只能匹配一个检测
        used_track_ids = set()

        # ====================================================
        # 每个检测框进行匹配
        # ====================================================

        for det in detections:

            det_box = det["bbox"]

            best_track_id = None
            best_iou = 0.0

            for track_id, track in tracks.items():

                if track_id in used_track_ids:
                    continue

                if MATCH_SAME_CLASS:

                    if (
                        track["class_id"]
                        != det["class_id"]
                    ):
                        continue

                score = calculate_iou(
                    det_box,
                    track["bbox"]
                )

                if score > best_iou:

                    best_iou = score
                    best_track_id = (
                        track_id
                    )

            # ------------------------------------------------
            # 匹配成功
            # ------------------------------------------------

            if (
                best_track_id
                is not None
                and best_iou
                >= IOU_THRESHOLD
            ):

                track_id = best_track_id

            # ------------------------------------------------
            # 新建轨迹
            # ------------------------------------------------

            else:

                track_id = next_track_id
                next_track_id += 1

            # ------------------------------------------------
            # 更新轨迹
            # ------------------------------------------------

            tracks[
                track_id
            ] = {
                "bbox": det_box,
                "class_id": det[
                    "class_id"
                ],
                "class_name": det[
                    "class_name"
                ],
                "last_frame": frame_index,
            }

            used_track_ids.add(
                track_id
            )

            x1, y1, x2, y2 = det_box

            center_x = (
                x1 + x2
            ) / 2.0

            center_y = (
                y1 + y2
            ) / 2.0

            output_rows.append({
                "scene": scene,
                "frame_index": frame_index,
                "video_time": det[
                    "video_time"
                ],
                "track_id": track_id,
                "class_id": det[
                    "class_id"
                ],
                "class_name": det[
                    "class_name"
                ],
                "confidence": det[
                    "confidence"
                ],
                "center_x": round(
                    center_x,
                    2
                ),
                "center_y": round(
                    center_y,
                    2
                ),
                "x1": round(x1, 2),
                "y1": round(y1, 2),
                "x2": round(x2, 2),
                "y2": round(y2, 2),
                "match_iou": round(
                    best_iou,
                    4
                )
            })

    # ========================================================
    # 保存结果
    # ========================================================

    fieldnames = [
        "scene",
        "frame_index",
        "video_time",
        "track_id",
        "class_id",
        "class_name",
        "confidence",
        "center_x",
        "center_y",
        "x1",
        "y1",
        "x2",
        "y2",
        "match_iou",
    ]

    with open(
        output_path,
        "w",
        newline="",
        encoding="utf-8-sig"
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames
        )

        writer.writeheader()
        writer.writerows(
            output_rows
        )

    print(
        "参与检测的帧数:",
        len(frames)
    )

    print(
        "检测框数量:",
        total_detections
    )

    print(
        "生成Track ID数量:",
        next_track_id - 1
    )

    print(
        "输出:",
        output_path
    )


# ============================================================
# 主程序
# ============================================================

def main():

    print("=" * 70)
    print(
        "GT 对齐检测结果 -> IoU Tracker"
    )
    print("=" * 70)

    print(
        "Tracker IoU阈值:",
        IOU_THRESHOLD
    )

    print(
        "MAX_AGE:",
        MAX_AGE
    )

    for scene in [
        "park",
        "security",
        "border"
    ]:

        run_scene(
            scene,
            INPUT_FILES[scene],
            OUTPUT_FILES[scene]
        )

    print()
    print("=" * 70)
    print("三个场景Tracker处理完成")
    print("=" * 70)


if __name__ == "__main__":
    main()