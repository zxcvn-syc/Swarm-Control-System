from ultralytics import YOLO
import cv2
import csv
import os


MODEL_PATH = r"D:\UAV_detection\runs\detect\train\weights\best.pt"

GT_DIR = r"D:\UAV_detection\tracking_gt"
VIDEO_DIR = r"D:\UAV_detection\videos"
OUTPUT_DIR = r"D:\UAV_detection\real_tracker"

SCENES = ["park", "security", "border"]

CONF = 0.50


def get_gt_frames(gt_path):

    frames = set()

    with open(gt_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)

        for row in reader:
            frames.add(int(row["frame_index"]))

    return sorted(frames)


def process_scene(model, scene):

    gt_path = os.path.join(
        GT_DIR,
        f"tracking_gt_{scene}.csv"
    )

    video_path = os.path.join(
        VIDEO_DIR,
        f"{scene}.mp4"
    )

    output_path = os.path.join(
        OUTPUT_DIR,
        f"detections_eval_{scene}.csv"
    )

    if not os.path.exists(gt_path):
        print(f"找不到GT: {gt_path}")
        return

    if not os.path.exists(video_path):
        print(f"找不到视频: {video_path}")
        return

    frames = get_gt_frames(gt_path)

    print()
    print("=" * 60)
    print("场景:", scene)
    print("GT实际标注帧数:", len(frames))
    print("GT帧范围:", min(frames), "~", max(frames))

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("视频打开失败")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)

    fields = [
        "frame_index",
        "video_time",
        "class_id",
        "class_name",
        "confidence",
        "x1",
        "y1",
        "x2",
        "y2"
    ]

    detection_count = 0

    with open(
        output_path,
        "w",
        newline="",
        encoding="utf-8-sig"
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fields
        )

        writer.writeheader()

        for frame_index in frames:

            cap.set(
                cv2.CAP_PROP_POS_FRAMES,
                frame_index
            )

            ret, frame = cap.read()

            if not ret:
                print("读取失败:", frame_index)
                continue

            results = model.predict(
                frame,
                conf=CONF,
                verbose=False
            )

            result = results[0]

            if result.boxes is None:
                continue

            for box in result.boxes:

                class_id = int(
                    box.cls[0].item()
                )

                confidence = float(
                    box.conf[0].item()
                )

                x1, y1, x2, y2 = (
                    box.xyxy[0]
                    .cpu()
                    .tolist()
                )

                # 不手工编造类别名称
                # 直接使用模型自身的类别名称
                class_name = model.names[
                    class_id
                ]

                writer.writerow({
                    "frame_index": frame_index,
                    "video_time": round(
                        frame_index / fps,
                        4
                    ),
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": round(
                        confidence,
                        4
                    ),
                    "x1": round(x1, 2),
                    "y1": round(y1, 2),
                    "x2": round(x2, 2),
                    "y2": round(y2, 2)
                })

                detection_count += 1

    cap.release()

    print("YOLO检测框数量:", detection_count)
    print("输出:", output_path)


def main():

    print("=" * 60)
    print("Tracking GT 对齐 YOLO 检测")
    print("conf =", CONF)
    print("=" * 60)

    model = YOLO(MODEL_PATH)

    print()
    print("模型类别：")
    print(model.names)

    for scene in SCENES:
        process_scene(
            model,
            scene
        )

    print()
    print("=" * 60)
    print("GT对齐检测完成")
    print("=" * 60)


if __name__ == "__main__":
    main()