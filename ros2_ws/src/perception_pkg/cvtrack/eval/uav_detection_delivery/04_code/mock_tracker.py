import csv
import os


# ============================================================
# Mock Tracker
# 用于在没有 ROS2 / 真实 YOLO 检测链路的情况下，
# 模拟目标检测结果和 Tracker 输入，验证跟踪链路。
# ============================================================


# -----------------------------
# 基本测试参数
# -----------------------------

TOTAL_FRAMES = 200

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080

OUTPUT_FILE = "mock_targets.csv"


# -----------------------------
# Mock 目标类别
# -----------------------------

CLASS_NAMES = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    4: "bus",
    5: "truck",
}


# -----------------------------
# Mock 目标定义
# -----------------------------

MOCK_TARGETS = [
    {
        "track_id": 1,
        "class_id": 0,
        "class_name": "person",
        "x": 300.0,
        "y": 300.0,
        "vx": 5.0,
        "vy": 2.0,
        "width": 80.0,
        "height": 160.0,
    },

    {
        "track_id": 2,
        "class_id": 2,
        "class_name": "car",
        "x": 800.0,
        "y": 500.0,
        "vx": -4.0,
        "vy": 3.0,
        "width": 180.0,
        "height": 100.0,
    },

    {
        "track_id": 3,
        "class_id": 3,
        "class_name": "motorcycle",
        "x": 1200.0,
        "y": 700.0,
        "vx": -3.0,
        "vy": -2.0,
        "width": 100.0,
        "height": 80.0,
    },
]


def update_target(target):
    """
    更新目标位置。

    使用简单的匀速运动模型。
    如果目标碰到画面边界，则反向运动。
    """

    target["x"] += target["vx"]
    target["y"] += target["vy"]

    half_width = target["width"] / 2
    half_height = target["height"] / 2

    # X方向边界
    if target["x"] - half_width <= 0:
        target["x"] = half_width
        target["vx"] = abs(target["vx"])

    elif target["x"] + half_width >= IMAGE_WIDTH:
        target["x"] = IMAGE_WIDTH - half_width
        target["vx"] = -abs(target["vx"])

    # Y方向边界
    if target["y"] - half_height <= 0:
        target["y"] = half_height
        target["vy"] = abs(target["vy"])

    elif target["y"] + half_height >= IMAGE_HEIGHT:
        target["y"] = IMAGE_HEIGHT - half_height
        target["vy"] = -abs(target["vy"])


def get_bbox(target):
    """
    根据目标中心点生成检测框。

    返回：
        x1, y1, x2, y2
    """

    x = target["x"]
    y = target["y"]

    width = target["width"]
    height = target["height"]

    x1 = max(0, x - width / 2)
    y1 = max(0, y - height / 2)
    x2 = min(IMAGE_WIDTH, x + width / 2)
    y2 = min(IMAGE_HEIGHT, y + height / 2)

    return x1, y1, x2, y2


def create_output_directory():
    """
    创建输出目录。
    """

    output_dir = os.path.dirname(os.path.abspath(OUTPUT_FILE))

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)


def run_mock_tracker():
    """
    执行 Mock Tracker 测试。
    """

    print("=" * 70)
    print("                    Mock Tracker Test")
    print("=" * 70)

    print(f"测试帧数: {TOTAL_FRAMES}")
    print(f"目标数量: {len(MOCK_TARGETS)}")
    print(f"图像尺寸: {IMAGE_WIDTH} x {IMAGE_HEIGHT}")

    print("\nMock目标:")

    for target in MOCK_TARGETS:
        print(
            f"  Track ID={target['track_id']} | "
            f"class={target['class_name']} | "
            f"start=({target['x']:.0f},{target['y']:.0f}) | "
            f"velocity=({target['vx']:.1f},{target['vy']:.1f})"
        )

    print("=" * 70)

    create_output_directory()

    # 复制目标，避免修改原始配置
    targets = []

    for target in MOCK_TARGETS:
        targets.append(target.copy())

    results = []

    # 每个目标的统计信息
    target_stats = {}

    for target in targets:
        target_stats[target["track_id"]] = {
            "expected": 0,
            "tracked": 0,
            "id_switch": 0,
            "continuous": 0,
            "max_continuous": 0,
            "last_track_id": target["track_id"],
        }

    # --------------------------------------------------------
    # 开始模拟 200 帧
    # --------------------------------------------------------

    for frame_id in range(TOTAL_FRAMES):

        for target in targets:

            # 更新目标位置
            update_target(target)

            # 当前目标理论上应该被 Tracker 跟踪
            expected_track_id = target["track_id"]

            # Mock Tracker 当前保持正确 ID
            actual_track_id = expected_track_id

            # 计算检测框
            x1, y1, x2, y2 = get_bbox(target)

            stats = target_stats[expected_track_id]

            stats["expected"] += 1
            stats["tracked"] += 1

            # 判断是否发生 ID Switch
            if (
                stats["last_track_id"] != actual_track_id
                and stats["last_track_id"] != -1
            ):
                stats["id_switch"] += 1

            stats["last_track_id"] = actual_track_id

            # 连续跟踪
            stats["continuous"] += 1

            if stats["continuous"] > stats["max_continuous"]:
                stats["max_continuous"] = stats["continuous"]

            # 保存结果
            results.append(
                {
                    "frame": frame_id + 1,
                    "track_id": actual_track_id,
                    "class_id": target["class_id"],
                    "class_name": target["class_name"],
                    "x": round(target["x"], 2),
                    "y": round(target["y"], 2),
                    "x1": round(x1, 2),
                    "y1": round(y1, 2),
                    "x2": round(x2, 2),
                    "y2": round(y2, 2),
                }
            )

    # --------------------------------------------------------
    # 保存 CSV
    # --------------------------------------------------------

    with open(
        OUTPUT_FILE,
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as f:

        fieldnames = [
            "frame",
            "track_id",
            "class_id",
            "class_name",
            "x",
            "y",
            "x1",
            "y1",
            "x2",
            "y2",
        ]

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
        )

        writer.writeheader()
        writer.writerows(results)

    # --------------------------------------------------------
    # 总体统计
    # --------------------------------------------------------

    total_expected = 0
    total_tracked = 0
    total_id_switch = 0

    for stats in target_stats.values():

        total_expected += stats["expected"]
        total_tracked += stats["tracked"]
        total_id_switch += stats["id_switch"]

    if total_expected > 0:
        tracking_success_rate = (
            total_tracked / total_expected * 100
        )
    else:
        tracking_success_rate = 0.0

    # --------------------------------------------------------
    # 输出总体结果
    # --------------------------------------------------------

    print("\n" + "=" * 70)
    print("                       测试结果")
    print("=" * 70)

    print(f"总测试帧数:       {TOTAL_FRAMES}")
    print(f"Mock目标数量:     {len(targets)}")
    print(f"理论目标帧数:     {total_expected}")
    print(f"成功跟踪数:       {total_tracked}")
    print(f"跟踪成功率:       {tracking_success_rate:.2f}%")
    print(f"ID Switch:        {total_id_switch}")

    print("\n" + "-" * 70)
    print("                       各目标结果")
    print("-" * 70)

    for target in targets:

        track_id = target["track_id"]
        stats = target_stats[track_id]

        if stats["expected"] > 0:
            success_rate = (
                stats["tracked"]
                / stats["expected"]
                * 100
            )
        else:
            success_rate = 0.0

        print(
            f"Target {track_id}: "
            f"class={target['class_name']} | "
            f"帧数={stats['expected']} | "
            f"成功={stats['tracked']} | "
            f"成功率={success_rate:.2f}% | "
            f"ID Switch={stats['id_switch']} | "
            f"最大连续跟踪={stats['max_continuous']}帧"
        )

    print("\n" + "=" * 70)
    print("                    Mock测试完成")
    print("=" * 70)

    print(f"结果文件: {os.path.abspath(OUTPUT_FILE)}")

    print("\n说明:")
    print("1. 当前使用固定 Mock 目标，不使用随机成功率。")
    print("2. 每个目标在 200 帧内保持稳定 Track ID。")
    print("3. 当前 Mock 场景未主动制造遮挡、丢失和 ID Switch。")
    print("4. 当前结果用于验证 Tracker 接口和数据格式，不代表真实检测性能。")


if __name__ == "__main__":
    run_mock_tracker()