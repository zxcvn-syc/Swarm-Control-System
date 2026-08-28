import csv
import time
from collections import defaultdict


INPUT_FILE = "enclosure_targets_mock.csv"


def main():


    # 按帧组织目标

    frames = defaultdict(list)


    with open(
        INPUT_FILE,
        "r",
        encoding="utf-8-sig"
    ) as f:


        reader = csv.DictReader(f)


        for row in reader:

            frame = int(row["frame"])

            frames[frame].append(row)



    print("="*60)
    print("Mock EnclosureTarget Publisher")
    print("="*60)


    print(
        "模拟发布话题: /enclosure_targets"
    )


    print(
        "总帧数:",
        len(frames)
    )


    print("="*60)



    # 模拟200帧发布

    for frame_id in sorted(frames.keys()):


        targets = frames[frame_id]


        print(
            f"\nFrame {frame_id}"
        )


        print(
            f"目标数量: {len(targets)}"
        )


        for t in targets:


            print(
                " Target:",
                t["target_id"],
                "|",
                t["class"],
                "|",
                "pos=(",
                t["x"],
                ",",
                t["y"],
                ")",
                "|",
                "conf=",
                t["confidence"]
            )



        # 模拟10Hz发布频率

        time.sleep(0.1)



    print("\n")
    print("="*60)
    print("Mock发布完成")
    print("="*60)



if __name__=="__main__":

    main()