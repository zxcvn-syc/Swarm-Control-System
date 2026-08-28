import csv
import math
from collections import defaultdict


INPUT_FILE = "mock_targets.csv"
OUTPUT_FILE = "enclosure_targets_mock.csv"


# 预测未来5帧
PRED_STEPS = 5

# 历史轨迹长度
HISTORY_LENGTH = 10


def calculate_speed(history):

    if len(history) < 2:
        return 0.0

    x1, y1 = history[-2]
    x2, y2 = history[-1]

    distance = math.sqrt(
        (x2-x1)**2 +
        (y2-y1)**2
    )

    return round(distance, 2)


def motion_mode(speed):

    if speed < 1:
        return "static"

    elif speed < 10:
        return "slow"

    else:
        return "fast"



def main():

    targets = defaultdict(list)


    # ===============================
    # 读取Mock Tracker结果
    # ===============================

    with open(
        INPUT_FILE,
        "r",
        encoding="utf-8-sig"
    ) as f:

        reader = csv.DictReader(f)

        for row in reader:

            tid = int(row["track_id"])

            targets[tid].append(
                {
                    "frame": int(row["frame"]),
                    "class_name": row["class_name"],

                    "x": float(row["x"]),
                    "y": float(row["y"]),

                    "x1": float(row["x1"]),
                    "y1": float(row["y1"]),
                    "x2": float(row["x2"]),
                    "y2": float(row["y2"]),
                }
            )


    output=[]


    # ===============================
    # 生成 EnclosureTarget 数据
    # ===============================

    for tid, records in targets.items():


        history=[]


        for index,item in enumerate(records):

            history.append(
                (
                    item["x"],
                    item["y"]
                )
            )


            current_history = history[-HISTORY_LENGTH:]


            speed = calculate_speed(
                current_history
            )


            # -------------------------
            # 未来轨迹预测
            # -------------------------

            pred_x=[]
            pred_y=[]


            vx=0
            vy=0


            if len(current_history)>=2:

                vx = (
                    current_history[-1][0]
                    -
                    current_history[-2][0]
                )

                vy = (
                    current_history[-1][1]
                    -
                    current_history[-2][1]
                )


            for i in range(1,PRED_STEPS+1):

                pred_x.append(
                    round(
                        item["x"]+vx*i,
                        2
                    )
                )

                pred_y.append(
                    round(
                        item["y"]+vy*i,
                        2
                    )
                )


            row={

                "frame":
                    item["frame"],


                "target_id":
                    tid,


                "class":
                    item["class_name"],


                "x":
                    round(item["x"],2),


                "y":
                    round(item["y"],2),


                "speed":
                    speed,


                "motion_mode":
                    motion_mode(speed),


                "confidence":
                    0.95,


                "box_x1":
                    round(item["x1"],2),

                "box_y1":
                    round(item["y1"],2),

                "box_x2":
                    round(item["x2"],2),

                "box_y2":
                    round(item["y2"],2),


            }


            # 加入预测轨迹

            for i in range(PRED_STEPS):

                row[
                    f"pred_x{i+1}"
                ]=pred_x[i]

                row[
                    f"pred_y{i+1}"
                ]=pred_y[i]


            # 加入历史轨迹

            for i in range(HISTORY_LENGTH):

                if i < len(current_history):

                    row[
                        f"history_x{i+1}"
                    ] = current_history[i][0]

                    row[
                        f"history_y{i+1}"
                    ] = current_history[i][1]

                else:

                    row[
                        f"history_x{i+1}"
                    ] = -1

                    row[
                        f"history_y{i+1}"
                    ] = -1


            output.append(row)



    # ===============================
    # 保存
    # ===============================

    fieldnames=list(output[0].keys())


    with open(
        OUTPUT_FILE,
        "w",
        newline="",
        encoding="utf-8-sig"
    ) as f:


        writer=csv.DictWriter(
            f,
            fieldnames=fieldnames
        )

        writer.writeheader()

        writer.writerows(output)



    print("==============================")
    print("EnclosureTarget Mock生成完成")
    print("==============================")

    print(
        f"输入文件: {INPUT_FILE}"
    )

    print(
        f"输出文件: {OUTPUT_FILE}"
    )

    print(
        f"目标数量: {len(targets)}"
    )

    print(
        f"输出数据量: {len(output)}"
    )


if __name__=="__main__":

    main()