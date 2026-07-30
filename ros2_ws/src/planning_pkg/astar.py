#!/usr/bin/env python3

import argparse
import heapq
import math
from pathlib import Path

import yaml


def load_grid(file_path):
    """从txt文件读取栅格地图。"""

    grid = []

    with open(file_path, "r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            try:
                row = [int(value) for value in line.split()]
            except ValueError as error:
                raise ValueError(
                    f"地图第{line_number}行包含非整数内容"
                ) from error

            grid.append(row)

    if not grid:
        raise ValueError("地图文件为空")

    width = len(grid[0])

    for row_number, row in enumerate(grid, start=1):
        if len(row) != width:
            raise ValueError(
                f"地图第{row_number}行的列数不一致"
            )

    return grid


def load_config(file_path):
    """读取YAML配置文件。"""

    with open(file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def manhattan(node, goal, resolution):
    """计算曼哈顿距离。"""

    dx = abs(node[0] - goal[0])
    dy = abs(node[1] - goal[1])

    return (dx + dy) * resolution


def get_neighbors(node, grid, obstacle_value):
    """返回当前节点的四个可通行邻居。"""

    x, y = node

    height = len(grid)
    width = len(grid[0])

    directions = [
        (1, 0),     # 右
        (-1, 0),    # 左
        (0, 1),     # 下
        (0, -1),    # 上
    ]

    neighbors = []

    for dx, dy in directions:
        new_x = x + dx
        new_y = y + dy

        inside_map = (
            0 <= new_x < width
            and 0 <= new_y < height
        )

        if not inside_map:
            continue

        if grid[new_y][new_x] == obstacle_value:
            continue

        neighbors.append((new_x, new_y))

    return neighbors


def reconstruct_path(came_from, current):
    """根据父节点关系反向恢复路径。"""

    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()

    return path


def astar(grid, start, goal, resolution, obstacle_value):
    """执行A*搜索。"""

    # OPEN优先队列：
    # 每个元素格式为(f值, 插入顺序, 节点)
    open_heap = []

    # 记录节点的父节点
    came_from = {}

    # 从起点到每个节点的最小已知代价
    g_score = {
        start: 0.0
    }

    # 已经正式扩展过的节点
    closed_set = set()

    sequence = 0

    start_f = manhattan(
        start,
        goal,
        resolution,
    )

    heapq.heappush(
        open_heap,
        (start_f, sequence, start),
    )

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current in closed_set:
            continue

        # 终点被取出，说明已经找到路径
        if current == goal:
            path = reconstruct_path(
                came_from,
                current,
            )

            return (
                path,
                len(closed_set),
                g_score[current],
            )

        closed_set.add(current)

        neighbors = get_neighbors(
            current,
            grid,
            obstacle_value,
        )

        for neighbor in neighbors:
            if neighbor in closed_set:
                continue

            # 相邻栅格距离为一个resolution
            tentative_g = (
                g_score[current]
                + resolution
            )

            old_g = g_score.get(
                neighbor,
                math.inf,
            )

            # 发现了一条更短路径
            if tentative_g < old_g:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                h_score = manhattan(
                    neighbor,
                    goal,
                    resolution,
                )

                f_score = tentative_g + h_score

                sequence += 1

                heapq.heappush(
                    open_heap,
                    (
                        f_score,
                        sequence,
                        neighbor,
                    ),
                )

    # OPEN为空仍未到达终点
    return None, len(closed_set), math.inf


def grid_to_world(
    point,
    map_height,
    resolution,
    origin_x,
    origin_y,
):
    """把栅格坐标转换为Gazebo世界坐标。"""

    grid_x, grid_y = point

    world_x = (
        origin_x
        + (grid_x + 0.5) * resolution
    )

    # 地图第一行在顶部，因此需要翻转y
    world_y = (
        origin_y
        + (map_height - grid_y - 0.5)
        * resolution
    )

    return world_x, world_y


def save_path_result(
    path,
    output_file,
    map_height,
    resolution,
    origin_x,
    origin_y,
    expanded_nodes,
    total_cost,
):
    """保存路径文本结果。"""

    output_file.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with open(
        output_file,
        "w",
        encoding="utf-8",
    ) as file:

        file.write("# A*路径规划结果\n")
        file.write(
            f"# 路径节点数: {len(path)}\n"
        )
        file.write(
            f"# 路径总代价(m): "
            f"{total_cost:.3f}\n"
        )
        file.write(
            f"# 扩展节点数: "
            f"{expanded_nodes}\n"
        )

        file.write(
            "# index,grid_x,grid_y,"
            "world_x,world_y\n"
        )

        for index, point in enumerate(path):

            world_x, world_y = grid_to_world(
                point,
                map_height,
                resolution,
                origin_x,
                origin_y,
            )

            file.write(
                f"{index},"
                f"{point[0]},"
                f"{point[1]},"
                f"{world_x:.3f},"
                f"{world_y:.3f}\n"
            )


def save_path_image(
    grid,
    path,
    start,
    goal,
    image_file,
):
    """绘制并保存路径图片。"""

    import matplotlib.pyplot as plt

    image_file.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    path_x = [
        point[0] for point in path
    ]

    path_y = [
        point[1] for point in path
    ]

    plt.figure(figsize=(7, 7))

    # 0显示为空地，1显示为障碍物
    plt.imshow(
        grid,
        cmap="gray_r",
        origin="upper",
    )

    # 绘制规划路径
    plt.plot(
        path_x,
        path_y,
        marker="o",
        linewidth=2,
        markersize=3,
        label="A* Path",
    )

    # 绘制起点
    plt.scatter(
        [start[0]],
        [start[1]],
        marker="s",
        s=100,
        label="Start",
    )

    # 绘制终点
    plt.scatter(
        [goal[0]],
        [goal[1]],
        marker="*",
        s=160,
        label="Goal",
    )

    plt.xticks(
        range(len(grid[0]))
    )

    plt.yticks(
        range(len(grid))
    )

    plt.grid(
        True,
        linewidth=0.4,
    )

    plt.xlabel("Grid x")
    plt.ylabel("Grid y")
    plt.title("A* Path Planning Result")
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        image_file,
        dpi=180,
    )

    plt.close()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="A*栅格路径规划"
    )

    parser.add_argument(
        "--map",
        required=True,
        type=Path,
        help="栅格地图文件",
    )

    parser.add_argument(
        "--config",
        required=True,
        type=Path,
        help="YAML配置文件",
    )

    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="路径文本输出文件",
    )

    parser.add_argument(
        "--image",
        required=True,
        type=Path,
        help="路径图片输出文件",
    )

    return parser.parse_args()


def main():
    args = parse_arguments()

    grid = load_grid(args.map)
    config = load_config(args.config)

    map_height = len(grid)
    map_width = len(grid[0])

    configured_width = int(
        config["map"]["width"]
    )

    configured_height = int(
        config["map"]["height"]
    )

    if (
        map_width != configured_width
        or map_height != configured_height
    ):
        raise ValueError(
            "地图实际尺寸与配置文件不一致"
        )

    resolution = float(
        config["map"]["resolution"]
    )

    obstacle_value = int(
        config["map"]["obstacle_value"]
    )

    origin_x = float(
        config["map"]["origin"]["x"]
    )

    origin_y = float(
        config["map"]["origin"]["y"]
    )

    start = (
        int(config["start"]["x"]),
        int(config["start"]["y"]),
    )

    goal = (
        int(config["goal"]["x"]),
        int(config["goal"]["y"]),
    )

    # 检查起点和终点
    for name, point in [
        ("起点", start),
        ("终点", goal),
    ]:
        x, y = point

        if not (
            0 <= x < map_width
            and 0 <= y < map_height
        ):
            raise ValueError(
                f"{name}{point}超出地图范围"
            )

        if grid[y][x] == obstacle_value:
            raise ValueError(
                f"{name}{point}位于障碍物内"
            )

    path, expanded_nodes, total_cost = astar(
        grid,
        start,
        goal,
        resolution,
        obstacle_value,
    )

    if path is None:
        raise RuntimeError(
            "规划失败：起点和终点之间不存在路径"
        )

    save_path_result(
        path,
        args.output,
        map_height,
        resolution,
        origin_x,
        origin_y,
        expanded_nodes,
        total_cost,
    )

    save_path_image(
        grid,
        path,
        start,
        goal,
        args.image,
    )

    print("A*规划成功")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"路径节点数: {len(path)}")
    print(
        f"路径总代价: "
        f"{total_cost:.3f} m"
    )
    print(
        f"扩展节点数: "
        f"{expanded_nodes}"
    )
    print(
        f"路径文本: "
        f"{args.output}"
    )
    print(
        f"路径图片: "
        f"{args.image}"
    )

    print("栅格路径:")
    print(path)


if __name__ == "__main__":
    main()

