"""A* path-planner on a 2D occupancy grid.

Pure-numpy / pure-python implementation with no ROS2 dependency, so it
can be unit tested standalone and driven from the ROS2 node in
:mod:`planning_pkg.planner_node`, or executed directly via CLI.
"""

from __future__ import annotations

import argparse
import heapq
import math
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import numpy as np
import yaml


# ===========================================================================
# 1. Core Algorithm (Subdirectory Version)
# ===========================================================================

def _is_passable(grid: np.ndarray, x: int, y: int) -> bool:
    """Return ``True`` if ``(x, y)`` lies inside the grid and is free."""
    h, w = grid.shape
    if 0 <= x < w and 0 <= y < h:
        return int(grid[y, x]) == 0
    return False


def _nearest_free(
    grid: np.ndarray, x: int, y: int, search_radius: int = 6
) -> Optional[Tuple[int, int]]:
    """Find the nearest free cell to ``(x, y)`` within ``search_radius``."""
    h, w = grid.shape
    for r in range(0, search_radius + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if abs(dx) != r and abs(dy) != r:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and int(grid[ny, nx]) == 0:
                    return (nx, ny)
    return None


def _heuristic(a: Tuple[int, int], b: Tuple[int, int], diagonal: bool) -> float:
    """Heuristic cost between ``a`` and ``b``."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if diagonal:
        return float(np.hypot(dx, dy))
    return float(dx + dy)


def _neighbors(
    x: int, y: int, diagonal: bool
) -> Iterable[Tuple[int, int, float]]:
    """Yield ``(nx, ny, step_cost)`` for valid neighbour offsets."""
    if not diagonal:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            yield x + dx, y + dy, 1.0
        return
    # Octile / 8-neighbour
    for dx, dy in (
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1),
    ):
        nx, ny = x + dx, y + dy
        step = 1.0 if dx == 0 or dy == 0 else float(np.sqrt(2.0))
        yield nx, ny, step


def _reconstruct(came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
    """Walk the ``came_from`` chain backwards."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(
    grid: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    diagonal: bool = True,
    return_stats: bool = False,
):
    """Run A* on ``grid`` from ``start`` to ``goal``.
    
    If `return_stats` is True, returns: (path, expanded_nodes, total_grid_cost)
    Otherwise, strictly returns `path` (List of Tuples) to maintain legacy ROS 2 compatibility.
    """
    grid = np.asarray(grid)
    if grid.ndim != 2 or grid.shape[0] == 0 or grid.shape[1] == 0:
        raise ValueError(f"grid must be a non-empty 2D array, got shape {grid.shape}")

    h, w = grid.shape

    def _coerce(point: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        x, y = int(point[0]), int(point[1])
        swapped = (y, x) if 0 <= y < w and 0 <= x < h and not (0 <= x < w and 0 <= y < h) else None
        if not (0 <= x < w and 0 <= y < h):
            if swapped is not None and 0 <= swapped[0] < w and 0 <= swapped[1] < h:
                x, y = swapped
            else:
                raise ValueError(f"point {(int(point[0]), int(point[1]))} outside grid shape {grid.shape}")
        return x, y

    s = _coerce(start)
    g = _coerce(goal)
    assert s is not None and g is not None

    if s == g:
        return ([s], 0, 0.0) if return_stats else [s]

    if not _is_passable(grid, *s):
        s_recover = _nearest_free(grid, *s)
        if s_recover is None:
            return ([], 0, 0.0) if return_stats else []
        s = s_recover
    if not _is_passable(grid, *g):
        g_recover = _nearest_free(grid, *g)
        if g_recover is None:
            return ([], 0, 0.0) if return_stats else []
        g = g_recover

    g_score: dict = {s: 0.0}
    f_score: dict = {s: _heuristic(s, g, diagonal)}
    came_from: dict = {}
    counter = 0
    open_heap: list = [(f_score[s], 0.0, counter, s)]
    counter += 1
    closed: set = set()

    while open_heap:
        _, _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == g:
            path = _reconstruct(came_from, current)
            return (path, len(closed), g_score[current]) if return_stats else path

        closed.add(current)
        cur_g = g_score[current]

        for nx, ny, step in _neighbors(*current, diagonal=diagonal):
            if not _is_passable(grid, nx, ny):
                continue
            neighbour = (nx, ny)
            if neighbour in closed:
                continue
            tentative_g = cur_g + step
            if tentative_g < g_score.get(neighbour, float("inf")):
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                f = tentative_g + _heuristic(neighbour, g, diagonal)
                f_score[neighbour] = f
                heapq.heappush(open_heap, (f, -tentative_g, counter, neighbour))
                counter += 1

    return ([], len(closed), float("inf")) if return_stats else []


# ===========================================================================
# 2. CLI, IO, & Data Transform Functions (Root Directory Version)
# ===========================================================================

def load_grid(file_path, obstacle_value: int) -> np.ndarray:
    """从txt文件读取栅格地图并转换为底层的 NumPy 数组。"""
    grid = []
    with open(file_path, "r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                row = [int(value) for value in line.split()]
            except ValueError as error:
                raise ValueError(f"地图第{line_number}行包含非整数内容") from error
            grid.append(row)

    if not grid:
        raise ValueError("地图文件为空")
    
    width = len(grid[0])
    for row_number, row in enumerate(grid, start=1):
        if len(row) != width:
            raise ValueError(f"地图第{row_number}行的列数不一致")

    # Normalize map for NumPy algorithm: 0 = free, 1 = obstacle
    grid_array = np.zeros((len(grid), width), dtype=np.int8)
    for y in range(len(grid)):
        for x in range(width):
            if grid[y][x] == obstacle_value:
                grid_array[y, x] = 1
                
    return grid_array


def load_config(file_path):
    """读取YAML配置文件。"""
    with open(file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def grid_to_world(point, map_height, resolution, origin_x, origin_y):
    """把栅格坐标转换为Gazebo世界坐标。"""
    grid_x, grid_y = point
    world_x = origin_x + (grid_x + 0.5) * resolution
    world_y = origin_y + (map_height - grid_y - 0.5) * resolution
    return world_x, world_y


def save_path_result(path, output_file, map_height, resolution, origin_x, origin_y, expanded_nodes, total_cost):
    """保存路径文本结果。"""
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, "w", encoding="utf-8") as file:
        file.write("# A*路径规划结果\n")
        file.write(f"# 路径节点数: {len(path)}\n")
        file.write(f"# 路径总代价(m): {total_cost:.3f}\n")
        file.write(f"# 扩展节点数: {expanded_nodes}\n")
        file.write("# index,grid_x,grid_y,world_x,world_y\n")

        for index, point in enumerate(path):
            world_x, world_y = grid_to_world(
                point, map_height, resolution, origin_x, origin_y
            )
            file.write(
                f"{index},{point[0]},{point[1]},{world_x:.3f},{world_y:.3f}\n"
            )


def save_path_image(grid, path, start, goal, image_file):
    """绘制并保存路径图片。"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed, skipping image generation.")
        return

    image_file.parent.mkdir(parents=True, exist_ok=True)
    path_x = [point[0] for point in path]
    path_y = [point[1] for point in path]

    plt.figure(figsize=(7, 7))
    plt.imshow(grid, cmap="gray_r", origin="upper")
    plt.plot(path_x, path_y, marker="o", linewidth=2, markersize=3, label="A* Path")
    plt.scatter([start[0]], [start[1]], marker="s", s=100, label="Start")
    plt.scatter([goal[0]], [goal[1]], marker="*", s=160, label="Goal")
    
    plt.xticks(range(grid.shape[1]))
    plt.yticks(range(grid.shape[0]))
    plt.grid(True, linewidth=0.4)
    plt.xlabel("Grid x")
    plt.ylabel("Grid y")
    plt.title("A* Path Planning Result")
    plt.legend()
    plt.tight_layout()
    plt.savefig(image_file, dpi=180)
    plt.close()


def parse_arguments():
    parser = argparse.ArgumentParser(description="A*栅格路径规划")
    parser.add_argument("--map", type=Path, help="栅格地图文件")
    parser.add_argument("--config", type=Path, help="YAML配置文件")
    parser.add_argument("--output", type=Path, help="路径文本输出文件")
    parser.add_argument("--image", type=Path, help="路径图片输出文件")
    return parser.parse_args()


# ===========================================================================
# 3. Main Execution & Testing
# ===========================================================================

def _self_test() -> None:
    """A small smoke test runnable when no CLI args are provided."""
    print("ast._self_test: starting")

    # 1) Straight free path on a 5x5 grid.
    grid_free = np.zeros((5, 5), dtype=np.int8)
    path = astar(grid_free, (0, 0), (4, 4))
    assert path[0] == (0, 0) and path[-1] == (4, 4), path
    assert len(path) == 5, f"diagonal path expected 5 nodes, got {len(path)}: {path}"
    print("  [OK]  straight diagonal path")

    # 2) Wall in the middle that only blocks the diagonal; route should detour.
    grid_wall = np.zeros((6, 6), dtype=np.int8)
    grid_wall[2, 1:5] = 1 
    path = astar(grid_wall, (0, 0), (5, 5))
    assert path[0] == (0, 0) and path[-1] == (5, 5), path
    for x, y in path:
        if y == 2:
            assert x in (0, 5), f"path crosses the wall at (x={x}, y={y}): {path}"
    print(f"  [OK]  obstacle detour ({len(path)} nodes)")

    print("ast._self_test: all legacy checks passed.")


def main():
    args = parse_arguments()
    
    # Fallback to internal test if no arguments are provided to support legacy usage
    if not (args.map and args.config and args.output and args.image):
        print("Missing CLI arguments. Falling back to internal self-test...\n")
        _self_test()
        return

    config = load_config(args.config)
    obstacle_value = int(config["map"]["obstacle_value"])
    resolution = float(config["map"]["resolution"])
    origin_x = float(config["map"]["origin"]["x"])
    origin_y = float(config["map"]["origin"]["y"])
    
    # Load and normalize grid
    grid_array = load_grid(args.map, obstacle_value)
    map_height, map_width = grid_array.shape
    
    configured_width = int(config["map"]["width"])
    configured_height = int(config["map"]["height"])
    if map_width != configured_width or map_height != configured_height:
        raise ValueError("地图实际尺寸与配置文件不一致")

    start = (int(config["start"]["x"]), int(config["start"]["y"]))
    goal = (int(config["goal"]["x"]), int(config["goal"]["y"]))

    # We use diagonal=False to strictly mimic the Root Version's Manhattan behavior, 
    # but the new underlying engine fully supports `True` if you choose to enable it.
    path, expanded_nodes, base_cost = astar(
        grid_array, 
        start, 
        goal, 
        diagonal=False, 
        return_stats=True
    )

    if not path:
        raise RuntimeError("规划失败：起点和终点之间不存在路径 (或均位于无法脱困的障碍物内)")

    # Multiply algorithmic abstract cost by physical map resolution
    physical_total_cost = base_cost * resolution

    save_path_result(
        path, args.output, map_height, resolution, 
        origin_x, origin_y, expanded_nodes, physical_total_cost
    )

    save_path_image(grid_array, path, start, goal, args.image)

    print("A*规划成功")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"路径节点数: {len(path)}")
    print(f"路径总代价: {physical_total_cost:.3f} m")
    print(f"扩展节点数: {expanded_nodes}")
    print(f"路径文本: {args.output}")
    print(f"路径图片: {args.image}")
    print("栅格路径:")
    print(path)


if __name__ == "__main__":
    main()

