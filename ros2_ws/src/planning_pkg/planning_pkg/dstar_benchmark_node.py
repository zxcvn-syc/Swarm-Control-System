#!/usr/bin/env python3
"""D* Lite 随机障碍物响应耗时 benchmark 节点（并入 planning_pkg）。

测量在随机障碍物地图上 ``DStarLite.plan()`` 的重新规划响应耗时，
每轮随机生成 5~15 个障碍物（避开起点/终点半径 2），共 10 轮取平均。

用法::

    ros2 run planning_pkg dstar_benchmark
    ros2 run planning_pkg dstar_benchmark \\
        --ros-args -p output_path:=/abs/path/result.txt

参数:
    output_path (str, 默认 "dstar_benchmark_result.txt")
        结果输出文件路径（支持相对/绝对路径）。
"""

import random
import time

import numpy as np
import rclpy
from rclpy.node import Node

from .dstar_lite import DStarLite


class DStarBenchmarkNode(Node):
    def __init__(self):
        super().__init__('dstar_benchmark_node')

        self.declare_parameter('output_path', 'dstar_benchmark_result.txt')
        self.output_path = self.get_parameter('output_path').value

        self.grid_size = 50
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        self.start = (5, 5)
        self.goal = (self.grid_size - 5, self.grid_size - 5)

        # DStarLite(grid, start, goal) —— 仓库 planning_pkg 的实现
        self.dstar = DStarLite(self.grid, self.start, self.goal)

        self.timings = []
        self.round = 0
        self.max_rounds = 10
        self._done = False

        self.get_logger().info("=== D*Lite 随机障碍物响应耗时测试开始 (共10轮) ===")
        self.timer = self.create_timer(1.0, self.run_test)

    def run_test(self):
        if self.round >= self.max_rounds:
            avg_time = sum(self.timings) / len(self.timings)
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"✅ 测试完成！平均响应耗时: {avg_time*1000:.3f} 毫秒")
            self.get_logger().info(f"详细耗时 (毫秒): {[round(t*1000, 3) for t in self.timings]}")

            with open(self.output_path, "w") as f:
                f.write(f"Average: {avg_time*1000:.6f} ms\n")
                for i, t in enumerate(self.timings):
                    f.write(f"Round {i+1}: {t*1000:.6f} ms\n")
            self.get_logger().info(f"结果已保存至: {self.output_path}")

            self.destroy_timer(self.timer)
            self._done = True
            return

        # 重置地图并随机生成障碍物
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        num_obs = random.randint(5, 15)
        placed = 0
        attempts = 0
        while placed < num_obs and attempts < 500:
            x = random.randint(0, self.grid_size - 1)
            y = random.randint(0, self.grid_size - 1)

            if (abs(x - self.start[0]) <= 2 and abs(y - self.start[1]) <= 2) or \
               (abs(x - self.goal[0]) <= 2 and abs(y - self.goal[1]) <= 2):
                attempts += 1
                continue

            if self.grid[x][y] == 0:
                self.grid[x][y] = 1
                placed += 1
            attempts += 1

        self.dstar = DStarLite(self.grid, self.start, self.goal)

        start_time = time.perf_counter()
        path = self.dstar.plan()
        end_time = time.perf_counter()

        duration = end_time - start_time
        self.timings.append(duration)
        self.round += 1

        path_len = len(path) if path else 0
        self.get_logger().info(
            f"Round {self.round:2d}/{self.max_rounds} | "
            f"障碍物数量: {placed:2d} | "
            f"响应耗时: {duration*1000:8.3f} ms | "
            f"路径长度: {path_len:3d}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DStarBenchmarkNode()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
