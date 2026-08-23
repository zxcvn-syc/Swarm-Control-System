import rclpy
from rclpy.node import Node
import random
import time
import numpy as np

from .dstar_lite import DStarLite

class DStarBenchmarkNode(Node):
    def __init__(self):
        super().__init__('dstar_benchmark_node')
        
        self.grid_size = 50
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        
        self.start = (5, 5)
        self.goal = (self.grid_size - 5, self.grid_size - 5)
        
        # 修正：正确的顺序是 (grid, start, goal)
        self.dstar = DStarLite(self.grid, self.start, self.goal)
        
        self.timings = []
        self.round = 0
        self.max_rounds = 10
        
        self.get_logger().info("=== D*Lite 随机障碍物响应耗时测试开始 (共10轮) ===")
        self.timer = self.create_timer(1.0, self.run_test)

    def run_test(self):
        if self.round >= self.max_rounds:
            avg_time = sum(self.timings) / len(self.timings)
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"✅ 测试完成！平均响应耗时: {avg_time*1000:.3f} 毫秒")
            self.get_logger().info(f"详细耗时 (毫秒): {[round(t*1000, 3) for t in self.timings]}")
            
            with open("/home/hsako/ros2_ws/dstar_result.txt", "w") as f:
                f.write(f"Average: {avg_time*1000:.6f} ms\n")
                for i, t in enumerate(self.timings):
                    f.write(f"Round {i+1}: {t*1000:.6f} ms\n")
            self.get_logger().info("结果已保存至: /home/hsako/ros2_ws/dstar_result.txt")
            
            self.destroy_timer(self.timer)
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

        # 修正：正确的顺序是 (grid, start, goal)
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
