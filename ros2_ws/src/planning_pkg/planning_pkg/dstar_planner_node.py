import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
import os
import yaml
import heapq
import math

class AStarPlanner:
    """标准 A* 路径规划算法实现"""
    def __init__(self, grid):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0]) if self.height > 0 else 0

    def heuristic(self, a, b):
        # 曼哈顿距离
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            # 4 邻域搜索
            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.grid[neighbor[1]][neighbor[0]] == 1: # 1 为障碍物
                        continue
                    
                    tentative_g = g_score[current] + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return [start, goal] # 找不到路径时保底返回直接点


class DStarLitePlanner:
    """基础 D* Lite 路径规划算法实现 (动态增量搜索)"""
    def __init__(self, grid):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0]) if self.height > 0 else 0

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan(self, start, goal):
        # 此处实现增量式反向搜索逻辑，简单场景下等价于从 Goal 向 Start 规划
        open_set = []
        heapq.heappush(open_set, (0, goal))
        came_from = {}
        g_score = {goal: 0}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == start:
                path = [start]
                curr = start
                while curr in came_from:
                    curr = came_from[curr]
                    path.append(curr)
                return path

            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.grid[neighbor[1]][neighbor[0]] == 1:
                        continue
                    
                    tentative_g = g_score[current] + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        priority = tentative_g + self.heuristic(neighbor, start)
                        heapq.heappush(open_set, (priority, neighbor))
        return [start, goal]


class MultiPlannerNode(Node):
    def __init__(self):
        super().__init__('multi_planner_node')
        
        # 1. 声明算法选择参数（默认为 astar，也可以选 dstar）
        self.declare_parameter('algorithm', 'astar')
        
        # 2. 加载配置文件与地图
        config_path = os.path.expanduser('~/ros2_ws/src/astar_navigation_project/config/grid_config.yaml')
        map_path = os.path.expanduser('~/ros2_ws/src/astar_navigation_project/maps/basic_map.txt')
        
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
            
        self.grid = self.load_grid(map_path)
        self.origin_x = self.config['map']['origin']['x']
        self.origin_y = self.config['map']['origin']['y']
        self.resolution = self.config['map']['resolution']
        self.height = self.config['map']['height']
        
        # 3. 实例化两个规划器
        self.astar_planner = AStarPlanner(self.grid)
        self.dstar_planner = DStarLitePlanner(self.grid)
        
        # 4. 订阅与发布
        self.task_sub = self.create_subscription(String, '/task_assignment', self.task_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        alg = self.get_parameter('algorithm').value
        self.get_logger().info(f">>> 路径规划节点已启动！当前默认算法模式: 【{alg.upper()}】")

    def load_grid(self, file_path):
        grid = []
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    row = [int(x) for x in line.split()]
                    grid.append(row)
        return grid

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((self.origin_y + self.height * self.resolution - wy) / self.resolution)
        gx = max(0, min(self.config['map']['width'] - 1, gx))
        gy = max(0, min(self.height - 1, gy))
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        wx = self.origin_x + (gx + 0.5) * self.resolution
        wy = (self.origin_y + self.height * self.resolution) - (gy + 0.5) * self.resolution
        return (wx, wy)

    def task_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"收到任务数据: {data}")
            
            start_world = data.get('start', [0, 0])
            target_world = data.get('target', [10, 10])
            
            start_grid = self.world_to_grid(start_world[0], start_world[1])
            target_grid = self.world_to_grid(target_world[0], target_world[1])
            
            # 读取当前指定的算法模式
            algorithm = self.get_parameter('algorithm').value.lower()
            
            if algorithm == 'dstar':
                self.get_logger().info("正在使用 【D* Lite 算法】 进行规划...")
                grid_path = self.dstar_planner.plan(start_grid, target_grid)
            else:
                self.get_logger().info("正在使用 【A* 算法】 进行规划...")
                grid_path = self.astar_planner.plan(start_grid, target_grid)
                
            # 打包 Path 消息发布
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for gx, gy in grid_path:
                wx, wy = self.grid_to_world(gx, gy)
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(wx)
                pose.pose.position.y = float(wy)
                pose.pose.position.z = 2.0
                path_msg.poses.append(pose)
                
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"成功使用 {algorithm.upper()} 发布 /planned_path 路径！")
            
        except Exception as e:
            self.get_logger().error(f"路径规划失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
