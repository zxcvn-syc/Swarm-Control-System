import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class RflySimFollower(Node):
    def __init__(self):
        super().__init__('rflysim_follower_node')
        
        # 订阅规划好的路径
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        # 订阅无人机当前实际位置 (来自 RflySim/MAVROS)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        
        # 发布给 RflySim 的控制指令点
        self.target_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        self.path_waypoints = []
        self.current_idx = 0
        self.current_pose = None
        self.reach_threshold = 0.3  # 到达阈值：距离目标点 < 0.3米 视为到达下一个点
        
        # 定时器：50Hz 频率持续向 RflySim 发送当前航向点
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(">>> RflySim 轨迹跟踪节点已启动！")

    def path_callback(self, msg):
        if not msg.poses:
            return
        self.path_waypoints = msg.poses
        self.current_idx = 0
        self.get_logger().info(f"收到新路径，包含 {len(self.path_waypoints)} 个航向点，开始执行跟踪！")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def control_loop(self):
        if not self.path_waypoints or self.current_pose is None:
            return
            
        if self.current_idx < len(self.path_waypoints):
            target_pt = self.path_waypoints[self.current_idx].pose.position
            
            # 计算当前位置与目标航点的欧氏距离
            dist = math.sqrt(
                (self.current_pose.x - target_pt.x)**2 +
                (self.current_pose.y - target_pt.y)**2 +
                (self.current_pose.z - target_pt.z)**2
            )
            
            # 如果到达当前航点，切换到下一个点
            if dist < self.reach_threshold:
                self.get_logger().info(f"已到达第 {self.current_idx + 1} 个路径点")
                self.current_idx += 1
                
            # 发送指令给 RflySim
            if self.current_idx < len(self.path_waypoints):
                cmd_msg = self.path_waypoints[self.current_idx]
                cmd_msg.header.stamp = self.get_clock().now().to_msg()
                self.target_pub.publish(cmd_msg)
            else:
                self.get_logger().info(">>> 全部路径点飞完，已到达最终终点！")

def main(args=None):
    rclpy.init(args=args)
    node = RflySimFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

