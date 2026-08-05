import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class UAVPosePublisher(Node):
    def __init__(self):
        super().__init__('uav_pose_publisher_node')
        
        # 订阅 RflySim/MAVROS 真实反馈位置
        self.subscription = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
            
        # 转发给陈思睿的多机调度节点
        self.publisher = self.create_publisher(
            PoseStamped, '/uav1/current_pose', 10)
            
        self.get_logger().info(">>> 实时位姿发布节点已启动，提供数据给陈思睿调度节点...")

    def pose_callback(self, msg):
        # 转发当前无人机位姿
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UAVPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

