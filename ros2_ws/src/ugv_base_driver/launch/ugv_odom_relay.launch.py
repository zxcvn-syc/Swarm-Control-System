"""Launch the odometry relay (/odom -> /ugv_pose).

Typical full stack on the real UGV (all on the MentorPi container):

    ros2 launch ugv_base_driver ugv_odom_relay.launch.py
    ros2 launch ugv_base_driver ugv_path_follower.launch.py target_frame_id:=drone_4

The relay feeds the follower's ``/ugv_pose`` input from the vendor EKF
odometry so no external motion capture / UWB is needed for bench and
short-range field experiments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchArgument
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="nav_msgs/Odometry source (vendor ekf_filter_node)",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/ugv_pose",
                description="PoseStamped output consumed by ugv_path_follower",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Output frame_id; empty passes the odom header through",
            ),
            DeclareLaunchArgument(
                "restamp",
                default_value="true",
                description="Stamp output with this node's clock (avoids stale-clock timeouts)",
            ),
            Node(
                package="ugv_base_driver",
                executable="ugv_odom_relay",
                name="ugv_odom_relay",
                output="screen",
                parameters=[
                    {
                        "odom_topic": LaunchArgument("odom_topic"),
                        "pose_topic": LaunchArgument("pose_topic"),
                        "frame_id": LaunchArgument("frame_id"),
                        "restamp": LaunchArgument("restamp"),
                    },
                ],
            ),
        ]
    )
