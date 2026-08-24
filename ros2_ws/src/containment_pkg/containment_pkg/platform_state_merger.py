"""Merge UAV and UGV state topics into one DroneStateArray stream.

The RflySim scene bridge (rfly_ros_scene.py) publishes platform states on
two separate topics:

* ``/drone_pose_external``  -- UAV states (platform_type = 0)
* ``/ground_vehicle_states`` -- UGV states (platform_type = 1)

``enclosure_node`` subscribes to a single ``/drone_states`` topic and needs
both platform families in one message so its layered Voronoi assignment can
split platforms by ``platform_type`` (monitor layer for UAVs, block layer
for UGVs).  This node subscribes to both sources, caches the latest sample
of each, and republishes the union on ``/drone_states`` at a fixed period.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import DroneStateArray


class PlatformStateMerger(Node):
    """Merge two DroneStateArray streams into a single output topic."""

    def __init__(self) -> None:
        super().__init__("platform_state_merger")
        self.declare_parameter("uav_topic", "/drone_pose_external")
        self.declare_parameter("ugv_topic", "/ground_vehicle_states")
        self.declare_parameter("output_topic", "/drone_states")
        self.declare_parameter("publish_period", 0.25)

        uav_topic = str(self.get_parameter("uav_topic").value).strip()
        ugv_topic = str(self.get_parameter("ugv_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        period = max(float(self.get_parameter("publish_period").value), 0.05)

        self._uav_states: list = []
        self._ugv_states: list = []

        self._uav_sub = self.create_subscription(
            DroneStateArray, uav_topic, self.on_uav, 10
        )
        self._ugv_sub = self.create_subscription(
            DroneStateArray, ugv_topic, self.on_ugv, 10
        )
        self._pub = self.create_publisher(DroneStateArray, output_topic, 10)
        self._timer = self.create_timer(period, self.publish_merged)

        self.get_logger().info(
            f"platform_state_merger: {uav_topic} + {ugv_topic} -> {output_topic} "
            f"(period={period}s)"
        )

    def on_uav(self, message: DroneStateArray) -> None:
        self._uav_states = list(getattr(message, "drones", []))

    def on_ugv(self, message: DroneStateArray) -> None:
        self._ugv_states = list(getattr(message, "drones", []))

    def publish_merged(self) -> None:
        if not self._uav_states and not self._ugv_states:
            return
        merged = DroneStateArray()
        merged.drones = list(self._uav_states) + list(self._ugv_states)
        merged.num_drones = len(merged.drones)
        self._pub.publish(merged)

    @property
    def uav_count(self) -> int:
        return len(self._uav_states)

    @property
    def ugv_count(self) -> int:
        return len(self._ugv_states)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlatformStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
