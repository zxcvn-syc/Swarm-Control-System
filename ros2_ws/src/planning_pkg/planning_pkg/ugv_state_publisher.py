"""Pure ROS2 differential-drive UGV state simulator."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import DroneState, DroneStateArray, TaskAssignment


class UGVStatePublisher(Node):
    """Simulate configurable UGVs and publish them as platform-aware states."""

    def __init__(self) -> None:
        super().__init__("ugv_state_publisher")
        self.declare_parameter("num_ugv", 2)
        self.declare_parameter("initial_positions", [2.0, 2.0, 4.0, 2.0])
        self.declare_parameter("initial_heading", 0.0)
        self.declare_parameter("update_period", 0.1)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("state_topic", "/drone_states")
        self.declare_parameter("task_topic", "/task_assignment")

        self.num_ugv = max(0, int(self.get_parameter("num_ugv").value))
        positions = list(self.get_parameter("initial_positions").value or [])
        heading = float(self.get_parameter("initial_heading").value)
        self.max_speed = max(0.0, float(self.get_parameter("max_speed").value))
        self.max_angular_speed = max(0.0, float(self.get_parameter("max_angular_speed").value))
        self.states: Dict[int, List[float]] = {}
        self.targets: Dict[int, Tuple[float, float]] = {}
        for index in range(self.num_ugv):
            x = float(positions[2 * index]) if 2 * index < len(positions) else 2.0 + index
            y = float(positions[2 * index + 1]) if 2 * index + 1 < len(positions) else 2.0
            self.states[index] = [x, y, heading, 0.0, 0.0]

        self.task_sub = self.create_subscription(
            TaskAssignment, str(self.get_parameter("task_topic").value), self.on_task, 10
        )
        self.state_pub = self.create_publisher(
            DroneStateArray, str(self.get_parameter("state_topic").value), 10
        )
        self.timer = self.create_timer(max(0.01, float(self.get_parameter("update_period").value)), self.tick)

    def on_task(self, message: TaskAssignment) -> None:
        """Treat the assignment target id as a deterministic demo coordinate."""
        ugv_id = int(message.drone_id)
        if ugv_id not in self.states:
            return
        target_id = int(message.target_id)
        self.targets[ugv_id] = (float(target_id % 100), float(target_id // 100))

    def tick(self) -> None:
        """Advance each vehicle using x'=v cos(theta), y'=v sin(theta)."""
        dt = max(0.01, float(self.get_parameter("update_period").value))
        for ugv_id, state in self.states.items():
            x, y, theta, _, _ = state
            target = self.targets.get(ugv_id)
            if target is not None:
                dx, dy = target[0] - x, target[1] - y
                desired = math.atan2(dy, dx)
                error = math.atan2(math.sin(desired - theta), math.cos(desired - theta))
                omega = max(-self.max_angular_speed, min(self.max_angular_speed, error / dt))
                speed = self.max_speed if abs(error) < math.pi / 2 else 0.0
                if math.hypot(dx, dy) < self.max_speed * dt:
                    speed = 0.0
            else:
                omega = 0.0
                speed = 0.0
            state[2] = theta + omega * dt
            state[0] = x + speed * math.cos(state[2]) * dt
            state[1] = y + speed * math.sin(state[2]) * dt
            state[3], state[4] = speed, omega

        message = DroneStateArray()
        message.num_drones = self.num_ugv
        message.drones = []
        for ugv_id, (x, y, theta, speed, omega) in self.states.items():
            state = DroneState()
            state.drone_id = ugv_id
            state.x, state.y, state.z = x, y, 0.0
            state.vx, state.vy, state.vz = speed * math.cos(theta), speed * math.sin(theta), 0.0
            state.available = True
            state.platform_type = 1
            message.drones.append(state)
        self.state_pub.publish(message)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = UGVStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
