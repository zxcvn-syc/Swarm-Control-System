import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from swarm_interfaces.msg import (
    DroneState,
    DroneStateArray,
    EnclosureCommand,
    EnclosureCommandArray,
    EnclosureTargetArray,
    TargetTrackArray,
)

from .voronoi import voronoi_enclose

# Platform type constants (aligned with DroneState.msg; fall back to the
# design-spec values when running against an older message build).
PLATFORM_DRONE = int(getattr(DroneState, "PLATFORM_DRONE", 0))
PLATFORM_CAR = int(getattr(DroneState, "PLATFORM_CAR", 1))

# Enclosure layer identifiers (aligned with EnclosureCommand.msg).
LAYER_MONITOR = 0  # UAV monitor layer (outer ring)
LAYER_BLOCK = 1    # UGV block layer (inner ring)
LAYER_COMMAND = 2  # human command layer (reserved)


class EnclosureNode(Node):
    """Compute dynamic Voronoi enclosure commands from ROS2 state updates.

    Subscribes to both DroneStateArray (multi-drone batch) and PoseStamped
    (single-drone real-time position, e.g. from planning_pkg/uav_pose_pub)
    so that Voronoi regions update whenever any real platform moves.

    Three-layer containment: platforms are grouped by ``platform_type`` --
    UAVs are projected onto the monitor layer radius, UGVs onto the block
    layer radius, and the command layer is reserved for future manual
    override.  Each layer runs its own Voronoi assignment.
    """

    def __init__(self):
        super().__init__("enclosure_node")
        self.declare_parameter("enclosure_radius", 25.0)
        self.declare_parameter("monitor_radius", 0.0)
        self.declare_parameter("block_radius", 15.0)
        self.declare_parameter("min_dist", 5.0)
        self.declare_parameter("update_period", 1.0)
        self.declare_parameter("pose_topic", "/uav1/current_pose")
        self.declare_parameter("pose_drone_id", 1)

        self._targets = []
        self._batch_drones = []      # from DroneStateArray
        self._pose_drones = {}       # from PoseStamped, keyed by drone_id
        self._dirty = False
        self._last_update_time = None
        self._update_count = 0

        self._target_track_sub = self.create_subscription(
            TargetTrackArray, "/target_track", self.on_target_track, 10
        )
        self._enclosure_target_sub = self.create_subscription(
            EnclosureTargetArray, "/enclosure_targets", self.on_enclosure_targets, 10
        )
        self._drone_sub = self.create_subscription(
            DroneStateArray, "/drone_states", self.on_drone, 10
        )
        # Subscribe to real-time PoseStamped from planning_pkg / uav_pose_pub
        pose_topic = str(self.get_parameter("pose_topic").value)
        self._pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.on_pose, 10
        )
        self._publisher = self.create_publisher(
            EnclosureCommandArray, "/enclosure_command", 10
        )
        period = max(float(self.get_parameter("update_period").value), 0.01)
        self._timer = self.create_timer(period, self.tick)

    def on_target_track(self, message):
        """Use tracker output as the primary target source."""
        self._targets = list(getattr(message, "tracks", []))
        self._dirty = True

    def on_enclosure_targets(self, message):
        """Accept the legacy enclosure-target message as a fallback input."""
        self._targets = list(getattr(message, "targets", []))
        self._dirty = True

    def on_drone(self, message):
        """Receive batch drone states (DroneStateArray)."""
        self._batch_drones = self._state_list(message)
        self._dirty = True

    def on_pose(self, message):
        """Convert PoseStamped (real-time UAV position) to a DroneState.

        When the real platform moves, this callback fires and marks state
        dirty so the next tick recomputes Voronoi regions with the updated
        position -- achieving dynamic region update as the target moves.
        """
        drone_id = int(self.get_parameter("pose_drone_id").value)
        state = DroneState()
        state.drone_id = drone_id
        state.x = float(message.pose.position.x)
        state.y = float(message.pose.position.y)
        state.z = float(message.pose.position.z)
        state.vx = 0.0
        state.vy = 0.0
        state.vz = 0.0
        state.available = True
        # platform_type is optional; set only if the running DroneState msg
        # supports it (older builds of swarm_interfaces may lack the field).
        if hasattr(state, "platform_type"):
            state.platform_type = 0  # PLATFORM_DRONE
        self._pose_drones[drone_id] = state
        self._dirty = True

    # Compatibility aliases for callers using the old callback names.
    _targets_callback = on_enclosure_targets
    _drones_callback = on_drone

    @staticmethod
    def _state_list(message):
        return list(getattr(message, "drones", getattr(message, "states", [])))

    def _merged_drones(self):
        """Merge batch drones with pose overlays (pose takes priority)."""
        merged = {int(s.drone_id): s for s in self._batch_drones}
        merged.update(self._pose_drones)
        return list(merged.values())

    def tick(self):
        """Recalculate at most once per timer period when state changed."""
        if not self._dirty or not self._targets:
            return False
        drones = self._merged_drones()
        if not drones:
            return False
        self._recalculate(drones)
        return True

    @staticmethod
    def _layer_of(state):
        """Map a platform state to its enclosure layer by platform_type.

        Unknown / legacy states without the field default to the monitor
        layer so the old single-layer behaviour is preserved.
        """
        ptype = int(getattr(state, "platform_type", PLATFORM_DRONE))
        if ptype == PLATFORM_CAR:
            return LAYER_BLOCK
        if ptype == 2:  # reserved command-layer platform
            return LAYER_COMMAND
        return LAYER_MONITOR

    def _monitor_radius(self):
        """Effective monitor radius; falls back to legacy enclosure_radius."""
        value = float(self.get_parameter("monitor_radius").value)
        if value <= 0:
            value = float(self.get_parameter("enclosure_radius").value)
        return value

    def _layer_results(self, states, target_xy, radius, min_dist):
        """Voronoi enclosure points for one layer.

        Returns a list of ``(state, point, radius)`` tuples.  When a layer
        has more platforms than targets the extra platforms get ``None``
        (standby), matching the previous single-layer behaviour.
        """
        if not states:
            return []
        drone_xy = np.array([[state.x, state.y] for state in states], dtype=float)
        points, radii = voronoi_enclose(target_xy, drone_xy, radius, min_dist)
        # All platforms in a containment layer are active: they form a
        # cooperative ring around the target(s).  Standby is only used for
        # command-layer platforms awaiting manual override.
        results = []
        for index, state in enumerate(states):
            results.append((state, points[index], radii[index]))
        return results

    def _recalculate(self, states):
        targets = self._targets
        target_xy = np.array([[target.x, target.y] for target in targets], dtype=float)
        monitor_radius = self._monitor_radius()
        block_radius = float(self.get_parameter("block_radius").value)
        min_dist = float(self.get_parameter("min_dist").value)
        start = time.perf_counter()

        monitor = [s for s in states if self._layer_of(s) == LAYER_MONITOR]
        block = [s for s in states if self._layer_of(s) == LAYER_BLOCK]
        command = [s for s in states if self._layer_of(s) == LAYER_COMMAND]

        plan = []  # (state, layer, point_or_None, radius)
        for state, point, radius in self._layer_results(
            monitor, target_xy, monitor_radius, min_dist
        ):
            plan.append((state, LAYER_MONITOR, point, radius))
        for state, point, radius in self._layer_results(
            block, target_xy, block_radius, min_dist
        ):
            plan.append((state, LAYER_BLOCK, point, radius))
        # Command layer is reserved: platforms wait for manual override.
        for state in command:
            self.get_logger().debug(
                f"platform {state.drone_id}: command layer reserved, standby"
            )
            plan.append((state, LAYER_COMMAND, None, 0.0))

        elapsed_ms = (time.perf_counter() - start) * 1000.0
        self.get_logger().debug(f"Voronoi update completed in {elapsed_ms:.3f} ms")

        command_msg = EnclosureCommandArray()
        command_msg.num_drones = len(plan)
        command_msg.commands = []
        for state, layer, point, radius in plan:
            item = EnclosureCommand()
            item.drone_id = int(state.drone_id)
            item.layer = int(layer)
            if point is None:
                item.target_x = float("nan")
                item.target_y = float("nan")
                item.target_z = float("nan")
                item.enclosure_radius = 0.0
            else:
                item.target_x = float(point[0])
                item.target_y = float(point[1])
                item.target_z = float(getattr(state, "z", 0.0))
                item.enclosure_radius = float(radius)
            command_msg.commands.append(item)
        self._publisher.publish(command_msg)
        self._dirty = False
        self._last_update_time = self.get_clock().now()
        self._update_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = EnclosureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
