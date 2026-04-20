import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from maze_solver_interfaces.msg import DistanceRegions


class MazeDiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__('maze_diagnostics_node')

        self.declare_parameter('regions_topic', '/distance_regions')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/maze_solver/goal_reached')
        self.declare_parameter('status_topic', '/maze_solver/status')
        self.declare_parameter('collision_threshold', 0.14)
        self.declare_parameter('goal_x', 5.4)
        self.declare_parameter('goal_y', 1.8)
        self.declare_parameter('goal_radius', 0.35)

        self.collision_threshold = float(self.get_parameter('collision_threshold').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_radius = float(self.get_parameter('goal_radius').value)

        self.start_time = self.get_clock().now()
        self.collision_seen = False
        self.goal_seen = False
        self.last_regions: Optional[DistanceRegions] = None

        self.goal_pub = self.create_publisher(Bool, str(self.get_parameter('goal_topic').value), 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter('status_topic').value), 10)

        self.create_subscription(
            DistanceRegions,
            str(self.get_parameter('regions_topic').value),
            self.regions_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            10,
        )
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Diagnostics node ready. Tracking collisions and solve time.')

    def regions_callback(self, msg: DistanceRegions) -> None:
        self.last_regions = msg
        minimum_distance = min(msg.front, msg.left, msg.right)
        if minimum_distance <= self.collision_threshold and not self.collision_seen:
            self.collision_seen = True
            self.get_logger().warn(
                f'Collision watchdog triggered. Minimum regional distance {minimum_distance:.3f} m.'
            )

    def odom_callback(self, msg: Odometry) -> None:
        if self.goal_seen:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        distance_to_goal = math.hypot(self.goal_x - x, self.goal_y - y)
        if distance_to_goal <= self.goal_radius:
            self.goal_seen = True
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.goal_pub.publish(Bool(data=True))
            self.get_logger().info(f'Maze solved in {elapsed:.2f} s at pose ({x:.2f}, {y:.2f}).')

    def publish_status(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        front = self.last_regions.front if self.last_regions is not None else float('nan')
        status = String()
        status.data = (
            f'elapsed={elapsed:.1f}s '
            f'collision={str(self.collision_seen).lower()} '
            f'goal={str(self.goal_seen).lower()} '
            f'front={front:.2f}'
        )
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MazeDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
