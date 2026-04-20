import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .control_core import PredictiveWallFollower
from maze_solver_interfaces.msg import DistanceRegions


class MazeLogicNode(Node):
    def __init__(self) -> None:
        super().__init__('maze_logic_node')

        self.declare_parameter('regions_topic', '/distance_regions')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('desired_wall_distance', 0.45)
        self.declare_parameter('wall_detect_threshold', 0.8)
        self.declare_parameter('front_block_threshold', 0.6)
        self.declare_parameter('turn_exit_front_threshold', 0.9)
        self.declare_parameter('front_clearance_target', 1.2)
        self.declare_parameter('base_linear_speed', 0.22)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('search_linear_speed', 0.08)
        self.declare_parameter('search_turn_speed', 0.55)
        self.declare_parameter('turn_linear_speed', 0.04)
        self.declare_parameter('turn_angular_speed', 0.9)
        self.declare_parameter('max_angular_speed', 1.4)
        self.declare_parameter('max_speed_reduction', 0.22)
        self.declare_parameter('predictive_gain', 0.35)
        self.declare_parameter('angular_kp', 2.4)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.25)
        self.declare_parameter('angular_integral_limit', 0.6)
        self.declare_parameter('linear_kp', 0.9)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.05)
        self.declare_parameter('linear_integral_limit', 0.4)

        config = {name: self.get_parameter(name).value for name in [
            'desired_wall_distance',
            'wall_detect_threshold',
            'front_block_threshold',
            'turn_exit_front_threshold',
            'front_clearance_target',
            'base_linear_speed',
            'min_linear_speed',
            'max_linear_speed',
            'search_linear_speed',
            'search_turn_speed',
            'turn_linear_speed',
            'turn_angular_speed',
            'max_angular_speed',
            'max_speed_reduction',
            'predictive_gain',
            'angular_kp',
            'angular_ki',
            'angular_kd',
            'angular_integral_limit',
            'linear_kp',
            'linear_ki',
            'linear_kd',
            'linear_integral_limit',
        ]}

        self.controller = PredictiveWallFollower(config)
        self.publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )
        self.subscription = self.create_subscription(
            DistanceRegions,
            str(self.get_parameter('regions_topic').value),
            self.regions_callback,
            10,
        )
        self.get_logger().info('Maze logic node ready. Waiting for regional distance data.')

    def regions_callback(self, msg: DistanceRegions) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        scan = self.controller.update_scan(float(msg.front), float(msg.left), float(msg.right), stamp_sec)
        scan.left_rate = float(msg.left_rate)
        command = self.controller.compute_command(scan)
        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MazeLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()
