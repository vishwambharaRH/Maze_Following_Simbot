import math
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Twist

from .pid import PIDController
from .state_machine import MazeStateMachine, RobotState


@dataclass
class ScanSnapshot:
    front: float
    left: float
    right: float
    left_rate: float
    dt: float
    stamp_sec: float


class PredictiveWallFollower:
    def __init__(self, config: dict) -> None:
        self.desired_wall_distance = config['desired_wall_distance']
        self.base_linear_speed = config['base_linear_speed']
        self.min_linear_speed = config['min_linear_speed']
        self.max_linear_speed = config['max_linear_speed']
        self.search_linear_speed = config['search_linear_speed']
        self.search_turn_speed = config['search_turn_speed']
        self.turn_linear_speed = config['turn_linear_speed']
        self.turn_angular_speed = config['turn_angular_speed']
        self.front_clearance_target = config['front_clearance_target']
        self.max_speed_reduction = config['max_speed_reduction']
        self.predictive_gain = config['predictive_gain']

        self.angular_pid = PIDController(
            kp=config['angular_kp'],
            ki=config['angular_ki'],
            kd=config['angular_kd'],
            output_min=-config['max_angular_speed'],
            output_max=config['max_angular_speed'],
            integral_limit=config['angular_integral_limit'],
        )
        self.linear_pid = PIDController(
            kp=config['linear_kp'],
            ki=config['linear_ki'],
            kd=config['linear_kd'],
            output_min=-config['max_speed_reduction'],
            output_max=config['max_speed_reduction'],
            integral_limit=config['linear_integral_limit'],
        )
        self.state_machine = MazeStateMachine(
            wall_detect_threshold=config['wall_detect_threshold'],
            front_block_threshold=config['front_block_threshold'],
            turn_exit_front_threshold=config['turn_exit_front_threshold'],
        )
        self.previous_scan: Optional[ScanSnapshot] = None

    def update_scan(self, front: float, left: float, right: float, stamp_sec: float) -> ScanSnapshot:
        left_rate = 0.0
        dt = 0.0
        if self.previous_scan is not None:
            dt = max(1e-3, stamp_sec - self.previous_scan.stamp_sec)
            left_rate = (left - self.previous_scan.left) / dt

        scan = ScanSnapshot(
            front=front,
            left=left,
            right=right,
            left_rate=left_rate,
            dt=dt,
            stamp_sec=stamp_sec,
        )
        self.previous_scan = scan
        return scan

    def compute_command(self, scan: ScanSnapshot) -> Twist:
        decision = self.state_machine.update(scan.front, scan.left, scan.right)
        twist = Twist()

        if decision.state == RobotState.SEARCHING:
            self.angular_pid.reset()
            self.linear_pid.reset()
            twist.linear.x = self.search_linear_speed
            twist.angular.z = self.search_turn_speed
            return twist

        if decision.state == RobotState.TURNING:
            self.angular_pid.reset()
            self.linear_pid.reset()
            twist.linear.x = self.turn_linear_speed
            if decision.turn_direction == 'left':
                twist.angular.z = abs(self.turn_angular_speed)
            elif decision.turn_direction == 'right':
                twist.angular.z = -abs(self.turn_angular_speed)
            else:
                twist.angular.z = abs(self.turn_angular_speed)
            return twist

        wall_error = self.desired_wall_distance - scan.left
        predictive_correction = -self.predictive_gain * scan.left_rate
        angular_command = self.angular_pid.compute(wall_error + predictive_correction, scan.dt)

        front_error = self.front_clearance_target - scan.front
        speed_penalty = max(0.0, self.linear_pid.compute(front_error, scan.dt))
        linear_command = self.base_linear_speed - speed_penalty - min(abs(angular_command) * 0.1, 0.15)

        twist.linear.x = max(self.min_linear_speed, min(linear_command, self.max_linear_speed))
        twist.angular.z = angular_command
        return twist

    @staticmethod
    def sanitize_range(value: float, range_min: float, range_max: float) -> float:
        if math.isinf(value) or math.isnan(value):
            return range_max
        return min(max(value, range_min), range_max)
