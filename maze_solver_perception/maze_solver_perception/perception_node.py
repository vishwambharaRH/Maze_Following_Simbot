import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from maze_solver_interfaces.msg import DistanceRegions


@dataclass
class RegionSnapshot:
    front: float
    left: float
    right: float
    stamp_sec: float


class MazePerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('maze_perception_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('filtered_scan_topic', '/scan_filtered')
        self.declare_parameter('regions_topic', '/distance_regions')
        self.declare_parameter('front_angle_deg', 20.0)
        self.declare_parameter('side_center_deg', 90.0)
        self.declare_parameter('side_width_deg', 35.0)
        self.declare_parameter('window_filter_size', 5)
        self.declare_parameter('max_valid_range', 8.0)

        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.side_center_deg = float(self.get_parameter('side_center_deg').value)
        self.side_width_deg = float(self.get_parameter('side_width_deg').value)
        self.window_filter_size = max(1, int(self.get_parameter('window_filter_size').value))
        self.max_valid_range = float(self.get_parameter('max_valid_range').value)

        self.previous_snapshot: Optional[RegionSnapshot] = None

        self.filtered_scan_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter('filtered_scan_topic').value),
            10,
        )
        self.region_pub = self.create_publisher(
            DistanceRegions,
            str(self.get_parameter('regions_topic').value),
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.scan_callback,
            10,
        )
        self.get_logger().info('Perception node ready. Waiting for /scan.')

    def scan_callback(self, msg: LaserScan) -> None:
        filtered_ranges = self._median_filter(msg.ranges, msg.range_min, min(msg.range_max, self.max_valid_range))

        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = min(msg.range_max, self.max_valid_range)
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = list(msg.intensities)
        self.filtered_scan_pub.publish(filtered_msg)

        front = self._region_min(filtered_msg, [(-self.front_angle_deg, self.front_angle_deg)])
        left = self._region_min(
            filtered_msg,
            [(self.side_center_deg - self.side_width_deg, self.side_center_deg + self.side_width_deg)],
        )
        right = self._region_min(
            filtered_msg,
            [(-self.side_center_deg - self.side_width_deg, -self.side_center_deg + self.side_width_deg)],
        )

        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        front_rate = 0.0
        left_rate = 0.0
        right_rate = 0.0
        if self.previous_snapshot is not None:
            dt = max(1e-3, stamp_sec - self.previous_snapshot.stamp_sec)
            front_rate = (front - self.previous_snapshot.front) / dt
            left_rate = (left - self.previous_snapshot.left) / dt
            right_rate = (right - self.previous_snapshot.right) / dt
        self.previous_snapshot = RegionSnapshot(front=front, left=left, right=right, stamp_sec=stamp_sec)

        region_msg = DistanceRegions()
        region_msg.header = msg.header
        region_msg.front = float(front)
        region_msg.left = float(left)
        region_msg.right = float(right)
        region_msg.front_rate = float(front_rate)
        region_msg.left_rate = float(left_rate)
        region_msg.right_rate = float(right_rate)
        self.region_pub.publish(region_msg)

    def _median_filter(self, ranges: Sequence[float], range_min: float, range_max: float) -> List[float]:
        cleaned = [self._sanitize_range(value, range_min, range_max) for value in ranges]
        if self.window_filter_size <= 1:
            return cleaned

        radius = self.window_filter_size // 2
        filtered: List[float] = []
        for index in range(len(cleaned)):
            start = max(0, index - radius)
            end = min(len(cleaned), index + radius + 1)
            window = sorted(cleaned[start:end])
            filtered.append(window[len(window) // 2])
        return filtered

    def _region_min(self, msg: LaserScan, windows_deg: List[Tuple[float, float]]) -> float:
        samples: List[float] = []
        for angle_rad in self._iter_window_angles(msg, windows_deg):
            index = int(round((angle_rad - msg.angle_min) / msg.angle_increment))
            if 0 <= index < len(msg.ranges):
                samples.append(self._sanitize_range(msg.ranges[index], msg.range_min, msg.range_max))
        return min(samples) if samples else msg.range_max

    def _iter_window_angles(
        self,
        msg: LaserScan,
        windows_deg: Iterable[Tuple[float, float]],
    ) -> Iterable[float]:
        for start_deg, end_deg in windows_deg:
            low = min(math.radians(start_deg), math.radians(end_deg))
            high = max(math.radians(start_deg), math.radians(end_deg))
            current = low
            while current <= high:
                yield current
                current += max(msg.angle_increment, math.radians(1.0))

    @staticmethod
    def _sanitize_range(value: float, range_min: float, range_max: float) -> float:
        if math.isnan(value) or math.isinf(value):
            return range_max
        return min(max(value, range_min), range_max)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MazePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
