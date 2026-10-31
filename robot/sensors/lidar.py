"""
360-Degree 2D LiDAR Simulator (LDS-01 / LDS-02 Model)
Robotics Laboratory ESE

This module simulates the laser distance sensor mounted on the TurtleBot3.
It performs raycasting against lines, polygons, and circular obstacles,
generating realistic LaserScan readings with optional Gaussian noise.

Key Concepts:
- Raycasting: Projecting rays from the sensor center at incremental angles [0 to 2*pi).
- Ray-Line Intersection: Uses vector cross-products / determinants to find collision distances.
- Ray-Circle Intersection: Quadratic equation root-finding.
- Sensor Characteristics: Min range (0.12 m), Max range (3.5 m), 360 beams (1-deg resolution).
"""

import math
import random
from typing import List, Tuple, Optional, Dict, Any


class SimulatedLiDAR:
    """Simulates 360-degree planar LiDAR laser range scanner."""

    def __init__(
        self,
        min_range: float = 0.12,
        max_range: float = 3.50,
        samples: int = 360,
        noise_stddev: float = 0.01,
        scan_rate_hz: float = 10.0
    ):
        self.min_range = min_range
        self.max_range = max_range
        self.samples = samples
        self.angle_min = 0.0
        self.angle_max = 2.0 * math.pi
        self.angle_increment = (2.0 * math.pi) / samples
        self.noise_stddev = noise_stddev
        self.scan_rate_hz = scan_rate_hz

        # Precompute relative beam angles in sensor frame
        self.beam_angles = [i * self.angle_increment for i in range(self.samples)]

    def ray_segment_intersection(
        self,
        rx: float, ry: float, cos_a: float, sin_a: float,
        x1: float, y1: float, x2: float, y2: float
    ) -> Optional[float]:
        """
        Computes intersection distance t of ray (rx, ry) + t*(cos_a, sin_a)
        with line segment (x1, y1) -> (x2, y2).
        Returns distance t >= 0 if intersecting, else None.
        """
        dx = x2 - x1
        dy = y2 - y1

        denom = cos_a * dy - sin_a * dx
        if abs(denom) < 1e-9:
            return None  # Parallel or collinear

        t = ((x1 - rx) * dy - (y1 - ry) * dx) / denom
        u = ((x1 - rx) * sin_a - (y1 - ry) * cos_a) / denom

        if t > 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    def ray_circle_intersection(
        self,
        rx: float, ry: float, cos_a: float, sin_a: float,
        cx: float, cy: float, radius: float
    ) -> Optional[float]:
        """
        Computes intersection distance of ray with a cylinder/circle obstacle.
        Solves ||(rx + t*cos_a, ry + t*sin_a) - (cx, cy)||^2 = radius^2.
        """
        ox = rx - cx
        oy = ry - cy

        # Quadratic: a*t^2 + 2*b*t + c = 0 where a = cos_a^2 + sin_a^2 = 1
        b = ox * cos_a + oy * sin_a
        c = ox * ox + oy * oy - radius * radius

        discriminant = b * b - c
        if discriminant < 0.0:
            return None

        sqrt_disc = math.sqrt(discriminant)
        t1 = -b - sqrt_disc
        t2 = -b + sqrt_disc

        if t1 > 0.0:
            return t1
        elif t2 > 0.0:
            return t2
        return None

    def ray_box_intersection(
        self,
        rx: float, ry: float, cos_a: float, sin_a: float,
        bx: float, by: float, width: float, height: float
    ) -> Optional[float]:
        """
        Computes intersection distance with an axis-aligned box
        centered at (bx, by) with dimensions (width, height).
        """
        x_min = bx - width / 2.0
        x_max = bx + width / 2.0
        y_min = by - height / 2.0
        y_max = by + height / 2.0

        # Segment checks for 4 sides
        sides = [
            (x_min, y_min, x_max, y_min),  # South
            (x_max, y_min, x_max, y_max),  # East
            (x_max, y_max, x_min, y_max),  # North
            (x_min, y_max, x_min, y_min)   # West
        ]

        min_t = None
        for x1, y1, x2, y2 in sides:
            t = self.ray_segment_intersection(rx, ry, cos_a, sin_a, x1, y1, x2, y2)
            if t is not None:
                if min_t is None or t < min_t:
                    min_t = t
        return min_t

    def scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        walls: List[List[float]],
        obstacles: List[Dict[str, Any]]
    ) -> Tuple[List[float], List[Tuple[float, float]]]:
        """
        Performs 360-degree raycasting across all environment walls and obstacles.
        Returns:
            ranges: List of 360 float range measurements (meters).
            hit_points: List of global (x, y) hit points for visualization.
        """
        ranges = []
        hit_points = []

        for rel_angle in self.beam_angles:
            # Absolute ray direction in world frame
            abs_angle = robot_theta + rel_angle
            cos_a = math.cos(abs_angle)
            sin_a = math.sin(abs_angle)

            closest_dist = self.max_range

            # Check wall line segments
            for wall in walls:
                dist = self.ray_segment_intersection(
                    robot_x, robot_y, cos_a, sin_a,
                    wall[0], wall[1], wall[2], wall[3]
                )
                if dist is not None and dist < closest_dist:
                    closest_dist = dist

            # Check static & dynamic obstacles
            for obs in obstacles:
                obs_type = obs.get("type", "circle")
                if obs_type == "circle":
                    dist = self.ray_circle_intersection(
                        robot_x, robot_y, cos_a, sin_a,
                        obs["x"], obs["y"], obs["radius"]
                    )
                elif obs_type == "box":
                    dist = self.ray_box_intersection(
                        robot_x, robot_y, cos_a, sin_a,
                        obs["x"], obs["y"], obs["width"], obs["height"]
                    )
                else:
                    dist = None

                if dist is not None and dist < closest_dist:
                    closest_dist = dist

            # Apply sensor noise and thresholding
            if closest_dist >= self.max_range:
                measured_dist = float('inf')
                hit_x = robot_x + self.max_range * cos_a
                hit_y = robot_y + self.max_range * sin_a
            elif closest_dist < self.min_range:
                measured_dist = 0.0  # Inside blind zone
                hit_x = robot_x + closest_dist * cos_a
                hit_y = robot_y + closest_dist * sin_a
            else:
                noise = random.gauss(0.0, self.noise_stddev) if self.noise_stddev > 0 else 0.0
                measured_dist = max(self.min_range, min(self.max_range, closest_dist + noise))
                hit_x = robot_x + measured_dist * cos_a
                hit_y = robot_y + measured_dist * sin_a

            ranges.append(measured_dist)
            hit_points.append((hit_x, hit_y))

        return ranges, hit_points

    def create_ros_laserscan_msg(
        self,
        ranges: List[float],
        timestamp_sec: float = 0.0,
        frame_id: str = "base_scan"
    ):
        """
        Constructs a genuine ROS 2 sensor_msgs/LaserScan message.
        Falls back to a dictionary representation if rclpy is headlessly mocked.
        """
        try:
            from sensor_msgs.msg import LaserScan
            from builtin_interfaces.msg import Time

            msg = LaserScan()
            msg.header.frame_id = frame_id
            msg.header.stamp = Time(
                sec=int(timestamp_sec),
                nanosec=int((timestamp_sec - int(timestamp_sec)) * 1e9)
            )
            msg.angle_min = float(self.angle_min)
            msg.angle_max = float(self.angle_max)
            msg.angle_increment = float(self.angle_increment)
            msg.time_increment = float(1.0 / (self.scan_rate_hz * self.samples))
            msg.scan_time = float(1.0 / self.scan_rate_hz)
            msg.range_min = float(self.min_range)
            msg.range_max = float(self.max_range)
            msg.ranges = [float(r) for r in ranges]
            return msg
        except Exception:
            return {
                "header": {"frame_id": frame_id, "timestamp": timestamp_sec},
                "angle_min": self.angle_min,
                "angle_max": self.angle_max,
                "angle_increment": self.angle_increment,
                "range_min": self.min_range,
                "range_max": self.max_range,
                "ranges": ranges
            }
