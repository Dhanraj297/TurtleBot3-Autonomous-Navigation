"""
Reactive Local Obstacle Avoidance (Artificial Potential Field & Sector Gating)
Robotics Laboratory ESE

Theoretical Foundation:
-----------------------
While A* computes a global optimal path based on a static prior map,
unmodeled, moving, or dynamic obstacles require real-time reactive avoidance.

This module implements an Artificial Potential Field (APF) and Sector Safety Gating
operating directly on the 360-degree LiDAR range stream:

1. Attractive Potential (Goal):
   F_att = -grad(U_att) = K_att * (p_goal - p_robot)
   Pulls robot towards the current waypoint.

2. Repulsive Potential (LiDAR Range Rays):
   For each laser beam i at relative angle theta_i with distance d_i < d_warning:
   F_rep,i = K_rep * (1/d_i - 1/d_warning) * (1/d_i^2) * (-cos(theta_i), -sin(theta_i))
   Obstacles exert a repelling force inversely proportional to squared distance.

3. Emergency Collision Mitigation:
   If min(front_sector_distances) < d_emergency:
       Instant linear deceleration / stop, and turn towards maximum clearance direction.
"""

import math
from typing import List, Tuple, Dict, Any


class ObstacleAvoidance:
    """Reactive obstacle avoidance controller using LiDAR scan analysis and potential fields."""

    def __init__(
        self,
        emergency_stop_dist: float = 0.20,
        warning_dist: float = 0.55,
        front_sector_deg: float = 60.0,
        repulsion_gain: float = 0.8
    ):
        self.emergency_dist = emergency_stop_dist
        self.warning_dist = warning_dist
        self.front_sector_rad = math.radians(front_sector_deg / 2.0)
        self.repulsion_gain = repulsion_gain

    def analyze_scan(self, ranges: List[float], angle_increment: float = math.radians(1.0)) -> Dict[str, Any]:
        """
        Segments 360-degree laser scan into spatial sectors:
        - Front (-30 deg to +30 deg)
        - Front-Left (30 deg to 75 deg)
        - Front-Right (-75 deg to -30 deg)
        - Left (75 deg to 135 deg)
        - Right (-135 deg to -75 deg)
        """
        num_beams = len(ranges)
        min_front = float('inf')
        min_left = float('inf')
        min_right = float('inf')
        overall_min = float('inf')
        overall_min_angle = 0.0

        for i, dist in enumerate(ranges):
            if math.isinf(dist) or math.isnan(dist) or dist <= 0.0:
                continue

            # Convert beam index to relative angle in [-pi, +pi]
            rel_ang = i * angle_increment
            if rel_ang > math.pi:
                rel_ang -= 2.0 * math.pi

            if dist < overall_min:
                overall_min = dist
                overall_min_angle = rel_ang

            # Front cone
            if abs(rel_ang) <= self.front_sector_rad:
                if dist < min_front:
                    min_front = dist
            # Left side
            elif 0.0 < rel_ang <= math.radians(90.0):
                if dist < min_left:
                    min_left = dist
            # Right side
            elif -math.radians(90.0) <= rel_ang < 0.0:
                if dist < min_right:
                    min_right = dist

        return {
            "min_front": min_front,
            "min_left": min_left,
            "min_right": min_right,
            "overall_min": overall_min,
            "overall_min_angle": overall_min_angle,
            "in_emergency": min_front <= self.emergency_dist,
            "in_warning": min_front <= self.warning_dist
        }

    def compute_avoidance_twist(
        self,
        nominal_v: float,
        nominal_omega: float,
        ranges: List[float],
        angle_increment: float = math.radians(1.0)
    ) -> Tuple[float, float, str]:
        """
        Blends nominal path-following commands with reactive obstacle avoidance.
        Returns:
            v_cmd: Linear velocity (m/s)
            omega_cmd: Angular velocity (rad/s)
            status: 'CLEAR', 'REPULSION_ACTIVE', 'EMERGENCY_STOP'
        """
        scan_info = self.analyze_scan(ranges, angle_increment)

        # 1. Emergency stop condition
        if scan_info["in_emergency"]:
            # Hard stop linear velocity; rotate away from the closer side
            if scan_info["min_left"] < scan_info["min_right"]:
                # Turn right
                omega = -1.5
            else:
                # Turn left
                omega = 1.5
            return 0.0, omega, "EMERGENCY_STOP"

        # 2. Warning zone: apply artificial potential field repulsive steering
        if scan_info["in_warning"]:
            # Calculate repulsive lateral force
            repulsive_omega = 0.0
            num_beams = len(ranges)

            for i, dist in enumerate(ranges):
                if dist > self.warning_dist or math.isinf(dist) or dist <= 0.0:
                    continue

                rel_ang = i * angle_increment
                if rel_ang > math.pi:
                    rel_ang -= 2.0 * math.pi

                # Focus on forward hemisphere
                if abs(rel_ang) < math.radians(85.0):
                    # Repulsive torque pushes in direction opposite to obstacle
                    # Obstacle on left (rel_ang > 0) -> push right (negative omega)
                    # Obstacle on right (rel_ang < 0) -> push left (positive omega)
                    proximity_weight = (self.warning_dist - dist) / self.warning_dist
                    direction = -1.0 if rel_ang > 0 else 1.0
                    repulsive_omega += direction * proximity_weight * self.repulsion_gain

            # Speed modulation: slow down proportionally to obstacle proximity
            slowdown_ratio = max(0.15, (scan_info["min_front"] - self.emergency_dist) / (self.warning_dist - self.emergency_dist))
            safe_v = nominal_v * slowdown_ratio
            blended_omega = nominal_omega + repulsive_omega
            blended_omega = max(-2.5, min(2.5, blended_omega))

            return safe_v, blended_omega, "REPULSION_ACTIVE"

        # 3. Path is clear
        return nominal_v, nominal_omega, "CLEAR"
