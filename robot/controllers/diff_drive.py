"""
Differential Drive Motion Controller & Trajectory Follower
Robotics Laboratory ESE

This module implements closed-loop steering and velocity control for
TurtleBot3 differential-drive robots.

Control Law:
------------
1. Heading Error (alpha):
   alpha = atan2(y_target - y_robot, x_target - x_robot) - theta_robot
   Normalized to [-pi, +pi].

2. Angular Velocity Control (PID):
   omega = Kp_angular * alpha + Ki_angular * integral(alpha) + Kd_angular * d(alpha)/dt

3. Linear Velocity Control (Proportional with Heading Damping):
   v = Kp_linear * distance_to_goal * cos(alpha)
   When |alpha| > 60 deg, v is reduced to allow turning-in-place without large lateral drift.

4. Terminal Heading Alignment:
   When within goal_distance_tolerance, rotate to desired target orientation theta_goal.
"""

import math
from typing import Tuple, Optional


class DifferentialDriveController:
    """PID-based trajectory tracking and waypoint controller for differential drive."""

    def __init__(
        self,
        max_linear_velocity: float = 0.22,
        max_angular_velocity: float = 2.84,
        kp_linear: float = 0.8,
        kp_angular: float = 2.5,
        ki_angular: float = 0.02,
        kd_angular: float = 0.15,
        goal_dist_tol: float = 0.08,
        goal_angle_tol: float = 0.15
    ):
        self.max_v = max_linear_velocity
        self.max_omega = max_angular_velocity
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular
        self.ki_angular = ki_angular
        self.kd_angular = kd_angular
        self.goal_dist_tol = goal_dist_tol
        self.goal_angle_tol = goal_angle_tol

        # PID state accumulators
        self.prev_heading_error = 0.0
        self.integral_heading_error = 0.0

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wraps angle into interval [-pi, +pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def reset_pid(self):
        """Resets PID integral and derivative terms."""
        self.prev_heading_error = 0.0
        self.integral_heading_error = 0.0

    def compute_waypoint_velocity(
        self,
        current_x: float,
        current_y: float,
        current_theta: float,
        target_x: float,
        target_y: float,
        target_theta: Optional[float] = None,
        dt: float = 0.05
    ) -> Tuple[float, float, bool]:
        """
        Computes (v, omega) command to drive from current pose towards target waypoint.
        Returns:
            v: Linear velocity (m/s)
            omega: Angular velocity (rad/s)
            reached: True if target pose reached within tolerance
        """
        dx = target_x - current_x
        dy = target_y - current_y
        dist_to_goal = math.hypot(dx, dy)

        # Check if translation goal reached
        if dist_to_goal <= self.goal_dist_tol:
            # If target orientation is specified, align in-place
            if target_theta is not None:
                angle_error = self.normalize_angle(target_theta - current_theta)
                if abs(angle_error) <= self.goal_angle_tol:
                    self.reset_pid()
                    return 0.0, 0.0, True  # Fully arrived!
                else:
                    # In-place pure rotation
                    omega = self.kp_angular * angle_error
                    omega = max(-self.max_omega, min(self.max_omega, omega))
                    return 0.0, omega, False
            else:
                self.reset_pid()
                return 0.0, 0.0, True

        # Compute heading to target waypoint
        desired_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_heading - current_theta)

        # PID Angular calculation
        if dt > 0.0:
            self.integral_heading_error += heading_error * dt
            # Anti-windup clamping
            self.integral_heading_error = max(-1.0, min(1.0, self.integral_heading_error))
            derivative = (heading_error - self.prev_heading_error) / dt
        else:
            derivative = 0.0

        self.prev_heading_error = heading_error

        omega = (
            self.kp_angular * heading_error +
            self.ki_angular * self.integral_heading_error +
            self.kd_angular * derivative
        )
        omega = max(-self.max_omega, min(self.max_omega, omega))

        # Linear speed calculation with heading error gating
        # If facing opposite or large angle (> 45 deg), slow down or turn in place
        cos_scale = max(0.0, math.cos(heading_error))
        v = self.kp_linear * dist_to_goal * cos_scale
        v = max(0.0, min(self.max_v, v))

        # Minimum creep speed to avoid motor deadband when close
        if dist_to_goal > self.goal_dist_tol and v < 0.04 and abs(heading_error) < math.radians(45):
            v = 0.04

        return v, omega, False
