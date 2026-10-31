"""
TurtleBot3 Waffle Pi Kinematic & Physical Model
Robotics Laboratory ESE

This module encapsulates the mechanical and kinematic specifications of the
larger ROBOTIS TurtleBot3 Waffle / Waffle Pi platform.
"""

import math
from typing import Tuple, Dict, Any


class TurtleBot3Waffle:
    """Representation of TurtleBot3 Waffle Pi differential drive robot."""

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x: float = float(x)
        self.y: float = float(y)
        self.theta: float = float(theta)
        self.v: float = 0.0
        self.omega: float = 0.0

        # Physical constants (Waffle specs)
        self.wheel_radius: float = 0.033       # 33 mm
        self.wheel_separation: float = 0.287   # 287 mm (wider track width)
        self.chassis_length: float = 0.281     # 281 mm
        self.chassis_width: float = 0.306      # 306 mm
        self.chassis_radius: float = 0.170     # Equivalent bounding radius
        self.safety_inflation_radius: float = 0.25
        self.weight: float = 1.8               # kg

        # Kinematic limits
        self.max_linear_velocity: float = 0.26
        self.min_linear_velocity: float = -0.26
        self.max_angular_velocity: float = 1.82
        self.linear_accel_limit: float = 0.5
        self.angular_accel_limit: float = 2.0

        self.omega_left: float = 0.0
        self.omega_right: float = 0.0
        self.distance_traveled: float = 0.0
        self.trajectory_history = [(self.x, self.y)]

    def inverse_kinematics(self, v: float, omega: float) -> Tuple[float, float]:
        r = self.wheel_radius
        L = self.wheel_separation
        omega_r = (2.0 * v + omega * L) / (2.0 * r)
        omega_l = (2.0 * v - omega * L) / (2.0 * r)
        return omega_l, omega_r

    def forward_kinematics(self, omega_l: float, omega_r: float) -> Tuple[float, float]:
        r = self.wheel_radius
        L = self.wheel_separation
        v = (r / 2.0) * (omega_r + omega_l)
        omega = (r / L) * (omega_r - omega_l)
        return v, omega

    def update_pose(self, cmd_v: float, cmd_omega: float, dt: float) -> Tuple[float, float, float]:
        if dt <= 0.0:
            return self.x, self.y, self.theta

        # Apply actuator limits
        max_delta_v = self.linear_accel_limit * dt
        self.v += max(-max_delta_v, min(max_delta_v, cmd_v - self.v))
        self.v = max(self.min_linear_velocity, min(self.max_linear_velocity, self.v))

        max_delta_omega = self.angular_accel_limit * dt
        self.omega += max(-max_delta_omega, min(max_delta_omega, cmd_omega - self.omega))
        self.omega = max(-self.max_angular_velocity, min(self.max_angular_velocity, self.omega))

        self.omega_left, self.omega_right = self.inverse_kinematics(self.v, self.omega)

        theta_mid = self.theta + 0.5 * self.omega * dt
        dx = self.v * math.cos(theta_mid) * dt
        dy = self.v * math.sin(theta_mid) * dt
        dtheta = self.omega * dt

        self.x += dx
        self.y += dy
        self.theta = (self.theta + dtheta + math.pi) % (2.0 * math.pi) - math.pi

        self.distance_traveled += math.hypot(dx, dy)
        if len(self.trajectory_history) == 0 or math.hypot(self.x - self.trajectory_history[-1][0], self.y - self.trajectory_history[-1][1]) > 0.03:
            self.trajectory_history.append((self.x, self.y))

        return self.x, self.y, self.theta
