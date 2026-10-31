"""
TurtleBot3 Burger Kinematic & Physical Model
Robotics Laboratory ESE

This module encapsulates the mechanical, kinematic, and dynamic constraints
of the official ROBOTIS TurtleBot3 Burger mobile robot.

Differential Drive Kinematic Equations:
---------------------------------------
A differential-drive robot has two independently driven wheels on a common axis
separated by track width L, with radius r.

Forward Kinematics (Wheel Speeds -> Robot Twist):
    v = (r / 2) * (omega_R + omega_L)          [Linear velocity, m/s]
    omega = (r / L) * (omega_R - omega_L)      [Angular velocity, rad/s]

Inverse Kinematics (Robot Twist -> Wheel Speeds):
    omega_R = (2*v + omega*L) / (2*r)          [Right wheel angular velocity, rad/s]
    omega_L = (2*v - omega*L) / (2*r)          [Left wheel angular velocity, rad/s]

Pose Integration (Euler / Runge-Kutta in Discrete Time dt):
    theta_{t+1} = theta_t + omega * dt
    x_{t+1}     = x_t + v * cos(theta_mid) * dt
    y_{t+1}     = y_t + v * sin(theta_mid) * dt
    where theta_mid = theta_t + 0.5 * omega * dt (Second-Order Runge-Kutta)
"""

import math
from typing import Tuple, Dict, Any


class TurtleBot3Burger:
    """Mathematical and physical representation of TurtleBot3 Burger."""

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        # State: Pose in global frame (x [m], y [m], theta [rad])
        self.x: float = float(x)
        self.y: float = float(y)
        self.theta: float = float(theta)

        # Current velocities
        self.v: float = 0.0       # Linear velocity (m/s)
        self.omega: float = 0.0   # Angular velocity (rad/s)

        # Physical constants (ROBOTIS official specs)
        self.wheel_radius: float = 0.033       # r = 33 mm
        self.wheel_separation: float = 0.160   # L = 160 mm
        self.chassis_radius: float = 0.105     # 105 mm footprint
        self.safety_inflation_radius: float = 0.20  # 200 mm safety perimeter
        self.weight: float = 1.0               # kg

        # Kinematic limits
        self.max_linear_velocity: float = 0.22      # m/s
        self.min_linear_velocity: float = -0.22     # m/s
        self.max_angular_velocity: float = 2.84     # rad/s (~162.7 deg/s)
        self.linear_accel_limit: float = 0.5        # m/s^2
        self.angular_accel_limit: float = 2.0       # rad/s^2

        # Wheel speeds
        self.omega_left: float = 0.0
        self.omega_right: float = 0.0

        # Odometry tracking accumulator
        self.distance_traveled: float = 0.0
        self.trajectory_history = [(self.x, self.y)]

    def clamp_velocities(self, target_v: float, target_omega: float, dt: float) -> Tuple[float, float]:
        """
        Enforces maximum velocity and acceleration limits.
        Prevents unrealistic instantaneous velocity jumps.
        """
        # Acceleration limiting
        max_delta_v = self.linear_accel_limit * dt
        delta_v = target_v - self.v
        delta_v = max(-max_delta_v, min(max_delta_v, delta_v))
        new_v = self.v + delta_v

        max_delta_omega = self.angular_accel_limit * dt
        delta_omega = target_omega - self.omega
        delta_omega = max(-max_delta_omega, min(max_delta_omega, delta_omega))
        new_omega = self.omega + delta_omega

        # Static friction / deadband threshold for coming to complete rest
        if target_v == 0.0 and abs(new_v) < 0.05:
            new_v = 0.0
        if target_omega == 0.0 and abs(new_omega) < 0.05:
            new_omega = 0.0

        # Velocity saturation
        new_v = max(self.min_linear_velocity, min(self.max_linear_velocity, new_v))
        new_omega = max(-self.max_angular_velocity, min(self.max_angular_velocity, new_omega))

        return new_v, new_omega

    def inverse_kinematics(self, v: float, omega: float) -> Tuple[float, float]:
        """
        Computes required left and right wheel angular velocities (rad/s)
        to produce desired robot body twist (v, omega).
        """
        r = self.wheel_radius
        L = self.wheel_separation
        omega_r = (2.0 * v + omega * L) / (2.0 * r)
        omega_l = (2.0 * v - omega * L) / (2.0 * r)
        return omega_l, omega_r

    def forward_kinematics(self, omega_l: float, omega_r: float) -> Tuple[float, float]:
        """
        Computes body twist (v, omega) produced by given wheel speeds.
        """
        r = self.wheel_radius
        L = self.wheel_separation
        v = (r / 2.0) * (omega_r + omega_l)
        omega = (r / L) * (omega_r - omega_l)
        return v, omega

    def update_pose(self, cmd_v: float, cmd_omega: float, dt: float) -> Tuple[float, float, float]:
        """
        Simulates one discrete time-step of differential drive motion.
        Uses 2nd-order Runge-Kutta integration for accurate curvilinear motion.
        """
        if dt <= 0.0:
            return self.x, self.y, self.theta

        # Apply actuator limits
        self.v, self.omega = self.clamp_velocities(cmd_v, cmd_omega, dt)
        self.omega_left, self.omega_right = self.inverse_kinematics(self.v, self.omega)

        # Midpoint orientation for Runge-Kutta (RK2) integration
        theta_mid = self.theta + 0.5 * self.omega * dt

        # Update pose in world coordinates
        dx = self.v * math.cos(theta_mid) * dt
        dy = self.v * math.sin(theta_mid) * dt
        dtheta = self.omega * dt

        self.x += dx
        self.y += dy
        self.theta = (self.theta + dtheta + math.pi) % (2.0 * math.pi) - math.pi

        # Update distance accumulator & breadcrumb path
        dist_step = math.hypot(dx, dy)
        self.distance_traveled += dist_step
        if len(self.trajectory_history) == 0 or math.hypot(self.x - self.trajectory_history[-1][0], self.y - self.trajectory_history[-1][1]) > 0.03:
            self.trajectory_history.append((self.x, self.y))

        return self.x, self.y, self.theta

    def stop(self):
        """Forces velocities to zero (emergency/full stop)."""
        self.v = 0.0
        self.omega = 0.0
        self.omega_left = 0.0
        self.omega_right = 0.0

    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Resets the robot pose and clears telemetry history."""
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.v = 0.0
        self.omega = 0.0
        self.distance_traveled = 0.0
        self.trajectory_history = [(self.x, self.y)]

    def get_footprint_polygon(self) -> list:
        """
        Returns vertices of TurtleBot3 Burger rectangular/circular body in world frame
        for collision checking and rendering.
        """
        # Circular footprint approximation for Burger
        num_points = 16
        pts = []
        for i in range(num_points):
            ang = self.theta + (2.0 * math.pi * i / num_points)
            px = self.x + self.chassis_radius * math.cos(ang)
            py = self.y + self.chassis_radius * math.sin(ang)
            pts.append((px, py))
        return pts

    def get_state_dict(self) -> Dict[str, Any]:
        """Serializes current robot state for telemetry/ROS messages."""
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "theta_deg": math.degrees(self.theta),
            "linear_velocity": self.v,
            "angular_velocity": self.omega,
            "wheel_speed_left": self.omega_left,
            "wheel_speed_right": self.omega_right,
            "distance_traveled": self.distance_traveled
        }
