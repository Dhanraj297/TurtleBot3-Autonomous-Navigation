"""
Robot Odometry, Dead Reckoning & Coordinate Frame Localization
Robotics Laboratory ESE

Theoretical Foundation:
-----------------------
Dead reckoning estimates robot position by integrating discrete motion increments
from wheel encoders over time.

Coordinate Frames:
- map: Global fixed world reference frame.
- odom: Continuous odometric frame (origin at robot startup, drifts over long trajectories).
- base_footprint: 2D planar projection of the robot chassis on the floor.
- base_link: Rigid origin of the robot body center.
- base_scan: Center of the 2D laser range finder.

Odometry Kinematic Integration:
    delta_s = 0.5 * (delta_s_R + delta_s_L)
    delta_theta = (delta_s_R - delta_s_L) / track_width
    x_{t+1} = x_t + delta_s * cos(theta_t + 0.5 * delta_theta)
    y_{t+1} = y_t + delta_s * sin(theta_t + 0.5 * delta_theta)
    theta_{t+1} = theta_t + delta_theta
"""

import math
from typing import Tuple, Dict, Any, Optional


class OdometryLocalizer:
    """Maintains robot pose estimate, covariance, and ROS 2 Odometry messages."""

    def __init__(
        self,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_theta: float = 0.0,
        wheel_radius: float = 0.033,
        wheel_separation: float = 0.160,
        slip_noise_stddev: float = 0.005
    ):
        self.x = float(initial_x)
        self.y = float(initial_y)
        self.theta = float(initial_theta)
        self.v = 0.0
        self.omega = 0.0

        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.slip_noise = slip_noise_stddev

        # Covariance diagonal [x, y, z, roll, pitch, yaw]
        self.pose_covariance = [0.001, 0.001, 1e6, 1e6, 1e6, 0.002]
        self.twist_covariance = [0.002, 0.002, 1e6, 1e6, 1e6, 0.005]

    def update_from_wheel_speeds(
        self,
        omega_left: float,
        omega_right: float,
        dt: float
    ) -> Tuple[float, float, float]:
        """Integrates encoder ticks / wheel angular speeds to update estimated pose."""
        if dt <= 0.0:
            return self.x, self.y, self.theta

        # Wheel linear displacements
        ds_l = omega_left * self.wheel_radius * dt
        ds_r = omega_right * self.wheel_radius * dt

        ds = (ds_r + ds_l) / 2.0
        dtheta = (ds_r - ds_l) / self.wheel_separation

        mid_theta = self.theta + 0.5 * dtheta
        self.x += ds * math.cos(mid_theta)
        self.y += ds * math.sin(mid_theta)
        self.theta = (self.theta + dtheta + math.pi) % (2.0 * math.pi) - math.pi

        self.v = ds / dt
        self.omega = dtheta / dt

        return self.x, self.y, self.theta

    def set_pose(self, x: float, y: float, theta: float):
        """Re-initializes localized pose (e.g. from 2D Pose Estimate / AMCL)."""
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.v = 0.0
        self.omega = 0.0

    def get_pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.theta

    def create_ros_odometry_msg(self, timestamp_sec: float = 0.0, odom_frame: str = "odom", base_frame: str = "base_footprint"):
        """Constructs a genuine ROS 2 nav_msgs/Odometry message."""
        try:
            from nav_msgs.msg import Odometry
            from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
            from builtin_interfaces.msg import Time

            msg = Odometry()
            msg.header.frame_id = odom_frame
            msg.child_frame_id = base_frame
            msg.header.stamp = Time(
                sec=int(timestamp_sec),
                nanosec=int((timestamp_sec - int(timestamp_sec)) * 1e9)
            )

            # 2D orientation to Quaternion (Euler yaw -> Quaternion [0, 0, sin(yaw/2), cos(yaw/2)])
            half_yaw = self.theta / 2.0
            msg.pose.pose.position = Point(x=float(self.x), y=float(self.y), z=0.0)
            msg.pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=float(math.sin(half_yaw)),
                w=float(math.cos(half_yaw))
            )

            msg.twist.twist.linear = Vector3(x=float(self.v), y=0.0, z=0.0)
            msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=float(self.omega))
            return msg
        except Exception:
            return {
                "header": {"frame_id": odom_frame, "timestamp": timestamp_sec},
                "child_frame_id": base_frame,
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "v": self.v,
                "omega": self.omega
            }
