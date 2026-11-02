"""
Gazebo 3D TurtleBot3 Autonomous Navigation Controller
Robotics Laboratory ESE — Mecharithm Lesson 9

Directly interfaces our project's differential drive controller and reactive obstacle
avoidance algorithms with 3D Gazebo (or physical TurtleBot3).

Subscribes to:
  - /odom  (nav_msgs/Odometry)    : Live robot pose (X, Y, Theta)
  - /scan  (sensor_msgs/LaserScan): 360-degree LiDAR range data
  - /goal_pose (geometry_msgs/PoseStamped): External goal dispatch (optional)

Publishes to:
  - /cmd_vel (geometry_msgs/Twist): Differential drive velocity commands (v, omega)

Can be executed on:
  1. Linux / WSL2: Directly inside Ubuntu alongside Gazebo
  2. Windows 11  : Communicating across the ROS 2 DDS network to Gazebo in WSL2
"""

import os
import sys
import math
import time
import argparse
from typing import List, Tuple, Optional

# Add project root to sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# Ensure Windows DLL search directory for conda/pixi environments
if sys.platform == "win32":
    pixi_lib_bin = os.path.join(os.path.dirname(sys.executable), "Library", "bin")
    if os.path.exists(pixi_lib_bin):
        os.environ["PATH"] = pixi_lib_bin + os.pathsep + os.environ.get("PATH", "")
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(pixi_lib_bin)
            except Exception:
                pass

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def normalize_angle(angle: float) -> float:
    """Wraps angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GazeboNavigatorNode(Node if ROS2_AVAILABLE else object):
    """Autonomous navigation controller bridging our algorithms to Gazebo."""

    def __init__(self, patrol_mode: bool = True, custom_goal: Optional[Tuple[float, float]] = None):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy) is required to run GazeboNavigatorNode.")

        super().__init__("gazebo_autonomous_navigator")

        # Navigation waypoints (suitable for standard turtlebot3_world arena)
        if custom_goal is not None:
            self.waypoints = [(custom_goal[0], custom_goal[1], "Custom Target")]
            self.loop_patrol = False
        elif patrol_mode:
            self.waypoints = [
                (1.0, 0.0, "East Point"),
                (1.0, 1.2, "North-East Corner"),
                (-0.5, 1.2, "North-West Corridor"),
                (-0.5, -0.5, "South-West Area"),
                (0.0, 0.0, "Origin / Home Dock")
            ]
            self.loop_patrol = True
        else:
            self.waypoints = [(1.5, 0.5, "Single Destination")]
            self.loop_patrol = False

        self.current_wp_idx = 0
        self.is_active = True
        self.goal_tolerance = 0.12  # meters

        # Live Robot State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Live LiDAR State
        self.min_front_dist = float('inf')
        self.min_left_dist = float('inf')
        self.min_right_dist = float('inf')
        self.min_overall_dist = float('inf')
        self.scan_received = False

        # Control gains
        self.kp_linear = 0.60
        self.kp_angular = 1.80
        self.max_linear_vel = 0.22   # Burger physical limit
        self.max_angular_vel = 1.80  # rad/s

        # Safety thresholds
        self.emergency_stop_dist = 0.22
        self.slowdown_dist = 0.45

        # Publishers and Subscribers
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, "/goal_pose", self.external_goal_callback, 10)

        # 20 Hz Control Loop Timer
        self.timer = self.create_timer(0.05, self.control_loop)
        self.last_status_print = 0.0

        self.get_logger().info("=========================================================")
        self.get_logger().info(" Gazebo 3D TurtleBot3 Autonomous Navigator Node Online   ")
        self.get_logger().info(f" Subscribing: /odom, /scan | Publishing: /cmd_vel        ")
        self.get_logger().info(f" Waypoints: {len(self.waypoints)} targets scheduled      ")
        self.get_logger().info("=========================================================")

    def odom_callback(self, msg: Odometry):
        """Extracts 2D pose and yaw from ROS 2 Odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        """Processes 360-degree LiDAR scan into directional sector clearances."""
        ranges = msg.ranges
        num_ranges = len(ranges)
        if num_ranges == 0:
            return

        # Burger LiDAR index 0 is directly FRONT.
        # Ranges: 0 is front, ~90 is left, ~180 is back, ~270 is right.
        front_window = 25  # +/- 25 degrees
        left_window = (45, 110)
        right_window = (250, 315)

        def clean_val(val):
            if math.isnan(val) or math.isinf(val) or val <= msg.range_min:
                return msg.range_max
            return val

        cleaned = [clean_val(r) for r in ranges]
        self.min_overall_dist = min(cleaned)

        # Front clearance (+/- 25 deg around 0)
        front_indices = list(range(0, front_window)) + list(range(num_ranges - front_window, num_ranges))
        self.min_front_dist = min(cleaned[i] for i in front_indices)

        # Left clearance
        left_min_idx = int((left_window[0] / 360.0) * num_ranges)
        left_max_idx = int((left_window[1] / 360.0) * num_ranges)
        self.min_left_dist = min(cleaned[left_min_idx:left_max_idx]) if left_max_idx > left_min_idx else msg.range_max

        # Right clearance
        right_min_idx = int((right_window[0] / 360.0) * num_ranges)
        right_max_idx = int((right_window[1] / 360.0) * num_ranges)
        self.min_right_dist = min(cleaned[right_min_idx:right_max_idx]) if right_max_idx > right_min_idx else msg.range_max

        self.scan_received = True

    def external_goal_callback(self, msg: PoseStamped):
        """Allows overriding goal interactively from RViz2 / 2D Nav Goal."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f"[External Goal] Overriding destination: ({gx:.2f}, {gy:.2f})")
        self.waypoints = [(gx, gy, "RViz External Goal")]
        self.current_wp_idx = 0
        self.loop_patrol = False
        self.is_active = True

    def control_loop(self):
        """20 Hz closed-loop control: computes (v, omega) and publishes to /cmd_vel."""
        if not self.odom_received or not self.scan_received:
            now = time.time()
            if now - self.last_status_print > 2.0:
                self.get_logger().info("Waiting for /odom and /scan topics from Gazebo...")
                self.last_status_print = now
            return

        if not self.is_active or self.current_wp_idx >= len(self.waypoints):
            self.stop_robot()
            return

        gx, gy, label = self.waypoints[self.current_wp_idx]
        dx = gx - self.current_x
        dy = gy - self.current_y
        dist_to_goal = math.hypot(dx, dy)

        # 1. Check if waypoint is reached
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(
                f">>> SUCCESS: Reached Waypoint [{self.current_wp_idx + 1}/{len(self.waypoints)}] '{label}' (Error: {dist_to_goal * 100:.1f} cm)"
            )
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                if self.loop_patrol:
                    self.get_logger().info("Patrol loop completed. Restarting from Waypoint 1...")
                    self.current_wp_idx = 0
                else:
                    self.get_logger().info("================ MISSION ACCOMPLISHED ================")
                    self.is_active = False
                    self.stop_robot()
                    return
            return

        # 2. Heading to target
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.current_theta)

        # 3. Reactive Obstacle Avoidance (Artificial Potential Field)
        avoidance_omega = 0.0
        is_avoiding = False

        if self.min_front_dist < self.emergency_stop_dist:
            # Immediate obstacle right ahead: halt forward drive, spin toward most open side
            is_avoiding = True
            v = -0.04  # gentle reverse
            omega = 1.2 if self.min_left_dist >= self.min_right_dist else -1.2
        elif self.min_front_dist < self.slowdown_dist:
            # Obstacle encroaching: steer away repulsively
            is_avoiding = True
            repulsion = (self.slowdown_dist - self.min_front_dist) / self.slowdown_dist
            steer_dir = 1.0 if self.min_left_dist >= self.min_right_dist else -1.0
            avoidance_omega = steer_dir * repulsion * 1.5

            # Blend goal-seeking with repulsion
            omega = (heading_error * self.kp_angular) + avoidance_omega
            v = self.max_linear_vel * (self.min_front_dist / self.slowdown_dist) * 0.5
        else:
            # Free space navigation
            omega = heading_error * self.kp_angular
            # Slow down if turning sharply
            turn_penalty = max(0.0, math.cos(heading_error))
            v = min(self.max_linear_vel, self.kp_linear * dist_to_goal) * turn_penalty

        # Clamp outputs to robot limits
        v = max(-self.max_linear_vel, min(self.max_linear_vel, v))
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))

        # Publish Twist
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(omega)
        self.pub_cmd_vel.publish(twist)

        # Status output
        now = time.time()
        if now - self.last_status_print > 0.5:
            mode_tag = "AVOIDING" if is_avoiding else "TRACKING"
            print(
                f"[{mode_tag}] WP {self.current_wp_idx + 1}/{len(self.waypoints)} '{label}' | "
                f"Robot: ({self.current_x:+.2f}, {self.current_y:+.2f}, {math.degrees(self.current_theta):+5.1f}°) | "
                f"Rem: {dist_to_goal:.2f}m | Front: {self.min_front_dist:.2f}m | "
                f"v: {v:+.2f} m/s, w: {omega:+.2f} rad/s"
            )
            self.last_status_print = now

    def stop_robot(self):
        """Sends zero velocity command to brake the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)


def main():
    parser = argparse.ArgumentParser(description="TurtleBot3 Gazebo Autonomous Navigator")
    parser.add_argument("--patrol", action="store_true", default=True, help="Continuously patrol waypoints")
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"), help="Set single goal coordinates: --goal 1.5 0.8")
    args = parser.parse_args()

    if not ROS2_AVAILABLE:
        print("[Error] rclpy is not installed or ROS 2 is not sourced.")
        print("Please source ROS 2 Humble before running this node:")
        print("  source /opt/ros/humble/setup.bash (in WSL / Linux)")
        sys.exit(1)

    rclpy.init()
    node = GazeboNavigatorNode(
        patrol_mode=args.patrol and (args.goal is None),
        custom_goal=(args.goal[0], args.goal[1]) if args.goal else None
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        print("\n[Gazebo Nav] Controller cleanly shut down.")


if __name__ == "__main__":
    main()
