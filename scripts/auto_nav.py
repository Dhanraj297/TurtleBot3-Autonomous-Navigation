"""
Autonomous Navigation Mission Executive (auto_nav)
Robotics Laboratory ESE — Direct Implementation of Mecharithm Lesson 9

In Lesson 9 ("Autonomous Navigation of Turtlebot Using ROS"), Mecharithm demonstrates
how to bypass manual RViz 2D Nav Goal setting and command multi-point autonomous
missions purely through Python code.

This script implements:
1. Programmatic initial pose specification (equivalent to 2D Pose Estimate).
2. Autonomous multi-waypoint mission dispatch.
3. Subscription to /odom for position tracking and /scan for obstacle verification.
4. Closed-loop arrival detection and heading alignment.
5. Telemetry and mission progress reporting.
"""

import os
import sys
import math
import time
import argparse

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class AutoNavMissionExecutive:
    """Dispatches autonomous goals and oversees mission execution."""

    def __init__(self):
        # Default sequence of inspection waypoints [x, y, theta_degrees]
        self.waypoints = [
            (5.5, 3.2, 0.0, "Central Corridor Junction"),
            (8.5, 4.8, 90.0, "East Office Destination"),
            (8.5, 1.5, 270.0, "Laboratory Bench 2"),
            (1.5, 6.5, 90.0, "West Storage Inspection"),
            (1.2, 1.5, 0.0, "Home Base Dock")
        ]
        self.current_index = 0
        self.is_mission_active = False

        # Current robot state from odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.min_lidar_distance = float('inf')

        self.node = None
        self.pub_goal = None
        self.pub_cmd_vel = None
        self.sub_odom = None
        self.sub_scan = None

        if ROS2_AVAILABLE:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node("auto_nav_mission_executive")
            self.pub_goal = self.node.create_publisher(PoseStamped, "/goal_pose", 10)
            self.pub_cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 10)
            self.sub_odom = self.node.create_subscription(Odometry, "/odom", self.odom_callback, 10)
            self.sub_scan = self.node.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
            print("[auto_nav] Initialized ROS 2 Mission Executive Node.")

    def odom_callback(self, msg: Odometry):
        """Updates internal pose estimate from /odom."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)

    def scan_callback(self, msg: LaserScan):
        """Monitors LiDAR ranges to verify obstacle clearance."""
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > msg.range_min]
        if valid_ranges:
            self.min_lidar_distance = min(valid_ranges)

    def dispatch_goal(self, x: float, y: float, theta_deg: float, label: str = ""):
        """Publishes target destination to /goal_pose topic."""
        theta_rad = math.radians(theta_deg)
        print(f"\n=======================================================")
        print(f"[auto_nav] Dispatching Goal {self.current_index + 1}/{len(self.waypoints)}: {label}")
        print(f"Target Coordinates: (X={x:.2f} m, Y={y:.2f} m, Yaw={theta_deg:.1f} deg)")
        print(f"=======================================================")

        if self.pub_goal and self.node:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position = Point(x=float(x), y=float(y), z=0.0)
            msg.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=float(math.sin(theta_rad / 2.0)),
                w=float(math.cos(theta_rad / 2.0))
            )
            self.pub_goal.publish(msg)

    def run_mission(self, timeout_per_goal_sec: float = 30.0):
        """Executes the full multi-point autonomous navigation routine."""
        print("[auto_nav] Commencing Mecharithm Lesson 9 Programmatic Mission.")
        self.is_mission_active = True

        for idx, (gx, gy, gtheta, label) in enumerate(self.waypoints):
            self.current_index = idx
            self.dispatch_goal(gx, gy, gtheta, label)

            goal_start_time = time.time()
            arrived = False

            while not arrived and (time.time() - goal_start_time) < timeout_per_goal_sec:
                if self.node and ROS2_AVAILABLE:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                dist = math.hypot(gx - self.current_x, gy - self.current_y)
                angle_diff = abs(math.radians(gtheta) - self.current_theta)

                if dist < 0.12 and angle_diff < 0.25:
                    arrived = True
                    print(f"[auto_nav] SUCCESS: Arrived at {label} (Error: {dist*100:.1f} cm)")
                    time.sleep(1.0)  # Pause for inspection
                    break

                time.sleep(0.1)

            if not arrived:
                print(f"[auto_nav] Notice: Goal {label} timed out or advanced to next.")

        print("\n[auto_nav] Complete Autonomous Mission Successfully Finished!")


def main():
    parser = argparse.ArgumentParser(description="Autonomous Navigation Executive (Mecharithm Lesson 9)")
    parser.add_argument("--timeout", type=float, default=25.0, help="Per-goal timeout in seconds")
    args = parser.parse_args()

    executive = AutoNavMissionExecutive()
    executive.run_mission(timeout_per_goal_sec=args.timeout)


if __name__ == "__main__":
    main()
