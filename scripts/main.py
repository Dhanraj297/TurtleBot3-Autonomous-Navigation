"""
TurtleBot3 Autonomous Navigation Master Orchestrator
Robotics Laboratory ESE — Mecharithm Lesson 9 Implementation

This script unites:
1. Simulated TurtleBot3 Burger differential-drive kinematics.
2. 360-degree LiDAR raycasting sensor simulation.
3. Indoor laboratory arena with walls, partitions, and dynamic obstacles.
4. Global A* path planning with Euclidean heuristic and line-of-sight smoothing.
5. Reactive local obstacle avoidance (Artificial Potential Field).
6. Closed-loop PID waypoint trajectory follower.
7. Genuine ROS 2 Humble node graph (publishing /scan, /odom, subscribing /cmd_vel, /goal_pose).
8. Interactive 60 FPS graphical simulation window with live telemetry HUD.
"""

import os
import sys
import math
import time
import argparse
from typing import Tuple, List, Optional, Dict, Any
import pygame

# Add project root to Python module search path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot.robot_model.turtlebot3_burger import TurtleBot3Burger
from robot.sensors.lidar import SimulatedLiDAR
from robot.controllers.diff_drive import DifferentialDriveController
from simulation.worlds.world import IndoorWorld
from simulation.visualizer import SimulationVisualizer
from navigation.a_star import AStarPlanner
from navigation.obstacle_avoidance import ObstacleAvoidance
from navigation.localization import OdometryLocalizer
from navigation.path_planning import AutonomousNavigator, NavState

# Attempt ROS 2 rclpy import
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class TurtleBot3SimulationNode:
    """ROS 2 wrapper and simulation execution coordinator."""

    def __init__(self, headless: bool = False, config_path: str = None):
        self.headless = headless

        # 1. Initialize Simulation World
        if config_path is None:
            config_path = os.path.join(PROJECT_ROOT, "config", "world_config.yaml")
        self.world = IndoorWorld(config_path)

        # 2. Initialize Robot & Odometry at start pose
        sx, sy, stheta = self.world.start_pose
        self.robot = TurtleBot3Burger(x=sx, y=sy, theta=stheta)
        self.localizer = OdometryLocalizer(initial_x=sx, initial_y=sy, initial_theta=stheta)

        # 3. Initialize LiDAR Sensor
        self.lidar = SimulatedLiDAR(min_range=0.12, max_range=3.50, samples=360, noise_stddev=0.008)

        # 4. Initialize Navigation Subsystems
        self.grid, self.grid_meta = self.world.generate_occupancy_grid(inflation_radius=0.22)
        self.planner = AStarPlanner(resolution=self.world.resolution, inflation_radius=0.22)
        self.planner.set_map(self.grid, self.grid_meta)

        self.diff_controller = DifferentialDriveController(
            max_linear_velocity=self.robot.max_linear_velocity,
            max_angular_velocity=self.robot.max_angular_velocity,
            kp_linear=0.8,
            kp_angular=2.2
        )
        self.avoidance = ObstacleAvoidance(emergency_stop_dist=0.18, warning_dist=0.50, repulsion_gain=0.9)

        self.navigator = AutonomousNavigator(
            planner=self.planner,
            controller=self.diff_controller,
            avoidance=self.avoidance,
            waypoint_switch_dist=0.25,
            goal_tolerance_dist=0.08
        )

        # 5. Initialize Visualizer
        self.visualizer = SimulationVisualizer(
            world_width=self.world.width,
            world_height=self.world.height,
            window_width=1200,
            window_height=800,
            headless=self.headless
        )

        # Interaction Mode: 'AUTONOMOUS' or 'TELEOPERATION'
        self.mode = "AUTONOMOUS"
        self.teleop_v = 0.0
        self.teleop_omega = 0.0

        # ROS 2 Communication Objects
        self.ros_node = None
        self.pub_scan = None
        self.pub_odom = None
        self.sub_cmd_vel = None
        self.sub_goal_pose = None
        self.init_ros2()

        # Set default goal from world configuration
        self.set_goal(self.world.goal_pose)

    def init_ros2(self):
        """Initializes ROS 2 Humble node and standard publishers/subscribers."""
        if not ROS2_AVAILABLE:
            print("[ROS 2] Warning: rclpy not detected. Running pure Python simulation mode.")
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            self.ros_node = rclpy.create_node("turtlebot3_simulation_node")
            self.pub_scan = self.ros_node.create_publisher(LaserScan, "/scan", 10)
            self.pub_odom = self.ros_node.create_publisher(Odometry, "/odom", 10)
            self.sub_cmd_vel = self.ros_node.create_subscription(
                Twist, "/cmd_vel", self.ros_cmd_vel_callback, 10
            )
            self.sub_goal_pose = self.ros_node.create_subscription(
                PoseStamped, "/goal_pose", self.ros_goal_callback, 10
            )
            print("[ROS 2] Successfully initialized node '/turtlebot3_simulation_node' on native Windows.")
        except Exception as e:
            print(f"[ROS 2] Init notice: {e}. Running integrated simulation.")

    def ros_cmd_vel_callback(self, msg):
        """Subscribes to external ROS 2 /cmd_vel messages."""
        if self.mode == "TELEOPERATION":
            self.teleop_v = float(msg.linear.x)
            self.teleop_omega = float(msg.angular.z)

    def ros_goal_callback(self, msg):
        """Subscribes to external ROS 2 /goal_pose messages."""
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        # Convert quaternion to yaw
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        gyaw = 2.0 * math.atan2(qz, qw)
        print(f"[ROS 2] Received new goal from /goal_pose: ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.1f} deg)")
        self.set_goal((gx, gy, gyaw))

    def set_goal(self, goal_pose: Tuple[float, float, float]):
        """Sets target goal and initiates A* path planning."""
        self.navigator.set_goal(
            robot_pose=(self.robot.x, self.robot.y, self.robot.theta),
            goal_pose=goal_pose
        )

    def handle_input_events(self) -> bool:
        """Processes interactive mouse clicks and keyboard commands."""
        if self.headless:
            return True

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                # Check if click is inside arena area
                if mx < self.visualizer.arena_w:
                    wx, wy = self.visualizer.screen_to_world(mx, my)
                    if event.button == 1:  # Left click: Set new navigation goal
                        print(f"[UI] Setting new Goal Pose at: ({wx:.2f}, {wy:.2f})")
                        self.set_goal((wx, wy, 0.0))
                    elif event.button == 3:  # Right click: Place dynamic obstacle
                        print(f"[UI] Placing dynamic obstacle at: ({wx:.2f}, {wy:.2f})")
                        self.world.add_dynamic_obstacle(wx, wy, radius=0.28)
                        # Re-rasterize grid for future replanning
                        self.grid, self.grid_meta = self.world.generate_occupancy_grid(0.22)
                        self.planner.set_map(self.grid, self.grid_meta)

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_t:
                    self.mode = "TELEOPERATION" if self.mode == "AUTONOMOUS" else "AUTONOMOUS"
                    print(f"[UI] Mode toggled to: {self.mode}")
                elif event.key == pygame.K_a or event.key == pygame.K_SPACE:
                    self.mode = "AUTONOMOUS"
                    if self.navigator.current_goal:
                        self.set_goal(self.navigator.current_goal)
                    print("[UI] Autonomous Navigation Triggered.")
                elif event.key == pygame.K_r:
                    sx, sy, stheta = self.world.start_pose
                    self.robot.reset_pose(sx, sy, stheta)
                    self.localizer.set_pose(sx, sy, stheta)
                    if self.navigator.current_goal:
                        self.set_goal(self.navigator.current_goal)
                    print("[UI] Reset robot to starting pose.")
                elif event.key == pygame.K_c:
                    self.world.create_default_world()
                    self.grid, self.grid_meta = self.world.generate_occupancy_grid(0.22)
                    self.planner.set_map(self.grid, self.grid_meta)
                    print("[UI] Cleared custom obstacles.")
                elif event.key == pygame.K_s:
                    shot_path = os.path.join(PROJECT_ROOT, "screenshots", f"nav_screen_{int(time.time())}.png")
                    self.visualizer.capture_screenshot(shot_path)

        # Handle continuous keyboard teleoperation
        if self.mode == "TELEOPERATION" and not self.headless:
            keys = pygame.key.get_pressed()
            cmd_v = 0.0
            cmd_omega = 0.0
            if keys[pygame.K_w] or keys[pygame.K_UP]:
                cmd_v += 0.20
            if keys[pygame.K_s] or keys[pygame.K_DOWN]:
                cmd_v -= 0.15
            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                cmd_omega += 1.5
            if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                cmd_omega -= 1.5
            self.teleop_v = cmd_v
            self.teleop_omega = cmd_omega

        return True

    def step(self, dt: float = 0.05):
        """Simulates one time step of the complete robotic system."""
        # 1. Update moving obstacles
        self.world.update(dt)

        # 2. Acquire simulated 360-degree LiDAR scan
        obs_dicts = self.world.get_all_obstacle_dicts()
        ranges, hit_points = self.lidar.scan(
            self.robot.x, self.robot.y, self.robot.theta,
            self.world.all_walls, obs_dicts
        )

        # 3. Determine velocity commands based on operating mode
        if self.mode == "AUTONOMOUS":
            cmd_v, cmd_omega = self.navigator.update(
                (self.robot.x, self.robot.y, self.robot.theta),
                ranges,
                dt=dt
            )
        else:
            cmd_v = self.teleop_v
            cmd_omega = self.teleop_omega

        # 4. Integrate Differential Drive Kinematics
        self.robot.update_pose(cmd_v, cmd_omega, dt)

        # 5. Integrate Odometry Dead Reckoning
        self.localizer.update_from_wheel_speeds(
            self.robot.omega_left, self.robot.omega_right, dt
        )

        # 6. Publish ROS 2 Messages
        now_sec = time.time()
        if self.ros_node is not None:
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.001)
                scan_msg = self.lidar.create_ros_laserscan_msg(ranges, timestamp_sec=now_sec)
                odom_msg = self.localizer.create_ros_odometry_msg(timestamp_sec=now_sec)
                if self.pub_scan:
                    self.pub_scan.publish(scan_msg)
                if self.pub_odom:
                    self.pub_odom.publish(odom_msg)
            except Exception:
                pass

        # 7. Package Telemetry for Visualizer HUD
        min_clearance = min([r for r in ranges if not math.isinf(r)] + [self.lidar.max_range])
        goal = self.navigator.current_goal or (0.0, 0.0, 0.0)
        dist_to_goal = math.hypot(goal[0] - self.robot.x, goal[1] - self.robot.y)

        telemetry = {
            "mode": self.mode,
            "state": self.navigator.state.value,
            "x": self.robot.x,
            "y": self.robot.y,
            "theta_deg": math.degrees(self.robot.theta),
            "v": self.robot.v,
            "omega": self.robot.omega,
            "goal_x": goal[0],
            "goal_y": goal[1],
            "dist_to_goal": dist_to_goal,
            "min_lidar_dist": min_clearance,
            "waypoints_left": max(0, len(self.navigator.path) - self.navigator.current_waypoint_idx),
            "total_waypoints": len(self.navigator.path),
            "dist_traveled": self.robot.distance_traveled,
            "status_message": self.navigator.status_message
        }

        # 8. Render Frame
        self.visualizer.render(
            robot_pose=(self.robot.x, self.robot.y, self.robot.theta),
            robot_chassis_radius=self.robot.chassis_radius,
            trajectory_history=self.robot.trajectory_history,
            planned_path=self.navigator.path,
            goal_pose=self.navigator.current_goal,
            walls=self.world.all_walls,
            obstacles=obs_dicts,
            lidar_hit_points=hit_points,
            telemetry=telemetry
        )

        return telemetry

    def run(self, max_seconds: Optional[float] = None):
        """Runs the main real-time loop."""
        print("[Simulation] Starting TurtleBot3 Autonomous Navigation loop.")
        running = True
        start_time = time.time()
        prev_time = time.time()

        try:
            while running:
                now = time.time()
                dt = min(0.05, max(0.01, now - prev_time))
                prev_time = now

                running = self.handle_input_events()
                telemetry = self.step(dt)

                if max_seconds and (now - start_time) >= max_seconds:
                    print(f"[Simulation] Reached target runtime: {max_seconds} seconds.")
                    break

        except KeyboardInterrupt:
            print("[Simulation] Interrupted by user.")
        finally:
            self.visualizer.close()
            if self.ros_node is not None and ROS2_AVAILABLE:
                try:
                    self.ros_node.destroy_node()
                    rclpy.shutdown()
                except Exception:
                    pass
            print("[Simulation] Shutdown cleanly.")


def main():
    parser = argparse.ArgumentParser(description="TurtleBot3 Autonomous Navigation (Mecharithm Lesson 9)")
    parser.add_argument("--headless", action="store_true", help="Run without graphical display")
    parser.add_argument("--duration", type=float, default=None, help="Maximum execution time in seconds")
    parser.add_argument("--config", type=str, default=None, help="Custom world configuration YAML")
    args = parser.parse_args()

    sim = TurtleBot3SimulationNode(headless=args.headless, config_path=args.config)
    sim.run(max_seconds=args.duration)


if __name__ == "__main__":
    main()
