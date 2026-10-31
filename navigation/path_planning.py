"""
Autonomous Navigation Coordinator & Waypoint Navigator
Robotics Laboratory ESE

This module coordinates global A* path planning, local obstacle avoidance,
waypoint tracking, and mission state transitions.
It implements the core logic taught in the Mecharithm Lesson 9 auto_nav package,
enabling fully autonomous navigation without manual GUI intervention.

Finite State Machine (FSM):
    IDLE -> PLANNING -> NAVIGATING <-> OBSTACLE_AVOIDANCE -> ALIGNING_ORIENTATION -> GOAL_REACHED
"""

import math
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum
from navigation.a_star import AStarPlanner
from navigation.obstacle_avoidance import ObstacleAvoidance
from robot.controllers.diff_drive import DifferentialDriveController


class NavState(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    AVOIDING_OBSTACLE = "AVOIDING_OBSTACLE"
    ALIGNING_ORIENTATION = "ALIGNING_ORIENTATION"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"


class AutonomousNavigator:
    """High-level autonomous navigation manager."""

    def __init__(
        self,
        planner: AStarPlanner,
        controller: DifferentialDriveController,
        avoidance: ObstacleAvoidance,
        waypoint_switch_dist: float = 0.25,
        goal_tolerance_dist: float = 0.08,
        goal_tolerance_angle: float = 0.15
    ):
        self.planner = planner
        self.controller = controller
        self.avoidance = avoidance

        self.waypoint_switch_dist = waypoint_switch_dist
        self.goal_tol_dist = goal_tolerance_dist
        self.goal_tol_angle = goal_tolerance_angle

        # Navigation mission state
        self.state = NavState.IDLE
        self.current_goal: Optional[Tuple[float, float, float]] = None
        self.path: List[Tuple[float, float]] = []
        self.current_waypoint_idx: int = 0

        # Telemetry
        self.replan_count: int = 0
        self.status_message: str = "Ready for navigation goal"

    def set_goal(
        self,
        robot_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float]
    ) -> bool:
        """
        Commands the robot to autonomously navigate to a target destination (x, y, theta).
        Calculates global A* collision-free path.
        """
        self.current_goal = goal_pose
        self.state = NavState.PLANNING
        self.status_message = f"Planning path to goal ({goal_pose[0]:.2f}, {goal_pose[1]:.2f})..."

        # Plan trajectory using A*
        start_pt = (robot_pose[0], robot_pose[1])
        goal_pt = (goal_pose[0], goal_pose[1])
        planned_path = self.planner.plan(start_pt, goal_pt, smooth=True)

        if planned_path and len(planned_path) >= 1:
            self.path = planned_path
            self.current_waypoint_idx = 0
            self.state = NavState.NAVIGATING
            self.status_message = f"Path planned with {len(self.path)} waypoints. Navigating..."
            return True
        else:
            self.state = NavState.FAILED
            self.status_message = "Path planning failed: No valid collision-free path to goal!"
            return False

    def update(
        self,
        robot_pose: Tuple[float, float, float],
        lidar_ranges: List[float],
        dt: float = 0.05
    ) -> Tuple[float, float]:
        """
        Executes one navigation cycle:
        1. Checks arrival condition at goal.
        2. Advances through intermediate waypoints.
        3. Computes nominal tracking velocity.
        4. Blends with reactive LiDAR obstacle avoidance.
        Returns:
            (cmd_v, cmd_omega): Commanded linear and angular velocities.
        """
        rx, ry, rtheta = robot_pose

        if self.state in [NavState.IDLE, NavState.GOAL_REACHED, NavState.FAILED]:
            return 0.0, 0.0

        if not self.path or self.current_goal is None:
            self.state = NavState.IDLE
            return 0.0, 0.0

        gx, gy, gtheta = self.current_goal
        dist_to_final_goal = math.hypot(gx - rx, gy - ry)

        # 1. Check if final destination position reached
        if dist_to_final_goal <= self.goal_tol_dist:
            self.state = NavState.ALIGNING_ORIENTATION
            self.status_message = "Destination reached. Aligning final orientation..."

            # In-place final heading alignment
            heading_err = DifferentialDriveController.normalize_angle(gtheta - rtheta)
            if abs(heading_err) <= self.goal_tol_angle:
                self.state = NavState.GOAL_REACHED
                self.status_message = f"Goal successfully reached at ({gx:.2f}, {gy:.2f})!"
                return 0.0, 0.0
            else:
                omega = 2.0 * heading_err
                omega = max(-1.8, min(1.8, omega))
                return 0.0, omega

        # 2. Advance through waypoints
        while self.current_waypoint_idx < len(self.path) - 1:
            wp_x, wp_y = self.path[self.current_waypoint_idx]
            if math.hypot(wp_x - rx, wp_y - ry) < self.waypoint_switch_dist:
                self.current_waypoint_idx += 1
            else:
                break

        target_wp = self.path[self.current_waypoint_idx]

        # 3. Calculate nominal tracking velocity towards waypoint
        nominal_v, nominal_omega, _ = self.controller.compute_waypoint_velocity(
            rx, ry, rtheta,
            target_wp[0], target_wp[1],
            target_theta=None,
            dt=dt
        )

        # 4. Filter through reactive obstacle avoidance
        safe_v, safe_omega, avoidance_status = self.avoidance.compute_avoidance_twist(
            nominal_v, nominal_omega, lidar_ranges
        )

        if avoidance_status == "EMERGENCY_STOP":
            self.state = NavState.AVOIDING_OBSTACLE
            self.status_message = "Obstacle detected in immediate path! Performing avoidance maneuver..."
        elif avoidance_status == "REPULSION_ACTIVE":
            self.state = NavState.AVOIDING_OBSTACLE
            self.status_message = "Repulsive field active: Steer-clear maneuver in progress."
        else:
            self.state = NavState.NAVIGATING
            self.status_message = f"Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.path)}"

        return safe_v, safe_omega

    def cancel_navigation(self):
        """Cancels current mission and stops robot."""
        self.state = NavState.IDLE
        self.path = []
        self.current_goal = None
        self.status_message = "Navigation cancelled by user."
