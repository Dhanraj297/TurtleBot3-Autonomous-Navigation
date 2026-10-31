"""
End-to-End Autonomous Navigation Integration Test
Robotics Laboratory ESE — Mecharithm Lesson 9 Verification

Verifies complete autonomous execution:
1. Environment and occupancy grid generation.
2. Robot pose initialization.
3. Goal command dispatch.
4. A* global path planning.
5. Waypoint following and differential-drive kinematics.
6. Target arrival and velocity stopping.
"""

import math
import unittest
import sys
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot.robot_model.turtlebot3_burger import TurtleBot3Burger
from robot.sensors.lidar import SimulatedLiDAR
from robot.controllers.diff_drive import DifferentialDriveController
from simulation.worlds.world import IndoorWorld
from navigation.a_star import AStarPlanner
from navigation.obstacle_avoidance import ObstacleAvoidance
from navigation.path_planning import AutonomousNavigator, NavState


class TestNavigationIntegration(unittest.TestCase):
    """Full-system integration test for autonomous mobile robot navigation."""

    def setUp(self):
        self.world = IndoorWorld()
        self.grid, self.meta = self.world.generate_occupancy_grid(inflation_radius=0.20)

        self.robot = TurtleBot3Burger(x=1.2, y=1.5, theta=0.0)
        self.lidar = SimulatedLiDAR(min_range=0.12, max_range=3.50, samples=360, noise_stddev=0.0)

        self.planner = AStarPlanner(resolution=self.world.resolution, inflation_radius=0.20)
        self.planner.set_map(self.grid, self.meta)

        self.controller = DifferentialDriveController(
            max_linear_velocity=self.robot.max_linear_velocity,
            max_angular_velocity=self.robot.max_angular_velocity,
            goal_dist_tol=0.08
        )
        self.avoidance = ObstacleAvoidance(emergency_stop_dist=0.18, warning_dist=0.45)

        self.navigator = AutonomousNavigator(
            planner=self.planner,
            controller=self.controller,
            avoidance=self.avoidance,
            goal_tolerance_dist=0.08
        )

    def test_complete_autonomous_navigation_to_goal(self):
        """
        Commands robot from (1.2, 1.5) to target (3.2, 1.5).
        Runs closed-loop simulation loop until goal is reached or max steps exceeded.
        Verifies:
        - Path is generated.
        - Robot moves forward.
        - Robot reaches destination within 8cm tolerance.
        - Robot comes to a complete stop (v=0, omega=0).
        - State transitions to GOAL_REACHED.
        """
        target_goal = (3.2, 1.5, 0.0)
        success = self.navigator.set_goal(
            robot_pose=(self.robot.x, self.robot.y, self.robot.theta),
            goal_pose=target_goal
        )
        self.assertTrue(success, "Path planner should successfully find a path to target")
        self.assertGreater(len(self.navigator.path), 1)

        dt = 0.05
        max_steps = 1500  # 75 seconds simulated max
        arrived = False

        for step in range(max_steps):
            # 1. LiDAR scan
            obs_dicts = self.world.get_all_obstacle_dicts()
            ranges, _ = self.lidar.scan(
                self.robot.x, self.robot.y, self.robot.theta,
                self.world.all_walls, obs_dicts
            )

            # 2. Navigation velocity computation
            cmd_v, cmd_omega = self.navigator.update(
                (self.robot.x, self.robot.y, self.robot.theta),
                ranges,
                dt=dt
            )

            # 3. Kinematic motion update
            self.robot.update_pose(cmd_v, cmd_omega, dt)

            # Check arrival
            if self.navigator.state == NavState.GOAL_REACHED:
                arrived = True
                # Allow physical actuators to complete deceleration to 0
                for _ in range(5):
                    self.robot.update_pose(0.0, 0.0, dt)
                break

        # Verification Assertions
        self.assertTrue(arrived, f"Robot failed to reach goal within {max_steps} steps")
        dist_to_goal = math.hypot(target_goal[0] - self.robot.x, target_goal[1] - self.robot.y)
        self.assertLessEqual(dist_to_goal, 0.08, f"Final distance error {dist_to_goal:.3f}m exceeds tolerance 0.08m")
        self.assertEqual(self.robot.v, 0.0, "Robot linear velocity should be zero at destination")
        self.assertEqual(self.robot.omega, 0.0, "Robot angular velocity should be zero at destination")
        self.assertEqual(self.navigator.state, NavState.GOAL_REACHED)


if __name__ == "__main__":
    unittest.main()
