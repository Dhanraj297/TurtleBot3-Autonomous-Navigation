"""
Unit Tests: TurtleBot3 Differential Drive Kinematics
Robotics Laboratory ESE
"""

import math
import unittest
import sys
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot.robot_model.turtlebot3_burger import TurtleBot3Burger


class TestTurtleBot3Kinematics(unittest.TestCase):
    """Verifies differential-drive kinematics and velocity limits."""

    def setUp(self):
        self.robot = TurtleBot3Burger(x=0.0, y=0.0, theta=0.0)

    def test_forward_linear_motion(self):
        """Verifies robot moves forward along X axis when commanded linear velocity."""
        # Drive forward at 0.1 m/s for 1.0 second in steps of 0.1s
        dt = 0.1
        for _ in range(10):
            self.robot.update_pose(cmd_v=0.10, cmd_omega=0.0, dt=dt)

        self.assertAlmostEqual(self.robot.x, 0.10, delta=0.015)
        self.assertAlmostEqual(self.robot.y, 0.0, delta=0.005)
        self.assertAlmostEqual(self.robot.theta, 0.0, delta=0.005)
        self.assertGreater(self.robot.distance_traveled, 0.08)

    def test_reverse_linear_motion(self):
        """Verifies robot moves backward along X axis when commanded negative velocity."""
        dt = 0.1
        for _ in range(10):
            self.robot.update_pose(cmd_v=-0.10, cmd_omega=0.0, dt=dt)

        self.assertAlmostEqual(self.robot.x, -0.10, delta=0.015)
        self.assertAlmostEqual(self.robot.y, 0.0, delta=0.005)

    def test_pure_in_place_rotation(self):
        """Verifies robot rotates without linear displacement when commanded omega."""
        dt = 0.05
        target_angle = math.pi / 2.0
        while self.robot.theta < target_angle:
            self.robot.update_pose(cmd_v=0.0, cmd_omega=1.0, dt=dt)

        # Position should stay at origin (pure in-place rotation)
        self.assertAlmostEqual(self.robot.x, 0.0, delta=0.005)
        self.assertAlmostEqual(self.robot.y, 0.0, delta=0.005)
        # Orientation reached target
        self.assertAlmostEqual(self.robot.theta, target_angle, delta=0.1)

    def test_inverse_forward_kinematics_consistency(self):
        """Verifies that forward kinematics perfectly inverts inverse kinematics."""
        target_v = 0.15
        target_omega = 0.80

        omega_l, omega_r = self.robot.inverse_kinematics(target_v, target_omega)
        rec_v, rec_omega = self.robot.forward_kinematics(omega_l, omega_r)

        self.assertAlmostEqual(target_v, rec_v, places=5)
        self.assertAlmostEqual(target_omega, rec_omega, places=5)

    def test_velocity_saturation_limits(self):
        """Verifies actuators cannot exceed ROBOTIS official speed limits."""
        dt = 0.1
        # Command excessive speeds
        for _ in range(20):
            self.robot.update_pose(cmd_v=5.0, cmd_omega=10.0, dt=dt)

        self.assertLessEqual(self.robot.v, self.robot.max_linear_velocity + 1e-4)
        self.assertLessEqual(self.robot.omega, self.robot.max_angular_velocity + 1e-4)


if __name__ == "__main__":
    unittest.main()
