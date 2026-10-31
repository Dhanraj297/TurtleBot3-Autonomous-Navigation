"""
Unit Tests: 360-Degree LiDAR Raycaster & Sensor Accuracy
Robotics Laboratory ESE
"""

import math
import unittest
import sys
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot.sensors.lidar import SimulatedLiDAR


class TestSimulatedLiDAR(unittest.TestCase):
    """Verifies raycasting accuracy and obstacle detection."""

    def setUp(self):
        # Deterministic zero-noise sensor for unit testing
        self.lidar = SimulatedLiDAR(min_range=0.12, max_range=3.50, samples=360, noise_stddev=0.0)

    def test_wall_detection_distance(self):
        """
        Places robot at (2.0, 2.0) facing theta=0 (East).
        Wall is located at x=3.5 (1.5 meters directly ahead).
        Verifies forward laser beam (index 0) measures exactly 1.50 meters.
        """
        walls = [[3.5, 0.0, 3.5, 5.0]]  # Vertical wall at x=3.5
        obstacles = []

        ranges, _ = self.lidar.scan(robot_x=2.0, robot_y=2.0, robot_theta=0.0, walls=walls, obstacles=obstacles)

        # Beam 0 corresponds to relative angle 0.0 (directly forward)
        self.assertAlmostEqual(ranges[0], 1.50, delta=0.02)

    def test_cylinder_obstacle_detection(self):
        """
        Places robot at (1.0, 1.0) facing theta=0 (East).
        A cylinder obstacle with center at (3.0, 1.0) and radius 0.40m is placed directly ahead.
        Expected distance = (3.0 - 1.0) - radius = 2.0 - 0.40 = 1.60 meters.
        """
        walls = []
        obstacles = [{"type": "circle", "x": 3.0, "y": 1.0, "radius": 0.40}]

        ranges, _ = self.lidar.scan(robot_x=1.0, robot_y=1.0, robot_theta=0.0, walls=walls, obstacles=obstacles)

        self.assertAlmostEqual(ranges[0], 1.60, delta=0.02)

    def test_360_sample_completeness(self):
        """Verifies exactly 360 laser range beams are returned."""
        walls = [[0.0, 0.0, 10.0, 0.0]]
        obstacles = []
        ranges, hit_pts = self.lidar.scan(robot_x=5.0, robot_y=5.0, robot_theta=0.0, walls=walls, obstacles=obstacles)

        self.assertEqual(len(ranges), 360)
        self.assertEqual(len(hit_pts), 360)

    def test_out_of_range_beam_returns_infinity(self):
        """Verifies that beams seeing no obstacle within max_range report infinity."""
        walls = []
        obstacles = []
        ranges, _ = self.lidar.scan(robot_x=0.0, robot_y=0.0, robot_theta=0.0, walls=walls, obstacles=obstacles)

        for r in ranges:
            self.assertTrue(math.isinf(r))


if __name__ == "__main__":
    unittest.main()
