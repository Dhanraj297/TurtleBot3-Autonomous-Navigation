"""
Unit Tests: Reactive Obstacle Avoidance (APF & Sector Gating)
Robotics Laboratory ESE
"""

import math
import unittest
import sys
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.obstacle_avoidance import ObstacleAvoidance


class TestObstacleAvoidance(unittest.TestCase):
    """Verifies reactive obstacle avoidance logic and emergency stopping."""

    def setUp(self):
        self.avoidance = ObstacleAvoidance(
            emergency_stop_dist=0.18,
            warning_dist=0.50,
            front_sector_deg=60.0
        )

    def test_clear_path_retains_nominal_velocity(self):
        """When all ranges are large, nominal velocities should pass through unchanged."""
        ranges = [2.5] * 360
        nom_v = 0.20
        nom_w = 0.10

        safe_v, safe_w, status = self.avoidance.compute_avoidance_twist(nom_v, nom_w, ranges)

        self.assertEqual(status, "CLEAR")
        self.assertAlmostEqual(safe_v, nom_v, places=4)
        self.assertAlmostEqual(safe_w, nom_w, places=4)

    def test_emergency_stop_trigger(self):
        """When an obstacle is closer than 0.18m in front, linear velocity must become 0.0."""
        ranges = [2.0] * 360
        # Place close obstacle directly ahead (index 0)
        ranges[0] = 0.12

        safe_v, safe_w, status = self.avoidance.compute_avoidance_twist(0.20, 0.0, ranges)

        self.assertEqual(status, "EMERGENCY_STOP")
        self.assertEqual(safe_v, 0.0, "Linear velocity must be zero during emergency stop")
        self.assertNotEqual(safe_w, 0.0, "Robot should rotate to escape collision")

    def test_repulsive_steering_away_from_left_obstacle(self):
        """When obstacle is on the left, robot should steer to the right (negative omega)."""
        ranges = [2.0] * 360
        # Obstacle at 30 degrees left (index 30) at warning distance 0.35m
        ranges[30] = 0.35

        safe_v, safe_w, status = self.avoidance.compute_avoidance_twist(0.20, 0.0, ranges)

        self.assertEqual(status, "REPULSION_ACTIVE")
        self.assertLess(safe_w, 0.0, "Should steer right to avoid obstacle on the left")
        self.assertLess(safe_v, 0.20, "Should slow down when in warning zone")


if __name__ == "__main__":
    unittest.main()
