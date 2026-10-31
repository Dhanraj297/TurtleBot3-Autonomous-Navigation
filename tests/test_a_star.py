"""
Unit Tests: A* Global Path Planning Algorithm
Robotics Laboratory ESE
"""

import unittest
import numpy as np
import sys
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.a_star import AStarPlanner


class TestAStarPlanner(unittest.TestCase):
    """Verifies A* heuristic search, optimality, and obstacle avoidance."""

    def setUp(self):
        self.planner = AStarPlanner(resolution=0.10)
        # Create a 50x50 grid (5.0m x 5.0m arena)
        self.grid = np.zeros((50, 50), dtype=np.int8)
        self.meta = {
            "resolution": 0.10,
            "width": 50,
            "height": 50,
            "origin_x": 0.0,
            "origin_y": 0.0
        }
        self.planner.set_map(self.grid, self.meta)

    def test_straight_line_path_in_free_space(self):
        """Verifies A* connects start to goal across empty space."""
        start = (1.0, 1.0)
        goal = (4.0, 4.0)

        path = self.planner.plan(start, goal, smooth=True)

        self.assertIsNotNone(path)
        self.assertGreaterEqual(len(path), 2)
        self.assertAlmostEqual(path[0][0], start[0], delta=0.05)
        self.assertAlmostEqual(path[0][1], start[1], delta=0.05)
        self.assertAlmostEqual(path[-1][0], goal[0], delta=0.05)
        self.assertAlmostEqual(path[-1][1], goal[1], delta=0.05)

    def test_path_detours_around_wall(self):
        """
        Places a vertical wall barrier between start (1.0, 2.5) and goal (4.0, 2.5).
        Wall is located at column 25 (x = 2.5m) extending from y=1.0 to y=4.0.
        Verifies A* routes around the wall without penetrating it.
        """
        # Wall at col 25, rows 10 to 40
        self.grid[10:40, 25] = 100
        self.planner.set_map(self.grid, self.meta)

        start = (1.0, 2.5)
        goal = (4.0, 2.5)

        path = self.planner.plan(start, goal, smooth=False)

        self.assertIsNotNone(path)
        # Verify no waypoint in path touches an occupied cell
        for wx, wy in path:
            c, r = self.planner.world_to_grid(wx, wy)
            self.assertFalse(self.grid[r, c] > 50, f"Path waypoint ({wx}, {wy}) collides with obstacle cell ({c}, {r})")

    def test_unreachable_goal_returns_none(self):
        """Verifies A* gracefully returns None when goal is completely enclosed by walls."""
        # Enclose cell (40, 40) in a box
        self.grid[38:43, 38] = 100
        self.grid[38:43, 42] = 100
        self.grid[38, 38:43] = 100
        self.grid[42, 38:43] = 100
        self.planner.set_map(self.grid, self.meta)

        start = (1.0, 1.0)
        goal = (4.0, 4.0)  # inside box

        path = self.planner.plan(start, goal)
        self.assertIsNone(path)


if __name__ == "__main__":
    unittest.main()
