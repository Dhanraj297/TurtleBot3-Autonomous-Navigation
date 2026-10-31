"""
Comprehensive Test Runner & Verification Suite
Robotics Laboratory ESE

Executes all unit and integration tests, verifying:
1. Differential drive forward, reverse, and rotation kinematics.
2. 360-degree LiDAR raycasting and obstacle detection.
3. A* optimal global path planning and obstacle detours.
4. Reactive obstacle avoidance and emergency stopping.
5. End-to-end autonomous navigation and goal arrival.
"""

import unittest
import sys
import os
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from tests.test_kinematics import TestTurtleBot3Kinematics
from tests.test_lidar import TestSimulatedLiDAR
from tests.test_a_star import TestAStarPlanner
from tests.test_obstacle_avoidance import TestObstacleAvoidance
from tests.test_navigation_integration import TestNavigationIntegration


def run_full_test_suite():
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    suite.addTests(loader.loadTestsFromTestCase(TestTurtleBot3Kinematics))
    suite.addTests(loader.loadTestsFromTestCase(TestSimulatedLiDAR))
    suite.addTests(loader.loadTestsFromTestCase(TestAStarPlanner))
    suite.addTests(loader.loadTestsFromTestCase(TestObstacleAvoidance))
    suite.addTests(loader.loadTestsFromTestCase(TestNavigationIntegration))

    print("\n" + "=" * 65)
    print("      ROBOTICS LABORATORY ESE: AUTOMATED TEST SUITE      ")
    print("=" * 65)
    start_time = time.time()

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    elapsed = time.time() - start_time
    print("=" * 65)
    print(f"Total Tests Run : {result.testsRun}")
    print(f"Passed          : {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures        : {len(result.failures)}")
    print(f"Errors          : {len(result.errors)}")
    print(f"Execution Time  : {elapsed:.2f} seconds")
    print("=" * 65)

    if result.wasSuccessful():
        print(">>> ALL VERIFICATION TESTS PASSED SUCCESSFULLY! <<<\n")
        return 0
    else:
        print(">>> SOME TESTS FAILED. PLEASE CHECK THE TRACEBACK ABOVE. <<<\n")
        return 1


if __name__ == "__main__":
    sys.exit(run_full_test_suite())
