"""
Demonstration & Screenshot Recorder
Robotics Laboratory ESE

Runs the simulation through representative navigation scenarios,
capturing high-resolution screenshots for the final laboratory report,
viva presentation, and PPT.
"""

import os
import sys
import math
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from scripts.main import TurtleBot3SimulationNode


def record_demonstration_screenshots():
    shots_dir = os.path.join(PROJECT_ROOT, "screenshots")
    os.makedirs(shots_dir, exist_ok=True)

    print("[Recorder] Launching simulation to capture visual evidence...")
    # Initialize visual simulation
    sim = TurtleBot3SimulationNode(headless=False)
    dt = 0.05

    # 1. Capture Initial State (Start pose, walls, planned A* path, goal marker)
    print("[Recorder] Capturing Stage 1: Initial World & Planned Path...")
    for _ in range(15):
        sim.step(dt)
    sim.visualizer.capture_screenshot(os.path.join(shots_dir, "01_initial_world_and_path.png"))

    # 2. Capture Navigation in Progress (Robot moving, golden trail, live LiDAR rays)
    print("[Recorder] Capturing Stage 2: Autonomous Navigation in Progress...")
    for _ in range(90):  # Drive for 4.5 seconds
        sim.step(dt)
    sim.visualizer.capture_screenshot(os.path.join(shots_dir, "02_autonomous_navigation_in_progress.png"))

    # 3. Insert an obstacle in front and capture Obstacle Avoidance maneuver
    print("[Recorder] Capturing Stage 3: Dynamic Obstacle Avoidance...")
    # Place an obstacle directly in front of the robot's current position
    rx, ry, rtheta = sim.robot.x, sim.robot.y, sim.robot.theta
    obs_x = rx + 0.45 * math.cos(rtheta)
    obs_y = ry + 0.45 * math.sin(rtheta)
    sim.world.add_dynamic_obstacle(obs_x, obs_y, radius=0.25, label="Intervention Obstacle")
    # Step into avoidance
    for _ in range(40):
        sim.step(dt)
    sim.visualizer.capture_screenshot(os.path.join(shots_dir, "03_lidar_sensing_and_obstacle_avoidance.png"))

    # 4. Clear obstacle, command close goal to capture Goal Reached Success
    print("[Recorder] Capturing Stage 4: Destination Arrival & Stopping...")
    # Set goal just ahead so it reaches promptly
    cur_x, cur_y = sim.robot.x, sim.robot.y
    close_goal = (cur_x + 0.50, cur_y + 0.30, 0.0)
    sim.set_goal(close_goal)
    for _ in range(120):
        sim.step(dt)
        if sim.navigator.state.value == "GOAL_REACHED":
            break
    sim.visualizer.capture_screenshot(os.path.join(shots_dir, "04_goal_reached_success.png"))

    sim.visualizer.close()
    print("[Recorder] All 4 demonstration screenshots recorded successfully!")


if __name__ == "__main__":
    record_demonstration_screenshots()
