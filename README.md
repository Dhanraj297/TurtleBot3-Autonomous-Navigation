# TurtleBot3 Autonomous Navigation Using ROS 2 (Windows 11 Native)

[![Platform: Windows 11](https://img.shields.io/badge/Platform-Windows%2011%20Native-blue.svg)](https://microsoft.com/windows)
[![ROS 2: Humble Hawksbill](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill%20LTS-orange.svg)](https://docs.ros.org/en/humble/)
[![Environment: Pixi Isolated](https://img.shields.io/badge/Env-Pixi%200.83.0-green.svg)](https://pixi.sh)
[![Tests: 16 Passed](https://img.shields.io/badge/Tests-16%20Passed%20(100%25)-brightgreen.svg)](tests/)
[![Tutorial: Mecharithm Lesson 9](https://img.shields.io/badge/Tutorial-Mecharithm%20Lesson%209-red.svg)](https://youtu.be/lW-c88vxLRE)

---

## 1. Project Overview

This repository contains the complete implementation of the **Autonomous Mobile Robot Navigation Project** for the **Robotics Laboratory End-Semester Examination (ESE)**.

The project directly mirrors the architecture and workflow taught in **Mecharithm's Lesson 9 ("Autonomous Navigation of Turtlebot Using ROS", video `lW-c88vxLRE`)**, transitioning from manual RViz 2D Nav Goal setting to **fully programmatic autonomous navigation**, adapted for **100% native execution on Windows 11**.

```
                +-------------------------------------------------------+
                |        High-Level Autonomous Mission Executive        |
                |        (scripts/auto_nav.py - Mecharithm Lesson 9)    |
                +---------------------------+---------------------------+
                                            | Publishes /goal_pose
                                            v
+-----------------------+       +---------------------------+       +-------------------------+
|  Simulated 360 LiDAR  | ----> |  Global A* Path Planner   | ----> |  Differential Drive PID |
|  (/scan LaserScan)    |       |  (navigation/a_star.py)   |       |  (/cmd_vel Twist)       |
+-----------------------+       +---------------------------+       +-------------------------+
            |                                 ^                                  |
            v                                 |                                  v
+-----------------------+                     |                     +-------------------------+
| Reactive Obstacle APF | --------------------+                     |  TurtleBot3 Kinematics  |
| (navigation/avoid.py) |                                           |  (robot/robot_model/)   |
+-----------------------+                                           +-------------------------+
```

---

## 2. Key Features

- **Genuine ROS 2 Humble Graph:** Operates real ROS 2 nodes, publishers, and subscribers (`/scan`, `/odom`, `/cmd_vel`, `/goal_pose`) on Windows 11 via RoboStack and Pixi.
- **Differential-Drive Kinematic Engine:** Accurate physical parameters of TurtleBot3 Burger ($r=0.033$ m, $L=0.160$ m) with Runge-Kutta pose integration and acceleration saturation.
- **360° Planar LiDAR Simulation:** 360 discrete laser beams with 1° resolution, Gaussian noise, and fast 2D raycasting against walls and obstacles.
- **A\* Optimal Global Path Planner:** 8-connected grid search with admissible Euclidean distance heuristic ($f = g + h$) and Line-of-Sight path smoothing.
- **Reactive Local Obstacle Avoidance:** Artificial Potential Field (APF) and sector safety gating to evade unexpected obstacles and prevent collisions.
- **Interactive 60 FPS Visual Simulation Window:** Dark-mode Pygame arena featuring dynamic laser ray rendering, breadcrumb trails, click-to-nav, interactive obstacle placement, and telemetry HUD.
- **Automated Verification Suite:** 16 comprehensive unit and integration tests passing with 100% success.

---

## 3. Quick Start (Execution Guide)

All commands are executed inside the project root (`c:\dev\TurtleBot3_ESE`):

### 1. Launch Interactive Master Simulation (GUI + ROS 2 Node)
```powershell
pixi run python scripts/main.py
```

### 2. Launch Programmatic Autonomous Navigation (Mecharithm Lesson 9)
In a second terminal:
```powershell
pixi run python scripts/auto_nav.py
```

### 3. Launch Keyboard Teleoperation
In a second terminal:
```powershell
pixi run python scripts/teleoperation.py
```

### 4. Run the Complete Automated Verification Test Suite
```powershell
pixi run python tests/run_all_tests.py
```

### 5. Record Demonstration Screenshots
```powershell
pixi run python scripts/record_demonstration.py
```

---

## 4. Interactive Simulation Controls

When running `scripts/main.py`, the following controls are available directly in the simulation window:

| Control | Action |
| :--- | :--- |
| **Left Mouse Click** | Set new 2D Navigation Goal pose (clicks anywhere on the arena floor). |
| **Right Mouse Click** | Place a new dynamic obstacle on the fly (automatically re-routes the bot around it!). |
| **Space / A** | Trigger/restart autonomous navigation to current goal. |
| **T** | Toggle between **Autonomous** and **Manual Keyboard Teleoperation** modes. |
| **W / A / S / D** | Drive robot manually (when in Teleoperation mode). |
| **R** | Reset robot back to starting pose `(1.2, 1.5, 0.0)`. |
| **C** | Clear user-placed dynamic obstacles and restore default arena. |
| **P** | Capture a high-resolution screenshot to `screenshots/`. |

---

## 5. Project Directory Structure

```
TurtleBot3_ESE/
├── pixi.toml                   # Isolated package manifest (Humble + Pygame + NumPy)
├── pixi.lock                   # Deterministic dependency lockfile
├── requirements.md             # Functional and non-functional requirements
├── README.md                   # Project overview and execution guide
├── config/
│   ├── robot_config.yaml       # Physical & sensor specifications
│   └── world_config.yaml       # Indoor arena, walls & obstacle definitions
├── docs/
│   ├── architecture.md         # Software architecture & node communication
│   ├── cse_project_guide.md    # Complete CSE beginner-friendly project guide
│   ├── installation.md         # Windows 11 setup & dependency documentation
│   ├── project_report.md       # Complete academic laboratory project report
│   ├── viva_questions.md       # 40+ exhaustive viva voce Q&A with CS bridges
│   └── ppt_outline.md          # Slide-by-slide presentation outline
├── robot/
│   ├── robot_model/
│   │   ├── turtlebot3_burger.py# Forward/inverse kinematics & limits
│   │   └── turtlebot3_waffle.py# Waffle Pi platform specifications
│   ├── sensors/
│   │   └── lidar.py            # 360-degree LiDAR raycasting & LaserScan
│   └── controllers/
│       └── diff_drive.py       # Waypoint tracking & PID steering controller
├── simulation/
│   ├── worlds/
│   │   └── world.py            # Arena geometry & occupancy grid rasterizer
│   ├── obstacles/
│   │   └── obstacle.py         # Circle, Box, and Dynamic moving obstacles
│   └── visualizer.py           # 60 FPS Pygame renderer with telemetry HUD
├── navigation/
│   ├── a_star.py               # Optimal global A* planner & smoothing
│   ├── obstacle_avoidance.py   # Reactive APF & emergency collision stop
│   ├── localization.py         # Odometry dead reckoning & TF transforms
│   └── path_planning.py        # Autonomous state machine & coordinator
├── scripts/
│   ├── main.py                 # Master orchestrator & ROS 2 bridge
│   ├── auto_nav.py             # Mecharithm Lesson 9 programmatic mission
│   ├── teleoperation.py        # Terminal keyboard teleoperation
│   └── record_demonstration.py # Automated screenshot capture
├── tests/
│   ├── test_kinematics.py      # Unit tests for robot movement & speed limits
│   ├── test_lidar.py           # Unit tests for raycasting & range detection
│   ├── test_a_star.py          # Unit tests for path optimality & detours
│   ├── test_obstacle_avoidance.py # Unit tests for APF & emergency stop
│   ├── test_navigation_integration.py # Full closed-loop integration test
│   └── run_all_tests.py        # Test runner & verification reporter
└── screenshots/                # Demonstration visual artifacts
    ├── 01_initial_world_and_path.png
    ├── 02_autonomous_navigation_in_progress.png
    ├── 03_lidar_sensing_and_obstacle_avoidance.png
    └── 04_goal_reached_success.png
```

---

## 6. Academic ESE Deliverables

All documentation files required for the laboratory examination are located in the `docs/` folder:
- [Architecture & Node Communication](file:///c:/dev/TurtleBot3_ESE/docs/architecture.md)
- [Installation & Setup Documentation](file:///c:/dev/TurtleBot3_ESE/docs/installation.md)
- [Complete Academic Project Report](file:///c:/dev/TurtleBot3_ESE/docs/project_report.md)
- [40+ Viva Voce Questions & Answers](file:///c:/dev/TurtleBot3_ESE/docs/viva_questions.md)
- [PowerPoint Presentation Outline](file:///c:/dev/TurtleBot3_ESE/docs/ppt_outline.md)
