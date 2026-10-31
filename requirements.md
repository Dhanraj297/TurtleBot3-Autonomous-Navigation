# TurtleBot3 Autonomous Navigation: Requirements Specification

**Target Platform:** Windows 11 Native (Lenovo LOQ / RTX 3050 / Intel Core i5 13th Gen)  
**Project:** TurtleBot3 Autonomous Navigation Using ROS 2 (Mecharithm Lesson 9 Implementation)  
**Coursework:** Robotics Laboratory End-Semester Examination (ESE)  

---

## 1. Functional Requirements (FR)

- **FR-1 [Differential-Drive Kinematics]:** The system must accurately model TurtleBot3 Burger differential drive mechanics ($r=0.033$ m, $L=0.160$ m) with forward and inverse kinematic solvers and acceleration saturation ($a_{lin} \le 0.5$ m/s², $a_{ang} \le 2.0$ rad/s²).
- **FR-2 [LiDAR Environmental Sensing]:** The system must simulate a 360-degree planar rangefinder (LDS-01) with 360 discrete beams, 1-degree resolution, min range $0.12$ m, max range $3.50$ m, and realistic Gaussian measurement noise.
- **FR-3 [Occupancy Grid Mapping]:** The system must rasterize continuous 2D geometry into discrete cell grids ($0.05$ m/cell) with inflation radii to provide collision-free clearance around obstacles.
- **FR-4 [Global Path Planning]:** The system must implement the A* algorithm over 8-connected grid nodes using an admissible Euclidean distance heuristic ($f(n) = g(n) + h(n)$) and Line-of-Sight path smoothing.
- **FR-5 [Local Obstacle Avoidance]:** The system must implement a real-time reactive potential field / sector gating controller that detects dynamic obstacles in front, left, and right cones and applies repulsive torques or emergency braking ($d \le 0.18$ m).
- **FR-6 [Closed-Loop Trajectory Tracking]:** The system must follow planned waypoints using a Pure Pursuit / PID steering controller that modulates linear speed based on heading alignment error.
- **FR-7 [Programmatic Autonomous Navigation]:** The system must replicate Mecharithm Lesson 9 by programmatically dispatching goal poses (`/goal_pose`), tracking progress via odometry (`/odom`), and terminating upon arrival without requiring manual RViz GUI interactions.
- **FR-8 [Interactive Visual Simulation]:** The system must render an interactive 60 FPS graphical interface displaying the robot, LiDAR beams, static walls, dynamic obstacles, planned path, breadcrumb trail, and a live telemetry HUD.
- **FR-9 [Native ROS 2 Humble Integration]:** The system must utilize genuine ROS 2 nodes, topics, publishers, and subscribers (`/scan`, `/odom`, `/cmd_vel`, `/goal_pose`) on native Windows 11 without WSL2 or virtual machines.

---

## 2. Non-Functional Requirements (NFR)

- **NFR-1 [Native Windows 11 Compatibility]:** Must run 100% natively on Windows 11 Home 64-bit without requiring Ubuntu, dual-boot, VirtualBox, or WSL2.
- **NFR-2 [Disk Footprint & Resource Safety]:** Total project footprint must not exceed 10 GB (actual: ~7.5 GB), leaving ample headroom on drive C: (77.7 GB free).
- **NFR-3 [Real-Time Performance]:** Visualizer and simulation loop must execute at a sustained $\ge 60$ frames per second with minimal CPU overhead.
- **NFR-4 [Zero Host Pollution]:** All package dependencies must remain isolated inside `c:\dev\TurtleBot3_ESE\.pixi` using Pixi, leaving Windows system files and system Python untouched.
- **NFR-5 [Academic Rigor & Transparency]:** No custom simulation shall be falsely represented as genuine Gazebo; differences must be explicitly documented and bridged through genuine ROS 2 message interfaces.

---

## 3. Software Dependencies

| Package | Version | Source Channel | Purpose |
| :--- | :--- | :--- | :--- |
| `python` | `3.12.*` | `conda-forge` | Core programming language runtime |
| `pixi` | `0.83.0` | Host toolchain | Isolated package and environment manager |
| `ros-humble-rclpy` | `3.3.21+` | `robostack-humble` | ROS 2 Python client library |
| `ros-humble-geometry-msgs` | `4.9.1+` | `robostack-humble` | Standard ROS 2 Twist, PoseStamped messages |
| `ros-humble-sensor-msgs` | `4.9.1+` | `robostack-humble` | Standard ROS 2 LaserScan message |
| `ros-humble-nav-msgs` | `4.9.1+` | `robostack-humble` | Standard ROS 2 Odometry, OccupancyGrid messages |
| `pygame` | `2.6.1+` | `conda-forge` | 60 FPS 2D visual simulation window & HUD |
| `numpy` | `1.26.4+` | `conda-forge` | Matrix calculations, trigonometry & grid arrays |
| `pyyaml` | `6.0.3+` | `conda-forge` | Configuration parser |
| `scipy` | `1.15.2+` | `conda-forge` | Scientific computations |
