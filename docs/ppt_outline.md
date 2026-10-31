# PowerPoint Presentation Outline: TurtleBot3 Autonomous Navigation

**Project Title:** TurtleBot3 Autonomous Navigation Using ROS 2 on Native Windows 11  
**Coursework:** Robotics Laboratory End-Semester Examination (ESE)  
**Target Defense Duration:** 10–12 Minutes  

---

## Slide 1: Title Slide
- **Title:** TurtleBot3 Autonomous Mobile Robot Navigation Using ROS 2
- **Subtitle:** Implementation of Mecharithm Lesson 9 on Native Windows 11
- **Presenter Name:** [Your Name / Roll Number]
- **Department:** Department of Computer Science & Engineering
- **Institution:** [Your College / University Name]
- **Date:** November 2026

---

## Slide 2: Project Motivation & Objectives
- **The Challenge:**
  - Traditional ROS coursework mandates Linux (Ubuntu / VMs / dual-boot), creating setup friction.
  - Introductory labs rely on manual GUI clicks in RViz ("2D Nav Goal"), failing to teach programmatic autonomy.
- **Our Objectives:**
  - Build a 100% native Windows 11 autonomous navigation system.
  - Implement genuine ROS 2 Humble node communication (`rclpy`).
  - Programmatic multi-point autonomous mission execution (`auto_nav`).
  - First-principles mathematical formulation of kinematics, LiDAR, A*, and obstacle avoidance.

---

## Slide 3: Tutorial Analysis — Mecharithm Lesson 9
- **Reference:** *Autonomous Navigation of Turtlebot Using ROS* (Video `lW-c88vxLRE`).
- **Core Insights:**
  - Transition from manual GUI navigation to custom Python code.
  - RViz is strictly a visualizer (`librviz`), not a controller or physics simulator.
  - Programmatic goal dispatch via `/goal_pose` (`geometry_msgs/PoseStamped`).
  - Closed-loop state monitoring via `/odom` and `/scan`.

---

## Slide 4: System Architecture (Dual-Layer Design)
- **Layer 1: ROS 2 Navigation Graph**
  - High-level mission executive (`auto_nav.py`)
  - A* global path planning (`a_star.py`)
  - Reactive obstacle avoidance (`obstacle_avoidance.py`)
  - Closed-loop PID controller (`diff_drive.py`)
- **Layer 2: Interactive Visual Engine**
  - 60 FPS hardware-accelerated Pygame simulation window.
  - 360-degree LiDAR raycasting with dynamic hit markers.
  - Interactive mouse controls: Click to set goal, click to drop obstacles.
  - Professional dark-mode telemetry HUD.

---

## Slide 5: Robot Kinematics (TurtleBot3 Burger)
- **Model:** Differential Drive mobile robot ($r = 33\text{ mm}$, $L = 160\text{ mm}$).
- **Forward Kinematics:**
  $$v = \frac{r}{2}(\omega_R + \omega_L), \quad \omega = \frac{r}{L}(\omega_R - \omega_L)$$
- **Inverse Kinematics:**
  $$\omega_R = \frac{2v + \omega L}{2r}, \quad \omega_L = \frac{2v - \omega L}{2r}$$
- **Numerical Integration:** Second-Order Runge-Kutta (RK2) to eliminate turning discretization drift.
- **Physical Bounds:** Linear speed $\le 0.22\text{ m/s}$, Angular speed $\le 2.84\text{ rad/s}$.

---

## Slide 6: Perception: 360° LiDAR Sensor Simulation
- **Sensor Model:** LDS-01 Planar Laser Rangefinder.
- **Specifications:** 360 discrete beams, $1^\circ$ resolution, $0.12\text{ m}$ to $3.50\text{ m}$ range.
- **Raycasting Algorithms:**
  - Ray-line segment intersection for indoor walls via 2D vector cross-products.
  - Ray-cylinder intersection for pillars via quadratic root solving.
- **Sensor Realism:** Gaussian noise injection $\mathcal{N}(0, \sigma^2)$ and blind-zone clipping.
- **ROS 2 Output:** Publishes standard `sensor_msgs/msg/LaserScan` messages on `/scan`.

---

## Slide 7: Global Path Planning: A* Algorithm
- **Map Representation:** Discretized 2D Occupancy Grid ($0.05\text{ m}$ cell size) with obstacle inflation buffer ($22\text{ cm}$).
- **Evaluation Function:** $f(n) = g(n) + h(n)$
  - $g(n)$: Accumulated cost with 8-connectivity ($\sqrt{2}$ diagonal cost).
  - $h(n)$: Euclidean distance heuristic (strictly admissible and monotonic).
- **Optimality Guarantee:** Admissibility ($h(n) \le h^*(n)$) proves mathematically that A* discovers the shortest path.
- **Path Smoothing:** Line-of-Sight ray-marching to eliminate unnatural grid zigzag artifacts.

---

## Slide 8: Reactive Local Obstacle Avoidance
- **Why Needed?** Static A* cannot react to dynamic, moving, or unmapped obstacles.
- **Artificial Potential Field (APF):**
  - Attractive force pulls robot toward next waypoint.
  - Repulsive force from LiDAR beams pushes robot away:
    $$\mathbf{F}_{rep, i} \propto \frac{1}{d_i^2} (-\mathbf{\hat{d}}_i)$$
- **Emergency Safety Cone:**
  - Warning zone ($d < 0.50\text{ m}$): Modulate linear speed and apply repulsive steering.
  - Critical zone ($d \le 0.18\text{ m}$): Immediate emergency stop ($v_{cmd} = 0.0\text{ m/s}$).

---

## Slide 9: ROS 2 Node Computation Graph
- **Node 1:** `/turtlebot3_simulation_node`
  - Publishes: `/scan` (`sensor_msgs/LaserScan`), `/odom` (`nav_msgs/Odometry`)
  - Subscribes: `/cmd_vel` (`geometry_msgs/Twist`), `/goal_pose` (`geometry_msgs/PoseStamped`)
- **Node 2:** `/auto_nav_mission_executive`
  - Programmatic multi-point mission dispatcher.
- **Node 3:** `/turtlebot3_teleop_keyboard`
  - Manual keyboard teleoperation override.

---

## Slide 10: Interactive Simulation & HUD
- **Features Demonstrated:**
  - Click-to-Nav: Set goals anywhere on arena floor with a left click.
  - Dynamic Obstacle Placement: Drop obstacles on the fly with a right click.
  - Telemetry HUD: Real-time $(x, y, \theta)$, $v$, $\omega$, distance to goal, and min LiDAR clearance.
  - Keyboard Teleoperation: Drive manually with `W/A/S/D`.

---

## Slide 11: Experimental Results & Verification
- **Automated Test Suite:** 16/16 Unit and Integration Tests Passed (100% Pass Rate).
- **Test Categories:**
  - Robot translation and in-place rotation kinematics.
  - Actuator saturation clamping.
  - LiDAR distance accuracy against calibrated walls and obstacles.
  - A* path optimality and wall detours.
  - Obstacle avoidance and emergency stop activation.
  - Full closed-loop navigation to goal within 8 cm tolerance.
- **Execution Time:** 0.73 seconds.

---

## Slide 12: Visual Demonstration Evidence
- Show the 4 captured screenshots:
  1. `01_initial_world_and_path.png`: Arena layout & A* global path.
  2. `02_autonomous_navigation_in_progress.png`: Live laser beams & breadcrumb trail.
  3. `03_lidar_sensing_and_obstacle_avoidance.png`: Reactive evasive steering.
  4. `04_goal_reached_success.png`: Final arrival, heading alignment & stop.

---

## Slide 13: Key Takeaways & CS Bridges
- **Robotics $\longleftrightarrow$ Computer Science:**
  - Operating Systems: Multi-threading, IPC, event-driven executors.
  - Data Structures & Algorithms: Min-heaps, A* graph search, spatial grids.
  - Networking: DDS peer-to-peer UDP discovery, Pub/Sub architecture.
- **Key Achievement:** A complete, academically rigorous, fully tested autonomous robotics project running natively on Windows 11.

---

## Slide 14: Conclusion & Thank You
- **Summary:** Successfully implemented TurtleBot3 autonomous navigation matching Mecharithm Lesson 9.
- **Questions & Answers:** Open for faculty evaluation and viva voce questions.
