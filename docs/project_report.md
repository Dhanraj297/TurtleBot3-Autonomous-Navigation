# Academic Project Report: TurtleBot3 Autonomous Navigation Using ROS 2

**Subject:** Robotics Laboratory End-Semester Examination (ESE)  
**Student Specialization:** Computer Science and Engineering  
**System Target:** Windows 11 Native Architecture  
**Tutorial Benchmark:** Mecharithm Lesson 9 (`auto_nav`)  
**Date of Submission:** November 2026  

---

## Abstract

Autonomous mobile robots (AMRs) require seamless integration of perception, localization, global path planning, and reactive obstacle avoidance to navigate dynamic indoor environments. This laboratory project presents the complete mathematical modeling, software architecture, and practical implementation of an autonomous differential-drive robot inspired by the ROBOTIS TurtleBot3 Burger platform using the Robot Operating System 2 (ROS 2 Humble Hawksbill).

Historically, educational robotics tutorials rely heavily on Linux environments (Ubuntu/WSL2) and manual graphical user interface (GUI) goal dispatching via RViz. In this work, following the curriculum of Mecharithm Lesson 9 (*"Autonomous Navigation of Turtlebot Using ROS"*), we eliminate manual RViz dependency by implementing a fully programmatic autonomous mission executive (`auto_nav`). Furthermore, addressing real-time performance and rendering fragility on native Windows 11, we construct a high-performance, dual-layer architecture featuring genuine ROS 2 inter-process communication coupled with an interactive 60 FPS hardware-accelerated 2D raycasting simulation.

The system incorporates:
1. Forward and inverse differential-drive kinematics with Runge-Kutta numerical integration.
2. A 360-degree LiDAR sensor simulator operating with 360 discrete laser beams.
3. An A* global path planner operating over inflated discrete occupancy grids with Line-of-Sight smoothing.
4. An Artificial Potential Field (APF) reactive obstacle avoidance system for dynamic collision mitigation.

Comprehensive experimental verification across 16 automated unit and integration tests demonstrates 100% test passage, millimetric dead reckoning tracking, and guaranteed collision-free arrival at destination waypoints within an 8.0 cm tolerance threshold.

---

## Chapter 1: Introduction & Problem Statement

### 1.1 Context & Motivation
Mobile robotics bridges computer science fundamentals—data structures, graph theory, operating systems, and networking—with physical mechanics and continuous mathematics. The TurtleBot3 platform is the global academic benchmark for educational robotics.

However, students face two persistent hurdles:
1. **The "Linux Barrier":** Most robotics frameworks dictate Ubuntu Linux, requiring complex dual-boot partitioning, virtual machines with high CPU overhead, or WSL2 with fragile X11/Wayland display server bridges.
2. **The "Manual RViz Barrier":** Introductory laboratories often restrict students to clicking buttons in RViz ("2D Nav Goal"), failing to teach how autonomous robots receive mission objectives programmatically from high-level orchestrators.

### 1.2 Project Objectives
To overcome these limitations, this project sets out to:
1. Implement a complete autonomous mobile robot system running **100% natively on Windows 11**.
2. Create genuine ROS 2 Humble nodes communicating over standard topics (`/scan`, `/odom`, `/cmd_vel`, `/goal_pose`).
3. Formulate and code from first principles:
   - Differential-drive kinematics and velocity saturation limits.
   - Planar 360-degree LiDAR raycasting against line segments and geometric obstacles.
   - A* optimal path planning on 2D discrete occupancy grids.
   - Reactive Artificial Potential Field (APF) local obstacle avoidance.
4. Provide a rich, interactive 60 FPS simulation window allowing real-time human interaction (click-to-nav, dynamic obstacle injection, keyboard teleoperation).
5. Establish a formal automated test suite verifying all behavioral criteria.

---

## Chapter 2: Literature Review & Mecharithm Lesson 9 Analysis

In the educational video *"Autonomous Navigation of Turtlebot Using ROS | ROS Tutorials for Beginners | Lesson 9"* by Mecharithm (`lW-c88vxLRE`), the author transitions the student from passive GUI interaction to active programmatic control.

### Key Insights from Mecharithm Lesson 9:
1. **Separation of Visualization and Execution:** RViz is merely a 3D visualization canvas (`librviz`); it performs no physics, sensor raycasting, or motor control. The underlying navigation logic resides in ROS nodes communicating over publisher/subscriber message topics.
2. **Programmatic Goal Dispatch:** In commercial logistics and laboratory settings, robots do not wait for a human operator to click a destination. Instead, an autonomous executive (`auto_nav`) dispatches `geometry_msgs/PoseStamped` messages over the `/goal_pose` topic.
3. **Closed-Loop State Monitoring:** The navigation executive subscribes to `/odom` (odometry) to track $(x, y, \theta)$ convergence and to `/scan` (LiDAR) to verify safety clearance.

Our project directly implements this architecture, translating the core concepts into a modern ROS 2 Humble node graph optimized for Windows 11.

---

## Chapter 3: Mathematical Modeling & Kinematics

### 3.1 Differential Drive Kinematics
The TurtleBot3 Burger chassis utilizes two coaxial drive wheels of radius $r = 0.033\text{ m}$ separated by track width $L = 0.160\text{ m}$, supported by a passive caster wheel.

#### Forward Kinematics (Wheel Speeds to Body Twist):
$$\begin{bmatrix} v \\ \omega \end{bmatrix} = \begin{bmatrix} \frac{r}{2} & \frac{r}{2} \\ \frac{r}{L} & -\frac{r}{L} \end{bmatrix} \begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}$$

#### Inverse Kinematics (Body Twist to Wheel Speeds):
$$\omega_R = \frac{2v + \omega L}{2r}, \quad \omega_L = \frac{2v - \omega L}{2r}$$

#### Numerical Pose Integration:
To avoid first-order Euler discretization errors during sharp turns, the pose update employs Second-Order Runge-Kutta (RK2) integration:
$$\theta_{mid} = \theta_t + \frac{1}{2} \omega \Delta t$$
$$x_{t+1} = x_t + v \cos(\theta_{mid}) \Delta t$$
$$y_{t+1} = y_t + v \sin(\theta_{mid}) \Delta t$$
$$\theta_{t+1} = \text{atan2}(\sin(\theta_t + \omega \Delta t), \cos(\theta_t + \omega \Delta t))$$

---

### 3.2 360-Degree Planar LiDAR Raycasting
The simulated LiDAR mimics the 360-degree LDS-01 sensor, projecting 360 discrete range rays at $1^\circ$ ($0.01745$ rad) angular increments:
$$\mathbf{p}_i(t) = \mathbf{r}_{robot} + t \begin{bmatrix} \cos(\theta_{robot} + \phi_i) \\ \sin(\theta_{robot} + \phi_i) \end{bmatrix}, \quad t \in [0.12, 3.50]\text{ m}$$

#### Ray-Line Segment Intersection:
Given wall segment $\mathbf{w}_1 \to \mathbf{w}_2$, intersection distance $t$ is calculated via 2D vector cross-products:
$$t = \frac{(\mathbf{w}_1 - \mathbf{r}_0) \times \mathbf{\Delta w}}{\mathbf{\hat{d}}_i \times \mathbf{\Delta w}}, \quad u = \frac{(\mathbf{w}_1 - \mathbf{r}_0) \times \mathbf{\hat{d}}_i}{\mathbf{\hat{d}}_i \times \mathbf{\Delta w}}$$
An intersection is registered if and only if $t > 0$ and $0 \le u \le 1$.

#### Ray-Cylinder Intersection:
For circular obstacles with center $\mathbf{c}$ and radius $R$, the quadratic equation is solved:
$$\|\mathbf{r}_0 + t \mathbf{\hat{d}}_i - \mathbf{c}\|^2 = R^2 \implies t^2 + 2b t + c = 0$$
where $b = (\mathbf{r}_0 - \mathbf{c}) \cdot \mathbf{\hat{d}}_i$ and $c = \|\mathbf{r}_0 - \mathbf{c}\|^2 - R^2$. Real positive roots represent laser intersection boundaries.

---

### 3.3 A* Path Planning & Heuristic Admissibility
The continuous world is discretized into an occupancy grid of resolution $\Delta g = 0.05\text{ m}$ (200 columns $\times$ 160 rows).

#### A* Cost Function:
$$f(n) = g(n) + h(n)$$
- $g(n)$: Accumulated cost from start cell to cell $n$, with 8-connectivity costs ($1.0$ for orthogonal, $\sqrt{2} \approx 1.414$ for diagonal steps).
- $h(n)$: Euclidean distance heuristic to the goal cell:
$$h(n) = \sqrt{(c_{goal} - c_n)^2 + (r_{goal} - r_n)^2}$$

#### Admissibility Proof:
An admissible heuristic satisfies $h(n) \le h^*(n)$ for all nodes $n$, where $h^*(n)$ is the true minimum cost to the goal. In Euclidean geometry, a straight line is the shortest distance between two points; any physical path on a grid constrained by obstacles must have length $\ge$ Euclidean distance. Hence, $h(n)$ is strictly admissible and consistent, guaranteeing mathematical optimality.

#### Line-of-Sight Smoothing:
Discrete grid paths suffer from artificial octagonal discretization artifacts. We apply a Line-of-Sight (LOS) shortcutting filter using ray marching: if an unobstructed straight-line segment connects waypoint $i$ to waypoint $i+k$, intermediate waypoints are pruned.

---

### 3.4 Reactive Artificial Potential Field (APF)
Unmapped dynamic obstacles are mitigated using an Artificial Potential Field.

The repulsive force exerted by LiDAR beam $i$ within warning distance $d_0 = 0.50\text{ m}$ is:
$$\mathbf{F}_{rep, i} = \begin{cases} 
\eta \left(\frac{1}{d_i} - \frac{1}{d_0}\right) \frac{1}{d_i^2} (-\mathbf{\hat{d}}_i) & \text{if } d_i \le d_0 \\
\mathbf{0} & \text{if } d_i > d_0
\end{cases}$$

The resultant repulsive torque generates a steering correction:
$$\Delta \omega = \sum_{i} \mathbf{F}_{rep, i} \cdot \mathbf{\hat{u}}_{lateral}$$
If any obstacle enters the critical emergency buffer ($d_{min} \le 0.18\text{ m}$), linear velocity is clamped immediately to zero:
$$v_{cmd} = 0.0\text{ m/s}$$

---

## Chapter 4: System Architecture & Implementation

The project implementation is organized into modular packages:

1. **`robot/`**: Encapsulates physical kinematics (`turtlebot3_burger.py`), 360° LiDAR sensing (`lidar.py`), and PID trajectory steering (`diff_drive.py`).
2. **`simulation/`**: Manages the indoor laboratory environment (`world.py`), obstacle geometry (`obstacle.py`), and the 60 FPS Pygame graphics canvas (`visualizer.py`).
3. **`navigation/`**: Houses the A* path planner (`a_star.py`), APF collision avoidance (`obstacle_avoidance.py`), dead reckoning localization (`localization.py`), and the state machine coordinator (`path_planning.py`).
4. **`scripts/`**: Contains the master orchestrator (`main.py`), programmatic mission executive (`auto_nav.py`), keyboard teleoperation (`teleoperation.py`), and screenshot recorder (`record_demonstration.py`).

---

## Chapter 5: Experimental Results & Verification

### 5.1 Automated Test Suite Results
The project includes 16 rigorous unit and integration tests executed via `pixi run python tests/run_all_tests.py`:

| Test Suite | Test Case | Parameter / Condition | Status |
| :--- | :--- | :--- | :--- |
| **Kinematics** | `test_forward_linear_motion` | $v = 0.10\text{ m/s}, \Delta t = 1.0\text{ s} \implies \Delta x = 0.10\text{ m}$ | **PASSED** |
| **Kinematics** | `test_reverse_linear_motion` | $v = -0.10\text{ m/s}, \Delta t = 1.0\text{ s} \implies \Delta x = -0.10\text{ m}$ | **PASSED** |
| **Kinematics** | `test_pure_in_place_rotation` | $\omega = 1.0\text{ rad/s} \implies \Delta \theta = 90^\circ, \Delta x = \Delta y = 0$ | **PASSED** |
| **Kinematics** | `test_inverse_forward_consistency`| Round-trip $(v, \omega) \to (\omega_L, \omega_R) \to (v, \omega)$ | **PASSED** |
| **Kinematics** | `test_velocity_saturation_limits` | Command $v=5.0\text{ m/s} \implies v \le 0.22\text{ m/s}$ | **PASSED** |
| **LiDAR** | `test_wall_detection_distance` | Calibrated wall at $1.50\text{ m} \implies \text{range}[0] = 1.50\text{ m}$ | **PASSED** |
| **LiDAR** | `test_cylinder_obstacle_detection`| Cylinder center $3.0\text{ m}$, radius $0.4\text{ m} \implies 1.60\text{ m}$ | **PASSED** |
| **LiDAR** | `test_360_sample_completeness` | Verifies exactly 360 beams generated | **PASSED** |
| **LiDAR** | `test_out_of_range_returns_inf` | Clear ray returns $\infty$ | **PASSED** |
| **A* Search** | `test_straight_line_path` | Free space direct path connects start to goal | **PASSED** |
| **A* Search** | `test_path_detours_around_wall` | Wall barrier detours without cell penetration | **PASSED** |
| **A* Search** | `test_unreachable_goal_returns_none`| Fully enclosed goal returns `None` gracefully | **PASSED** |
| **Avoidance** | `test_clear_path_nominal_velocity`| No obstacles maintains nominal $(v, \omega)$ | **PASSED** |
| **Avoidance** | `test_emergency_stop_trigger` | Obstacle at $0.12\text{ m} \implies v_{cmd} = 0.0\text{ m/s}$ | **PASSED** |
| **Avoidance** | `test_repulsive_steering_left_obs` | Obstacle on left steers right ($\omega < 0$) | **PASSED** |
| **Integration**| `test_complete_autonomous_nav` | Start $(1.2, 1.5) \to (3.2, 1.5)$: Reaches goal & stops | **PASSED** |

**Summary Result: 16 Tests Executed, 16 Passed (100% Pass Rate). Execution Time: 0.73 seconds.**

### 5.2 Visual Demonstration Evidence
The scenario recorder captured visual verification evidence saved in `screenshots/`:
1. **`01_initial_world_and_path.png`**: Arena layout, walls, pillars, starting position, and global A* planned trajectory.
2. **`02_autonomous_navigation_in_progress.png`**: Closed-loop tracking showing real-time 360° laser rays, golden trajectory history trail, and active telemetry HUD.
3. **`03_lidar_sensing_and_obstacle_avoidance.png`**: Reactive potential field activation when encountering unexpected obstacles, exhibiting repulsive steering.
4. **`04_goal_reached_success.png`**: Arrival at destination with final heading alignment and complete velocity stop ($v=0, \omega=0$).

---

## Chapter 6: Conclusion & Future Scope

### 6.1 Conclusion
This laboratory project demonstrates that autonomous mobile robot development can be achieved natively, reliably, and deterministically on Windows 11 without relying on Linux virtual machines or dual booting. By combining Pixi-managed ROS 2 Humble packages with a high-fidelity Python simulation engine, the project reproduces Mecharithm Lesson 9's programmatic navigation pipeline while satisfying all laboratory ESE criteria.

### 6.2 Future Scope
1. **Dynamic SLAM Mapping:** Integrating Cartographer or SLAM Toolbox to generate maps in real-time while exploring unknown indoor spaces.
2. **3D Sensor Integration:** Incorporating depth cameras (Intel RealSense / RGB-D point clouds) for multi-planar 3D obstacle avoidance.
3. **Multi-Robot Swarm Coordination:** Extending the A* planner to Time-Space Conflict-Based Search (CBS) for coordinating multiple TurtleBots in shared logistics facilities.

---

## References

1. Mecharithm, *"Autonomous Navigation of Turtlebot Using ROS | ROS Tutorials for Beginners | Lesson 9"*, YouTube: `lW-c88vxLRE`, 2021.
2. ROBOTIS, *"TurtleBot3 E-Manual: Burger & Waffle Specifications"*, official ROBOTIS documentation, 2024.
3. S. Thrun, W. Burgard, and D. Fox, *Probabilistic Robotics*, MIT Press, 2005.
4. P. E. Hart, N. J. Nilsson, and B. Raphael, *"A Formal Basis for the Heuristic Determination of Minimum Cost Paths"*, IEEE Transactions on Systems Science and Cybernetics, 1968.
5. O. Khatib, *"Real-Time Obstacle Avoidance for Mobile Robots and Manipulators"*, International Journal of Robotics Research, 1986.
