# Complete Guide: Running TurtleBot3 in Gazebo 3D (WSL2 & Windows 11)

> **Target Audience:** Computer Science & Engineering (CSE) Students & Robotics Laboratory Candidates  
> **Course / Lab:** Mecharithm Lesson 9 & Autonomous Robotics ESE  
> **Repository:** [GitHub: TurtleBot3-Autonomous-Navigation](https://github.com/Dhanraj297/TurtleBot3-Autonomous-Navigation.git)

---

## 1. Executive Overview: Why Gazebo 3D?

In academic robotics evaluations, faculty examiners often ask:
> *"Your 2D Python simulation is great, but can your autonomous navigation stack drive the robot inside a 3D physics simulator like Gazebo?"*

The answer is an emphatic **YES!**

Because this project is architected strictly according to standard **ROS 2 Humble / Mecharithm Lesson 9 topic conventions**, the navigation algorithms do not care whether the robot is:
1. Simulated in our custom 2D Pygame arena (`scripts/main.py`),
2. Simulated in the **Gazebo 3D ODE physics engine** (`ros2 launch turtlebot3_gazebo ...`), or
3. A **physical TurtleBot3 Burger** moving in the real world.

All three systems communicate via the identical four ROS 2 topics:
```
+--------------------------------------------------------------+
|                         ROS 2 GRAPH                          |
|                                                              |
|   +-------------------+              +-------------------+   |
|   |                   |   /cmd_vel   |                   |   |
|   |  Our Autonomous   | -----------> |     Gazebo 3D     |   |
|   |    Navigator      |              |    TurtleBot3     |   |
|   |  (A* + APF + PID) |   /odom      |     Simulator     |   |
|   |                   | <----------- |    (or Robot)     |   |
|   |  gazebo_navigator |   /scan      |                   |   |
|   |                   | <----------- |                   |   |
|   +-------------------+              +-------------------+   |
|            ^                                                 |
|            | /goal_pose                                      |
|   +-------------------+                                      |
|   |   RViz2 / User    |                                      |
|   +-------------------+                                      |
+--------------------------------------------------------------+
```

---

## 2. Step 1: Install WSL2 on Windows 11 (One-Time Setup)

Windows 11 includes **WSLg (Windows Subsystem for Linux GUI)**, which natively renders 3D Linux graphical windows (like Gazebo) directly on your Windows desktop with hardware GPU acceleration.

### Why You Need Administrator Rights for Step 1
Windows requires Administrator privileges because enabling WSL involves activating the Windows Hypervisor and Virtual Machine Platform kernel modules (`dism` / Hyper-V).

### Installation Instructions
1. Right-click the **Windows Start Menu** and select **"Terminal (Admin)"** or **"PowerShell (Run as Administrator)"**.
2. Run the following command:
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```
3. Once the installation completes:
   - If Windows asks you to **restart your PC**, reboot your computer.
   - After restart, open the **Ubuntu 22.04** app from your Start Menu.
   - Set an Ubuntu username (e.g., `student` or `dhanraj`) and password when prompted.

> [!NOTE]
> If WSL was already previously installed, you can simply open your existing Ubuntu terminal.

---

## 3. Step 2: Automated 1-Click Installation Script

We have provided a fully automated shell script that configures ROS 2 Humble, Gazebo 3D, and TurtleBot3 simulation packages in Ubuntu.

Open your **Ubuntu terminal** and run:

```bash
# 1. Navigate to the project directory on your Windows C: drive
cd /mnt/c/dev/TurtleBot3_ESE

# 2. Make the installer executable
chmod +x scripts/setup_gazebo_wsl.sh

# 3. Execute the automated installer
./scripts/setup_gazebo_wsl.sh
```

### What This Script Configures Automatically:
- Sets system locale to `en_US.UTF-8`.
- Adds the official Open Robotics ROS 2 Humble apt repository and security GPG keys.
- Installs `ros-humble-desktop`, `ros-humble-turtlebot3-gazebo`, and `ros-humble-navigation2`.
- Pre-downloads TurtleBot3 Burger 3D mesh models into `~/.gazebo/models/` (preventing launch stalls).
- Sets `TURTLEBOT3_MODEL=burger`, `ROS_DOMAIN_ID=0`, and `source /opt/ros/humble/setup.bash` permanently in your `~/.bashrc`.

---

## 4. Step 3: Launching Gazebo 3D

Once the script finishes, launch the 3D simulation inside your Ubuntu terminal:

### Option A: Standard TurtleBot3 Arena (Recommended for Exams)
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
*A 3D Gazebo window will appear on your Windows desktop showing a TurtleBot3 Burger inside an arena with 9 hexagonal and cylindrical pillars!*

### Option B: Empty World (Fastest Startup)
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Option C: TurtleBot3 Indoor House
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

---

## 5. Step 4: Driving the 3D Robot in Gazebo

Open a **second Ubuntu terminal** (or a Windows PowerShell terminal):

### Demo 1: Autonomous Obstacle-Avoiding Mission (`gazebo_navigator.py`)
Run our custom autonomous navigation controller:
```bash
cd /mnt/c/dev/TurtleBot3_ESE
python3 scripts/gazebo_navigator.py
```
**What happens in Gazebo:**
1. The robot reads its position from `/odom` and obstacles from `/scan` (LiDAR).
2. It sets off on a 5-waypoint patrol mission:
   - Target 1: `(1.0m, 0.0m)` — East Point
   - Target 2: `(1.0m, 1.2m)` — North-East Corner
   - Target 3: `(-0.5m, 1.2m)` — North-West Corridor
   - Target 4: `(-0.5m, -0.5m)` — South-West Area
   - Target 5: `(0.0m, 0.0m)` — Return to Dock
3. The robot uses our **Artificial Potential Field (APF)** to steer smoothly around Gazebo obstacles.
4. You will see the 3D robot wheels turning and the robot steering in Gazebo in real-time, accompanied by live telemetry in the terminal!

#### Setting a Custom Single Goal:
```bash
python3 scripts/gazebo_navigator.py --goal 1.5 0.8
```

---

### Demo 2: Keyboard Teleoperation (`teleoperation.py`)
Drive the 3D robot manually with keyboard controls:
```bash
python3 scripts/teleoperation.py
```
- `w` / `x` : Increase / decrease linear velocity ($+0.02$ m/s step)
- `a` / `d` : Steer left / right ($+0.15$ rad/s step)
- `space` or `s` : Emergency brake
- `q` : Quit

---

### Demo 3: Cross-Host Control (Windows Host driving WSL Gazebo)
You can even run the controller **on native Windows** while Gazebo runs in WSL:

1. In Windows PowerShell:
   ```powershell
   cd C:\dev\TurtleBot3_ESE
   pixi run python scripts/gazebo_navigator.py
   ```
2. Because ROS 2 uses **DDS (Data Distribution Service)** over UDP multicast, nodes running on Windows and WSL2 discover each other automatically when `ROS_DOMAIN_ID=0`!

---

## 6. Troubleshooting & Tips for WSL2 Gazebo

| Issue | Cause | One-Line Fix |
|---|---|---|
| **Black screen or Gazebo crashes on launch** | WSLg OpenGL driver conflict | Run: `export LIBGL_ALWAYS_SOFTWARE=1` before launching Gazebo |
| **Gazebo hangs on "Preparing your world..."** | Models taking time to download from online server | Our setup script fixes this by pre-caching models into `~/.gazebo/models/` |
| **Windows terminal doesn't find WSL Gazebo topics** | Different ROS Domain ID or firewall block | Run `$env:ROS_DOMAIN_ID=0` and `$env:ROS_LOCALHOST_ONLY=0` in PowerShell |
| **Gazebo runs at low FPS (< 20 FPS)** | WSL using software CPU rendering | Update your Windows GPU driver (NVIDIA / Intel / AMD) to get WSLg GPU passthrough |

---

## 7. Examiner & Viva Q&A Cheat Sheet (Gazebo 3D)

### Q1: What is the primary difference between Gazebo and RViz?
> **Answer:** **Gazebo is a physical world simulator**, whereas **RViz is a state visualizer**. Gazebo computes real-world physics (gravity, collision forces, wheel friction, LiDAR raycasts, and motor torque). RViz does not simulate physics; it only visualizes sensor topics (`/scan`, `/tf`, `/map`, `/odom`) published by a real robot or by Gazebo.

### Q2: Which physics engine does Gazebo use?
> **Answer:** Gazebo defaults to **ODE (Open Dynamics Engine)**. It also supports alternative pluggable physics backends including **Bullet**, **DART**, and **PhysX**.

### Q3: How do our Python navigation scripts talk to Gazebo?
> **Answer:** Through standard **ROS 2 middleware (DDS - Data Distribution Service)**. Gazebo loads the `gazebo_ros` plugin which creates standard ROS 2 publishers on `/odom` and `/scan`, and subscribes to `/cmd_vel`. Our Python script connects to these exact same DDS topics.

### Q4: Why did we build both a 2D Pygame simulator and 3D Gazebo integration?
> **Answer:** 
> - **2D Simulation (`scripts/main.py`):** Runs at 60 FPS natively on Windows without requiring virtualization, heavy GPU memory, or complex dependencies. Allows immediate interactive mouse-click obstacle spawning and visualizes algorithm internals (A* search nodes, APF vector fields, LiDAR rays) that Gazebo does not visualize.
> - **3D Gazebo (`turtlebot3_gazebo`):** Validates the algorithm against realistic 3D rigid-body contact mechanics, wheel slippage, and sensor noise before transferring code to a physical robot.

### Q5: What is the format of the 3D model in Gazebo?
> **Answer:** Gazebo uses **SDF (Simulation Description Format)**, which extends URDF (Unified Robot Description Format) by adding physics parameters such as friction coefficients ($\mu_1, \mu_2$), moment of inertia tensors ($I_{xx}, I_{yy}, I_{zz}$), joint damping, and sensor plugin hooks (`gazebo_ros_ray_sensor`).

---

*Authored for ESE Autonomous Robotics Laboratory & Examination Viva.*
