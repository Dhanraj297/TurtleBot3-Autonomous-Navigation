# Installation & Configuration Guide (Windows 11 Native)

**Project:** TurtleBot3 Autonomous Navigation Using ROS 2  
**Target Host:** Microsoft Windows 11 (64-bit)  
**Package Manager:** Pixi 0.83.0 (Conda-Forge & RoboStack Channels)  

---

## 1. Prerequisites & System Requirements

Before running the project, ensure your Windows 11 computer has:
1. **Operating System:** Windows 11 Home / Pro (64-bit).
2. **Terminal:** Windows PowerShell or Windows Terminal (Run as normal user; Admin is NOT required).
3. **Storage:** At least 10 GB of available space on drive C: (actual footprint is ~7.5 GB).
4. **Graphics:** Integrated or Dedicated GPU (e.g. Intel Iris / NVIDIA RTX 3050) with standard WDDM graphics drivers.

---

## 2. Environment Setup via Pixi

Pixi creates a 100% reproducible, isolated environment inside `c:\dev\TurtleBot3_ESE\.pixi`. It will not modify your Windows system environment variables or interfere with existing software.

### Step 1: Verify or Install Pixi
If Pixi is already installed on your system (as detected in your Lenovo LOQ):
```powershell
pixi --version
# Output: pixi 0.83.0 (or newer)
```

If installing on a new Windows machine, run:
```powershell
iwr -useb https://pixi.sh/install.ps1 | iex
```

### Step 2: Navigate to Project Workspace
```powershell
cd c:\dev\TurtleBot3_ESE
```

### Step 3: Install All Dependencies
Run the deterministic installation command:
```powershell
pixi install
```
*Note: This automatically downloads Python 3.12, NumPy, Pygame, SciPy, Matplotlib, PyYAML, and the precompiled ROS 2 Humble packages from the RoboStack channel. The entire installation completes in 1–3 minutes depending on network speed.*

### Step 4: Verify Installation
Verify that the ROS 2 and graphics libraries load correctly:
```powershell
pixi run python -c "import rclpy, pygame, numpy; print('>>> ENVIRONMENT VERIFIED SUCCESSFULLY! <<<')"
```

---

## 3. Execution Commands

### Scenario 1: Interactive Simulation Window (Recommended for Viva Demo)
Launches the 60 FPS real-time Pygame simulator, runs the indoor arena, generates the occupancy grid, and initiates A* autonomous navigation to the default waypoint:
```powershell
pixi run python scripts/main.py
```

### Scenario 2: Programmatic Autonomous Navigation (Mecharithm Lesson 9)
In a second terminal, command the robot to execute a multi-waypoint autonomous mission:
```powershell
pixi run python scripts/auto_nav.py
```

### Scenario 3: Manual Keyboard Teleoperation
Take manual control over the robot via terminal keypresses:
```powershell
pixi run python scripts/teleoperation.py
```

### Scenario 4: Headless Automated Test Verification
Runs all 16 unit and integration tests and prints the pass/fail report:
```powershell
pixi run python tests/run_all_tests.py
```

### Scenario 5: Automated Demonstration & Screenshot Capture
Runs automated scenario stages and saves high-resolution PNG captures to `screenshots/`:
```powershell
pixi run python scripts/record_demonstration.py
```

---

## 4. Troubleshooting Windows 11 Edge Cases

### Issue 1: PowerShell Script Execution Policy
If PowerShell displays `Execution of scripts is disabled on this system`:
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### Issue 2: Pygame AVX2 Detection Notice
You may see:
`RuntimeWarning: Your system is avx2 capable but pygame was not built with support for it.`
*Status:* Completely benign warning from the Pygame SDL layer. The visualizer will still run at a silky smooth 60 FPS.

### Issue 3: Windows MAX_PATH (260 Character Limit)
Windows historically limits path lengths to 260 characters. By placing the project in `c:\dev\TurtleBot3_ESE` (a short root path), all nested `.pixi` links remain well within limits.
