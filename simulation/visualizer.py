"""
Interactive 2D Visual Simulation Engine for TurtleBot3
Robotics Laboratory ESE

This module provides a real-time, hardware-accelerated 60 FPS interactive graphical
simulation window using Pygame.

Features:
- Real-time rendering of TurtleBot3 chassis, drive wheels, and LiDAR turret.
- 360-degree laser raycasting visualization with dynamic hit markers.
- Indoor walls, static obstacles, and interactive dynamic obstacles.
- Overlay of A* global planned path and driven trajectory trail.
- Interactive mouse controls:
  * Left Click: Set new Navigation Goal (2D Nav Goal).
  * Right Click: Place a new dynamic obstacle to test avoidance in real time!
- Interactive keyboard controls:
  * T: Toggle Teleoperation mode.
  * A or Space: Start Autonomous Navigation.
  * R: Reset robot to starting position.
  * C: Clear custom obstacles.
  * S: Capture high-resolution screenshot to screenshots/ folder.
- Professional Dark Mode HUD displaying state, velocities, and telemetry.
"""

import math
import os
import pygame
from typing import List, Tuple, Optional, Dict, Any


class SimulationVisualizer:
    """Interactive visual simulation display."""

    def __init__(
        self,
        world_width: float = 10.0,
        world_height: float = 8.0,
        window_width: int = 1200,
        window_height: int = 800,
        headless: bool = False
    ):
        self.world_w = world_width
        self.world_h = world_height
        self.win_w = window_width
        self.win_h = window_height
        self.headless = headless

        # Margins for HUD sidebar
        self.sidebar_w = 320
        self.arena_w = self.win_w - self.sidebar_w
        self.arena_h = self.win_h

        # Coordinate transformation scales
        self.scale_x = self.arena_w / self.world_w
        self.scale_y = self.arena_h / self.world_h
        self.scale = min(self.scale_x, self.scale_y) * 0.95

        # Offset to center arena
        self.offset_x = (self.arena_w - self.world_w * self.scale) / 2.0
        self.offset_y = (self.arena_h - self.world_h * self.scale) / 2.0

        # Color Palette (Dark Mode Aesthetics)
        self.C_BG = (18, 22, 32)            # Deep slate background
        self.C_ARENA_BG = (24, 30, 44)      # Arena floor
        self.C_GRID = (36, 44, 64)          # Grid lines
        self.C_SIDEBAR = (28, 34, 50)       # Telemetry sidebar
        self.C_SIDEBAR_BORDER = (45, 55, 80)
        self.C_WALL = (140, 160, 190)       # Walls
        self.C_OBSTACLE = (220, 80, 80)     # Red obstacle
        self.C_OBSTACLE_BUF = (80, 40, 40)  # Obstacle safety buffer
        self.C_LIDAR_RAY = (0, 220, 255, 45)# Translucent cyan laser beams
        self.C_LIDAR_HIT = (0, 255, 200)    # Laser impact points
        self.C_PATH = (46, 204, 113)        # Planned path (vibrant green)
        self.C_TRAIL = (241, 196, 15)       # Trajectory breadcrumbs (golden yellow)
        self.C_GOAL = (230, 126, 34)        # Goal marker (orange)
        self.C_ROBOT = (52, 73, 94)         # Robot body
        self.C_ROBOT_HEADING = (231, 76, 60)# Heading arrow (red)
        self.C_TEXT_MAIN = (240, 244, 250)
        self.C_TEXT_MUTED = (160, 175, 200)
        self.C_TEXT_ACCENT = (52, 152, 219)

        if not self.headless:
            pygame.init()
            pygame.font.init()
            pygame.display.set_caption("TurtleBot3 Autonomous Navigation — Robotics Laboratory ESE")
            self.screen = pygame.display.set_mode((self.win_w, self.win_h))
            self.clock = pygame.time.Clock()
            self.font_title = pygame.font.SysFont("Segoe UI", 18, bold=True)
            self.font_body = pygame.font.SysFont("Consolas", 14)
            self.font_small = pygame.font.SysFont("Segoe UI", 12)
        else:
            self.screen = None

    def world_to_screen(self, wx: float, wy: float) -> Tuple[int, int]:
        """Maps world coordinates (meters) to screen pixel coordinates."""
        # Note: In world, y=0 is at bottom; in screen, y=0 is at top
        sx = int(self.offset_x + wx * self.scale)
        sy = int(self.win_h - (self.offset_y + wy * self.scale))
        return sx, sy

    def screen_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        """Maps screen pixel coordinates back to continuous world coordinates."""
        wx = (sx - self.offset_x) / self.scale
        wy = (self.win_h - sy - self.offset_y) / self.scale
        return max(0.0, min(self.world_w, wx)), max(0.0, min(self.world_h, wy))

    def render(
        self,
        robot_pose: Tuple[float, float, float],
        robot_chassis_radius: float,
        trajectory_history: List[Tuple[float, float]],
        planned_path: List[Tuple[float, float]],
        goal_pose: Optional[Tuple[float, float, float]],
        walls: List[List[float]],
        obstacles: List[Dict[str, Any]],
        lidar_hit_points: List[Tuple[float, float]],
        telemetry: Dict[str, Any]
    ):
        """Draws one complete frame."""
        if self.headless or self.screen is None:
            return

        self.screen.fill(self.C_BG)

        # 1. Draw Arena Floor & Grid
        arena_rect = pygame.Rect(
            int(self.offset_x),
            int(self.win_h - (self.offset_y + self.world_h * self.scale)),
            int(self.world_w * self.scale),
            int(self.world_h * self.scale)
        )
        pygame.draw.rect(self.screen, self.C_ARENA_BG, arena_rect)
        pygame.draw.rect(self.screen, self.C_WALL, arena_rect, 2)

        # Subtle 1-meter grid lines
        for x in range(int(self.world_w) + 1):
            p1 = self.world_to_screen(x, 0.0)
            p2 = self.world_to_screen(x, self.world_h)
            pygame.draw.line(self.screen, self.C_GRID, p1, p2, 1)

        for y in range(int(self.world_h) + 1):
            p1 = self.world_to_screen(0.0, y)
            p2 = self.world_to_screen(self.world_w, y)
            pygame.draw.line(self.screen, self.C_GRID, p1, p2, 1)

        # 2. Draw Walls
        for x1, y1, x2, y2 in walls:
            sp1 = self.world_to_screen(x1, y1)
            sp2 = self.world_to_screen(x2, y2)
            pygame.draw.line(self.screen, self.C_WALL, sp1, sp2, 3)

        # 3. Draw Obstacles
        for obs in obstacles:
            obs_type = obs.get("type", "circle")
            if obs_type == "circle":
                cx, cy = self.world_to_screen(obs["x"], obs["y"])
                r_pix = max(2, int(obs["radius"] * self.scale))
                # Safety inflation outline
                buf_pix = int((obs["radius"] + 0.15) * self.scale)
                pygame.draw.circle(self.screen, self.C_OBSTACLE_BUF, (cx, cy), buf_pix, 1)
                pygame.draw.circle(self.screen, self.C_OBSTACLE, (cx, cy), r_pix)
            elif obs_type == "box":
                w_pix = int(obs["width"] * self.scale)
                h_pix = int(obs["height"] * self.scale)
                bx, by = self.world_to_screen(obs["x"] - obs["width"] / 2.0, obs["y"] + obs["height"] / 2.0)
                pygame.draw.rect(self.screen, self.C_OBSTACLE, (bx, by, w_pix, h_pix))

        # 4. Draw LiDAR Beams & Impact Points
        rx, ry, rtheta = robot_pose
        s_robot = self.world_to_screen(rx, ry)

        # Draw a subsample of laser rays to ensure crisp 60fps performance
        laser_surface = pygame.Surface((self.win_w, self.win_h), pygame.SRCALPHA)
        for i in range(0, len(lidar_hit_points), 4):  # Every 4th ray
            hx, hy = lidar_hit_points[i]
            s_hit = self.world_to_screen(hx, hy)
            pygame.draw.line(laser_surface, (0, 220, 255, 30), s_robot, s_hit, 1)
            pygame.draw.circle(laser_surface, (0, 255, 200, 180), s_hit, 2)
        self.screen.blit(laser_surface, (0, 0))

        # 5. Draw Driven Trajectory Trail
        if len(trajectory_history) >= 2:
            trail_pts = [self.world_to_screen(tx, ty) for tx, ty in trajectory_history]
            pygame.draw.lines(self.screen, self.C_TRAIL, False, trail_pts, 2)

        # 6. Draw Planned A* Path
        if planned_path and len(planned_path) >= 2:
            path_pts = [self.world_to_screen(px, py) for px, py in planned_path]
            pygame.draw.lines(self.screen, self.C_PATH, False, path_pts, 3)
            for pt in path_pts:
                pygame.draw.circle(self.screen, self.C_PATH, pt, 4)

        # 7. Draw Goal Marker
        if goal_pose:
            gx, gy, gtheta = goal_pose
            sg = self.world_to_screen(gx, gy)
            pygame.draw.circle(self.screen, self.C_GOAL, sg, 12, 2)
            pygame.draw.circle(self.screen, self.C_GOAL, sg, 4)
            # Desired heading arrow
            ghx = sg[0] + int(24 * math.cos(gtheta))
            ghy = sg[1] - int(24 * math.sin(gtheta))
            pygame.draw.line(self.screen, self.C_GOAL, sg, (ghx, ghy), 2)

        # 8. Draw TurtleBot3 Chassis & Wheels
        r_pix = max(6, int(robot_chassis_radius * self.scale))
        pygame.draw.circle(self.screen, self.C_ROBOT, s_robot, r_pix)
        pygame.draw.circle(self.screen, (200, 210, 230), s_robot, r_pix, 2)

        # Top LiDAR Turret
        turret_r = max(3, int(r_pix * 0.45))
        pygame.draw.circle(self.screen, (192, 57, 43), s_robot, turret_r)

        # Forward Heading Arrow
        head_len = r_pix + 12
        head_x = s_robot[0] + int(head_len * math.cos(rtheta))
        head_y = s_robot[1] - int(head_len * math.sin(rtheta))
        pygame.draw.line(self.screen, self.C_ROBOT_HEADING, s_robot, (head_x, head_y), 3)

        # 9. Draw Telemetry Sidebar HUD
        self.render_hud(telemetry)

        pygame.display.flip()
        self.clock.tick(60)

    def render_hud(self, telemetry: Dict[str, Any]):
        """Renders information panel on right sidebar."""
        sidebar_rect = pygame.Rect(self.win_w - self.sidebar_w, 0, self.sidebar_w, self.win_h)
        pygame.draw.rect(self.screen, self.C_SIDEBAR, sidebar_rect)
        pygame.draw.line(self.screen, self.C_SIDEBAR_BORDER, (self.win_w - self.sidebar_w, 0), (self.win_w - self.sidebar_w, self.win_h), 2)

        y = 20
        # Title
        title_surf = self.font_title.render("TURTLEBOT3 TELEMETRY", True, self.C_TEXT_MAIN)
        self.screen.blit(title_surf, (self.win_w - self.sidebar_w + 16, y))
        y += 30

        # Subtitle
        sub_surf = self.font_small.render("Robotics Lab ESE — Mecharithm Lesson 9", True, self.C_TEXT_ACCENT)
        self.screen.blit(sub_surf, (self.win_w - self.sidebar_w + 16, y))
        y += 25

        pygame.draw.line(self.screen, self.C_SIDEBAR_BORDER, (self.win_w - self.sidebar_w + 16, y), (self.win_w - 16, y), 1)
        y += 15

        # Telemetry Items
        metrics = [
            ("Mode:", telemetry.get("mode", "AUTONOMOUS")),
            ("Nav State:", telemetry.get("state", "IDLE")),
            ("Robot Pose (x, y):", f"{telemetry.get('x', 0.0):.2f} m, {telemetry.get('y', 0.0):.2f} m"),
            ("Heading (Theta):", f"{telemetry.get('theta_deg', 0.0):.1f} deg"),
            ("Linear Vel (v):", f"{telemetry.get('v', 0.0):.2f} m/s"),
            ("Angular Vel (w):", f"{telemetry.get('omega', 0.0):.2f} rad/s"),
            ("Goal (x, y):", f"{telemetry.get('goal_x', 0.0):.2f} m, {telemetry.get('goal_y', 0.0):.2f} m" if telemetry.get("has_goal") else "None (Click arena)"),
            ("Distance to Goal:", f"{telemetry.get('dist_to_goal', 0.0):.2f} m" if telemetry.get("has_goal") else "--"),
            ("Min LiDAR Clearance:", f"{telemetry.get('min_lidar_dist', 0.0):.2f} m"),
            ("Path Waypoints:", f"{telemetry.get('waypoints_left', 0)} / {telemetry.get('total_waypoints', 0)}" if telemetry.get("has_goal") else "--"),
            ("Traveled Distance:", f"{telemetry.get('dist_traveled', 0.0):.2f} m")
        ]

        for label, val in metrics:
            lbl_surf = self.font_small.render(label, True, self.C_TEXT_MUTED)
            self.screen.blit(lbl_surf, (self.win_w - self.sidebar_w + 16, y))
            y += 16
            val_surf = self.font_body.render(str(val), True, self.C_TEXT_MAIN)
            self.screen.blit(val_surf, (self.win_w - self.sidebar_w + 24, y))
            y += 22

        y += 10
        pygame.draw.line(self.screen, self.C_SIDEBAR_BORDER, (self.win_w - self.sidebar_w + 16, y), (self.win_w - 16, y), 1)
        y += 15

        # Interactive Controls Guide
        ctrl_title = self.font_title.render("INTERACTIVE CONTROLS", True, self.C_TEXT_MAIN)
        self.screen.blit(ctrl_title, (self.win_w - self.sidebar_w + 16, y))
        y += 28

        controls = [
            ("Left Click", "Set Goal & Start Path"),
            ("Right Click", "Place Dynamic Obstacle"),
            ("O", "Load Lab Obstacles"),
            ("C", "Clear All Obstacles"),
            ("R", "Reset Robot (Idle)"),
            ("T", "Toggle Teleoperation"),
            ("W/A/S/D", "Drive in Teleop"),
            ("S", "Capture Screenshot")
        ]

        for key, desc in controls:
            k_surf = self.font_body.render(f"[{key}]", True, self.C_TEXT_ACCENT)
            self.screen.blit(k_surf, (self.win_w - self.sidebar_w + 16, y))
            d_surf = self.font_small.render(desc, True, self.C_TEXT_MUTED)
            self.screen.blit(d_surf, (self.win_w - self.sidebar_w + 110, y + 2))
            y += 20

    def capture_screenshot(self, filepath: str):
        """Saves current frame to PNG file."""
        if self.screen is not None:
            os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
            pygame.image.save(self.screen, filepath)
            print(f"[Visualizer] Screenshot saved to: {filepath}")

    def close(self):
        """Closes Pygame window."""
        if not self.headless and pygame.get_init():
            pygame.quit()
