"""
Indoor Simulation World & Occupancy Grid Generator
Robotics Laboratory ESE

This module manages the environment geometry, wall line segments, obstacles,
and rasterization into discrete 2D occupancy grid maps for path planning.
"""

import math
import os
import yaml
import numpy as np
from typing import List, Dict, Any, Tuple, Optional
from simulation.obstacles.obstacle import CircleObstacle, BoxObstacle, DynamicObstacle


class IndoorWorld:
    """Manages world layout, obstacles, collision checking, and occupancy grids."""

    def __init__(self, config_path: Optional[str] = None):
        # Default arena dimensions: 10m x 8m
        self.width = 10.0
        self.height = 8.0
        self.resolution = 0.05  # 5 cm per cell
        self.name = "Indoor_Arena"

        self.boundary_walls: List[List[float]] = []
        self.interior_walls: List[List[float]] = []
        self.all_walls: List[List[float]] = []

        self.obstacles: List[Any] = []
        self.dynamic_obstacles: List[DynamicObstacle] = []

        self.start_pose = (1.2, 1.5, 0.0)
        self.goal_pose = (8.5, 4.8, math.radians(90.0))
        self.patrol_points: List[Tuple[float, float, float]] = []

        if config_path and os.path.exists(config_path):
            self.load_from_yaml(config_path)
        else:
            self.create_default_world()

    def create_default_world(self):
        """Creates clean arena layout with boundary walls and no initial obstacles."""
        self.boundary_walls = [
            [0.0, 0.0, 10.0, 0.0],
            [10.0, 0.0, 10.0, 8.0],
            [10.0, 8.0, 0.0, 8.0],
            [0.0, 8.0, 0.0, 0.0]
        ]
        self.interior_walls = []
        self.all_walls = list(self.boundary_walls)
        self.obstacles = []
        self.dynamic_obstacles = []
        self.start_pose = (1.2, 1.5, 0.0)
        self.goal_pose = (8.5, 4.8, math.radians(90.0))

    def load_preset_lab_obstacles(self):
        """Populates standard laboratory obstacles and dividers on demand (e.g. key 'O')."""
        self.interior_walls = [
            [4.5, 0.0, 4.5, 2.6],
            [4.5, 4.2, 4.5, 8.0],
            [0.0, 4.5, 2.8, 4.5],
            [7.0, 3.5, 10.0, 3.5],
            [7.0, 3.5, 7.0, 6.2]
        ]
        self.all_walls = self.boundary_walls + self.interior_walls
        self.obstacles = [
            CircleObstacle(2.2, 2.2, radius=0.35, label="Lab Pillar A"),
            CircleObstacle(2.2, 6.2, radius=0.35, label="Lab Pillar B"),
            BoxObstacle(8.2, 1.8, width=1.2, height=0.8, label="Workstation Table"),
            BoxObstacle(8.2, 6.0, width=1.0, height=1.0, label="Storage Cabinet"),
            CircleObstacle(5.8, 4.8, radius=0.30, label="Hallway Column")
        ]
        self.dynamic_obstacles = [
            DynamicObstacle(4.5, 3.4, radius=0.25, patrol_x2=6.2, patrol_y2=3.4, speed=0.15, label="Patrolling Cart")
        ]

    def load_from_yaml(self, filepath: str):
        """Loads world specifications from YAML file."""
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)

        world_info = data.get("world", {})
        self.name = world_info.get("name", "Indoor_Arena")
        self.width = float(world_info.get("width", 10.0))
        self.height = float(world_info.get("height", 8.0))
        self.resolution = float(world_info.get("grid_resolution", 0.05))

        self.boundary_walls = data.get("boundary_walls", [])
        self.interior_walls = data.get("interior_walls", [])
        self.all_walls = self.boundary_walls + self.interior_walls

        self.obstacles = []
        for obs in data.get("obstacles", []):
            obs_type = obs.get("type", "circle")
            if obs_type == "circle":
                self.obstacles.append(CircleObstacle(
                    obs["x"], obs["y"], radius=obs.get("radius", 0.35), label=obs.get("label", "Pillar")
                ))
            elif obs_type == "box":
                self.obstacles.append(BoxObstacle(
                    obs["x"], obs["y"], width=obs.get("width", 1.0), height=obs.get("height", 1.0), label=obs.get("label", "Box")
                ))

        waypoints = data.get("waypoints", {})
        if "start_position" in waypoints:
            sp = waypoints["start_position"]
            self.start_pose = (float(sp[0]), float(sp[1]), math.radians(float(sp[2])))
        if "goal_position" in waypoints:
            gp = waypoints["goal_position"]
            self.goal_pose = (float(gp[0]), float(gp[1]), math.radians(float(gp[2])))

    def add_dynamic_obstacle(self, x: float, y: float, radius: float = 0.30, label: str = "User Dynamic Obstacle"):
        """Interactively adds a new obstacle (e.g. via mouse click during simulation)."""
        self.obstacles.append(CircleObstacle(x, y, radius=radius, label=label))

    def update(self, dt: float):
        """Updates positions of moving obstacles."""
        for dyn in self.dynamic_obstacles:
            dyn.update(dt)

    def get_all_obstacle_dicts(self) -> List[Dict[str, Any]]:
        """Returns union of static and dynamic obstacle definitions."""
        obs_list = [obs.to_dict() for obs in self.obstacles]
        for dyn in self.dynamic_obstacles:
            obs_list.append(dyn.to_dict())
        return obs_list

    def is_in_collision(self, px: float, py: float, safety_radius: float = 0.15) -> bool:
        """
        Tests if point (px, py) is in collision with any wall or obstacle
        taking into account robot safety radius.
        """
        # Outer boundary check
        if px - safety_radius < 0.0 or px + safety_radius > self.width:
            return True
        if py - safety_radius < 0.0 or py + safety_radius > self.height:
            return True

        # Wall line segments collision check (point-to-segment distance)
        for x1, y1, x2, y2 in self.all_walls:
            dist = self.point_to_segment_distance(px, py, x1, y1, x2, y2)
            if dist < safety_radius:
                return True

        # Obstacles collision check
        for obs in self.obstacles:
            if obs.contains_point(px, py, margin=safety_radius):
                return True

        for dyn in self.dynamic_obstacles:
            if dyn.contains_point(px, py, margin=safety_radius):
                return True

        return False

    @staticmethod
    def point_to_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculates minimum Euclidean distance from point (px, py) to line segment (x1, y1)-(x2, y2)."""
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-9:
            return math.hypot(px - x1, py - y1)

        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / length_sq))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    def generate_occupancy_grid(self, inflation_radius: float = 0.20) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Rasterizes the continuous world geometry into a discrete 2D occupancy grid.
        Returns:
            grid: 2D numpy array [height_cells, width_cells] where:
                  0 = Free space
                  100 = Occupied / Obstacle / Wall
            metadata: dict containing resolution, width, height, origin
        """
        cols = int(math.ceil(self.width / self.resolution))
        rows = int(math.ceil(self.height / self.resolution))

        grid = np.zeros((rows, cols), dtype=np.int8)

        # Iterate over cells and test occupancy against walls & obstacles
        for r in range(rows):
            cy = (r + 0.5) * self.resolution
            for c in range(cols):
                cx = (c + 0.5) * self.resolution
                if self.is_in_collision(cx, cy, safety_radius=inflation_radius):
                    grid[r, c] = 100

        metadata = {
            "resolution": self.resolution,
            "width": cols,
            "height": rows,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "inflation_radius": inflation_radius
        }
        return grid, metadata
