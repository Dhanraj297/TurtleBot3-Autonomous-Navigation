"""
A* (A-Star) Optimal Global Path Planning Algorithm
Robotics Laboratory ESE

Theoretical Foundation:
-----------------------
A* is a graph traversal and path-planning algorithm that finds the shortest
collision-free path from a start cell to a goal cell on a discrete grid.

Evaluation Function:
    f(n) = g(n) + h(n)

Where:
    g(n) = Exact cost from start node to current node n (accumulated Euclidean distance).
    h(n) = Admissible heuristic estimating cost from node n to goal (Euclidean distance).
    f(n) = Total estimated cost of cheapest solution through node n.

Admissibility Condition:
    An admissible heuristic never overestimates the true cost to reach the goal:
    h(n) <= h*(n).
    Since straight-line Euclidean distance is the shortest possible distance in
    continuous space, h(n) = hypot(dx, dy) is strictly admissible and consistent,
    guaranteeing that A* produces the mathematically optimal shortest path.

Key Features:
- 8-connected grid neighborhood with diagonal movement penalty (sqrt(2)).
- Priority queue implementation via Python's min-heap (`heapq`).
- Path smoothing via Line-of-Sight (Bresenham's / ray-marching shortcutting).
"""

import math
import heapq
import numpy as np
from typing import List, Tuple, Optional, Dict, Any


class AStarPlanner:
    """A* grid-based path planning with heuristic evaluation and path smoothing."""

    def __init__(self, resolution: float = 0.05, inflation_radius: float = 0.20):
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.grid: Optional[np.ndarray] = None
        self.rows: int = 0
        self.cols: int = 0
        self.origin_x: float = 0.0
        self.origin_y: float = 0.0

        # 8-connectivity: (d_col, d_row, step_cost)
        sqrt2 = math.sqrt(2.0)
        self.motions = [
            (1, 0, 1.0),      # East
            (-1, 0, 1.0),     # West
            (0, 1, 1.0),      # North
            (0, -1, 1.0),     # South
            (1, 1, sqrt2),    # North-East
            (-1, 1, sqrt2),   # North-West
            (1, -1, sqrt2),   # South-East
            (-1, -1, sqrt2)   # South-West
        ]

    def set_map(self, grid: np.ndarray, metadata: Dict[str, Any]):
        """Sets the occupancy grid map and spatial calibration."""
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.resolution = float(metadata.get("resolution", self.resolution))
        self.origin_x = float(metadata.get("origin_x", 0.0))
        self.origin_y = float(metadata.get("origin_y", 0.0))

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Converts continuous world coordinates (meters) to discrete grid indices (col, row)."""
        col = int((wx - self.origin_x) / self.resolution)
        row = int((wy - self.origin_y) / self.resolution)
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Converts discrete grid indices (col, row) to continuous cell center (meters)."""
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = self.origin_y + (row + 0.5) * self.resolution
        return wx, wy

    def is_valid_cell(self, col: int, row: int) -> bool:
        """Checks if cell is inside map boundaries and not an obstacle."""
        if col < 0 or col >= self.cols or row < 0 or row >= self.rows:
            return False
        if self.grid is not None and self.grid[row, col] > 50:
            return False
        return True

    @staticmethod
    def heuristic(c1: int, r1: int, c2: int, r2: int) -> float:
        """Euclidean distance heuristic (admissible and monotonic)."""
        return math.hypot(c2 - c1, r2 - r1)

    def plan(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        smooth: bool = True
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Calculates optimal path from start to goal.
        Returns:
            List of (x, y) waypoints in meters, or None if no collision-free path exists.
        """
        if self.grid is None:
            raise ValueError("Map has not been set. Call set_map() first.")

        start_c, start_r = self.world_to_grid(start_world[0], start_world[1])
        goal_c, goal_r = self.world_to_grid(goal_world[0], goal_world[1])

        # Validate start and goal
        if not self.is_valid_cell(start_c, start_r):
            # Find nearest free neighbor cell for start
            start_c, start_r = self.find_nearest_free_cell(start_c, start_r)
            if start_c is None:
                return None

        if not self.is_valid_cell(goal_c, goal_r):
            goal_c, goal_r = self.find_nearest_free_cell(goal_c, goal_r)
            if goal_c is None:
                return None

        start_node = (start_c, start_r)
        goal_node = (goal_c, goal_r)

        if start_node == goal_node:
            return [start_world, goal_world]

        # Priority queue stores tuples: (f_score, h_score, (col, row))
        open_set = []
        h_start = self.heuristic(start_c, start_r, goal_c, goal_r)
        heapq.heappush(open_set, (h_start, h_start, start_node))

        # g_score maps node -> lowest known cost from start
        g_score: Dict[Tuple[int, int], float] = {start_node: 0.0}

        # came_from maps node -> predecessor for path reconstruction
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

        # Closed set for fast visited lookup
        closed_set = set()

        while open_set:
            current_f, current_h, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            if current == goal_node:
                # Reconstruct discrete path
                grid_path = self.reconstruct_path(came_from, current)
                # Convert to world coordinates
                world_path = [self.grid_to_world(c, r) for c, r in grid_path]
                world_path[0] = start_world  # Clamp start
                world_path[-1] = goal_world  # Clamp goal

                if smooth:
                    return self.smooth_path(world_path)
                return world_path

            closed_set.add(current)
            curr_c, curr_r = current
            curr_g = g_score[current]

            for dc, dr, move_cost in self.motions:
                neighbor = (curr_c + dc, curr_r + dr)
                nbr_c, nbr_r = neighbor

                if not self.is_valid_cell(nbr_c, nbr_r) or neighbor in closed_set:
                    continue

                # Diagonal corner-cutting prevention
                if dc != 0 and dr != 0:
                    if not self.is_valid_cell(curr_c + dc, curr_r) or not self.is_valid_cell(curr_c, curr_r + dr):
                        continue

                tentative_g = curr_g + move_cost

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h_score = self.heuristic(nbr_c, nbr_r, goal_c, goal_r)
                    f_score = tentative_g + h_score
                    heapq.heappush(open_set, (f_score, h_score, neighbor))

        return None  # No path found

    def find_nearest_free_cell(self, c: int, r: int, max_radius: int = 25) -> Tuple[Optional[int], Optional[int]]:
        """Spirals outwards to find nearest non-colliding cell if query point is on obstacle boundary."""
        for rad in range(1, max_radius + 1):
            for dc in range(-rad, rad + 1):
                for dr in range(-rad, rad + 1):
                    nc, nr = c + dc, r + dr
                    if self.is_valid_cell(nc, nr):
                        return nc, nr
        return None, None

    @staticmethod
    def reconstruct_path(
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        current: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """Backtracks from goal node to start node to assemble trajectory."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Path Shortcutting via Line-of-Sight Ray Marching.
        Removes unnecessary intermediate grid-induced jagged turns while guaranteeing
        clearance from obstacles.
        """
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            furthest_visible = current_idx + 1
            for next_idx in range(len(path) - 1, current_idx + 1, -1):
                if self.has_line_of_sight(path[current_idx], path[next_idx]):
                    furthest_visible = next_idx
                    break
            smoothed.append(path[furthest_visible])
            current_idx = furthest_visible

        return smoothed

    def has_line_of_sight(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        """Checks if straight line segment connecting p1 and p2 is collision-free."""
        x1, y1 = p1
        x2, y2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = int(math.ceil(dist / (self.resolution * 0.5)))
        if steps == 0:
            return True

        for s in range(steps + 1):
            alpha = s / steps
            px = x1 + alpha * (x2 - x1)
            py = y1 + alpha * (y2 - y1)
            col, row = self.world_to_grid(px, py)
            if not self.is_valid_cell(col, row):
                return False
        return True
