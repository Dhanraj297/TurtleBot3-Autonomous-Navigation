"""
Obstacle Models & Dynamic Interactive Obstacles
Robotics Laboratory ESE

This module provides geometric representations for static and moving obstacles
inside the simulated environment.
"""

import math
from typing import Dict, Any, Tuple


class Obstacle:
    """Base class for environment obstacles."""

    def __init__(self, x: float, y: float, label: str = "Obstacle"):
        self.x = float(x)
        self.y = float(y)
        self.label = label

    def contains_point(self, px: float, py: float, margin: float = 0.0) -> bool:
        raise NotImplementedError

    def to_dict(self) -> Dict[str, Any]:
        raise NotImplementedError


class CircleObstacle(Obstacle):
    """Cylindrical/circular obstacle (pillars, wastebaskets, pedestrians)."""

    def __init__(self, x: float, y: float, radius: float = 0.35, label: str = "Cylinder Pillar"):
        super().__init__(x, y, label)
        self.radius = float(radius)

    def contains_point(self, px: float, py: float, margin: float = 0.0) -> bool:
        return math.hypot(px - self.x, py - self.y) <= (self.radius + margin)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": "circle",
            "x": self.x,
            "y": self.y,
            "radius": self.radius,
            "label": self.label
        }


class BoxObstacle(Obstacle):
    """Rectangular obstacle (tables, shelves, cabinets)."""

    def __init__(self, x: float, y: float, width: float = 1.0, height: float = 1.0, label: str = "Cabinet"):
        super().__init__(x, y, label)
        self.width = float(width)
        self.height = float(height)

    def contains_point(self, px: float, py: float, margin: float = 0.0) -> bool:
        half_w = self.width / 2.0 + margin
        half_h = self.height / 2.0 + margin
        return (self.x - half_w <= px <= self.x + half_w) and (self.y - half_h <= py <= self.y + half_h)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": "box",
            "x": self.x,
            "y": self.y,
            "width": self.width,
            "height": self.height,
            "label": self.label
        }


class DynamicObstacle(CircleObstacle):
    """
    Moving obstacle to test reactive obstacle avoidance and dynamic replanning.
    Can patrol between two points or be placed interactively.
    """

    def __init__(
        self,
        x: float,
        y: float,
        radius: float = 0.30,
        patrol_x2: float = 5.0,
        patrol_y2: float = 5.0,
        speed: float = 0.10,
        label: str = "Dynamic Obstacle"
    ):
        super().__init__(x, y, radius, label)
        self.start_x = x
        self.start_y = y
        self.target_x = patrol_x2
        self.target_y = patrol_y2
        self.speed = speed
        self.moving_to_target = True

    def update(self, dt: float):
        """Oscillates smoothly between start and target points."""
        dest_x = self.target_x if self.moving_to_target else self.start_x
        dest_y = self.target_y if self.moving_to_target else self.start_y

        dx = dest_x - self.x
        dy = dest_y - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            self.moving_to_target = not self.moving_to_target
        else:
            step = min(dist, self.speed * dt)
            self.x += (dx / dist) * step
            self.y += (dy / dist) * step
