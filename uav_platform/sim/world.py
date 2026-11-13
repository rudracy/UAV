"""
world.py — Terrain, WeatherModel, ObstacleField
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Tuple


# ---------------------------------------------------------------------------
# Terrain
# ---------------------------------------------------------------------------

class Terrain:
    """
    100 × 100 m conceptual grid.  Elevation is computed analytically via
    perlin-like sine waves so any (x, y) can be queried without storing the
    full grid.
    """

    def elevation(self, x: float, y: float) -> float:
        """Return terrain height (m) at world position (x, y)."""
        return (
            2.0 * math.sin(x / 30.0)
            + 1.5 * math.sin(y / 20.0)
            + 0.5 * math.sin(x / 10.0 + y / 15.0)
        )

    def line_of_sight(
        self,
        p1: Tuple[float, float, float],
        p2: Tuple[float, float, float],
        samples: int = 20,
    ) -> bool:
        """
        Return True if the straight line from p1=(x,y,z) to p2=(x,y,z) is
        not blocked by terrain.  Uses linear interpolation along the segment
        and checks that altitude stays above terrain at every sample.
        """
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        for i in range(samples + 1):
            t = i / samples
            xc = x1 + t * (x2 - x1)
            yc = y1 + t * (y2 - y1)
            zc = z1 + t * (z2 - z1)
            if zc <= self.elevation(xc, yc):
                return False
        return True


# ---------------------------------------------------------------------------
# WeatherModel
# ---------------------------------------------------------------------------

@dataclass
class WeatherModel:
    """
    Constant mean wind plus stochastic gusts.

    Parameters
    ----------
    wx, wy, wz : float
        Mean wind vector components (m/s).
    turbulence_intensity : float
        Scales the magnitude of random gusts (m/s RMS).
    seed : int
        Random seed for reproducible weather.
    """

    wx: float = 2.0
    wy: float = 1.0
    wz: float = 0.0
    turbulence_intensity: float = 0.5
    seed: int = 0

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def gust(self, t: float) -> Tuple[float, float, float]:
        """
        Return a random wind perturbation vector at time *t*.

        Uses a simple band-limited random process: the gust amplitude is
        drawn from a normal distribution scaled by turbulence_intensity.
        The seed is mixed with time so successive calls at the same t return
        the same value (deterministic per tick).
        """
        rng = random.Random(hash((self.seed, round(t, 2))))
        sigma = self.turbulence_intensity
        gx = rng.gauss(0.0, sigma)
        gy = rng.gauss(0.0, sigma)
        gz = rng.gauss(0.0, sigma * 0.3)  # vertical gusts are weaker
        return gx, gy, gz

    def wind_at(self, t: float) -> Tuple[float, float, float]:
        """Total wind (mean + gust) at time t."""
        gx, gy, gz = self.gust(t)
        return self.wx + gx, self.wy + gy, self.wz + gz


# ---------------------------------------------------------------------------
# ObstacleField
# ---------------------------------------------------------------------------

@dataclass
class Cylinder:
    x: float
    y: float
    r: float   # radius (m)
    h: float   # height (m)


class ObstacleField:
    """
    A collection of vertical cylindrical obstacles.

    Each obstacle is defined by centre (x, y), radius r, and height h.
    """

    def __init__(self, obstacles: List[dict] | None = None) -> None:
        """
        Parameters
        ----------
        obstacles : list of dicts with keys {x, y, r, h}
        """
        self.obstacles: List[Cylinder] = []
        if obstacles:
            for o in obstacles:
                self.obstacles.append(Cylinder(o["x"], o["y"], o["r"], o["h"]))

    # ------------------------------------------------------------------
    def check_collision(self, x: float, y: float, z: float, r: float) -> bool:
        """
        Return True if a sphere of radius *r* centred at (x, y, z) intersects
        any cylindrical obstacle.
        """
        for obs in self.obstacles:
            dx = x - obs.x
            dy = y - obs.y
            horiz_dist = math.sqrt(dx * dx + dy * dy)
            if horiz_dist < (obs.r + r) and z < obs.h:
                return True
        return False

    def nearest_obstacle(self, x: float, y: float) -> float:
        """
        Return the horizontal distance (m) to the nearest obstacle surface.
        Returns +inf when there are no obstacles.
        """
        if not self.obstacles:
            return math.inf
        min_dist = math.inf
        for obs in self.obstacles:
            dx = x - obs.x
            dy = y - obs.y
            surface_dist = math.sqrt(dx * dx + dy * dy) - obs.r
            if surface_dist < min_dist:
                min_dist = surface_dist
        return max(0.0, min_dist)

    @classmethod
    def generate_random(
        cls,
        n: int = 10,
        arena: float = 400.0,
        rng: random.Random | None = None,
    ) -> "ObstacleField":
        """Generate *n* random cylindrical obstacles within the arena."""
        if rng is None:
            rng = random.Random(42)
        obstacles = []
        for _ in range(n):
            obstacles.append(
                {
                    "x": rng.uniform(-arena / 2, arena / 2),
                    "y": rng.uniform(-arena / 2, arena / 2),
                    "r": rng.uniform(2.0, 8.0),
                    "h": rng.uniform(10.0, 40.0),
                }
            )
        return cls(obstacles)
