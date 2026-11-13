"""
coverage.py — 2D coverage grid for arena survey tracking.
"""
from __future__ import annotations

import json
import math
from typing import List

import numpy as np


class CoverageGrid:
    """
    A 2D boolean occupancy grid that tracks which cells of the arena have
    been surveyed by any UAV's camera footprint.

    The grid is centred at the arena origin (0, 0).  The arena is assumed
    to be square: [-arena_size/2, +arena_size/2] in both X and Y.

    Parameters
    ----------
    arena_size_m : float
        Side length of the square arena (m).
    cell_size_m : float
        Side length of each grid cell (m).  Default 5.0 m.
    """

    def __init__(self, arena_size_m: float, cell_size_m: float = 5.0) -> None:
        self.arena_size_m = arena_size_m
        self.cell_size_m = cell_size_m
        self.n_cells = int(math.ceil(arena_size_m / cell_size_m))
        self._grid = np.zeros((self.n_cells, self.n_cells), dtype=bool)

    # ------------------------------------------------------------------

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world coordinates to grid indices (clamped)."""
        half = self.arena_size_m / 2.0
        gx = int((x + half) / self.cell_size_m)
        gy = int((y + half) / self.cell_size_m)
        gx = max(0, min(self.n_cells - 1, gx))
        gy = max(0, min(self.n_cells - 1, gy))
        return gx, gy

    # ------------------------------------------------------------------

    def mark(
        self,
        x: float,
        y: float,
        footprint_w: float,
        footprint_h: float,
        yaw: float,
    ) -> None:
        """
        Mark all grid cells that fall within a camera footprint centred at
        (x, y) with dimensions footprint_w × footprint_h (m) rotated by yaw.

        Uses a rotated rectangle rasterisation: for each grid cell, transform
        its centre into the footprint frame and check bounds.
        """
        # Pre-compute rotation coefficients (body frame)
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        half_w = footprint_w / 2.0
        half_h = footprint_h / 2.0
        half = self.arena_size_m / 2.0

        # Determine bounding box of the rotated footprint in world space
        corners_local = [
            ( half_w,  half_h),
            (-half_w,  half_h),
            (-half_w, -half_h),
            ( half_w, -half_h),
        ]
        corners_world_x = []
        corners_world_y = []
        for lx, ly in corners_local:
            wx = x + lx * math.cos(yaw) - ly * math.sin(yaw)
            wy = y + lx * math.sin(yaw) + ly * math.cos(yaw)
            corners_world_x.append(wx)
            corners_world_y.append(wy)

        min_x = max(-half, min(corners_world_x))
        max_x = min(half, max(corners_world_x))
        min_y = max(-half, min(corners_world_y))
        max_y = min(half, max(corners_world_y))

        # Rasterise bounding box — check each cell centre against rotated rect
        gx_min, gy_min = self._world_to_grid(min_x, min_y)
        gx_max, gy_max = self._world_to_grid(max_x, max_y)

        for gx in range(gx_min, gx_max + 1):
            for gy in range(gy_min, gy_max + 1):
                # Cell centre in world coordinates
                cx_w = -half + (gx + 0.5) * self.cell_size_m
                cy_w = -half + (gy + 0.5) * self.cell_size_m
                # Transform into footprint body frame
                dx = cx_w - x
                dy = cy_w - y
                bx = dx * cos_y - dy * sin_y
                by = dx * sin_y + dy * cos_y
                if abs(bx) <= half_w and abs(by) <= half_h:
                    self._grid[gy, gx] = True

    # ------------------------------------------------------------------

    def coverage_pct(self) -> float:
        """Return the fraction [0, 1] of arena cells marked as covered."""
        total = self.n_cells * self.n_cells
        if total == 0:
            return 0.0
        return float(np.sum(self._grid)) / total

    # ------------------------------------------------------------------

    def to_json(self) -> str:
        """
        Serialize the grid for frontend visualization.

        Returns a JSON string with:
        - ``n_cells``: grid dimension
        - ``cell_size_m``: metres per cell
        - ``arena_size_m``: arena side length
        - ``coverage_pct``: fraction covered
        - ``grid``: flattened list of 0/1 values, row-major (y outer, x inner)
        """
        payload = {
            "n_cells": self.n_cells,
            "cell_size_m": self.cell_size_m,
            "arena_size_m": self.arena_size_m,
            "coverage_pct": round(self.coverage_pct(), 6),
            "grid": self._grid.astype(int).tolist(),
        }
        return json.dumps(payload)
