"""
task_allocator.py — Disaster search mission task allocator.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from uav_platform.sim.uav import UAVState
from uav_platform.sim.sensors import CameraFOV


# ---------------------------------------------------------------------------
# PointOfInterest
# ---------------------------------------------------------------------------

@dataclass
class PointOfInterest:
    id: int
    x: float
    y: float
    priority: int = 3            # 1 (low) – 5 (critical)
    surveyed: bool = False
    survey_progress_pct: float = 0.0
    discovered_by_uav: int = -1  # -1 = not yet discovered


# ---------------------------------------------------------------------------
# TaskAllocator
# ---------------------------------------------------------------------------

class TaskAllocator:
    """
    Greedy auction-style task allocator for disaster search missions.

    At each allocation tick, every eligible UAV is matched to the highest-
    scoring unfinished PoI.  The scoring function balances mission priority,
    proximity, battery health, obstacle risk, and GCS connectivity.
    """

    def __init__(self, obstacles: "ObstacleField | None" = None) -> None:  # noqa: F821
        self._obstacles = obstacles

    # ------------------------------------------------------------------
    # assign
    # ------------------------------------------------------------------

    def assign(
        self,
        uavs: List[UAVState],
        pois: List[PointOfInterest],
        comm_graph: "ConnectivityGraph",  # noqa: F821
        params: dict,
    ) -> Dict[int, int]:
        """
        Assign alive, non-returning UAVs to unsurveyed PoIs.

        Returns
        -------
        dict
            Mapping {uav_id: poi_id}.
        """
        neighbour_count = params.get("neighbour_count", 4)
        assignment: Dict[int, int] = {}

        eligible_uavs = [
            u for u in uavs if u.alive and not u.returning_home and u.role != "failed"
        ]
        open_pois = [p for p in pois if not p.surveyed]

        if not eligible_uavs or not open_pois:
            return assignment

        # Build coverage: which PoIs are already assigned to retain continuity
        claimed: Dict[int, int] = {}  # poi_id -> uav_id

        # Sort UAVs by battery (highest first) so better-resourced UAVs bid first
        eligible_uavs_sorted = sorted(
            eligible_uavs, key=lambda u: u.battery_pct, reverse=True
        )

        for uav in eligible_uavs_sorted:
            best_score = -math.inf
            best_poi = None

            for poi in open_pois:
                if poi.id in claimed:
                    continue  # already taken this tick

                score = self._score(uav, poi, comm_graph, neighbour_count)
                if score > best_score:
                    best_score = score
                    best_poi = poi

            if best_poi is not None:
                assignment[uav.uav_id] = best_poi.id
                claimed[best_poi.id] = uav.uav_id

        return assignment

    # ------------------------------------------------------------------
    # update_survey_progress
    # ------------------------------------------------------------------

    def update_survey_progress(
        self,
        uav_state: UAVState,
        poi: PointOfInterest,
        camera: CameraFOV,
        dt: float,
    ) -> None:
        """
        Advance survey progress when the UAV is actively observing the PoI.

        Conditions for progress:
        - UAV is alive
        - UAV is within camera footprint (covers PoI)
        - Altitude AGL < 50 m
        - Horizontal speed < 8 m/s
        """
        if poi.surveyed or not uav_state.alive:
            return

        spd_horiz = math.sqrt(uav_state.vx**2 + uav_state.vy**2)
        if spd_horiz >= 8.0:
            return
        if uav_state.z >= 50.0:
            return

        camera.altitude_agl = uav_state.z
        in_view = camera.covers(
            (poi.x, poi.y), (uav_state.x, uav_state.y, uav_state.z), uav_state.yaw
        )
        if not in_view:
            return

        # Progress per second: 100% takes exactly dwell_time_required_s
        progress_rate = 100.0 / camera.dwell_time_required_s  # %/s
        poi.survey_progress_pct = min(100.0, poi.survey_progress_pct + progress_rate * dt)

        if poi.survey_progress_pct >= 100.0:
            poi.surveyed = True
            if poi.discovered_by_uav < 0:
                poi.discovered_by_uav = uav_state.uav_id

    # ------------------------------------------------------------------
    # handle_uav_failure
    # ------------------------------------------------------------------

    def handle_uav_failure(
        self,
        failed_id: int,
        assignments: Dict[int, int],
        uavs: List[UAVState],
    ) -> Dict[int, int]:
        """
        Redistribute the failed UAV's task to the nearest alive UAV that is
        not already returning home.

        Returns the updated assignments dict.
        """
        if failed_id not in assignments:
            return assignments

        poi_id = assignments.pop(failed_id)
        candidates = [
            u for u in uavs
            if u.alive and not u.returning_home and u.uav_id != failed_id
        ]
        if not candidates:
            return assignments

        # Find failed UAV position (we don't have it here, default to 0,0)
        # Find the candidate not already assigned to a task (prefer idle)
        unassigned = [c for c in candidates if c.uav_id not in assignments]
        pool = unassigned if unassigned else candidates

        # Pick nearest to GCS origin as proxy (we don't have POI coords here)
        # Use lowest battery consumption (highest battery) heuristic
        nearest = max(pool, key=lambda u: u.battery_pct)
        assignments[nearest.uav_id] = poi_id
        return assignments

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _score(
        self,
        uav: UAVState,
        poi: PointOfInterest,
        comm_graph: "ConnectivityGraph",  # noqa: F821
        neighbour_count: int,
    ) -> float:
        """
        score = priority * 10 / (dist + 1) * battery_weight
                        * risk_weight * connectivity_weight
        """
        dist = math.sqrt((uav.x - poi.x) ** 2 + (uav.y - poi.y) ** 2)
        battery_weight = uav.battery_pct / 100.0

        # Obstacle density near PoI (0 = clear, 1 = very dense)
        risk_weight = 1.0 - self._obstacle_density(poi.x, poi.y)

        # Connectivity weight
        connected = comm_graph.is_connected_to_gcs(uav.uav_id)
        if connected:
            connectivity_weight = 1.0
        else:
            # In k-mesh mode, relay chain quality matters more
            if neighbour_count < 8:
                connectivity_weight = 0.2
            else:
                connectivity_weight = 0.3

        score = (
            poi.priority * 10.0 / (dist + 1.0)
            * battery_weight
            * risk_weight
            * connectivity_weight
        )
        # Slight bonus for resuming own partial survey
        if poi.survey_progress_pct > 0 and poi.discovered_by_uav == uav.uav_id:
            score *= 1.2

        return score

    def _obstacle_density(self, x: float, y: float, radius: float = 30.0) -> float:
        """
        Returns a value in [0, 1] representing obstacle density near (x, y).
        0 = no obstacles, 1 = maximum density.
        """
        if self._obstacles is None:
            return 0.0
        count = 0
        for obs in self._obstacles.obstacles:
            d = math.sqrt((x - obs.x) ** 2 + (y - obs.y) ** 2)
            if d < radius:
                count += 1
        # Saturates at 5 obstacles nearby
        return min(1.0, count / 5.0)
