"""
runner.py — Main simulation runner for the UAV swarm.
"""
from __future__ import annotations

import copy
import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from uav_platform.sim.world import Terrain, WeatherModel, ObstacleField
from uav_platform.sim.uav import UAVState, UAVDynamics
from uav_platform.sim.connectivity import ConnectivityGraph
from uav_platform.sim.sensors import IMUSensor, GPSSensor, CameraFOV
from uav_platform.sim.task_allocator import TaskAllocator, PointOfInterest
from uav_platform.sim.coverage import CoverageGrid


# ---------------------------------------------------------------------------
# SimulationConfig
# ---------------------------------------------------------------------------

@dataclass
class SimulationConfig:
    seed: int = 42
    duration_s: float = 120.0
    n_uavs: int = 10
    n_pois: int = 10
    params: dict = field(default_factory=lambda: {
        "comm_rate_hz": 10.0,
        "neighbour_count": 4,
        "safety_margin_m": 2.0,
        "sensor_fusion_gain": 0.22,
        "failsafe_threshold_pct": 15.0,
    })
    stress: dict = field(default_factory=lambda: {
        "sensor_noise": 0.1,
        "gps_dropout_prob": 0.0,
        "motor_failure_prob": 0.0,
        "packet_loss_extra": 0.0,
        "outage_uav_ids": [],
    })


# ---------------------------------------------------------------------------
# SimulationState
# ---------------------------------------------------------------------------

@dataclass
class SimulationState:
    t: float = 0.0
    uavs: List[UAVState] = field(default_factory=list)
    pois: List[PointOfInterest] = field(default_factory=list)
    comm_graph: Optional[ConnectivityGraph] = None
    events: List[dict] = field(default_factory=list)
    coverage_pct: float = 0.0
    collisions: int = 0
    alive_uavs: int = 0


# ---------------------------------------------------------------------------
# SimulationRunner
# ---------------------------------------------------------------------------

ARENA_SIZE = 400.0   # metres
DT = 0.1             # simulation time step (s)
CAMERA_HFOV = 60.0   # degrees
CAMERA_VFOV = 45.0   # degrees


class SimulationRunner:
    """
    Drives the UAV swarm simulation forward in time.

    Usage::

        cfg = SimulationConfig(seed=7, duration_s=300, n_uavs=15, n_pois=10)
        runner = SimulationRunner(cfg)
        snapshots = runner.run()          # returns states at 1 Hz
        metrics  = runner.get_metrics()   # final performance dict
    """

    def __init__(self, config: SimulationConfig) -> None:
        self.cfg = config
        self._rng = random.Random(config.seed)
        self._np_rng = np.random.default_rng(config.seed)
        self._t = 0.0
        self._total_ticks = 0
        self._collision_count = 0
        self._events: List[dict] = []
        self._motor_failure_ticks: Dict[int, int] = {}  # uav_id -> tick of failure

        # --- World ---
        self._terrain = Terrain()
        self._weather = WeatherModel(
            wx=self._rng.gauss(2.0, 1.0),
            wy=self._rng.gauss(0.5, 0.5),
            wz=0.0,
            turbulence_intensity=0.5,
            seed=config.seed,
        )
        self._obstacles = ObstacleField.generate_random(
            n=12, arena=ARENA_SIZE, rng=random.Random(config.seed + 1)
        )

        # --- UAVs ---
        self._uavs: List[UAVState] = self._init_uavs()
        self._dynamics = UAVDynamics(config.params)

        # --- Sensors (per UAV) ---
        fusion_gain = config.params.get("sensor_fusion_gain", 0.22)
        base_noise = config.stress.get("sensor_noise", 0.1)
        dropout_prob = config.stress.get("gps_dropout_prob", 0.0)
        self._imus: Dict[int, IMUSensor] = {
            u.uav_id: IMUSensor(base_noise, fusion_gain, seed=config.seed + u.uav_id)
            for u in self._uavs
        }
        self._gps_sensors: Dict[int, GPSSensor] = {
            u.uav_id: GPSSensor(dropout_prob, seed=config.seed + u.uav_id + 1000)
            for u in self._uavs
        }
        self._cameras: Dict[int, CameraFOV] = {
            u.uav_id: CameraFOV(CAMERA_HFOV, CAMERA_VFOV, u.z)
            for u in self._uavs
        }

        # --- PoIs ---
        self._pois: List[PointOfInterest] = self._init_pois()

        # --- Connectivity ---
        self._comm_graph = ConnectivityGraph(
            radio_range_m=150.0,
            comm_rate_hz=config.params.get("comm_rate_hz", 10.0),
            neighbour_count=config.params.get("neighbour_count", 4),
        )
        # Pre-apply outages from stress config
        for oid in config.stress.get("outage_uav_ids", []):
            self._comm_graph.simulate_outage(oid)

        # --- Task allocator ---
        self._allocator = TaskAllocator(obstacles=self._obstacles)
        self._assignments: Dict[int, int] = {}

        # --- Coverage grid ---
        self._coverage = CoverageGrid(ARENA_SIZE, cell_size_m=5.0)

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def _init_uavs(self) -> List[UAVState]:
        n = self.cfg.n_uavs
        uavs = []
        for i in range(n):
            angle = 2 * math.pi * i / n
            radius = 20.0  # metres from GCS
            u = UAVState(
                uav_id=i,
                x=radius * math.cos(angle),
                y=radius * math.sin(angle),
                z=10.0,
                yaw=angle + math.pi,  # face outward from centre
                battery_pct=100.0,
                alive=True,
                role="survey",
                mission_id=1,
            )
            uavs.append(u)
        return uavs

    def _init_pois(self) -> List[PointOfInterest]:
        pois = []
        half = ARENA_SIZE / 2.0
        for i in range(self.cfg.n_pois):
            px = self._rng.uniform(-half, half)
            py = self._rng.uniform(-half, half)
            priority = self._rng.randint(1, 5)
            pois.append(PointOfInterest(id=i, x=px, y=py, priority=priority))
        return pois

    # ------------------------------------------------------------------
    # step
    # ------------------------------------------------------------------

    def _get_area_coverage_target(self, uav_id: int, t: float) -> Tuple[float, float, float]:
        """
        Generates continuous disaster area coverage sweep waypoints around obstacle fields
        when a UAV is not locked onto a discrete PoI.
        """
        n = max(1, self.cfg.n_uavs)
        sector_angle = (uav_id / float(n)) * 2.0 * math.pi + (0.12 * t)
        radius = 45.0 + 35.0 * math.sin(0.08 * t + uav_id)
        tx = radius * math.cos(sector_angle)
        ty = radius * math.sin(sector_angle)
        tz = 22.0 + 3.0 * math.sin(0.2 * t + uav_id)
        return (tx, ty, tz)

    def step(self) -> SimulationState:
        """Advance simulation by DT seconds and return current state snapshot."""
        t = self._t
        params = self.cfg.params
        stress = self.cfg.stress
        events: List[dict] = []

        # 1. Update weather (wind gust is deterministic per tick)
        # (weather object recalculates on each wind_at(t) call)

        # 2. Update connectivity with packet loss stress
        extra_loss = float(stress.get("packet_loss_extra_pct", stress.get("packet_loss_extra", 0.0)))
        # Apply any active outages
        for oid in stress.get("outage_uav_ids", []):
            self._comm_graph.simulate_outage(oid)
            
        self._comm_graph.update(self._uavs, self._terrain, t, extra_packet_loss=extra_loss)

        # 3. Task allocation (runs every tick; allocator skips already-assigned)
        self._assignments = self._allocator.assign(
            self._uavs, self._pois, self._comm_graph, params
        )

        # 4. Step UAV dynamics
        motor_fail_prob = stress.get("motor_failure_prob", 0.0)
        forced_failures = {int(uid) for uid in stress.get("forced_failure_uav_ids", [])}
        for uav in self._uavs:
            if not uav.alive:
                continue

            # --- Motor failure injection ---
            if uav.uav_id in forced_failures or motor_fail_prob > 0.0:
                # Probability per tick (DT-adjusted for given Hz)
                per_tick_prob = motor_fail_prob * DT
                if uav.uav_id in forced_failures or self._rng.random() < per_tick_prob:
                    uav.alive = False
                    uav.role = "failed"
                    uav.battery_pct = 0.0
                    events.append({
                        "type": "operator_failure" if uav.uav_id in forced_failures else "motor_failure",
                        "uav_id": uav.uav_id,
                        "t": t,
                    })
                    self._assignments = self._allocator.handle_uav_failure(
                        uav.uav_id, self._assignments, self._uavs
                    )
                    continue

            # --- GPS stress ---
            gps_sensor = self._gps_sensors[uav.uav_id]
            gps_meas = gps_sensor.measure((uav.x, uav.y, uav.z), t)
            if gps_meas is None:
                uav.gps_ok = False
            else:
                uav.gps_ok = True

            # --- IMU noise injection ---
            imu = self._imus[uav.uav_id]
            noisy_accel = imu.measure((uav.ax, uav.ay, uav.az))
            uav.imu_noise_level = imu.noise_sigma / imu.fusion_gain

            # --- Determine target position (PoI or Area Coverage) ---
            poi_id = self._assignments.get(uav.uav_id, None)
            if uav.returning_home:
                target = (0.0, 0.0, 30.0)
            elif poi_id is not None:
                poi = next((p for p in self._pois if p.id == poi_id), None)
                if poi is not None and not poi.surveyed:
                    target = (poi.x, poi.y, 20.0)   # survey altitude 20 m
                else:
                    target = self._get_area_coverage_target(uav.uav_id, t)
            else:
                target = self._get_area_coverage_target(uav.uav_id, t)

            # --- Dynamics step ---
            self._dynamics.step(
                uav, target, DT, self._weather, self._obstacles, params, t
            )

            # --- Update camera ---
            cam = self._cameras[uav.uav_id]
            cam.altitude_agl = uav.z

            # --- Survey progress ---
            if poi_id is not None:
                poi = next((p for p in self._pois if p.id == poi_id), None)
                if poi and not poi.surveyed:
                    self._allocator.update_survey_progress(uav, poi, cam, DT)
                    if poi.surveyed:
                        events.append({
                            "type": "poi_surveyed",
                            "poi_id": poi.id,
                            "uav_id": uav.uav_id,
                            "t": t,
                        })

            # --- Coverage marking ---
            fp_w, fp_h = cam._footprint_dims
            self._coverage.mark(uav.x, uav.y, fp_w, fp_h, uav.yaw)

        # 4b. Inter-UAV collision avoidance (repulsive safety shield)
        safety_margin = params.get("safety_margin_m", 2.0)
        for i, ua in enumerate(self._uavs):
            if not ua.alive:
                continue
            for ub in self._uavs[i + 1:]:
                if not ub.alive:
                    continue
                dx = ua.x - ub.x
                dy = ua.y - ub.y
                dz = ua.z - ub.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if 0.001 < dist < safety_margin:
                    overlap = safety_margin - dist
                    nx = dx / dist
                    ny = dy / dist
                    nz = dz / dist
                    push = 0.5 * overlap * 0.3
                    ua.x += nx * push
                    ua.y += ny * push
                    ua.z += nz * push * 0.3
                    ub.x -= nx * push
                    ub.y -= ny * push
                    ub.z -= nz * push * 0.3
                    # Velocity separation impulse
                    v_dot = (ua.vx - ub.vx) * nx + (ua.vy - ub.vy) * ny
                    if v_dot < 0:
                        ua.vx -= 0.5 * v_dot * nx
                        ua.vy -= 0.5 * v_dot * ny
                        ub.vx += 0.5 * v_dot * nx
                        ub.vy += 0.5 * v_dot * ny

        # 5. Collision detection (Physical airframe contact)
        uav_r = 0.3  # UAV physical radius in meters (0.6m diameter)
        for i, ua in enumerate(self._uavs):
            if not ua.alive:
                continue
            # UAV-obstacle collisions
            if self._obstacles.check_collision(ua.x, ua.y, ua.z, uav_r):
                self._collision_count += 1
                events.append({"type": "uav_obstacle_collision", "uav_id": ua.uav_id, "t": t})

            # UAV-UAV physical collisions
            for ub in self._uavs[i + 1:]:
                if not ub.alive:
                    continue
                d = math.sqrt((ua.x - ub.x)**2 + (ua.y - ub.y)**2 + (ua.z - ub.z)**2)
                if d < (2.0 * uav_r):
                    self._collision_count += 1
                    events.append({
                        "type": "uav_uav_collision",
                        "uav_a": ua.uav_id,
                        "uav_b": ub.uav_id,
                        "t": t,
                    })

        # 6. Update comm_links field on each UAV state
        for uav in self._uavs:
            uav.comm_links = [
                lnk.uav_b if lnk.uav_a == uav.uav_id else lnk.uav_a
                for lnk in self._comm_graph._adj.get(uav.uav_id, [])
                if lnk.active
            ]

        self._t += DT
        self._total_ticks += 1
        self._events.extend(events)

        return SimulationState(
            t=t,
            uavs=copy.deepcopy(self._uavs),
            pois=copy.deepcopy(self._pois),
            comm_graph=self._comm_graph,
            events=events,
            coverage_pct=self._coverage.coverage_pct(),
            collisions=self._collision_count,
            alive_uavs=sum(1 for u in self._uavs if u.alive),
        )

    def apply_runtime_settings(self, settings: dict) -> None:
        """Apply live stress-control changes from the running dashboard."""
        noise_multiplier = max(0.0, min(3.0, float(settings.get("sensor_noise_multiplier", 1.0))))
        packet_loss = max(0.0, min(60.0, float(settings.get("packet_loss_extra_pct", 0.0))))
        gps_dropout = max(0.0, min(1.0, float(settings.get("gps_dropout_prob", 0.0))))
        failures = [int(uid) for uid in settings.get("forced_failure_uav_ids", [])]
        self.cfg.stress.update(
            sensor_noise_multiplier=noise_multiplier,
            sensor_noise=0.1 * noise_multiplier,
            packet_loss_extra_pct=packet_loss,
            gps_dropout_prob=gps_dropout,
            forced_failure_uav_ids=failures,
        )
        for sensor in self._imus.values():
            sensor.noise_sigma = 0.1 * noise_multiplier
        for sensor in self._gps_sensors.values():
            sensor.dropout_prob = gps_dropout

    # ------------------------------------------------------------------
    # run
    # ------------------------------------------------------------------

    def run(self, on_snapshot=None) -> List[SimulationState]:
        """
        Run the full simulation and return snapshots at 2 Hz.

        Internally ticks at DT=0.1s; records a snapshot every 10 ticks.
        """
        snapshots: List[SimulationState] = []
        snapshot_interval = max(1, int(round(0.5 / DT)))  # one snapshot every 0.5 simulated seconds
        tick = 0
        total_ticks = int(round(self.cfg.duration_s / DT))

        while tick < total_ticks:
            state = self.step()
            tick += 1
            if tick % snapshot_interval == 0:
                if tick == total_ticks and state.t < self.cfg.duration_s:
                    state = copy.deepcopy(state)
                    state.t = self.cfg.duration_s
                snapshots.append(state)
                if on_snapshot is not None:
                    on_snapshot(state, tick / total_ticks)

        return snapshots

    # ------------------------------------------------------------------
    # get_metrics
    # ------------------------------------------------------------------

    def get_metrics(self) -> dict:
        """
        Return a dict of final simulation performance metrics.
        """
        alive = [u for u in self._uavs if u.alive]
        total_pois = len(self._pois)
        surveyed = sum(1 for p in self._pois if p.surveyed)

        metrics = {
            "duration_s": round(self._t, 2),
            "n_uavs": self.cfg.n_uavs,
            "alive_uavs": len(alive),
            "failed_uavs": self.cfg.n_uavs - len(alive),
            "n_pois": total_pois,
            "surveyed_pois": surveyed,
            "mission_completion_pct": round(surveyed / max(1, total_pois) * 100.0, 2),
            "coverage_pct": round(self._coverage.coverage_pct() * 100.0, 2),
            "total_collisions": self._collision_count,
            "connectivity_ratio": round(self._comm_graph.connectivity_ratio(), 4),
            "avg_latency_ms": round(self._comm_graph.avg_latency_ms(), 2),
            "queuing_delay_ms": round(self._comm_graph.avg_queuing_delay_ms(), 2),
            "propagation_delay_ms": round(self._comm_graph.avg_propagation_delay_ms(), 4),
            "information_freshness_ms": round(self._comm_graph.information_freshness_ms(), 1),
            "packet_delivery_ratio": round(self._comm_graph.packet_delivery_ratio(), 4),
            "avg_battery_pct": round(
                sum(u.battery_pct for u in self._uavs) / max(1, len(self._uavs)), 2
            ),
            "total_events": len(self._events),
            "params": self.cfg.params,
            "stress": self.cfg.stress,
        }
        return metrics
