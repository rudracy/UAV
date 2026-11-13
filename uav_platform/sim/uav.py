"""
uav.py — UAV state dataclass and dynamics engine.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from uav_platform.sim.world import WeatherModel, ObstacleField


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
RHO = 1.225          # kg/m³  — air density at sea level
CD = 0.6             # drag coefficient (bluff body / multi-rotor)
A_REF = 0.09         # m²     — reference cross-sectional area
MAX_ACCEL = 3.0      # m/s²
MAX_SPEED = 15.0     # m/s
MAX_CLIMB = 5.0      # m/s
HOVER_POWER = 200.0  # W
FWD_POWER_BASE = 250.0  # W (forward flight, no drag)
CLIMB_POWER = 320.0  # W
BATTERY_CAP_WH = 50.0   # Wh = 50 × 3600 J
HOME = (0.0, 0.0, 30.0)


# ---------------------------------------------------------------------------
# UAVState
# ---------------------------------------------------------------------------

@dataclass
class UAVState:
    uav_id: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 10.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    yaw: float = 0.0    # radians, 0 = +X axis
    pitch: float = 0.0  # radians
    roll: float = 0.0   # radians
    battery_pct: float = 100.0
    alive: bool = True
    role: str = "survey"          # "survey" | "relay" | "failed"
    mission_id: int = 0
    task_id: int = -1
    gps_ok: bool = True
    imu_noise_level: float = 0.1
    comm_links: List[int] = field(default_factory=list)
    returning_home: bool = False


# ---------------------------------------------------------------------------
# UAVDynamics
# ---------------------------------------------------------------------------

class UAVDynamics:
    """
    Physics engine for a single multi-rotor UAV.

    Parameters
    ----------
    params : dict
        Must contain the five research parameters:
        - comm_rate_hz       : float  1-20
        - neighbour_count    : int    1-8
        - safety_margin_m    : float  0.5-5.0
        - sensor_fusion_gain : float  0.05-1.0
        - failsafe_threshold_pct : float  5-30
    """

    def __init__(self, params: dict) -> None:
        self.params = params
        self._rng = random.Random()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(
        self,
        state: UAVState,
        target_pos: Tuple[float, float, float],
        dt: float,
        weather: WeatherModel,
        obstacles: ObstacleField,
        params: dict,
        t: float = 0.0,
    ) -> UAVState:
        """
        Advance *state* by one time step *dt* (seconds).

        Returns the mutated state (same object).
        """
        if not state.alive:
            return state

        safety_margin = params.get("safety_margin_m", 2.0)
        fusion_gain = params.get("sensor_fusion_gain", 0.22)
        failsafe_pct = params.get("failsafe_threshold_pct", 15.0)

        # --- Failsafe: override target if battery low ------------------
        if state.battery_pct <= failsafe_pct:
            target_pos = HOME
            state.returning_home = True

        tx, ty, tz = target_pos

        # --- Desired direction vector ----------------------------------
        dx = tx - state.x
        dy = ty - state.y
        dz = tz - state.z
        horiz_dist = math.sqrt(dx * dx + dy * dy)
        total_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        # --- Desired velocity (proportional navigation, clamped) ------
        if total_dist > 0.01:
            speed_desired = min(MAX_SPEED, total_dist * 1.5)
            vx_des = (dx / total_dist) * speed_desired
            vy_des = (dy / total_dist) * speed_desired
            vz_des = max(-MAX_CLIMB, min(MAX_CLIMB, (dz / total_dist) * speed_desired))
        else:
            vx_des, vy_des, vz_des = 0.0, 0.0, 0.0

        # --- Compute required acceleration ----------------------------
        ax = (vx_des - state.vx) / max(dt, 1e-6)
        ay = (vy_des - state.vy) / max(dt, 1e-6)
        az = (vz_des - state.vz) / max(dt, 1e-6)

        # Clamp acceleration
        a_mag = math.sqrt(ax * ax + ay * ay + az * az)
        if a_mag > MAX_ACCEL:
            scale = MAX_ACCEL / a_mag
            ax, ay, az = ax * scale, ay * scale, az * scale

        # --- Quadratic air drag ----------------------------------------
        spd = math.sqrt(state.vx**2 + state.vy**2 + state.vz**2)
        f_drag = 0.5 * RHO * CD * A_REF * spd * spd  # Newtons
        mass = 2.5  # kg  — representative quadrotor
        if spd > 0.001:
            drag_ax = -(state.vx / spd) * (f_drag / mass)
            drag_ay = -(state.vy / spd) * (f_drag / mass)
            drag_az = -(state.vz / spd) * (f_drag / mass)
        else:
            drag_ax = drag_ay = drag_az = 0.0

        ax += drag_ax
        ay += drag_ay
        az += drag_az

        # Store accelerations
        state.ax = ax
        state.ay = ay
        state.az = az

        # --- Integrate velocity ----------------------------------------
        state.vx += ax * dt
        state.vy += ay * dt
        state.vz += az * dt

        # Clamp speed
        spd_new = math.sqrt(state.vx**2 + state.vy**2 + state.vz**2)
        if spd_new > MAX_SPEED:
            scale = MAX_SPEED / spd_new
            state.vx *= scale
            state.vy *= scale
            state.vz *= scale

        # Clamp climb rate
        state.vz = max(-MAX_CLIMB, min(MAX_CLIMB, state.vz))

        # --- Wind drift ------------------------------------------------
        wx, wy, wz = weather.wind_at(t)
        # Wind acceleration effect (proportional to wind vs current vel)
        state.vx += (wx - state.vx) * 0.01 * dt
        state.vy += (wy - state.vy) * 0.01 * dt
        state.vz += (wz - state.vz) * 0.005 * dt

        # --- Integrate position ----------------------------------------
        state.x += state.vx * dt
        state.y += state.vy * dt
        state.z += state.vz * dt

        # Minimum altitude: terrain + 1 m
        from uav_platform.sim.world import Terrain
        terrain = Terrain()
        min_z = terrain.elevation(state.x, state.y) + 1.0
        if state.z < min_z:
            state.z = min_z
            if state.vz < 0:
                state.vz = 0.0

        # --- Sensor noise on position (fusion_gain effect) -------------
        if fusion_gain >= 0.8:
            # Raw / noisy mode
            state.x += self._rng.gauss(0.0, 0.3)
            state.y += self._rng.gauss(0.0, 0.3)
            state.z += self._rng.gauss(0.0, 0.3)
        elif fusion_gain <= 0.3:
            # EKF-smoothed
            state.x += self._rng.gauss(0.0, 0.05)
            state.y += self._rng.gauss(0.0, 0.05)
            state.z += self._rng.gauss(0.0, 0.05)
        else:
            # Intermediate regime
            sigma = 0.05 + (fusion_gain - 0.3) / 0.5 * (0.3 - 0.05)
            state.x += self._rng.gauss(0.0, sigma)
            state.y += self._rng.gauss(0.0, sigma)
            state.z += self._rng.gauss(0.0, sigma)

        # --- GPS dropout: dead-reckoning drift -------------------------
        if not state.gps_ok:
            drift_sigma = 0.1 * fusion_gain  # worse sensor fusion = more drift
            state.x += self._rng.gauss(0.0, drift_sigma)
            state.y += self._rng.gauss(0.0, drift_sigma)
            state.z += self._rng.gauss(0.0, drift_sigma * 0.5)

        # --- Yaw convergence ------------------------------------------
        if horiz_dist > 0.5:
            target_yaw = math.atan2(dy, dx)
            yaw_err = target_yaw - state.yaw
            # Wrap to [-π, π]
            yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi
            yaw_rate = min(abs(yaw_err), math.pi / 4) * math.copysign(1.0, yaw_err)
            state.yaw += yaw_rate * dt
            state.yaw = (state.yaw + math.pi) % (2 * math.pi) - math.pi

        # Approximate pitch/roll from acceleration commands
        g = 9.81
        state.pitch = math.atan2(state.ax, g)
        state.roll = math.atan2(-state.ay, g)

        # --- Safety shield: obstacle avoidance -------------------------
        for obs in obstacles.obstacles:
            odx = state.x - obs.x
            ody = state.y - obs.y
            horiz = math.sqrt(odx * odx + ody * ody)
            if horiz < (obs.r + safety_margin) and state.z < obs.h + safety_margin:
                # Reverse horizontal velocity component toward obstacle
                if horiz > 0.001:
                    nx = odx / horiz
                    ny = ody / horiz
                    # Push away
                    dot = state.vx * nx + state.vy * ny
                    if dot < 0:
                        state.vx -= dot * nx
                        state.vy -= dot * ny
                    # Repulsive displacement
                    overlap = (obs.r + safety_margin) - horiz
                    state.x += nx * overlap * 0.5
                    state.y += ny * overlap * 0.5

        # --- Energy drain ----------------------------------------------
        self.energy_step(state, dt)

        return state

    def energy_step(self, state: UAVState, dt: float) -> None:
        """
        Compute power draw and drain battery.

        Power model:
          - Hover base:        200 W
          - Forward flight:    250 W + drag_power  (additional to hover)
          - Climb bonus:       320 W when vz > 0
          - Drag power:        F_drag * |v|  (recovers to drag term)
        Battery capacity: 50 Wh = 180,000 J
        """
        if not state.alive:
            return

        spd = math.sqrt(state.vx**2 + state.vy**2 + state.vz**2)
        spd_horiz = math.sqrt(state.vx**2 + state.vy**2)

        # Drag power
        f_drag = 0.5 * RHO * CD * A_REF * spd * spd
        p_drag = f_drag * spd  # Watts

        # Total power
        p_total = HOVER_POWER
        if spd_horiz > 0.5:
            p_total += FWD_POWER_BASE + p_drag
        if state.vz > 0.5:
            p_total += CLIMB_POWER

        energy_j = p_total * dt
        battery_j = BATTERY_CAP_WH * 3600.0
        drain_pct = (energy_j / battery_j) * 100.0

        state.battery_pct = max(0.0, state.battery_pct - drain_pct)
        if state.battery_pct <= 0.0:
            state.alive = False
            state.role = "failed"
