"""
sensors.py — IMU, GPS, and Camera FOV sensor models.
"""
from __future__ import annotations

import math
import random
from typing import Optional, Tuple


# ---------------------------------------------------------------------------
# IMUSensor
# ---------------------------------------------------------------------------

class IMUSensor:
    """
    Inertial Measurement Unit with configurable noise level and EKF-like
    complementary filter attenuation.

    Parameters
    ----------
    noise_sigma : float
        Base 1-σ noise standard deviation (m/s²) in raw (no-fusion) mode.
    fusion_gain : float
        EKF complementary filter weight in [0.05, 1.0].
        Lower = more filtering (less noise).  Noise sigma scales as
        ``noise_sigma / fusion_gain``.
    seed : int
        RNG seed for reproducibility.
    """

    def __init__(
        self,
        noise_sigma: float = 0.1,
        fusion_gain: float = 0.22,
        seed: int = 0,
    ) -> None:
        self.noise_sigma = noise_sigma
        self.fusion_gain = max(0.05, fusion_gain)  # prevent division by zero
        self._rng = random.Random(seed)

    def measure(
        self, true_accel: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """
        Return noisy acceleration measurement.

        Noise = true + N(0, noise_sigma / fusion_gain)
        High fusion_gain (0.8-1.0) → raw/noisy; low (0.05-0.3) → smooth.
        """
        effective_sigma = self.noise_sigma / self.fusion_gain
        ax = true_accel[0] + self._rng.gauss(0.0, effective_sigma)
        ay = true_accel[1] + self._rng.gauss(0.0, effective_sigma)
        az = true_accel[2] + self._rng.gauss(0.0, effective_sigma)
        return ax, ay, az


# ---------------------------------------------------------------------------
# GPSSensor
# ---------------------------------------------------------------------------

class GPSSensor:
    """
    GPS receiver with configurable dropout probability.

    Parameters
    ----------
    dropout_prob : float
        Probability [0, 1] of GPS being unavailable on any given call.
    seed : int
        RNG seed.
    """

    POSITION_NOISE_SIGMA = 0.3  # m — typical consumer GPS accuracy

    def __init__(self, dropout_prob: float = 0.0, seed: int = 0) -> None:
        self.dropout_prob = dropout_prob
        self._rng = random.Random(seed)
        self._dropout_active = False

    def measure(
        self,
        true_pos: Tuple[float, float, float],
        t: float,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Return GPS position fix or None on dropout.

        A dropout is triggered if either the persistent flag is set
        (set externally) or the random coin flip fires.
        """
        if self._dropout_active or self._rng.random() < self.dropout_prob:
            return None
        sigma = self.POSITION_NOISE_SIGMA
        x = true_pos[0] + self._rng.gauss(0.0, sigma)
        y = true_pos[1] + self._rng.gauss(0.0, sigma)
        z = true_pos[2] + self._rng.gauss(0.0, sigma)
        return x, y, z


# ---------------------------------------------------------------------------
# CameraFOV
# ---------------------------------------------------------------------------

class CameraFOV:
    """
    Downward-looking camera with configurable horizontal and vertical FOV.

    Parameters
    ----------
    hfov_deg : float
        Horizontal field of view in degrees.
    vfov_deg : float
        Vertical field of view in degrees.
    altitude_agl : float
        Current altitude above ground level (m).  Should be updated each tick.
    """

    dwell_time_required_s: float = 3.0

    def __init__(
        self,
        hfov_deg: float = 60.0,
        vfov_deg: float = 45.0,
        altitude_agl: float = 20.0,
    ) -> None:
        self.hfov_deg = hfov_deg
        self.vfov_deg = vfov_deg
        self.altitude_agl = altitude_agl

    @property
    def _footprint_dims(self) -> Tuple[float, float]:
        """
        Compute ground footprint width and height from FOV and altitude.

        footprint_w = 2 * altitude * tan(hfov/2)
        footprint_h = 2 * altitude * tan(vfov/2)
        """
        alt = max(1.0, self.altitude_agl)
        w = 2.0 * alt * math.tan(math.radians(self.hfov_deg / 2.0))
        h = 2.0 * alt * math.tan(math.radians(self.vfov_deg / 2.0))
        return w, h

    def footprint(
        self,
        pos: Tuple[float, float, float],
        yaw: float,
    ) -> Tuple[float, float, float, float, float]:
        """
        Return the ground footprint as (cx, cy, width, height, yaw).

        The footprint is a rectangle on the ground plane centred directly
        below the UAV (nadir pointing).  Yaw rotates the rectangle.

        Returns
        -------
        (cx, cy, w, h, yaw_rad)
            cx, cy: ground centre (same as UAV x, y)
            w, h: footprint dimensions in metres
            yaw_rad: orientation of the footprint
        """
        w, h = self._footprint_dims
        return pos[0], pos[1], w, h, yaw

    def covers(
        self,
        poi_pos: Tuple[float, float],
        uav_pos: Tuple[float, float, float],
        yaw: float = 0.0,
    ) -> bool:
        """
        Return True if *poi_pos* (x, y) falls within the camera footprint.

        Transforms PoI into the camera's body frame (rotated by yaw) and
        checks bounds.
        """
        px, py = poi_pos
        ux, uy, _ = uav_pos
        dx = px - ux
        dy = py - uy

        # Rotate into body frame (inverse yaw rotation)
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        bx = dx * cos_y - dy * sin_y
        by = dx * sin_y + dy * cos_y

        w, h = self._footprint_dims
        return abs(bx) <= w / 2.0 and abs(by) <= h / 2.0
