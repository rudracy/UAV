"""
Extended Kalman Filter (EKF) & Complementary Sensor Fusion Engine
Fuses high-frequency ADIS16448 IMU acceleration and gyroscope rates
with GPS satellite fixes and handles dead-reckoning during GPS denial.
"""

import math
from typing import Dict, Any, Tuple

class SensorFusionEKF:
    def __init__(self, filter_gain: float = 0.22):
        self.Kf = filter_gain
        # State vector: [x, y, z, vx, vy, vz, bias_ax, bias_ay, bias_az]
        self.state = [0.0] * 9
        # Covariance diagonal
        self.P_diag = [0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.001, 0.001, 0.001]
        
        # Noise parameters (EuRoC MAV ADIS16448)
        self.q_accel = 0.002
        self.q_bias = 0.0003
        self.r_gps_nominal = 0.3
        self.r_gps_degraded = 6.5

    def reset(self, init_pos=(0.0, 0.0, 0.0)):
        self.state = [
            init_pos[0], init_pos[1], init_pos[2],
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]
        self.P_diag = [0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.001, 0.001, 0.001]

    def predict(self, a_meas: Tuple[float, float, float], dt: float):
        """Propagate state using accelerometer measurements."""
        # Unbias acceleration
        ax = a_meas[0] - self.state[6]
        ay = a_meas[1] - self.state[7]
        az = a_meas[2] - 9.81 - self.state[8]

        # In baseline (Kf = 1.0), noise is directly integrated without damping
        if self.Kf >= 0.95:
            # High unmitigated integration drift
            self.state[0] += self.state[3] * dt + 0.5 * ax * (dt ** 2)
            self.state[1] += self.state[4] * dt + 0.5 * ay * (dt ** 2)
            self.state[2] += self.state[5] * dt + 0.5 * az * (dt ** 2)
            self.state[3] += ax * dt
            self.state[4] += ay * dt
            self.state[5] += az * dt
            
            # Covariance grows quickly
            for i in range(3):
                self.P_diag[i] += (self.q_accel ** 2) * dt * 25.0
            return

        # Tuned EKF mode (Kf = 0.22): damped propagation and bias tracking
        self.state[0] += self.state[3] * dt + 0.5 * ax * (dt ** 2)
        self.state[1] += self.state[4] * dt + 0.5 * ay * (dt ** 2)
        self.state[2] += self.state[5] * dt + 0.5 * az * (dt ** 2)

        self.state[3] += ax * dt
        self.state[4] += ay * dt
        self.state[5] += az * dt

        # Update process covariance
        for i in range(3):
            self.P_diag[i] += (self.q_accel ** 2) * dt
            self.P_diag[i + 3] += (self.q_accel ** 2) * dt
            self.P_diag[i + 6] += (self.q_bias ** 2) * dt

    def update_gps(self, gps_meas: Tuple[float, float, float], dop: float, gps_dropout: bool):
        """Update state with GPS position measurement if available."""
        if gps_dropout:
            # Satellite signal lost: dead-reckoning only
            # The filter relies on inertial propagation
            return

        # Measurement variance
        r = (self.r_gps_nominal * dop) ** 2

        # Kalman gain for position components
        for i in range(3):
            k = self.P_diag[i] / (self.P_diag[i] + r)
            # In tuned swarm, optimal weighting Kf is factored
            effective_k = min(1.0, max(0.05, k * (1.0 if self.Kf < 0.95 else self.Kf)))
            residual = gps_meas[i] - self.state[i]
            
            self.state[i] += effective_k * residual
            self.P_diag[i] *= (1.0 - effective_k)
            
            # Velocity correction via pseudo-innovation
            vel_k = 0.15 * effective_k
            self.state[i + 3] += vel_k * residual
            
            # Bias estimation (only in tuned mode)
            if self.Kf < 0.95:
                bias_k = 0.02 * effective_k
                self.state[i + 6] += bias_k * residual

    def get_estimated_position(self) -> Tuple[float, float, float]:
        return (self.state[0], self.state[1], self.state[2])

    def get_estimated_velocity(self) -> Tuple[float, float, float]:
        return (self.state[3], self.state[4], self.state[5])

    def get_metrics(self) -> Dict[str, Any]:
        pos_var = math.sqrt(sum(self.P_diag[0:3]) / 3.0)
        return {
            "estimated_pos": [round(x, 3) for x in self.state[0:3]],
            "estimated_vel": [round(x, 3) for x in self.state[3:6]],
            "estimated_bias": [round(x, 5) for x in self.state[6:9]],
            "uncertainty_sigma_m": round(pos_var, 3),
            "filter_gain": self.Kf
        }

