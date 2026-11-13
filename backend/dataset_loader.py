"""
Dataset Loader & Telemetry Processor
Loads and processes empirical flight telemetry and Monte Carlo datasets.
Grounds simulation in ETH Zurich EuRoC MAV ADIS16448 IMU telemetry.
"""

import csv
import json
import os
import math
from typing import Dict, List, Any, Optional

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, "data")

class DatasetLoader:
    def __init__(self):
        self.telemetry_path = os.path.join(DATA_DIR, "euroc_mav_telemetry.csv")
        self.monte_carlo_path = os.path.join(DATA_DIR, "monte_carlo_runs.csv")
        self.calibration_path = os.path.join(DATA_DIR, "sensor_calibration.json")
        self.ablation_path = os.path.join(DATA_DIR, "ablation_matrix.json")

        self._telemetry_cache: List[Dict[str, Any]] = []
        self._monte_carlo_cache: List[Dict[str, Any]] = []
        self._calibration_cache: Dict[str, Any] = {}
        self._ablation_cache: Dict[str, Any] = {}

        self.load_all()

    def load_all(self):
        """Loads and caches all datasets into memory."""
        self._load_telemetry()
        self._load_monte_carlo()
        self._load_calibration()
        self._load_ablation()

    def _load_telemetry(self):
        if not os.path.exists(self.telemetry_path):
            return
        rows = []
        with open(self.telemetry_path, mode="r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Type conversions
                parsed = {
                    "timestamp_ns": int(row["timestamp_ns"]),
                    "t_sec": float(row["t_sec"]),
                    "w_RS_S_x": float(row["w_RS_S_x_rad_s"]),
                    "w_RS_S_y": float(row["w_RS_S_y_rad_s"]),
                    "w_RS_S_z": float(row["w_RS_S_z_rad_s"]),
                    "a_RS_S_x": float(row["a_RS_S_x_m_s2"]),
                    "a_RS_S_y": float(row["a_RS_S_y_m_s2"]),
                    "a_RS_S_z": float(row["a_RS_S_z_m_s2"]),
                    "gps_lat": float(row["gps_lat"]),
                    "gps_lon": float(row["gps_lon"]),
                    "gps_alt": float(row["gps_alt_m"]),
                    "gps_dop": float(row["gps_dop"]),
                    "gps_dropout": bool(int(row["gps_dropout_active"])),
                    "gt_x": float(row["ground_truth_x"]),
                    "gt_y": float(row["ground_truth_y"]),
                    "gt_z": float(row["ground_truth_z"]),
                    "flight_phase": row["flight_phase"]
                }
                rows.append(parsed)
        self._telemetry_cache = rows

    def _load_monte_carlo(self):
        if not os.path.exists(self.monte_carlo_path):
            return
        rows = []
        with open(self.monte_carlo_path, mode="r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append({
                    "run_id": int(row["run_id"]),
                    "swarm_size": int(row["swarm_size"]),
                    "configuration": row["configuration"],
                    "coverage_pct": float(row["coverage_pct"]),
                    "collisions_count": int(row["collisions_count"]),
                    "active_agents": int(row["active_agents"]),
                    "gps_drift_m": float(row["gps_drift_m"]),
                    "comm_overhead_kbps": float(row["comm_overhead_kbps"]),
                    "latency_ms": float(row["latency_ms"]),
                    "scalability_idx": float(row["scalability_idx"]),
                    "robustness_idx": float(row["robustness_idx"]),
                    "seed": int(row["seed"])
                })
        self._monte_carlo_cache = rows

    def _load_calibration(self):
        if os.path.exists(self.calibration_path):
            with open(self.calibration_path, mode="r", encoding="utf-8") as f:
                self._calibration_cache = json.load(f)

    def _load_ablation(self):
        if os.path.exists(self.ablation_path):
            with open(self.ablation_path, mode="r", encoding="utf-8") as f:
                self._ablation_cache = json.load(f)

    def get_dataset_summary(self) -> Dict[str, Any]:
        """Returns comprehensive summary metadata of the loaded datasets."""
        phases = set(r["flight_phase"] for r in self._telemetry_cache)
        dropout_count = sum(1 for r in self._telemetry_cache if r["gps_dropout"])
        
        return {
            "status": "ready",
            "telemetry": {
                "record_count": len(self._telemetry_cache),
                "duration_seconds": self._telemetry_cache[-1]["t_sec"] if self._telemetry_cache else 0.0,
                "sampling_frequency_hz": 20.0,
                "sensor_platform": "EuRoC MAV AscTec Firefly (ADIS16448 IMU)",
                "phases": list(phases),
                "gps_dropout_samples": dropout_count,
                "gps_dropout_duration_sec": round(dropout_count * 0.05, 2)
            },
            "monte_carlo": {
                "total_trial_records": len(self._monte_carlo_cache),
                "swarm_sizes": [5, 10, 25, 50],
                "runs_per_configuration": 30,
                "configurations": ["Baseline", "Tuned"]
            },
            "calibration": self._calibration_cache.get("imu_noise_parameters", {})
        }

    def get_telemetry_sample(self, offset: int = 0, limit: int = 25, phase: Optional[str] = None) -> Dict[str, Any]:
        """Returns paginated or phase-filtered telemetry records."""
        data = self._telemetry_cache
        if phase:
            data = [r for r in data if r["flight_phase"].lower() == phase.lower()]
        
        total = len(data)
        sliced = data[offset : offset + limit]
        return {
            "total": total,
            "offset": offset,
            "limit": limit,
            "records": sliced
        }

    def get_benchmark_statistics(self) -> Dict[str, Any]:
        """Calculates mean, std dev, and performance gains from the Monte Carlo runs."""
        swarm_sizes = [5, 10, 25, 50]
        baseline_stats = {}
        tuned_stats = {}
        improvements = {}

        for n in swarm_sizes:
            # Baseline
            b_runs = [r for r in self._monte_carlo_cache if r["swarm_size"] == n and r["configuration"] == "Baseline"]
            t_runs = [r for r in self._monte_carlo_cache if r["swarm_size"] == n and r["configuration"] == "Tuned"]

            if b_runs:
                b_covs = [r["coverage_pct"] for r in b_runs]
                b_cols = [r["collisions_count"] for r in b_runs]
                b_scals = [r["scalability_idx"] for r in b_runs]
                b_robs = [r["robustness_idx"] for r in b_runs]

                b_cov_mean = sum(b_covs) / len(b_covs)
                b_cov_std = math.sqrt(sum((x - b_cov_mean) ** 2 for x in b_covs) / len(b_covs))
                b_col_mean = sum(b_cols) / len(b_cols)
                b_col_std = math.sqrt(sum((x - b_col_mean) ** 2 for x in b_cols) / len(b_cols))
                b_scal_mean = sum(b_scals) / len(b_scals)
                b_rob_mean = sum(b_robs) / len(b_robs)

                baseline_stats[str(n)] = {
                    "coverage_mean": round(b_cov_mean, 1),
                    "coverage_std": round(b_cov_std, 1),
                    "collisions_mean": round(b_col_mean, 1),
                    "collisions_std": round(b_col_std, 1),
                    "scalability_idx": round(b_scal_mean, 1),
                    "robustness_idx": round(b_rob_mean, 1)
                }

            if t_runs:
                t_covs = [r["coverage_pct"] for r in t_runs]
                t_cols = [r["collisions_count"] for r in t_runs]
                t_scals = [r["scalability_idx"] for r in t_runs]
                t_robs = [r["robustness_idx"] for r in t_runs]

                t_cov_mean = sum(t_covs) / len(t_covs)
                t_cov_std = math.sqrt(sum((x - t_cov_mean) ** 2 for x in t_covs) / len(t_covs))
                t_col_mean = sum(t_cols) / len(t_cols)
                t_col_std = math.sqrt(sum((x - t_col_mean) ** 2 for x in t_cols) / len(t_cols))
                t_scal_mean = sum(t_scals) / len(t_scals)
                t_rob_mean = sum(t_robs) / len(t_robs)

                tuned_stats[str(n)] = {
                    "coverage_mean": round(t_cov_mean, 1),
                    "coverage_std": round(t_cov_std, 1),
                    "collisions_mean": round(t_col_mean, 1),
                    "collisions_std": round(t_col_std, 1),
                    "scalability_idx": round(t_scal_mean, 1),
                    "robustness_idx": round(t_rob_mean, 1)
                }

            if str(n) in baseline_stats and str(n) in tuned_stats:
                b = baseline_stats[str(n)]
                t = tuned_stats[str(n)]
                scal_gain = ((t["scalability_idx"] - b["scalability_idx"]) / b["scalability_idx"]) * 100.0
                rob_gain = ((t["robustness_idx"] - b["robustness_idx"]) / b["robustness_idx"]) * 100.0
                improvements[str(n)] = {
                    "scalability_gain_pct": round(scal_gain, 1),
                    "robustness_gain_pct": round(rob_gain, 1)
                }

        return {
            "swarm_sizes": swarm_sizes,
            "baseline": baseline_stats,
            "tuned": tuned_stats,
            "improvements": improvements,
            "ablation_study": self._ablation_cache.get("ablation_study_records", [])
        }

