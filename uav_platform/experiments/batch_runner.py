"""
batch_runner.py — 30-run Monte Carlo batch experimental campaign engine.
Evaluates Baseline vs Tuned configurations under identical random seeds,
environmental settings, and stress scenarios.
"""
import math
from typing import Dict, List, Any
import numpy as np

from uav_platform.sim.runner import SimulationConfig, SimulationRunner
from uav_platform.api.metrics import compute_scalability_index, compute_robustness_index, compute_improvement
from uav_platform.api.simulation_service import BASELINE_PARAMS, TUNED_PARAMS

def run_single_trial(seed: int, n_uavs: int, params: Dict[str, Any], duration_s: float = 20.0, stress: Dict[str, Any] = None) -> Dict[str, Any]:
    if stress is None:
        stress = {
            "sensor_noise": 0.5,
            "gps_dropout_prob": 0.1,
            "motor_failure_prob": 0.0,
            "packet_loss_extra": 0.1,
            "outage_uav_ids": []
        }

    cfg = SimulationConfig(
        seed=seed,
        duration_s=duration_s,
        n_uavs=n_uavs,
        n_pois=max(6, n_uavs // 2),
        params=params,
        stress=stress
    )

    runner = SimulationRunner(cfg)
    runner.run()
    metrics = runner.get_metrics()
    s_idx = compute_scalability_index(metrics)
    r_idx = compute_robustness_index(metrics)

    return {
        "seed": seed,
        "n_uavs": n_uavs,
        "coverage_pct": metrics["coverage_pct"],
        "collision_count": metrics["total_collisions"],
        "mission_completion_pct": metrics["mission_completion_pct"],
        "avg_latency_ms": metrics["avg_latency_ms"],
        "packet_delivery_ratio": metrics["packet_delivery_ratio"],
        "scalability_index": s_idx,
        "robustness_index": r_idx
    }

def run_30_trial_benchmark(
    swarm_sizes: List[int] = [5, 10, 25, 50],
    trials: int = 30,
    duration_s: float = 15.0,
    progress_callback=None,
) -> Dict[str, Any]:
    """
    Executes 30 reproducible runs for Baseline and Tuned swarms
    across the required swarm sizes under fixed random seeds.
    """
    results_matrix = {}
    completed = 0
    total = len(swarm_sizes) * trials

    for size in swarm_sizes:
        b_runs = []
        t_runs = []

        for trial_idx in range(trials):
            seed = 1000 + trial_idx
            b_run = run_single_trial(seed=seed, n_uavs=size, params=BASELINE_PARAMS, duration_s=duration_s)
            t_run = run_single_trial(seed=seed, n_uavs=size, params=TUNED_PARAMS, duration_s=duration_s)
            b_runs.append(b_run)
            t_runs.append(t_run)
            completed += 1
            if progress_callback:
                progress_callback(completed, total, size, trial_idx + 1)

        def calc_stats(run_list):
            def ms(key):
                vals = [r[key] for r in run_list]
                mean = float(np.mean(vals))
                std = float(np.std(vals))
                return round(mean, 2), round(std, 2)

            cov_m, cov_s = ms("coverage_pct")
            col_m, col_s = ms("collision_count")
            lat_m, lat_s = ms("avg_latency_ms")
            pdr_m, pdr_s = ms("packet_delivery_ratio")
            scal_m, scal_s = ms("scalability_index")
            rob_m, rob_s = ms("robustness_index")

            return {
                "coverage_mean": cov_m, "coverage_std": cov_s,
                "collisions_mean": col_m, "collisions_std": col_s,
                "latency_mean": lat_m, "latency_std": lat_s,
                "pdr_mean": pdr_m, "pdr_std": pdr_s,
                "scalability_mean": scal_m, "scalability_std": scal_s,
                "robustness_mean": rob_m, "robustness_std": rob_s,
            }

        b_stat = calc_stats(b_runs)
        t_stat = calc_stats(t_runs)

        results_matrix[str(size)] = {
            "swarm_size": size,
            "trials_conducted": trials,
            "baseline": b_stat,
            "tuned": t_stat,
            "improvements": {
                "coverage_gain_pct": compute_improvement(b_stat["coverage_mean"], t_stat["coverage_mean"], True),
                "collision_reduction_pct": compute_improvement(b_stat["collisions_mean"], t_stat["collisions_mean"], False),
                "latency_reduction_pct": compute_improvement(b_stat["latency_mean"], t_stat["latency_mean"], False),
                "scalability_gain_pct": compute_improvement(b_stat["scalability_mean"], t_stat["scalability_mean"], True),
                "robustness_gain_pct": compute_improvement(b_stat["robustness_mean"], t_stat["robustness_mean"], True),
            }
        }

    return {
        "methodology": "30-Run Seeded Monte Carlo Benchmark",
        "swarm_sizes": swarm_sizes,
        "trials_per_size": trials,
        "data": results_matrix
    }

