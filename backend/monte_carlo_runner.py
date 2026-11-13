"""
Monte Carlo Simulation Runner
Executes multi-run statistical simulations dynamically on the Python backend.
Calculates statistical distributions (mean, std dev) and scalability/robustness gains.
"""

import random
import math
from typing import Dict, Any, List
from backend.swarm_simulator import SwarmSimulator

class MonteCarloRunner:
    def __init__(self):
        pass

    def run_experiment(
        self,
        swarm_size: int = 10,
        runs: int = 10,
        noise_mult: float = 1.0,
        gps_dropout: bool = False,
        packet_loss: float = 0.0,
        inject_fail: bool = False
    ) -> Dict[str, Any]:
        """Runs parallel or sequential Monte Carlo trials for Baseline vs Tuned configurations."""
        b_results = []
        t_results = []

        # Run Baseline trials
        for r in range(runs):
            sim_b = SwarmSimulator(is_tuned=False)
            sim_b.reset_swarm(swarm_size)
            if inject_fail:
                sim_b.inject_failure(2)
            
            # Step simulation forward for 100 ticks (dt = 0.1)
            for _ in range(80):
                sim_b.step(0.1, noise_mult=noise_mult, gps_dropout=gps_dropout, packet_loss=packet_loss)
            
            b_results.append(sim_b.get_state())

        # Run Tuned trials
        for r in range(runs):
            sim_t = SwarmSimulator(is_tuned=True)
            sim_t.reset_swarm(swarm_size)
            if inject_fail:
                sim_t.inject_failure(2)
            
            for _ in range(80):
                sim_t.step(0.1, noise_mult=noise_mult, gps_dropout=gps_dropout, packet_loss=packet_loss)
            
            t_results.append(sim_t.get_state())

        # Aggregate metrics
        def stats_for(results: List[Dict[str, Any]]) -> Dict[str, Any]:
            covs = [x["coverage_pct"] for x in results]
            cols = [x["collisions"] for x in results]
            scals = [x["scalability_index"] for x in results]
            robs = [x["robustness_index"] for x in results]

            def calc_mean_std(arr):
                m = sum(arr) / len(arr)
                std = math.sqrt(sum((v - m) ** 2 for v in arr) / len(arr))
                return round(m, 1), round(std, 1)

            cov_m, cov_s = calc_mean_std(covs)
            col_m, col_s = calc_mean_std(cols)
            scal_m, scal_s = calc_mean_std(scals)
            rob_m, rob_s = calc_mean_std(robs)

            return {
                "coverage_mean": cov_m,
                "coverage_std": cov_s,
                "collisions_mean": col_m,
                "collisions_std": col_s,
                "scalability_mean": scal_m,
                "scalability_std": scal_s,
                "robustness_mean": rob_m,
                "robustness_std": rob_s,
                "latency_mean": round(sum(x["network_latency_ms"] for x in results) / len(results), 1),
                "comm_overhead_mean": round(sum(x["comm_overhead_kbps"] for x in results) / len(results), 1)
            }

        b_stat = stats_for(b_results)
        t_stat = stats_for(t_results)

        scal_gain = round(((t_stat["scalability_mean"] - b_stat["scalability_mean"]) / max(0.1, b_stat["scalability_mean"])) * 100.0, 1)
        rob_gain = round(((t_stat["robustness_mean"] - b_stat["robustness_mean"]) / max(0.1, b_stat["robustness_mean"])) * 100.0, 1)

        return {
            "parameters": {
                "swarm_size": swarm_size,
                "runs_executed": runs,
                "sensor_noise_multiplier": noise_mult,
                "gps_dropout": gps_dropout,
                "packet_loss_rate": packet_loss,
                "failure_injected": inject_fail
            },
            "baseline": b_stat,
            "tuned": t_stat,
            "improvements": {
                "scalability_gain_pct": scal_gain,
                "robustness_gain_pct": rob_gain
            }
        }

