#!/usr/bin/env python3
"""
Executes the full 240-run Monte Carlo empirical benchmark for the UAV Swarm Research Platform.
Runs 30 trials across 4 swarm sizes (N = 5, 10, 25, 50) for both Baseline and Tuned configurations.
Uses multiprocessing across all CPU cores for fast execution.
Outputs data/monte_carlo_runs.csv and data/benchmark_summary.json.
"""

import concurrent.futures
import csv
import json
import os
import sys
import numpy as np

# Ensure project root is in python path
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from uav_platform.sim.runner import SimulationConfig, SimulationRunner
from uav_platform.api.metrics import compute_scalability_index, compute_robustness_index, compute_improvement
from uav_platform.api.simulation_service import BASELINE_PARAMS, TUNED_PARAMS

SWARM_SIZES = [5, 10, 25, 50]
TRIALS = 30
DURATION_S = 15.0

STRESS = {
    "sensor_noise": 0.5,
    "gps_dropout_prob": 0.10,
    "motor_failure_prob": 0.005,
    "packet_loss_extra": 0.05,
    "outage_uav_ids": []
}

def execute_single_run(args):
    run_id, size, config_name, seed = args
    params = BASELINE_PARAMS if config_name == "Baseline" else TUNED_PARAMS
    cfg = SimulationConfig(
        seed=seed,
        duration_s=DURATION_S,
        n_uavs=size,
        n_pois=max(6, size // 2),
        params=params,
        stress=STRESS
    )
    runner = SimulationRunner(cfg)
    runner.run()
    m = runner.get_metrics()
    s_idx = compute_scalability_index(m)
    r_idx = compute_robustness_index(m)

    return {
        "run_id": run_id,
        "swarm_size": size,
        "configuration": config_name,
        "coverage_pct": round(m["coverage_pct"], 2),
        "collisions_count": int(m["total_collisions"]),
        "alive_agents": int(m["alive_uavs"]),
        "failed_agents": int(m["failed_uavs"]),
        "mission_completion_pct": round(m["mission_completion_pct"], 2),
        "avg_latency_ms": round(m["avg_latency_ms"], 2),
        "queuing_delay_ms": round(m["queuing_delay_ms"], 2),
        "information_freshness_ms": round(m["information_freshness_ms"], 1),
        "packet_delivery_ratio": round(m["packet_delivery_ratio"], 4),
        "avg_battery_pct": round(m["avg_battery_pct"], 2),
        "scalability_index": round(s_idx, 2),
        "robustness_index": round(r_idx, 2),
        "seed": seed
    }

def main():
    print(f"Preparing 240 Monte Carlo experiments across {os.cpu_count()} CPU cores...")
    tasks = []
    run_id = 1
    for size in SWARM_SIZES:
        for seed_idx in range(TRIALS):
            seed = 1000 + seed_idx
            tasks.append((run_id, size, "Baseline", seed))
            run_id += 1
            tasks.append((run_id, size, "Tuned", seed))
            run_id += 1

    print(f"Total task list length: {len(tasks)}")

    records = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=os.cpu_count()) as executor:
        futures = {executor.submit(execute_single_run, task): task for task in tasks}
        for future in concurrent.futures.as_completed(futures):
            res = future.result()
            records.append(res)
            if len(records) % 40 == 0 or len(records) == len(tasks):
                print(f"Progress: {len(records)}/{len(tasks)} runs completed...")

    # Sort records by run_id
    records.sort(key=lambda r: r["run_id"])

    # Aggregation
    stats_summary = {}
    for size in SWARM_SIZES:
        b_runs = [r for r in records if r["swarm_size"] == size and r["configuration"] == "Baseline"]
        t_runs = [r for r in records if r["swarm_size"] == size and r["configuration"] == "Tuned"]

        def aggregate(run_list):
            def m_s(k):
                vals = [r[k] for r in run_list]
                return round(float(np.mean(vals)), 2), round(float(np.std(vals)), 2)
            cov_m, cov_s = m_s("coverage_pct")
            col_m, col_s = m_s("collisions_count")
            lat_m, lat_s = m_s("avg_latency_ms")
            que_m, que_s = m_s("queuing_delay_ms")
            frs_m, frs_s = m_s("information_freshness_ms")
            pdr_m, pdr_s = m_s("packet_delivery_ratio")
            scal_m, scal_s = m_s("scalability_index")
            rob_m, rob_s = m_s("robustness_index")
            comp_m, comp_s = m_s("mission_completion_pct")
            return {
                "coverage_mean": cov_m, "coverage_std": cov_s,
                "collisions_mean": col_m, "collisions_std": col_s,
                "latency_mean": lat_m, "latency_std": lat_s,
                "queuing_mean": que_m, "queuing_std": que_s,
                "freshness_mean": frs_m, "freshness_std": frs_s,
                "pdr_mean": pdr_m, "pdr_std": pdr_s,
                "scalability_mean": scal_m, "scalability_std": scal_s,
                "robustness_mean": rob_m, "robustness_std": rob_s,
                "mission_completion_mean": comp_m, "mission_completion_std": comp_s
            }

        b_stat = aggregate(b_runs)
        t_stat = aggregate(t_runs)

        stats_summary[str(size)] = {
            "swarm_size": size,
            "trials": TRIALS,
            "baseline": b_stat,
            "tuned": t_stat,
            "improvements": {
                "coverage_gain_pct": compute_improvement(b_stat["coverage_mean"], t_stat["coverage_mean"], True),
                "collision_reduction_pct": compute_improvement(b_stat["collisions_mean"], t_stat["collisions_mean"], False),
                "latency_reduction_pct": compute_improvement(b_stat["latency_mean"], t_stat["latency_mean"], False),
                "pdr_gain_pct": compute_improvement(b_stat["pdr_mean"], t_stat["pdr_mean"], True),
                "scalability_gain_pct": compute_improvement(b_stat["scalability_mean"], t_stat["scalability_mean"], True),
                "robustness_gain_pct": compute_improvement(b_stat["robustness_mean"], t_stat["robustness_mean"], True)
            }
        }
        print(f"\n--- Swarm Size N={size} Summary ---")
        print(f"  Scalability Index: Baseline {b_stat['scalability_mean']} ± {b_stat['scalability_std']} -> Tuned {t_stat['scalability_mean']} ± {t_stat['scalability_std']} (Gain: +{stats_summary[str(size)]['improvements']['scalability_gain_pct']}%)")
        print(f"  Robustness Index:  Baseline {b_stat['robustness_mean']} ± {b_stat['robustness_std']} -> Tuned {t_stat['robustness_mean']} ± {t_stat['robustness_std']} (Gain: +{stats_summary[str(size)]['improvements']['robustness_gain_pct']}%)")
        print(f"  Collisions Mean:   Baseline {b_stat['collisions_mean']} ± {b_stat['collisions_std']} -> Tuned {t_stat['collisions_mean']} ± {t_stat['collisions_std']} (Reduction: {stats_summary[str(size)]['improvements']['collision_reduction_pct']}%)")
        print(f"  PDR Mean:          Baseline {b_stat['pdr_mean']} -> Tuned {t_stat['pdr_mean']} (Gain: +{stats_summary[str(size)]['improvements']['pdr_gain_pct']}%)")

    # Overall cross-scale aggregate gains
    overall_scal_b = np.mean([stats_summary[str(s)]["baseline"]["scalability_mean"] for s in SWARM_SIZES])
    overall_scal_t = np.mean([stats_summary[str(s)]["tuned"]["scalability_mean"] for s in SWARM_SIZES])
    overall_rob_b = np.mean([stats_summary[str(s)]["baseline"]["robustness_mean"] for s in SWARM_SIZES])
    overall_rob_t = np.mean([stats_summary[str(s)]["tuned"]["robustness_mean"] for s in SWARM_SIZES])
    overall_col_b = np.mean([stats_summary[str(s)]["baseline"]["collisions_mean"] for s in SWARM_SIZES])
    overall_col_t = np.mean([stats_summary[str(s)]["tuned"]["collisions_mean"] for s in SWARM_SIZES])

    overall_improvements = {
        "scalability_gain_pct": compute_improvement(overall_scal_b, overall_scal_t, True),
        "robustness_gain_pct": compute_improvement(overall_rob_b, overall_rob_t, True),
        "collision_reduction_pct": compute_improvement(overall_col_b, overall_col_t, False),
    }

    print("\n================ OVERALL CROSS-SCALE BENCHMARK RESULTS ================")
    print(f"Overall Scalability Index: {overall_scal_b:.2f} -> {overall_scal_t:.2f} (+{overall_improvements['scalability_gain_pct']}%)")
    print(f"Overall Robustness Index:  {overall_rob_b:.2f} -> {overall_rob_t:.2f} (+{overall_improvements['robustness_gain_pct']}%)")
    print(f"Overall Collision Count:   {overall_col_b:.2f} -> {overall_col_t:.2f} ({overall_improvements['collision_reduction_pct']}% reduction)")
    print("=======================================================================")

    # Write CSV
    data_dir = os.path.join(ROOT_DIR, "data")
    os.makedirs(data_dir, exist_ok=True)
    csv_file = os.path.join(data_dir, "monte_carlo_runs.csv")
    with open(csv_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)

    # Write JSON
    json_file = os.path.join(data_dir, "benchmark_summary.json")
    with open(json_file, "w", encoding="utf-8") as f:
        json.dump({
            "metadata": {
                "experiment": "30-Trial Monte Carlo Benchmark (240 Runs Total)",
                "swarm_sizes": SWARM_SIZES,
                "trials_per_size": TRIALS,
                "total_runs": len(records),
                "duration_per_run_s": DURATION_S,
                "stress_conditions": STRESS,
                "baseline_params": BASELINE_PARAMS,
                "tuned_params": TUNED_PARAMS,
                "overall_improvements": overall_improvements
            },
            "by_swarm_size": stats_summary
        }, f, indent=2)

    print(f"\nSaved CSV: {csv_file}")
    print(f"Saved JSON: {json_file}")

if __name__ == "__main__":
    main()

