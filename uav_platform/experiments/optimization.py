"""
optimization.py — Grid search, Bayesian surrogate optimization, and ablation studies.
Optimizes across ALL FIVE mandatory research parameters:
  theta_1: comm_rate_hz (1 - 20 Hz)
  theta_2: neighbour_count (1 - 8)
  theta_3: safety_margin_m (0.5 - 5.0 m)
  theta_4: sensor_fusion_gain (0.05 - 1.0)
  theta_5: failsafe_threshold_pct (5 - 30 %)
"""
import itertools
import math
import random
from typing import Dict, List, Any
import numpy as np

from uav_platform.sim.runner import SimulationConfig, SimulationRunner
from uav_platform.api.metrics import compute_scalability_index, compute_robustness_index

BASELINE_PARAMS = {
    "comm_rate_hz": 20.0,
    "neighbour_count": 8,
    "safety_margin_m": 0.6,
    "sensor_fusion_gain": 1.0,
    "failsafe_threshold_pct": 5.0,
}

TUNED_PARAMS = {
    "comm_rate_hz": 5.0,
    "neighbour_count": 4,
    "safety_margin_m": 2.8,
    "sensor_fusion_gain": 0.22,
    "failsafe_threshold_pct": 15.0,
}

PARAM_BOUNDS = {
    "comm_rate_hz": (1.0, 20.0),
    "neighbour_count": (1, 8),
    "safety_margin_m": (0.5, 5.0),
    "sensor_fusion_gain": (0.05, 1.0),
    "failsafe_threshold_pct": (5.0, 30.0),
}

def evaluate_parameters(
    params: Dict[str, Any],
    n_uavs: int = 10,
    duration_s: float = 15.0,
    seed: int = 42,
    stress: Dict[str, Any] = None
) -> Dict[str, Any]:
    if stress is None:
        stress = {
            "sensor_noise": 0.5,
            "gps_dropout_prob": 0.1,
            "motor_failure_prob": 0.0,
            "packet_loss_extra": 0.05,
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
    objective = round(0.5 * s_idx + 0.5 * r_idx, 2)

    return {
        "params": params,
        "metrics": metrics,
        "scalability_index": s_idx,
        "robustness_index": r_idx,
        "objective_score": objective
    }

def run_grid_search(
    comm_rates: List[float] = [2.0, 5.0, 10.0, 20.0],
    neighbour_counts: List[int] = [2, 4, 8],
    safety_margins: List[float] = [0.6, 2.8],
    sensor_fusion_gains: List[float] = [0.22, 1.0],
    failsafe_thresholds: List[float] = [5.0, 15.0, 25.0],
    n_uavs: int = 10,
    duration_s: float = 15.0,
    seed: int = 42
) -> Dict[str, Any]:
    """
    Runs a grid search across ALL FIVE research parameters.
    """
    grid = list(itertools.product(
        comm_rates, neighbour_counts, safety_margins, sensor_fusion_gains, failsafe_thresholds
    ))
    results = []

    # Select representative stratified sample of 16 runs for fast interactive execution
    step = max(1, len(grid) // 16)
    selected_grid = grid[::step][:16]

    for cr, nc, sm, sfg, fth in selected_grid:
        p = {
            "comm_rate_hz": cr,
            "neighbour_count": nc,
            "safety_margin_m": sm,
            "sensor_fusion_gain": sfg,
            "failsafe_threshold_pct": fth
        }
        res = evaluate_parameters(p, n_uavs=n_uavs, duration_s=duration_s, seed=seed)
        results.append(res)

    results.sort(key=lambda x: x["objective_score"], reverse=True)
    best = results[0]

    return {
        "parameters_optimized": ["comm_rate_hz", "neighbour_count", "safety_margin_m", "sensor_fusion_gain", "failsafe_threshold_pct"],
        "total_combinations_evaluated": len(results),
        "best_configuration": best["params"],
        "best_objective_score": best["objective_score"],
        "best_scalability_index": best["scalability_index"],
        "best_robustness_index": best["robustness_index"],
        "results": results
    }

def run_ablation_study(
    base_params: Dict[str, Any] = None,
    n_uavs: int = 10,
    duration_s: float = 20.0,
    seed: int = 42,
    stress: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Evaluates one-parameter-at-a-time reversion against baseline for all 5 parameters.
    """
    if base_params is None:
        base_params = dict(TUNED_PARAMS)

    tuned_eval = evaluate_parameters(base_params, n_uavs, duration_s, seed, stress)

    variants = [{
        "name": "Fully Tuned Swarm (Optimal Reference)",
        "reverted_parameter": "None",
        "parameters": dict(base_params),
        "scalability_index": tuned_eval["scalability_index"],
        "robustness_index": tuned_eval["robustness_index"],
        "objective_score": tuned_eval["objective_score"],
        "impact": "Reference Benchmark"
    }]

    param_keys = ["comm_rate_hz", "neighbour_count", "safety_margin_m", "sensor_fusion_gain", "failsafe_threshold_pct"]
    labels = {
        "comm_rate_hz": f"Revert Comm Rate ({base_params['comm_rate_hz']}Hz -> {BASELINE_PARAMS['comm_rate_hz']}Hz)",
        "neighbour_count": f"Revert Neighbour Count (k={base_params['neighbour_count']} -> Broadcast)",
        "safety_margin_m": f"Revert Safety Margin ({base_params['safety_margin_m']}m -> {BASELINE_PARAMS['safety_margin_m']}m)",
        "sensor_fusion_gain": f"Revert Sensor Fusion ({base_params['sensor_fusion_gain']} -> {BASELINE_PARAMS['sensor_fusion_gain']} raw)",
        "failsafe_threshold_pct": f"Revert Failsafe ({base_params['failsafe_threshold_pct']}% -> {BASELINE_PARAMS['failsafe_threshold_pct']}%)",
    }

    for k in param_keys:
        p_revert = dict(base_params)
        p_revert[k] = BASELINE_PARAMS[k]
        ev = evaluate_parameters(p_revert, n_uavs, duration_s, seed, stress)
        s_drop = round(ev["scalability_index"] - tuned_eval["scalability_index"], 2)
        r_drop = round(ev["robustness_index"] - tuned_eval["robustness_index"], 2)
        variants.append({
            "name": labels[k],
            "reverted_parameter": k,
            "parameters": p_revert,
            "scalability_index": ev["scalability_index"],
            "robustness_index": ev["robustness_index"],
            "objective_score": ev["objective_score"],
            "impact": f"ΔS: {s_drop}%, ΔR: {r_drop}%"
        })

    base_eval = evaluate_parameters(BASELINE_PARAMS, n_uavs, duration_s, seed, stress)
    s_gain = round(tuned_eval["scalability_index"] - base_eval["scalability_index"], 2)
    r_gain = round(tuned_eval["robustness_index"] - base_eval["robustness_index"], 2)
    variants.append({
        "name": "Default Baseline Swarm (All Defaults)",
        "reverted_parameter": "All 5 Parameters",
        "parameters": dict(BASELINE_PARAMS),
        "scalability_index": base_eval["scalability_index"],
        "robustness_index": base_eval["robustness_index"],
        "objective_score": base_eval["objective_score"],
        "impact": f"Net Gap: +{s_gain}% Scalability, +{r_gain}% Robustness"
    })

    return {
        "n_uavs": n_uavs,
        "seed": seed,
        "ablation_variants": variants
    }

def run_bayesian_optimization(
    n_iterations: int = 10,
    n_uavs: int = 10,
    duration_s: float = 15.0,
    seed: int = 42
) -> Dict[str, Any]:
    """
    Gaussian Process / Upper Confidence Bound (UCB) surrogate optimization
    across all 5 parameter dimensions.
    """
    rng = random.Random(seed)
    history = []
    observed_X = []
    observed_y = []

    def normalize_params(p):
        return [
            (p["comm_rate_hz"] - 1.0) / 19.0,
            (p["neighbour_count"] - 1) / 7.0,
            (p["safety_margin_m"] - 0.5) / 4.5,
            (p["sensor_fusion_gain"] - 0.05) / 0.95,
            (p["failsafe_threshold_pct"] - 5.0) / 25.0
        ]

    def denormalize_params(norm_vec):
        return {
            "comm_rate_hz": round(1.0 + norm_vec[0] * 19.0, 1),
            "neighbour_count": max(1, min(8, int(round(1.0 + norm_vec[1] * 7.0)))),
            "safety_margin_m": round(0.5 + norm_vec[2] * 4.5, 1),
            "sensor_fusion_gain": round(0.05 + norm_vec[3] * 0.95, 2),
            "failsafe_threshold_pct": round(5.0 + norm_vec[4] * 25.0, 1)
        }

    # Surrogate kernel distance
    def rbf_surrogate(query_norm, X, y, length_scale=0.3):
        if not X:
            return 50.0, 25.0
        dists = [sum((a - b)**2 for a, b in zip(query_norm, x_obs)) for x_obs in X]
        weights = [math.exp(-d / (2 * length_scale**2)) for d in dists]
        total_w = sum(weights) + 1e-6
        mean = sum(w * y_val for w, y_val in zip(weights, y)) / total_w
        # Uncertainty is higher far from observed points
        min_dist = math.sqrt(min(dists))
        std = 30.0 * (1.0 - math.exp(-min_dist / length_scale))
        return mean, std

    best_score = -float('inf')
    best_params = None

    for i in range(n_iterations):
        if i == 0:
            params = dict(BASELINE_PARAMS)
        elif i == 1:
            params = dict(TUNED_PARAMS)
        else:
            # UCB acquisition: sample 25 candidates and pick argmax(mu + 1.8 * sigma)
            best_acq = -float('inf')
            best_cand = None
            for _ in range(25):
                cand_norm = [rng.random() for _ in range(5)]
                mu, sigma = rbf_surrogate(cand_norm, observed_X, observed_y)
                ucb = mu + 1.8 * sigma
                if ucb > best_acq:
                    best_acq = ucb
                    best_cand = cand_norm
            params = denormalize_params(best_cand)

        ev = evaluate_parameters(params, n_uavs=n_uavs, duration_s=duration_s, seed=seed)
        observed_X.append(normalize_params(params))
        observed_y.append(ev["objective_score"])

        history.append({
            "iteration": i + 1,
            "params": params,
            "scalability_index": ev["scalability_index"],
            "robustness_index": ev["robustness_index"],
            "objective_score": ev["objective_score"]
        })

        if ev["objective_score"] > best_score:
            best_score = ev["objective_score"]
            best_params = params

    return {
        "parameters_optimized": ["comm_rate_hz", "neighbour_count", "safety_margin_m", "sensor_fusion_gain", "failsafe_threshold_pct"],
        "iterations": n_iterations,
        "best_configuration": best_params,
        "best_objective_score": best_score,
        "convergence_trace": history
    }
