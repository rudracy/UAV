"""
metrics.py — Mathematical formulation of Scalability and Robustness Indices,
and rigorous percentage improvement calculations.

PROVABLE BOUNDS:
Both S_idx and R_idx are strictly bounded convex combinations of normalized sub-metrics in [0, 100]
with non-negative weights summing to exactly 1.0. Therefore:
  0.0 <= S_idx <= 100.0
  0.0 <= R_idx <= 100.0
for all possible simulation inputs.

All components are strictly derived from actual measured simulation outputs:
- S_idx: Coverage scaling retention, throughput/latency retention, spatial conflict safety.
- R_idx: Mission completion, vehicle survivability/collision penalty, link delivery (PDR), and measured battery reserve.
"""
import math
from typing import Dict, Any

# Scalability Index Weights (Sum = 1.0)
W_SCAL_COVERAGE = 0.35      # Coverage scaling efficiency per swarm scale
W_SCAL_THROUGHPUT = 0.35    # Communication latency & PDR retention under density
W_SCAL_SAFETY = 0.30        # Spatial separation & collision-free scaling

# Robustness Index Weights (Sum = 1.0)
W_ROB_MISSION = 0.35        # Actual mission tasks successfully completed
W_ROB_SURVIVAL = 0.30       # Measured airframe survivability minus collision penalties
W_ROB_CONNECTIVITY = 0.20   # Measured Packet Delivery Ratio under multi-modal stress
W_ROB_BATTERY = 0.15        # Measured average battery reserve / safe energy margin

LATENCY_MAX_MS = 250.0      # Maximum acceptable BVLOS C2 latency ceiling in ms

def compute_scalability_index(metrics: Dict[str, Any]) -> float:
    """
    Computes Scalability Index S_idx strictly bounded in [0, 100].
    
    Formula:
      S_idx = 0.35 * C_scale + 0.35 * T_ret + 0.30 * S_safe
      where:
        - C_scale = clamp(100 * Coverage% / ExpectedCoverage(N), 0, 100)
          where ExpectedCoverage(N) = min(35.0, 3.2 * sqrt(N))
          [Justification: In a 160,000 m² arena over 15s flight at 15 m/s with 
           camera footprint 25m x 18.75m, analytical geometry bounds clustered 
           coverage to ~25-35% maximum. ExpectedCoverage models ideal sub-linear scaling].
        - T_ret = clamp(100 * (1 - Latency / 250ms) * PDR, 0, 100)
          [Quantifies RF throughput retention against spectrum saturation].
        - S_safe = clamp(100 * (1 - Collisions / max(1, 0.5 * N)), 0, 100)
          [Normalizes spatial conflict rate against vehicle population].
    """
    n_uavs = max(1, int(metrics.get("n_uavs", 10)))
    coverage = float(metrics.get("coverage_pct", 0.0))
    collisions = int(metrics.get("total_collisions", 0))
    latency = float(metrics.get("avg_latency_ms", 0.0))
    pdr = float(metrics.get("packet_delivery_ratio", 1.0))

    # 1. Coverage scaling retention in [0, 100]
    expected_cov = max(1.0, min(35.0, 3.2 * math.sqrt(n_uavs)))
    c_scale = max(0.0, min(100.0, (coverage / expected_cov) * 100.0))

    # 2. Network throughput & latency retention under density in [0, 100]
    lat_factor = max(0.0, min(1.0, 1.0 - (latency / LATENCY_MAX_MS)))
    pdr_factor = max(0.0, min(1.0, pdr))
    t_ret = max(0.0, min(100.0, 100.0 * lat_factor * pdr_factor))

    # 3. Spatial collision-free safety retention in [0, 100]
    col_norm_denom = max(1.0, 0.5 * n_uavs)
    col_ratio = collisions / col_norm_denom
    s_safe = max(0.0, min(100.0, 100.0 * max(0.0, 1.0 - col_ratio)))

    s_idx = (W_SCAL_COVERAGE * c_scale) + (W_SCAL_THROUGHPUT * t_ret) + (W_SCAL_SAFETY * s_safe)
    return round(max(0.0, min(100.0, s_idx)), 2)


def compute_robustness_index(metrics: Dict[str, Any]) -> float:
    """
    Computes Robustness Index R_idx strictly bounded in [0, 100].
    
    Formula:
      R_idx = 0.35 * M_meas + 0.30 * U_surv + 0.20 * L_pdr + 0.15 * B_res
      where:
        - M_meas = clamp(mission_completion_pct, 0, 100)
          [Actual percentage of disaster survey tasks completed].
        - U_surv = clamp(100 * (N_alive / N) * max(0, 1 - Collisions / N), 0, 100)
          [Actual surviving airframes penalized by physical collision damage].
        - L_pdr = clamp(100 * PDR, 0, 100)
          [Actual measured Packet Delivery Ratio under multi-hop RF stress].
        - B_res = clamp(avg_battery_pct, 0, 100)
          [Actual measured swarm battery reserve; prevents dead-stick crashes].
    """
    n_uavs = max(1, int(metrics.get("n_uavs", 10)))
    alive = int(metrics.get("alive_uavs", n_uavs))
    completion = float(metrics.get("mission_completion_pct", 0.0))
    pdr = float(metrics.get("packet_delivery_ratio", 1.0))
    collisions = int(metrics.get("total_collisions", 0))
    battery = float(metrics.get("avg_battery_pct", 100.0))

    # 1. Measured Mission Completion in [0, 100]
    m_meas = max(0.0, min(100.0, completion))

    # 2. Measured Vehicle Survivability & Airframe Integrity in [0, 100]
    surv_ratio = max(0.0, min(1.0, alive / float(n_uavs)))
    safe_ratio = max(0.0, min(1.0, 1.0 - (collisions / float(n_uavs))))
    u_surv = max(0.0, min(100.0, 100.0 * surv_ratio * safe_ratio))

    # 3. Measured Communication Reliability under stress in [0, 100]
    l_pdr = max(0.0, min(100.0, 100.0 * max(0.0, min(1.0, pdr))))

    # 4. Measured Swarm Battery Reserve & Safe Return Margin in [0, 100]
    b_res = max(0.0, min(100.0, battery))

    r_idx = (W_ROB_MISSION * m_meas) + (W_ROB_SURVIVAL * u_surv) + (W_ROB_CONNECTIVITY * l_pdr) + (W_ROB_BATTERY * b_res)
    return round(max(0.0, min(100.0, r_idx)), 2)


def compute_pp_diff(baseline: float, tuned: float) -> float:
    """
    Computes absolute percentage-point difference (Δ pp):
      Δ pp = Tuned - Baseline
    """
    return round(tuned - baseline, 2)


def compute_improvement(baseline: float, tuned: float, higher_is_better: bool = True) -> float:
    """
    Computes relative percentage improvement (% change):
      Higher-is-better: ((Tuned - Baseline) / |Baseline|) * 100
      Lower-is-better : ((Baseline - Tuned) / |Baseline|) * 100
    
    Zero-denominator edge cases are handled safely.
    """
    if abs(baseline) < 1e-6:
        if abs(tuned) < 1e-6:
            return 0.0
        return 100.0 if higher_is_better else -100.0

    if higher_is_better:
        return round(((tuned - baseline) / abs(baseline)) * 100.0, 2)
    else:
        return round(((baseline - tuned) / abs(baseline)) * 100.0, 2)
