"""
simulation_service.py — Orchestrates single/batch simulations, frame serialization,
and WebSocket frame generation.
"""
import asyncio
import threading
import time
from typing import List, Dict, Any, Tuple, AsyncIterator, Callable
from uav_platform.sim.runner import SimulationConfig, SimulationRunner, SimulationState
from uav_platform.api.models import SimulationRequest, SimulationFrame, UAVTelemetry, MetricsResult, ResearchParams, StressConfig
from uav_platform.api.metrics import compute_scalability_index, compute_robustness_index
from uav_platform.api.database import db

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

class SimulationService:
    @staticmethod
    def get_baseline_params() -> Dict[str, Any]:
        return dict(BASELINE_PARAMS)

    @staticmethod
    def get_tuned_params() -> Dict[str, Any]:
        return dict(TUNED_PARAMS)

    @staticmethod
    def convert_state_to_frame(state: SimulationState) -> SimulationFrame:
        uav_telemetries = []
        for u in state.uavs:
            uav_telemetries.append(UAVTelemetry(
                id=int(u.uav_id),
                x=round(float(u.x), 3),
                y=round(float(u.y), 3),
                z=round(float(u.z), 3),
                vx=round(float(u.vx), 3),
                vy=round(float(u.vy), 3),
                vz=round(float(u.vz), 3),
                yaw=round(float(u.yaw), 3),
                battery_pct=round(float(u.battery_pct), 2),
                alive=bool(u.alive),
                role=str(getattr(u, "role", "survey")),
                gps_ok=bool(getattr(u, "gps_ok", True)),
                task_id=getattr(u, "assigned_poi_id", None),
                comm_links=list(getattr(u, "comm_links", []))
            ))

        poi_dicts = []
        for p in state.pois:
            poi_dicts.append({
                "id": int(p.id),
                "x": round(float(p.x), 3),
                "y": round(float(p.y), 3),
                "priority": int(p.priority),
                "surveyed": bool(p.surveyed),
                "survey_progress_pct": round(float(getattr(p, "survey_progress_pct", 0.0)), 1),
                "discovered_by_uav": getattr(p, "discovered_by_uav", -1)
            })

        return SimulationFrame(
            t=round(float(state.t), 2),
            uavs=uav_telemetries,
            pois=poi_dicts,
            coverage_pct=round(float(state.coverage_pct) * 100.0, 2),
            collisions=int(state.collisions),
            alive_uavs=int(state.alive_uavs),
            connectivity_ratio=round(float(state.comm_graph.connectivity_ratio()) if state.comm_graph else 0.0, 4),
            avg_latency_ms=round(float(state.comm_graph.avg_latency_ms()) if state.comm_graph else 0.0, 2),
            comm_graph={"links": [
                {"uav_a": int(link.uav_a), "uav_b": int(link.uav_b),
                 "active": bool(link.active), "packet_loss_pct": round(float(link.packet_loss_pct), 2)}
                for link in (state.comm_graph._links.values() if state.comm_graph else [])
            ]},
            events=list(state.events)
        )

    @classmethod
    def execute_simulation_sync(cls, req: SimulationRequest) -> Tuple[str, List[SimulationFrame], MetricsResult]:
        """Runs the simulation synchronously in the current thread."""
        params_dict = req.params.model_dump()
        stress_dict = req.stress.model_dump()
        stress_dict["sensor_noise"] = 0.1 * float(stress_dict.get("sensor_noise_multiplier", 1.0))

        sim_cfg = SimulationConfig(
            seed=req.seed,
            duration_s=req.duration_s,
            n_uavs=req.n_uavs,
            n_pois=req.n_pois,
            params=params_dict,
            stress=stress_dict
        )

        runner = SimulationRunner(sim_cfg)
        raw_states = runner.run()
        raw_metrics = runner.get_metrics()

        # Compute research indices
        s_idx = compute_scalability_index(raw_metrics)
        r_idx = compute_robustness_index(raw_metrics)

        metrics = MetricsResult(
            duration_s=raw_metrics["duration_s"],
            n_uavs=raw_metrics["n_uavs"],
            alive_uavs=raw_metrics["alive_uavs"],
            failed_uavs=raw_metrics["failed_uavs"],
            n_pois=raw_metrics["n_pois"],
            surveyed_pois=raw_metrics["surveyed_pois"],
            mission_completion_pct=raw_metrics["mission_completion_pct"],
            coverage_pct=raw_metrics["coverage_pct"],
            collision_count=raw_metrics["total_collisions"],
            avg_latency_ms=raw_metrics["avg_latency_ms"],
            packet_delivery_ratio=raw_metrics["packet_delivery_ratio"],
            avg_battery_pct=raw_metrics["avg_battery_pct"],
            scalability_index=s_idx,
            robustness_index=r_idx,
            seed=req.seed,
            params=params_dict,
            stress=stress_dict
        )

        frames = [cls.convert_state_to_frame(s) for s in raw_states]

        # Save to database
        record = {
            "config_name": req.config_name,
            "params": params_dict,
            "stress": stress_dict,
            "n_uavs": req.n_uavs,
            "seed": req.seed,
            "status": "completed",
            "duration_s": req.duration_s,
            "metrics": metrics.model_dump()
        }
        exp_id = db.save_experiment(record)

        return exp_id, frames, metrics

    @classmethod
    def execute_simulation_stream_sync(
        cls,
        req: SimulationRequest,
        on_frame: Callable[[SimulationFrame, float], None],
        stop_signal: threading.Event = None,
        pause_signal: threading.Event = None,
        continuous: bool = False,
        runtime_controls: dict = None,
    ) -> Tuple[str, MetricsResult]:
        """Run a simulation and stream snapshots; continuous missions stop on signal."""
        params_dict = req.params.model_dump()
        stress_dict = req.stress.model_dump()
        stress_dict["sensor_noise"] = 0.1 * float(stress_dict.get("sensor_noise_multiplier", 1.0))
        runner = SimulationRunner(SimulationConfig(
            seed=req.seed, duration_s=req.duration_s, n_uavs=req.n_uavs, n_pois=req.n_pois,
            params=params_dict, stress=stress_dict
        ))
        if continuous:
            stop_signal = stop_signal or threading.Event()
            pause_signal = pause_signal or threading.Event()
            tick_seconds = 0.1
            snapshot_ticks = 5  # 2 Hz telemetry, same cadence as the finite runner.
            tick = 0
            next_tick_at = time.monotonic()
            runtime_version = -1
            while not stop_signal.is_set():
                if pause_signal.is_set():
                    stop_signal.wait(0.05)
                    next_tick_at = time.monotonic() + tick_seconds
                    continue
                delay = next_tick_at - time.monotonic()
                if delay > 0 and stop_signal.wait(delay):
                    break
                if stop_signal.is_set():
                    break
                if runtime_controls:
                    with runtime_controls["lock"]:
                        version = runtime_controls["version"]
                        settings = dict(runtime_controls["values"])
                    if version != runtime_version:
                        runner.apply_runtime_settings(settings)
                        runtime_version = version
                state = runner.step()
                tick += 1
                next_tick_at += tick_seconds
                if tick % snapshot_ticks == 0:
                    window = max(1.0, float(req.duration_s))
                    progress = (state.t % window) / window
                    on_frame(cls.convert_state_to_frame(state), progress)
        else:
            runner.run(lambda state, progress: on_frame(cls.convert_state_to_frame(state), progress))
        raw = runner.get_metrics()
        metrics = MetricsResult(
            duration_s=raw["duration_s"], n_uavs=raw["n_uavs"], alive_uavs=raw["alive_uavs"],
            failed_uavs=raw["failed_uavs"], n_pois=raw["n_pois"], surveyed_pois=raw["surveyed_pois"],
            mission_completion_pct=raw["mission_completion_pct"], coverage_pct=raw["coverage_pct"],
            collision_count=raw["total_collisions"], avg_latency_ms=raw["avg_latency_ms"],
            packet_delivery_ratio=raw["packet_delivery_ratio"], avg_battery_pct=raw["avg_battery_pct"],
            scalability_index=compute_scalability_index(raw), robustness_index=compute_robustness_index(raw),
            seed=req.seed, params=params_dict, stress=stress_dict
        )
        exp_id = db.save_experiment({
            "config_name": req.config_name, "params": params_dict, "stress": stress_dict,
            "n_uavs": req.n_uavs, "seed": req.seed, "status": "completed",
            "duration_s": metrics.duration_s, "metrics": metrics.model_dump()
        })
        return exp_id, metrics

    @classmethod
    async def run_simulation(cls, req: SimulationRequest) -> Tuple[str, List[SimulationFrame], MetricsResult]:
        return await asyncio.to_thread(cls.execute_simulation_sync, req)
