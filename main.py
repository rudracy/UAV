"""
main.py — FastAPI application entry point.
Hosts REST APIs, WebSocket live simulation streaming, and static frontend assets.
"""
import asyncio
from contextlib import suppress
import json
import os
import sys
import threading
from typing import Dict, Any

# Ensure project root is on Python path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from uav_platform.api.routes.simulation import router as sim_router
from uav_platform.api.routes.experiments import router as exp_router
from uav_platform.api.routes.analytics import router as analytics_router
from uav_platform.api.routes.optimization import router as opt_router
from uav_platform.api.simulation_service import SimulationService
from uav_platform.api.models import SimulationRequest, ResearchParams
from uav_platform.experiments.batch_runner import run_30_trial_benchmark

_benchmark_lock = threading.Lock()
_benchmark_status = {"state": "idle", "completed": 0, "total": 0, "swarm_size": None, "trial": 0, "error": None}

app = FastAPI(
    title="UAV Swarm Scalability & Robustness Research Platform",
    description="Deterministic BVLOS Multi-UAV Swarm Simulation, Parameter Optimization, and Analytics API",
    version="2.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register REST Routers
app.include_router(sim_router)
app.include_router(exp_router)
app.include_router(analytics_router)
app.include_router(opt_router)

# The dual-arena comparison is the primary application screen.
@app.get("/dual-arena")
def serve_dual_arena():
    return FileResponse(os.path.join(PROJECT_ROOT, "web", "index.html"))

@app.get("/api/health")
def health():
    return {
        "status": "healthy",
        "service": "UAV Swarm Research Platform",
        "version": "2.0.0",
        "engine": "UAV-X Resilient BVLOS Core"
    }

@app.post("/api/benchmark/batch")
async def trigger_batch_benchmark(trials: int = 5):
    """Triggers multi-run benchmark across swarm sizes (5, 10, 25, 50)."""
    trials = max(1, min(30, int(trials)))
    with _benchmark_lock:
        _benchmark_status.update(state="running", completed=0, total=4 * trials,
                                 swarm_size=None, trial=0, error=None)

    def report_progress(completed, total, swarm_size, trial):
        with _benchmark_lock:
            _benchmark_status.update(completed=completed, total=total,
                                     swarm_size=swarm_size, trial=trial)

    try:
        result = await asyncio.to_thread(
            run_30_trial_benchmark, [5, 10, 25, 50], trials, 15.0, report_progress
        )
        with _benchmark_lock:
            _benchmark_status.update(state="complete", completed=4 * trials,
                                     total=4 * trials, swarm_size=None, trial=0)
        return result
    except Exception as exc:
        with _benchmark_lock:
            _benchmark_status.update(state="error", error=str(exc))
        raise

@app.get("/api/benchmark/status")
def get_batch_benchmark_status():
    with _benchmark_lock:
        return dict(_benchmark_status)

# -----------------------------------------------------------------------------
# WebSocket Live Simulation Stream
# -----------------------------------------------------------------------------
@app.websocket("/ws/simulation")
async def simulation_websocket(websocket: WebSocket):
    await websocket.accept()
    stop_event = asyncio.Event()
    pause_event = asyncio.Event()
    worker_stop_signal = threading.Event()
    worker_pause_signal = threading.Event()
    runtime_controls = {"lock": threading.Lock(), "version": 0, "values": {}}
    running_task = None

    async def run_single(req):
        loop = asyncio.get_running_loop()
        queue = asyncio.Queue()
        def publish(frame, progress):
            loop.call_soon_threadsafe(queue.put_nowait, (frame, progress))
        worker = asyncio.create_task(asyncio.to_thread(
            SimulationService.execute_simulation_stream_sync, req, publish
        ))
        while not worker.done():
            if stop_event.is_set():
                return
            if pause_event.is_set():
                await asyncio.sleep(0.1)
                continue
            try:
                frame, progress = await asyncio.wait_for(queue.get(), timeout=0.1)
                await websocket.send_json({"type":"frame", "frame":frame.model_dump(), "progress":round(progress*100,1)})
                await websocket.send_json({"type":"progress", "progress":round(progress*100,1), "sim_time_s":frame.t})
            except asyncio.TimeoutError:
                pass
        exp_id, metrics = await worker
        if not stop_event.is_set():
            await websocket.send_json({"type":"completed", "experiment_id":exp_id, "metrics":metrics.model_dump()})

    async def run_comparison(req, continuous=True):
        loop = asyncio.get_running_loop()
        queue = asyncio.Queue()
        pending = {"baseline":{}, "tuned":{}}
        pair_lock = threading.Lock()
        def make_runner(run_req):
            name = run_req.config_name
            def on_frame(frame, progress):
                frame_index = max(1, int(round(frame.t * 2)))
                other = "tuned" if name == "baseline" else "baseline"
                with pair_lock:
                    pending[name][frame_index] = frame
                    stale_before = frame_index - 4
                    for arena_frames in pending.values():
                        for stale_index in tuple(arena_frames):
                            if stale_index < stale_before:
                                arena_frames.pop(stale_index, None)
                    other_frame = pending[other].pop(frame_index, None)
                    if other_frame is not None:
                        pending[name].pop(frame_index, None)
                        baseline = frame if name == "baseline" else other_frame
                        tuned = frame if name == "tuned" else other_frame
                        loop.call_soon_threadsafe(queue.put_nowait, (frame_index, baseline, tuned))
            return (
                SimulationService.execute_simulation_stream_sync,
                run_req,
                on_frame,
                worker_stop_signal,
                worker_pause_signal,
                continuous,
                runtime_controls,
            )
        b_req=req.model_copy(deep=True); b_req.config_name="baseline"; b_req.params=ResearchParams(**SimulationService.get_baseline_params())
        t_req=req.model_copy(deep=True); t_req.config_name="tuned"; t_req.params=ResearchParams(**SimulationService.get_tuned_params())
        b_task=asyncio.create_task(asyncio.to_thread(*make_runner(b_req)))
        t_task=asyncio.create_task(asyncio.to_thread(*make_runner(t_req)))
        frame_count = max(1, int(round(req.duration_s * 2)))
        next_index = 1
        waiting_pairs = {}
        while continuous or next_index <= frame_count:
            if stop_event.is_set():
                worker_stop_signal.set()
                break
            if pause_event.is_set():
                await asyncio.sleep(0.1)
                continue
            pair = waiting_pairs.pop(next_index, None)
            if pair is None:
                try:
                    frame_index, baseline, tuned = await asyncio.wait_for(queue.get(), timeout=0.1)
                    waiting_pairs[frame_index] = (baseline, tuned)
                except asyncio.TimeoutError:
                    if b_task.done() and t_task.done() and queue.empty():
                        break
                continue
            baseline, tuned = pair
            progress = (min(baseline.t, tuned.t) % max(1.0, req.duration_s)) / max(1.0, req.duration_s)
            await websocket.send_json({"type":"frame", "frame":{"baseline":baseline.model_dump(),"tuned":tuned.model_dump()}, "progress":round(progress*100,1)})
            await websocket.send_json({"type":"progress", "progress":round(progress*100,1), "sim_time_s":min(baseline.t,tuned.t)})
            next_index += 1
        worker_stop_signal.set()
        b_id,b_metrics=await b_task; t_id,t_metrics=await t_task
        await websocket.send_json({"type":"completed", "experiment_id":b_id,"baseline_experiment_id":b_id,"tuned_experiment_id":t_id,"stopped":stop_event.is_set(),"metrics":{"baseline":b_metrics.model_dump(),"tuned":t_metrics.model_dump()}})

    try:
        while True:
            if running_task and not running_task.done():
                command_task=asyncio.create_task(websocket.receive_text())
                done,_=await asyncio.wait({command_task,running_task},return_when=asyncio.FIRST_COMPLETED)
                if running_task in done:
                    try:
                        running_task.result()
                    except asyncio.CancelledError:
                        pass
                    except Exception as exc:
                        await websocket.send_json({"type":"error","message":str(exc)})
                    running_task=None
                if command_task not in done:
                    command_task.cancel()
                    with suppress(asyncio.CancelledError):
                        await command_task
                    continue
                raw=command_task.result()
            else:
                raw=await websocket.receive_text()
            data=json.loads(raw); action=data.get("action","start")
            if action=="pause":
                pause_event.set(); worker_pause_signal.set(); await websocket.send_json({"type":"status","message":"Simulation paused"})
            elif action=="resume":
                pause_event.clear(); worker_pause_signal.clear(); await websocket.send_json({"type":"status","message":"Simulation resumed"})
            elif action=="update":
                incoming = data.get("settings", {})
                with runtime_controls["lock"]:
                    values = runtime_controls["values"]
                    for key, low, high in (("sensor_noise_multiplier", 0.0, 3.0),
                                           ("packet_loss_extra_pct", 0.0, 60.0),
                                           ("gps_dropout_prob", 0.0, 1.0)):
                        if key in incoming:
                            values[key] = max(low, min(high, float(incoming[key])))
                    if "forced_failure_uav_ids" in incoming:
                        values["forced_failure_uav_ids"] = [int(uid) for uid in incoming["forced_failure_uav_ids"]]
                    runtime_controls["version"] += 1
                await websocket.send_json({"type":"status","message":"Live settings updated"})
            elif action=="stop":
                stop_event.set(); pause_event.clear()
                worker_pause_signal.clear(); worker_stop_signal.set()
                await websocket.send_json({"type":"status","message":"Stopping mission…"})
                if running_task and not running_task.done():
                    try:
                        await running_task
                    except Exception:
                        pass
                running_task = None
            elif action=="start":
                stop_event=asyncio.Event(); pause_event.clear()
                worker_stop_signal=threading.Event(); worker_pause_signal=threading.Event()
                req=SimulationRequest(**data.get("config",{}))
                with runtime_controls["lock"]:
                    runtime_controls["values"] = req.stress.model_dump()
                    runtime_controls["version"] += 1
                running_task=asyncio.create_task(run_comparison(req, data.get("continuous",False)) if data.get("compare",False) else run_single(req))
    except WebSocketDisconnect:
        stop_event.set()
        worker_pause_signal.clear()
        worker_stop_signal.set()
        if running_task and not running_task.done(): running_task.cancel()
# -----------------------------------------------------------------------------
# Static Frontend Serving
# -----------------------------------------------------------------------------
FRONTEND_DIST = os.path.join(PROJECT_ROOT, "web")
if os.path.exists(FRONTEND_DIST):
    app.mount("/static", StaticFiles(directory=FRONTEND_DIST), name="static")
    app.mount("/assets", StaticFiles(directory=FRONTEND_DIST), name="assets")

@app.get("/")
def serve_index():
    index_file = os.path.join(PROJECT_ROOT, "web", "index.html")
    if os.path.exists(index_file):
        return FileResponse(index_file)
    fallback = os.path.join(PROJECT_ROOT, "index.html")
    if os.path.exists(fallback):
        return FileResponse(fallback)
    return {"message": "UAV Swarm Research Platform Backend is Running"}

if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", "8000"))
    uvicorn.run("main:app", host="0.0.0.0", port=port, reload=False)

