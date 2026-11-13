"""
simulation.py — REST routes for simulation execution and configuration presets.
"""
from fastapi import APIRouter, HTTPException
from uav_platform.api.models import SimulationRequest, ResearchParams
from uav_platform.api.simulation_service import SimulationService
from uav_platform.api.database import db

router = APIRouter(prefix="/api/simulation", tags=["simulation"])

@router.post("/run")
async def run_simulation(req: SimulationRequest):
    try:
        exp_id, frames, metrics = await SimulationService.run_simulation(req)
        return {
            "status": "success",
            "experiment_id": exp_id,
            "metrics": metrics,
            "total_frames": len(frames),
            "frames": frames
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Simulation failed: {str(e)}")

@router.get("/configs/baseline")
def get_baseline_config():
    return {
        "config_name": "baseline",
        "description": "Default un-tuned industry configuration (O(N^2) broadcast, rigid margin, raw sensor noise)",
        "params": SimulationService.get_baseline_params()
    }

@router.get("/configs/tuned")
def get_tuned_config():
    return {
        "config_name": "tuned",
        "description": "Proposed autonomous parameter optimization (k-nearest mesh O(N), EKF filtering, adaptive margin)",
        "params": SimulationService.get_tuned_params()
    }

@router.get("/{exp_id}")
def get_simulation_run(exp_id: str):
    record = db.get_experiment(exp_id)
    if not record:
        raise HTTPException(status_code=404, detail="Experiment not found")
    return record

