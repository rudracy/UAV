"""
optimization.py — REST endpoints for Grid Search, Bayesian Optimization, and Ablation.
"""
from fastapi import APIRouter, HTTPException
from uav_platform.api.models import GridSearchRequest, AblationRequest
from uav_platform.experiments.optimization import run_grid_search, run_bayesian_optimization, run_ablation_study

router = APIRouter(prefix="/api/optimization", tags=["optimization"])

@router.post("/grid-search")
async def execute_grid_search(req: GridSearchRequest):
    try:
        result = run_grid_search(
            comm_rates=req.comm_rates,
            neighbour_counts=req.neighbour_counts,
            safety_margins=req.safety_margins,
            sensor_fusion_gains=req.sensor_fusion_gains,
            n_uavs=req.n_uavs,
            duration_s=req.duration_s,
            seed=req.seed
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Grid search failed: {str(e)}")

@router.post("/bayesian")
async def execute_bayesian_optimization(iterations: int = 10, n_uavs: int = 10, duration_s: float = 15.0, seed: int = 42):
    try:
        result = run_bayesian_optimization(
            n_iterations=iterations,
            n_uavs=n_uavs,
            duration_s=duration_s,
            seed=seed
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Bayesian optimization failed: {str(e)}")

@router.post("/ablation")
async def execute_ablation(req: AblationRequest):
    try:
        p_dict = req.base_params.model_dump() if req.base_params else None
        s_dict = req.stress.model_dump() if req.stress else None
        result = run_ablation_study(
            base_params=p_dict,
            n_uavs=req.n_uavs,
            duration_s=req.duration_s,
            seed=req.seed,
            stress=s_dict
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Ablation study failed: {str(e)}")

