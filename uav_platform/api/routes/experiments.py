"""
experiments.py — REST routes for experiment history, comparisons, and export.
"""
import io
import csv
from typing import Optional
from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import StreamingResponse, JSONResponse
from uav_platform.api.database import db

router = APIRouter(prefix="/api/experiments", tags=["experiments"])

@router.get("")
def list_experiments(limit: int = 50, config_name: Optional[str] = None):
    return db.list_experiments(limit=limit, config_name=config_name)

@router.get("/stats")
def get_experiment_stats():
    return db.get_stats()

@router.get("/{exp_id}")
def get_experiment(exp_id: str):
    record = db.get_experiment(exp_id)
    if not record:
        raise HTTPException(status_code=404, detail="Experiment not found")
    return record

@router.delete("/{exp_id}")
def delete_experiment(exp_id: str):
    success = db.delete_experiment(exp_id)
    if not success:
        raise HTTPException(status_code=404, detail="Experiment not found")
    return {"status": "success", "message": f"Deleted {exp_id}"}

@router.get("/export/csv")
def export_csv():
    records = db.list_experiments(limit=500)
    output = io.StringIO()
    writer = csv.writer(output)

    writer.writerow([
        "experiment_id", "config_name", "n_uavs", "seed", "duration_s",
        "coverage_pct", "collision_count", "mission_completion_pct",
        "avg_latency_ms", "packet_delivery_ratio", "avg_battery_pct",
        "scalability_index", "robustness_index", "created_at"
    ])

    for r in records:
        m = r.get("metrics") or {}
        writer.writerow([
            r.get("id"),
            r.get("config_name"),
            r.get("n_uavs"),
            r.get("seed"),
            r.get("duration_s"),
            m.get("coverage_pct"),
            m.get("collision_count"),
            m.get("mission_completion_pct"),
            m.get("avg_latency_ms"),
            m.get("packet_delivery_ratio"),
            m.get("avg_battery_pct"),
            m.get("scalability_index"),
            m.get("robustness_index"),
            r.get("created_at")
        ])

    output.seek(0)
    return StreamingResponse(
        iter([output.getvalue()]),
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=uav_swarm_experiments.csv"}
    )

@router.get("/export/json")
def export_json():
    records = db.list_experiments(limit=500)
    return JSONResponse(
        content={"experiments": records},
        headers={"Content-Disposition": "attachment; filename=uav_swarm_experiments.json"}
    )

