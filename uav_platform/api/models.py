"""
models.py — Pydantic models for API request/response schemas.
"""
from __future__ import annotations
from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field

class ResearchParams(BaseModel):
    comm_rate_hz: float = Field(5.0, ge=1.0, le=20.0, description="Communication Rate (Hz)")
    neighbour_count: int = Field(4, ge=1, le=8, description="Neighbour Count k (k-mesh vs broadcast)")
    safety_margin_m: float = Field(2.8, ge=0.5, le=5.0, description="Safety Margin (m)")
    sensor_fusion_gain: float = Field(0.22, ge=0.05, le=1.0, description="Sensor-Fusion Gain (EKF weight)")
    failsafe_threshold_pct: float = Field(15.0, ge=5.0, le=30.0, description="Failsafe Threshold (%)")

class StressConfig(BaseModel):
    sensor_noise_multiplier: float = Field(1.0, ge=0.0, le=3.0)
    gps_dropout_prob: float = Field(0.0, ge=0.0, le=1.0)
    motor_failure_prob: float = Field(0.0, ge=0.0, le=0.01)
    packet_loss_extra_pct: float = Field(0.0, ge=0.0, le=60.0)
    outage_uav_ids: List[int] = Field(default_factory=list)
    forced_failure_uav_ids: List[int] = Field(default_factory=list)

class SimulationRequest(BaseModel):
    seed: int = 42
    duration_s: float = Field(60.0, ge=10.0, le=2700.0)
    n_uavs: int = Field(10, ge=5, le=50)
    n_pois: int = Field(10, ge=3, le=20)
    params: ResearchParams = Field(default_factory=ResearchParams)
    stress: StressConfig = Field(default_factory=StressConfig)
    config_name: str = "custom"

class UAVTelemetry(BaseModel):
    id: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    yaw: float
    battery_pct: float
    alive: bool
    role: str
    gps_ok: bool
    task_id: Optional[int] = None
    comm_links: List[int] = Field(default_factory=list)

class SimulationFrame(BaseModel):
    t: float
    uavs: List[UAVTelemetry]
    pois: List[Dict[str, Any]]
    coverage_pct: float
    collisions: int
    alive_uavs: int
    connectivity_ratio: float
    avg_latency_ms: float
    comm_graph: Dict[str, Any] = Field(default_factory=lambda: {"links": []})
    events: List[Dict[str, Any]] = Field(default_factory=list)

class MetricsResult(BaseModel):
    duration_s: float
    n_uavs: int
    alive_uavs: int
    failed_uavs: int
    n_pois: int
    surveyed_pois: int
    mission_completion_pct: float
    coverage_pct: float
    collision_count: int
    avg_latency_ms: float
    packet_delivery_ratio: float
    avg_battery_pct: float
    scalability_index: float
    robustness_index: float
    seed: int
    params: Dict[str, Any]
    stress: Dict[str, Any]

class ExperimentRecord(BaseModel):
    id: str
    config_name: str
    params: Dict[str, Any]
    stress: Dict[str, Any]
    n_uavs: int
    seed: int
    status: str
    metrics: Optional[MetricsResult] = None
    created_at: str
    duration_s: float

class GridSearchRequest(BaseModel):
    comm_rates: List[float] = [2.0, 5.0, 10.0, 20.0]
    neighbour_counts: List[int] = [2, 4, 8]
    safety_margins: List[float] = [1.0, 2.8]
    sensor_fusion_gains: List[float] = [0.22, 1.0]
    n_uavs: int = 10
    duration_s: float = 30.0
    seed: int = 42

class AblationRequest(BaseModel):
    base_params: Optional[ResearchParams] = None
    n_uavs: int = 15
    duration_s: float = 30.0
    seed: int = 42
    stress: Optional[StressConfig] = None

