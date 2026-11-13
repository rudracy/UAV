#!/usr/bin/env python3
"""
UAV Swarm Simulation Server & Empirical Data Engine
Part of "Improving Scalability and Robustness of UAV Swarms" research study.
Integrates Python HTTP REST API server with ETH Zurich EuRoC MAV ADIS16448 dataset.
"""

import http.server
import socketserver
import json
import os
import sys
import urllib.parse
from typing import Dict, Any

from backend.dataset_loader import DatasetLoader
from backend.ekf import SensorFusionEKF
from backend.swarm_simulator import SwarmSimulator
from backend.monte_carlo_runner import MonteCarloRunner

PORT = int(os.environ.get("PORT", 8000))
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

# Initialize Backend Engines
dataset_loader = DatasetLoader()
mc_runner = MonteCarloRunner()
ekf_baseline = SensorFusionEKF(filter_gain=1.0)
ekf_tuned = SensorFusionEKF(filter_gain=0.22)

current_stress_params = {
    "swarm_size": 10,
    "gps_dropout": False,
    "sensor_noise_multiplier": 1.0,
    "packet_loss_pct": 0.0,
    "injected_failure": False
}

class UAVSimulationHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def _send_json(self, data: Any, status: int = 200):
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(json.dumps(data, indent=2).encode("utf-8"))

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed_url = urllib.parse.urlparse(self.path)
        path = parsed_url.path
        query = urllib.parse.parse_qs(parsed_url.query)

        # 1. API: System Health & Status
        if path == "/api/health":
            summary = dataset_loader.get_dataset_summary()
            self._send_json({
                "status": "healthy",
                "backend_engine": "Python 3 Standard Library",
                "port": PORT,
                "dataset_loaded": summary["telemetry"]["record_count"] > 0,
                "telemetry_records": summary["telemetry"]["record_count"],
                "monte_carlo_records": summary["monte_carlo"]["total_trial_records"]
            })
            return

        # 2. API: Dataset Summary Metadata
        elif path == "/api/dataset/info":
            self._send_json(dataset_loader.get_dataset_summary())
            return

        # 3. API: Telemetry Records Sample
        elif path == "/api/dataset/sample":
            offset = int(query.get("offset", [0])[0])
            limit = int(query.get("limit", [25])[0])
            phase = query.get("phase", [None])[0]
            self._send_json(dataset_loader.get_telemetry_sample(offset=offset, limit=limit, phase=phase))
            return

        # 4. API: 30-Run Statistical Benchmarks
        elif path == "/api/benchmark":
            stats = dataset_loader.get_benchmark_statistics()
            self._send_json(stats)
            return

        # 5. API: Ablation Study Matrix
        elif path == "/api/ablation":
            ablation_records = dataset_loader._ablation_cache.get("ablation_study_records", [])
            self._send_json({"ablation_study": ablation_records})
            return

        # 6. API: Stress Parameters
        elif path == "/api/stress":
            self._send_json(current_stress_params)
            return

        # 7. API: Live Physics State from Backend
        elif path == "/api/telemetry/live":
            n = current_stress_params["swarm_size"]
            sim_b = SwarmSimulator(is_tuned=False)
            sim_b.reset_swarm(n)
            sim_b.step(
                0.1,
                noise_mult=current_stress_params["sensor_noise_multiplier"],
                gps_dropout=current_stress_params["gps_dropout"],
                packet_loss=current_stress_params["packet_loss_pct"] / 100.0
            )

            sim_t = SwarmSimulator(is_tuned=True)
            sim_t.reset_swarm(n)
            sim_t.step(
                0.1,
                noise_mult=current_stress_params["sensor_noise_multiplier"],
                gps_dropout=current_stress_params["gps_dropout"],
                packet_loss=current_stress_params["packet_loss_pct"] / 100.0
            )

            self._send_json({
                "baseline": sim_b.get_state(),
                "tuned": sim_t.get_state(),
                "active_stress": current_stress_params
            })
            return

        # Default static file serving (e.g. index.html)
        super().do_GET()

    def do_POST(self):
        parsed_url = urllib.parse.urlparse(self.path)
        path = parsed_url.path

        content_length = int(self.headers.get("Content-Length", 0))
        post_data = self.rfile.read(content_length)
        body = {}
        if post_data:
            try:
                body = json.loads(post_data.decode("utf-8"))
            except Exception as e:
                self._send_json({"error": f"Invalid JSON payload: {str(e)}"}, status=400)
                return

        # 1. API: Ingest and update stress parameters
        if path == "/api/stress":
            current_stress_params.update(body)
            self._send_json({
                "status": "success",
                "message": "Stress parameters updated",
                "current": current_stress_params
            })
            return

        # 2. API: Dynamic Monte Carlo Simulation Execution
        elif path == "/api/benchmark/run":
            n = body.get("swarm_size", current_stress_params["swarm_size"])
            runs = body.get("runs", 10)
            noise_mult = body.get("sensor_noise_multiplier", current_stress_params["sensor_noise_multiplier"])
            gps_dropout = body.get("gps_dropout", current_stress_params["gps_dropout"])
            packet_loss = body.get("packet_loss_rate", current_stress_params["packet_loss_pct"] / 100.0)
            inject_fail = body.get("injected_failure", current_stress_params["injected_failure"])

            result = mc_runner.run_experiment(
                swarm_size=int(n),
                runs=int(runs),
                noise_mult=float(noise_mult),
                gps_dropout=bool(gps_dropout),
                packet_loss=float(packet_loss),
                inject_fail=bool(inject_fail)
            )
            self._send_json(result)
            return

        self._send_json({"error": "Endpoint not found"}, status=404)

class ReusableTCPServer(socketserver.TCPServer):
    allow_reuse_address = True

def run():
    with ReusableTCPServer(("", PORT), UAVSimulationHandler) as httpd:
        print(f"\n==================================================================")
        print(f" UAV Swarm Dual-Arena 3D Simulation & Backend Engine")
        print(f"==================================================================")
        print(f" Local Server running at: http://localhost:{PORT}")
        print(f" Serving directory: {DIRECTORY}")
        print(f" Dataset:")
        print(f"   • EuRoC MAV Telemetry: data/euroc_mav_telemetry.csv (2,000 samples)")
        print(f"   • Monte Carlo Trials : data/monte_carlo_runs.csv (240 runs)")
        print(f"   • Sensor Calibration : data/sensor_calibration.json")
        print(f" REST API endpoints:")
        print(f"   - GET  /api/health")
        print(f"   - GET  /api/dataset/info")
        print(f"   - GET  /api/dataset/sample")
        print(f"   - GET  /api/benchmark")
        print(f"   - POST /api/benchmark/run")
        print(f"   - GET  /api/ablation")
        print(f"   - POST /api/stress")
        print(f"   - GET  /api/telemetry/live")
        print(f"==================================================================\n")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down simulation server...")
            httpd.shutdown()

if __name__ == "__main__":
    run()
