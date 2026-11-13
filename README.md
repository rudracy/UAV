# AERO-SWARM: UAV swarm simulation and comparison

AERO-SWARM is a local web application for comparing a baseline UAV swarm with a tuned swarm. It includes a Three.js 3D display, a Python/FastAPI simulation service, REST endpoints, a live WebSocket stream, experiment records, optimization endpoints, and a paired Monte Carlo benchmark.

This repository is a **software simulation and research prototype**. It does not connect to real aircraft, and its simplified physics, communications, sensing, and failure models are not flight-control or safety certification software. The equations and data notes below describe what the code currently computes; they should not be read as validation against real flight performance.

## Contents

- [Project layout](#project-layout)
- [Requirements](#requirements)
- [Run on macOS](#run-on-macos)
- [Run on Windows](#run-on-windows)
- [Use the application](#use-the-application)
- [Backend and data flow](#backend-and-data-flow)
- [Simulation model](#simulation-model)
- [Configuration parameters](#configuration-parameters)
- [Metrics and formulas](#metrics-and-formulas)
- [Bundled datasets and stored results](#bundled-datasets-and-stored-results)
- [Benchmark, optimization, and ablation](#benchmark-optimization-and-ablation)
- [API reference](#api-reference)
- [Troubleshooting](#troubleshooting)
- [Known limitations](#known-limitations)

## Project layout

| Path | Purpose |
| --- | --- |
| `main.py` | **Primary application entry point.** Starts FastAPI, serves the web UI, REST APIs, live simulation WebSocket, and batch benchmark. |
| `web/index.html` | Active single-page Three.js interface, controls, visualisation, and benchmark panel. The browser downloads Three.js from a CDN, so an internet connection is needed for that library. |
| `uav_platform/` | Active simulation engine, API models/routes, metrics, experiment store, benchmark and optimization logic. |
| `data/` | Bundled CSV/JSON telemetry, calibration metadata, historical experiment data and benchmark artifacts. |
| `requirements.txt` | Python dependencies for the active FastAPI application. |
| `tests/` | Python unit tests. |
| `scripts/run_monte_carlo_campaign.py` | Separate command-line campaign runner that writes Monte Carlo CSV/summary files. |
| `server.py`, `index.html`, `backend/` | Older, separate implementation. It is not the app shown by `main.py` and is not needed for the primary UI. |
| `frontend/` | Placeholder package metadata; the active interface is served directly from `web/index.html` and does not require an npm build. |

Run `main.py` for the dual-arena app. Do not start `server.py` at the same time unless you specifically intend to inspect the older implementation.

## Requirements

- Python 3.10 or newer (3.11 is a good choice).
- macOS or Windows 10/11.
- A modern browser with WebGL enabled (Chrome, Edge, Firefox, or Safari).
- Internet access on first use if the browser needs to fetch the Three.js CDN script.
- About 1 GB of available memory is generally sufficient for the normal UI; benchmark workload and large swarms increase CPU and memory use.

The frontend is static HTML/JavaScript. Node.js/npm are not required to run the active application.

## Run on macOS

Open Terminal and run these commands from the project directory:

```bash
cd /Users/lakshayvijay/Downloads/UAV-main
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
PORT=8001 python main.py
```

Use `PORT=8001` if port 8000 is already in use. You can choose another unused port, for example `PORT=8002 python main.py`. To use the default port, run `python main.py` instead. Keep the terminal open while using the site; press **Ctrl+C** in that terminal to stop the server.

Open <http://localhost:8001/dual-arena> in the browser. `http://127.0.0.1:8001/dual-arena` also works. Do not enter `0.0.0.0` in the browser: that is the server bind address, not the usual local browser address.

To restart later, return to the folder and activate the existing environment:

```bash
cd /Users/lakshayvijay/Downloads/UAV-main
source .venv/bin/activate
PORT=8001 python main.py
```

If `python3` is unavailable, install Python 3 from python.org or through your preferred package manager, then reopen Terminal.

## Run on Windows

Open **PowerShell** in the project folder. If the repository is in Downloads:

```powershell
cd "$HOME\Downloads\UAV-main"
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
$env:PORT = "8001"
python main.py
```

Open <http://localhost:8001/dual-arena>. Keep PowerShell running while using the app and press **Ctrl+C** to stop the server. For the default port, omit `$env:PORT = "8001"` and use `python main.py`.

If PowerShell cannot find Python, install Python 3.10+ and enable the Python launcher (`py`) during installation. If activation is unavailable or restricted on your machine, you can run the environment's interpreter directly without changing PowerShell policy:

```powershell
cd "$HOME\Downloads\UAV-main"
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
$env:PORT = "8001"
.\.venv\Scripts\python.exe main.py
```

### Install dependencies without a virtual environment

Virtual environments are recommended so project packages do not affect other Python projects. If you intentionally want to install into your current Python environment, use:

```bash
python3 -m pip install -r requirements.txt
```

On Windows, use `py -3 -m pip install -r requirements.txt`.

## Use the application

1. Start the backend using the platform-specific instructions above.
2. Visit `/dual-arena` on the same host and port printed by Uvicorn.
3. Choose the swarm size and duration/window in the top controls.
4. Select **Execute Dual** to run the baseline and tuned configurations side by side. The two runs share the requested seed, swarm size, duration, and stress settings; only their configuration presets differ.
5. Use **Pause** and **Resume** to suspend/continue a live mission. **Stop / Reset** stops it and returns the display to its initial state.
6. Use the bottom controls to change swarm size, IMU noise multiplier, and packet-loss stress. GPS dropout toggles GPS stress. The kill-drone control injects a failure for UAV #3 as a failover scenario.
7. The speed slider changes playback/simulation pacing. Camera choices change the Three.js view, not the backend simulation.
8. Select **30-run matrix** for the paired benchmark. The result panel displays progress and then summary statistics. The UI requests 30 trials; this is a CPU-intensive calculation and can take several minutes. Keep the browser tab and backend process open until it completes.

The mission duration is a simulated-time window. The live UI can continue cycling the mission until stopped; a completed one-shot REST simulation instead ends at its requested duration. Stress and swarm-size behavior can differ from a prior run because runs use seeds and noise, and the visual appearance depends on the selected swarm size, camera, stress, and elapsed time.

## Backend and data flow

The active request path is:

```text
Browser (web/index.html / Three.js)
        | REST: setup, history, analytics, benchmark
        | WebSocket: start/pause/resume/stop/settings + streamed frames
        v
FastAPI application (main.py)
        v
SimulationService / SimulationRunner (uav_platform/)
        |-- world, UAV dynamics, sensors, coverage, task allocation
        |-- connectivity graph and mission metrics
        v
Experiment database/store -> API results and downloadable CSV/JSON
```

`main.py` mounts REST routers, serves `web/index.html` at `/` and `/dual-arena`, exposes `/api/health`, and accepts the live connection at `/ws/simulation`. The browser derives the WebSocket host and `ws`/`wss` scheme from the page URL. For local use, serve the page from this same backend origin; opening the HTML file directly with `file://` will not connect correctly.

The live path runs simulation work off the asynchronous request loop and streams frame and progress messages over the WebSocket. The message actions include `start`, `pause`, `resume`, `stop`, and `update`. `update` changes supported stress inputs during a run. Swarm-size changes are for the next run. The one-shot REST route returns its generated frames in the response, so the WebSocket route is the preferred interactive path.

The experiment store is implemented in `uav_platform/api/database.py`; inspect that file for the exact storage location and persistence behavior of the current checkout. The data CSVs in `data/` are distinct from this live experiment store. The browser does not directly load the telemetry CSV to draw the active simulation.

`server.py` is an older standard-library HTTP app and uses a different `backend/` simulation/data-loading path. Its endpoints and data schema are not interchangeable with the active FastAPI service.

## Simulation model

The primary engine is implemented in `uav_platform/sim/` and uses a fixed-step, software model:

- **Arena:** 400 m by 400 m square, represented by a 5 m coverage grid (80 × 80 cells). A simulation tick is 0.1 s.
- **Initial placement:** UAVs start on a 20 m radius circle around the ground control station at 10 m altitude. Points of interest (POIs) are distributed across the arena by the seeded initializer.
- **Obstacles and terrain:** The runner generates 12 seeded cylindrical obstacles. Terrain elevation and wind are simplified models in `world.py`; wind contains a seeded mean and time-varying gust component.
- **Flight motion:** UAVs steer toward a POI or continuous sweep target, limit desired speed to 15 m/s, limit acceleration to 3 m/s², and limit climb rate. Obstacle/vehicle avoidance is simulated by software rules, not a real autopilot.
- **Coverage:** A camera field-of-view model projects a footprint onto the 5 m grid. A cell counts once when marked covered. The UI renders the backend coverage and a separate visual heat/footprint layer.
- **Tasks/failures:** The task allocator assigns survey work to live UAVs and responds to failures. Failure and GPS/packet-loss stress are simulation inputs, not hardware events.
- **Communications:** `ConnectivityGraph` constructs links within a nominal 150 m radio range, applies a simplified signal/packet-delivery model, and uses either broadcast-like or nearest-neighbour connectivity settings.
- **Sensors:** IMU and GPS behavior is simplified and includes configurable noise/dropout. A low sensor-fusion gain currently changes the model's noise behavior; it is not a full navigation-grade Extended Kalman Filter.
- **Energy:** The motion model estimates battery use from hover, forward movement, and climb power. It is an illustrative energy budget, not an aircraft battery/propulsion validation.

Some details (for example radio propagation, obstacle visibility, sensor distributions, and camera footprint) are assumptions encoded by this project. Results are model outputs rather than a claim that a real swarm would achieve the displayed numbers.

## Configuration parameters

The presets in `uav_platform/api/simulation_service.py` are:

| Parameter | Baseline | Tuned | Meaning in this model |
| --- | ---: | ---: | --- |
| `comm_rate_hz` | 20 Hz | 5 Hz | Configured communication/beacon update rate. |
| `neighbour_count` | 8 | 4 | Maximum nearest-neighbour setting used by the connectivity model; high settings can use its broadcast-like path. |
| `safety_margin_m` | 0.6 m | 2.8 m | Separation/avoidance threshold used by motion behavior. |
| `sensor_fusion_gain` | 1.0 | 0.22 | Gain passed to the simplified sensor/dynamics noise behavior. The UI may describe the tuned preset as “EKF”; the active engine does not implement a complete EKF. |
| `failsafe_threshold_pct` | 5% | 15% | Threshold supplied to the failsafe behavior. It is not equivalent to a certified battery return-to-home system. |

Live stress controls are separate from those preset parameters:

| Stress input | UI range/behavior | Effect |
| --- | --- | --- |
| IMU noise multiplier | 0× to 3× | Scales the sensor-noise stress setting for the active simulation. |
| Extra link packet loss | 0% to 60% | Adds loss to the communications model. |
| GPS dropout | Toggle | Enables/disables GPS dropout stress. |
| Forced UAV failure | UAV #3 button | Adds UAV ID 3 to the forced failure/outage set for the current mission. |

The API accepts a `seed` to seed environment, POI, sensor, and related random generators. This improves paired comparisons, but it does not promise bit-for-bit reproducibility across every code path or software version.

## Metrics and formulas

Metric calculations are in `uav_platform/api/metrics.py`. The inputs are outputs from the simulation runner. `clamp(x, a, b)` means `min(b, max(a, x))`.

### Scalability index

The score is a weighted average on a 0–100 scale:

```text
S_idx = 0.35 * C_scale + 0.35 * T_ret + 0.30 * S_safe
```

Components:

```text
expected_coverage(N) = max(1, min(35, 3.2 * sqrt(N)))
C_scale = clamp(100 * coverage_pct / expected_coverage(N), 0, 100)
T_ret   = clamp(100 * clamp(1 - latency_ms / 250, 0, 1)
                     * clamp(packet_delivery_ratio, 0, 1), 0, 100)
S_safe  = clamp(100 * (1 - collisions / max(1, 0.5 * N)), 0, 100)
```

`coverage_pct` is a percentage (0–100), `packet_delivery_ratio` is a fraction (0–1), latency is milliseconds, and collisions is the runner's collision count. The `expected_coverage` curve is a normalization chosen in code, not a value learned from the supplied CSV.

### Robustness index

```text
R_idx = 0.35 * M_meas + 0.30 * U_surv + 0.20 * L_pdr + 0.15 * B_res
M_meas = clamp(mission_completion_pct, 0, 100)
U_surv = clamp(100 * (alive_uavs / N) * max(0, 1 - collisions / N), 0, 100)
L_pdr  = clamp(100 * packet_delivery_ratio, 0, 100)
B_res  = clamp(avg_battery_pct, 0, 100)
```

These weights sum to one, and each component is clamped to 0–100, so both indices are bounded from 0 to 100.

### Improvement percentages

For a higher-is-better metric, improvement is `(tuned - baseline) / abs(baseline) * 100`. For lower-is-better metrics (such as collisions or latency), improvement is `(baseline - tuned) / abs(baseline) * 100`. The helper has explicit handling for a zero baseline. A percentage improvement is relative change; it is not the same as a percentage-point difference.

### Motion and communications examples

In `uav_platform/sim/uav.py`, desired velocity is based on target displacement and capped; acceleration is derived from velocity error over `dt` and bounded. Drag uses a quadratic term of the form `F_drag = 0.5 * rho * Cd * A * v^2`. Battery energy is integrated from the configured hover/forward/climb power model over each step.

In `uav_platform/sim/connectivity.py`, the model calculates distance/line-of-sight and uses an RSSI approximation and packet-delivery probability. The code's path-loss approximation is simplified; it does not simulate a standards-compliant RF channel, spectrum contention, modulation/coding, or a real radio. The model's neighbor selection reduces the retained topology, but graph candidate construction can still examine UAV pairs, so describing the implementation as guaranteed O(N) total computation would be inaccurate.

## Bundled datasets and stored results

| File | What is in the repository | How the primary app uses it |
| --- | --- | --- |
| `data/euroc_mav_telemetry.csv` | Bundled telemetry-like CSV with IMU, GPS, ground-truth position and flight-phase columns. The checked-in file contains 2,000 data rows at 0.05 s spacing (approximately 20 Hz for 100 s). | Not read by the active live simulation runner or its sensor initialization. It is a separate data artifact. |
| `data/sensor_calibration.json` | Calibration/noise metadata whose text attributes values to EuRoC MAV / ADIS16448 and GPS/environment assumptions. | Not loaded by the active FastAPI simulation path. |
| `data/monte_carlo_runs.csv` | Checked-in per-run records from a prior campaign (240 data rows in this checkout). | Not the source of the live 30-run matrix; the active matrix computes fresh trials. |
| `data/benchmark_summary.json` | Checked-in summary artifact for prior benchmark results. | Some analytics fallback routes can read this artifact when no stored baseline/tuned experiments are available. It should be identified as a stored artifact, not a newly computed live result. |
| `data/ablation_matrix.json` | Stored ablation-related results. | Separate from a newly requested ablation endpoint calculation. |
| `data/experiments_db.json` | Repository data file related to experiment records. | The active experiment database implementation determines its own persistence location/behavior; do not assume this JSON file is automatically populated by the live app without checking `uav_platform/api/database.py`. |

The names and metadata in the repository are project-provided provenance descriptions. The active simulation does not ingest the CSV as raw EuRoC flight data or calibrate its sensor model from those rows. Verify the original dataset source, processing history, and license from the official publisher before academic redistribution or attributing results to real EuRoC measurements.

## Benchmark, optimization, and ablation

### Interactive paired matrix

The UI calls `POST /api/benchmark/batch?trials=30`. The backend evaluates sizes N=5, 10, 25, and 50. For each size it runs baseline and tuned simulations for each trial with the same nominal seed, then returns means and standard deviations for coverage, collisions, latency, PDR, scalability, and robustness, plus relative improvements. Default UI settings request 30 trials per size: 4 × 30 × 2 = **240 simulation executions**. The service reports progress through `GET /api/benchmark/status` while the request is running. The benchmark currently uses its code-defined duration and stress defaults; it does not use the current live UI mission state or read the historical CSV.

The endpoint clamps `trials` to the range 1–30. A smaller value is useful for a quick check, for example:

```text
http://localhost:8001/docs
```

In Swagger UI, expand `POST /api/benchmark/batch`, try `trials=1`, then try 30 for the full campaign. The benchmark does not persist each returned trial to the historical CSV automatically. Statistical standard deviation is computed by NumPy with its default population convention (`ddof=0`).

### Standalone CSV campaign

`scripts/run_monte_carlo_campaign.py` is a separate command-line runner and writes campaign artifacts under `data/`. It is not the same code path as the browser matrix. Review the script before running it: it may use a process pool and several hundred tasks, consuming substantial CPU. Do not launch it while expecting the interactive backend to remain responsive on a small machine.

### Optimization and ablation

- **Grid search:** `/api/optimization/grid-search` forms a parameter grid and evaluates a representative subset of up to 16 configurations, ordered by a 50/50 average of scalability and robustness indices. It does not exhaustively run all combinations when the grid is larger.
- **Bayesian-style search:** `/api/optimization/bayesian` starts with baseline and tuned settings, then uses a lightweight radial-basis weighted surrogate and an upper-confidence-style candidate rule. It is a project implementation, not a call to a full Gaussian-process library.
- **Ablation:** `/api/optimization/ablation` evaluates the tuned configuration, reverts one parameter at a time to its baseline value, and evaluates the all-baseline configuration for comparison.

These are model-based experiments. A high score means better according to the code's chosen metric and assumptions, not proven real-world flight safety or a universal optimum.

## API reference

The interactive API documentation is at `http://localhost:8001/docs` (replace 8001 with the port you chose). The OpenAPI schema is at `/openapi.json`.

| Method | Path | Purpose |
| --- | --- | --- |
| `GET` | `/` or `/dual-arena` | Serve the main UI. |
| `GET` | `/api/health` | Check that the API process is responding. |
| `WS` | `/ws/simulation` | Live simulation commands and frames. |
| `POST` | `/api/simulation/run` | Run a finite simulation and return frames and final metrics. |
| `GET` | `/api/simulation/configs/baseline` | Return baseline preset. |
| `GET` | `/api/simulation/configs/tuned` | Return tuned preset. |
| `GET` | `/api/simulation/{experiment_id}` | Fetch a stored simulation record. |
| `GET` | `/api/experiments` | List experiment records. |
| `GET` | `/api/experiments/stats` | Summarize stored experiments. |
| `GET` | `/api/experiments/export/csv` or `/api/experiments/export/json` | Download stored records. |
| `DELETE` | `/api/experiments/{experiment_id}` | Delete one experiment record. |
| `POST` | `/api/analytics/compare` | Compare stored baseline/tuned runs or use the stored benchmark fallback. |
| `GET` | `/api/analytics/scalability` | Read scalability analysis. |
| `GET` | `/api/analytics/ablation` | Read ablation analysis. |
| `POST` | `/api/optimization/grid-search` | Run the sampled grid search. |
| `POST` | `/api/optimization/bayesian` | Run the surrogate search. |
| `POST` | `/api/optimization/ablation` | Run a fresh one-at-a-time ablation. |
| `POST` | `/api/benchmark/batch?trials=30` | Run a paired benchmark across four swarm sizes. |
| `GET` | `/api/benchmark/status` | Read benchmark progress/error state. |

For exact request JSON schemas, use `/docs`; they are defined by Pydantic models in `uav_platform/api/models.py`.

## Troubleshooting

### “Address already in use” on port 8000

Another process is listening on 8000. Either stop the old process in its terminal with Ctrl+C, or start this app on another port:

```bash
PORT=8001 python main.py
```

PowerShell:

```powershell
$env:PORT = "8001"
python main.py
```

Then browse to the matching `localhost` URL. Do not repeatedly start another server on the same occupied port.

### Backend connected indicator is off or the page stays empty

- Confirm the terminal says `Uvicorn running on ...:8001` and has no traceback.
- Open the same port and `/dual-arena` from the URL printed by the server.
- Check `http://localhost:8001/api/health` for a JSON healthy response.
- Keep the backend terminal running. A page opened as a local file or through a different host/port will not share the WebSocket endpoint.
- Ensure the browser allows WebGL and can reach the Three.js CDN.
- If the page was already open before the server restarted, reload the page.

### Sliders or stress buttons appear inactive

Use the active `web/index.html` served by `main.py`, not the root legacy `index.html` served by `server.py`. Start a mission before using controls intended for live updates. Swarm size applies to the next run; stress controls are the ones designed to update an active run.

### Benchmark looks stuck or shows a running message

Thirty trials mean 240 sequential simulation executions across the four swarm sizes. It can take several minutes, especially at N=50. Check `/api/benchmark/status` and the server terminal. A successful response replaces the progress panel with the results; an error is surfaced in the panel and backend log. Use fewer trials for a smoke check, then run 30 when ready. Do not treat the progress message as completed results.

### `ModuleNotFoundError` during startup

Activate the `.venv` created for this project, then rerun `python -m pip install -r requirements.txt`. On Windows use the `.venv` interpreter shown above. Confirm `python --version` reports 3.10 or newer.

## Known limitations

- The simulator is a simplified, discrete-time research visualization. It does not model full 6-DOF rigid-body aerodynamics, actuator/motor dynamics, certified collision avoidance, or real radio standards.
- Active sensor fusion is not a complete EKF, despite labels in some UI text and old comments.
- Bundled EuRoC-named telemetry/calibration artifacts are not used by the primary live simulator; their exact provenance and license need external verification before publication or redistribution.
- Seed handling supports controlled comparisons, but do not promise exact reproducibility across versions/platforms without verifying every source of randomness.
- The preset comparison is not guaranteed to make the tuned swarm outperform baseline on every metric, seed, swarm size, or stress level.
- Historical CSV/JSON results and fresh API results are different sources. Reports should state which source, parameters, stress settings, swarm sizes, trial count, and software revision produced them.
- This project is not suitable for controlling real UAVs or making operational safety decisions.

## Tests

To run the repository's Python unit tests from an activated environment:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
```

The active app does not have a frontend npm build step. The tests cover Python behavior only; they do not replace browser/WebGL end-to-end verification.
