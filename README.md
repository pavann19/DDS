# DDS — Driving Decision System

A real-time driving decision support system that processes OBD-II vehicle telemetry to classify driving intent, detect anomalies, and visualise the driving experience through a 3D browser-based HMI.

![Python](https://img.shields.io/badge/Python-3.13-blue?logo=python)
![Next.js](https://img.shields.io/badge/Next.js-15-black?logo=next.js)
![FastAPI](https://img.shields.io/badge/FastAPI-0.115-009688?logo=fastapi)
![XGBoost](https://img.shields.io/badge/XGBoost-ML-orange)
![Three.js](https://img.shields.io/badge/Three.js-3D-black?logo=three.js)

## Overview

DDS takes real OBD-II telemetry data (speed, RPM, fuel rate, coolant temperature, CO₂ emissions) and builds a complete driving-decision inference pipeline:

- **ML Classification** — XGBoost classifier trained on real OBD-II data to infer driving decisions (Accelerate / Decelerate / Maintain Speed)
- **Explainability** — Real-time SHAP explanations computed at 10 Hz via TreeExplainer, showing which telemetry features drive each decision
- **Anomaly Detection** — Isolation Forest + per-feature range checks for out-of-distribution input detection
- **Confidence-Gated Safety** — Automatic fallback to safe actions when model confidence drops below threshold
- **Physics Simulation** — Kinematic bicycle model with jerk-limited longitudinal control, Frenet-frame local planning, and pure-pursuit lateral control
- **Real Road Routing** — OSRM-powered route following with centripetal Catmull-Rom path smoothing
- **Simulated Traffic** — Server-side NPC vehicles with forward range sensing
- **3D Visualisation** — Tesla FSD-inspired browser HMI built with React Three Fiber, featuring chase camera, HUD, road geometry, and live sensor rays

## Architecture

```
OBD-II Telemetry (10 Hz)
    │
    ▼
┌─────────────────────────────────────────┐
│  Inference Service (FastAPI + WebSocket) │
│                                         │
│  ┌──────────┐  ┌──────────┐  ┌───────┐ │
│  │ XGBoost  │  │ Anomaly  │  │ SHAP  │ │
│  │Classifier│  │ Detector │  │Explain│ │
│  └────┬─────┘  └────┬─────┘  └───┬───┘ │
│       └──────┬───────┘            │     │
│              ▼                    │     │
│     Confidence Gate ◄─────────────┘     │
│              │                          │
│              ▼                          │
│     Physics Engine                      │
│     (Bicycle + Frenet + Pure Pursuit)   │
│              │                          │
│              ▼                          │
│     WebSocket Stream (10 Hz)            │
└──────────────┬──────────────────────────┘
               │
               ▼
┌──────────────────────────────────────┐
│  3D Browser HMI (Next.js + Three.js) │
│  Road · Car · Traffic · HUD · SHAP  │
└──────────────────────────────────────┘
```

## Project Structure

```
DDS/
├── app/                        # Backend (FastAPI)
│   ├── api/
│   │   ├── rest.py             # REST endpoints (/api/health, /api/predict)
│   │   └── websockets.py       # WebSocket streaming loop
│   ├── core/
│   │   ├── config.py           # App configuration
│   │   └── database.py         # SQLite telemetry storage
│   ├── services/
│   │   ├── anomaly_detector.py # Isolation Forest + range checks
│   │   ├── driver_scoring.py   # Driver behaviour scoring
│   │   ├── explainability.py   # SHAP TreeExplainer
│   │   ├── frenet.py           # Frenet frame projection
│   │   ├── inference.py        # ML inference + confidence gate
│   │   ├── path_smoothing.py   # Catmull-Rom route smoothing
│   │   ├── physics_engine.py   # Kinematic bicycle model + control
│   │   ├── planner.py          # Lateral offset planner
│   │   ├── routing.py          # OSRM route fetching
│   │   └── traffic.py          # NPC traffic simulation
│   └── main.py                 # App entrypoint
├── frontend/                   # Frontend (Next.js + Three.js)
│   └── src/app/
│       ├── components/
│       │   ├── DriveScene.tsx   # 3D driving scene
│       │   ├── DriveHUD.tsx     # Speed/gear/turn HUD overlay
│       │   ├── SimulatedTraffic.tsx
│       │   ├── SHAPPanel.tsx    # Live SHAP visualisation
│       │   └── ...
│       └── page.tsx             # Main dashboard page
├── tests/                      # Backend test suite (pytest)
├── data_prep.py                # Dataset preprocessing
├── model_pipeline.py           # Model training & evaluation
├── genetic_optimizer.py        # Feature subset selection
├── baselines.py                # Baseline model comparisons
├── robustness_eval.py          # OOD robustness evaluation
├── OBD_2_dataset.csv           # Raw OBD-II telemetry dataset
└── requirements.txt            # Python dependencies
```

## Getting Started

### Prerequisites

- Python 3.13+
- Node.js 18+
- npm

### Backend Setup

```bash
# Create virtual environment
python -m venv venv

# Activate (Windows)
venv\Scripts\activate
# Activate (Linux/Mac)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Train the model (generates .pkl artifacts)
python model_pipeline.py

# Start the backend server
python -m app.main
```

The backend starts at `http://localhost:8000`. Verify with:
```bash
curl http://localhost:8000/api/health
```

### Frontend Setup

```bash
cd frontend
npm install
npm run dev
```

The frontend starts at `http://localhost:3000`.

### Running Tests

```bash
# Backend tests
pip install -r requirements-dev.txt
pytest tests/ -v

# Frontend type check
cd frontend && npx tsc --noEmit
```

## Tech Stack

| Layer | Technology |
|-------|-----------|
| ML Pipeline | XGBoost, scikit-learn, SHAP |
| Backend | FastAPI, WebSocket, SQLite |
| Physics | Custom kinematic bicycle model |
| Routing | OSRM (public demo API) |
| Frontend | Next.js 15, React Three Fiber, Three.js |
| UI | Tailwind CSS, Framer Motion |
| Testing | pytest, pytest-asyncio |
| CI | GitHub Actions |

## Dataset

The system uses a real OBD-II telemetry dataset (`OBD_2_dataset.csv`) containing ~900 timestamped readings with:
- Vehicle speed, RPM, fuel consumption rate
- Coolant temperature, CO₂ emissions
- Derived delta features (speed change, RPM change, etc.)

The driving decision label is derived from a ±2 km/h speed delta threshold, classifying each timestep as Accelerate, Decelerate, or Maintain Speed.

## License

This project is developed as an academic research project.
