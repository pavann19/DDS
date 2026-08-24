# DDS Frontend

3D driving visualisation dashboard built with Next.js 15, React Three Fiber, and Three.js.

## Setup

```bash
npm install
npm run dev
```

Opens at [http://localhost:3000](http://localhost:3000). Requires the backend running at `http://localhost:8000`.

## Key Components

| Component | Purpose |
|-----------|---------|
| `DriveScene.tsx` | 3D road geometry, ego car, chase camera |
| `DriveHUD.tsx` | Speed, gear, turn-by-turn HUD overlay |
| `SimulatedTraffic.tsx` | NPC vehicle rendering from backend state |
| `SHAPPanel.tsx` | Live SHAP feature attribution chart |
| `ConnectionStatus.tsx` | WebSocket connection state indicator |
| `LandingView.tsx` | Pre-connection landing screen |
| `TripSummary.tsx` | Trip statistics panel |
| `SettingsPanel.tsx` | User preferences (units, alerts) |

## Tech

- **Next.js 15** with App Router
- **React Three Fiber** + **Three.js** for 3D rendering
- **Framer Motion** for UI animations
- **Tailwind CSS** for styling
- **WebSocket** for real-time telemetry streaming
