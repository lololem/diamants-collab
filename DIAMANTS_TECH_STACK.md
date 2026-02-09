# DIAMANTS — Technical Stack

**Version:** V2 (June 2025)

---

## Runtime

| Component | Version | Notes |
|-----------|---------|-------|
| Node.js | 20.20.0 | via nvm (.nvmrc) |
| npm | 10.x | |
| Python | 3.12+ | ROS2 Jazzy default |

## Frontend

| Library | Version | Role |
|---------|---------|------|
| Vite | 4.5.3 | Build + HMR dev server |
| Three.js | 0.167.x | WebGL 3D rendering |
| ES6 modules | native | All imports via `import` |

Dev server: `http://localhost:5550` (configurable in `vite.config.js`).

## Backend

| Component | Role |
|-----------|------|
| ROS2 Jazzy | Message bus + node lifecycle |
| Gazebo Harmonic | Physics simulation (optional) |
| rclpy | Python ROS2 bindings |

### ROS2 Microservices (4 nodes)

| Node | Responsibility |
|------|----------------|
| SwarmController | Fleet state, formation logic |
| PositionBroadcaster | Telemetry aggregation + WebSocket fan-out |
| SLAMFusion | Map merge multi-drone |
| MissionCoordinator | Mission lifecycle, waypoint dispatch |

## API / Bridge

| Service | Port | Protocol |
|---------|------|----------|
| FastAPI REST | 8000 | HTTP |
| DiamantsBridge | 8765 | WebSocket |

The WebSocket bridge (`services/websocket_bridge.py`) relays bidirectional messages between the frontend and the ROS2 graph.

## Flight Engine (Frontend)

The `AutonomousFlightEngine` is the single source of truth for drone positions when no backend is connected (CAS 2 mode).

| Module | File | Role |
|--------|------|------|
| PID controller | `autonomous-flight-engine.js` | Position + altitude + yaw |
| Drone profiles | `physics/profiles/*.json` | Per-model parameters |
| DronePhysicsRegistry | `drone-physics-registry.js` | Singleton profile loader |
| SwarmIntelligenceInterface | `intelligence/swarm-intelligence-interface.js` | Abstract slot for swarm algorithms |

### Drone Profiles

Profiles are JSON files loaded at runtime. Each profile defines: mass, arm length, max speed, cruise altitude, PID gains, visual scale.

| Profile | File | Category |
|---------|------|----------|
| Crazyflie 2.1 | `crazyflie-2.1.json` | nano |
| Mavic Pro | `mavic-pro.json` | consumer |
| Phantom 4 | `phantom-4.json` | consumer |

Schema: `profiles/drone-profile.schema.json`.

## Protocol

Unified telemetry contract: `drone-state.schema.json` (v1.0.0).

Fields: `drone_id`, `timestamp`, `position`, `velocity`, `attitude`, `battery`, `armed`, `mode`, `status`, `gps`, `source`, `profile`, `sysid`.

## Ports Summary

| Port | Service |
|------|---------|
| 5550 | Vite dev server (frontend) |
| 8000 | FastAPI REST API |
| 8765 | WebSocket bridge (DiamantsBridge) |

## Dependencies

### Frontend (package.json)

```
three  ^0.167.0
vite   ^4.5.3
```

### Backend (ROS2 packages)

```
crazyflie_interfaces
multi_agent_framework
slam_map_merge
ros_gz_crazyflie_*
```

### API (requirements.txt)

```
fastapi
uvicorn
websockets
rclpy (system — ROS2)
```

## Build & Run

```bash
# Frontend
cd DIAMANTS_FRONTEND && npm run dev

# API + Bridge
cd DIAMANTS_API && ./start.sh

# Full stack
./launch_diamants.sh
```

See [Launch-Guide.md](Launch-Guide.md) for details.
