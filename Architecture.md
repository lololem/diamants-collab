# Architecture

## Overview

DIAMANTS has three layers. Only the first is required.

```
┌─────────────────────────────────────────────────────────┐
│  FRONTEND (always)                                      │
│  Vite + Three.js + WebGL                                │
│  Autonomous Flight Engine (PID, 60 FPS)                 │
│  Drone Physics Registry (JSON profiles)                 │
│  Swarm Intelligence Interface (plugin slot)             │
│                                                         │
│        ▼ WebSocket (port 8765) — optional ▼             │
├─────────────────────────────────────────────────────────┤
│  ADAPTER (one of, optional)                             │
│  ┌───────────────┐  ┌──────────────────────────┐        │
│  │ ROS2 Backend  │  │ MAVLink Gateway          │        │
│  │ + Gazebo      │  │ pymavlink + websockets   │        │
│  │ + FastAPI     │  │ PX4 real/SITL            │        │
│  └───────────────┘  └──────────────────────────┘        │
├─────────────────────────────────────────────────────────┤
│  DRONES (physical or simulated)                         │
│  Crazyflie 2.1  |  Mavic Pro  |  PX4 SITL  |  Custom   │
└─────────────────────────────────────────────────────────┘
```

The frontend is fully autonomous. Without any WebSocket source, it runs its own PID engine and simulates drone flights in 3D. When a telemetry source connects on port 8765, the frontend switches to tracking mode and renders real positions.

## Three operating modes (CAS)

The frontend uses a 3-tier fallback system internally called CAS (Cascade Autonomy System):

| Mode | Trigger | What happens |
|------|---------|-------------|
| CAS 1 — Backend | WebSocket connected, receiving `drone_positions` | Frontend renders positions from external source |
| CAS 2 — Autonomous | WebSocket down or no backend | AutonomousFlightEngine runs PID at 60 FPS |
| CAS 3 — Minimal | Engine fails to initialize | Basic position update, no PID |

Transition is automatic. If the WebSocket drops, the frontend falls back to CAS 2 with exponential backoff reconnection (1s, 2s, 4s, ..., 30s max).

## Frontend architecture

```
Mission_system/
  main.js                          # Entry point, initializes everything
  core/
    diamant-system.js              # System orchestrator
    app-state.js                   # Global state
    event-manager.js               # Event bus
  physics/
    autonomous-flight-engine.js    # PID flight engine (CAS 2)
    drone-physics-registry.js      # Singleton, loads profiles
    pid-controller.js              # Generic PID controller
    drone-physics.js               # Physics calculations
    profiles/                      # JSON drone definitions
  intelligence/
    swarm-intelligence-interface.js # Abstract plugin interface
    collective-intelligence.js     # Built-in behaviors
  net/
    ros-bridge-simple.js           # WebSocket client
  visual/
    three-visualizer.js            # Scene, camera, lights
    drone-visual.js                # Drone 3D meshes
    environment-renderer.js        # Terrain, trees, buildings
  ui/
    control-panel.js               # UI overlays
  controllers/
    camera-controller.js           # Orbit camera
  behaviors/
    flight-behaviors.js            # Exploration, patrol, RTL patterns
  src/protocols/
    drone-state.schema.json        # WebSocket message format
```

### Data flow (CAS 2 — standalone)

```
main.js
  → AutonomousFlightEngine.initialize()
      → DronePhysicsRegistry.load() — reads JSON profiles
      → SwarmIntelligenceInterface.initialize()
  → render loop (60 FPS):
      1. engine.update(dt)
         a. For each drone: PID computes thrust from (position, target)
         b. SwarmIntelligence.computeInfluences(droneStates, dt)
         c. Merge PID output + swarm influences
         d. Apply collision avoidance (trees, other drones)
         e. Integrate velocity → new position
      2. visualizer.render(droneStates)
```

### Data flow (CAS 1 — with backend)

```
External source (ROS2 backend or MAVLink gateway)
  → WebSocket message: { type: "drone_positions", drones: {...} }
  → ros-bridge-simple.js receives and parses
  → Updates drone positions in scene
  → visualizer.render()
```

## Drone Physics Registry

The registry is a singleton that loads drone profiles from JSON files. Each profile defines:

- Physical properties (mass, arm length, bounding radius)
- Performance envelope (max speed, max altitude, agility)
- PID gains (position, altitude, yaw)
- Visual properties (scale, color, 3D model reference)

Built-in profiles: Crazyflie 2.1, Mavic Pro, Phantom 4.

To add a drone, drop a JSON file in `physics/profiles/` following `drone-profile.schema.json`. The registry normalizes missing optional fields with defaults.

The registry does NOT control real drones. PX4 does its own physics. The registry informs the frontend about each drone's expected flight envelope for visualization and simulation purposes.

## Swarm Intelligence Interface

The flight engine exposes 4 hooks for swarm intelligence:

1. `initialize(config)` — called once with drone count, arena size, profiles
2. `computeInfluences(droneStates, dt)` — called every frame, returns influence map
3. `onDroneAdded(id, profile)` — a drone joined
4. `onDroneRemoved(id)` — a drone left

The default implementation (`NoopSwarmIntelligence`) returns empty maps. To implement your own algorithm, extend `SwarmIntelligenceInterface` and register it with the engine.

Influences are suggestions, not commands. The PID controller has final authority. An influence can modify the target position, bias the velocity, or override priority — but the PID always enforces safety limits.

## WebSocket protocol

All adapters (ROS2 backend, MAVLink gateway, custom) must send messages in the format defined by `src/protocols/drone-state.schema.json`.

Core message types:

| Type | Direction | Description |
|------|-----------|-------------|
| `drone_positions` | adapter → frontend | Telemetry for all drones |
| `drone_command` | frontend → adapter | Command for one drone |
| `swarm_command` | frontend → adapter | Command for all drones |
| `mission_command` | frontend → adapter | Mission start/stop/update |
| `ping` / `pong` | bidirectional | Keepalive |

The `drone_positions` message contains a `drones` object keyed by drone ID. Each drone entry has: position (Three.js Y-up coordinates), velocity, battery percentage, status, armed flag, PX4 mode, GPS data, source identifier, and profile reference.

## Backend architecture (optional)

### ROS2 microservices

The backend runs 4 independent ROS2 nodes:

```
ros2_microservices/
  swarm_controller/        # Swarm-level coordination logic
  position_broadcaster/    # Publishes positions to WebSocket
  slam_fusion/             # Merges SLAM maps from multiple drones
  mission_coordinator/     # Mission planning and execution
```

Each node is a standalone ROS2 process. They communicate via ROS2 topics (DDS). The PositionBroadcaster node opens a WebSocket server on port 8765 and sends `drone_positions` messages to the frontend.

### Gazebo integration

Gazebo Harmonic provides physics simulation. The backend spawns drone models in Gazebo, which publishes sensor data (IMU, range, camera) and receives velocity commands. The ROS2-Gazebo bridge (`ros_gz_bridge`) translates between Gazebo transport and ROS2 topics.

Gazebo is optional. The backend can run without it in "headless" mode, reading positions from MAVLink SITL instances instead.

### SLAM

The `slam_collaboratif` workspace contains packages for multi-drone SLAM:
- Individual drone mapping (each drone builds its own map)
- Map merging (a fusion node combines maps from all drones)
- Frontier detection (identifies unexplored areas)

## Adapter pattern

Anyone can write an adapter that connects to the frontend. The contract is simple:

1. Open a WebSocket server on port 8765
2. Send `drone_positions` messages at regular intervals (10-50 Hz)
3. Receive `drone_command` / `swarm_command` / `mission_command` messages
4. Translate them to your drone's native protocol

Examples of adapters:
- ROS2 backend (included) — reads ROS2 topics, sends to WebSocket
- MAVLink gateway — reads pymavlink, sends to WebSocket
- Custom adapter — any language, any drone, same WebSocket contract

## Coordinate systems

The frontend uses Three.js coordinates (Y-up, right-handed):
- X: right
- Y: up
- Z: forward (toward camera)

PX4/MAVLink uses NED (North-East-Down):
- X: north
- Y: east
- Z: down

The adapter is responsible for the conversion. The formula:
```
three_x =  ned_y   (east → right)
three_y = -ned_z   (down → up, inverted)
three_z =  ned_x   (north → forward)
```
