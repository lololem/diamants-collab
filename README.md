# DIAMANTS — Distributed Autonomous Multi-Agent Systems

Open-source framework for multi-drone simulation, visualization, and real-world deployment via MAVLink/PX4.

DIAMANTS is an infrastructure: a simulation + hypervision platform where anyone can plug their own drone type, flight controller, and swarm intelligence algorithm. You write a JSON file for your drone, you implement a JavaScript interface for your algorithm, and the system handles the rest — 3D rendering, telemetry, collision avoidance, WebSocket communication.

## What it does

- 3D simulation of heterogeneous drone swarms (Crazyflie, Mavic, Phantom, or your custom drone)
- PID-based autonomous flight engine running at 60 FPS in the browser
- Plugin system for drone profiles (JSON) and swarm intelligence (JS interface)
- WebSocket protocol for real-time telemetry between gateway and frontend
- Optional ROS2/Gazebo backend for hardware-in-the-loop simulation
- Designed to connect to real PX4 drones via MAVLink gateway

## Current state

The frontend works standalone: 8 drones fly autonomously with PID control, tree collision avoidance, and inter-drone separation. No backend required. When a WebSocket source is available (ROS2 backend or MAVLink gateway), the frontend receives real telemetry and switches to tracking mode.

Three operating modes:
1. **Standalone** — Frontend PID engine drives all drones (default, works out of the box)
2. **ROS2/Gazebo** — Backend publishes positions via WebSocket bridge
3. **MAVLink** — Gateway connects to real/SITL PX4 drones and forwards telemetry

## Quick start

```bash
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab/DIAMANTS_FRONTEND/Mission_system
npm install
npm run dev
```

Open the browser at the URL shown (default: http://localhost:5550). Drones take off and explore autonomously.

## Project structure

```
diamants-collab/
  DIAMANTS_FRONTEND/Mission_system/    # Main application
    physics/
      autonomous-flight-engine.js      # PID flight engine (core)
      drone-physics-registry.js        # Loads drone profiles
      pid-controller.js                # PID controller implementation
      profiles/                        # Drop your drone JSON here
        crazyflie-2.1.json
        mavic-pro.json
        phantom-4.json
        drone-profile.schema.json      # Schema for validation
    intelligence/
      swarm-intelligence-interface.js  # Abstract interface for your algorithm
      collective-intelligence.js       # Built-in collective behaviors
    net/
      ros-bridge-simple.js             # WebSocket client (telemetry receiver)
    src/protocols/
      drone-state.schema.json          # WebSocket message format specification
    visual/                            # Three.js 3D rendering
    core/                              # Application state, events
    ui/                                # Control panels
    behaviors/                         # Flight patterns
    controllers/                       # Camera, input handling
  DIAMANTS_BACKEND/                    # ROS2 Jazzy + Gazebo (optional)
    ros2_microservices/                # 4 ROS2 nodes
    slam_collaboratif/                 # Collaborative SLAM workspace
  DIAMANTS_API/                        # FastAPI WebSocket bridge (optional)
```

## Add your drone

1. Create a JSON file in `DIAMANTS_FRONTEND/Mission_system/physics/profiles/`:

```json
{
    "id": "MY_DRONE",
    "label": "My Custom Drone",
    "manufacturer": "Your Company",
    "category": "compact",
    "physical": {
        "mass": 0.5,
        "armLength": 0.15,
        "boundingRadius": 0.4,
        "propCount": 4
    },
    "performance": {
        "maxSpeed": 5.0,
        "maxClimb": 2.0,
        "cruiseAlt": 5.0,
        "maxAlt": 20.0,
        "agility": 1.2,
        "explorationRadius": 80,
        "endurance_min": 15
    },
    "pid": {
        "pos":  { "kp": 2.5, "ki": 0.05, "kd": 1.0 },
        "alt":  { "kp": 3.5, "ki": 0.1,  "kd": 1.2 },
        "yaw":  { "kp": 2.0, "ki": 0.0,  "kd": 0.3 }
    },
    "visual": {
        "scale": 20,
        "color": "0xFF6600",
        "model": "generic"
    }
}
```

The schema is documented in `profiles/drone-profile.schema.json`. Required fields: `id`, `label`, `physical`, `performance`, `pid`. The engine loads all JSON files in the profiles directory at startup.

2. Register it in `drone-physics-registry.js` (add the import and push to `BUILT_IN`), or load it dynamically at runtime.

## Add your swarm intelligence

Implement `SwarmIntelligenceInterface` from `intelligence/swarm-intelligence-interface.js`:

```javascript
import { SwarmIntelligenceInterface } from './swarm-intelligence-interface.js';

export class MySwarmAlgorithm extends SwarmIntelligenceInterface {
    constructor() {
        super();
        this.name = 'my-algorithm';
        this.version = '1.0.0';
    }

    initialize(config) {
        // Called once at startup with { droneCount, arena, profiles }
    }

    computeInfluences(droneStates, dt) {
        // Called every frame. droneStates = Map<id, {position, velocity, target, ...}>
        // Return a Map<id, {targetModifier, velocityBias, priorityOverride}>
        // Return empty map for no influence
        return new Map();
    }
}
```

The engine calls `computeInfluences()` every frame and merges your outputs with the PID controller. You observe, you suggest — the PID decides.

## WebSocket protocol

The frontend expects messages on WebSocket port 8765 in this format:

```json
{
    "type": "drone_positions",
    "drones": {
        "drone_1": {
            "position": { "x": 1.0, "y": 2.5, "z": 3.0 },
            "velocity": { "x": 0.1, "y": 0.0, "z": 0.0 },
            "battery": 85,
            "status": "exploring",
            "armed": true,
            "mode": "OFFBOARD",
            "source": "mavlink"
        }
    }
}
```

Full specification: `DIAMANTS_FRONTEND/Mission_system/src/protocols/drone-state.schema.json`

When the frontend receives a valid `drone_positions` message, it switches from PID simulation to real telemetry tracking. If the WebSocket drops, it falls back to PID mode with exponential backoff reconnection.

## Backend (optional)

The ROS2 backend requires Ubuntu 22.04+, ROS2 Jazzy, and optionally Gazebo Harmonic. It runs 4 microservice nodes:

- SwarmController — swarm-level coordination
- PositionBroadcaster — publishes drone positions to WebSocket
- SLAMFusion — merges maps from multiple drones
- MissionCoordinator — mission planning and execution

```bash
cd DIAMANTS_BACKEND
./setup.sh
./launch_slam_collaborative.sh
```

The backend is not required for the frontend to work. It adds physics simulation (Gazebo) and SLAM capabilities.

## Tech stack

| Component | Technology | Version |
|-----------|-----------|---------|
| 3D Engine | Three.js | 0.167.x |
| Build | Vite | 4.5.3 |
| Test | Vitest | 1.6.x |
| Runtime | Node.js | 20.x (nvm) |
| Backend | ROS2 Jazzy | optional |
| Simulation | Gazebo Harmonic | optional |
| Language (frontend) | ES6 modules | — |
| Language (backend) | Python 3.12 | — |

## Videos

- [Backend ROS2 SLAM](https://www.youtube.com/watch?v=iYg9Jf1jv4Y) — 8 Crazyflie drones, collaborative mapping
- [Frontend 3D](https://www.youtube.com/watch?v=fyEmYu4lbzo) — Three.js interface
- [Multi-Agent Systems](https://www.youtube.com/watch?v=1Av_o-9fzrE) — Distributed coordination
- [Gradient Navigation](https://www.youtube.com/watch?v=ElABxOde6ak) — Path planning
- [Swarm Coordination](https://www.youtube.com/watch?v=L8V64LajM2w) — Formation control
- [Stigmergy Demo](https://www.youtube.com/watch?v=SyqeRwcbDO4) — Bio-inspired coordination

Additional demos and video files in `DEMO/`.

## Documentation

- [Architecture.md](Architecture.md) — System design, data flow, adapter pattern
- [Contributing.md](Contributing.md) — How to add drones, algorithms, and contribute
- [Installation.md](Installation.md) — Full installation guide with ROS2/Gazebo
- [Launch-Guide.md](Launch-Guide.md) — Running the system
- [Known-Issues.md](Known-Issues.md) — Current limitations

## Contributing

We are building this as a community framework. Contributions are welcome:

- Add a drone profile (JSON file + optional 3D model)
- Implement a swarm intelligence algorithm (extend the JS interface)
- Write a MAVLink adapter for your flight controller
- Improve the 3D visualization
- Fix bugs, write tests, improve documentation

Fork, branch, PR. See [Contributing.md](Contributing.md) for details.

## License

MIT. See [LICENSE](LICENSE).

## Contact

- Issues: [github.com/lololem/diamants-collab/issues](https://github.com/lololem/diamants-collab/issues)
- Email: loic.lemasle@gmail.com
