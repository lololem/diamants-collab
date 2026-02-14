# DIAMANTS — Distributed Autonomous Multi-Agent Systems

Open-source 3D simulation and visualization platform for multi-drone swarms.

DIAMANTS is an infrastructure: a simulation + visualization platform where anyone can plug their own drone type
and swarm intelligence algorithm. You write a JSON file for your drone, you implement a JavaScript interface
for your algorithm, and the system handles the rest — 3D rendering, PID control, collision avoidance,
autonomous exploration.

## What it does

- 3D simulation of heterogeneous drone swarms (Crazyflie, Mavic, Phantom, or your custom drone)
- PID-based autonomous flight engine running at 60 FPS in the browser
- Plugin system for drone profiles (JSON) and swarm intelligence (JS interface)
- Doctrine system for switching swarm behaviors in real time
- Exploration minimap with coverage tracking
- Designed for research, education, and algorithm prototyping

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
    behaviors/                         # Flight patterns and scouting
    missions/                          # Mission management and doctrines
    environment/                       # 3D Provençal landscape
    drones/                            # Drone visuals (Crazyflie model)
    visual/                            # Three.js rendering enhancements
    core/                              # Application state, events, formulas
    ui/                                # Control panels, minimap, debug
  DEMO/                                # Video demos and sample data
  Collective-Intelligence.md           # Theory behind swarm formulas
```

## Add your drone

Create a JSON file in `DIAMANTS_FRONTEND/Mission_system/physics/profiles/`:

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

## Tech stack

| Component | Technology | Version |
|-----------|-----------|---------|
| 3D Engine | Three.js | 0.167.x |
| Build | Vite | 4.5.3 |
| Test | Vitest | 1.6.x |
| Runtime | Node.js | 20.x (nvm) |
| Language | ES6 modules | — |

## Videos

- [3D Frontend Demo](https://www.youtube.com/watch?v=fyEmYu4lbzo) — Three.js interface
- [Multi-Agent Systems](https://www.youtube.com/watch?v=1Av_o-9fzrE) — Distributed coordination
- [Gradient Navigation](https://www.youtube.com/watch?v=ElABxOde6ak) — Path planning
- [Swarm Coordination](https://www.youtube.com/watch?v=L8V64LajM2w) — Formation control
- [Stigmergy Demo](https://www.youtube.com/watch?v=SyqeRwcbDO4) — Bio-inspired coordination

Additional demos and video files in `DEMO/`.

## Documentation

- [Collective-Intelligence.md](Collective-Intelligence.md) — Theory behind swarm formulas
- [Contributing.md](Contributing.md) — How to add drones, algorithms, and contribute

## Contributing

We are building this as a community framework. Contributions are welcome:

- Add a drone profile (JSON file + optional 3D model)
- Implement a swarm intelligence algorithm (extend the JS interface)
- Improve the 3D visualization
- Fix bugs, write tests, improve documentation

Fork, branch, PR. See [Contributing.md](Contributing.md) for details.

## License

MIT. See [LICENSE](LICENSE).

## Contact

- Issues: [github.com/lololem/diamants-collab/issues](https://github.com/lololem/diamants-collab/issues)
- Email: loic.lemasle@gmail.com
