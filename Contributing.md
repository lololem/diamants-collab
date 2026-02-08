# Contributing

## How to contribute

1. Fork the repository
2. Create a branch: `git checkout -b feature/your-feature`
3. Make your changes
4. Test: `npm run dev` (frontend), `npm test` (unit tests)
5. Commit with a clear message
6. Open a pull request

## What you can contribute

### 1. A drone profile

Add your drone to the simulation by creating a JSON file.

**File:** `DIAMANTS_FRONTEND/Mission_system/physics/profiles/your-drone.json`

**Schema:** Follow `drone-profile.schema.json` in the same directory.

**Required fields:**

| Field | Type | Description |
|-------|------|-------------|
| `id` | string | Unique ID, uppercase with underscores (e.g. `X500_V2`) |
| `label` | string | Human-readable name |
| `physical.mass` | number | Mass in kg |
| `physical.boundingRadius` | number | Collision sphere radius in meters |
| `performance.maxSpeed` | number | Maximum horizontal speed in m/s |
| `performance.maxAlt` | number | Maximum altitude in meters |
| `performance.cruiseAlt` | number | Default cruising altitude in meters |
| `pid.pos` | object | Position PID gains: `{ kp, ki, kd }` |
| `pid.alt` | object | Altitude PID gains: `{ kp, ki, kd }` |
| `pid.yaw` | object | Yaw PID gains: `{ kp, ki, kd }` |

**Optional fields:** `manufacturer`, `category` (nano/micro/compact/medium/heavy/custom), `performance.agility`, `performance.explorationRadius`, `performance.endurance_min`, `visual.scale`, `visual.color`, `visual.model`.

**Example — minimal profile:**

```json
{
    "id": "X500_V2",
    "label": "Holybro X500 V2",
    "physical": {
        "mass": 1.2,
        "boundingRadius": 0.5
    },
    "performance": {
        "maxSpeed": 8.0,
        "maxClimb": 3.0,
        "cruiseAlt": 10.0,
        "maxAlt": 50.0
    },
    "pid": {
        "pos": { "kp": 2.0, "ki": 0.05, "kd": 1.0 },
        "alt": { "kp": 3.0, "ki": 0.1, "kd": 1.5 },
        "yaw": { "kp": 2.0, "ki": 0.0, "kd": 0.3 }
    }
}
```

After adding the file, register it in `drone-physics-registry.js`:

```javascript
import myProfile from './profiles/your-drone.json';
// Add to BUILT_IN array
```

Or leave it as a standalone file — the registry can load profiles dynamically.

### 2. A swarm intelligence algorithm

Implement the `SwarmIntelligenceInterface` abstract class.

**File:** `DIAMANTS_FRONTEND/Mission_system/intelligence/your-algorithm.js`

**Interface contract:**

```javascript
import { SwarmIntelligenceInterface } from './swarm-intelligence-interface.js';

export class YourAlgorithm extends SwarmIntelligenceInterface {
    constructor() {
        super();
        this.name = 'your-algorithm';
        this.version = '1.0.0';
    }

    // Called once at startup
    initialize(config) {
        // config = { droneCount, arena: { width, height }, profiles }
    }

    // Called every frame (60 Hz)
    // Return Map<droneId, influence>
    // influence = { targetModifier: Vector3, velocityBias: Vector3, priorityOverride: string }
    computeInfluences(droneStates, dt) {
        const influences = new Map();
        // Your algorithm here
        return influences;
    }
}
```

**Rules:**
- `computeInfluences` must return within 1-2ms (it runs every frame)
- Return an empty Map if you have no suggestions
- Influences are merged with the PID — you suggest, the PID decides
- You have read-only access to all drone states (position, velocity, target, profile)

**How to test:** Replace `NoopSwarmIntelligence` in `autonomous-flight-engine.js` with your implementation.

### 3. A WebSocket adapter

Write a program that:
1. Opens a WebSocket server on port 8765
2. Sends `drone_positions` messages (see `src/protocols/drone-state.schema.json`)
3. Receives commands from the frontend

Any language works. The protocol is JSON over WebSocket. See [Architecture.md](Architecture.md) for the message format and coordinate system.

### 4. Bug fixes, tests, documentation

Standard open-source workflow:
- Check [issues](https://github.com/lololem/diamants-collab/issues) for known bugs
- Run `npm test` to verify your changes
- Write tests for new features (Vitest)
- Keep comments in English in the code

## Code conventions

### JavaScript (frontend)

- ES6 modules (`import`/`export`, no `require`)
- camelCase for variables and functions
- PascalCase for classes
- File names: kebab-case (e.g. `drone-physics-registry.js`)
- No global variables (`window.something`) — use the event bus
- JSDoc comments for public methods

### Python (backend)

- Python 3.12+
- Type hints on function signatures
- PEP 8
- Docstrings for public functions
- Use `asyncio` for I/O-bound operations

### JSON profiles

- `id` field: UPPER_CASE
- All physical values in SI units (kg, m, m/s)
- PID gains: use values appropriate for the drone's mass and responsiveness

## Development setup

```bash
# Frontend only (sufficient for most contributions)
cd DIAMANTS_FRONTEND/Mission_system
npm install
npm run dev      # Dev server with HMR
npm test         # Run tests
npm run build    # Verify build passes

# Backend (only if working on ROS2 integration)
# Requires Ubuntu 22.04+, ROS2 Jazzy
cd DIAMANTS_BACKEND
./setup.sh
```

## Pull request checklist

- [ ] Code compiles / builds without errors
- [ ] `npm run build` passes (frontend)
- [ ] Existing tests pass
- [ ] New tests added for new features
- [ ] No hardcoded values — use config or profiles
- [ ] No console.log left in production code
- [ ] Commit message describes what changed and why

## Contact

Questions: open an issue or email loic.lemasle@gmail.com.
