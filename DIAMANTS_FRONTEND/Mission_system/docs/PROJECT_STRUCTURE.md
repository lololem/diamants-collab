# Project structure — Mission_system

What lives where in the simulator. Generated from the repository, not from memory:
every path below exists.

## Root

Seven files, and nothing else.

```
Mission_system/
├── index.html          # web entry point
├── index-v2.html       # alternative layout
├── main.js             # JavaScript entry point
├── package.json        # npm configuration and scripts
├── package-lock.json   # locked dependencies
├── vite.config.js      # Vite configuration
└── fleet_config.json   # default fleet composition
```

## Modules

### `core/` — engine internals

```
cas-controller.js          collision-avoidance system
config.js                  global configuration
diamants-formulas.js       field metrics (public stub — returns zeros)
diamants-initializer.js    startup sequencing
field-visualizer-3d.js     field visualisation and harmonics HUD
flow-particles.js          flow particles
fractal-hypervision.js     fractal view
fractal-layers.js          fractal layers
logger.js                  logging
sssp-algorithm.js          shortest paths (Duan et al. 2025)
swarm-memory.js            shared swarm memory
```

### `physics/` — flight

```
autonomous-flight-engine.js   the flight engine: state machine, cascaded PID
pid-controller.js             a single PID axis
collision-detection.js        obstacle and drone collision tests
drone-physics.js              flight physics
drone-physics-registry.js     profile registry
realistic-flight-dynamics.js  aerodynamic model
x500-flight-dynamics.js       X500-specific dynamics
flight-config.json            engine defaults
profiles/                     one JSON per airframe, plus the schema
```

Five airframes ship: `crazyflie-2.1`, `mavic-pro`, `phantom-4`, `s500`,
`x500-v2`. Each is validated against `profiles/drone-profile.schema.json`.

### `intelligence/` — swarm behaviour

```
swarm-intelligence-interface.js  the contract every engine implements
stigmergy-interface.js           the pheromone-grid contract
stigmergy-loader.js              loads a stigmergy engine at runtime
collective-intelligence.js       collective behaviours
advanced-collective-intelligence.js  attractors, emergent leadership
drone-pathfinder.js              path planning
environment-voxelizer.js         voxel view of the environment
```

Seven modules in this folder and in `core/` are public stubs — the right shape,
callable, but inert: `stigmergy-engine.js`, `distributed-swarm-engine.js`,
`swarm-comm-manager.js`, `drone-intelligence.js`, `scenario-engine.js`,
`optimized-search.js` and `core/diamants-formulas.js`. See the README section
*What is not included*.

### `agent/` — agent decision-making

```
autonomous-agent.js         the agent loop, frontier scoring, CBBA-style bidding
agent-brain.js              brain dispatch
reactive-brain.js           reactive policy
cognitive-brain.js          deliberative policy
brain-interface.js          brain contract
agent-perception.js         what an agent can sense
agent-communication.js      inter-agent messages
agent-drone-registry.js     agent-to-drone binding
multi-agent-coordinator.js  fleet-level coordination
raft-consensus.js           Raft consensus
gymnasium-env.js            Gymnasium-style RL environment
reward-shaper.js            reward shaping
```

### `ui/` — panels and maps

```
diamant-ui.js                  main interface
diamants-ui-controller.js      UI controller
panel-controller.js            panel management
panel-utils.js                 shared panel helpers
orchestration-console.js       command console
drone-select-panel.js          drone picker
follow-drone-fab.js            follow-camera control
llm-chat-panel.js              LLM intelligence panel
marl-training-panel.js         MARL training panel
optimal-search-ui.js           search interface
comm-panel.js                  communications panel
fractal-hypervision-panel.js   fractal view panel
quality-control-panel.js       (in environment/) rendering quality
```

The six live maps:

```
exploration-minimap.js         stigmergy heat-trail
perception-minimap.js          SLAM reconstruction
discovery-minimap.js           coverage occupancy grid
sitac-minimap.js               tactical radar
federated-learning-minimap.js  federated RL
comm-panel.js                  peer-to-peer link graph
```

### The rest

```
missions/      doctrine catalogue, mission lifecycle, import/export
behaviors/     flight patterns, collaborative scouting
drones/        3D models and the visual factory
environment/   terrain, sky, grass, undergrowth, trees
shaders/       GLSL for grass and sky
sensors/       depth camera and multi-ranger simulation
services/      LLM mission service, view persistence
net/           ROS bridge
controllers/   Crazyflie ROS controller
visual/        visual effects, communication waves
tools/         top-level controller, benchmarks, THREE bootstrap, beacons
assets/        meshes and textures
styles/        CSS
third-party/   bundled dependencies, each under its own licence
```

## Tests

```
tests/
├── doctrine-combinatorics.test.js  doctrine × course-of-action matrix
├── ros-bridge.test.js              bridge contract
├── button-test-suite.js            UI button coverage
├── diagnostic.html                 diagnostic page
├── webgl-test.html                 WebGL capability check
├── physics-smoke.html              physics smoke test
└── *.cjs                           Puppeteer-driven browser checks
```

`npm test` runs the Vitest suites.

## Commands

```bash
npm run dev        # development server, http://localhost:5550
npm run build      # production build
npm test           # test suite
npm run coverage   # test suite with coverage
npm run preview    # serve the production build
```
