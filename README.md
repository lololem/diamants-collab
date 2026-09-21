# DIAMANTS

**Fly a drone swarm in your browser, and plug in your own intelligence.**

A fleet takes off from a helipad in a forest and explores on its own. Rendering,
flight physics and collision avoidance are handled. You bring the coordination
algorithm — **and your own drone and your own model**: a language model, a
reinforcement-learning policy, any decision maker, kept in check by symbolic
rules. See [Bring your own model](#bring-your-own-model-neurosymbolic).

> 🎬 **New: two demonstration films (September 2026)** — [▶ watch them online](https://lololem.github.io/diamants-collab/), or see [Demonstration films](#demonstration-films-september-2026) below.
> They were recorded from the current internal development version: **the code published here is not up to date** and does not include every capability shown in them.
>
>
> [![Live demo — diamants-hypervision](https://img.shields.io/badge/Live%20demo-diamants--hypervision-4fd8ff?style=for-the-badge&logo=cloudflarepages&logoColor=white&labelColor=0e151d)](https://diamants-hypervision.pages.dev) [![Watch the demonstration films](https://img.shields.io/badge/Watch%20the%20films-online%20player-48e0a0?style=for-the-badge&logo=githubpages&logoColor=white&labelColor=0e151d)](https://lololem.github.io/diamants-collab/)
>
> <sub>The live demo lets you try DIAMANTS in your browser — it is not up to date either compared with the demonstration films.</sub>

### Why wildfire

A forest fire is cheap to stop and ruinous to chase. In its first minutes one
vehicle and a few hundred litres are enough; an hour later it takes a squadron,
and the outcome no longer depends on you. The difficulty was never the water —
it is knowing *where* to send it, across terrain that swallows radio and has no
roads, faster than the front moves.

That is a search problem before it is a firefighting one, and it is a poor fit
for a single operator watching a single feed. It suits a swarm: many cheap
sensors covering ground in parallel, a few better ones to confirm what the cheap
ones flagged, and heavy machines committed only once there is something real to
attack. Each tier does what it is actually equipped for, and no link in the
chain is allowed to become the one that breaks it.

This simulation runs that mission end to end.

https://github.com/lololem/diamants-collab/raw/main/docs/video/wildfire-mission.mp4

> **Eleven drones, two tracked UGVs, one wildfire — and nobody in charge.**
> Micro-drones sweep the forest and flag what they cannot identify. Camera
> platforms fly out, confirm the fire with a YOLO detector running on their own
> pixels, and put it on the radio. A ground vehicle elects itself, drives in, and
> does not open its water monitor until its thermal camera holds the flame.
> [How it works ↓](#wildfire-response-three-tiers-no-dispatcher)

> ⚠️ **Preview — this code is not in this repository yet.** The wildfire mission
> shown above (task network, onboard perception, ground vehicles, fire and
> suppression physics) is still being finalised and has not been published here.
> What you can run today is the swarm exploration simulator documented below.

> **PolyForm Noncommercial 1.0.0** — free for research, teaching, personal and
> non-profit use. Commercial use is not permitted. See [LICENSE](LICENSE).

![The simulation running: a heterogeneous fleet — two large X500/S500 quadcopters and several Crazyflie micro-drones — on a helipad in a forest, each drone showing its own status panel](docs/images/simulation.jpg)

*A heterogeneous fleet on the helipad. Every drone carries its own status panel
(id, phase, autonomy mode). Larger X500/S500 platforms and Crazyflie
micro-drones fly side by side, each with its own physics profile.*

![The mission control panel open on the left: Launch, Stop, Takeoff, Land and Return-to-Home buttons, a doctrine selector, beacon placement, scenario picker and an autonomy slider](docs/images/controls.jpg)

*The control panel. Launch a mission, switch flight doctrine and course of
action on the fly, drop beacons, or slide the autonomy from centrally guided to
fully distributed — where the agents coordinate with no central control.*

![The whole fleet lifting off the helipad, each drone carrying its own live panel: exploration phase, autonomy mode, current waypoint and a short rationale for its next move](docs/images/swarm.jpg)

*Launch, and the fleet fans out on its own. Every drone carries its own panel —
phase, waypoint, autonomy mode and a live rationale — so you can watch eleven
agents decide in parallel. Wire in your own algorithm and this is where it shows.*

![Follow-camera locked on one Crazyflie mid-flight, with its telemetry panel and a fleet cycler to jump between drones](docs/images/flight.jpg)

*Lock the camera onto any drone and ride along. The follow view cycles through
the whole fleet (1/11, 2/11 …) — handy for debugging a single agent's behaviour
while the rest keep exploring.*

---

## Wildfire response: three tiers, no dispatcher

> **Not shipped yet.** This mission is running, filmed and described here, but
> its code is still being finalised and is not part of this repository. Nothing
> below can be cloned and run today — treat it as a look ahead, not a feature
> list.

![A Colossus tracked UGV advancing through the forest while the onboard OAK-D camera view, bottom left, draws live YOLO detection boxes on smoke and fire](docs/images/wildfire-poster.jpg)

*The onboard view, bottom left: what the cognitive drone's OAK-D Pro W actually
sees, with the detector's boxes drawn on it and a stereo depth map in the
corner. [Full run, 2 min 28](docs/video/wildfire-mission.mp4).*

Three kinds of agent, three jobs. Nobody hands out assignments: every decision
is made locally, and the mission falls out of it.

**Crazyflie — survey.** The micro-drones carry no camera. They split the area
between themselves — each derives its own lane from its rank among its peers,
so the division needs no coordinator — and sweep it boustrophedon, lifting the
fog of war. Passing near a heat source, a Crazyflie raises an **unconfirmed
contact**. It cannot tell a fire from a hot roof, and it does not pretend to.

**X500 / S500 — inspect and confirm.** The cognitive platforms carry an OAK-D
Pro W depth camera. Each claims the nearest open contact — again by local
comparison — flies out to look, and runs a real YOLO detector on the pixels its
own camera produces. The boxes you see in the picture-in-picture come from
inference on those pixels, not from the scene graph. A fire is confirmed only
by the detector, together with a stereo range fix, and the confirmation goes out
on the radio.

**Colossus — suppress.** The tracked UGVs hear the broadcast and one elects
itself: the closest free vehicle takes the call. It drives to a standoff, and
its own thermal camera must hold flame in its field of view before the water
monitor opens — a radio report is a vector to the area, never a firing solution.
The jet is ballistic, and the fire goes out because water reaches it, not
because a timer expired. The rover then holds position, re-scans, and only
declares the fire out once its optics stay clear.

Information travels **upward** — Crazyflie to X500, X500 to Colossus — as
contacts and confirmations, never as orders coming down. Take any agent out and
the rest carry on.

---

## Live maps

The **Maps** button (bottom-right) opens a picker of six live views. Every one
is fed in real time by the drones themselves — open one, launch a mission, and
watch it fill in. No backend required.

![The Maps picker: a list of six views — SITAC detection, Exploration stigmergy, SLAM reconstruction, Discovery pixels, Federated RL and P2P communication](docs/images/maps-picker.jpg)

![Four maps side by side: a green stigmergy heat-trail of where the fleet has flown, a SLAM terrain reconstruction with a coverage percentage, a blue occupancy grid shaded by coverage, and a tactical radar with range rings and friend markers](docs/images/maps.jpg)

| View | What it shows |
|---|---|
| **Exploration — Stigmergy** | A pheromone-style heat-trail of everywhere the fleet has flown — the swarm's shared memory of the terrain. |
| **SLAM — Reconstruction** | The forest rebuilt from the drones' sensors, canopy and obstacles filling in with a live coverage %. |
| **Discovery — Pixels** | An occupancy grid, each cell shaded from *Low* to *Full* as the area gets covered. |
| **SITAC — Detection** | A tactical radar: range rings, headings, and friend / hazard / unknown markers. |
| **Federated — RL** | Model-sharing and convergence between the agents. |
| **Communication — P2P** | The peer-to-peer link graph as drones come in and out of range. |

Each view opens fullscreen, or docks as a small draggable, resizable minimap so
you can keep several on screen while a mission runs.

---

## Getting started

Node.js 20 or newer, and a browser with WebGL 2.

```bash
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab/DIAMANTS_FRONTEND/Mission_system
npm install
npm run dev
```

Vite prints the address to open, usually **http://localhost:5550**.

```bash
npm run build      # production build
npm test           # test suite
```

A black screen usually means hardware acceleration is off in the browser.

---

## What is not included

This repository publishes what you need to **fly a swarm, add your own drone
and plug your own model** — and nothing beyond it. The research work stays in a
private repository.

**What flies here:** the PID flight engine and its state machine (take-off,
exploration, obstacle avoidance, return, landing), the 3D environment, the
sensors, the panels and maps, the beacons, and the neurosymbolic contract with
its providers.

**What is a shell** — right shape, callable, inert:

| Module | Missing behaviour |
|---|---|
| `stigmergy-engine.js`, `stigmergy-loader.js` | no trace laid or followed; the tuned constants are not published |
| `distributed-swarm-engine.js` | no coordinated autonomous agents |
| `swarm-comm-manager.js` | drones do not share their map |
| `drone-intelligence.js` | the internal language-model bridge |
| `scenario-engine.js` | empty scenario list |
| `optimized-search.js` | hierarchical search absent |
| `core/diamants-formulas.js` | field metrics stay at zero |
| `intelligence/collective-intelligence.js`, `advanced-collective-intelligence.js` | consensus map, emergent patterns, attractors |
| `behaviors/collaborative-scouting.js` | trace-based coverage |
| `agent/multi-agent-coordinator.js`, `agent/brain-interface.js`, `agent/agent-drone-registry.js` | the trainable multi-agent layer: learning agents, the protocol they learn, cooperative reward, weight transfer |
| `ui/marl-training-panel.js` | the training console |
| `services/llm-mission-service.js` | turning an operator sentence into a mission |

Writing your own is the intended use. Contracts are in
`intelligence/agent-model-interface.js` (your model),
`intelligence/swarm-intelligence-interface.js` (your coordination),
`intelligence/stigmergy-interface.js` (your traces) and
`agent/brain-interface.js` (your decision maker).

The ROS 2 backend and WebSocket gateway are not published. The frontend runs
standalone.

The "LLM Intelligence" panel expects a local [Ollama](https://ollama.com) server.
Without one, and without your own models plugged in (see
[Bring your own model](#bring-your-own-model-neurosymbolic)), it shows
**simulated** decisions for demonstration — not model output.

The demonstration films were recorded on the internal build and show behaviour
this code does not have: shared traces, a distributed journal, on-board models.
They are there to show what the contracts are for, not what you get by cloning.

---

## Layout

```
DIAMANTS_FRONTEND/Mission_system/     the whole application
  physics/        PID flight engine, drone profiles
  intelligence/   swarm interfaces, shells, and the bring-your-own-model contract
    agent-model-interface.js    Observation → model → rules → decision
    neurosymbolic-bridge.js     connects your models to the running simulator
    model-providers/            ollama, openai-compatible, http-policy, example rules
  environment/    terrain, vegetation, sky
  shaders/        grass and sky (GLSL)
  drones/         3D models
  ui/             panels, minimaps
  assets/         meshes and textures
```

---

## Adding a drone

Drop a JSON file into `physics/profiles/` — the engine loads everything it finds
there at startup.

```json
{
    "id": "MY_DRONE",
    "label": "My Custom Drone",
    "physical":    { "mass": 0.5, "armLength": 0.15, "boundingRadius": 0.4, "propCount": 4 },
    "performance": { "maxSpeed": 5.0, "maxClimb": 2.0, "cruiseAlt": 5.0, "maxAlt": 20.0,
                     "agility": 1.2, "explorationRadius": 80, "endurance_min": 15 },
    "pid": {
        "pos": { "kp": 2.5, "ki": 0.05, "kd": 1.0 },
        "alt": { "kp": 3.5, "ki": 0.1,  "kd": 1.2 },
        "yaw": { "kp": 2.0, "ki": 0.0,  "kd": 0.3 }
    },
    "visual": { "scale": 20, "color": "0xFF6600", "model": "generic" }
}
```

`id`, `label`, `physical`, `performance` and `pid` are required. Full schema in
`profiles/drone-profile.schema.json`.

Restart `npm run dev` and your drone is in the registry: the fleet can be
composed with it, `DIAMANTS.spawnDrone('my_drone_01', 'MY_DRONE', {x, y, z})`
puts one in the air, and it flies on the same PID engine and state machine as
the built-ins. A profile the role table does not know flies as a hybrid
explorer — no other file to edit.

**Checked end to end**: a fresh clone, `npm install`, a new profile dropped in,
`npm run dev` — eleven drones take off and explore, the new profile spawns and
flies, two model providers answer in parallel and a battery rule still imposes
its landing.

---

## Plugging in an algorithm

```javascript
import { SwarmIntelligenceInterface } from './swarm-intelligence-interface.js';

export class MySwarmAlgorithm extends SwarmIntelligenceInterface {
    initialize(config) {
        // once at startup: { droneCount, arena, profiles }
    }

    computeInfluences(droneStates, dt) {
        // every frame
        // in:  Map<id, {position, velocity, target, ...}>
        // out: Map<id, {targetModifier, velocityBias, priorityOverride}>
        return new Map();
    }
}
```

**You observe and suggest, the PID controller decides.** Output is merged with
the flight command, not substituted for it — stability is not your problem.

---

## Bring your own model (neurosymbolic)

Anyone can put **their own drone** (a JSON profile, above) **with their own
model** into the swarm. The model can be anything that turns what a drone
knows into a proposed decision:

- a language model served by Ollama, llama.cpp, vLLM, LM Studio…
- a reinforcement-learning policy, an ONNX network, a planner behind an HTTP endpoint;
- a provider you write yourself in a few lines.

### The model proposes, the rules dispose

A model is never trusted with the aircraft. Every decision goes through a
symbolic rule layer — that is what *neurosymbolic* means here:

```
 Observation ──► rules.before ──► model.decide ──► rules.after ──► bounded influence
 (own view:           │ veto              (neural)         │ reject        on the waypoint
  neighbours,         ▼                                    ▼               — the PID flies
  battery, …)   imposed action                     reactive fallback
```

| Step | Who | What happens |
|---|---|---|
| `rules.before` | symbolic | critical situations (battery, separation…) **veto**: the action is imposed, the model is not even asked |
| `model.decide` | your model | returns `{ action, direction, confidence, reasoning }` — or `null` |
| `rules.after` | symbolic | action not allowed for this drone, unknown direction, low confidence → **rejected** |
| engine | physics | an accepted decision shifts the next waypoint by at most 8 m, clamped to the arena |

A model that times out, answers garbage or proposes something forbidden
changes nothing: the drone keeps flying on its reactive behaviour. **Swap the
model, keep the rules** — a better model never needs the safety behaviour to be
re-validated.

### In three steps

**1. Your drone** — `physics/profiles/my-drone.json` (see [Adding a drone](#adding-a-drone)).

**2. Your model** — copy the registry and give each profile id a provider:

```bash
cd DIAMANTS_FRONTEND/Mission_system/intelligence/model-providers
cp agent-models.example.json agent-models.json     # git-ignored
```

```json
{
  "X500": {
    "provider": "ollama",
    "model": "my-drone-model:latest",
    "allowedActions": ["EXPLORE", "AVOID", "HOVER", "RTL", "REPLAN", "COORDINATE"],
    "minConfidence": 0.6,
    "systemPrompt": "You are an X500 survey drone. Answer with one JSON object: {\"action\", \"direction\", \"confidence\", \"reasoning\"}"
  },
  "MY_DRONE": {
    "provider": "http-policy",
    "name": "ppo-explorer-v3",
    "url": "http://localhost:9000/decide",
    "allowedActions": ["EXPLORE", "AVOID", "HOVER", "RTL"],
    "timeoutMs": 2000
  }
}
```

Several profiles can run **different models at the same time** — that is the
point: heterogeneous agents, one contract. Profiles without an entry fly without
a model. `_settings.maxConcurrent` (default 2) caps requests in flight for the
whole swarm: a local server answers one at a time, so past the cap a drone skips
that thought instead of queueing into a timeout. Rule vetoes are never skipped.

> `agent-models.json` is read from the **dev server only** and never bundled
> into `npm run build` output. Keep API keys there or in your server's own
> config — never in a committed file.

**3. Run** — `npm run dev`. When `agent-models.json` exists, the controller
replaces the inert LLM shell by `neurosymbolic-bridge.js`; the console prints
`Own models attached: N drones`. They start enabled; the **LLM ON/OFF** button
of the *LLM Intelligence* panel pauses and resumes them. Every decision, veto
and rejection appears in its feed with the model name and latency.

Hot-swap from the browser console, rules untouched:

```javascript
const mgr = diamantsSystem.integratedController.droneIntelligenceManager;
mgr.setModelForType('X500', 'my-drone-model-v2:latest');
mgr.getStats();   // { requests, accepted, rejected, vetoes, failures, activeBrains }
```

### Writing a provider

```javascript
import { AgentModel } from '../agent-model-interface.js';
import { registerProvider } from './index.js';

class MyPolicy extends AgentModel {
    get name() { return `mine:${this.config.checkpoint}`; }
    async decide(observation, { allowedActions, signal }) {
        // observation = { agentId, agentType, phase, position{x,z,alt}, speed,
        //                 battery, neighbours[{id,dist,dir}], sharedFindings[], coverage }
        // pass `signal` to fetch(): on timeout the request is cancelled server-side
        return { action: 'EXPLORE', direction: 'NE', confidence: 0.8, reasoning: 'frontier NE' };
    }
}
registerProvider('my-policy', MyPolicy);   // then "provider": "my-policy" in agent-models.json
```

### Writing your rules

`model-providers/example-rules.js` is a deliberately small example (battery
veto, separation veto, allowed actions, confidence threshold). Extend
`RuleLayer` with your own and pass it to `NeuroSymbolicIntelligenceManager`.
The rules are the part to review and test — whatever model sits behind them.

### Making a better model

A general-purpose language model knows language, not your drone. Asked *"battery
21 %, home is north, what now?"* it may answer in prose, invent an action, or
ignore the battery. Two ways to do better, both compatible with the contract:

- **Prompt and rules first.** A tighter system prompt in your registry entry, and
  stricter rules in your `RuleLayer`, already remove most of the nonsense — and
  cost nothing to try.
- **Fine-tune a small model on your own examples.** Continuing the training of a
  small instruct model on *(situation → decision)* pairs turns the behaviour into
  a reflex. LoRA adapters make this cheap on a single consumer GPU
  ([Unsloth](https://github.com/unslothai/unsloth), Hugging Face PEFT), and
  `ollama create` serves the result under a name you put in the registry.

Whatever you train, **measure it before you swap it in**: keep situations the
model never saw, and count valid answers, allowed actions and unsafe proposals.
`NeuroSymbolicAgent.stats` gives you accepted / rejected / vetoed counts at
runtime, and the rules stay in the loop either way.

The teacher rules, the training set and the pipeline of the research build are
not published.

---

## Stack

Three.js 0.167, Vite 4.5, Vitest, ES modules, Node 20+.

---

## Videos

### Demonstration films (September 2026)

**▶ Online player: [https://lololem.github.io/diamants-collab/](https://lololem.github.io/diamants-collab/)**

[![Live demo — diamants-hypervision](https://img.shields.io/badge/Live%20demo-diamants--hypervision-4fd8ff?style=for-the-badge&logo=cloudflarepages&logoColor=white&labelColor=0e151d)](https://diamants-hypervision.pages.dev) [![Watch the demonstration films](https://img.shields.io/badge/Watch%20the%20films-online%20player-48e0a0?style=for-the-badge&logo=githubpages&logoColor=white&labelColor=0e151d)](https://lololem.github.io/diamants-collab/)

<sub>Try DIAMANTS directly in your browser with the live demo — note that it is not up to date either compared with the demonstration films.</sub>

> ⚠️ **These are demonstration videos.** They were recorded from the current internal development version of DIAMANTS. **The code published in this repository is not up to date** and does not include every capability shown here.

#### Film 1: Unmapped Sector Reconnaissance (22 min) — [▶ watch online](https://lololem.github.io/diamants-collab/#film1) · [download 1080p](https://github.com/lololem/diamants-collab/releases/download/demo-videos-2026-09/DIAMANTS-film1-unmapped-sector-reconnaissance.mp4)

[![Film 1: Unmapped Sector Reconnaissance — click to watch](https://lololem.github.io/diamants-collab/film1-play.jpg)](https://lololem.github.io/diamants-collab/#film1)

This scenario showcases a heterogeneous swarm operating within a fully decentralized architecture:

- **Heterogeneous Swarm & Division of Labor:** Micro-drones act as reactive scouts, while camera-equipped platforms use on-board AI to verify targets in a two-stage detection process.
- **Distributed Autonomy:** All decisions are made on-board without a central planner, utilizing collaborative coverage methods (stigmergy, consensus-based allocation, and area partitioning).
- **True Flight Physics & Neurosymbolic AI:** Action proposals from the AI are strictly vetted by a deterministic rule layer before execution. The simulation incorporates the true flight physics of the drones (mass, inertia, rotor thrusts), ensuring highly realistic and safe control.
- **Federated Learning & Operator Control:** Agents collaboratively learn while keeping their observation data private. Operators can issue natural-language commands that seamlessly override autonomous missions, visualized through a live command-chain display.

#### Film 2: Distributed Wildfire Response (17 min) — [▶ watch online](https://lololem.github.io/diamants-collab/#film2) · [download 1080p](https://github.com/lololem/diamants-collab/releases/download/demo-videos-2026-09/DIAMANTS-film2-distributed-wildfire-response.mp4)

[![Film 2: Distributed Wildfire Response — click to watch](https://lololem.github.io/diamants-collab/film2-play.jpg)](https://lololem.github.io/diamants-collab/#film2)

This scenario focuses on dynamic task allocation and emergent air-ground coordination during a rapidly evolving fire simulation:

- **Air-Ground Synergy:** Drones and autonomous ground vehicles collaborate to execute a complete detection-to-extinction chain with no human in the loop.
- **Consensus-Based Allocation:** Tasks are broken down via a Hierarchical Task Network (HTN). Vehicles allocate assignments among themselves through consensus, eliminating the need for a central dispatcher.
- **Resilience & AI Doctrine:** The swarm adapts live to partial information, agent loss, and multiple simultaneous events. The AI advises on strategy, while the vehicles manage their own navigation and physical suppression.
- **Live Adaptation:** Federated learning runs continuously during the response, while the operator retains the ability to dynamically issue high-level commands (e.g., zone search, patrol, return to base).

▶ **Watch both films in the browser: [lololem.github.io/diamants-collab](https://lololem.github.io/diamants-collab/)** — the full-quality files are also on the [release page](https://github.com/lololem/diamants-collab/releases/tag/demo-videos-2026-09).

### Earlier videos

[3D frontend](https://www.youtube.com/watch?v=fyEmYu4lbzo) ·
[Multi-agent systems](https://www.youtube.com/watch?v=1Av_o-9fzrE) ·
[Gradient navigation](https://www.youtube.com/watch?v=ElABxOde6ak) ·
[Swarm coordination](https://www.youtube.com/watch?v=L8V64LajM2w) ·
[Stigmergy](https://www.youtube.com/watch?v=SyqeRwcbDO4)

---

## Contributing

Drone profiles, models and model providers, rule layers, swarm algorithms,
rendering improvements, bug fixes, tests, docs.
Fork, branch, pull request — see [Contributing.md](Contributing.md).
Contributions are distributed under the project licence.

---

## Licence

**PolyForm Noncommercial 1.0.0** — [LICENSE](LICENSE).

Research, teaching, learning, non-profit: yes. Commercial product, paid service,
integration into an offering: no.

Bundled third-party components keep their own licence, listed at the end of the
LICENSE file.

---

[Open an issue](https://github.com/lololem/diamants-collab/issues)
