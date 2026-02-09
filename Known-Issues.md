# Known Issues

**Updated:** June 2025 (post V2 refactor)

---

## Resolved

### THREE.js Import Conflicts

**Was:** Multiple contradictory import strategies (CDN, ES6, dynamic, globals).
**Fix:** All files now use `import * as THREE from 'three'` via Vite ES6 module resolution. CDN script tags removed from `index.html`.
**Status:** Resolved.

### Physics System Duplication

**Was:** Three separate physics engine instances (physics-engine.js, cannon-physics.js, diamant-system.js).
**Fix:** `AutonomousFlightEngine` is the single source of truth for drone positions. PID controller, drone profiles loaded from JSON via `DronePhysicsRegistry`. Legacy physics files are inert (not imported).
**Status:** Resolved.

### WebSocket Port Confusion (9090 / 9001)

**Was:** Documentation and scripts referenced ports 9090 and 9001 for the WebSocket bridge.
**Fix:** All services consolidated on port 8765 (`DiamantsBridge`). Stale references cleaned from `stop.sh`, `status.sh`, `stop-all.sh`, test files, and README.
**Status:** Resolved.

---

## Open

### UI-Controller Bridge (missing global state)

**Severity:** Medium.
**Description:** `mission-interface.js` references `application.controller` and `window.missionController` which are not exposed. The mission UI buttons are non-functional.
**Impact:** UI mission controls do not trigger backend actions. Drone flight still works (engine runs autonomously).
**Workaround:** Drones fly via `AutonomousFlightEngine` — no mission interface needed for exploration demo.
**Next step:** Wire `MissionController` to a global `window.DIAMANTS` namespace, or replace with event-driven architecture via `CustomEvent`.

### CAS Fallback Not Formalized

**Severity:** Low.
**Description:** The three-tier fallback (CAS 1: backend ROS2, CAS 2: AutonomousFlightEngine, CAS 3: simple update) is documented in Architecture.md but not implemented as an explicit state machine in code. The fallback happens implicitly: if WebSocket is down, the engine takes over.
**Impact:** No functional impact — the implicit fallback works. But there is no explicit CAS state indicator in the UI.
**Next step:** Add a `cas_level` property to `main.js` and display it in the HUD.

### Hardcoded Constants in Flight Engine

**Severity:** Low.
**Description:** Several magic numbers in `autonomous-flight-engine.js`:
- Tree avoidance distance: 5m (line ~260)
- Drone-drone avoidance distance: 4m (line ~280)
- Waypoint timeout: 12s (line ~195)
- Base RPM for visual effect: 12000 (line ~310)
- Exploration spiral angle increment: 0.4-0.6 rad (line ~215)
**Impact:** These work for the current arena (30×30m, mixed fleet). Not configurable per-profile.
**Next step:** Move to drone profiles or a `flight-config.json`.

### Swarm Intelligence Slot (noop)

**Severity:** N/A (expected).
**Description:** `SwarmIntelligenceInterface` is wired into the engine with 4 hooks (computeNextWaypoint, modulateVelocity, updateEnvironment, tick). The default `NoopSwarmIntelligence` does nothing — drones explore with the built-in spiral/coverage algorithm.
**Impact:** None. The slot is ready for a concrete implementation (stigmergy, RL, etc.) to be plugged in.
**Next step:** Implement a stigmergy engine.

---

## Environment Notes

- **ROS2 not required for frontend-only demo.** The flight engine runs entirely client-side.
- **Gazebo is optional.** Only needed for physics-accurate simulation with ROS2 backend.
- **Node.js 20.20.0** via nvm. The `.nvmrc` file enforces this version.
