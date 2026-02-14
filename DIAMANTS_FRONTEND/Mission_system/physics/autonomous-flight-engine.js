/**
 * DIAMANTS - Autonomous Flight Engine (Frontend)
 * =================================================
 * ⚠️  v0-origin  (commit 47cec8ee — tag v0-origin)
 * Ce fichier est le CŒUR du système qui fait "scooter" les drones.
 * Toute modification DOIT être testée contre cette baseline.
 * Restaurer : git checkout v0-origin -- physics/autonomous-flight-engine.js
 * Archive  : /home/loic/v0-origin-backup/diamants-collab-v0-origin.tar.gz
 *
 * Fast, fluid, dynamic exploration with proper PID control.
 * Handles mixed drone types (Crazyflie + larger drones),
 * tree bounding-box collisions, and inter-drone avoidance.
 *
 * This is the SINGLE source of truth for frontend drone positions
 * when running in autonomous mode (no backend / backend too slow).
 */
import * as THREE from 'three';

// ─── Drone profiles — loaded from JSON via DronePhysicsRegistry ──────
// Profiles are defined in physics/profiles/*.json
// The registry normalises them to the exact same shape as the old
// hardcoded DRONE_PROFILES object → zero regression.
import { DRONE_PROFILES, DronePhysicsRegistry } from './drone-physics-registry.js';
import { NoopSwarmIntelligence } from '../intelligence/swarm-intelligence-interface.js';
export { DRONE_PROFILES };

// ─── Engine-level flight constants (externalized from code) ──────────
import FLIGHT_CONFIG from './flight-config.json' assert { type: 'json' };
const AVOID  = FLIGHT_CONFIG.avoidance;
const EXPLORE = FLIGHT_CONFIG.exploration;
const VISUAL  = FLIGHT_CONFIG.visual;

// ─── Simple PID ──────────────────────────────────────────────────────
class PID {
    constructor({ kp = 1, ki = 0, kd = 0 } = {}) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this._integral = 0;
        this._prevError = 0;
    }
    reset() { this._integral = 0; this._prevError = 0; }
    compute(error, dt) {
        this._integral += error * dt;
        this._integral = Math.max(-5, Math.min(5, this._integral)); // anti-windup
        const derivative = dt > 0 ? (error - this._prevError) / dt : 0;
        this._prevError = error;
        return this.kp * error + this.ki * this._integral + this.kd * derivative;
    }
}

// ─── Per-drone state ─────────────────────────────────────────────────
class DroneFlightState {
    constructor(id, profile, startPos) {
        this.id = id;
        this.profile = profile;

        // Kinematic state
        this.position = startPos.clone();
        this.velocity = new THREE.Vector3();
        this.heading = Math.random() * Math.PI * 2; // yaw

        // PIDs
        this.pidX = new PID(profile.pid.pos);
        this.pidZ = new PID(profile.pid.pos);
        this.pidY = new PID(profile.pid.alt);
        this.pidYaw = new PID(profile.pid.yaw);

        // Exploration state machine
        this.phase = 'IDLE'; // IDLE → TAKEOFF → EXPLORE → LAND → LANDED
        this.waypoint = null;
        this.waypointTimer = 0;
        this.explorationAngle = Math.random() * Math.PI * 2;
        this.lastWaypointTime = 0;
        this.waypointsVisited = 0;

        // Smoothing
        this.smoothVelocity = new THREE.Vector3();
    }
}

// ─── Main engine ────────────────────────────────────────────────────
export class AutonomousFlightEngine {
    constructor(options = {}) {
        this.drones = new Map();          // id → DroneFlightState
        this.treeBounds = [];             // [{center: Vec3, radius: number}]
        this.enabled = true;
        this.explorationBounds = options.explorationBounds || 50; // m from center
        this.gravity = -9.81;

        // Coverage tracking
        this.visitedCells = new Set();
        this.cellSize = EXPLORE.cellSize;

        // Swarm intelligence slot (pluggable from diamants-private)
        this.swarmIntelligence = new NoopSwarmIntelligence();

        // Doctrine system slot (waypoint patterns from DoctrineManager)
        this.doctrineManager = null;
        // Per-drone doctrine waypoint queues: droneId → [Vec3, ...]
        this._doctrineWaypoints = new Map();

        console.log('🚀 AutonomousFlightEngine initialised');
    }

    // ─── Registration ────────────────────────────────────────────────
    registerDrone(id, profileName, startPos) {
        const profile = DRONE_PROFILES[profileName] || DRONE_PROFILES.CRAZYFLIE;
        const state = new DroneFlightState(id, profile, startPos);
        this.drones.set(id, state);
        return state;
    }

    registerTree(center, radius) {
        this.treeBounds.push({ center: center.clone(), radius });
    }

    clearTrees() {
        this.treeBounds = [];
    }

    /**
     * Plug a SwarmIntelligenceInterface implementation.
     * The concrete engine (stigmergy, RL, etc.) lives in diamants-private.
     */
    setSwarmIntelligence(impl) {
        this.swarmIntelligence = impl || new NoopSwarmIntelligence();
        this.swarmIntelligence.initialize(this.drones, this.treeBounds);
        console.log(`🧠 Swarm intelligence loaded: ${impl?.constructor?.name || 'noop'}`);
    }

    /**
     * Command a drone to land
     */
    land(droneId) {
        const state = this.drones.get(droneId);
        if (state) {
            const prevPhase = state.phase;
            state.phase = 'LAND';
            state.waypoint = state.position.clone();
            state.waypoint.y = 0.15; // Platform surface (PLATFORM_HEIGHT)
            console.log(`[STATE-MACHINE] 🛬 ${droneId} ${prevPhase} → LAND`);
        }
    }

    /**
     * Command a drone to takeoff
     */
    takeoff(droneId, targetAltitude) {
        const state = this.drones.get(droneId);
        if (state) {
            if (targetAltitude !== undefined) {
                state.profile.cruiseAlt = targetAltitude;
            }
            state.phase = 'TAKEOFF';
            state.hoverPosition = null; // Will be set fresh when reaching HOVER
            console.log(`[STATE-MACHINE] 🛫 ${droneId} → TAKEOFF (alt: ${state.profile.cruiseAlt}m)`);
        }
    }

    /**
     * Transition all drones from HOVER to EXPLORE (called by Launch)
     */
    startExploration() {
        let count = 0;
        // Flush any cached doctrine waypoints so drones get fresh routes
        this._doctrineWaypoints.clear();
        for (const [id, state] of this.drones) {
            if (state.phase === 'HOVER' || state.phase === 'TAKEOFF') {
                state.phase = 'EXPLORE';
                state.waypointTimer = 0;
                state.lastWaypointTime = 0;
                state.waypoint = null; // Force new waypoint pick
                count++;
                console.log(`[STATE-MACHINE] 🚀 ${id} HOVER → EXPLORE`);
            }
        }
        return count;
    }

    // ─── Main update (call every frame) ──────────────────────────────
    update(dt) {
        if (!this.enabled) return;
        const clamped = Math.min(dt, 1 / 20); // cap at 50ms

        for (const [id, state] of this.drones) {
            this._updateDrone(state, clamped);
        }

        // Global swarm intelligence tick (pheromone evaporation, etc.)
        this.swarmIntelligence.tick(clamped);
    }

    // ─── Per-drone update ────────────────────────────────────────────
    _updateDrone(state, dt) {
        const prof = state.profile;

        // 1. State machine — pick target
        this._updateStateMachine(state, dt);

        if (!state.waypoint) return;

        // 2. PID → desired velocity
        const errX = state.waypoint.x - state.position.x;
        const errZ = state.waypoint.z - state.position.z;
        const errY = state.waypoint.y - state.position.y;

        let vxCmd = state.pidX.compute(errX, dt);
        let vzCmd = state.pidZ.compute(errZ, dt);
        let vyCmd = state.pidY.compute(errY, dt);

        // 3. Clamp speed
        const hSpeed = Math.sqrt(vxCmd * vxCmd + vzCmd * vzCmd);
        if (hSpeed > prof.maxSpeed) {
            const scale = prof.maxSpeed / hSpeed;
            vxCmd *= scale; vzCmd *= scale;
        }
        vyCmd = Math.max(-prof.maxClimb, Math.min(prof.maxClimb, vyCmd));

        // 4-5. Avoidance & swarm intelligence — ONLY during EXPLORE
        // During HOVER/TAKEOFF/LAND, drones must hold position without drift
        if (state.phase === 'EXPLORE') {
            // 4. Tree avoidance (repulsion force)
            const avoidance = this._computeTreeAvoidance(state);
            vxCmd += avoidance.x;
            vzCmd += avoidance.z;
            vyCmd += avoidance.y;

            // 5. Drone-drone avoidance
            const droneAvoid = this._computeDroneAvoidance(state);
            vxCmd += droneAvoid.x;
            vzCmd += droneAvoid.z;

            // 5b. Swarm intelligence velocity modulation (stigmergy, RL, etc.)
            const modulated = this.swarmIntelligence.modulateVelocity(
                state.id, { x: vxCmd, y: vyCmd, z: vzCmd }, state
            );
            vxCmd = modulated.x;
            vyCmd = modulated.y;
            vzCmd = modulated.z;

            // 5c. Boundary containment — soft repulsion near zone edges
            const boundary = this._computeBoundaryForce(state);
            vxCmd += boundary.x;
            vzCmd += boundary.z;
        }

        // 6. Smooth velocity (low-pass)
        const alpha = Math.min(1.0, prof.agility * 3.0 * dt);
        state.smoothVelocity.x += (vxCmd - state.smoothVelocity.x) * alpha;
        state.smoothVelocity.y += (vyCmd - state.smoothVelocity.y) * alpha;
        state.smoothVelocity.z += (vzCmd - state.smoothVelocity.z) * alpha;

        // 7. Integrate position
        state.position.x += state.smoothVelocity.x * dt;
        state.position.y += state.smoothVelocity.y * dt;
        state.position.z += state.smoothVelocity.z * dt;

        // 8. Ground clamp — platform surface is at y=0.15 (PLATFORM_HEIGHT)
        const PLATFORM_SURFACE = 0.15;
        if (state.position.y < PLATFORM_SURFACE) {
            state.position.y = PLATFORM_SURFACE;
            state.smoothVelocity.y = Math.max(0, state.smoothVelocity.y);
        }

        // 8b. Horizontal boundary hard clamp — keep drones inside zone
        const halfZone = this._getHalfZone();
        if (Math.abs(state.position.x) > halfZone) {
            state.position.x = Math.sign(state.position.x) * halfZone;
            state.smoothVelocity.x = 0;
        }
        if (Math.abs(state.position.z) > halfZone) {
            state.position.z = Math.sign(state.position.z) * halfZone;
            state.smoothVelocity.z = 0;
        }

        // 9. Update heading (face movement direction)
        if (hSpeed > 0.3) {
            const targetYaw = Math.atan2(state.smoothVelocity.x, state.smoothVelocity.z);
            let yawErr = targetYaw - state.heading;
            while (yawErr > Math.PI) yawErr -= 2 * Math.PI;
            while (yawErr < -Math.PI) yawErr += 2 * Math.PI;
            state.heading += state.pidYaw.compute(yawErr, dt);
        }

        // 10. Coverage tracking
        const cx = Math.floor(state.position.x / this.cellSize);
        const cz = Math.floor(state.position.z / this.cellSize);
        this.visitedCells.add(`${cx},${cz}`);

        // 11. Swarm intelligence environment update (deposit pheromones, etc.)
        this.swarmIntelligence.updateEnvironment(state.id, state.position, {
            speed: hSpeed,
            heading: state.heading,
            phase: state.phase,
            waypointsVisited: state.waypointsVisited,
        });

        // Store velocity for external use
        state.velocity.copy(state.smoothVelocity);
    }

    // ─── State machine ──────────────────────────────────────────────
    _updateStateMachine(state, dt) {
        const prof = state.profile;

        switch (state.phase) {
            case 'TAKEOFF': {
                // Climb to cruise altitude above current position
                state.waypoint = state.position.clone();
                state.waypoint.y = prof.cruiseAlt;
                if (state.position.y > prof.cruiseAlt * 0.85) {
                    state.phase = 'HOVER';
                    state.lastWaypointTime = 0;
                    // Save the hover hold position (fixed X/Z)
                    state.hoverPosition = state.position.clone();
                    state.hoverPosition.y = prof.cruiseAlt;
                    console.log(`[STATE-MACHINE] 🛫 ${state.id} TAKEOFF → HOVER (alt=${state.position.y.toFixed(2)}, holdAt x=${state.hoverPosition.x.toFixed(2)} z=${state.hoverPosition.z.toFixed(2)})`);
                }
                break;
            }
            case 'HOVER': {
                // Maintenir position FIXE — pas de dérive
                // Utiliser la position sauvegardée au moment de la transition
                if (!state.hoverPosition) {
                    state.hoverPosition = state.position.clone();
                    state.hoverPosition.y = prof.cruiseAlt;
                }
                if (!state.waypoint) state.waypoint = state.hoverPosition.clone();
                state.waypoint.x = state.hoverPosition.x;
                state.waypoint.z = state.hoverPosition.z;
                state.waypoint.y = prof.cruiseAlt;
                // Freiner toute vélocité horizontale
                state.smoothVelocity.x *= 0.90;
                state.smoothVelocity.z *= 0.90;
                break;
            }
            case 'EXPLORE': {
                state.waypointTimer += dt;
                const distToWP = state.waypoint ?
                    Math.sqrt(
                        (state.position.x - state.waypoint.x) ** 2 +
                        (state.position.z - state.waypoint.z) ** 2
                    ) : Infinity;

                // Pick a new waypoint when close or timeout
                if (!state.waypoint || distToWP < EXPLORE.waypointReachDist || state.waypointTimer > EXPLORE.waypointTimeout) {
                    // Ask swarm intelligence first (stigmergy, RL, etc.)
                    const neighbors = this._getNeighbors(state.id);
                    const suggested = this.swarmIntelligence.computeNextWaypoint(
                        state.id, state, neighbors
                    );
                    state.waypoint = suggested || this._pickExplorationWaypoint(state);
                    state.waypointTimer = 0;
                    state.waypointsVisited++;
                }
                break;
            }
            case 'LAND': {
                // Descend to platform surface (y=0.15 = PLATFORM_HEIGHT)
                state.waypoint = state.position.clone();
                state.waypoint.y = 0.15;
                if (state.position.y < 0.20) {
                    state.phase = 'LANDED';
                    state.velocity.set(0, 0, 0);
                    state.smoothVelocity.set(0, 0, 0);
                    console.log(`[STATE-MACHINE] 🛬 ${state.id} LAND → LANDED`);
                }
                break;
            }
            case 'LANDED': {
                // Stay on platform surface
                state.waypoint = state.position.clone();
                state.waypoint.y = 0.15;
                state.velocity.set(0, 0, 0);
                break;
            }
            case 'IDLE': {
                // Drone au sol, en attente de commande takeoff
                state.waypoint = null;
                state.velocity.set(0, 0, 0);
                state.smoothVelocity.set(0, 0, 0);
                break;
            }
        }
    }

    // ─── Doctrine integration ────────────────────────────────────────
    /**
     * Attach the DoctrineManager so the engine uses doctrine-based waypoints.
     */
    setDoctrineManager(dm) {
        this.doctrineManager = dm;
        console.log('[ENGINE] DoctrineManager connecté');

        // Listen for real-time doctrine/COA changes to flush cached waypoints
        if (dm && dm.addListener) {
            dm.addListener((event, data) => {
                if (event === 'doctrine' || event === 'coa') {
                    this.flushDoctrineWaypoints();
                }
            });
        }
    }

    /**
     * Flush all cached doctrine waypoints.
     * Called when doctrine or COA changes so drones immediately
     * pick up the new pattern on their next waypoint request.
     */
    flushDoctrineWaypoints() {
        this._doctrineWaypoints.clear();
        // Also force every EXPLORE drone to re-pick a waypoint NOW
        for (const [id, state] of this.drones) {
            if (state.phase === 'EXPLORE') {
                state.waypoint = null;
                state.waypointTimer = 999; // trigger immediate re-pick
            }
        }
        const coaName = this.doctrineManager?.currentCOA?.name || '?';
        console.log(`[ENGINE] Waypoints flushed → nouveau pattern: ${coaName}`);
    }

    // ─── Waypoint generation ─────────────────────────────────────────
    _pickExplorationWaypoint(state) {
        // ── 1. Try doctrine-based waypoints first ──
        if (this.doctrineManager && this.doctrineManager.missionState?.isActive) {
            const wp = this._pickDoctrineWaypoint(state);
            if (wp) return wp;
        }

        // ── 2. Fallback: coverage-optimised spiral ──
        return this._pickDefaultExplorationWaypoint(state);
    }

    /**
     * Pick next waypoint from doctrine waypoint queue.
     * Lazily generates a queue per drone, pops one waypoint at a time.
     */
    _pickDoctrineWaypoint(state) {
        let queue = this._doctrineWaypoints.get(state.id);

        // Generate a fresh queue when empty
        if (!queue || queue.length === 0) {
            try {
                const raw = this.doctrineManager.generateWaypoints(
                    state.id,
                    { x: state.position.x, y: state.position.y, z: state.position.z }
                );
                if (raw && raw.length > 0) {
                    queue = raw.map(p => new THREE.Vector3(p.x, p.y, p.z));
                    this._doctrineWaypoints.set(state.id, queue);
                    console.log(`[DOCTRINE] ${state.id}: ${queue.length} waypoints (${this.doctrineManager.currentCOA.name})`);
                }
            } catch (e) {
                console.warn('[DOCTRINE] waypoint generation error:', e);
            }
        }

        if (queue && queue.length > 0) {
            return queue.shift();
        }
        return null;
    }

    /**
     * Default coverage-optimised exploration (spiral + unvisited bias).
     * Used when no doctrine is active or as fallback.
     */
    _pickDefaultExplorationWaypoint(state) {
        const prof = state.profile;
        // Use doctrine zone bounds if available, else engine default
        let maxR = Math.min(prof.explorationRadius, this.explorationBounds);
        if (this.doctrineManager) {
            const z = this.doctrineManager.zoneParams;
            maxR = Math.min(maxR, Math.max(z.sizeX, z.sizeZ) / 2);
        }

        // Try multiple candidates, pick the one with least coverage
        let bestWP = null;
        let bestScore = -Infinity;

        for (let i = 0; i < EXPLORE.candidates; i++) {
            // Bias towards expanding outward in a spiral pattern
            state.explorationAngle += EXPLORE.spiralAngleMin + Math.random() * (EXPLORE.spiralAngleMax - EXPLORE.spiralAngleMin);
            const dist = EXPLORE.minWaypointDist + Math.random() * (maxR - EXPLORE.minWaypointDist);
            const angle = state.explorationAngle;

            const wx = Math.cos(angle) * dist;
            const wz = Math.sin(angle) * dist;

            // Vary altitude slightly for dynamic feel
            const wy = prof.cruiseAlt + (Math.random() - 0.5) * EXPLORE.altitudeVariation;

            // Score: prefer unvisited cells
            const cx = Math.floor(wx / this.cellSize);
            const cz = Math.floor(wz / this.cellSize);
            const cellKey = `${cx},${cz}`;
            const visited = this.visitedCells.has(cellKey) ? 0 : 10;

            // Prefer points away from current position for dynamism
            const dx = wx - state.position.x;
            const dz = wz - state.position.z;
            const moveDist = Math.sqrt(dx * dx + dz * dz);

            // Penalty for being too close to trees
            let treePenalty = 0;
            for (const tree of this.treeBounds) {
                const td = Math.sqrt((wx - tree.center.x) ** 2 + (wz - tree.center.z) ** 2);
                if (td < tree.radius + 3) treePenalty -= 5;
            }

            const score = visited + moveDist * 0.2 + treePenalty + Math.random() * 2;
            if (score > bestScore) {
                bestScore = score;
                bestWP = new THREE.Vector3(wx, Math.max(1.5, Math.min(prof.maxAlt, wy)), wz);
            }
        }

        return bestWP || new THREE.Vector3(
            (Math.random() - 0.5) * maxR,
            prof.cruiseAlt,
            (Math.random() - 0.5) * maxR
        );
    }

    // ─── Tree collision avoidance ────────────────────────────────────
    _computeTreeAvoidance(state) {
        const force = new THREE.Vector3();
        for (const tree of this.treeBounds) {
            const dx = state.position.x - tree.center.x;
            const dz = state.position.z - tree.center.z;
            const dist = Math.sqrt(dx * dx + dz * dz);
            const minDist = tree.radius + AVOID.treeDistance;

            if (dist < minDist && dist > 0.01) {
                const strength = AVOID.treeForceScale * (1 - dist / minDist);
                force.x += (dx / dist) * strength;
                force.z += (dz / dist) * strength;

                // If really close, push up too
                if (dist < tree.radius + 2) {
                    force.y += AVOID.treeVerticalPush;
                }
            }
        }
        return force;
    }

    // ─── Inter-drone collision avoidance ──────────────────────────────
    _computeDroneAvoidance(state) {
        const force = new THREE.Vector3();
        for (const [otherId, other] of this.drones) {
            if (otherId === state.id) continue;
            const dx = state.position.x - other.position.x;
            const dz = state.position.z - other.position.z;
            const dist = Math.sqrt(dx * dx + dz * dz);

            if (dist < AVOID.droneDistance && dist > 0.01) {
                const strength = AVOID.droneForceScale * (1 - dist / AVOID.droneDistance);
                force.x += (dx / dist) * strength;
                force.z += (dz / dist) * strength;
            }
        }
        return force;
    }

    // ─── Boundary containment force ─────────────────────────────────
    _getHalfZone() {
        if (this.doctrineManager) {
            const z = this.doctrineManager.zoneParams;
            return Math.max(z.sizeX, z.sizeZ) / 2;
        }
        return this.explorationBounds;
    }

    _computeBoundaryForce(state) {
        const force = new THREE.Vector3();
        const half = this._getHalfZone();
        const margin = 5; // start pushing back 5m before edge
        const strength = 8.0;

        // X boundaries
        if (state.position.x > half - margin) {
            const pen = (state.position.x - (half - margin)) / margin;
            force.x -= strength * Math.min(pen, 1);
        } else if (state.position.x < -(half - margin)) {
            const pen = (-(half - margin) - state.position.x) / margin;
            force.x += strength * Math.min(pen, 1);
        }

        // Z boundaries
        if (state.position.z > half - margin) {
            const pen = (state.position.z - (half - margin)) / margin;
            force.z -= strength * Math.min(pen, 1);
        } else if (state.position.z < -(half - margin)) {
            const pen = (-(half - margin) - state.position.z) / margin;
            force.z += strength * Math.min(pen, 1);
        }

        return force;
    }

    // ─── Apply state to Three.js drone ──────────────────────────────
    applyToDrone(drone, state) {
        if (!drone || !drone.mesh) return;

        // Position
        drone.position.copy(state.position);
        drone.mesh.position.copy(state.position);

        // Heading → rotation
        try {
            drone.mesh.rotation.y = state.heading;
            // Slight roll/pitch based on velocity for visual dynamism
            const rollAngle = Math.max(-VISUAL.maxRollPitch, Math.min(VISUAL.maxRollPitch, state.velocity.x * VISUAL.velocityTilt));
            const pitchAngle = Math.max(-VISUAL.maxRollPitch, Math.min(VISUAL.maxRollPitch, -state.velocity.z * VISUAL.velocityTilt));
            drone.mesh.rotation.z = rollAngle;
            drone.mesh.rotation.x = pitchAngle;
        } catch (_) { /* safe */ }

        // Mark as flying (only if engine phase is active)
        if (state.phase !== 'IDLE' && state.phase !== 'LANDED') {
            if (drone.state === 'IDLE') drone.state = 'FLYING';
        } else {
            drone.state = 'IDLE';
        }

        // Motor RPM: idle RPM when hovering, full RPM when exploring
        const speed = state.velocity.length();
        const isHovering = state.phase === 'HOVER';
        const baseRPM = isHovering
            ? VISUAL.baseRPM * 0.7
            : VISUAL.baseRPM + speed * VISUAL.rpmSpeedFactor;
        if (drone.motors) {
            for (let i = 0; i < 4; i++) {
                drone.motors[i].rpm = baseRPM + (Math.random() - 0.5) * VISUAL.rpmJitter;
                drone.motors[i].omega = (drone.motors[i].rpm / 60) * 2 * Math.PI;
            }
        }
    }

    // ─── Statistics ─────────────────────────────────────────────────
    getStats() {
        return {
            dronesActive: this.drones.size,
            treesRegistered: this.treeBounds.length,
            cellsVisited: this.visitedCells.size,
            coverageArea: this.visitedCells.size * this.cellSize * this.cellSize,
            swarmIntelligence: this.swarmIntelligence.getMetrics(),
            profiles: DronePhysicsRegistry.getInstance().listProfiles(),
        };
    }

    getDroneState(id) {
        return this.drones.get(id);
    }

    // ─── Helpers ────────────────────────────────────────────────────
    _getNeighbors(droneId) {
        const neighbors = [];
        for (const [id, state] of this.drones) {
            if (id !== droneId) neighbors.push(state);
        }
        return neighbors;
    }
}
