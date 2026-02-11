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
        this.phase = 'TAKEOFF'; // TAKEOFF → EXPLORE → RETURN
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

        // 6. Smooth velocity (low-pass)
        const alpha = Math.min(1.0, prof.agility * 3.0 * dt);
        state.smoothVelocity.x += (vxCmd - state.smoothVelocity.x) * alpha;
        state.smoothVelocity.y += (vyCmd - state.smoothVelocity.y) * alpha;
        state.smoothVelocity.z += (vzCmd - state.smoothVelocity.z) * alpha;

        // 7. Integrate position
        state.position.x += state.smoothVelocity.x * dt;
        state.position.y += state.smoothVelocity.y * dt;
        state.position.z += state.smoothVelocity.z * dt;

        // 8. Ground clamp
        if (state.position.y < 0.05) {
            state.position.y = 0.05;
            state.smoothVelocity.y = Math.max(0, state.smoothVelocity.y);
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
                    state.phase = 'EXPLORE';
                    state.lastWaypointTime = 0;
                    console.log(`🛫 ${state.id} takeoff complete → EXPLORE`);
                }
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
        }
    }

    // ─── Waypoint generation ─────────────────────────────────────────
    _pickExplorationWaypoint(state) {
        const prof = state.profile;
        const maxR = Math.min(prof.explorationRadius, this.explorationBounds);

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

        // Mark as flying
        if (drone.state === 'IDLE') drone.state = 'FLYING';

        // Motor RPM proportional to speed for visual effect
        const speed = state.velocity.length();
        const baseRPM = VISUAL.baseRPM + speed * VISUAL.rpmSpeedFactor;
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
