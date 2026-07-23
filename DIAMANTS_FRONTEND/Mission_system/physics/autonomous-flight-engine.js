/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Autonomous Flight Engine (Frontend)
 * =================================================
 * ⚠️  v0-origin  (commit 47cec8ee — tag v0-origin)
 * Ce fichier est le CŒUR du système qui fait "scooter" les drones.
 * Toute modification DOIT être testée contre cette baseline.
 * Restaurer : git checkout v0-origin -- physics/autonomous-flight-engine.js
 *
 * Fast, fluid, dynamic exploration with proper PID control.
 * Handles mixed drone types (Crazyflie + larger drones),
 * tree bounding-box collisions, and inter-drone avoidance.
 *
 * This is the SINGLE source of truth for frontend drone positions
 * when running in autonomous mode (no backend / backend too slow).
 */
import * as THREE from 'three';

// ─── Multi-Ranger Sensor pour détection d'obstacles ──────────────────
import { MultiRangerSensor, MultiRangerManager, MULTI_RANGER_CONFIG } from '../sensors/multi-ranger-sensor.js';

// ─── Drone profiles — loaded from JSON via DronePhysicsRegistry ──────
// Profiles are defined in physics/profiles/*.json
// The registry normalises them to the exact same shape as the old
// hardcoded DRONE_PROFILES object → zero regression.
import { DRONE_PROFILES, DronePhysicsRegistry } from './drone-physics-registry.js';
import { SwarmCommManager } from '../intelligence/swarm-comm-manager.js';
import { CommWaveRenderer } from '../visual/comm-wave-renderer.js';
import { NoopSwarmIntelligence } from '../intelligence/swarm-intelligence-interface.js';
import { X500FlightDynamics } from './x500-flight-dynamics.js';
export { DRONE_PROFILES };

// ─── Engine-level flight constants (externalized from code) ──────────
import flightConfigRaw from './flight-config.json?raw';
const FLIGHT_CONFIG = JSON.parse(flightConfigRaw);
const AVOID  = FLIGHT_CONFIG.avoidance;
const EXPLORE = FLIGHT_CONFIG.exploration;
const VISUAL  = FLIGHT_CONFIG.visual;

// ─── Simple PID with derivative filter ───────────────────────────────
class PID {
    constructor({ kp = 1, ki = 0, kd = 0 } = {}) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this._integral = 0;
        this._prevError = 0;
        this._prevDerivative = 0; // filtered derivative
    }
    reset() { this._integral = 0; this._prevError = 0; this._prevDerivative = 0; }
    compute(error, dt) {
        this._integral += error * dt;
        this._integral = Math.max(-5, Math.min(5, this._integral)); // anti-windup
        const rawDerivative = dt > 0 ? (error - this._prevError) / dt : 0;
        // Low-pass filter on derivative to prevent noise spikes
        const derivAlpha = 0.3;
        this._prevDerivative += (rawDerivative - this._prevDerivative) * derivAlpha;
        this._prevError = error;
        return this.kp * error + this.ki * this._integral + this.kd * this._prevDerivative;
    }
}

// ─── Per-drone state ─────────────────────────────────────────────────
class DroneFlightState {
    constructor(id, profile, startPos) {
        this.id = id;
        this.profile = profile;

        // Kinematic state
        this.position = startPos.clone();
        this.homePosition = startPos.clone(); // Original spawn position for return-to-base
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

        // ── Realistic flight dynamics for medium quads ──
        // X500 and S500 (cognitive/patrol agents) use physics-based dynamics
        // instead of the simple PID→velocity→Euler kinematic model.
        this.x500Dynamics = null;
        this.attitude = { roll: 0, pitch: 0 }; // Attitude angles for rendering
        if (profile.id === 'X500' || profile.model === 'x500' ||
            profile.id === 'S500' || profile.model === 's500') {
            this.x500Dynamics = new X500FlightDynamics(profile);
        }
        
        // Per-drone recent path memory — circular buffer of last N cell keys
        this._recentCells = [];       // last 200 visited cell keys
        this._recentCellsMax = 200;
        this._recentCellSet = new Set(); // fast lookup
        
        // Initial dispersion flag — first EXPLORE waypoint goes to territory center
        this._needsInitialDispersion = true;
        
        // Multi-Ranger sensor data (populated by _computeSensorAvoidance)
        this.sensorRanges = [3.5, 3.5, 3.5, 3.5, 3.5]; // [front, back, left, right, up]
        this.sensorAlerts = { warning: [], critical: [], obstacleVector: new THREE.Vector3() };
    }
}

// ─── Main engine ────────────────────────────────────────────────────
export class AutonomousFlightEngine {
    constructor(options = {}) {
        this.drones = new Map();          // id → DroneFlightState
        this.treeBounds = [];             // [{center: Vec3, radius: number}]
        this.enabled = true;
        this.explorationBounds = options.explorationBounds || 60; // m from center (match zoneSize/2)
        this.halfBounds = null; // {x, z} — asymmetric bounds, set from UI
        this.safetyDistanceOverride = null; // override AVOID.droneDistance/treeDistance from UI
        this.gravity = -9.81;

        // Coverage tracking
        this.visitedCells = new Set();
        this.cellSize = EXPLORE.cellSize;

        // ═══ ORGANIC MODE ═══
        // true  = v0-style fluid spiral exploration (biomimetic, fast, organic)
        // false = termite/swarm intelligence mode (sector sweep, pheromones)
        // Switch at runtime: engine.setOrganicMode(true/false)
        this.useOrganicMode = true;

        // GroundY cache — avoid recomputing trig 3× per drone per frame
        this._groundYCache = new Map(); // key "x|z" → height (cleared each frame)

        // ═══ SPATIAL HASH for O(N) drone avoidance ═══
        this._spatialHash = new Map();     // "gx,gz" → [droneState, ...]
        this._spatialCellSize = 12;        // X500 avoidDist ~10m, need larger cells
        this._spatialHashFrame = -1;       // last frame hash was rebuilt

        // ═══ DIAGNOSTIC ═══
        this._diagFrame = 0;
        this._diagInterval = 600; // log every N frames (~10s at 60fps — reduced to avoid saturation)
        this._diagEnabled = true;  // set false to silence

        // Swarm intelligence slot (moteur branché via stigmergy-loader)
        this.swarmIntelligence = new NoopSwarmIntelligence();

        // Doctrine system slot (waypoint patterns from DoctrineManager)
        this.doctrineManager = null;
        // Per-drone doctrine waypoint queues: droneId → [Vec3, ...]
        this._doctrineWaypoints = new Map();

        // NOUVEAU: Pathfinder Dijkstra pour navigation avec évitement d'obstacles
        this.pathfinder = null;
        this.voxelizer = null;
        // Per-drone intermediate path waypoints: droneId → [Vec3, ...]
        this._pathWaypoints = new Map();

        // NOUVEAU: Fonction de hauteur terrain pour collision réaliste
        this.getTerrainHeight = null; // (x, z) => y — injectable, mais fallback intégré ci-dessous
        this.minAltitudeAboveGround = 2.0; // Hauteur minimale au-dessus du sol (2m — herbe ~0.6m + marge visuelle)

        // ═══ INTER-DRONE COMMUNICATION ═══
        // Realistic radio comm: per-drone local knowledge, range-limited exchange,
        // cognitive→ant directives, visible on 3D labels.
        this.commManager = new SwarmCommManager();
        // Expose commManager globally so emergence formulas can access P2P info flow
        if (typeof window !== 'undefined') window._diamantsCommManager = this.commManager;

        // BOUNDING BOX SOL — Fallback intégré : reproduit la formule terrain provençal
        // Identique à terrain-environment.js getHeightAt()
        // Garantit un plancher même si getTerrainHeight n'est jamais injecté
        this._builtinTerrainHeight = (x, z) => {
            const distFromCenter = Math.sqrt(x * x + z * z);
            const hillFade = Math.max(0.0, Math.min(1.0, (distFromCenter - 12.0) / 3.0));
            const roughness = (Math.sin(x * 17.3) * Math.cos(z * 13.7) * 0.5 + 0.5) * 0.3 - 0.15;
            return hillFade * (
                Math.sin(x * 0.04) * Math.cos(z * 0.03) * 4.0 +
                Math.sin(x * 0.08) * Math.cos(z * 0.06) * 2.0 +
                Math.sin(x * 0.15) * Math.cos(z * 0.12) * 1.0 +
                roughness
            );
        };

        // NOUVEAU: Multi-Ranger Sensor Manager pour simulation capteurs ToF
        this.multiRangerManager = null; // Initialisé via setScene()
        this.scene = null; // THREE.Scene pour raycasting
        this.useSensorAvoidance = false; // DISABLED — 11 drones × 35 raycasts × 30Hz = 11K intersections/sec freezes GPU

        // ═══ AUTONOMY LEVEL (slider centralisé↔distribué) ═══
        // 0   = Centralisé: formation serrée, tous ensemble
        // 50  = Hybride: secteurs de groupe + liberté locale
        // 100 = Distribué: chaque drone est un agent autonome indépendant
        this.autonomyLevel = 100; // Default: fully distributed (original behavior)
        this._formationCenter = new THREE.Vector3(0, 0, 0); // Leader/centroid for formation
        this._formationDirty = true;

        // ═══ DRONE INTELLIGENCE (LLM/Agent brains) ═══
        // Pluggable: set via setIntelligenceManager()
        // When active, LLM enriches waypoint decisions during EXPLORE
        this.intelligenceManager = null;
        // Per-drone cached LLM results (async → stored here, consumed sync)
        this._llmResults = new Map(); // droneId → {reasoning, influence, safetyAction}

        // ═══ MISSION WAYPOINT PROVIDER (LLM mission system hook) ═══
        // When set, this function is called before autonomy-driven waypoint selection.
        // Signature: (droneId, state) => THREE.Vector3 | null
        // If it returns a Vector3, the engine uses it directly; if null, normal selection proceeds.
        this.missionWaypointProvider = null;

        console.log('🚀 AutonomousFlightEngine initialised');
    }

    // ═══════════════════════════════════════════════════════════════════
    //  FULL RESET — call on mission reset / re-init
    //  Cleans ALL state: coverage, per-drone memory, formation, caches
    // ═══════════════════════════════════════════════════════════════════
    resetAll() {
        // ── 1. Global coverage ──
        this.visitedCells.clear();

        // ── 2. Waypoint caches ──
        this._doctrineWaypoints.clear();
        this._pathWaypoints.clear();

        // ── 3. Formation state ──
        this._formationCenter.set(0, 0, 0);
        this._formationPhase = 0;
        this._formationAdvanceTimer = 0;
        this._formationDirty = true;

        // ── 4. Spatial hash ──
        this._spatialHash.clear();
        this._spatialHashFrame = -1;

        // ── 5. Per-drone state (deep reset) ──
        for (const [id, state] of this.drones) {
            state.phase = 'IDLE';
            state.velocity.set(0, 0, 0);
            state.smoothVelocity.set(0, 0, 0);
            state.waypoint = null;
            state.waypointTimer = 0;
            state.waypointsVisited = 0;
            state.lastWaypointTime = 0;
            state.hoverPosition = null;

            // Per-drone exploration memory
            state._recentCells = [];
            state._recentCellSet.clear();
            state._needsInitialDispersion = true;

            // Stall / oscillation counters
            state._stallTimer = 0;
            state._escaleCount = 0;
            state._oscillationCount = 0;
            state._oscillationTimer = 0;
            state._lastVelSign = null;
            state._wpInitialDist = null;

            // Territory (forces recomputation on next EXPLORE)
            state._territoryCenter = null;
            state._lastFleetSize = null;
            state._sectorAngle = null;
            state._sectorWidth = null;
            state._cruiseAlt = null;
            state._altLayer = null;

            // Takeoff ramp state — MUST clear to avoid stale S-curve on re-launch
            delete state._takeoffStartY;
            delete state._takeoffTimer;

            // Landing flags
            state._landInPlace = false;
            state._returningToBase = false;
            state._landTarget = null;
            state._landTimer = 0;

            // Visual heading — clear to prevent stale rotation after reset
            state.heading = 0;
            state._displayHeading = 0;

            // Smooth avoidance force accumulator
            if (state._smoothAvoid) state._smoothAvoid.set(0, 0, 0);

            // Reset ALL PIDs (kinematic + X500) — stale integral causes takeoff oscillation
            state.pidX.reset();
            state.pidY.reset();
            state.pidZ.reset();
            state.pidYaw.reset();
            if (state.x500Dynamics) state.x500Dynamics.resetPIDs();
        }

        // ── 6. Swarm intelligence ──
        if (typeof this.swarmIntelligence.reset === 'function') {
            this.swarmIntelligence.reset();
        }

        // ── 7. LLM results cache ──
        if (this._llmResults) this._llmResults.clear();

        // ── 8. Inter-drone communication ──
        if (this.commManager) this.commManager.reset();

        // ── 9. Comm wave renderer — hide all active beams/pulses ──
        if (this._commWaveRenderer) {
            this._commWaveRenderer.setEnabled(false);
            // Re-enable so new waves can spawn on next mission
            this._commWaveRenderer.setEnabled(true);
        }

        console.log(`[ENGINE] 🔄 resetAll() — ${this.drones.size} drones, visitedCells cleared, formation reset`);
    }

    // ─── DISTRIBUTED KNOWLEDGE: per-drone local cells ────────────────
    // Instead of the omniscient global `visitedCells`, each drone uses
    // only the cells IT has explored + cells received via P2P MAP_SYNC.
    // This makes communication MATTER: drones with no comm overlap → re-explore.
    // Falls back to global visitedCells if commManager is unavailable.
    _getDroneLocalCells(droneId) {
        if (this.commManager) {
            return this.commManager.getLocalCellsSet(droneId);
        }
        return this.visitedCells; // fallback: legacy omniscient mode
    }

    // ─── DISTRIBUTED NEIGHBORS: comm-range-limited peer set ──────────
    // Returns IDs of drones within radio range of the given drone.
    // Used to restrict flocking/alignment to peers this drone can "sense"
    // via radio, instead of the omniscient global spatial hash.
    _getCommNeighborIds(droneId) {
        if (this.commManager) {
            return this.commManager.getCommNeighbors(droneId);
        }
        // fallback: all drone IDs (legacy omniscient)
        const all = new Set();
        for (const id of this.drones.keys()) {
            if (id !== droneId) all.add(id);
        }
        return all;
    }

    /**
     * Hauteur du sol à (x, z) — utilise l'injection si dispo, sinon le builtin.
     * TOUJOURS fiable, même si setTerrainHeightFunction() n'a pas été appelé.
     * @param {number} x
     * @param {number} z
     * @returns {number} Hauteur Y du terrain
     */
    _groundY(x, z) {
        // Cache to avoid recomputing expensive trig multiple times per drone per frame
        const key = `${(x * 10) | 0}|${(z * 10) | 0}`; // ~0.1m precision
        const cached = this._groundYCache.get(key);
        if (cached !== undefined) return cached;

        const injected = this.getTerrainHeight ? this.getTerrainHeight(x, z) : -Infinity;
        const builtin = this._builtinTerrainHeight(x, z);
        // Retourne le MAX des deux — garantit que le plancher est toujours respecté
        const result = Math.max(injected, builtin);
        this._groundYCache.set(key, result);
        return result;
    }
    // ─── Registration ────────────────────────────────────────────────
    registerDrone(id, profileName, startPos) {
        const profile = DRONE_PROFILES[profileName] || DRONE_PROFILES.CRAZYFLIE;
        const state = new DroneFlightState(id, profile, startPos);

        // ═══ SECTOR HEADING — deferred assignment ═══
        // Heading is assigned LATER by _assignSectorHeadings() when all
        // drones are registered. During registerDrone, fleetSize is not
        // yet final → computing sectors here produces a -Z bias because
        // the denominator grows with each registration.
        // Store the drone index for later use.
        const match = String(id).match(/(\d+)$/);
        state._droneIdx = match ? parseInt(match[1], 10) - 1 : this.drones.size;

        this.drones.set(id, state);

        // Re-assign sector headings for ALL drones with correct fleet size
        this._assignSectorHeadings();

        return state;
    }

    /**
     * Assign heading + explorationAngle for every drone using the FINAL fleet size.
     * Called after each registerDrone so headings are always evenly distributed.
     * Convention: heading uses atan2(x,z) → 0 = +Z, π/2 = +X (matches yaw computation).
     */
    _assignSectorHeadings() {
        const N = this.drones.size;
        if (N === 0) return;
        for (const [, state] of this.drones) {
            const idx = state._droneIdx ?? 0;
            const sectorAngle = (idx / N) * Math.PI * 2;
            // Convention: heading = atan2(x, z) throughout the engine
            // So heading=0 → +Z, heading=π/2 → +X
            state.heading = sectorAngle;
            state.explorationAngle = sectorAngle;
            // Reset drift to match new heading
            state._driftAngle = sectorAngle;
        }
    }

    registerTree(center, radius, height = 12) {
        this.treeBounds.push({ center: center.clone(), radius, height });
        // Mark that swarm intelligence needs obstacle re-init (deferred to avoid per-tree overhead)
        this._treeBoundsDirty = true;
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
        // Forward terrain height function if already set
        if (this.getTerrainHeight && typeof this.swarmIntelligence.setTerrainHeightFunction === 'function') {
            this.swarmIntelligence.setTerrainHeightFunction(this.getTerrainHeight);
        }
        // Sync organic mode to agents
        if (this.useOrganicMode && typeof impl?.setOrganicMode === 'function') {
            impl.setOrganicMode(true);
        }

        // ── Apply initial doctrine/COA lightweightMode ──
        // At startup, lightweightMode defaults to true in constructors, but if the
        // current doctrine uses pheromones (e.g. EXPLORATION), we must activate them now.
        if (this.doctrineManager) {
            const needsPheromones = this.doctrineManager.currentDoctrine?.params?.usePheromones
                                 || this.doctrineManager.currentCOA?.params?.usePheromones
                                 || this.doctrineManager.currentCOA?.id === 'stigmergy';
            if (needsPheromones && typeof this.swarmIntelligence.setLightweightMode === 'function') {
                this.swarmIntelligence.setLightweightMode(false);
                console.log('🧪 Stigmergy pheromones activated (initial doctrine needs them)');
            }
        }

        console.log(`🧠 Swarm intelligence loaded: ${impl?.constructor?.name || 'noop'}`);
    }

    /**
     * Configurer la scène Three.js pour le raycasting des capteurs Multi-Ranger
     * @param {THREE.Scene} scene - La scène Three.js principale
     */
    setScene(scene) {
        this.scene = scene;
        if (scene) {
            this.multiRangerManager = new MultiRangerManager(scene);
            console.log('📡 Multi-Ranger sensor system initialized');

            // ═══ 3D Communication Wave Visualiser ═══
            // Animated beams + pulses travel between drones during MAP_SYNC / DIRECTIVE events
            this._commWaveRenderer = new CommWaveRenderer(scene);
            // Default: enabled (UI can toggle via window.DIAMANTS_SHOW_COMM_WAVES)
            if (typeof window !== 'undefined' && window.DIAMANTS_SHOW_COMM_WAVES === undefined) {
                window.DIAMANTS_SHOW_COMM_WAVES = true;
            }
        }
    }

    /**
     * Activer/désactiver l'évitement basé sur les capteurs Multi-Ranger
     * @param {boolean} enabled
     */
    setSensorAvoidance(enabled) {
        this.useSensorAvoidance = enabled;
        console.log(`📡 Sensor-based avoidance: ${enabled ? 'ENABLED' : 'DISABLED'}`);
    }

    /**
     * Activer le mode debug pour visualiser les rayons des capteurs
     * @param {boolean} enabled
     */
    setSensorDebugMode(enabled) {
        if (this.multiRangerManager) {
            this.multiRangerManager.setDebugMode(enabled);
            console.log(`📡 Sensor debug visualization: ${enabled ? 'ON' : 'OFF'}`);
        }
    }

    /**
     * Obtenir les lectures des capteurs pour un drone
     * @param {string} droneId
     * @returns {Object|null} Résultat du dernier scan
     */
    getSensorReadings(droneId) {
        if (!this.multiRangerManager) return null;
        const sensor = this.multiRangerManager.getSensor(droneId);
        return sensor ? sensor.getLastResult() : null;
    }

    /**
     * Command a drone to land
     */
    land(droneId) {
        const state = this.drones.get(droneId);
        if (state) {
            const prevPhase = state.phase;
            state.phase = 'LAND';
            state._landInPlace = true;
            state._landingSlot = null;
            // Snapshot current position as FIXED landing target
            state._landTarget = state.position.clone();
            state._landTimer = 0;
            // Reset PIDs to clear integral buildup from EXPLORE that opposes descent
            if (state.pidX) { state.pidX.reset(); state.pidY.reset(); state.pidZ.reset(); }
            if (state.x500Dynamics) state.x500Dynamics.resetPIDs();
            // Clear residual avoidance forces
            if (state._smoothAvoidance) state._smoothAvoidance.set(0, 0, 0);
            if (state._smoothAvoid) state._smoothAvoid.set(0, 0, 0);
            if (state._smoothFlock) { state._smoothFlock.x = 0; state._smoothFlock.z = 0; }
            // Type-aware landing target
            const terrY = this._groundY(state.position.x, state.position.z);
            const ground = Math.max(0.15, terrY);
            const restY = state.x500Dynamics
                ? ground + (state.profile.scale || 10) * 0.22  // match spawn height — gear on ground
                : ground + 0.05;
            // Store start altitude + target for S-curve descent trajectory
            state._landStartY = state.position.y;
            state._landRestY = restY;
            state.waypoint = state.position.clone();
            state.waypoint.y = state.position.y; // start at current altitude, will ramp down
            console.log(`[STATE-MACHINE] 🛬 ${droneId} ${prevPhase} → LAND in-place (from=${state._landStartY.toFixed(2)} target=${restY.toFixed(2)})`)
        }
    }

    /**
     * Command a drone to return to home position and then land
     */
    returnToHome(droneId) {
        const state = this.drones.get(droneId);
        if (state && state.phase !== 'IDLE' && state.phase !== 'LANDED') {
            state._returningToBase = true;
            state._landInPlace = false;
            if (state.phase !== 'EXPLORE') {
                state.phase = 'EXPLORE';
            }
            const home = state.homePosition;
            state.waypoint = new THREE.Vector3(home.x, state.position.y, home.z);
            console.log(`[STATE-MACHINE] 🏠 ${droneId} → RTH (home=${home.x.toFixed(1)},${home.z.toFixed(1)})`);
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
            // Clear stale S-curve state for a clean takeoff ramp
            delete state._takeoffStartY;
            delete state._takeoffTimer;
            // Pre-assign altitude layer so TAKEOFF knows correct target
            this._preAssignAltitudeLayer(state);
            // Reset X500/S500 PIDs to prevent stale integral buildup from previous phase
            if (state.x500Dynamics) state.x500Dynamics.resetPIDs();
            const effectiveAlt = state._cruiseAlt || state.profile.cruiseAlt;
            console.log(`[STATE-MACHINE] 🛫 ${droneId} → TAKEOFF (layerAlt: ${effectiveAlt.toFixed(1)}m, profileAlt: ${state.profile.cruiseAlt}m)`);
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
                // Reset X500/S500 PIDs — stale hover integrals cause initial lurching
                if (state.x500Dynamics) state.x500Dynamics.resetPIDs();
                count++;
                console.log(`[STATE-MACHINE] 🚀 ${id} HOVER → EXPLORE`);
            }
        }
        return count;
    }

    // ─── Autonomy level control ──────────────────────────────────────
    /**
     * Set autonomy level (0-100).
     * 0   = Centralisé: tight formation, drones follow leader
     * 25  = Guidé: formation with local obstacle avoidance
     * 50  = Hybride: sub-group sectors + individual exploration
     * 75  = Semi-autonome: individual territories, stigmergy
     * 100 = Distribué: fully independent agents
     */
    setAutonomyLevel(level) {
        const old = this.autonomyLevel;
        this.autonomyLevel = Math.max(0, Math.min(100, level));
        if (old !== this.autonomyLevel) {
            this._formationDirty = true;

            // If entering formation mode (< 25) from a higher level,
            // reset formation state so drones start from center, not from
            // a stale sweepAngle left over from a previous formation session.
            if (this.autonomyLevel < 25 && old >= 25) {
                this._formationCenter.set(0, 0, 0);
                this._formationPhase = 0;
                this._formationAdvanceTimer = 0;
            }

            // Force all EXPLORE drones to re-pick waypoints with new autonomy
            for (const [id, state] of this.drones) {
                if (state.phase === 'EXPLORE') {
                    state.waypoint = null;
                    state.waypointTimer = 999;
                }
            }
            console.log(`[ENGINE] 🎛️ Autonomy: ${old}% → ${this.autonomyLevel}%`);
        }
    }

    /**
     * Generate a formation waypoint for centralized mode.
     * Drones form a circle around the formation center and move as a unit.
     */
    _pickFormationWaypoint(state) {
        const fleetSize = this.drones.size || 1;
        const match = String(state.id).match(/(\d+)/);
        const idx = match ? parseInt(match[1], 10) - 1 : 0;

        // Formation radius grows with fleet size (min 2m, max 8m)
        const formRadius = Math.min(8, 2 + fleetSize * 0.5);
        const angle = (idx / fleetSize) * Math.PI * 2;

        // Slowly advance the formation center (coordinated sweep)
        if (!this._formationPhase) this._formationPhase = 0;
        if (!this._formationAdvanceTimer) this._formationAdvanceTimer = 0;
        this._formationAdvanceTimer++;

        // Every ~180 frames (~3s at 60fps), advance the formation
        if (this._formationAdvanceTimer > 180) {
            this._formationAdvanceTimer = 0;
            this._formationPhase++;
            const halfZone = this._getHalfZone();
            const sweepR = halfZone * 0.6;
            const sweepAngle = this._formationPhase * 0.3; // slow sweep
            this._formationCenter.x = Math.cos(sweepAngle) * sweepR * 0.5;
            this._formationCenter.z = Math.sin(sweepAngle) * sweepR * 0.5;
        }

        const cruiseAlt = state._cruiseAlt || state.profile.cruiseAlt;
        const fx = this._formationCenter.x + Math.cos(angle) * formRadius;
        const fz = this._formationCenter.z + Math.sin(angle) * formRadius;
        const terrY = this._groundY(fx, fz);

        return new THREE.Vector3(fx, terrY + cruiseAlt, fz);
    }

    /**
     * Generate a sub-group waypoint for hybrid mode (autonomy 25-75%).
     * Drones split into 2-4 sub-groups, each exploring a sector together.
     * COA bias is applied: multiple candidates are scored and the best is picked.
     */
    _pickHybridWaypoint(state) {
        const fleetSize = this.drones.size || 1;
        const match = String(state.id).match(/(\d+)/);
        const idx = match ? parseInt(match[1], 10) - 1 : 0;
        const halfZone = this._getHalfZone();
        const autonomy = this.autonomyLevel;

        // Number of sub-groups: 2 at 25%, 4 at 75%
        const numGroups = Math.max(2, Math.min(fleetSize, Math.round(2 + (autonomy - 25) / 25 * 2)));
        const groupIdx = idx % numGroups;
        const groupAngle = (groupIdx / numGroups) * Math.PI * 2 + (groupIdx * 0.4);

        // Group center
        const groupRadius = halfZone * (0.3 + autonomy / 200); // 0.43 at 25%, 0.68 at 75%
        const gcx = Math.cos(groupAngle) * groupRadius;
        const gcz = Math.sin(groupAngle) * groupRadius;

        // Scatter within group — more scatter at higher autonomy
        const scatter = 3 + (autonomy / 100) * 10; // 5.5m at 25%, 13m at 100%

        // Generate multiple candidates and pick probabilistically (ACO roulette-wheel)
        // Instead of always picking the best, scores are converted to probabilities
        // via softmax — this maintains exploration diversity (as in ant colony optimization)
        const numCandidates = 8;
        const candidateList = [];
        const cruiseAlt = state._cruiseAlt || state.profile.cruiseAlt;
        const cs = this.cellSize || 3;
        // ═══ DISTRIBUTED: per-drone local knowledge (not global omniscient) ═══
        const localKnowledge = this._getDroneLocalCells(state.id);

        for (let c = 0; c < numCandidates; c++) {
            const subAngle = Math.random() * Math.PI * 2;
            const subR = Math.random() * scatter;
            const wx = gcx + Math.cos(subAngle) * subR;
            const wz = gcz + Math.sin(subAngle) * subR;
            const cx = Math.max(-halfZone, Math.min(halfZone, wx));
            const cz = Math.max(-halfZone, Math.min(halfZone, wz));

            // Score: COA bias + visited penalty (per-drone knowledge)
            let score = this._computeCOABias(cx, cz, state);
            const gx = Math.round(cx / cs);
            const gz = Math.round(cz / cs);
            if (localKnowledge.has(`${gx},${gz}`)) score -= 15;

            candidateList.push({ x: cx, z: cz, score });
        }

        // ACO-style probabilistic selection (softmax roulette)
        const selected = this._selectProbabilistic(candidateList);
        if (selected) {
            const terrY = this._groundY(selected.x, selected.z);
            return new THREE.Vector3(selected.x, terrY + cruiseAlt, selected.z);
        }
        // Fallback: group center
        const terrY = this._groundY(gcx, gcz);
        return new THREE.Vector3(gcx, terrY + cruiseAlt, gcz);
    }

    // ─── Runtime mode switch ─────────────────────────────────────────
    setOrganicMode(enabled) {
        this.useOrganicMode = enabled;
        // Propagate lightweight mode to swarm intelligence
        // UNLESS doctrine requires pheromones (stigmergy COA or exploration doctrine)
        const needsPheromones = this.doctrineManager?.currentDoctrine?.params?.usePheromones
                             || this.doctrineManager?.currentCOA?.params?.usePheromones
                             || this.doctrineManager?.currentCOA?.id === 'stigmergy';
        if (this.swarmIntelligence && typeof this.swarmIntelligence.setLightweightMode === 'function') {
            this.swarmIntelligence.setLightweightMode(needsPheromones ? false : enabled);
        }
        // Tell agents to skip journal updates in organic mode (engine.visitedCells is source of truth)
        if (this.swarmIntelligence && typeof this.swarmIntelligence.setOrganicMode === 'function') {
            this.swarmIntelligence.setOrganicMode(enabled);
        }
        console.log(`[ENGINE] 🔄 Mode → ${enabled ? 'ORGANIC' : 'SWARM INTELLIGENCE'} | pheromones: ${needsPheromones ? 'ON' : 'lightweight'}`);
    }

    // ─── Main update (call every frame) ──────────────────────────────
    update(dt) {
        if (!this.enabled) return;
        const clamped = Math.min(dt, 0.15); // cap at 150ms — matches 10Hz physics rate

        // Track simulation wall-clock time
        this._simTime = (this._simTime || 0) + clamped;

        // Clear groundY cache once per frame (not per drone)
        this._groundYCache.clear();

        // Flush treeBounds to swarm intelligence if new trees were registered
        if (this._treeBoundsDirty && this.treeBounds.length > 0) {
            this._treeBoundsDirty = false;
            if (typeof this.swarmIntelligence.lateInitObstacles === 'function') {
                this.swarmIntelligence.lateInitObstacles(this.treeBounds);
            }
        }

        for (const [id, state] of this.drones) {
            this._updateDrone(state, clamped);
        }

        // ═══ INTER-DRONE COMMUNICATION ═══
        // Process range-limited peer-to-peer exchanges AFTER coverage tracking.
        // Throttled to 2 Hz — comm is expensive (O(n²) pairs + Set iterations).
        if (this.commManager) {
            if (!this.commManager._engine) this.commManager.attach(this);
            this._commAccum = (this._commAccum || 0) + clamped;
            if (this._commAccum >= 0.5) {
                this.commManager.update(this._commAccum);
                this._commAccum = 0;
            }

            // ═══ 3D Communication Wave Rendering ═══
            // Drain pending wave events and update active wave animations
            if (this._commWaveRenderer) {
                this._commWaveRenderer.update(
                    clamped,
                    this.commManager.getPendingWaves(),
                    this.drones
                );
            }
        }

        // Global swarm intelligence tick (pheromone evaporation, etc.)
        // Throttled: evaporate/diffuse 30-40K cells is expensive — run every 5th call
        this._stigTickCount = (this._stigTickCount || 0) + 1;
        if (this._stigTickCount >= 5) {
            this.swarmIntelligence.tick(clamped * 5);
            this._stigTickCount = 0;
        }

        // ─── FLEET POSITION LOG (every 5s = ~300 frames at 60fps) ───
        if (this._diagEnabled && this._diagFrame % 1800 === 0 && this.drones.size > 0) {
            const lines = [];
            for (const [id, st] of this.drones) {
                const tc = st._territoryCenter;
                const tcStr = tc ? `ter=(${tc.x.toFixed(0)},${tc.z.toFixed(0)})` : 'ter=?';
                const altStr = st._cruiseAlt ? `alt=${st._cruiseAlt.toFixed(1)}m` : '';
                const layerStr = st._altLayer !== undefined ? ['LOW','MID','HIGH'][st._altLayer] : '';
                lines.push(
                    `  ${id}: pos=(${st.position.x.toFixed(1)},${st.position.y.toFixed(1)},${st.position.z.toFixed(1)}) ` +
                    `${tcStr} ${altStr} ${layerStr} phase=${st.phase}`
                );
            }
            console.log(`[FLEET] ${this.drones.size} drones:\n${lines.join('\n')}`);
        }

        this._diagFrame++;
    }

    // ─── Per-drone update ────────────────────────────────────────────
    _updateDrone(state, dt) {
        const prof = state.profile;

        // ── Scenario: drone disabled (e.g. leader loss test) ──
        if (state._scenarioDisabled) {
            // Drone hovers in place — zero velocity, keep current position
            state.velocity.x = 0; state.velocity.y = 0; state.velocity.z = 0;
            state._lastDecision = '💀 HORS SERVICE (scénario)';
            return;
        }

        // ─── Per-drone exploration noise (individual behavioral variation) ──
        // Ornstein-Uhlenbeck–style drift: slow random walk on preferred angle.
        // This is the stochastic element essential for self-organization —
        // without individual noise, alignment mechanisms collapse into
        // unanimous consensus (Vicsek et al. 1995).
        // NOT centralized control: each drone's noise is independent.
        if (state._driftAngle == null) state._driftAngle = state.heading || 0;
        state._driftAngle += (Math.random() - 0.5) * 0.5 * dt; // ~0.5 rad/s noise rate

        // 1. State machine — pick target
        this._updateStateMachine(state, dt);

        if (!state.waypoint) return;

        // 1b. TERRAIN CLAMP — ensure waypoint is ALWAYS above terrain + safety margin
        //     This MUST happen BEFORE PID so the PID never targets underground
        //     SKIP for LAND/LANDED — those phases intentionally target surface-level waypoints
        if (state.waypoint && state.phase !== 'LAND' && state.phase !== 'LANDED') {
            const wpTerrainY = this._groundY(state.waypoint.x, state.waypoint.z);
            const wpSafeY = wpTerrainY + this.minAltitudeAboveGround;
            if (state.waypoint.y < wpSafeY) {
                state.waypoint.y = wpSafeY;
            }
        }

        // ═══════════════════════════════════════════════════════════
        // X500 REALISTIC FLIGHT DYNAMICS PATH
        // Force-based physics with cascaded PID (position→velocity→thrust)
        // Only used for X500 cognitive drones; Crazyflie uses kinematic model below.
        // ═══════════════════════════════════════════════════════════
        if (state.x500Dynamics) {
            // LANDED — drone is resting on ground, no physics needed
            if (state.phase === 'LANDED') {
                state.attitude = { roll: 0, pitch: 0 };
                state.velocity.set(0, 0, 0);
            } else {
            // ═══ FULL PHYSICS for ALL active phases (TAKEOFF, HOVER, EXPLORE, LAND) ═══
            // During LAND: avoidance reduced to 25% — enough to prevent collisions,
            // weak enough for PID to drive toward landing target
            if (!state._smoothAvoidance) state._smoothAvoidance = new THREE.Vector3();
            {
                const avoidanceForce = new THREE.Vector3();
                // Drone-drone avoidance
                const droneAvoid = this._computeDroneAvoidance(state);
                avoidanceForce.add(droneAvoid);
                // Tree + boundary: disabled during TAKEOFF
                if (state.phase !== 'TAKEOFF') {
                    const treeAvoid = this._computeTreeAvoidance(state);
                    avoidanceForce.add(treeAvoid);
                    const boundaryForce = this._computeBoundaryForce(state);
                    avoidanceForce.x += boundaryForce.x;
                    avoidanceForce.z += boundaryForce.z;
                }
                // Reduce avoidance during LAND — more so near ground
                // At cruise: 25%, linearly → 0% at ground rest height
                let avoidScale = 1.0;
                if (state.phase === 'LAND') {
                    const altAboveGround = state.position.y - this._groundY(state.position.x, state.position.z);
                    avoidScale = Math.min(0.25, Math.max(0, altAboveGround / 20));
                }
                avoidanceForce.multiplyScalar(avoidScale);
                // Smooth the avoidance force
                const avoidLerp = 0.40;
                state._smoothAvoidance.x += (avoidanceForce.x - state._smoothAvoidance.x) * avoidLerp;
                state._smoothAvoidance.y += (avoidanceForce.y - state._smoothAvoidance.y) * avoidLerp;
                state._smoothAvoidance.z += (avoidanceForce.z - state._smoothAvoidance.z) * avoidLerp;
            }

            // Full physics step — realistic cascaded PID for ALL phases including LAND
            // During LAND: pass minAlt=0 so PID can descend to landing height
            // (gearOffset in step() still prevents going underground)
            const landingMinAlt = state.phase === 'LAND' ? 0 : this.minAltitudeAboveGround;
            const attitude = state.x500Dynamics.step(
                state, dt, state._smoothAvoidance,
                (x, z) => this._groundY(x, z),
                landingMinAlt
            );

            // Store attitude for visual rendering
            state.attitude = attitude;

            // LAND phase: attitude leveling only — NO velocity mods after X500 step()
            // The X500 PID already integrates position with its velocity. Post-step velocity
            // changes create phase mismatches (position moved at V, velocity changed to V')
            // causing the PID to generate oscillating corrections → stuttering.
            if (state.phase === 'LAND') {
                const levelDecay = Math.exp(-8.0 * dt);
                state.attitude.roll  *= levelDecay;
                state.attitude.pitch *= levelDecay;
            }

            // Collision enforcement — skip during LAND when near ground
            // At low altitude, collision pushes create position snaps that fight the PID.
            // Avoidance forces (computed before PID step) are sufficient for separation.
            if (state.phase !== 'LANDED') {
                const landNearGround = state.phase === 'LAND' &&
                    (state.position.y - this._groundY(state.position.x, state.position.z)) < 5;
                if (!landNearGround) {
                    this._enforceTreeCollision(state, dt);
                    this._enforceDroneCollision(state, dt);
                }
            }

            // Altitude ceiling for X500 — proportional to cruiseAlt, minimum 12m
            const x500MaxAlt = Math.max(12, prof.cruiseAlt * 2.5);
            if (state.position.y > x500MaxAlt) {
                state.position.y = x500MaxAlt;
                state.velocity.y = Math.min(0, state.velocity.y);
            }

            // Boundary hard clamp
            const halfZone = this._getHalfZone();
            if (Math.abs(state.position.x) > halfZone) {
                state.position.x = Math.sign(state.position.x) * halfZone;
                state.velocity.x *= -0.3;
            }
            if (Math.abs(state.position.z) > halfZone) {
                state.position.z = Math.sign(state.position.z) * halfZone;
                state.velocity.z *= -0.3;
            }

            } // end else (not LANDED)
            const hSpeed = Math.sqrt(state.velocity.x ** 2 + state.velocity.z ** 2);
            this._updateCoverage(state, hSpeed);

            return;
        }
        // ═══ END X500 PATH ═══

        // LANDED kinematic drones — resting on ground, no physics needed
        if (state.phase === 'LANDED') {
            return;
        }

        // 2. PID → desired velocity
        const errX = state.waypoint.x - state.position.x;
        const errZ = state.waypoint.z - state.position.z;
        const errY = state.waypoint.y - state.position.y;

        let vxCmd = state.pidX.compute(errX, dt);
        let vzCmd = state.pidZ.compute(errZ, dt);
        let vyCmd = state.pidY.compute(errY, dt);

        // 2b. Approach deceleration — DISABLED in organic mode (coverage priority)
        // In organic mode, we want drones to maintain speed and quickly cover area.
        // In swarm mode, gentle deceleration for precision waypoint arrival.
        const distToWP = Math.sqrt(errX * errX + errZ * errZ);
        if (!this.useOrganicMode) {
            const decelRadius = 5.0;
            if (distToWP < decelRadius && distToWP > 0.5) {
                const decelFactor = 0.3 + 0.7 * (distToWP / decelRadius);
                vxCmd *= decelFactor;
                vzCmd *= decelFactor;
            }
        }

        // ─── DIAGNOSTIC: capture pre-avoidance velocity ───
        const isDiagDrone = this._diagEnabled && state.id === this.drones.keys().next().value;
        let diagPreAvoid, diagTreeF, diagDroneF, diagBoundF;
        if (isDiagDrone) {
            diagPreAvoid = { x: vxCmd, z: vzCmd, y: vyCmd };
            diagTreeF = { x: 0, z: 0, y: 0 };
            diagDroneF = { x: 0, z: 0 };
            diagBoundF = { x: 0, z: 0 };
        }

        // 3-5. Avoidance — active during EXPLORE/HOVER/TAKEOFF/LAND
        // During LAND: reduced avoidance so PID drives toward landing target
        if (state.phase === 'EXPLORE' || state.phase === 'HOVER' || state.phase === 'TAKEOFF' || state.phase === 'LAND') {
            // 3. Tree avoidance (repulsion force)
            const avoidance = this._computeTreeAvoidance(state);
            vxCmd += avoidance.x;
            vzCmd += avoidance.z;
            vyCmd += avoidance.y;
            if (isDiagDrone) diagTreeF = { x: avoidance.x, z: avoidance.z, y: avoidance.y };

            // 4. Drone-drone avoidance (3D: horizontal + vertical)
            //    Smooth the avoidance force to prevent abrupt impulses
            const rawDroneAvoid = this._computeDroneAvoidance(state);
            if (!state._smoothAvoid) state._smoothAvoid = new THREE.Vector3();
            const avoidAlpha = 0.35; // responsive: converges in ~3 frames
            state._smoothAvoid.x += (rawDroneAvoid.x - state._smoothAvoid.x) * avoidAlpha;
            state._smoothAvoid.y += (rawDroneAvoid.y - state._smoothAvoid.y) * avoidAlpha;
            state._smoothAvoid.z += (rawDroneAvoid.z - state._smoothAvoid.z) * avoidAlpha;
            // During LAND: reduce drone avoidance so PID can reach landing slot
            // Hard collision safety net still prevents actual overlap
            const droneAvoidScale = state.phase === 'LAND' ? 0.25 : 1.0;
            vxCmd += state._smoothAvoid.x * droneAvoidScale;
            vyCmd += state._smoothAvoid.y * droneAvoidScale;
            vzCmd += state._smoothAvoid.z * droneAvoidScale;
            if (isDiagDrone) diagDroneF = { x: state._smoothAvoid.x, z: state._smoothAvoid.z };

            // ════════════════════════════════════════════════════════════════
            // 4a. REYNOLDS FLOCKING: alignment + cohesion (real inter-agent
            // influence — the physical basis for measurable emergence).
            // ═══ DISABLED DURING RTH: returning drones fly direct to home ═══
            // ════════════════════════════════════════════════════════════════
            if (!state._returningToBase) {
                const flockForce = this._computeFlockingForce(state);
                const flockScale = 0.2 + 0.8 * (this.autonomyLevel / 100);
                flockForce.x *= flockScale;
                flockForce.z *= flockScale;
                if (!state._smoothFlock) state._smoothFlock = { x: 0, z: 0 };
                const flockAlpha = 0.35;
                state._smoothFlock.x += (flockForce.x - state._smoothFlock.x) * flockAlpha;
                state._smoothFlock.z += (flockForce.z - state._smoothFlock.z) * flockAlpha;
                vxCmd += state._smoothFlock.x;
                vzCmd += state._smoothFlock.z;
            } else {
                // RTH: decay flocking smoothly to zero
                if (state._smoothFlock) {
                    state._smoothFlock.x *= 0.9;
                    state._smoothFlock.z *= 0.9;
                }
            }

            // 4b. Swarm intelligence velocity modulation (pheromone gradient)
            // ═══ DISABLED DURING RTH — pheromone pull fights direct approach ═══
            const stigmergyCOA = this.doctrineManager?.currentCOA?.id === 'stigmergy'
                              || this.doctrineManager?.currentDoctrine?.params?.usePheromones;
            if (!state._returningToBase && (stigmergyCOA || (!this.useOrganicMode && this.autonomyLevel < 75))) {
                const modulated = this.swarmIntelligence.modulateVelocity(
                    state.id, { x: vxCmd, y: vyCmd, z: vzCmd }, state
                );
                vxCmd = modulated.x;
                vyCmd = modulated.y;
                vzCmd = modulated.z;
            }

            // 4c. Boundary containment — soft repulsion near zone edges
            const boundary = this._computeBoundaryForce(state);
            vxCmd += boundary.x;
            vzCmd += boundary.z;
            if (isDiagDrone) diagBoundF = { x: boundary.x, z: boundary.z };
        }

        // 5. Clamp speed AFTER all forces — prevents overshoot from avoidance
        // During LAND: tighter envelope for gentle controlled descent
        const landSpeedMul = state.phase === 'LAND' ? 0.5 : 1.0;
        const maxH = prof.maxSpeed * landSpeedMul;
        const maxV = prof.maxClimb * landSpeedMul;
        const hSpeed = Math.sqrt(vxCmd * vxCmd + vzCmd * vzCmd);
        if (hSpeed > maxH) {
            const scale = maxH / hSpeed;
            vxCmd *= scale; vzCmd *= scale;
        }
        vyCmd = Math.max(-maxV, Math.min(maxV, vyCmd));

        // 6. Smooth velocity (exponential low-pass — frame-rate independent)
        // During LAND: moderate convergence (not too fast to cause jitter)
        const agilityMul = state.phase === 'LAND' ? 6.0 : 3.0;
        const alpha = 1 - Math.exp(-prof.agility * agilityMul * dt);
        state.smoothVelocity.x += (vxCmd - state.smoothVelocity.x) * alpha;
        state.smoothVelocity.y += (vyCmd - state.smoothVelocity.y) * alpha;
        state.smoothVelocity.z += (vzCmd - state.smoothVelocity.z) * alpha;

        // 6b. Minimum velocity enforcement during EXPLORE — prevent frozen appearance
        // If smooth velocity is near-zero but we have a waypoint, give a gentle nudge
        if (state.phase === 'EXPLORE' && state.waypoint) {
            const sv2D = Math.sqrt(state.smoothVelocity.x ** 2 + state.smoothVelocity.z ** 2);
            const minExploreSpeed = 0.6; // m/s floor — drone should always visibly move
            if (sv2D < minExploreSpeed && sv2D > 0.001) {
                const boost = minExploreSpeed / sv2D;
                state.smoothVelocity.x *= boost;
                state.smoothVelocity.z *= boost;
            } else if (sv2D <= 0.001) {
                // Dead stop — kick toward waypoint
                const toWpX = state.waypoint.x - state.position.x;
                const toWpZ = state.waypoint.z - state.position.z;
                const toWpLen = Math.sqrt(toWpX * toWpX + toWpZ * toWpZ);
                if (toWpLen > 0.1) {
                    state.smoothVelocity.x = (toWpX / toWpLen) * minExploreSpeed;
                    state.smoothVelocity.z = (toWpZ / toWpLen) * minExploreSpeed;
                }
            }
        }

        // 7. Integrate position
        state.position.x += state.smoothVelocity.x * dt;
        state.position.y += state.smoothVelocity.y * dt;
        state.position.z += state.smoothVelocity.z * dt;

        // 8. Tree hard collision
        if (state.phase !== 'LANDED') this._enforceTreeCollision(state, dt);

        // 8. Drone-drone hard collision — active during LAND for realistic avoidance
        if (state.phase !== 'LANDED') this._enforceDroneCollision(state, dt);

        // 8a. HARD ALTITUDE CEILING — proportional to cruiseAlt (min 12m)
        //     Ceiling = cruiseAlt × 2.5 — enough headroom for tree avoidance overshoot
        //     MUST be AFTER tree collision so ceiling has final authority
        const maxAltitude = Math.max(12, prof.cruiseAlt * 2.5);
        if (state.position.y > maxAltitude) {
            state.position.y = maxAltitude;
            state.smoothVelocity.y = Math.min(-0.5, state.smoothVelocity.y); // force descent
        } else if (state.position.y > maxAltitude * 0.85) {
            // Soft ceiling zone — progressive damping of upward velocity
            const overFrac = (state.position.y - maxAltitude * 0.85) / (maxAltitude * 0.15);
            const maxUpward = prof.maxClimb * (1 - overFrac);
            state.smoothVelocity.y = Math.min(maxUpward, state.smoothVelocity.y);
        }

        // 8b. Ground clamp — terrain-aware (HARD clamp — drones must NEVER go below ground)
        const groundHeight = this._groundY(state.position.x, state.position.z);
        const ABSOLUTE_FLOOR = 0.15; // platform surface — NEVER go below this in world space
        const effectiveGround = Math.max(groundHeight, ABSOLUTE_FLOOR);
        // During LAND/LANDED, allow descent to surface — only enforce absolute floor
        if (state.phase === 'LAND' || state.phase === 'LANDED') {
            // Type-aware resting height: X500/S500 landing gear needs clearance
            const restY = state.x500Dynamics
                ? effectiveGround + (state.profile.scale || 10) * 0.22  // match spawn height
                : effectiveGround + 0.05;
            if (state.position.y < restY) {
                // Soft spring: push toward restY with damping (frame-rate independent)
                const penetration = restY - state.position.y;
                state.position.y += penetration * (1 - Math.exp(-10.0 * dt));
                state.smoothVelocity.y *= Math.exp(-10.0 * dt);
            }
        } else {
            // Type-aware minimum altitude: X500/S500 have large landing gear
            // that extends ~1m below mesh origin — need extra clearance above grass (~0.6m)
            const extraGear = state.x500Dynamics ? 1.2 : 0;
            const minY = effectiveGround + this.minAltitudeAboveGround + extraGear;
            if (state.position.y < minY) {
                state.position.y = minY; // HARD clamp
                state.smoothVelocity.y = Math.max(0.5, state.smoothVelocity.y);
            }
        }

        // 8c. Horizontal boundary — soft push inward (no instant velocity=0)
        const halfZone = this._getHalfZone();
        const boundaryMargin = 2.0;
        let boundClampX = false, boundClampZ = false;
        if (Math.abs(state.position.x) > halfZone - boundaryMargin) {
            const overX = Math.abs(state.position.x) - (halfZone - boundaryMargin);
            const pushX = -Math.sign(state.position.x) * overX * 2.0;
            state.smoothVelocity.x += pushX * dt * 10;
            if (Math.abs(state.position.x) > halfZone) {
                state.position.x = Math.sign(state.position.x) * halfZone;
                state.smoothVelocity.x *= 0.5; // dampen, don't zero
            }
            boundClampX = true;
        }
        if (Math.abs(state.position.z) > halfZone - boundaryMargin) {
            const overZ = Math.abs(state.position.z) - (halfZone - boundaryMargin);
            const pushZ = -Math.sign(state.position.z) * overZ * 2.0;
            state.smoothVelocity.z += pushZ * dt * 10;
            if (Math.abs(state.position.z) > halfZone) {
                state.position.z = Math.sign(state.position.z) * halfZone;
                state.smoothVelocity.z *= 0.5;
            }
            boundClampZ = true;
        }

        // ─── DIAGNOSTIC LOG (sampled, first drone only) ───
        if (isDiagDrone && this._diagFrame % this._diagInterval === 0) {
            const spd = Math.sqrt(state.smoothVelocity.x ** 2 + state.smoothVelocity.z ** 2);
            const wpDist = state.waypoint ? Math.sqrt(
                (state.position.x - state.waypoint.x) ** 2 +
                (state.position.z - state.waypoint.z) ** 2
            ).toFixed(1) : '?';
            console.log(
                `[DIAG] ${state.id} phase=${state.phase} ` +
                `pos=(${state.position.x.toFixed(1)},${state.position.y.toFixed(1)},${state.position.z.toFixed(1)}) ` +
                `spd=${spd.toFixed(2)} wpDist=${wpDist} ` +
                `pidCmd=(${diagPreAvoid.x.toFixed(2)},${diagPreAvoid.z.toFixed(2)}) ` +
                `treeF=(${diagTreeF.x.toFixed(2)},${diagTreeF.z.toFixed(2)}) ` +
                `droneF=(${diagDroneF.x.toFixed(2)},${diagDroneF.z.toFixed(2)}) ` +
                `boundF=(${diagBoundF.x.toFixed(2)},${diagBoundF.z.toFixed(2)}) ` +
                `clamp=${boundClampX || boundClampZ ? 'YES' : 'no'} ` +
                `wp#=${state.waypointsVisited}`
            );
        }

        // 9. Update heading (face movement direction)
        //    Rate-limited like the X500 path to prevent yaw oscillation.
        //    The PID output is treated as an angular rate (rad/s),
        //    multiplied by dt, then clamped to max angular rate.
        if (hSpeed > 0.3) {
            const targetYaw = Math.atan2(state.smoothVelocity.x, state.smoothVelocity.z);
            let yawErr = targetYaw - state.heading;
            while (yawErr > Math.PI) yawErr -= 2 * Math.PI;
            while (yawErr < -Math.PI) yawErr += 2 * Math.PI;
            const yawRate = state.pidYaw.compute(yawErr, dt);
            // Rate-limit: max ~200°/s for agile micro-drones, smooth visual heading
            const maxAngRate = 3.5; // rad/s
            const yawDelta = Math.max(-maxAngRate * dt, Math.min(maxAngRate * dt, yawRate * dt));
            state.heading += yawDelta;
        }
        // Wrap heading to [-π, π]
        while (state.heading > Math.PI) state.heading -= 2 * Math.PI;
        while (state.heading < -Math.PI) state.heading += 2 * Math.PI;

        // 10. Coverage tracking + swarm intelligence (shared helper)
        this._updateCoverage(state, hSpeed);

        // Store velocity for external use
        state.velocity.copy(state.smoothVelocity);
    }

    // ─── Coverage tracking + swarm intelligence (shared by kinematic & X500 paths) ──
    _updateCoverage(state, hSpeed) {
        // Mark current cell + neighbors within sensor radius (~5m = 2 cells radius)
        const cx = Math.floor(state.position.x / this.cellSize);
        const cz = Math.floor(state.position.z / this.cellSize);
        const currentCellKey = `${cx},${cz}`;
        this.visitedCells.add(currentCellKey);
        // Splash: mark ring-2 neighbors as visited (drone sensor FOV ~5m around it)
        for (let sdx = -2; sdx <= 2; sdx++) {
            for (let sdz = -2; sdz <= 2; sdz++) {
                if (sdx === 0 && sdz === 0) continue;
                this.visitedCells.add(`${cx + sdx},${cz + sdz}`);
            }
        }
        // Per-drone memory: track recent cells to avoid backtracking
        if (!state._recentCellSet.has(currentCellKey)) {
            state._recentCells.push(currentCellKey);
            state._recentCellSet.add(currentCellKey);
            if (state._recentCells.length > state._recentCellsMax) {
                const evicted = state._recentCells.shift();
                state._recentCellSet.delete(evicted);
            }
        }

        // Swarm intelligence environment update
        // In organic mode: journal-only (no pheromone deposit) for coverage tracking
        // In swarm mode: full pheromone deposit + journal
        this.swarmIntelligence.updateEnvironment(state.id, state.position, {
            speed: hSpeed,
            heading: state.heading,
            phase: state.phase,
            waypointsVisited: state.waypointsVisited,
        });
    }

    // ─── State machine ──────────────────────────────────────────────
    _updateStateMachine(state, dt) {
        const prof = state.profile;

        switch (state.phase) {
            case 'TAKEOFF': {
                // Use layer-assigned cruise altitude if available, else profile default
                const takeoffCruise = state._cruiseAlt || prof.cruiseAlt;
                const takeoffGround = this._groundY(state.position.x, state.position.z);
                const takeoffTargetY = takeoffGround + takeoffCruise;
                // Track takeoff start altitude
                if (state._takeoffStartY === undefined) {
                    state._takeoffStartY = state.position.y;
                    state._takeoffTimer = 0;
                    // Pre-assign altitude layer during takeoff so it's ready for HOVER/EXPLORE
                    this._preAssignAltitudeLayer(state);
                }
                state._takeoffTimer += dt;
                // Recalculate with potentially updated _cruiseAlt from pre-assignment
                const takeoffCruiseUpdated = state._cruiseAlt || prof.cruiseAlt;
                const takeoffTargetYUpdated = takeoffGround + takeoffCruiseUpdated;
                // Re-anchor: if the drone was pushed above the S-curve by ground clamp,
                // update the start position so the ramp never commands downward.
                if (state.position.y > state._takeoffStartY) {
                    state._takeoffStartY = state.position.y;
                }
                // Smooth S-curve climb: slow start, accelerate, slow finish
                // Uses smoothstep easing for natural, fluid ascent
                const totalClimbDist = takeoffTargetYUpdated - state._takeoffStartY;
                const baseClimbRate = state.x500Dynamics ? 1.2 : 1.0; // gentler climb
                const climbDuration = Math.max(1.5, Math.abs(totalClimbDist) / baseClimbRate);
                const t = Math.min(1.0, state._takeoffTimer / climbDuration);
                // Smoothstep: 3t² - 2t³ (slow start, fast middle, slow finish)
                const eased = t * t * (3 - 2 * t);
                const rampedY = state._takeoffStartY + eased * totalClimbDist;
                state.waypoint = state.position.clone();
                state.waypoint.y = Math.max(state.position.y, Math.min(takeoffTargetYUpdated, rampedY));
                state._lastAction = 'CLIMBING';
                if (state.position.y > takeoffTargetYUpdated * 0.90) {
                    state.phase = 'HOVER';
                    state.lastWaypointTime = 0;
                    state._lastDecision = `ALT ${state.position.y.toFixed(1)}m → HOVER`;
                    // Clean up takeoff ramp state
                    delete state._takeoffStartY;
                    delete state._takeoffTimer;
                    // Save the hover hold position (fixed X/Z)
                    state.hoverPosition = state.position.clone();
                    state.hoverPosition.y = takeoffTargetYUpdated;
                    console.log(`[STATE-MACHINE] 🛫 ${state.id} TAKEOFF → HOVER (alt=${state.position.y.toFixed(2)}, target=${takeoffCruiseUpdated.toFixed(1)}m, terrain=${takeoffGround.toFixed(1)})`);
                }
                break;
            }
            case 'HOVER': {
                // Hold position — PID targets saved hover position
                // No external forces (avoidance disabled in non-EXPLORE phases)
                // Velocity damping ensures clean stop
                const hoverCruise = state._cruiseAlt || prof.cruiseAlt;
                const hoverGround = this._groundY(state.hoverPosition?.x ?? state.position.x, state.hoverPosition?.z ?? state.position.z);
                const hoverTargetY = hoverGround + hoverCruise;
                if (!state.hoverPosition) {
                    state.hoverPosition = state.position.clone();
                    state.hoverPosition.y = hoverTargetY;
                }
                if (!state.waypoint) state.waypoint = state.hoverPosition.clone();
                state.waypoint.x = state.hoverPosition.x;
                state.waypoint.z = state.hoverPosition.z;
                state.waypoint.y = hoverTargetY;
                // Dampen horizontal velocity — PID converges to hold point
                state.smoothVelocity.x *= 0.92;
                state.smoothVelocity.z *= 0.92;
                state._lastAction = 'HOLD POS';
                break;
            }
            case 'EXPLORE': {
                state.waypointTimer += dt;

                // ── Return-to-base: fly to original spawn position, then land ──
                if (state._returningToBase) {
                    const home = state.homePosition;
                    const dx = state.position.x - home.x;
                    const dz = state.position.z - home.z;
                    const distToHome = Math.sqrt(dx * dx + dz * dz);
                    if (distToHome < 1.5) {
                        // Close enough to home slot — transition to LAND
                        state.phase = 'LAND';
                        state._returningToBase = false;
                        state._landInPlace = false;
                        // Snapshot home as FIXED landing target
                        state._landTarget = home.clone();
                        state._landTimer = 0;
                        state._landStartY = state.position.y; // S-curve descent start altitude
                        // Reset PIDs to clear integral buildup from flight
                        if (state.pidX) { state.pidX.reset(); state.pidY.reset(); state.pidZ.reset(); }
                        if (state.x500Dynamics) state.x500Dynamics.resetPIDs();
                        // Clear residual avoidance forces
                        if (state._smoothAvoidance) state._smoothAvoidance.set(0, 0, 0);
                        if (state._smoothAvoid) state._smoothAvoid.set(0, 0, 0);
                        if (state._smoothFlock) { state._smoothFlock.x = 0; state._smoothFlock.z = 0; }
                        console.log(`[STATE-MACHINE] 🏠 ${state.id} EXPLORE → LAND (at home slot x=${home.x.toFixed(1)}, z=${home.z.toFixed(1)})`);
                        break;
                    }
                    // ═══ STAGGERED RTH ALTITUDE — deconflict approach corridors ═══
                    // Each drone flies at a unique altitude offset based on its index
                    // to prevent all drones converging at the same height.
                    const rthBaseCruise = state._cruiseAlt || prof.cruiseAlt;
                    const match = String(state.id).match(/(\d+)$/);
                    const droneIdx = match ? parseInt(match[1], 10) - 1 : 0;
                    const rthAltOffset = (droneIdx % 5) * 1.2; // 0m, 1.2m, 2.4m, 3.6m, 4.8m stagger
                    const rthCruise = rthBaseCruise + rthAltOffset;
                    const rthGroundY = this._groundY(home.x, home.z);
                    // ═══ APPROACH DECELERATION — smooth arrival to home ═══
                    // When close (<15m), start descending toward cruise altitude
                    // When very close (<5m), lower altitude for landing approach
                    let rthAlt = rthCruise;
                    if (distToHome < 15) {
                        // Smooth altitude decrease: stagger → base cruise over last 15m
                        const approachFactor = distToHome / 15;
                        rthAlt = rthBaseCruise + rthAltOffset * approachFactor;
                    }
                    if (distToHome < 5) {
                        // Final approach: descend partially toward landing
                        const finalFactor = distToHome / 5;
                        rthAlt = rthBaseCruise * (0.6 + 0.4 * finalFactor);
                    }
                    state.waypoint = new THREE.Vector3(home.x, rthGroundY + rthAlt, home.z);
                    state.waypointTimer = 0;
                    state._stallTimer = 0;
                    state._lastAction = '🏠 RTH';
                    state._lastDecision = `RTH d=${distToHome.toFixed(0)}m alt=${rthAlt.toFixed(1)}`;
                    break;
                }

                // ── Live altitude sync ──────────────────────────────────
                // When the UI slider changes _cruiseAlt mid-flight, update
                // the current waypoint's Y so the PID immediately steers to
                // the new altitude instead of waiting for the next waypoint.
                if (state.waypoint) {
                    const liveCruise = state._cruiseAlt || prof.cruiseAlt;
                    const wpGroundY = this._groundY(state.waypoint.x, state.waypoint.z);
                    state.waypoint.y = wpGroundY + liveCruise;
                }

                const distToWP = state.waypoint ?
                    Math.sqrt(
                        (state.position.x - state.waypoint.x) ** 2 +
                        (state.position.z - state.waypoint.z) ** 2
                    ) : Infinity;

                // Stall detection: if drone barely moves, force new waypoint sooner
                const speed2D = Math.sqrt(state.smoothVelocity.x ** 2 + state.smoothVelocity.z ** 2);
                if (!state._stallTimer) state._stallTimer = 0;
                if (speed2D < 0.5) { // raised from 0.3 — oscillating at 0.4 still looks frozen
                    state._stallTimer += dt;
                } else {
                    state._stallTimer = Math.max(0, state._stallTimer - dt * 0.3); // slower decay — need sustained movement to clear
                }

                // Oscillation detection: track velocity sign changes
                if (!state._lastVelSign) state._lastVelSign = { x: 0, z: 0 };
                if (!state._oscillationCount) state._oscillationCount = 0;
                if (!state._oscillationTimer) state._oscillationTimer = 0;
                const signX = Math.sign(state.smoothVelocity.x);
                const signZ = Math.sign(state.smoothVelocity.z);
                if ((signX !== 0 && signX !== state._lastVelSign.x) || (signZ !== 0 && signZ !== state._lastVelSign.z)) {
                    state._oscillationCount++;
                }
                state._lastVelSign.x = signX || state._lastVelSign.x;
                state._lastVelSign.z = signZ || state._lastVelSign.z;
                state._oscillationTimer += dt;
                if (state._oscillationTimer > 2.0) {
                    // Reset count every 2s window
                    state._oscillationTimer = 0;
                    state._oscillationCount = 0;
                }
                const isOscillating = state.x500Dynamics
                    ? state._oscillationCount > 12  // X500/S500: physics oscillation is natural during avoidance
                    : state._oscillationCount > 6;  // Crazyflie: kinematic shouldn't oscillate this much
                if (isOscillating) {
                    state._stallTimer = 999; // force immediate re-pick
                    state._oscillationCount = 0;
                }

                const stallTimeout = state.x500Dynamics
                    ? state._stallTimer > 2.5  // X500/S500: longer grace — physics needs time to resolve avoidance
                    : state._stallTimer > 0.8; // Crazyflie: react faster (kinematic model)

                // Pick a new waypoint when close, timeout, or stalled
                // Dynamic timeout: proportional to initial waypoint distance (min 2.5s)
                const wpTimeout = Math.max(2.5, (state._wpInitialDist || 10) / prof.maxSpeed * 2.0);
                if (!state.waypoint || distToWP < EXPLORE.waypointReachDist || state.waypointTimer > wpTimeout || stallTimeout) {
                    let targetWP;
                    const autonomy = this.autonomyLevel;

                    // ═══ MISSION WAYPOINT PROVIDER (highest priority) ═══
                    // If an LLM mission is active, it provides search-pattern waypoints
                    if (this.missionWaypointProvider) {
                        try {
                            const missionWP = this.missionWaypointProvider(state.id, state);
                            // Garde anti-NaN : un waypoint non-fini (provider LLM défaillant)
                            // empoisonnerait le PID → drone erratique. On l'ignore proprement.
                            if (missionWP && Number.isFinite(missionWP.x) && Number.isFinite(missionWP.z)) {
                                const terrY = this._groundY(missionWP.x, missionWP.z);
                                const cruiseAlt = state._cruiseAlt || state.profile.cruiseAlt;
                                targetWP = new THREE.Vector3(missionWP.x, terrY + cruiseAlt, missionWP.z);
                                state.waypoint = targetWP;
                                state._wpInitialDist = Math.sqrt((state.position.x - targetWP.x) ** 2 + (state.position.z - targetWP.z) ** 2);
                                state._lastAction = missionWP._label || '🎯 MISSION';
                                state._lastDecision = `MISSION d=${state._wpInitialDist.toFixed(0)}m`;
                                state.waypointTimer = 0;
                                state._stallTimer = 0;
                                // Skip autonomy-driven selection but continue to event dispatch
                            }
                        } catch(e) {
                            console.warn('[ENGINE] missionWaypointProvider error:', e.message);
                        }
                    }

                    // ═══ AUTONOMY-DRIVEN WAYPOINT SELECTION (only if no mission waypoint) ═══
                    if (!this.missionWaypointProvider || !targetWP) {
                    if (autonomy < 25) {
                        // CENTRALISÉ: formation serrée, mouvement coordonné
                        targetWP = this._pickFormationWaypoint(state);
                        state.waypoint = targetWP;
                        state._wpInitialDist = Math.sqrt((state.position.x - targetWP.x) ** 2 + (state.position.z - targetWP.z) ** 2);
                        state._lastAction = 'FORMATION';
                        state._lastDecision = `FORM d=${state._wpInitialDist.toFixed(0)}m`;
                    } else if (autonomy < 75) {
                        // HYBRIDE: sub-groups with scatter
                        targetWP = this._pickHybridWaypoint(state);
                        state.waypoint = this._getPathAwareWaypoint(state, targetWP);
                        state._wpInitialDist = Math.sqrt((state.position.x - state.waypoint.x) ** 2 + (state.position.z - state.waypoint.z) ** 2);
                        state._lastAction = 'HYBRID GRP';
                        state._lastDecision = `GRP d=${state._wpInitialDist.toFixed(0)}m`;
                    } else if (this.useOrganicMode) {
                        // DISTRIBUÉ + ORGANIC MODE — v0 spiral + collaborative repulsion
                        if (stallTimeout) {
                            // Progressive escape: repeated stalls → bigger jumps
                            if (!state._escaleCount) state._escaleCount = 0;
                            state._escaleCount++;
                            targetWP = this._pickEscapeWaypoint(state);
                            state._stallTimer = 0;
                            state._lastAction = `ESCAPE #${state._escaleCount}`;
                            state._lastDecision = `STALL → ESCAPE r=${state._escaleCount}`;
                        } else {
                            state._escaleCount = 0; // reset escape counter when moving normally

                            // ═══ COGNITIVE → ANT DIRECTIVE: redirect if a cognitive drone sent an order ═══
                            const directive = this.commManager?.getDirective(state.id);
                            if (directive && directive.waypoint) {
                                const terrY = this._groundY(directive.waypoint.x, directive.waypoint.z);
                                const cruiseAlt = state._cruiseAlt || state.profile.cruiseAlt;
                                targetWP = new THREE.Vector3(directive.waypoint.x, terrY + cruiseAlt, directive.waypoint.z);
                                this.commManager.consumeDirective(state.id);
                                state._lastAction = `📨 REDIR←${directive.fromId.split('_')[1]}`;
                                state._lastDecision = `CMD cog→ant`;
                            } else {
                                targetWP = this._pickExplorationWaypoint(state);
                                // Action label reflects actual control mode
                                if (this.autonomyLevel >= 90) {
                                    // Full autonomy: no orchestrator, self-organised
                                    state._lastAction = '🔄 AUTO-ORG';
                                } else if (this.doctrineManager) {
                                    const dName = this.doctrineManager.currentDoctrine?.id?.toUpperCase() || 'EXPL';
                                    const cName = this.doctrineManager.currentCOA?.id?.toUpperCase() || 'ADAPT';
                                    state._lastAction = `${dName}/${cName}`;
                                } else {
                                    state._lastAction = 'ORGANIC EXPL';
                                }
                            }
                        }
                        state.waypoint = targetWP;
                        state._wpInitialDist = Math.sqrt((state.position.x - targetWP.x) ** 2 + (state.position.z - targetWP.z) ** 2);
                        state._lastDecision = `WP d=${state._wpInitialDist.toFixed(0)}m`;
                    } else {
                        // DISTRIBUÉ + SWARM INTELLIGENCE MODE — termite/voronoi sector sweep
                        const neighbors = this._getNeighbors(state.id);
                        const suggested = this.swarmIntelligence.computeNextWaypoint(
                            state.id, state, neighbors
                        );
                        targetWP = suggested || this._pickExplorationWaypoint(state);
                        // Utiliser pathfinder pour éviter les obstacles
                        state.waypoint = this._getPathAwareWaypoint(state, targetWP);
                        state._wpInitialDist = Math.sqrt((state.position.x - state.waypoint.x) ** 2 + (state.position.z - state.waypoint.z) ** 2);
                        state._lastAction = suggested ? 'SWARM INTEL' : 'ORGANIC EXPL';
                        state._lastDecision = suggested ? `SWARM d=${state._wpInitialDist.toFixed(0)}m` : `EXPL d=${state._wpInitialDist.toFixed(0)}m`;
                    }
                    } // end: autonomy-driven selection guard
                    // ═══ LLM INTELLIGENCE INFLUENCE (if active) ═══
                    const llmResult = this._llmResults.get(state.id);
                    if (llmResult) {
                        // Apply LLM reasoning to label
                        if (llmResult.reasoning) {
                            state._lastReasoning = llmResult.reasoning;
                        }
                        // Safety override
                        if (llmResult.safetyAction) {
                            state._lastDecision = `🛡️ ${llmResult.safetyAction}`;
                        }
                        // LLM directional influence: bias the waypoint
                        if (llmResult.influence?.direction && state.waypoint && llmResult.influence.weight > 0) {
                            const dir = llmResult.influence.direction;
                            const w = llmResult.influence.weight;
                            const biasStrength = 8.0; // meters — how far LLM can shift the waypoint
                            state.waypoint.x += dir.x * biasStrength * w;
                            state.waypoint.z += dir.z * biasStrength * w;
                            // Re-clamp to bounds
                            const hb = this._getHalfBoundsXZ();
                            state.waypoint.x = Math.max(-hb.x + 2, Math.min(hb.x - 2, state.waypoint.x));
                            state.waypoint.z = Math.max(-hb.z + 2, Math.min(hb.z - 2, state.waypoint.z));
                        }
                        // LLM weight adjustments for scoring
                        if (llmResult.influence?.adjustWeights) {
                            state._llmWeights = llmResult.influence.adjustWeights;
                        }
                    } else if (!this.intelligenceManager?.connector?.isAvailable) {
                        // ── DEMO MODE: generate simulated LLM reasoning when Ollama is offline ──
                        const demoReasons = [
                            'Analyse thermique → zone prioritaire N-E',
                            'Couverture sectorielle optimale détectée',
                            'Évitement obstacle + replanification',
                            'Coordination essaim → réduire redondance',
                            'Détection mouvement → investigation',
                            'Signal faible → approche prudente',
                            'Topologie favorable → accélération',
                            'Cellules voisines non-visitées → expansion',
                            'Vent contraire → altitude +2m recommandée',
                            'Couverture locale 87% → transition secteur',
                            'Analyse terrain → contournement colline',
                            'Peer #' + ((parseInt(state.id) + 3) % this.drones.size) + ' couvre déjà → déviation',
                        ];
                        state._lastReasoning = demoReasons[Math.floor(Math.random() * demoReasons.length)];
                    }

                    // ── Fire async LLM evaluation for NEXT decision ──
                    this._triggerLLMEvaluation(state);

                    state.waypointTimer = 0;
                    state.waypointsVisited++;

                    // ── Emit decision event for orchestration console ──
                    if (typeof window !== 'undefined') {
                        const dm = this.doctrineManager;
                        window.dispatchEvent(new CustomEvent('diamants:drone-decision', {
                            detail: {
                                droneId: state.id,
                                action: state._lastAction,
                                phase: state.phase,
                                autonomy: this.autonomyLevel,
                                waypointsVisited: state.waypointsVisited,
                                doctrine: dm?.currentDoctrine?.name || '',
                                coa: dm?.currentCOA?.name || '',
                            }
                        }));
                    }
                }
                break;
            }
            case 'LAND': {
                // ═══ S-CURVE DESCENT LANDING ═══
                // Progressive altitude ramp: waypoint.y descends smoothly from start to rest.
                // Uses cosine S-curve for natural deceleration at top and bottom.
                const home = state.homePosition || state.position;
                if (!state._landTarget) {
                    state._landTarget = state._landInPlace
                        ? state.position.clone()
                        : (state._landingSlot ? new THREE.Vector3(state._landingSlot.x, state._landingSlot.y, state._landingSlot.z) : home.clone());
                }
                const landTarget = state._landTarget;
                const landTerrainY = this._groundY(landTarget.x, landTarget.z);
                const landGround = Math.max(0.15, landTerrainY);
                const landRestY = state.x500Dynamics
                    ? landGround + (state.profile.scale || 10) * 0.22
                    : landGround + 0.05;

                // Timer
                if (!state._landTimer) state._landTimer = 0;
                state._landTimer += dt;

                // S-curve descent: total duration proportional to altitude drop
                const startY = state._landStartY || state.position.y;
                const altDrop = Math.max(0.1, startY - landRestY);
                // ~1.5 m/s average descent → duration = altDrop / 1.5, min 2s, max 8s
                const landDuration = Math.max(2.0, Math.min(8.0, altDrop / 1.5));
                const t = Math.min(1.0, state._landTimer / landDuration);
                // Cosine S-curve: smooth acceleration then deceleration
                const sCurve = 0.5 - 0.5 * Math.cos(t * Math.PI);
                let descentY = startY + (landRestY - startY) * sCurve;

                // ═══ GROUND-SAFE CLAMP: prevent PID vs ground constraint fight ═══
                // Terrain under the drone's CURRENT position may differ from terrain at
                // landing target. Ensure waypoint.y never goes below current ground rest
                // height — otherwise PID pushes down while X500 ground constraint pushes
                // up, causing oscillation.
                const curTerrY = this._groundY(state.position.x, state.position.z);
                const curGround = Math.max(0.15, curTerrY);
                const curMinRestY = state.x500Dynamics
                    ? curGround + (state.profile.scale || 10) * 0.22
                    : curGround + 0.05;
                descentY = Math.max(descentY, curMinRestY);

                // Reuse waypoint Vector3 to avoid GC pressure (60 allocs/s per drone)
                if (!state.waypoint) state.waypoint = new THREE.Vector3();
                state.waypoint.set(landTarget.x, descentY, landTarget.z);
                state._lastAction = 'LANDING';

                // Check if landed — physics has brought us close enough
                const landAltOk = Math.abs(state.position.y - landRestY) < 0.3;
                const landHDist = Math.sqrt(
                    (state.position.x - landTarget.x) ** 2 +
                    (state.position.z - landTarget.z) ** 2
                );
                const landPosOk = landHDist < 1.0;
                const landTimeout = state._landTimer > landDuration + 4; // grace period after ramp done
                if ((landAltOk && landPosOk) || landTimeout) {
                    state.phase = 'LANDED';
                    // Smooth final settle instead of hard snap
                    state.velocity.set(0, 0, 0);
                    state.smoothVelocity.set(0, 0, 0);
                    state._landInPlace = false;
                    state._landTarget = null;
                    state._landTimer = 0;
                    state._landStartY = null;
                    state._landRestY = null;
                    console.log(`[LAND] ${state.id} → LANDED (restY=${landRestY.toFixed(2)}, ${landTimeout ? 'TIMEOUT' : 'POS'})`);
                }
                break;
            }
            case 'LANDED': {
                // Stay on surface — type-aware resting height
                const landedTerrY = this._groundY(state.position.x, state.position.z);
                const landedGround = Math.max(0.15, landedTerrY);
                const landedRestY = state.x500Dynamics
                    ? landedGround + (state.profile.scale || 10) * 0.22  // match spawn height — gear on ground
                    : landedGround + 0.05;
                state.waypoint = state.position.clone();
                state.waypoint.y = landedRestY;
                state.position.y = landedRestY;
                state.velocity.set(0, 0, 0);
                state.smoothVelocity.set(0, 0, 0);
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
        // and log the updated weights + bias for debugging
        if (dm && dm.addListener) {
            dm.addListener((event, data) => {
                if (event === 'doctrine') {
                    const W = this._getDoctrineWeights();
                    console.log(`[ENGINE] 📋 Doctrine → ${data.icon} ${data.name} | Weights: fwd=${W.forward} ter=${W.territory} rich=${W.richness} rep=${W.repulsion} bt=${W.backtrack}`);
                    this.flushDoctrineWaypoints();
                } else if (event === 'coa') {
                    console.log(`[ENGINE] 🎯 COA → ${data.icon} ${data.name} (${data.id === 'adaptive' ? 'pas de biais' : 'biais directionnel actif'})`);
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
        this._pathWaypoints.clear(); // Also clear intermediate paths
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

    /**
     * NOUVEAU: Configurer le pathfinder Dijkstra pour navigation 3D
     * @param {Object} pathfinder - Instance de pathfinder avec findPath(start, goal)
     * @param {Object} voxelizer - Instance de EnvironmentVoxelizer (optionnel)
     */
    setPathfinder(pathfinder, voxelizer = null) {
        this.pathfinder = pathfinder;
        this.voxelizer = voxelizer;
        console.log('[ENGINE] 🧭 Pathfinder Dijkstra configuré');
    }

    /**
     * NOUVEAU: Obtenir le prochain waypoint via pathfinding
     * Si le chemin direct vers la cible a un obstacle, calcule un chemin
     */
    _getPathAwareWaypoint(state, finalTarget) {
        if (!this.pathfinder || !finalTarget) return finalTarget;

        const start = { x: state.position.x, y: state.position.y, z: state.position.z };
        const goal = { x: finalTarget.x, y: finalTarget.y, z: finalTarget.z };

        // Check if we have a cached path to this target
        let pathCache = this._pathWaypoints.get(state.id);
        if (pathCache && pathCache.length > 0) {
            // Check if we're close to current waypoint
            const currentWP = pathCache[0];
            const dist = Math.sqrt(
                (state.position.x - currentWP.x) ** 2 +
                (state.position.z - currentWP.z) ** 2
            );
            if (dist < 1.5) {
                pathCache.shift(); // Move to next waypoint
            }
            if (pathCache.length > 0) {
                return new THREE.Vector3(pathCache[0].x, pathCache[0].y, pathCache[0].z);
            }
        }

        // Check if direct path is clear using voxelizer
        const directClear = !this.voxelizer?.grid || 
            this.voxelizer.grid.hasLineOfSight(start, goal);

        if (directClear) {
            this._pathWaypoints.delete(state.id);
            return finalTarget;
        }

        // Compute Dijkstra path
        try {
            const path = this.pathfinder.findPath(start, goal);
            if (path && path.length > 1) {
                // Skip first point (current position)
                const pathTail = path.slice(1);
                this._pathWaypoints.set(state.id, pathTail);
                console.log(`[PATHFINDER] ${state.id}: ${path.length} waypoints calculés`);
                return new THREE.Vector3(pathTail[0].x, pathTail[0].y, pathTail[0].z);
            }
        } catch (e) {
            console.warn('[PATHFINDER] Erreur calcul chemin:', e.message);
        }

        // Fallback: direct path
        return finalTarget;
    }

    // ─── Waypoint generation ─────────────────────────────────────────
    _pickExplorationWaypoint(state) {
        // ── 0. Initial dispersion — first waypoint sends drone to its territory center ──
        // Only at low/medium autonomy (≤75%) where centralized control applies.
        // At high autonomy (>75%), drones are PRE-POSITIONED at their sectors
        // (see integrated-controller spawn spread) — no waypoints needed.
        if (state._needsInitialDispersion && this.useOrganicMode && this.autonomyLevel <= 75) {
            if (!state._territoryCenter) {
                this._pickDefaultExplorationWaypoint(state);
            }
            if (state._territoryCenter) {
                state._needsInitialDispersion = false;
                const tc = state._territoryCenter;
                const terrY = this._groundY(tc.x, tc.z);
                const cruiseAlt = state._cruiseAlt || state.profile.cruiseAlt;
                console.log(`[DISPERSION] ${state.id} → territory (${tc.x.toFixed(0)}, ${tc.z.toFixed(0)})`);
                return new THREE.Vector3(tc.x, terrY + cruiseAlt, tc.z);
            }
        }
        state._needsInitialDispersion = false;
        // Record dispersion time for group heading ramp
        if (!state._dispersionStartTime) state._dispersionStartTime = this._simTime || 0;

        // ── 1. Doctrine + COA integration ──
        // HYBRID ARCHITECTURE: Instead of bypassing doctrine in organic mode,
        // we inject doctrine weights and COA directional bias INTO the organic
        // frontier-crawl scoring. This preserves tree avoidance, territory sectors,
        // and altitude layers while letting doctrine/COA influence exploration strategy.
        //
        // - Doctrine (EXPLORATION, SWARM, COVERAGE, SEARCH, FORMATION) → adjusts scoring WEIGHTS
        // - COA (GRID, BOUSTROPHEDON, SPIRAL, RADIAL, PERIMETER) → adds directional BIAS
        // - ADAPTIVE COA = zero bias = pure organic exploration (backward compatible)
        //
        // Non-organic fallback: rigid doctrine waypoints (legacy path)
        if (!this.useOrganicMode && this.doctrineManager) {
            const wp = this._pickDoctrineWaypoint(state);
            if (wp) return wp;
        }

        // ── 2. Territory-aware organic exploration (enhanced by doctrine+COA) ──
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
     * Default coverage-optimised exploration.
     *
     * ORGANIC MODE (useOrganicMode=true):
     *   3D territory-aware organic exploration — biomimetic.
     *   Each drone is assigned:
     *     - A territory center (angular sector on the XZ plane)
     *     - A cruise layer (altitude band) for vertical distribution
     *   Forest-realistic altitude layers:
     *     - Low canopy:   2–4m  (under branches, close inspection)
     *     - Mid canopy:   4–7m  (between trees, main scouting)
     *     - High canopy:  7–12m (above trees, overview/relay)
     *   60% of candidates biased toward territory, 40% free.
     *
     * LEGACY MODE (useOrganicMode=false):
     *   Candidates from DRONE position, uniform angular distribution.
     */

    /**
     * Pre-assign altitude layer during TAKEOFF so the drone knows its target
     * altitude before EXPLORE phase begins. This is a lightweight version of
     * the territory assignment in _pickDefaultExplorationWaypoint().
     */
    _preAssignAltitudeLayer(state) {
        if (state._cruiseAlt) return; // already assigned

        // Fallback 8: used for sector angle geometry (sensible default = 8 sectors)
        const fleetSize = this.drones.size || 8;
        const match = String(state.id).match(/(\d+)$/);
        const idx = match ? parseInt(match[1], 10) - 1 : 0;

        // All layers stay at trunk height (below canopy ~6m) to avoid leaves
        const LAYERS = [
            { min: 2.5, max: 3.5 },  // low  — ground-level inspection
            { min: 3.5, max: 4.5 },  // mid  — main trunk exploration
            { min: 4.5, max: 5.5 },  // high — upper trunk, still below canopy
        ];
        const layerIdx = idx % 3;
        const layer = LAYERS[layerIdx];
        const subIdx = Math.floor(idx / 3);
        const subCount = Math.max(1, Math.ceil(fleetSize / 3));
        const layerFraction = subCount > 1 ? subIdx / (subCount - 1) : 0.5;
        state._cruiseAlt = layer.min + layerFraction * (layer.max - layer.min);
        state._altLayer = layerIdx;

        console.log(`[PRE-ASSIGN] ${state.id}: cruiseAlt=${state._cruiseAlt.toFixed(1)}m layer=${['low','mid','high'][layerIdx]}`);
    }

    // ═══════════════════════════════════════════════════════════════════
    //  DOCTRINE WEIGHTS — adjust scoring component strengths per doctrine
    // ═══════════════════════════════════════════════════════════════════
    _getDoctrineWeights() {
        // Default weights (= current organic behavior when no doctrine selected)
        const base = { forward: 20, territory: 8, richness: 3, repulsion: 6, backtrack: 25, wpRepulsion: 4 };
        let weights;
        if (!this.doctrineManager) {
            weights = base;
        } else {
            switch (this.doctrineManager.currentDoctrine.id) {
                case 'exploration':
                    weights = { forward: 20, territory: 15, richness: 4, repulsion: 8, backtrack: 25, wpRepulsion: 6 };
                    break;
                case 'swarm':
                    weights = { forward: 15, territory: 10, richness: 3, repulsion: 15, backtrack: 20, wpRepulsion: 8 };
                    break;
                case 'formation':
                    weights = { forward: 25, territory: 20, richness: 2, repulsion: 4, backtrack: 30, wpRepulsion: 3 };
                    break;
                case 'coverage':
                    weights = { forward: 15, territory: 12, richness: 6, repulsion: 12, backtrack: 35, wpRepulsion: 8 };
                    break;
                case 'search':
                    weights = { forward: 10, territory: 8, richness: 5, repulsion: 6, backtrack: 15, wpRepulsion: 4 };
                    break;
                default:
                    weights = base;
            }
        }

        // ═══ AUTONOMY MODULATION ═══
        // Territory weight scales linearly with centralization.
        // At full autonomy (100): territory = 0 → pure self-organization,
        // no centrally-assigned sectors. Initial heading diversity + group
        // heading ramp are sufficient to prevent herding.
        const centralFactor = 1 - this.autonomyLevel / 100;
        weights.territory = Math.round(weights.territory * centralFactor);
        return weights;
    }

    // ═══════════════════════════════════════════════════════════════════
    //  COA BIAS — directional preference injected into cell scoring
    //  Returns a score modifier (positive = attract, negative = repel)
    // ═══════════════════════════════════════════════════════════════════
    _computeCOABias(cellX, cellZ, state) {
        if (!this.doctrineManager) return 0;
        const coa = this.doctrineManager.currentCOA;
        if (!coa || coa.id === 'adaptive') return 0; // Adaptatif = zero bias = organic pur

        const tc = state._territoryCenter || { x: 0, z: 0 };
        const halfZone = this._getHalfZone();

        switch (coa.id) {
            case 'grid': {
                // GRILLE: préférer les cellules alignées en rangées (balayage E-O)
                const rowSize = coa.params.cellSize || 5;
                const rowIdx = Math.floor(cellZ / rowSize);
                const direction = rowIdx % 2 === 0 ? 1 : -1;
                const xProgress = (cellX - state.position.x) * direction;
                // Récompenser le mouvement dans la bonne direction X, pénaliser les sauts de rangée
                const zDrift = Math.abs(cellZ - state.position.z);
                return (xProgress > 0 ? 12 : -6) - zDrift * 2;
            }
            case 'boustrophedon': {
                // BOUSTROPHÉDON: zigzag systématique, forte préférence rangée
                const spacing = coa.params.lineSpacing || 4;
                const rowIdx = Math.floor(cellZ / spacing);
                const direction = rowIdx % 2 === 0 ? 1 : -1;
                const xDir = cellX - state.position.x;
                const zDrift = Math.abs(cellZ - state.position.z);
                // Rester sur la rangée (pénalité Z), avancer dans la bonne direction X
                let bias = -zDrift * 3;
                bias += (xDir * direction > 0 ? 14 : -8);
                return bias;
            }
            case 'spiral': {
                // SPIRALE: progression angulaire autour du centre de territoire
                const currentAngle = Math.atan2(state.position.z - tc.z, state.position.x - tc.x);
                const cellAngle = Math.atan2(cellZ - tc.z, cellX - tc.x);
                let angleDelta = cellAngle - currentAngle;
                while (angleDelta > Math.PI) angleDelta -= 2 * Math.PI;
                while (angleDelta < -Math.PI) angleDelta += 2 * Math.PI;
                const clockwise = coa.params.clockwise !== false;
                const preferredDir = clockwise ? -1 : 1;
                // Récompenser la progression angulaire dans la direction préférée
                let bias = (angleDelta * preferredDir > 0 ? 12 : -8);
                // Légère expansion vers l'extérieur
                const fromCenter = Math.sqrt((state.position.x - tc.x) ** 2 + (state.position.z - tc.z) ** 2);
                const cellFromCenter = Math.sqrt((cellX - tc.x) ** 2 + (cellZ - tc.z) ** 2);
                bias += Math.min(5, (cellFromCenter - fromCenter) * 0.5);
                return bias;
            }
            case 'radial': {
                // RADIAL: expansion depuis le centre de territoire
                const fromCenter = Math.sqrt((state.position.x - tc.x) ** 2 + (state.position.z - tc.z) ** 2);
                const cellDist = Math.sqrt((cellX - tc.x) ** 2 + (cellZ - tc.z) ** 2);
                // Forte préférence pour s'éloigner du centre
                return Math.min(15, (cellDist - fromCenter) * 2);
            }
            case 'perimeter': {
                // PÉRIMÈTRE: préférer les cellules proches des bords de zone
                const edgeDist = Math.min(
                    halfZone - Math.abs(cellX),
                    halfZone - Math.abs(cellZ)
                );
                const margin = coa.params.marginDistance || 3;
                if (edgeDist < margin * 2) return 15;   // Très proche du bord
                if (edgeDist < margin * 4) return 5;    // Zone périmétrique
                return -10;                              // Pénaliser l'intérieur
            }
            case 'stigmergy': {
                // STIGMERGIE: Read pheromone value at cell position.
                // Cells with HIGH pheromone (already visited) → strong penalty.
                // Cells with LOW or ZERO pheromone → strong attraction.
                // This creates true stigmergic exploration: drones are repelled from
                // their own pheromone trails and attracted to unexplored areas.
                // Safety: readPheromone may not exist on NoopSwarmIntelligence
                if (typeof this.swarmIntelligence.readPheromone !== 'function') return 0;
                const pherVal = this.swarmIntelligence.readPheromone(cellX, cellZ, 'exploration');
                const gradW = coa.params?.gradientWeight || 8.0;
                // Normalize: at maxIntensity → -gradW, at 0 → +gradW/2
                const maxI = this.swarmIntelligence._config?.maxIntensity || 100;
                const norm = pherVal / maxI; // 0..1
                let bias = -norm * gradW + (1 - norm) * (gradW * 0.5);
                // Also read DANGER pheromone — repel from obstacles
                const dangerVal = this.swarmIntelligence.readPheromone(cellX, cellZ, 'danger');
                if (dangerVal > 0.1) bias -= 15; // strong avoidance of detected obstacles
                return bias;
            }
            default:
                return 0;
        }
    }

    _pickDefaultExplorationWaypoint(state) {
        const prof = state.profile;
        // Use doctrine zone bounds if available, else engine default
        let maxR = Math.min(prof.explorationRadius, this.explorationBounds);
        if (this.doctrineManager) {
            const z = this.doctrineManager.zoneParams;
            maxR = Math.min(maxR, Math.max(z.sizeX, z.sizeZ) / 2);
        }

        const halfZone = this._getHalfZone();
        // Fallback 8: used for sector angle geometry (sensible default = 8 sectors)
        const fleetSize = this.drones.size || 8;

        // ─── 3D Territory assignment (recomputed when fleet changes) ──────
        if (this.useOrganicMode && (!state._territoryCenter || state._lastFleetSize !== fleetSize)) {
            state._lastFleetSize = fleetSize;
            // Extract TRAILING number from drone ID: "x500_09" → "09", "crazyflie_01" → "01"
            const match = String(state.id).match(/(\d+)$/);
            const idx = match ? parseInt(match[1], 10) - 1 : 0;

            // ═══ EMERGENT TERRITORY (stigmergic) ═══
            // Initial angle from index is just a SEED. The drone will re-orient
            // toward unexplored areas based on its LOCAL knowledge.
            // This makes territory assignment EMERGENT, not predetermined.
            state._sectorAngle = (idx / fleetSize) * Math.PI * 2;
            state._sectorWidth = (Math.PI * 2) / fleetSize;
            state._territoryRecalcFrame = 0; // trigger immediate recalc

            // Territory center in XZ plane — moderate push for group overlap
            // 35% of zone radius keeps drones within mutual flocking range (~25m)
            // while still providing territorial spread for coverage.
            // ═══ AUTONOMY MODULATION: territory pull scales linearly to zero ═══
            // At autonomy=100: centralFactor=0 → terR=0 → no territory pull.
            // Pure self-organization relies on initial heading diversity +
            // group heading ramp (8s) to break symmetry, not forced sectors.
            const centralFactor = 1 - this.autonomyLevel / 100;
            const terR = halfZone * 0.35 * centralFactor;
            state._territoryCenter = {
                x: Math.cos(state._sectorAngle) * terR,
                z: Math.sin(state._sectorAngle) * terR
            };

            // ═══ ALTITUDE LAYER ASSIGNMENT ═══
            // Distribute drones across 3 trunk-height altitude bands.
            // All layers stay below canopy (~6m) to avoid flying into leaves.
            // Cycle through layers so adjacent sectors are at different altitudes.
            // All layers stay at trunk height (below canopy ~6m) to avoid leaves
            //   Layer 0 (low):   2.5–3.5m — ground-level inspection
            //   Layer 1 (mid):   3.5–4.5m — main trunk exploration
            //   Layer 2 (high):  4.5–5.5m — upper trunk, still below canopy
            const LAYERS = [
                { min: 2.5, max: 3.5 },  // low
                { min: 3.5, max: 4.5 },  // mid
                { min: 4.5, max: 5.5 },  // high — still below canopy
            ];
            let layerIdx = idx % 3;
            const layer = LAYERS[layerIdx];
            // Within the layer, spread drones at different sub-altitudes
            const subIdx = Math.floor(idx / 3);
            const subCount = Math.max(1, Math.ceil(fleetSize / 3));
            const layerFraction = subCount > 1 ? subIdx / (subCount - 1) : 0.5;
            state._cruiseAlt = layer.min + layerFraction * (layer.max - layer.min);
            state._altLayer = layerIdx;

            console.log(`[TERRITORY] ${state.id}: sector=${(state._sectorAngle * 180 / Math.PI).toFixed(0)}° ter=(${state._territoryCenter.x.toFixed(0)},${state._territoryCenter.z.toFixed(0)}) alt=${state._cruiseAlt.toFixed(1)}m layer=${['low','mid','high'][layerIdx]}`);
        }

        // ═══ STIGMERGIC TERRITORY RECALCULATION ═══
        // Every ~120 frames (~2s), each drone re-orients its preferred sector
        // toward the direction with the MOST unexplored cells in its LOCAL knowledge.
        // This replaces the static index-based assignment with emergent self-organization:
        //   - Each drone gravitates toward the least-explored direction IT knows about
        //   - Two drones with different knowledge (no comm) may pick the same direction
        //     → comm makes them sync knowledge → diverge to different sectors
        //   - This is TRUE stigmergy: individual knowledge drives collective coordination
        if (this.useOrganicMode && state._territoryRecalcFrame !== undefined) {
            state._territoryRecalcFrame = (state._territoryRecalcFrame || 0) + 1;
            if (state._territoryRecalcFrame >= 120) {
                state._territoryRecalcFrame = 0;
                const localCells = this._getDroneLocalCells(state.id);
                const cs = this.cellSize;
                // Sample 8 angular directions and count unexplored cells in each
                const NUM_DIRS = 8;
                const SAMPLE_RADIUS = 15; // cells
                let bestAngle = state._sectorAngle;
                let bestUnexplored = -1;
                for (let d = 0; d < NUM_DIRS; d++) {
                    const angle = (d / NUM_DIRS) * Math.PI * 2;
                    let unexplored = 0;
                    // Sample a wedge of cells in this direction
                    for (let r = 3; r <= SAMPLE_RADIUS; r += 2) {
                        for (let spread = -1; spread <= 1; spread++) {
                            const sampleAngle = angle + spread * 0.3;
                            const gx = Math.round(state.position.x / cs + Math.cos(sampleAngle) * r);
                            const gz = Math.round(state.position.z / cs + Math.sin(sampleAngle) * r);
                            if (!localCells.has(`${gx},${gz}`)) unexplored++;
                        }
                    }
                    if (unexplored > bestUnexplored) {
                        bestUnexplored = unexplored;
                        bestAngle = angle;
                    }
                }
                // Smooth rotation: blend toward best direction (avoid snapping)
                let angleDiff = bestAngle - state._sectorAngle;
                while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
                while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
                state._sectorAngle += angleDiff * 0.3; // 30% per recalc = smooth adaptation
                // Update territory center
                const centralFactor = 1 - this.autonomyLevel / 100;
                const terR = halfZone * 0.35 * centralFactor;
                state._territoryCenter = {
                    x: Math.cos(state._sectorAngle) * terR,
                    z: Math.sin(state._sectorAngle) * terR
                };
            }
        }

        // Use assigned cruise altitude (or profile default for legacy)
        const cruiseAlt = state._cruiseAlt || prof.cruiseAlt;

        // ═══ TWO-PHASE FRONTIER-CRAWL STRATEGY ═══
        // Phase 1: Pick from NEARBY unvisited cells only (≤8m). This guarantees
        //          the drone never flies through visited territory — it "crawls"
        //          the frontier edge, one step at a time.
        // Phase 2: Only if Phase 1 is empty (all nearby cells explored), expand
        //          search to find the CLOSEST reachable frontier anywhere.
        // Forward-motion bias prevents U-turns through already-explored terrain.

        const cs = this.cellSize;
        const curCX = Math.floor(state.position.x / cs);
        const curCZ = Math.floor(state.position.z / cs);
        const maxCellIdx = Math.floor(halfZone / cs);
        const sAngle = state._sectorAngle || 0;
        const sWidth = state._sectorWidth || (Math.PI * 2 / Math.max(1, fleetSize));

        // Precompute forward direction from current velocity
        // Fallback to heading when velocity is too low (e.g. first waypoint pick).
        // heading uses atan2(x,z) convention: 0=+Z, π/2=+X
        // So forward = (sin(heading), cos(heading)) to get (X, Z) components.
        const svLen = Math.sqrt(state.smoothVelocity.x ** 2 + state.smoothVelocity.z ** 2);
        const hasVelocity = svLen > 0.2;
        const fwdX = hasVelocity ? state.smoothVelocity.x / svLen : Math.sin(state.heading);
        const fwdZ = hasVelocity ? state.smoothVelocity.z / svLen : Math.cos(state.heading);

        // Helper: check if a cell is blocked by a tree (altitude-aware)
        // At trunk height (below canopy), only avoid the trunk itself
        // At canopy height, avoid full canopy radius
        const isCellBlocked = (wx, wz) => {
            for (const tree of this.treeBounds) {
                const td = Math.sqrt((wx - tree.center.x) ** 2 + (wz - tree.center.z) ** 2);
                const treeHeight = tree.height || 12;
                const canopyStart = treeHeight * 0.35;
                // Use effective radius based on flight altitude
                let blockRadius;
                if (cruiseAlt < canopyStart) {
                    // Below canopy: only trunk blocks the path
                    blockRadius = Math.max(1.0, tree.radius * 0.3) + 2.0;
                } else {
                    // At canopy level: full canopy + margin
                    blockRadius = tree.radius + 3.0;
                }
                if (td < blockRadius) return true;
            }
            return false;
        };

        // ──── PHASE 1: Nearby frontier cells (within 7 cells = ~14m) ─────
        // ═══ DISTRIBUTED: each drone uses only its LOCAL knowledge ═══
        // Cells it explored itself + cells received via P2P MAP_SYNC.
        // Without comm, drones will re-explore each other's territory.
        const localKnowledge = this._getDroneLocalCells(state.id);

        const NEAR_RADIUS = 7;
        const nearCells = [];
        for (let ring = 1; ring <= NEAR_RADIUS; ring++) {
            for (let dx = -ring; dx <= ring; dx++) {
                for (let dz = -ring; dz <= ring; dz++) {
                    if (Math.abs(dx) !== ring && Math.abs(dz) !== ring) continue;
                    const gx = curCX + dx;
                    const gz = curCZ + dz;
                    if (Math.abs(gx) > maxCellIdx || Math.abs(gz) > maxCellIdx) continue;
                    if (localKnowledge.has(`${gx},${gz}`)) continue;
                    const wx = (gx + 0.5) * cs;
                    const wz = (gz + 0.5) * cs;
                    if (Math.abs(wx) > halfZone - 2 || Math.abs(wz) > halfZone - 2) continue;
                    if (isCellBlocked(wx, wz)) continue;
                    const dist = Math.sqrt((wx - state.position.x) ** 2 + (wz - state.position.z) ** 2);
                    nearCells.push({ x: wx, z: wz, dist, gx, gz });
                }
            }
        }

        // ═══ DOCTRINE-AWARE SCORING WEIGHTS ═══
        // Doctrine adjusts the relative importance of each scoring component.
        // COA adds a directional bias to guide exploration pattern.
        const W = this._getDoctrineWeights();

        // ═══ BEACON FIELD ZONES (from swarm comm) ═══
        // Known beacon positions this drone learned about (own discovery or P2P).
        // Cells NEAR these positions get a scoring bonus — "beacon field reinforcement".
        // This causes drones to converge on beacon-rich areas.
        const beaconZones = this.commManager?.getBeaconZones(state.id) || [];
        const BEACON_ATTRACT_RADIUS = 20; // meters — attraction zone around known beacons
        const BEACON_ATTRACT_WEIGHT = 12; // scoring bonus for cells near known beacons

        if (nearCells.length > 0) {
            // Score nearby cells — all within ~14m so proximity matters less.
            // Components: forward bias, territory, frontier richness, drone repulsion,
            //             backtrack penalty, COA directional bias.
            // ACO-style: collect all scored candidates, then probabilistic selection.
            const commPeers = this._getCommNeighborIds(state.id);
            const nearCandidates = [];
            for (const cell of nearCells) {
                let score = 0;

                // Forward-motion bias (doctrine-weighted)
                // Always active: when velocity is low, fwdX/fwdZ fall back
                // to heading (= sector angle), ensuring each drone's FIRST
                // waypoint pick is biased toward its own sector.
                {
                    const toCellX = cell.x - state.position.x;
                    const toCellZ = cell.z - state.position.z;
                    const dot = (toCellX * fwdX + toCellZ * fwdZ) / cell.dist;
                    score += dot * W.forward;
                }

                // Territory direction (doctrine-weighted)
                const cellAngle = Math.atan2(cell.z, cell.x);
                let angleDiff = cellAngle - sAngle;
                while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
                while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
                if (Math.abs(angleDiff) < sWidth) score += W.territory;
                else if (Math.abs(angleDiff) < sWidth * 2) score += W.territory * 0.33;

                // Frontier richness (doctrine-weighted) — uses per-drone local knowledge
                let neighbors = 0;
                for (let ndx = -1; ndx <= 1; ndx++) {
                    for (let ndz = -1; ndz <= 1; ndz++) {
                        if (ndx === 0 && ndz === 0) continue;
                        if (!localKnowledge.has(`${cell.gx + ndx},${cell.gz + ndz}`)) neighbors++;
                    }
                }
                score += neighbors * W.richness;

                // Drone repulsion (doctrine-weighted)
                // Position repulsion: sensor-based (detect all nearby drones within ~15m)
                // Waypoint repulsion: comm-based (only know peers' waypoints via radio)
                // commPeers is hoisted before the loop (see above)
                for (const [otherId, otherState] of this.drones) {
                    if (otherId === state.id) continue;
                    const odist = Math.sqrt((cell.x - otherState.position.x) ** 2 + (cell.z - otherState.position.z) ** 2);
                    if (odist < 15) score -= W.repulsion * (1 - odist / 15);
                    // Waypoint repulsion: only for comm-range peers (distributed)
                    if (commPeers.has(otherId) && otherState.waypoint) {
                        const owdist = Math.sqrt((cell.x - otherState.waypoint.x) ** 2 + (cell.z - otherState.waypoint.z) ** 2);
                        if (owdist < 10) score -= W.wpRepulsion * (1 - owdist / 10);
                    }
                }

                // Per-drone backtrack penalty (doctrine-weighted)
                if (state._recentCellSet.has(`${cell.gx},${cell.gz}`)) score -= W.backtrack;

                // ═══ INDIVIDUAL EXPLORATION DRIFT (self-organisation noise) ═══
                // Each drone has a slowly-evolving preferred direction.
                // This is the NEGATIVE FEEDBACK (diversity) that balances the
                // positive feedback (group heading alignment). Without this,
                // any alignment mechanism produces pathological consensus.
                // Ref: Vicsek et al. (1995) — noise parameter η
                {
                    const driftX = Math.sin(state._driftAngle || 0);
                    const driftZ = Math.cos(state._driftAngle || 0);
                    const toCellX = cell.x - state.position.x;
                    const toCellZ = cell.z - state.position.z;
                    const dot = (toCellX * driftX + toCellZ * driftZ) / cell.dist;
                    score += dot * 8; // individual noise weight
                }

                // ═══ LOCAL GROUP HEADING BIAS (self-organisation) ═══
                // Prefer cells aligned with the average heading of nearby neighbors.
                // This is the POSITIVE FEEDBACK mechanism of self-organization:
                // small heading correlations amplify → visible collective flow.
                // Ref: Reynolds (1987) + Vicsek et al. (1995)
                // ═══ AUTONOMY MODULATION: group heading scales WITH autonomy ═══
                // At 0% (centralized), drones follow orders → no collective bias.
                // At 100% (autonomous), group heading is the PRIMARY force.
                const groupDir = this._getLocalGroupHeading(state);
                if (groupDir) {
                    const toCellX = cell.x - state.position.x;
                    const toCellZ = cell.z - state.position.z;
                    const dot = (toCellX * groupDir.x + toCellZ * groupDir.z) / cell.dist;
                    // Suppress group heading during initial dispersion (first 8s)
                    // to prevent runaway positive feedback before drones have spread.
                    const timeSinceDispersion = (this._simTime || 0) - (state._dispersionStartTime || 0);
                    const dispersionRamp = Math.min(1, timeSinceDispersion / 8); // 0→1 over 8s
                    const groupHeadingWeight = 14 * (this.autonomyLevel / 100) * dispersionRamp;
                    score += dot * groupHeadingWeight;
                }

                // ═══ COA DIRECTIONAL BIAS ═══
                // Injects pattern-specific preference (grid rows, spiral angle, etc.)
                // ═══ AUTONOMY MODULATION: COA is orchestrator control → scales down ═══
                const coaCentralFactor = 1 - this.autonomyLevel / 100;
                score += this._computeCOABias(cell.x, cell.z, state) * coaCentralFactor;

                // ═══ BEACON FIELD REINFORCEMENT ═══
                // Attract toward areas where beacons were found/reported.
                // Stronger attraction for unexplored cells near beacon clusters.
                if (beaconZones.length > 0) {
                    let beaconBonus = 0;
                    for (const bz of beaconZones) {
                        const bdist = Math.sqrt((cell.x - bz.x) ** 2 + (cell.z - bz.z) ** 2);
                        if (bdist < BEACON_ATTRACT_RADIUS) {
                            // Linear decay: max bonus at beacon, 0 at ATTRACT_RADIUS
                            beaconBonus += BEACON_ATTRACT_WEIGHT * (1 - bdist / BEACON_ATTRACT_RADIUS);
                        }
                    }
                    score += beaconBonus;
                }

                nearCandidates.push({ x: cell.x, z: cell.z, score });
            }
            // ACO-style probabilistic selection (softmax roulette)
            // Like real ants: good paths have higher probability, but any path can be chosen
            const selectedNear = this._selectProbabilistic(nearCandidates);
            if (selectedNear) {
                const terrY = this._groundY(selectedNear.x, selectedNear.z);
                return new THREE.Vector3(selectedNear.x, terrY + cruiseAlt, selectedNear.z);
            }
        }

        // ──── PHASE 2: No nearby frontier — expand ring by ring to find CLOSEST frontier ─────
        // Drone is in the middle of explored territory. Pick the closest reachable
        // frontier cell, with mild sector bias for tie-breaking.
        const fullSearchRadius = Math.ceil(maxR / cs);
        for (let ring = NEAR_RADIUS + 1; ring <= fullSearchRadius; ring++) {
            const ringCandidates = [];
            for (let dx = -ring; dx <= ring; dx++) {
                for (let dz = -ring; dz <= ring; dz++) {
                    if (Math.abs(dx) !== ring && Math.abs(dz) !== ring) continue;
                    const gx = curCX + dx;
                    const gz = curCZ + dz;
                    if (Math.abs(gx) > maxCellIdx || Math.abs(gz) > maxCellIdx) continue;
                    if (localKnowledge.has(`${gx},${gz}`)) continue;
                    const wx = (gx + 0.5) * cs;
                    const wz = (gz + 0.5) * cs;
                    if (Math.abs(wx) > halfZone - 2 || Math.abs(wz) > halfZone - 2) continue;
                    if (isCellBlocked(wx, wz)) continue;

                    // Score: sector bonus + forward bias + COA bias (lighter weights for Phase 2)
                    let score = 0;
                    const cellAngle = Math.atan2(wz, wx);
                    let angleDiff = cellAngle - sAngle;
                    while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
                    while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
                    if (Math.abs(angleDiff) < sWidth * 1.5) score += W.territory * 0.67;

                    // Forward bias (always active — heading fallback at low velocity)
                    {
                        const toCellX = wx - state.position.x;
                        const toCellZ = wz - state.position.z;
                        const tcd = Math.sqrt(toCellX * toCellX + toCellZ * toCellZ);
                        if (tcd > 0.1) score += (toCellX * fwdX + toCellZ * fwdZ) / tcd * 5;
                    }

                    // Drone repulsion (lighter for Phase 2 — sensor-based position detection only).
                    // No waypoint repulsion here: Phase 2 is proximity-driven, drones are close
                    // enough (~14m) to see each other physically — no radio needed.
                    for (const [otherId, otherState] of this.drones) {
                        if (otherId === state.id) continue;
                        const odist = Math.sqrt((wx - otherState.position.x) ** 2 + (wz - otherState.position.z) ** 2);
                        if (odist < 15) score -= W.repulsion * 0.5 * (1 - odist / 15);
                    }

                    // Individual drift (Phase 2 — half weight)
                    {
                        const driftX = Math.sin(state._driftAngle || 0);
                        const driftZ = Math.cos(state._driftAngle || 0);
                        const toCX = wx - state.position.x;
                        const toCZ = wz - state.position.z;
                        const tcd = Math.sqrt(toCX * toCX + toCZ * toCZ);
                        if (tcd > 0.1) score += (toCX * driftX + toCZ * driftZ) / tcd * 4;
                    }

                    // Group heading bias (Phase 2 — half strength)
                    // ═══ AUTONOMY MODULATION: scales with autonomy level ═══
                    const groupDir2 = this._getLocalGroupHeading(state);
                    if (groupDir2) {
                        const toCX = wx - state.position.x;
                        const toCZ = wz - state.position.z;
                        const tcd2 = Math.sqrt(toCX * toCX + toCZ * toCZ);
                        const groupHeadingWeight2 = 11 * (this.autonomyLevel / 100);
                        if (tcd2 > 0.1) score += (toCX * groupDir2.x + toCZ * groupDir2.z) / tcd2 * groupHeadingWeight2;
                    }

                    // COA bias (half strength for Phase 2 — proximity is primary concern)
                    // ═══ AUTONOMY MODULATION: COA → 0 at full autonomy ═══
                    const coaCentralFactor2 = 1 - this.autonomyLevel / 100;
                    score += this._computeCOABias(wx, wz, state) * 0.5 * coaCentralFactor2;

                    // ═══ BEACON FIELD REINFORCEMENT (Phase 2) ═══
                    // Even stronger in Phase 2 — when all nearby cells are explored,
                    // beacons become the primary target for "search reinforcement".
                    if (beaconZones.length > 0) {
                        let beaconBonus = 0;
                        for (const bz of beaconZones) {
                            const bdist = Math.sqrt((wx - bz.x) ** 2 + (wz - bz.z) ** 2);
                            if (bdist < BEACON_ATTRACT_RADIUS * 1.5) {
                                beaconBonus += BEACON_ATTRACT_WEIGHT * 1.5 * (1 - bdist / (BEACON_ATTRACT_RADIUS * 1.5));
                            }
                        }
                        score += beaconBonus;
                    }

                    ringCandidates.push({ x: wx, z: wz, score });
                }
            }
            // Phase 2: higher temperature (0.5) for more exploration when traversing explored territory
            if (ringCandidates.length > 0) {
                const selectedRing = this._selectProbabilistic(ringCandidates, 0.5);
                if (selectedRing) {
                    const terrY = this._groundY(selectedRing.x, selectedRing.z);
                    return new THREE.Vector3(selectedRing.x, terrY + cruiseAlt, selectedRing.z);
                }
            }
        }

        // Exploration complete — no unvisited cells found anywhere.
        // ═══ BEACON FIELD SEARCH: Before returning home, check for known beacons ═══
        // If there are unfound beacon zones that this drone knows about, fly toward
        // the nearest one for detection instead of going home. This ensures drones
        // don't ignore beacons just because cells are explored.
        if (beaconZones.length > 0) {
            const beaconSystem = window.DIAMANTS?.llmChatPanel?.beaconSystem;
            if (beaconSystem) {
                // Filter to unfound beacons near known zones
                const unfoundBeacons = [...beaconSystem.beacons.values()].filter(b => !b.found);
                if (unfoundBeacons.length > 0) {
                    // Find closest unfound beacon to this drone
                    let closestDist = Infinity, closestBeacon = null;
                    for (const b of unfoundBeacons) {
                        const d = Math.sqrt((b.position.x - state.position.x) ** 2 + (b.position.z - state.position.z) ** 2);
                        if (d < closestDist) { closestDist = d; closestBeacon = b; }
                    }
                    if (closestBeacon && closestDist > 3) {
                        // Go investigate this beacon (fly within detection range)
                        state._lastAction = 'BEACON SEARCH';
                        state._lastDecision = `🎯 →beacon ${closestBeacon.id} @${closestDist.toFixed(0)}m`;
                        const terrY = this._groundY(closestBeacon.position.x, closestBeacon.position.z);
                        console.log(`[EXPLORE] ${state.id} — no frontier, but ${unfoundBeacons.length} unfound beacons → heading to ${closestBeacon.id} @${closestDist.toFixed(0)}m`);
                        return new THREE.Vector3(closestBeacon.position.x, terrY + cruiseAlt, closestBeacon.position.z);
                    }
                }
            }
        }

        // Also check globally: if beacons exist but we don't know about them,
        // pick a random exploration waypoint toward zones with the most beacons
        const beaconSystem2 = window.DIAMANTS?.llmChatPanel?.beaconSystem;
        if (beaconSystem2) {
            const unfound2 = [...beaconSystem2.beacons.values()].filter(b => !b.found);
            if (unfound2.length > 0) {
                // Cycle through unfound beacons per drone to avoid all going to the same one
                const idx = (state._droneIdx || 0) % unfound2.length;
                const target = unfound2[idx];
                const td = Math.sqrt((target.position.x - state.position.x) ** 2 + (target.position.z - state.position.z) ** 2);
                if (td > 3) {
                    state._lastAction = 'BEACON SCAN';
                    state._lastDecision = `🎯 scan ${target.id}`;
                    const terrY = this._groundY(target.position.x, target.position.z);
                    console.log(`[EXPLORE] ${state.id} — no frontier, scanning unfound beacon ${target.id} (drone idx redirect)`);
                    return new THREE.Vector3(target.position.x, terrY + cruiseAlt, target.position.z);
                }
            }
        }

        // Truly done: all explored AND all beacons found (or no beacons).
        // Mark drone as returning to its original spawn slot for landing.
        state._returningToBase = true;
        const home = state.homePosition;
        const fbTerrain = this._groundY(home.x, home.z);
        console.log(`[EXPLORE] ${state.id} — no frontier left, returning to home slot (${home.x.toFixed(1)}, ${home.z.toFixed(1)})`);
        return new THREE.Vector3(home.x, fbTerrain + cruiseAlt, home.z);
    }

    // ═══════════════════════════════════════════════════════════════════
    //  ACO-STYLE PROBABILISTIC SELECTION (Softmax Roulette Wheel)
    //  Inspired by Dorigo's ant colony edge-selection formula:
    //    p_xy = (τ_xy^α · η_xy^β) / Σ(τ_xz^α · η_xz^β)
    //  Instead of always picking the best-scored candidate, we convert
    //  scores to probabilities via softmax and sample. This preserves
    //  exploration diversity: even low-scored paths have a small chance,
    //  preventing premature convergence — exactly like real ants.
    //  Temperature τ controls exploitation vs exploration:
    //    τ → 0: greedy (always best)  |  τ → ∞: uniform random
    // ═══════════════════════════════════════════════════════════════════
    _selectProbabilistic(candidates, temperature = 0.3) {
        if (!candidates || candidates.length === 0) return null;
        if (candidates.length === 1) return candidates[0];

        // Shift scores so max = 0 (numerical stability for exp)
        let maxScore = -Infinity;
        for (const c of candidates) {
            if (c.score > maxScore) maxScore = c.score;
        }

        // Compute softmax weights: w_i = exp((score_i - maxScore) / temperature)
        const weights = [];
        let totalWeight = 0;
        for (const c of candidates) {
            const w = Math.exp((c.score - maxScore) / temperature);
            weights.push(w);
            totalWeight += w;
        }

        // Roulette-wheel selection
        let r = Math.random() * totalWeight;
        for (let i = 0; i < candidates.length; i++) {
            r -= weights[i];
            if (r <= 0) return candidates[i];
        }
        // Floating-point edge case — return last
        return candidates[candidates.length - 1];
    }

    /**
     * Escape waypoint — used when a drone is stalled (stuck between forces).
     * Picks a direction AWAY from the centroid of nearby drones,
     * targeting an unvisited area. Ensures the drone breaks free.
     */
    _pickEscapeWaypoint(state) {
        const prof = state.profile;
        const halfZone = this._getHalfZone();
        const cruiseAlt = state._cruiseAlt || prof.cruiseAlt;

        // Compute centroid of nearby drones (sensor-based: detect all within 20m,
        // regardless of comm range — this is physical proximity escape, not radio)
        let cxSum = 0, czSum = 0, nCount = 0;
        for (const [otherId, otherState] of this.drones) {
            if (otherId === state.id) continue;
            const dx = otherState.position.x - state.position.x;
            const dz = otherState.position.z - state.position.z;
            const dist = Math.sqrt(dx * dx + dz * dz);
            if (dist < 20) {
                cxSum += otherState.position.x;
                czSum += otherState.position.z;
                nCount++;
            }
        }

        // Try to find nearest unvisited cell direction (frontier-seeking escape)
        // ═══ DISTRIBUTED: uses per-drone local knowledge ═══
        const localKnowledge = this._getDroneLocalCells(state.id);
        let frontierAngle = null;
        let bestFrontierDist = Infinity;
        const searchRadius = 30;
        const cs = this.cellSize;
        const pcx = Math.floor(state.position.x / cs);
        const pcz = Math.floor(state.position.z / cs);
        for (let sdx = -Math.ceil(searchRadius / cs); sdx <= Math.ceil(searchRadius / cs); sdx += 2) {
            for (let sdz = -Math.ceil(searchRadius / cs); sdz <= Math.ceil(searchRadius / cs); sdz += 2) {
                const fKey = `${pcx + sdx},${pcz + sdz}`;
                if (!localKnowledge.has(fKey)) {
                    const fwx = (pcx + sdx + 0.5) * cs;
                    const fwz = (pcz + sdz + 0.5) * cs;
                    if (Math.abs(fwx) < halfZone - 3 && Math.abs(fwz) < halfZone - 3) {
                        const fd = Math.sqrt((fwx - state.position.x) ** 2 + (fwz - state.position.z) ** 2);
                        if (fd < bestFrontierDist && fd > 5) {
                            bestFrontierDist = fd;
                            frontierAngle = Math.atan2(fwz - state.position.z, fwx - state.position.x);
                        }
                    }
                }
            }
        }

        let escapeAngle;
        if (frontierAngle !== null) {
            // Head toward nearest unvisited frontier cell
            escapeAngle = frontierAngle + (Math.random() - 0.5) * 0.4;
        } else if (nCount > 0) {
            // No frontier found — flee AWAY from centroid of nearby drones
            const centX = cxSum / nCount;
            const centZ = czSum / nCount;
            escapeAngle = Math.atan2(state.position.z - centZ, state.position.x - centX);
        } else {
            // No nearby drones — pick random direction
            escapeAngle = Math.random() * Math.PI * 2;
        }

        // Add some randomness to avoid repeated escape in same direction
        escapeAngle += (Math.random() - 0.5) * 0.6;

        // Progressive escape distance: repeated stalls → bigger jumps (12-40m)
        const escaleCount = state._escaleCount || 1;
        const baseDist = 12 + Math.min(escaleCount * 5, 20); // 12m → 32m
        const dist = baseDist + Math.random() * 10;
        let wx = state.position.x + Math.cos(escapeAngle) * dist;
        let wz = state.position.z + Math.sin(escapeAngle) * dist;

        // Clamp to arena
        const clampX = Math.max(-(halfZone - 3), Math.min(halfZone - 3, wx));
        const clampZ = Math.max(-(halfZone - 3), Math.min(halfZone - 3, wz));

        // Tree collision check — avoid escaping directly into a tree
        for (const tree of this.treeBounds) {
            const tdx = clampX - tree.center.x;
            const tdz = clampZ - tree.center.z;
            const tDist = Math.sqrt(tdx * tdx + tdz * tdz);
            if (tDist < tree.radius + 6) {
                // Push waypoint away from tree with generous clearance
                if (tDist > 0.01) {
                    const pushDist = tree.radius + 7 - tDist;
                    wx = clampX + (tdx / tDist) * pushDist;
                    wz = clampZ + (tdz / tDist) * pushDist;
                }
            }
        }
        // Re-clamp after tree push
        const safeX = Math.max(-(halfZone - 3), Math.min(halfZone - 3, wx));
        const safeZ = Math.max(-(halfZone - 3), Math.min(halfZone - 3, wz));

        const terrainY = this._groundY(safeX, safeZ);
        const wy = Math.max(terrainY + this.minAltitudeAboveGround, terrainY + cruiseAlt);

        console.log(`[ESCAPE] ${state.id} stalled → escape angle=${(escapeAngle * 180 / Math.PI).toFixed(0)}° dist=${dist.toFixed(0)}m (${nCount} nearby)`);
        return new THREE.Vector3(safeX, wy, safeZ);
    }

    /**
     * NOUVEAU: Configurer la fonction de hauteur terrain
     * @param {Function} heightFn - (x, z) => y (hauteur du terrain)
     */
    setTerrainHeightFunction(heightFn) {
        this.getTerrainHeight = heightFn;
        // Forward to swarm intelligence if it supports terrain
        if (this.swarmIntelligence && typeof this.swarmIntelligence.setTerrainHeightFunction === 'function') {
            this.swarmIntelligence.setTerrainHeightFunction(heightFn);
        }
        console.log('[ENGINE] 🌳 Terrain height function configurée');
    }

    /**
     * Soft tree collision — pushes drone out gradually over frames
     * instead of teleporting. Canopy-aware: wider exclusion at canopy height.
     */
    _enforceTreeCollision(state, dt) {
        // Choose the correct velocity vector: X500 uses state.velocity,
        // kinematic drones use state.smoothVelocity
        const isX500 = !!state.x500Dynamics;
        const vel = isX500 ? state.velocity : state.smoothVelocity;

        for (const tree of this.treeBounds) {
            const dx = state.position.x - tree.center.x;
            const dz = state.position.z - tree.center.z;
            const dist2D = Math.sqrt(dx * dx + dz * dz);
            
            const treeHeight = tree.height || 12;
            // Canopy-aware radius: wider at canopy level (top 60%), narrower at trunk
            const trunkRadius = tree.radius * 0.4;
            const canopyBottom = treeHeight * 0.3;
            let effectiveRadius;
            if (state.position.y > canopyBottom) {
                const canopyFactor = Math.min(1.0, (state.position.y - canopyBottom) / (treeHeight * 0.3));
                effectiveRadius = trunkRadius + (tree.radius - trunkRadius) * canopyFactor;
            } else {
                effectiveRadius = trunkRadius;
            }
            
            // Safety margin — moderate at trunk height
            const safetyMargin = isX500 ? 3.0 : 2.0;
            const collisionRadius = effectiveRadius + safetyMargin;
            
            if (dist2D < collisionRadius && state.position.y < treeHeight + 2) {
                if (dist2D > 0.01) {
                    const penetration = collisionRadius - dist2D;
                    // Push strength proportional to penetration
                    const basePush = isX500 ? 8.0 : 6.0;
                    const maxPush = isX500 ? 20.0 : 15.0;
                    const pushStrength = Math.min(penetration * basePush, maxPush);
                    const nx = dx / dist2D;
                    const nz = dz / dist2D;
                    // Push via the CORRECT velocity vector
                    vel.x += nx * pushStrength * dt * 10;
                    vel.z += nz * pushStrength * dt * 10;
                    // Hard position correction if inside tree at all
                    if (penetration > 0.5) {
                        const correctionScale = isX500 ? 0.8 : 0.5;
                        state.position.x += nx * penetration * correctionScale;
                        state.position.z += nz * penetration * correctionScale;
                    }
                    // If inside canopy, push UP above tree (don't push down into trunk)
                    // BUT: skip vertical UP push when drone is ALREADY above its target altitude.
                    const aboveTarget = state.waypoint && (state.position.y > state.waypoint.y + 1.0);
                    if (state.position.y > canopyBottom && state.position.y < treeHeight && !aboveTarget) {
                        vel.y += 3.0 * dt * 10; // push UP over canopy
                    }
                    // Dampen velocity toward tree
                    const dotVel = (vel.x * (-nx) + vel.z * (-nz));
                    if (dotVel > 0) {
                        vel.x += nx * dotVel * 0.8;
                        vel.z += nz * dotVel * 0.8;
                    }
                } else {
                    vel.x += 5.0;
                }
            }
        }
    }

    // ─── Inter-drone hard collision (unconditional safety net for ALL phases) ────
    // Like _enforceTreeCollision, this runs AFTER position integration to guarantee
    // no two drones physically overlap, regardless of avoidance forces or phase gating.
    _enforceDroneCollision(state, dt) {
        const isX500 = !!state.x500Dynamics;
        const vel = isX500 ? state.velocity : state.smoothVelocity;
        const isLanding = state.phase === 'LAND' || state.phase === 'LANDED';

        this._rebuildSpatialHash();
        const cs = this._spatialCellSize;
        const gx = Math.floor(state.position.x / cs);
        const gz = Math.floor(state.position.z / cs);

        for (let ddx = -1; ddx <= 1; ddx++) {
            for (let ddz = -1; ddz <= 1; ddz++) {
                const bucket = this._spatialHash.get(`${gx + ddx},${gz + ddz}`);
                if (!bucket) continue;
                for (const other of bucket) {
                    if (other.id === state.id) continue;
                    const dx = state.position.x - other.position.x;
                    const dz = state.position.z - other.position.z;
                    const dy = state.position.y - other.position.y;
                    const dist2D = Math.sqrt(dx * dx + dz * dz);
                    const dist3D = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    const otherIsX500 = !!other.x500Dynamics;
                    const otherLanding = other.phase === 'LAND' || other.phase === 'LANDED';

                    // During landing: use TRUE 3D distance and smaller separation
                    // A Crazyflie at 0.2m and X500 at 3m are NOT colliding
                    // In flight: use 2D distance + altitude band (original logic)
                    let pairMinSep;
                    let effectiveDist;
                    if (isLanding || otherLanding) {
                        // Landing: physical body size only, 3D distance
                        pairMinSep = (isX500 && otherIsX500) ? 3.0 : (isX500 || otherIsX500) ? 1.8 : 0.8;
                        effectiveDist = dist3D;
                    } else {
                        // In flight: wider 2D separation for safety
                        pairMinSep = (isX500 || otherIsX500) ? 3.5 : 1.5;
                        effectiveDist = (Math.abs(dy) < 3.0) ? dist2D : Infinity;
                    }

                    if (effectiveDist < pairMinSep) {
                        if (dist2D > 0.01) {
                            const penetration = pairMinSep - effectiveDist;
                            const nx = dx / dist2D;
                            const nz = dz / dist2D;
                            // Reduced correction during landing to not fight PID
                            const corrScale = isLanding ? 0.25 : 0.5;
                            state.position.x += nx * penetration * corrScale;
                            state.position.z += nz * penetration * corrScale;
                            // Velocity push — gentler during landing
                            const pushMul = isLanding ? 3.0 : 6.0;
                            const pushStr = Math.min(penetration * pushMul, isLanding ? 5.0 : 15.0);
                            vel.x += nx * pushStr * dt * 10;
                            vel.z += nz * pushStr * dt * 10;
                            // Dampen velocity toward the other drone
                            const dotVel = vel.x * (-nx) + vel.z * (-nz);
                            if (dotVel > 0) {
                                vel.x += nx * dotVel * 0.5;
                                vel.z += nz * dotVel * 0.5;
                            }
                        } else {
                            // Exact overlap — push based on ID ordering
                            const dir = state.id > other.id ? 1 : -1;
                            state.position.x += dir * pairMinSep * 0.3;
                            vel.x += dir * 2.0;
                        }
                    }
                }
            }
        }
    }

    // ─── Tree collision avoidance (soft force + sensor-based) ────────────────────
    _computeTreeAvoidance(state) {
        const force = new THREE.Vector3();
        // X500 is larger but at trunk height margins can be moderate
        const isX500 = !!state.x500Dynamics;
        const extraMargin = isX500 ? 3.0 : 0.5;
        
        // 1. Force basée sur positions connues des arbres
        for (const tree of this.treeBounds) {
            const dx = state.position.x - tree.center.x;
            const dz = state.position.z - tree.center.z;
            const dist = Math.sqrt(dx * dx + dz * dz);

            // ── CANOPY-AWARE avoidance ──
            // Tree has trunk + canopy. If drone is BELOW tree height,
            // it must avoid the full canopy radius. If above, no avoidance needed.
            const treeHeight = tree.height || 12;
            const treeTopY = tree.center.y + treeHeight;
            const trunkBaseY = tree.center.y;

            // If drone is above the tree top, skip this tree entirely
            if (state.position.y > treeTopY + 1.0) continue;

            // Canopy starts at ~40% of tree height (realistic for deciduous trees)
            const canopyStartY = trunkBaseY + treeHeight * 0.35;
            // Below canopy: avoid trunk only (smaller radius ≈ 30% of tree bounding)
            // At canopy level: avoid full canopy radius
            // Transition between these zones
            let effectiveRadius;
            if (state.position.y < canopyStartY) {
                // Below canopy — trunk avoidance only
                effectiveRadius = Math.max(1.0, tree.radius * 0.3);
            } else {
                // At/in canopy — full canopy radius with expansion
                const canopyFraction = Math.min(1, (state.position.y - canopyStartY) / (treeHeight * 0.3));
                effectiveRadius = tree.radius * (0.3 + 0.7 * canopyFraction);
            }

            const treeAvoidDist = this.safetyDistanceOverride ?? AVOID.treeDistance;
            const minDist = effectiveRadius + treeAvoidDist + extraMargin;

            if (dist < minDist && dist > 0.01) {
                // Smoothstep — gradual onset, no spike at threshold
                const t = 1 - dist / minDist;
                const tSmooth = t * t * (3 - 2 * t);
                const forceScale = isX500 ? AVOID.treeForceScale * 1.5 : AVOID.treeForceScale;
                const strength = forceScale * tSmooth;

                // Horizontal push away from trunk/canopy center
                force.x += (dx / dist) * strength;
                force.z += (dz / dist) * strength;

                // Vertical push: if drone is IN the canopy, push it UP
                // BUT: skip vertical UP push when drone is ALREADY above its target.
                // Otherwise tree avoidance fights the PID descent and traps drones at high altitudes.
                const aboveTarget = state.waypoint && (state.position.y > state.waypoint.y + 1.0);
                if (!aboveTarget) {
                    if (state.position.y > canopyStartY && state.position.y < treeTopY) {
                        // Push UP strongly to clear canopy (don't let drone sit in leaves)
                        const canopyDepth = (state.position.y - canopyStartY) / (treeTopY - canopyStartY);
                        const vertPush = AVOID.treeVerticalPush * tSmooth * (canopyDepth > 0.5 ? 3.0 : 2.0);
                        force.y += vertPush;
                    } else if (dist < effectiveRadius + 2) {
                        force.y += AVOID.treeVerticalPush * tSmooth * (isX500 ? 2.0 : 1.0);
                    }
                }
            }
        }
        
        // 2. Force basée sur capteurs Multi-Ranger (réaliste)
        if (this.useSensorAvoidance && this.multiRangerManager && this.scene) {
            const sensorForce = this._computeSensorAvoidance(state);
            // Combiner forces: les capteurs ont priorité pour évitement proche
            force.add(sensorForce);
        }
        
        return force;
    }
    
    // ─── Sensor-based avoidance (Multi-Ranger ToF simulation) ────────────────
    _computeSensorAvoidance(state) {
        const force = new THREE.Vector3();
        
        if (!this.multiRangerManager) return force;
        
        // Obtenir ou créer le capteur pour ce drone
        let sensor = this.multiRangerManager.getSensor(state.id);
        if (!sensor) {
            sensor = this.multiRangerManager.createSensor(state.id);
        }
        
        // Effectuer un scan avec position et heading actuels
        const scanResult = sensor.scan(state.position, state.heading);
        
        // Utiliser le vecteur d'évitement calculé par le capteur
        const avoidanceVector = sensor.getAvoidanceVector();
        
        // Scalaire pour force basée sur proximité critique
        const minRange = Math.min(...scanResult.ranges.slice(0, 4)); // Horizontal only
        
        if (minRange < MULTI_RANGER_CONFIG.maxRange) {
            // Plus l'obstacle est proche, plus la force est forte
            const proximity = 1 - (minRange / MULTI_RANGER_CONFIG.maxRange);
            const multiplier = AVOID.treeForceScale * Math.pow(proximity, 1.5) * 3;
            
            force.x = avoidanceVector.x * multiplier;
            force.z = avoidanceVector.z * multiplier;
            
            // Si obstacle proche (< 1.5m), ajouter composante verticale (plonger sous canopée)
            if (minRange < 1.5) {
                force.y -= AVOID.treeVerticalPush * (1.5 - minRange) * 1.5;
            }
        }
        
        // Mettre à jour les données de capteur dans l'état du drone (pour UI/debug)
        state.sensorRanges = scanResult.ranges;
        state.sensorAlerts = scanResult.alerts;
        
        return force;
    }

    // ─── Inter-drone collision avoidance (3D, spatial-hashed O(N)) ──
    _rebuildSpatialHash() {
        if (this._spatialHashFrame === this._diagFrame) return; // once per frame
        this._spatialHashFrame = this._diagFrame;
        this._spatialHash.clear();
        const cs = this._spatialCellSize;
        for (const [id, st] of this.drones) {
            const gx = Math.floor(st.position.x / cs);
            const gz = Math.floor(st.position.z / cs);
            const key = `${gx},${gz}`;
            let bucket = this._spatialHash.get(key);
            if (!bucket) { bucket = []; this._spatialHash.set(key, bucket); }
            bucket.push(st);
        }
    }

    _computeDroneAvoidance(state) {
        const force = new THREE.Vector3();
        // Scale avoidance by autonomy: at 100% autonomy, only avoid very close collisions
        const autonomyFactor = 1 - (this.autonomyLevel / 100) * 0.4; // 100% → 0.6x radius
        // X500 drones are much larger (scale 10 ≈ 5m wingspan) — need larger avoidance radius
        const isX500 = !!state.x500Dynamics;
        const baseDroneDist = this.safetyDistanceOverride ?? AVOID.droneDistance;
        const baseAvoidDist = (isX500 ? baseDroneDist + 5.0 : baseDroneDist + 1.0) * autonomyFactor;
        const verticalAvoidDist = isX500 ? 6.0 : 2.0;
        const maxForcePerDrone = isX500 ? 5.0 : 2.5;

        // Build spatial hash once per frame (O(N) total, amortized)
        this._rebuildSpatialHash();

        // Only check neighboring cells in spatial hash (O(1) per drone)
        const cs = this._spatialCellSize;
        const gx = Math.floor(state.position.x / cs);
        const gz = Math.floor(state.position.z / cs);

        for (let ddx = -1; ddx <= 1; ddx++) {
            for (let ddz = -1; ddz <= 1; ddz++) {
                const bucket = this._spatialHash.get(`${gx + ddx},${gz + ddz}`);
                if (!bucket) continue;
                for (const other of bucket) {
                    if (other.id === state.id) continue;
                    const dx = state.position.x - other.position.x;
                    const dy = state.position.y - other.position.y;
                    const dz = state.position.z - other.position.z;
                    const dist2D = Math.sqrt(dx * dx + dz * dz);

                    // ═══ Horizontal avoidance: RADIAL repulsion + lateral deflection ═══
                    const otherIsX500 = !!other.x500Dynamics;
                    const avoidDist = (isX500 || otherIsX500) ? baseAvoidDist : (baseDroneDist + 1.5);
                    if (dist2D < avoidDist && dist2D > 0.01) {
                        const t = 1 - dist2D / avoidDist;
                        const tSmooth = t * t * (3 - 2 * t); // smoothstep
                        const strength = Math.min(AVOID.droneForceScale * tSmooth * 1.5, maxForcePerDrone);

                        const nx = dx / dist2D;
                        const nz = dz / dist2D;
                        // DIRECT radial repulsion (push AWAY from other drone)
                        force.x += nx * strength * 0.7;
                        force.z += nz * strength * 0.7;
                        // Lateral deflection (pass to the side)
                        const side = state.id < other.id ? 1 : -1;
                        const perpX = -nz * side;
                        const perpZ = nx * side;
                        force.x += perpX * strength * 0.5;
                        force.z += perpZ * strength * 0.5;
                    }

                    // ═══ Vertical separation ═══
                    if (dist2D < baseAvoidDist && Math.abs(dy) < verticalAvoidDist) {
                        const vertDir = state.id > other.id ? 1 : -1;
                        const vt = 1 - Math.abs(dy) / verticalAvoidDist;
                        const vertStrength = 1.5 * vt * vt;
                        force.y += vertDir * vertStrength;
                    }
                }
            }
        }

        // Total cap (3D) — stronger for X500 to prevent penetration
        const fMag = Math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z);
        const maxTotalForce = isX500 ? 12.0 : 7.0;
        if (fMag > maxTotalForce) {
            const s = maxTotalForce / fMag;
            force.x *= s;
            force.y *= s;
            force.z *= s;
        }
        return force;
    }

    // ─── Local group heading (for waypoint scoring) ─────────────────
    // Returns the average normalized velocity of neighbors within 25m,
    // or null if no moving neighbors found. Cached once per frame.
    // This drives waypoint selection toward group consensus direction.
    // ═══ DISTRIBUTED: only considers drones within comm range ═══
    // A drone can only know the heading of peers it receives radio data from.
    // Physical proximity (spatial hash) + comm-range filter = realistic sensing.
    // ═══ MIN_ALIGN_DIST gate: same as Reynolds flocking ═══
    // Close drones (< 12m) don't contribute to group heading.
    // On helipad, this returns null → no consensus lock.
    _getLocalGroupHeading(state) {
        // Cache per frame per drone (Map-based for multi-drone efficiency)
        if (!this._groupHeadingCache || this._groupHeadingCache.frame !== this._diagFrame) {
            this._groupHeadingCache = { frame: this._diagFrame, entries: new Map() };
        }
        if (this._groupHeadingCache.entries.has(state.id)) {
            return this._groupHeadingCache.entries.get(state.id);
        }

        const RADIUS = 25;
        const MIN_ALIGN_DIST = 12; // same as flocking — no alignment with close drones
        // ═══ DISTRIBUTED: only align with drones within comm range ═══
        const commNeighbors = this._getCommNeighborIds(state.id);
        this._rebuildSpatialHash();
        const cs = this._spatialCellSize;
        const gx = Math.floor(state.position.x / cs);
        const gz = Math.floor(state.position.z / cs);
        const sr = Math.ceil(RADIUS / cs);

        let sumVx = 0, sumVz = 0, count = 0;
        for (let ddx = -sr; ddx <= sr; ddx++) {
            for (let ddz = -sr; ddz <= sr; ddz++) {
                const bucket = this._spatialHash.get(`${gx + ddx},${gz + ddz}`);
                if (!bucket) continue;
                for (const other of bucket) {
                    if (other.id === state.id) continue;
                    if (!commNeighbors.has(other.id)) continue; // DISTRIBUTED: must be in comm range
                    if (other.phase !== 'EXPLORE') continue;
                    const dx = other.position.x - state.position.x;
                    const dz = other.position.z - state.position.z;
                    const distSq = dx * dx + dz * dz;
                    if (distSq > RADIUS * RADIUS) continue;
                    if (distSq < MIN_ALIGN_DIST * MIN_ALIGN_DIST) continue; // too close → separate, don't align
                    const ovx = other.smoothVelocity?.x || 0;
                    const ovz = other.smoothVelocity?.z || 0;
                    const ospd = Math.sqrt(ovx * ovx + ovz * ovz);
                    if (ospd < 0.3) continue; // skip stationary
                    sumVx += ovx / ospd; // unit vector
                    sumVz += ovz / ospd;
                    count++;
                }
            }
        }

        let result = null;
        if (count > 0) {
            const len = Math.sqrt(sumVx * sumVx + sumVz * sumVz);
            if (len > 0.1) result = { x: sumVx / len, z: sumVz / len };
        }
        this._groupHeadingCache.entries.set(state.id, result);
        return result;
    }

    // ─── Reynolds flocking forces (alignment + cohesion) ────────────
    // Real inter-agent influence: each drone adjusts its heading toward
    // neighbors' average heading (alignment) and toward their center of
    // mass (cohesion). This produces the visible "flocking" behavior
    // that Transfer Entropy can measure as genuine emergence.
    //
    // ═══ DISTRIBUTED: only flock with comm-range peers ═══
    // A drone can only align/attract toward drones it hears via radio.
    // Physical avoidance (_computeDroneAvoidance) remains sensor-based (global).
    // This separation means:
    //   - Without comm: drones don't flock → independent exploration
    //   - With short-range comm (BLE 10m): tight micro-flocks form
    //   - With long-range comm (WiFi 30m): wider coordination
    //
    // Forces scale with drone count: stronger per-neighbor force when
    // few drones (each neighbor matters more), weaker when many (avoid
    // over-cohesion into a single clump).
    _computeFlockingForce(state) {
        const FLOCK_RADIUS = 30;        // meters — wider perception range
        const ALIGN_WEIGHT = 0.8;       // strong heading matching
        const COHESION_WEIGHT = 0.4;    // moderate center-of-mass attraction
        const MAX_FLOCK_FORCE = 2.5;    // m/s² — competes meaningfully with PID
        // ═══ MINIMUM ALIGNMENT DISTANCE ═══
        // Reynolds (1987): separate close, align medium, attract far.
        // Drones closer than MIN_ALIGN_DIST only contribute to cohesion,
        // NOT to alignment. This prevents consensus lock on the helipad
        // where all drones are packed within ~5m: they separate and choose
        // different directions, then alignment kicks in once they've spread.
        const MIN_ALIGN_DIST = 12;      // meters — only align beyond this

        // ═══ DISTRIBUTED: only flock with drones within comm range ═══
        const commNeighbors = this._getCommNeighborIds(state.id);

        // Reuse spatial hash (already rebuilt this frame)
        this._rebuildSpatialHash();
        const cs = this._spatialCellSize;
        const gx = Math.floor(state.position.x / cs);
        const gz = Math.floor(state.position.z / cs);
        const searchRadius = Math.ceil(FLOCK_RADIUS / cs);

        let avgVx = 0, avgVz = 0;       // alignment: average neighbor velocity
        let alignCount = 0;              // only medium+ distance neighbors
        let comX = 0, comZ = 0;          // cohesion: center of mass
        let neighborCount = 0;

        for (let ddx = -searchRadius; ddx <= searchRadius; ddx++) {
            for (let ddz = -searchRadius; ddz <= searchRadius; ddz++) {
                const bucket = this._spatialHash.get(`${gx + ddx},${gz + ddz}`);
                if (!bucket) continue;
                for (const other of bucket) {
                    if (other.id === state.id) continue;
                    if (!commNeighbors.has(other.id)) continue; // DISTRIBUTED: must be in comm range
                    if (other.phase !== 'EXPLORE' && other.phase !== 'HOVER') continue;

                    const dx = other.position.x - state.position.x;
                    const dz = other.position.z - state.position.z;
                    const dist = Math.sqrt(dx * dx + dz * dz);
                    if (dist > FLOCK_RADIUS || dist < 0.1) continue;

                    // Weight by proximity (closer neighbors have more influence)
                    const w = 1 - dist / FLOCK_RADIUS; // linear falloff [1→0]

                    // Alignment: only for medium+ distance (>MIN_ALIGN_DIST)
                    // Close drones should separate, not align (Reynolds zones)
                    if (dist >= MIN_ALIGN_DIST) {
                        avgVx += (other.smoothVelocity?.x || 0) * w;
                        avgVz += (other.smoothVelocity?.z || 0) * w;
                        alignCount++;
                    }

                    // Cohesion: all neighbors contribute
                    comX += other.position.x * w;
                    comZ += other.position.z * w;

                    neighborCount++;
                }
            }
        }

        if (neighborCount === 0) return { x: 0, z: 0 };

        const fx = { x: 0, z: 0 };

        // ── Alignment: steer toward average neighbor heading ──
        // Only counts medium+ distance neighbors (>MIN_ALIGN_DIST)
        if (alignCount > 0) {
            avgVx /= alignCount;
            avgVz /= alignCount;
            // Alignment force = difference between avg neighbor velocity and own
            const myVx = state.smoothVelocity?.x || 0;
            const myVz = state.smoothVelocity?.z || 0;
            fx.x += (avgVx - myVx) * ALIGN_WEIGHT;
            fx.z += (avgVz - myVz) * ALIGN_WEIGHT;
        }

        // ── Cohesion: steer toward center of mass ──
        comX /= neighborCount;
        comZ /= neighborCount;
        const toComX = comX - state.position.x;
        const toComZ = comZ - state.position.z;
        const toComDist = Math.sqrt(toComX * toComX + toComZ * toComZ);
        if (toComDist > 1.0) { // only attract if CoM is >1m away
            fx.x += (toComX / toComDist) * COHESION_WEIGHT;
            fx.z += (toComZ / toComDist) * COHESION_WEIGHT;
        }

        // Cap total flocking force
        const fMag = Math.sqrt(fx.x * fx.x + fx.z * fx.z);
        if (fMag > MAX_FLOCK_FORCE) {
            const s = MAX_FLOCK_FORCE / fMag;
            fx.x *= s;
            fx.z *= s;
        }

        return fx;
    }

    // ─── Boundary containment force ─────────────────────────────────
    _getHalfZone() {
        if (this.halfBounds) {
            return Math.max(this.halfBounds.x, this.halfBounds.z);
        }
        if (this.doctrineManager) {
            const z = this.doctrineManager.zoneParams;
            return Math.max(z.sizeX, z.sizeZ) / 2;
        }
        return this.explorationBounds;
    }

    _getHalfBoundsXZ() {
        if (this.halfBounds) return this.halfBounds;
        const h = this._getHalfZone();
        return { x: h, z: h };
    }

    _computeBoundaryForce(state) {
        const force = new THREE.Vector3();
        const hb = this._getHalfBoundsXZ();
        const margin = 5; // start pushing back 5m before edge
        const strength = 8.0;

        // X boundaries
        if (state.position.x > hb.x - margin) {
            const pen = (state.position.x - (hb.x - margin)) / margin;
            force.x -= strength * Math.min(pen, 1);
        } else if (state.position.x < -(hb.x - margin)) {
            const pen = (-(hb.x - margin) - state.position.x) / margin;
            force.x += strength * Math.min(pen, 1);
        }

        // Z boundaries
        if (state.position.z > hb.z - margin) {
            const pen = (state.position.z - (hb.z - margin)) / margin;
            force.z -= strength * Math.min(pen, 1);
        } else if (state.position.z < -(hb.z - margin)) {
            const pen = (-(hb.z - margin) - state.position.z) / margin;
            force.z += strength * Math.min(pen, 1);
        }

        return force;
    }

    // ─── Apply state to Three.js drone ──────────────────────────────
    applyToDrone(drone, state) {
        if (!drone || !drone.mesh) return;

        // CRITICAL: Euler order must be YXZ for correct drone orientation
        // YXZ = heading(Y) first, then pitch(X) relative to heading, then roll(Z)
        // Default XYZ causes extreme visual tilt when heading is large
        if (drone.mesh.rotation.order !== 'YXZ') {
            drone.mesh.rotation.order = 'YXZ';
        }

        // Safety net: absolute ground floor — drones must NEVER visually go underground
        // Skip for IDLE/LANDED/LAND where drones are on or descending to the surface
        const ABSOLUTE_FLOOR = 0.15;
        // X500/S500 landing gear extends ~1m below mesh origin
        const gearOffset = state.x500Dynamics ? 1.2 : 0;
        if (state.phase !== 'IDLE' && state.phase !== 'LANDED' && state.phase !== 'LAND'
            && state.position.y < ABSOLUTE_FLOOR + 0.5 + gearOffset) {
            state.position.y = ABSOLUTE_FLOOR + 0.5 + gearOffset;
        }

        // Position with SMOOTHING — lerp mesh toward physics position for fluid movement
        // Raw physics position jumps cause visual jerkiness; lerp creates smooth interpolation
        // LAND/LANDED/IDLE: DIRECT SNAP (no lerp) — drone must visibly descend/sit on ground
        drone.position.copy(state.position); // internal state = exact physics
        const isX500Type = !!state.x500Dynamics;
        const isLanding = state.phase === 'LAND' || state.phase === 'LANDED' || state.phase === 'IDLE';
        if (isLanding) {
            // Direct snap — no visual lag during landing
            drone.mesh.position.copy(state.position);
        } else {
            const posLerp = isX500Type ? 0.12 : 0.18;
            drone.mesh.position.x += (state.position.x - drone.mesh.position.x) * posLerp;
            drone.mesh.position.y += (state.position.y - drone.mesh.position.y) * posLerp;
            drone.mesh.position.z += (state.position.z - drone.mesh.position.z) * posLerp;
        }

        // Heading → rotation
        try {
            // Smooth heading lerp — prevents visual snapping on large heading changes
            // (physics heading may still step, but visual rotation interpolates)
            if (state._displayHeading === undefined) state._displayHeading = state.heading;
            let headingDiff = state.heading - state._displayHeading;
            while (headingDiff > Math.PI) headingDiff -= 2 * Math.PI;
            while (headingDiff < -Math.PI) headingDiff += 2 * Math.PI;
            state._displayHeading += headingDiff * 0.25; // smooth lerp factor
            while (state._displayHeading > Math.PI) state._displayHeading -= 2 * Math.PI;
            while (state._displayHeading < -Math.PI) state._displayHeading += 2 * Math.PI;
            drone.mesh.rotation.y = state._displayHeading;

            // X500 realistic dynamics: use physics-computed attitude angles
            if (state.x500Dynamics && state.attitude) {
                // LAND/LANDED: force level (attitude already zeroed above)
                drone.mesh.rotation.z = state.attitude.roll;
                drone.mesh.rotation.x = state.attitude.pitch;
            } else {
                // Kinematic model: pitch/roll from velocity for visual dynamism
                // LAND/LANDED: velocity is 0 → target angles are 0 → level drone
                // MUST project world velocity into BODY frame (heading-relative)
                // because with YXZ Euler order, rotation.x/z are body-frame angles
                const h = state._displayHeading || 0;
                const cH = Math.cos(h), sH = Math.sin(h);
                // bodyForward > 0 = moving in drone's forward direction
                const bodyFwd   = -(state.velocity.x * sH + state.velocity.z * cH);
                const bodyRight = state.velocity.x * cH - state.velocity.z * sH;
                // Three.js YXZ: rotation.x < 0 = nose DOWN, rotation.z < 0 = bank RIGHT
                const pitchAngle = Math.max(-VISUAL.maxRollPitch, Math.min(VISUAL.maxRollPitch, -bodyFwd   * VISUAL.velocityTilt));
                const rollAngle  = Math.max(-VISUAL.maxRollPitch, Math.min(VISUAL.maxRollPitch, -bodyRight * VISUAL.velocityTilt));
                // Smooth lerp for roll/pitch — faster convergence when landing
                const tiltLerp = isLanding ? 0.5 : 0.20;
                const prevRz = drone.mesh.rotation.z || 0;
                const prevRx = drone.mesh.rotation.x || 0;
                drone.mesh.rotation.z = prevRz + (rollAngle - prevRz) * tiltLerp;
                drone.mesh.rotation.x = prevRx + (pitchAngle - prevRx) * tiltLerp;
            }
        } catch (_) { /* safe */ }

        // Sync velocity vector so swarm metrics (Vicsek alignment) can read it
        if (drone.velocity && state.velocity) {
            if (drone.velocity.copy) {
                drone.velocity.copy(state.velocity);
            } else {
                drone.velocity = { x: state.velocity.x, y: state.velocity.y, z: state.velocity.z };
            }
        }

        // Mark as flying (only if engine phase is active)
        // Use engine phase as canonical state (EXPLORE, TAKEOFF, HOVER, etc.)
        if (state.phase !== 'IDLE' && state.phase !== 'LANDED') {
            drone.state = state.phase || 'FLYING';
        } else {
            drone.state = 'IDLE';
        }

        // ── 3D label: show phase / doctrine / COA / autonomy / action / decision ──
        if (drone.updateStatusText) {
            // Throttle label redraws to ~2 Hz (every 500ms)
            const now = performance.now();
            if (!drone._labelLastDraw || now - drone._labelLastDraw > 500) {
                drone._labelLastDraw = now;
                const dm = this.doctrineManager;
                const autoOrg = this.autonomyLevel >= 90;
                drone.updateStatusText({
                    phase:            state.phase,
                    doctrine:         autoOrg ? '🔄 Auto-Organisé' : (dm?.currentDoctrine?.name || '—'),
                    coa:              autoOrg ? 'Distribué'        : (dm?.currentCOA?.name  || '—'),
                    autonomy:         this.autonomyLevel,
                    autonomyMode:     window.DIAMANTS_AUTONOMY_MODE || '',
                    action:           state._lastAction || '',
                    waypointsVisited: state.waypointsVisited || 0,
                    decision:         state._lastDecision || '',
                    reasoning:        state._lastReasoning || '',
                    comm:             this.commManager?.getCommLabel(state.id) || '',
                    commActive:       this.commManager?.isCommunicating(state.id) || false,
                    commDetails:      this.commManager?.getCommDetails(state.id) || null,
                    droneType:        state.profile?.id || 'CRAZYFLIE',
                    localKnowledge:   this.commManager?.getLocalKnowledge(state.id) || 0,
                    globalKnowledge:  this.visitedCells.size,
                    knowledgeDelta:   this.visitedCells.size - (this.commManager?.getLocalKnowledge(state.id) || 0),
                    commNeighborCount: this.commManager?.getCommNeighbors(state.id)?.size || 0,
                    beaconZones:      this.commManager?.getBeaconZones(state.id) || [],
                });
            }
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

    /** Set the drone intelligence manager (LLM brains) */
    setIntelligenceManager(manager) {
        this.intelligenceManager = manager;
        console.log('[Engine] 🧠 Intelligence manager connected');
    }

    /**
     * Trigger async LLM evaluation for a drone.
     * Result is stored in _llmResults and consumed on next waypoint pick.
     * @private
     */
    _triggerLLMEvaluation(state) {
        if (!this.intelligenceManager) return;
        const brain = this.intelligenceManager.getBrain(state.id);
        if (!brain || !brain.enabled) return;
        // Fire and forget — don't await in the sync update loop
        brain.evaluate(state, this).then(result => {
            this._llmResults.set(state.id, result);
        }).catch(() => {
            // Silently fail — FSM continues without LLM
        });
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
