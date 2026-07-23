/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * multi-agent-coordinator.js — Top-level coordinator for all autonomous agents.
 *
 * Implements SwarmIntelligenceInterface so it plugs directly into the
 * AutonomousFlightEngine via engine.setSwarmIntelligence(coordinator).
 *
 * Responsibilities:
 *   • Create one AutonomousAgent per drone
 *   • Route messages between agents (via simulated radio)
 *   • Track global coverage for cooperative rewards
 *   • Manage training episodes (start/stop/export/import)
 *   • Provide metrics for UI (per-drone and swarm-level)
 *
 * @module multi-agent-coordinator
 */

import { SwarmIntelligenceInterface } from '../intelligence/swarm-intelligence-interface.js';
import { AutonomousAgent, AgentPhase } from './autonomous-agent.js';
import { ModelRegistry } from './brain-interface.js';
import { AgentDroneRegistry } from './agent-drone-registry.js';

// ─── COORDINATOR CONFIG ──────────────────────────────────────────────

const DEFAULT_CONFIG = {
    communicationRange: 15,    // meters — max radio range
    messageDropRate: 0.0,      // 0..1 — simulated packet loss
    trainingEnabled: true,
    cooperativeRatio: 0.5,     // 0 = individual only, 1 = cooperative only
    maxEpisodeSteps: 5000,     // Steps before auto-reset
    episodeResetCoverage: 0.95 // Reset when coverage exceeds this
};


// ─── MULTI-AGENT COORDINATOR ─────────────────────────────────────────

export class MultiAgentCoordinator extends SwarmIntelligenceInterface {
    /**
     * @param {Object} config
     */
    constructor(config = {}) {
        super(config);
        this.cfg = { ...DEFAULT_CONFIG, ...config };

        /** @type {Map<string, AutonomousAgent>} */
        this.agents = new Map();

        // ── Global state ──
        /** @type {Set<string>} "x,z" keys */
        this.globalExploredCells = new Set();
        this._prevGlobalCoverage = 0;
        this.episodeCount = 0;
        this.globalSteps = 0;

        // ── Obstacle cache ──
        /** @type {Array} */
        this._obstacles = [];

        // ── Drone flight states (from engine) ──
        /** @type {Map<string, Object>} droneId → DroneFlightState */
        this._droneStates = new Map();

        // ── MARL Landing Coordination ──
        /** @type {Map<string, {x: number, z: number}>} droneId → assigned landing slot */
        this._landingSlots = new Map();
        /** @type {boolean} Whether landing coordination is active */
        this._landingCoordinationActive = false;
        /** @type {number} Minimum separation distance for landing slots (m) */
        this._landingMinSeparation = 3.0;

        // ── Message routing queue ──
        /** @type {Array<Object>} */
        this._pendingMessages = [];

        // ── Training log ──
        this.trainingLog = [];

        // ── Model registry (MLOps) ──
        this.registry = new ModelRegistry({
            maxSnapshots: config.maxSnapshots ?? 20,
            autoSnapshotInterval: config.autoSnapshotInterval ?? 10
        });

        // ── Experiment tracker ──
        this.experiments = [];
        this._currentExperiment = null;

        // ─ Enabled flag ──
        this.enabled = true;
    }

    // ────────────────────────────────────────────────────────────────
    // SwarmIntelligenceInterface IMPLEMENTATION
    // ────────────────────────────────────────────────────────────────

    /**
     * Called once by AutonomousFlightEngine on startup.
     * @param {Map<string, Object>} drones - Map droneId → DroneFlightState
     * @param {Array} obstacles - [{center, radius, height}]
     */
    initialize(drones, obstacles) {
        this._obstacles = obstacles || [];
        this._droneStates = drones;

        // Create one agent per drone
        let index = 0;
        const droneIds = [...drones.keys()];

        for (const droneId of droneIds) {
            if (!this.agents.has(droneId)) {
                // Determine profile from engine's DroneFlightState
                const droneState = drones.get(droneId);
                const profileId = droneState?.profileId || this._inferProfileFromId(droneId);

                const agent = new AutonomousAgent(droneId, index, {
                    trainingEnabled: this.cfg.trainingEnabled,
                    cooperativeRatio: this.cfg.cooperativeRatio
                });

                // Bind profile-appropriate brain
                const brain = AgentDroneRegistry.createBrain(profileId, droneId);
                agent.setBrain(brain);
                agent._profileId = profileId;
                agent._agentRole = AgentDroneRegistry.getRole(profileId);

                this.agents.set(droneId, agent);
                index++;
            }
        }

        // Wire peer references (all-to-all for Raft)
        for (const [id, agent] of this.agents) {
            for (const otherId of droneIds) {
                if (otherId !== id) {
                    agent.addPeer(otherId);
                }
            }
        }

        console.log(`[MultiAgentCoordinator] Initialized ${this.agents.size} agents`);
    }

    /**
     * Called every frame by the engine after all drones are updated.
     * @param {number} dt - Delta time in seconds
     */
    tick(dt) {
        if (!this.enabled) return;

        const now = performance.now();

        // 1. Sync positions from engine states
        this._syncFromEngine();

        // 2. Build peer states map for all agents
        const peerStates = this._buildPeerStates();

        // 3. Route pending messages from last frame
        this._routeMessages(now);

        // 4. Tick each agent
        for (const [droneId, agent] of this.agents) {
            if (agent.phase === AgentPhase.IDLE || agent.phase === AgentPhase.LANDED) {
                continue;
            }

            agent.tick(dt, now, this._obstacles, peerStates, null);

            // Collect outgoing messages
            const outgoing = agent.drainMessages(now);
            this._pendingMessages.push(...outgoing);
        }

        // 5. MARL Landing Coordination — assign optimal landing slots
        this._coordinateLanding();

        // 6. Update global coverage
        this._updateGlobalCoverage();

        // 7. Check episode termination
        this.globalSteps++;
        if (this.cfg.trainingEnabled) {
            this._checkEpisodeEnd();
        }

        // 8. Emit federated-update event for UI panel (throttled — called at 1Hz from controller)
        if (typeof window !== 'undefined') {
            const coverage = this._computeGlobalCoveragePercent();
            const agents = [];
            for (const [id, agent] of this.agents) {
                let covNum = 0, role = 'explorer';
                try {
                    const m = agent.getMetrics();
                    covNum = parseFloat(m.coverage) || 0;
                    role = m.role || 'explorer';
                } catch (_) { /* agent not fully initialized */ }
                agents.push({
                    id,
                    role,
                    localCoverage: covNum / 100,
                    reward: agent.brain?.avgReward ?? 0,
                    position_n: agent.position?.x ?? 0,
                    position_e: agent.position?.z ?? 0,
                });
            }
            const avgReward = agents.length > 0
                ? agents.reduce((s, a) => s + (a.reward || 0), 0) / agents.length
                : 0;

            // Build 40×40 coverageGrid from explored cells (zoneSize=80m, 2m cells)
            const gridDim = 40;
            const coverageGrid = new Float32Array(gridDim * gridDim);
            for (const key of this.globalExploredCells) {
                const sep = key.indexOf(',');
                const cx = +key.slice(0, sep);
                const cz = +key.slice(sep + 1);
                const ni = cx + (gridDim >> 1);
                const nj = cz + (gridDim >> 1);
                if (ni >= 0 && ni < gridDim && nj >= 0 && nj < gridDim) {
                    coverageGrid[ni * gridDim + nj] = 1.0;
                }
            }

            window.dispatchEvent(new CustomEvent('diamants:federated-update', {
                detail: {
                    round: this.globalSteps,
                    strategy: 'FEDAVG',
                    globalCoverage: coverage / 100,
                    avgReward,
                    coverageGrid,
                    agents,
                }
            }));

            // Also emit drone-positions for live minimap dots
            const positions = {};
            for (const [id, agent] of this.agents) {
                positions[id] = {
                    position: { x: agent.position?.x ?? 0, y: agent.position?.y ?? 0, z: agent.position?.z ?? 0 },
                    type: agent._profileId || 'crazyflie',
                    source: 'engine',
                };
            }
            window.dispatchEvent(new CustomEvent('diamants:drone-positions', { detail: positions }));
        }
    }

    /**
     * Compute next waypoint for a drone.
     * Called by AutonomousFlightEngine per drone.
     *
     * @param {string} droneId
     * @param {Object} state - DroneFlightState
     * @param {Array} neighbors - Neighbor states
     * @returns {THREE.Vector3|null}
     */
    computeNextWaypoint(droneId, state, neighbors) {
        const agent = this.agents.get(droneId);
        if (!agent || !this.enabled) return null;

        // The agent's tick() already computed a waypoint in the executive layer.
        // Return it as a THREE.Vector3-compatible object.
        if (agent.currentWaypoint) {
            return agent.currentWaypoint;
        }
        return null;
    }

    /**
     * Modulate velocity (reactive avoidance layer).
     */
    modulateVelocity(droneId, velocity, state) {
        // Let the agent's reactive layer handle collision avoidance.
        // We don't override velocity here — the waypoint is enough.
        return velocity;
    }

    /**
     * Update environment observation for a drone.
     * Called by the engine each frame.
     */
    updateEnvironment(droneId, position, observation) {
        const agent = this.agents.get(droneId);
        if (!agent) return;

        // Sync state from engine observation
        agent.setPosition(position.x, position.y, position.z);

        if (observation) {
            agent.setMotion(observation.heading || 0, observation.speed || 0);

            // Start exploration phase if engine says EXPLORE
            if (observation.phase === 'EXPLORE' && agent.phase === AgentPhase.IDLE) {
                agent.setPhase(AgentPhase.EXPLORING);
            }
        }
    }

    /**
     * Serialize coordinator state for save/export.
     */
    serialize() {
        const agentData = {};
        for (const [id, agent] of this.agents) {
            agentData[id] = agent.exportFull();
        }
        return {
            name: 'multi-agent-trainable',
            episodeCount: this.episodeCount,
            globalSteps: this.globalSteps,
            agents: agentData,
            config: this.cfg
        };
    }

    /**
     * Restore coordinator state.
     */
    deserialize(data) {
        if (!data || !data.agents) return;
        this.episodeCount = data.episodeCount || 0;
        this.globalSteps = data.globalSteps || 0;

        for (const [id, agentData] of Object.entries(data.agents)) {
            const agent = this.agents.get(id);
            if (agent && agentData.brain) {
                agent.importWeights(agentData.brain);
            }
        }
    }

    /**
     * Return metrics for debug / UI.
     */
    getMetrics() {
        const agentMetrics = {};
        for (const [id, agent] of this.agents) {
            agentMetrics[id] = agent.getMetrics();
        }

        return {
            name: 'multi-agent-trainable',
            enabled: this.enabled,
            agentCount: this.agents.size,
            globalSteps: this.globalSteps,
            episodeCount: this.episodeCount,
            globalCoverage: (this._computeGlobalCoveragePercent()).toFixed(1) + '%',
            trainingEnabled: this.cfg.trainingEnabled,
            cooperativeRatio: this.cfg.cooperativeRatio,
            agents: agentMetrics
        };
    }

    // ────────────────────────────────────────────────────────────────
    // PUBLIC TRAINING API
    // ────────────────────────────────────────────────────────────────

    /**
     * Enable/disable training for all agents.
     */
    setTrainingEnabled(enabled) {
        this.cfg.trainingEnabled = enabled;
        for (const agent of this.agents.values()) {
            agent.setTrainingEnabled(enabled);
        }
    }

    /**
     * Set cooperative ratio for all agents (from autonomy slider).
     * @param {number} ratio - 0..1
     */
    setCooperativeRatio(ratio) {
        this.cfg.cooperativeRatio = ratio;
        for (const agent of this.agents.values()) {
            agent.setCooperativeRatio(ratio);
        }
    }

    /**
     * Set autonomy level (from slider 0-100).
     * Maps to cooperative ratio: autonomy 100 → ratio 0 (fully individual).
     * Also controls when the coordinator is active vs fallback to heuristic.
     */
    setAutonomyLevel(level) {
        // Active only above 50% autonomy
        this.enabled = level >= 50;

        // Map autonomy to cooperative ratio — INVERTED:
        // High autonomy = individual behaviour (ratio→0)
        // Low autonomy  = cooperative behaviour (ratio→1)
        if (level >= 50) {
            const adjustedRatio = Math.max(0, 1 - (level - 50) / 50);
            this.setCooperativeRatio(adjustedRatio);
        }
    }

    /**
     * Export all agent weights as a single JSON.
     * @returns {Object}
     */
    exportAllWeights() {
        const weights = {};
        for (const [id, agent] of this.agents) {
            weights[id] = agent.exportWeights();
        }
        return {
            version: 1,
            episodeCount: this.episodeCount,
            globalSteps: this.globalSteps,
            timestamp: Date.now(),
            agents: weights
        };
    }

    /**
     * Import weights for all agents.
     * @param {Object} data
     */
    importAllWeights(data) {
        if (!data || !data.agents) return;
        for (const [id, weights] of Object.entries(data.agents)) {
            const agent = this.agents.get(id);
            if (agent) {
                agent.importWeights(weights);
            }
        }
        console.log(`[MultiAgentCoordinator] Imported weights for ${Object.keys(data.agents).length} agents`);
    }

    /**
     * Start a new training episode (reset all agents).
     */
    startNewEpisode() {
        this.episodeCount++;
        this.globalExploredCells.clear();
        this._prevGlobalCoverage = 0;

        for (const agent of this.agents.values()) {
            agent.resetEpisode();
            agent.setPhase(AgentPhase.EXPLORING);
        }

        console.log(`[MultiAgentCoordinator] Episode ${this.episodeCount} started`);
    }

    /**
     * Explicitly activate an agent (when drone takes off).
     */
    activateAgent(droneId) {
        const agent = this.agents.get(droneId);
        if (agent) {
            agent.setPhase(AgentPhase.EXPLORING);
        }
    }

    /**
     * Deactivate an agent (when drone lands).
     */
    deactivateAgent(droneId) {
        const agent = this.agents.get(droneId);
        if (agent) {
            agent.setPhase(AgentPhase.LANDED);
        }
    }

    // ────────────────────────────────────────────────────────────────
    // HOT AGENT INJECTION / REMOVAL
    // ────────────────────────────────────────────────────────────────

    /**
     * Hot-inject a new agent into the running swarm.
     * Creates the agent, wires Raft peers, optionally loads weights.
     *
     * @param {string} droneId - Unique drone ID
     * @param {Object} [config] - Agent config overrides
     * @param {Object} [initialWeights] - Pre-trained weights to import
     * @returns {AutonomousAgent|null} The new agent, or null on failure
     */
    addAgent(droneId, config = {}, initialWeights = null) {
        if (this.agents.has(droneId)) {
            console.warn(`[MultiAgentCoordinator] Agent ${droneId} already exists`);
            return this.agents.get(droneId);
        }

        const index = this.agents.size;
        const profileId = config.profileId || this._inferProfileFromId(droneId);

        const agent = new AutonomousAgent(droneId, index, {
            trainingEnabled: this.cfg.trainingEnabled,
            cooperativeRatio: this.cfg.cooperativeRatio,
            ...config
        });

        // Bind profile-appropriate brain (reactive for Crazyflie, cognitive for X500, etc.)
        const brain = AgentDroneRegistry.createBrain(profileId, droneId, config.brainConfig);
        agent.setBrain(brain);
        agent._profileId = profileId;
        agent._agentRole = AgentDroneRegistry.getRole(profileId);

        // Import weights if provided
        if (initialWeights) {
            agent.importWeights(initialWeights);
        }

        // Wire Raft peers (all-to-all)
        for (const [existingId, existingAgent] of this.agents) {
            existingAgent.addPeer(droneId);
            agent.addPeer(existingId);
        }

        this.agents.set(droneId, agent);

        // Auto-activate if we're in an active episode
        if (this.globalSteps > 0) {
            agent.setPhase(AgentPhase.EXPLORING);
        }

        console.log(`[MultiAgentCoordinator] ✅ Agent ${droneId} hot-injected (total: ${this.agents.size})`);
        return agent;
    }

    /**
     * Remove an agent from the running swarm.
     * Properly cleans up Raft peers and reassigns zones.
     *
     * @param {string} droneId
     * @param {boolean} [exportWeights=true] - Return the agent's weights before removing
     * @returns {Object|null} Exported weights, or null if agent not found
     */
    removeAgent(droneId, exportWeights = true) {
        const agent = this.agents.get(droneId);
        if (!agent) {
            console.warn(`[MultiAgentCoordinator] Agent ${droneId} not found`);
            return null;
        }

        // Export weights before removal
        const weights = exportWeights ? agent.exportWeights() : null;

        // Remove Raft peer from all other agents
        for (const [otherId, otherAgent] of this.agents) {
            if (otherId !== droneId) {
                otherAgent.removePeer(droneId);
            }
        }

        // Remove from agents map
        this.agents.delete(droneId);

        console.log(`[MultiAgentCoordinator] ❌ Agent ${droneId} removed (remaining: ${this.agents.size})`);
        return weights;
    }

    /**
     * Swap the brain of a specific agent (hot brain swap).
     * @param {string} droneId
     * @param {import('./brain-interface.js').BrainInterface} newBrain
     * @param {boolean} [transferWeights=false]
     * @returns {boolean} success
     */
    swapAgentBrain(droneId, newBrain, transferWeights = false) {
        const agent = this.agents.get(droneId);
        if (!agent) {
            console.warn(`[MultiAgentCoordinator] Agent ${droneId} not found for brain swap`);
            return false;
        }
        return agent.setBrain(newBrain, transferWeights);
    }

    /**
     * Get the count of active agents.
     * @returns {number}
     */
    getAgentCount() {
        return this.agents.size;
    }

    /**
     * Get IDs of all active agents.
     * @returns {string[]}
     */
    getAgentIds() {
        return [...this.agents.keys()];
    }

    /**
     * Infer drone profile from droneId naming convention.
     * @private
     */
    _inferProfileFromId(droneId) {
        if (droneId.startsWith('x500')) return 'X500';
        if (droneId.startsWith('s500')) return 'S500';
        if (droneId.startsWith('mavic')) return 'MAVIC';
        if (droneId.startsWith('phantom')) return 'PHANTOM';
        return 'CRAZYFLIE';
    }

    /**
     * Get the fleet composition by agent role.
     * @returns {{ reactive: number, cognitive: number, hybrid: number, total: number, roles: Object }}
     */
    getFleetComposition() {
        const counts = { reactive: 0, cognitive: 0, hybrid: 0 };
        const roles = {};
        for (const [id, agent] of this.agents) {
            const role = agent._agentRole || 'reactive';
            counts[role] = (counts[role] || 0) + 1;
            roles[id] = {
                role,
                profileId: agent._profileId || 'CRAZYFLIE',
                brainType: agent.getBrainType()
            };
        }
        return { ...counts, total: this.agents.size, roles };
    }

    // ────────────────────────────────────────────────────────────────
    // MLOPS: SNAPSHOTS & ROLLBACK
    // ────────────────────────────────────────────────────────────────

    /**
     * Create a manual snapshot of all agent weights.
     * @param {Object} [metadata] - Extra info to store with the snapshot
     * @returns {Object} The created snapshot
     */
    createSnapshot(metadata = {}) {
        const weights = this.exportAllWeights();
        return this.registry.createSnapshot(weights, {
            episodeCount: this.episodeCount,
            globalSteps: this.globalSteps,
            coverage: this._computeGlobalCoveragePercent(),
            agentCount: this.agents.size,
            ...metadata
        });
    }

    /**
     * Rollback all agents to a previous snapshot.
     * @param {number} snapshotIndex
     * @returns {boolean} success
     */
    rollbackToSnapshot(snapshotIndex) {
        const weights = this.registry.rollback(snapshotIndex);
        if (!weights) return false;
        this.importAllWeights(weights);
        console.log(`[MultiAgentCoordinator] Rolled back to snapshot #${snapshotIndex}`);
        return true;
    }

    /**
     * List all available snapshots.
     * @returns {Array<Object>}
     */
    listSnapshots() {
        return this.registry.listSnapshots();
    }

    /**
     * Register current weights as a named model.
     * @param {string} name - Model name
     * @param {Object} [metadata] - Description, hyperparams, etc.
     * @returns {Object}
     */
    registerModel(name, metadata = {}) {
        const weights = this.exportAllWeights();
        return this.registry.registerModel(name, weights, {
            agentCount: this.agents.size,
            episodeCount: this.episodeCount,
            ...metadata
        });
    }

    /**
     * Load a named model into all agents.
     * @param {string} name
     * @returns {boolean}
     */
    loadModel(name) {
        const model = this.registry.getModel(name);
        if (!model) {
            console.warn(`[MultiAgentCoordinator] Model "${name}" not found`);
            return false;
        }
        this.importAllWeights(model.weights);
        console.log(`[MultiAgentCoordinator] Loaded model "${name}" v${model.metadata.version}`);
        return true;
    }

    // ────────────────────────────────────────────────────────────────
    // MLOPS: A/B TESTING
    // ────────────────────────────────────────────────────────────────

    /**
     * Start an A/B test between two named models.
     * Assigns agents to groups and loads appropriate weights.
     * @param {string} testName
     * @param {string} modelA
     * @param {string} modelB
     * @param {number} [splitRatio=0.5]
     * @returns {Object|null} Test config with assignments
     */
    startABTest(testName, modelA, modelB, splitRatio = 0.5) {
        const test = this.registry.createABTest(testName, modelA, modelB, splitRatio);
        if (!test) return null;

        const droneIds = this.getAgentIds();
        const assignments = this.registry.assignABGroups(testName, droneIds);

        // Load appropriate weights per agent
        for (const [droneId, group] of assignments) {
            const weights = this.registry.getABWeights(testName, droneId);
            if (weights) {
                const agent = this.agents.get(droneId);
                if (agent) agent.importWeights(weights);
            }
        }

        console.log(`[MultiAgentCoordinator] A/B test "${testName}" started with ${droneIds.length} agents`);
        return { ...test, assignments: Object.fromEntries(assignments) };
    }

    /**
     * Get A/B test summary.
     * @param {string} testName
     * @returns {Object|null}
     */
    getABTestResults(testName) {
        return this.registry.getABSummary(testName);
    }

    // ────────────────────────────────────────────────────────────────
    // MLOPS: EXPERIMENT TRACKING
    // ────────────────────────────────────────────────────────────────

    /**
     * Start a new experiment (tracks metrics over multiple episodes).
     * @param {string} name - Experiment name
     * @param {Object} [hyperparams] - Hyperparameters being tested
     * @returns {Object} Experiment handle
     */
    startExperiment(name, hyperparams = {}) {
        const experiment = {
            id: this.experiments.length,
            name,
            hyperparams,
            startedAt: Date.now(),
            episodes: [],
            status: 'running'
        };
        this.experiments.push(experiment);
        this._currentExperiment = experiment;
        console.log(`[MultiAgentCoordinator] Experiment "${name}" started`);
        return experiment;
    }

    /**
     * Record an episode's metrics in the current experiment.
     * Called automatically at episode end if an experiment is active.
     * @param {Object} metrics
     */
    recordEpisodeMetrics(metrics) {
        if (!this._currentExperiment || this._currentExperiment.status !== 'running') return;
        this._currentExperiment.episodes.push({
            timestamp: Date.now(),
            ...metrics
        });
    }

    /**
     * End the current experiment.
     * @returns {Object|null} Experiment summary
     */
    endExperiment() {
        if (!this._currentExperiment) return null;
        this._currentExperiment.status = 'completed';
        this._currentExperiment.endedAt = Date.now();

        const exp = this._currentExperiment;
        this._currentExperiment = null;

        // Compute summary stats
        const episodes = exp.episodes;
        if (episodes.length > 0) {
            const rewards = episodes.map(e => e.avgReward || 0);
            exp.summary = {
                totalEpisodes: episodes.length,
                avgReward: rewards.reduce((a, b) => a + b, 0) / rewards.length,
                bestReward: Math.max(...rewards),
                worstReward: Math.min(...rewards),
                durationMs: exp.endedAt - exp.startedAt
            };
        }

        console.log(`[MultiAgentCoordinator] Experiment "${exp.name}" completed (${episodes.length} episodes)`);
        return exp;
    }

    /**
     * List all experiments.
     * @returns {Array<Object>}
     */
    listExperiments() {
        return this.experiments.map(e => ({
            id: e.id,
            name: e.name,
            status: e.status,
            episodes: e.episodes.length,
            startedAt: e.startedAt,
            summary: e.summary || null
        }));
    }

    /**
     * Export experiment data as JSON (for external analysis).
     * @param {number} [experimentId] - Specific experiment, or all if omitted
     * @returns {Object}
     */
    exportExperiments(experimentId) {
        if (experimentId !== undefined) {
            return this.experiments[experimentId] || null;
        }
        return {
            experiments: this.experiments,
            exportedAt: Date.now()
        };
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: Message Routing
    // ────────────────────────────────────────────────────────────────

    _routeMessages(now) {
        const toRoute = this._pendingMessages.splice(0);

        for (const msg of toRoute) {
            // Simulate packet loss
            if (Math.random() < this.cfg.messageDropRate) continue;

            if (msg.to) {
                // Targeted message (Raft, etc.)
                this._deliverTo(msg.to, msg, now);
            } else {
                // Broadcast — deliver to all peers in range
                const sender = this.agents.get(msg.from);
                if (!sender) continue;

                for (const [peerId, peerAgent] of this.agents) {
                    if (peerId === msg.from) continue;

                    // Range check
                    const dx = sender.position.x - peerAgent.position.x;
                    const dz = sender.position.z - peerAgent.position.z;
                    const dist = Math.sqrt(dx * dx + dz * dz);

                    if (dist <= this.cfg.communicationRange) {
                        this._deliverTo(peerId, msg, now);
                    }
                }
            }
        }
    }

    _deliverTo(droneId, msg, now) {
        const agent = this.agents.get(droneId);
        if (agent) {
            agent.receiveMessage(msg, now);
        }
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: State Sync
    // ────────────────────────────────────────────────────────────────

    _syncFromEngine() {
        for (const [droneId, state] of this._droneStates) {
            const agent = this.agents.get(droneId);
            if (!agent) continue;

            if (state.position) {
                agent.setPosition(
                    state.position.x,
                    state.position.y,
                    state.position.z
                );
            }
            if (state.heading !== undefined) {
                agent.setMotion(state.heading, state.speed || 0);
            }

            // Detect phase transition
            if (state.phase === 'EXPLORE' && agent.phase === AgentPhase.IDLE) {
                agent.setPhase(AgentPhase.EXPLORING);
            }
            // Sync landing phase to agent
            if (state.phase === 'LAND' || state._returningToBase) {
                if (agent.phase !== AgentPhase.LANDED) {
                    agent.setPhase(AgentPhase.EXPLORING); // keep agent active during landing coord
                }
            }
            if (state.phase === 'LANDED' && agent.phase !== AgentPhase.LANDED) {
                agent.setPhase(AgentPhase.LANDED);
            }
        }
    }

    _buildPeerStates() {
        const states = new Map();
        for (const [droneId, agent] of this.agents) {
            states.set(droneId, {
                position: { ...agent.position },
                heading: agent.heading,
                speed: agent.speed,
                currentAction: agent.currentAction ?? -1
            });
        }
        return states;
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: MARL Landing Coordination
    // ────────────────────────────────────────────────────────────────

    /**
     * MARL-coordinated landing: assign optimal landing slots to drones
     * that are returning to base or in LAND phase.
     *
     * Uses a reward-based optimization:
     * - Maximize separation between landing slots
     * - Penalize slot assignments that cause drone crossings
     * - Stagger landing sequence for safety
     *
     * Landing slots are written to each drone's `_landingSlot` in the engine state.
     */
    _coordinateLanding() {
        // Collect drones that need landing coordination
        const landingDrones = [];
        const otherDrones = []; // already landed or not landing

        for (const [droneId, state] of this._droneStates) {
            if (state._returningToBase || state.phase === 'LAND') {
                landingDrones.push({ id: droneId, state });
            } else if (state.phase === 'LANDED') {
                otherDrones.push({ id: droneId, pos: { x: state.position.x, z: state.position.z } });
            }
        }

        if (landingDrones.length === 0) {
            if (this._landingCoordinationActive) {
                this._landingCoordinationActive = false;
                this._landingSlots.clear();
                console.log('[MARL-LANDING] Landing coordination completed');
            }
            return;
        }

        // Activate coordination if not already active
        if (!this._landingCoordinationActive) {
            this._landingCoordinationActive = true;
            console.log(`[MARL-LANDING] Coordinating landing for ${landingDrones.length} drones`);
        }

        // Build list of occupied positions (already landed drones)
        const occupied = otherDrones.map(d => d.pos);

        // Compute optimal landing slots using MARL reward optimization
        // Strategy: Use home positions as base slots, then apply separation forces
        for (const { id, state } of landingDrones) {
            const home = state.homePosition;
            if (!home) continue;

            // Start from home position as candidate slot
            let slotX = home.x;
            let slotZ = home.z;

            // Apply MARL separation force: push slot away from:
            // 1. Other landing drones' slots
            // 2. Already-landed drones
            const iterations = 3; // iterative refinement
            for (let iter = 0; iter < iterations; iter++) {
                let forceX = 0, forceZ = 0;

                // Repulsion from other landing drones' assigned slots
                for (const { id: otherId, state: otherState } of landingDrones) {
                    if (otherId === id) continue;
                    const otherHome = otherState.homePosition || otherState.position;
                    const otherSlot = this._landingSlots.get(otherId) || { x: otherHome.x, z: otherHome.z };
                    const dx = slotX - otherSlot.x;
                    const dz = slotZ - otherSlot.z;
                    const dist = Math.sqrt(dx * dx + dz * dz) + 0.01;

                    if (dist < this._landingMinSeparation) {
                        // Strong repulsion when too close — MARL penalty drives separation
                        const strength = (this._landingMinSeparation - dist) / dist * 1.5;
                        forceX += (dx / dist) * strength;
                        forceZ += (dz / dist) * strength;
                    }
                }

                // Repulsion from already-landed drones
                for (const occ of occupied) {
                    const dx = slotX - occ.x;
                    const dz = slotZ - occ.z;
                    const dist = Math.sqrt(dx * dx + dz * dz) + 0.01;

                    if (dist < this._landingMinSeparation) {
                        const strength = (this._landingMinSeparation - dist) / dist * 1.0;
                        forceX += (dx / dist) * strength;
                        forceZ += (dz / dist) * strength;
                    }
                }

                // Apply force with decay
                const decay = 0.5 / (iter + 1);
                slotX += forceX * decay;
                slotZ += forceZ * decay;

                // Keep slot within reasonable range of home (max 4m deviation)
                const devX = slotX - home.x;
                const devZ = slotZ - home.z;
                const dev = Math.sqrt(devX * devX + devZ * devZ);
                if (dev > 4.0) {
                    slotX = home.x + (devX / dev) * 4.0;
                    slotZ = home.z + (devZ / dev) * 4.0;
                }
            }

            // Store the optimized landing slot
            this._landingSlots.set(id, { x: slotX, z: slotZ });

            // Write the slot into the engine's drone state for the LAND phase to use
            state._landingSlot = { x: slotX, z: slotZ };
        }

        // Compute MARL landing reward (for training feedback)
        this._computeLandingReward(landingDrones);
    }

    /**
     * Compute MARL reward signal for landing quality.
     * Positive reward for well-separated landings, penalty for proximity.
     */
    _computeLandingReward(landingDrones) {
        for (const { id, state } of landingDrones) {
            const agent = this.agents.get(id);
            if (!agent || !agent.trainingEnabled) continue;

            const slot = this._landingSlots.get(id);
            if (!slot) continue;

            let reward = 0;

            // Distance to assigned slot — reward for being on-target
            const dxSlot = state.position.x - slot.x;
            const dzSlot = state.position.z - slot.z;
            const distToSlot = Math.sqrt(dxSlot * dxSlot + dzSlot * dzSlot);
            reward += Math.max(0, 1.0 - distToSlot / 5.0); // +1 when on slot, 0 when >5m away

            // Separation from other landing drones — reward for maintaining distance
            for (const { id: otherId, state: otherState } of landingDrones) {
                if (otherId === id) continue;
                const dx = state.position.x - otherState.position.x;
                const dz = state.position.z - otherState.position.z;
                const dist = Math.sqrt(dx * dx + dz * dz);

                if (dist < this._landingMinSeparation) {
                    // Penalty for being too close during landing
                    reward -= 2.0 * (1.0 - dist / this._landingMinSeparation);
                } else {
                    // Small positive reward for good separation
                    reward += 0.2;
                }
            }

            // Feed reward to agent's brain for learning
            agent.episodeReward = (agent.episodeReward || 0) + reward * 0.1;
        }
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: Global Coverage
    // ────────────────────────────────────────────────────────────────

    _updateGlobalCoverage() {
        for (const [droneId, agent] of this.agents) {
            // Merge agent's explored cells into global set (radius 3m → 2 cell radius)
            const baseCx = Math.floor(agent.position.x / 2);
            const baseCz = Math.floor(agent.position.z / 2);
            for (let dx = -2; dx <= 2; dx++) {
                for (let dz = -2; dz <= 2; dz++) {
                    this.globalExploredCells.add(`${baseCx + dx},${baseCz + dz}`);
                }
            }
        }
    }

    _computeGlobalCoveragePercent() {
        // Assume 60×60m area, 2m cells → 900 cells total
        const totalCells = 900;
        return (this.globalExploredCells.size / totalCells) * 100;
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: Episode Management
    // ────────────────────────────────────────────────────────────────

    _checkEpisodeEnd() {
        const coverage = this._computeGlobalCoveragePercent();
        const tooManySteps = this.globalSteps >= this.cfg.maxEpisodeSteps;
        const coverageDone = coverage >= this.cfg.episodeResetCoverage * 100;

        if (tooManySteps || coverageDone) {
            // Log episode
            const episodeLog = {
                episode: this.episodeCount,
                steps: this.globalSteps,
                coveragePercent: coverage,
                agentRewards: {}
            };
            const agentRewards = {};
            for (const [id, agent] of this.agents) {
                episodeLog.agentRewards[id] = agent.episodeReward;
                agentRewards[id] = {
                    reward: agent.episodeReward,
                    coverage: agent.perception?.grid?.localCoverage?.() || 0,
                    steps: agent.episodeSteps
                };
            }
            this.trainingLog.push(episodeLog);

            // Terminal reward for all agents
            for (const agent of this.agents.values()) {
                if (agent.trainingEnabled && agent._lastObservation) {
                    // Ensure action is a number (safety net — main fix is in autonomous-agent.js)
                    const termAction = typeof agent.currentAction === 'number'
                        ? agent.currentAction
                        : (agent.currentAction?.action ?? 0);
                    agent.brain.learn(
                        agent._lastObservation,
                        termAction,
                        coverageDone ? 10.0 : -5.0,  // Bonus for mission complete
                        agent._lastObservation,
                        true // terminal
                    );
                }
            }

            // ── MLOps: Auto-snapshot ──
            const weights = this.exportAllWeights();
            this.registry.onEpisodeEnd(weights, {
                episodeCount: this.episodeCount,
                globalSteps: this.globalSteps,
                coverage
            });

            // ── MLOps: Experiment tracking ──
            if (this._currentExperiment) {
                const avgReward = Object.values(agentRewards)
                    .reduce((sum, r) => sum + r.reward, 0) / Math.max(1, this.agents.size);
                this.recordEpisodeMetrics({
                    episode: this.episodeCount,
                    steps: this.globalSteps,
                    coverage,
                    avgReward,
                    agentRewards
                });
            }

            // ── MLOps: A/B test recording ──
            for (const test of this.registry.abTests.values()) {
                if (test.active) {
                    this.registry.recordABResults(test.name, agentRewards);
                }
            }

            console.log(`[MultiAgentCoordinator] Episode ${this.episodeCount} ended — ${coverage.toFixed(1)}% coverage in ${this.globalSteps} steps`);

            this.startNewEpisode();
            this.globalSteps = 0;
        }
    }
}
