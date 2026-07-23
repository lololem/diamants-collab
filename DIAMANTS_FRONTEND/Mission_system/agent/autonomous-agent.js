/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * autonomous-agent.js — Complete autonomous agent for one drone.
 *
 * Orchestrates all sub-modules (brain, perception, communication, consensus,
 * rewards) into a coherent sense→decide→act→learn loop.
 *
 * Architecture (Hybrid 3-layer, Proposition B):
 *   ┌──────────────────────────────────────┐
 *   │  Deliberative (2 Hz)                │
 *   │  • Raft consensus leader tasks      │
 *   │  • Frontier assignment via CBBA      │
 *   │  • Strategy review                  │
 *   ├──────────────────────────────────────┤
 *   │  Executive (10 Hz)                  │
 *   │  • Q-learning decision              │
 *   │  • Communication scheduling         │
 *   │  • Reward computation & learning    │
 *   ├──────────────────────────────────────┤
 *   │  Reactive (60 Hz / every tick)      │
 *   │  • Collision avoidance              │
 *   │  • Waypoint interpolation           │
 *   │  • Perception update                │
 *   └──────────────────────────────────────┘
 *
 * @module autonomous-agent
 */

import { AgentBrain, AgentAction, OBS_DIM } from './agent-brain.js';
import { BrainInterface }                   from './brain-interface.js';
import { SimulatedPerception }              from './agent-perception.js';
import { AgentCommunicator }                from './agent-communication.js';
import { RaftNode, RaftCommand }            from './raft-consensus.js';
import { RewardShaper }                     from './reward-shaper.js';

// ─── AGENT PHASES ────────────────────────────────────────────────────

export const AgentPhase = Object.freeze({
    IDLE:      'idle',
    TAKEOFF:   'takeoff',
    EXPLORING: 'exploring',
    RETURNING: 'returning',
    LANDED:    'landed'
});

// ─── LAYER RATES ─────────────────────────────────────────────────────

const DELIBERATIVE_PERIOD_MS = 500;   // 2 Hz
const EXECUTIVE_PERIOD_MS    = 100;   // 10 Hz
// Reactive = every tick (60 Hz)

// ─── ACTION → DISPLACEMENT MAPPING ──────────────────────────────────

const ACTION_DISPLACEMENT = {
    [AgentAction.EXPLORE_FORWARD]: { dx:  0, dz: -1 },
    [AgentAction.EXPLORE_LEFT]:    { dx: -1, dz:  0 },
    [AgentAction.EXPLORE_RIGHT]:   { dx:  1, dz:  0 },
    [AgentAction.HOVER]:           { dx:  0, dz:  0 },
    [AgentAction.RETREAT]:         { dx:  0, dz:  1 },
};

const WAYPOINT_STEP = 3.0;  // meters per waypoint step


// ─── AUTONOMOUS AGENT ────────────────────────────────────────────────

export class AutonomousAgent {
    /**
     * @param {string} droneId
     * @param {number} droneIndex - Index in swarm (0..N-1) for spacing
     * @param {Object} config
     */
    constructor(droneId, droneIndex = 0, config = {}) {
        this.droneId    = droneId;
        this.droneIndex = droneIndex;
        this.phase      = AgentPhase.IDLE;

        // ── Sub-modules ──
        this.brain         = new AgentBrain(droneId, config.brain);
        this.perception    = new SimulatedPerception(droneId, config.perception);
        this.communicator  = new AgentCommunicator(droneId, config.communication);
        this.raft          = new RaftNode(droneId, config.raft);
        this.rewardShaper  = new RewardShaper(config.reward);

        // ── State ──
        this.position      = { x: 0, y: 0, z: 0 };
        this.heading       = 0;  // radians
        this.speed         = 0;
        this.battery       = 1.0;
        this.currentAction = AgentAction.HOVER;
        this.currentWaypoint = null;

        // ── Timers ──
        this._lastDeliberative = 0;
        this._lastExecutive    = 0;
        this._lastObservation  = null;

        // ── Training state ──
        this.trainingEnabled = config.trainingEnabled ?? true;
        this.totalSteps      = 0;
        this.episodeSteps    = 0;
        this.episodeReward   = 0;

        // ── Leader task: zone assignments from Raft ──
        this._zoneAssignments = new Map();  // droneId → zone bounds

        // ── Cooperative ratio (from autonomy slider) ──
        this._cooperativeRatio = config.cooperativeRatio ?? 0.5;

        // ── Configure Raft commit callback ──
        this.raft.onCommit = (cmd) => this._onRaftCommit(cmd);

        // ── Stats ──
        this.stats = {
            actionsChosen: new Map(),
            exploredCells: 0,
            consensusParticipations: 0,
            messagesReceived: 0,
            messagesSent: 0
        };
    }

    // ────────────────────────────────────────────────────────────────
    // PUBLIC API
    // ────────────────────────────────────────────────────────────────

    /**
     * Main per-frame tick. Called by MultiAgentCoordinator.
     *
     * @param {number} dt    - Delta time in seconds
     * @param {number} now   - Current time in ms
     * @param {Array}  obstacles - Tree/obstacle array [{center, radius, height}]
     * @param {Map}    peerStates - Map<droneId, {position, heading, speed}>
     * @param {Object} scene - THREE.Scene (optional, for raycasting)
     * @returns {Object|null} - { waypoint: {x,y,z} } or null if no update needed
     */
    tick(dt, now, obstacles, peerStates, scene = null) {
        if (this.phase === AgentPhase.IDLE || this.phase === AgentPhase.LANDED) {
            return null;
        }

        // ── Layer 1: Reactive (every tick) ──
        const reactiveResult = this._reactiveLayer(dt, now, obstacles, peerStates, scene);

        // ── Layer 2: Executive (10 Hz) ──
        let executiveResult = null;
        if (now - this._lastExecutive >= EXECUTIVE_PERIOD_MS) {
            this._lastExecutive = now;
            executiveResult = this._executiveLayer(dt, now, obstacles, peerStates);
        }

        // ── Layer 3: Deliberative (2 Hz) ──
        if (now - this._lastDeliberative >= DELIBERATIVE_PERIOD_MS) {
            this._lastDeliberative = now;
            this._deliberativeLayer(dt, now, peerStates);
        }

        // Return waypoint (executive takes priority over reactive avoidance)
        if (executiveResult && executiveResult.waypoint) {
            this.currentWaypoint = executiveResult.waypoint;
        }

        return this.currentWaypoint ? { waypoint: this.currentWaypoint } : null;
    }

    /**
     * Update agent's known position (called by coordinator from engine state).
     */
    setPosition(x, y, z) {
        this.position = { x, y, z };
    }

    /**
     * Update heading and speed.
     */
    setMotion(heading, speed) {
        this.heading = heading;
        this.speed = speed;
    }

    /**
     * Set battery level (0..1).
     */
    setBattery(level) {
        this.battery = Math.max(0, Math.min(1, level));
    }

    /**
     * Transition the agent's phase.
     */
    setPhase(phase) {
        this.phase = phase;
    }

    /**
     * Set cooperative/individual ratio (from autonomy slider).
     * @param {number} ratio - 0 = full individual, 1 = full cooperative
     */
    setCooperativeRatio(ratio) {
        this._cooperativeRatio = Math.max(0, Math.min(1, ratio));
        this.rewardShaper.setCooperativeRatio(this._cooperativeRatio);
    }

    /**
     * Enable or disable learning.
     */
    setTrainingEnabled(enabled) {
        this.trainingEnabled = enabled;
    }

    /**
     * Register a peer drone.
     */
    addPeer(peerId) {
        this.raft.addPeer(peerId);
    }

    /**
     * Remove a peer drone.
     */
    removePeer(peerId) {
        this.raft.removePeer(peerId);
    }

    /**
     * Hot-swap the agent's brain (MLOps: brain injection).
     * The new brain must implement BrainInterface.
     * @param {BrainInterface} newBrain
     * @param {boolean} [transferWeights=false] - If true, export old weights and import into new brain
     */
    setBrain(newBrain, transferWeights = false) {
        if (!(newBrain instanceof BrainInterface)) {
            console.warn(`[Agent:${this.droneId}] setBrain: must be a BrainInterface instance`);
            return false;
        }

        const oldBrain = this.brain;
        const oldType = oldBrain.getType?.() || 'unknown';

        if (transferWeights) {
            try {
                const weights = oldBrain.exportWeights();
                newBrain.importWeights(weights);
            } catch (e) {
                console.warn(`[Agent:${this.droneId}] Weight transfer failed: ${e.message}`);
            }
        }

        this.brain = newBrain;
        console.log(`[Agent:${this.droneId}] Brain swapped: ${oldType} → ${newBrain.getType()}`);
        return true;
    }

    /**
     * Get the current brain type.
     * @returns {string}
     */
    getBrainType() {
        return this.brain.getType?.() || 'unknown';
    }

    /**
     * Deliver a message to this agent (from coordinator routing).
     * @param {Object} msg - { from, type, data, timestamp }
     * @param {number} now
     */
    receiveMessage(msg, now) {
        this.stats.messagesReceived++;

        // Route Raft messages
        if (msg.data && msg.data.raftType) {
            this.raft.handleMessage(msg, now);
            return;
        }

        // Route to communicator
        this.communicator.processInbox([msg]);
    }

    /**
     * Drain outgoing messages (to be routed by coordinator).
     * @returns {Array<Object>}
     */
    drainMessages(now) {
        const commMsgs = this.communicator.drainOutbox(now);
        const raftMsgs = this.raft.tick(now);

        // Convert raft outbox to standard message format
        const raftFormatted = raftMsgs.map(m => ({
            from: this.droneId,
            to: m.to,
            type: 'raft',
            data: m.data,
            timestamp: now
        }));

        this.stats.messagesSent += commMsgs.length + raftFormatted.length;
        return [...commMsgs, ...raftFormatted];
    }

    /**
     * Export the agent's learned weights (brain only, lightweight).
     */
    exportWeights() {
        return this.brain.exportWeights();
    }

    /**
     * Import previously learned weights.
     */
    importWeights(data) {
        this.brain.importWeights(data);
    }

    /**
     * Export complete state (brain + perception + stats).
     */
    exportFull() {
        return {
            droneId: this.droneId,
            brain: this.brain.exportFull(),
            perception: this.perception.grid.exportSparse(),
            stats: {
                totalSteps: this.totalSteps,
                episodeReward: this.episodeReward,
                actionsChosen: Object.fromEntries(this.stats.actionsChosen)
            }
        };
    }

    /**
     * Reset for new episode (training).
     */
    resetEpisode() {
        this.episodeSteps = 0;
        this.episodeReward = 0;
        this.perception.grid.grid.fill(0);  // Reset occupancy grid
        this.communicator.reset();
        this.rewardShaper.resetEpisode();
        this.currentAction = AgentAction.HOVER;
        this.currentWaypoint = null;
        this._lastObservation = null;
    }

    /**
     * Get agent metrics for UI display.
     */
    getMetrics() {
        return {
            droneId: this.droneId,
            phase: this.phase,
            action: this.currentAction,
            totalSteps: this.totalSteps,
            episodeReward: this.episodeReward.toFixed(2),
            epsilon: this.brain.epsilon.toFixed(3),
            raftRole: this.raft.getRoleString(),
            raftTerm: this.raft.currentTerm,
            coverage: (this.perception.grid.localCoverage() * 100).toFixed(1) + '%',
            peers: this.communicator.getActivePeerCount(),
            brainSize: this.brain.getTotalParams()
        };
    }

    // ────────────────────────────────────────────────────────────────
    // LAYER 1: REACTIVE (60 Hz)
    // ────────────────────────────────────────────────────────────────

    _reactiveLayer(dt, now, obstacles, peerStates, scene) {
        // Update perception continuously
        const droneState = {
            position: this.position,
            velocity: { x: Math.sin(this.heading) * this.speed,
                        y: 0,
                        z: -Math.cos(this.heading) * this.speed },
            heading: this.heading,
            battery: this.battery,
            phaseTime: this.episodeSteps * 0.1
        };

        // Build otherDrones array from peerStates
        const otherDrones = [];
        if (peerStates) {
            for (const [peerId, peer] of peerStates) {
                if (peerId === this.droneId) continue;
                otherDrones.push({
                    id: peerId,
                    position: peer.position,
                    velocity: { x: 0, y: 0, z: 0 }
                });
            }
        }

        this.perception.update(
            droneState, obstacles || [], otherDrones,
            0, this.raft.getRoleString()
        );

        // Emergency collision avoidance (subsumption override)
        const avoid = this._checkEmergencyAvoidance(obstacles, peerStates);
        if (avoid) {
            this.currentWaypoint = avoid;
            return { waypoint: avoid };
        }

        return null;
    }

    // ────────────────────────────────────────────────────────────────
    // LAYER 2: EXECUTIVE (10 Hz)
    // ────────────────────────────────────────────────────────────────

    _executiveLayer(dt, now, obstacles, peerStates) {
        // 1. Build observation vector (COPY to avoid aliasing — perception overwrites buffer)
        const observation = new Float32Array(this.perception._obs);

        // 2. Choose action (Q-learning) — destructure to get numeric action
        const result = this.brain.chooseAction(observation);
        const action = result.action;   // number, not object
        const prevAction = this.currentAction;
        this.currentAction = action;

        // 3. Track action distribution
        this.stats.actionsChosen.set(action,
            (this.stats.actionsChosen.get(action) || 0) + 1);

        // 4. Compute reward for previous action
        if (this._lastObservation !== null) {
            const reward = this._computeReward(
                this._lastObservation, observation, prevAction, peerStates
            );
            this.episodeReward += reward;

            // 5. Learn from experience
            if (this.trainingEnabled) {
                this.brain.learn(
                    this._lastObservation,
                    prevAction,
                    reward,
                    observation,
                    false // not terminal
                );
            }
        }

        this._lastObservation = observation;
        this.totalSteps++;
        this.episodeSteps++;

        // 6. Translate action to waypoint
        const waypoint = this._actionToWaypoint(action, peerStates);

        // 7. Handle communication actions
        this._handleCommunicationActions(action, now);

        return { waypoint };
    }

    // ────────────────────────────────────────────────────────────────
    // LAYER 3: DELIBERATIVE (2 Hz)
    // ────────────────────────────────────────────────────────────────

    _deliberativeLayer(dt, now, peerStates) {
        // 1. Communicate heartbeat + map data
        this.communicator.queueHeartbeat(this.position, this.speed);

        // 2. Share map with peers periodically
        if (this.episodeSteps % 50 === 0) {
            const sparseMap = this.perception.grid.exportSparse();
            this.communicator.queueMapShare(sparseMap);
        }

        // 3. Frontier announcement
        const frontier = this.perception.grid.findNearestFrontier(
            this.position.x, this.position.z
        );
        if (frontier) {
            this.communicator.queueFrontierAnnouncement(
                frontier, this._estimateFrontierValue(frontier)
            );
        }

        // 4. Raft leader duties
        if (this.raft.isLeader()) {
            this._leaderAssignZones(peerStates);
        }

        // 5. Merge received map data
        this._mergeReceivedMaps();
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNALS
    // ────────────────────────────────────────────────────────────────

    /**
     * Convert a discrete action to a 3D waypoint.
     */
    _actionToWaypoint(action, peerStates) {
        const { x, y, z } = this.position;

        // Navigation actions
        const disp = ACTION_DISPLACEMENT[action];
        if (disp) {
            // Rotate displacement by heading
            const cos = Math.cos(this.heading);
            const sin = Math.sin(this.heading);
            const dx = disp.dx * cos - disp.dz * sin;
            const dz = disp.dx * sin + disp.dz * cos;
            return {
                x: x + dx * WAYPOINT_STEP,
                y: y,
                z: z + dz * WAYPOINT_STEP
            };
        }

        // GOTO_FRONTIER
        if (action === AgentAction.GOTO_FRONTIER) {
            const frontier = this.perception.grid.findNearestFrontier(x, z);
            if (frontier) {
                return { x: frontier.x, y, z: frontier.z };
            }
            // Fallback: random exploration
            const angle = Math.random() * Math.PI * 2;
            return {
                x: x + Math.cos(angle) * WAYPOINT_STEP * 2,
                y,
                z: z + Math.sin(angle) * WAYPOINT_STEP * 2
            };
        }

        // GOTO_NEIGHBOR — go toward closest peer for map merge
        if (action === AgentAction.GOTO_NEIGHBOR) {
            const closest = this._findClosestPeer(peerStates);
            if (closest) {
                const dx2 = closest.position.x - x;
                const dz2 = closest.position.z - z;
                const dist = Math.sqrt(dx2 * dx2 + dz2 * dz2);
                if (dist > 1) {
                    const approach = Math.min(WAYPOINT_STEP, dist * 0.5);
                    return {
                        x: x + (dx2 / dist) * approach,
                        y,
                        z: z + (dz2 / dist) * approach
                    };
                }
            }
            return { x, y, z }; // stay
        }

        // AVOID_NEIGHBOR — move away from closest peer
        if (action === AgentAction.AVOID_NEIGHBOR) {
            const closest = this._findClosestPeer(peerStates);
            if (closest) {
                const dx2 = x - closest.position.x;
                const dz2 = z - closest.position.z;
                const dist = Math.sqrt(dx2 * dx2 + dz2 * dz2);
                if (dist > 0.1) {
                    return {
                        x: x + (dx2 / dist) * WAYPOINT_STEP,
                        y,
                        z: z + (dz2 / dist) * WAYPOINT_STEP
                    };
                }
            }
            // Fallback: move forward
            return {
                x: x - Math.sin(this.heading) * WAYPOINT_STEP,
                y,
                z: z - Math.cos(this.heading) * WAYPOINT_STEP
            };
        }

        // SHARE_MAP, YIELD_TASK — no movement, handled separately
        return { x, y, z };
    }

    /**
     * Emergency reactive avoidance (subsumption override).
     */
    _checkEmergencyAvoidance(obstacles, peerStates) {
        const { x, y, z } = this.position;
        const EMERGENCY_DIST = 1.0;
        const PUSH_DIST = 2.5;
        let pushX = 0, pushZ = 0;
        let needPush = false;

        // Obstacle avoidance
        for (const obs of obstacles) {
            const dx = x - obs.center.x;
            const dz = z - obs.center.z;
            const dist = Math.sqrt(dx * dx + dz * dz);
            const minDist = (obs.radius || 0.5) + EMERGENCY_DIST;
            if (dist < minDist && dist > 0.01) {
                pushX += (dx / dist) * PUSH_DIST;
                pushZ += (dz / dist) * PUSH_DIST;
                needPush = true;
            }
        }

        // Drone-to-drone avoidance
        if (peerStates) {
            for (const [peerId, peer] of peerStates) {
                if (peerId === this.droneId) continue;
                const pos = peer.position || peer;
                const dx = x - pos.x;
                const dz = z - pos.z;
                const dist = Math.sqrt(dx * dx + dz * dz);
                if (dist < EMERGENCY_DIST && dist > 0.01) {
                    pushX += (dx / dist) * PUSH_DIST;
                    pushZ += (dz / dist) * PUSH_DIST;
                    needPush = true;
                }
            }
        }

        if (needPush) {
            return { x: x + pushX, y, z: z + pushZ };
        }
        return null;
    }

    /**
     * Compute reward for the transition (prevObs, action) → currentObs.
     */
    _computeReward(prevObs, currentObs, action, peerStates) {
        // Find nearest obstacle and drone distances from perception
        const nearestObstacle = Math.min(
            this.perception.ranges[0],
            this.perception.ranges[1],
            this.perception.ranges[2],
            this.perception.ranges[3]
        );

        let nearestDrone = Infinity;
        const interDroneDistances = [];
        const teamActions = [action]; // Start with this agent's action
        if (peerStates) {
            for (const [peerId, peer] of peerStates) {
                if (peerId === this.droneId) continue;
                const pos = peer.position || peer;
                const dx = this.position.x - pos.x;
                const dz = this.position.z - pos.z;
                const dist = Math.sqrt(dx * dx + dz * dz);
                interDroneDistances.push(dist);
                if (dist < nearestDrone) nearestDrone = dist;
                // Collect peer actions if available
                if (peer.currentAction !== undefined) teamActions.push(peer.currentAction);
            }
        }

        const prevCoverage = prevObs ? prevObs[15] || 0 : 0;  // LOCAL_COVERAGE
        const currCoverage = currentObs[15] || 0;
        const newCells = Math.max(0, (currCoverage - prevCoverage) * 100);

        const result = this.rewardShaper.computeReward(this.droneId, {
            action,
            position: this.position,
            velocity: {
                x: Math.sin(this.heading) * this.speed,
                z: -Math.cos(this.heading) * this.speed
            },
            battery: this.battery,
            localCoverage: currCoverage,
            globalCoverage: currentObs[20] || 0,  // SWARM_COVERAGE
            nearestObstacle,
            nearestDrone,
            frontierDist: (currentObs[14] || 1.0) * 40,  // FRONTIER_DIST denormalized
            newCellsThisStep: newCells,
            overlapCells: 0,
            collision: false,
            droneCollision: nearestDrone < 0.5,
            outOfBounds: Math.abs(this.position.x) > 35 || Math.abs(this.position.z) > 35,
            sharedMap: action === AgentAction.SHARE_MAP,
            participatedConsensus: this.raft.isLeader() || this.raft.votedFor !== null,
            teamActions,
            interDroneDistances
        });

        return result.total;
    }

    /**
     * Handle communication-specific actions.
     */
    _handleCommunicationActions(action, now) {
        if (action === AgentAction.SHARE_MAP) {
            const sparseMap = this.perception.grid.exportSparse();
            this.communicator.queueMapShare(sparseMap);
        }

        if (action === AgentAction.YIELD_TASK) {
            // Yield current zone to peer
            const frontier = this.perception.grid.findNearestFrontier(
                this.position.x, this.position.z
            );
            if (frontier) {
                this.communicator.queueYield(frontier, this.droneId);
            }
        }
    }

    /**
     * Find the closest peer from the states map.
     */
    _findClosestPeer(peerStates) {
        if (!peerStates || peerStates.size === 0) return null;

        let closest = null;
        let minDist = Infinity;

        for (const [peerId, peer] of peerStates) {
            if (peerId === this.droneId) continue;
            const pos = peer.position || peer;
            const dx = this.position.x - pos.x;
            const dz = this.position.z - pos.z;
            const dist = dx * dx + dz * dz;
            if (dist < minDist) {
                minDist = dist;
                closest = { id: peerId, position: pos };
            }
        }
        return closest;
    }

    /**
     * Estimate value of a frontier cell for CBBA bidding.
     */
    _estimateFrontierValue(frontier) {
        const dx = frontier.x - this.position.x;
        const dz = frontier.z - this.position.z;
        const dist = Math.sqrt(dx * dx + dz * dz);
        // Closer frontiers are more valuable (CBBA-compatible bid)
        return Math.max(0, 100 - dist * 2);
    }

    /**
     * Raft leader: assign zones to drones (simple Voronoi partition).
     */
    _leaderAssignZones(peerStates) {
        if (!this.raft.isLeader()) return;

        const allDrones = [this.droneId, ...this.raft.peers];
        const n = allDrones.length;
        if (n <= 1) return;

        // Simple stripe partitioning (can be upgraded to Voronoi)
        const bounds = { minX: -30, maxX: 30, minZ: -30, maxZ: 30 };
        const stripeWidth = (bounds.maxX - bounds.minX) / n;

        for (let i = 0; i < n; i++) {
            const zoneCmd = {
                type: RaftCommand.ZONE_ASSIGN,
                droneId: allDrones[i],
                payload: {
                    minX: bounds.minX + i * stripeWidth,
                    maxX: bounds.minX + (i + 1) * stripeWidth,
                    minZ: bounds.minZ,
                    maxZ: bounds.maxZ
                }
            };

            // Only propose if zone changed
            const existing = this._zoneAssignments.get(allDrones[i]);
            if (!existing || existing.minX !== zoneCmd.payload.minX) {
                this.raft.propose(zoneCmd);
            }
        }
    }

    /**
     * Handle committed Raft commands.
     */
    _onRaftCommit(command) {
        switch (command.type) {
            case RaftCommand.ZONE_ASSIGN:
                this._zoneAssignments.set(command.droneId, command.payload);
                break;

            case RaftCommand.PARAM_UPDATE:
                // Apply parameter changes
                if (command.payload.cooperativeRatio !== undefined) {
                    this.setCooperativeRatio(command.payload.cooperativeRatio);
                }
                break;

            case RaftCommand.EMERGENCY:
                this.phase = AgentPhase.RETURNING;
                break;

            case RaftCommand.MISSION_ORDER:
                // Could set mission-specific behaviors
                break;
        }
    }

    /**
     * Merge maps received from peers.
     */
    _mergeReceivedMaps() {
        const received = this.communicator.getReceivedMaps();
        for (const mapData of received) {
            if (mapData && mapData.cells) {
                this.perception.grid.importSparse(mapData);
            }
        }
    }
}
