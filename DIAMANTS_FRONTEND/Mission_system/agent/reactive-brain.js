/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * reactive-brain.js — Rule-based reactive brain for lightweight drones (Crazyflie).
 *
 * Implements BrainInterface with deterministic heuristics instead of Q-learning.
 * Designed for small drones with limited sensors (Multi-Ranger: 4 ToF beams).
 *
 * Behavior hierarchy (subsumption-like):
 *   1. EMERGENCY AVOIDANCE — Obstacle too close → retreat/turn
 *   2. WALL FOLLOWING      — Obstacle nearby → wall-follow for structured coverage
 *   3. FRONTIER SEEKING    — Go toward nearest unexplored frontier
 *   4. SWARM SEPARATION    — Avoid overlapping with neighbors
 *   5. DEFAULT EXPLORE     — Forward exploration with random perturbation
 *
 * Zero learnable parameters — deterministic, fast, predictable.
 * Can still contribute to team reward in MARL (shared reward channel).
 *
 * @module reactive-brain
 */

import { BrainInterface } from './brain-interface.js';
import { AgentAction, OBS_DIM, ObsIndex } from './agent-brain.js';

// ─── THRESHOLDS ──────────────────────────────────────────────────────

const EMERGENCY_DIST    = 0.15;   // obs value (normalized) — very close obstacle
const WALL_FOLLOW_DIST  = 0.40;   // obs value — wall-following band
const FRONTIER_CLOSE    = 0.85;   // obs value — frontier is "close enough" to seek
const SEPARATION_DIST   = 0.25;   // normalized neighbor distance — too close
const LOW_BATTERY       = 0.20;   // return home below this

// ─── REACTIVE BRAIN ──────────────────────────────────────────────────

export class ReactiveBrain extends BrainInterface {
    /**
     * @param {string} agentId
     * @param {Object} [config]
     * @param {number} [config.wallFollowSide='left'] - 'left' or 'right'
     * @param {number} [config.jitterProb=0.1] - Probability of random action
     */
    constructor(agentId, config = {}) {
        super(agentId, 'reactive');

        this.wallFollowSide = config.wallFollowSide || 'left';
        this.jitterProb     = config.jitterProb ?? 0.1;

        // ── Internal state ──
        this._lastAction    = AgentAction.EXPLORE_FORWARD;
        this._wallFollowing = false;
        this._wallSide      = null;      // 'left' | 'right' | 'front'
        this._turnCount     = 0;
        this._stuckCounter  = 0;
        this._totalSteps    = 0;

        // ── Stats ──
        this._actionCounts  = new Array(AgentAction.NUM_ACTIONS).fill(0);
        this._emergencies   = 0;
        this._wallFollows   = 0;

        this.trainingEnabled = false;  // Reactive brains don't learn
    }

    // ────────────────────────────────────────────────────────────────
    // BrainInterface IMPLEMENTATION
    // ────────────────────────────────────────────────────────────────

    /**
     * Select action based on deterministic rules.
     * Observation uses the same 24-dim vector as AgentBrain (OBS_DIM).
     *
     * @param {Float32Array|number[]} obs - 24-dim normalized observation
     * @param {Float32Array} [actionMask] - Optional mask (unused for rules)
     * @returns {{ action: number, qValues: null, explore: boolean, rule: string }}
     */
    chooseAction(obs, actionMask) {
        this._totalSteps++;

        if (!obs || obs.length < OBS_DIM) {
            return { action: AgentAction.HOVER, qValues: null, explore: false, rule: 'invalid-obs' };
        }

        const front  = obs[ObsIndex.OBSTACLE_FRONT];   // 0=close, 1=far
        const left   = obs[ObsIndex.OBSTACLE_LEFT];
        const right  = obs[ObsIndex.OBSTACLE_RIGHT];
        const back   = obs[ObsIndex.OBSTACLE_BACK];

        const frontierSin  = obs[ObsIndex.FRONTIER_DIR_SIN];
        const frontierCos  = obs[ObsIndex.FRONTIER_DIR_COS];
        const frontierDist = obs[ObsIndex.FRONTIER_DIST];
        const localCov     = obs[ObsIndex.LOCAL_COVERAGE];

        const neighborCount = obs[ObsIndex.NEIGHBOR_COUNT];
        const nearestDist   = obs[ObsIndex.NEAREST_DIST];
        const nearestSin    = obs[ObsIndex.NEAREST_DIR_SIN];
        const battery       = obs[ObsIndex.BATTERY_NORM];

        let action;
        let rule;

        // ── Priority 0: Low battery → return ──
        if (battery < LOW_BATTERY) {
            action = AgentAction.RETREAT;
            rule = 'low-battery-return';
        }

        // ── Priority 1: Emergency avoidance ──
        else if (front < EMERGENCY_DIST) {
            this._emergencies++;
            // Front blocked — turn toward most open side
            if (left > right) {
                action = AgentAction.EXPLORE_LEFT;
            } else if (right > left) {
                action = AgentAction.EXPLORE_RIGHT;
            } else {
                action = AgentAction.RETREAT;
            }
            rule = 'emergency-avoid';
        }

        // ── Priority 2: Wall following (structured coverage) ──
        else if (front < WALL_FOLLOW_DIST || left < WALL_FOLLOW_DIST || right < WALL_FOLLOW_DIST) {
            this._wallFollows++;
            const result = this._wallFollowAction(front, left, right, back);
            action = result.action;
            rule = result.rule;
        }

        // ── Priority 3: Neighbor separation ──
        else if (neighborCount > 0 && nearestDist < SEPARATION_DIST) {
            // Determine if neighbor is left or right
            const neighborAngle = Math.atan2(nearestSin, obs[ObsIndex.NEAREST_DIR_COS]);
            if (Math.abs(neighborAngle) < Math.PI / 4) {
                // Neighbor ahead → avoid
                action = AgentAction.AVOID_NEIGHBOR;
            } else if (neighborAngle > 0) {
                // Neighbor on left → go right
                action = AgentAction.EXPLORE_RIGHT;
            } else {
                // Neighbor on right → go left
                action = AgentAction.EXPLORE_LEFT;
            }
            rule = 'separation';
        }

        // ── Priority 4: Frontier seeking ──
        else if (frontierDist < FRONTIER_CLOSE && localCov < 0.9) {
            // Orient toward frontier
            const frontierAngle = Math.atan2(frontierSin, frontierCos);

            if (Math.abs(frontierAngle) < Math.PI / 6) {
                action = AgentAction.EXPLORE_FORWARD;
            } else if (frontierAngle > 0) {
                action = AgentAction.EXPLORE_LEFT;
            } else {
                action = AgentAction.EXPLORE_RIGHT;
            }
            rule = 'frontier-seek';
        }

        // ── Priority 5: Default explore (with jitter) ──
        else {
            if (Math.random() < this.jitterProb) {
                // Random perturbation for coverage diversity
                const choices = [
                    AgentAction.EXPLORE_FORWARD,
                    AgentAction.EXPLORE_LEFT,
                    AgentAction.EXPLORE_RIGHT,
                    AgentAction.GOTO_FRONTIER
                ];
                action = choices[Math.floor(Math.random() * choices.length)];
                rule = 'explore-jitter';
            } else {
                action = AgentAction.EXPLORE_FORWARD;
                rule = 'explore-forward';
            }
        }

        // Apply action mask if provided
        if (actionMask && !actionMask[action]) {
            // Fall back to first allowed action
            for (let i = 0; i < AgentAction.NUM_ACTIONS; i++) {
                if (actionMask[i]) { action = i; break; }
            }
        }

        // Stuck detection: same action too many times → force random
        if (action === this._lastAction) {
            this._stuckCounter++;
            if (this._stuckCounter > 30) {
                const alt = [AgentAction.EXPLORE_LEFT, AgentAction.EXPLORE_RIGHT, AgentAction.GOTO_FRONTIER];
                action = alt[Math.floor(Math.random() * alt.length)];
                rule = 'anti-stuck';
                this._stuckCounter = 0;
            }
        } else {
            this._stuckCounter = 0;
        }

        this._lastAction = action;
        this._actionCounts[action]++;

        return { action, qValues: null, explore: false, rule };
    }

    /**
     * No-op: reactive brains don't learn from transitions.
     * BUT: still tracks reward for MARL team-level metrics.
     */
    learn(obs, action, reward, nextObs, done) {
        // No weight updates — this brain is deterministic
        // Could log team reward for MARL analysis
        this._lastReward = reward;
    }

    /**
     * Export configuration (no weights to export).
     */
    exportWeights() {
        return {
            version: 1,
            brainType: 'reactive',
            agentId: this.agentId,
            timestamp: Date.now(),
            config: {
                wallFollowSide: this.wallFollowSide,
                jitterProb: this.jitterProb
            },
            stats: this.getStats()
        };
    }

    /**
     * Import configuration.
     */
    importWeights(data) {
        if (data?.config) {
            this.wallFollowSide = data.config.wallFollowSide || this.wallFollowSide;
            this.jitterProb = data.config.jitterProb ?? this.jitterProb;
        }
        return true;
    }

    /**
     * Get statistics.
     */
    getStats() {
        return {
            ...super.getStats(),
            totalSteps: this._totalSteps,
            emergencies: this._emergencies,
            wallFollows: this._wallFollows,
            actionDistribution: Object.fromEntries(
                this._actionCounts.map((c, i) => [i, c])
            ),
            stuckResets: Math.floor(this._totalSteps / 30) // approximate
        };
    }

    getTotalParams() {
        return 0;  // No learnable parameters
    }

    reset() {
        this._lastAction = AgentAction.EXPLORE_FORWARD;
        this._wallFollowing = false;
        this._wallSide = null;
        this._turnCount = 0;
        this._stuckCounter = 0;
        this._actionCounts.fill(0);
        this._emergencies = 0;
        this._wallFollows = 0;
    }

    // ────────────────────────────────────────────────────────────────
    // WALL FOLLOWING LOGIC
    // ────────────────────────────────────────────────────────────────

    /**
     * Bug algorithm: follow wall contour for systematic coverage.
     * @private
     */
    _wallFollowAction(front, left, right, back) {
        // Determine which side has the wall
        if (front < WALL_FOLLOW_DIST) {
            // Wall ahead — turn to preferred side
            if (this.wallFollowSide === 'left') {
                return { action: AgentAction.EXPLORE_LEFT, rule: 'wall-turn-left' };
            } else {
                return { action: AgentAction.EXPLORE_RIGHT, rule: 'wall-turn-right' };
            }
        }

        if (this.wallFollowSide === 'left') {
            // Keep wall on left: if left wall disappears, turn left to find it
            if (left >= WALL_FOLLOW_DIST) {
                return { action: AgentAction.EXPLORE_LEFT, rule: 'wall-seek-left' };
            }
            // Wall on left, clear ahead → go forward
            return { action: AgentAction.EXPLORE_FORWARD, rule: 'wall-follow-left' };
        } else {
            // Keep wall on right
            if (right >= WALL_FOLLOW_DIST) {
                return { action: AgentAction.EXPLORE_RIGHT, rule: 'wall-seek-right' };
            }
            return { action: AgentAction.EXPLORE_FORWARD, rule: 'wall-follow-right' };
        }
    }
}

export default ReactiveBrain;
