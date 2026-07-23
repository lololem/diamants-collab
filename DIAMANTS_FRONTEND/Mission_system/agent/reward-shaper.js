/**
 * reward-shaper.js — Multi-agent reward functions for trainable drone agents.
 *
 * Computes a reward signal that combines:
 *   1. Individual reward (per-drone: exploration, safety, efficiency)
 *   2. Cooperative reward (team: coverage, non-overlap, coordination)
 *   3. Communication reward (sharing useful information)
 *
 * The balance between individual and cooperative reward is configurable,
 * and can be adjusted via the autonomy slider.
 *
 * Compatible with the Python training pipeline reward structure:
 *   - Same reward scale (~[-50, +50] per step)
 *   - Same component names for logging
 *
 * @module reward-shaper
 */

import { AgentAction } from './agent-brain.js';

// ─── REWARD WEIGHTS (tunable hyperparameters) ────────────────────────

export const DEFAULT_REWARD_WEIGHTS = Object.freeze({
    // ── Individual (per-drone) ──
    exploration:     1.0,    // Reward for visiting new cells
    frontierProgress: 0.5,   // Getting closer to nearest frontier
    safety:          2.0,    // Penalty for near-collisions
    efficiency:      0.3,    // Penalty for wasted movement (oscillation)
    batteryConserve: 0.2,    // Penalty for low battery far from home

    // ── Cooperative (team-level, shared) ──
    teamCoverage:    1.5,    // Global coverage improvement
    nonOverlap:      1.0,    // Exploring unique areas (not what others explored)
    separation:      0.8,    // Maintaining good inter-drone distance
    taskComplete:    5.0,    // Completing assigned tasks

    // ── Communication ──
    usefulShare:     0.5,    // Sharing map data that helps others
    consensusParticipation: 0.3, // Participating in consensus rounds

    // ── Emergence (synergy bonuses for collective behaviors) ──
    roleSpecialization: 1.2,    // Reward for consistently filling a unique role
    spontaneousFormation: 0.8,  // Reward when agents form efficient patterns
    synergyBonus:        1.5,   // Reward when team coverage > sum of individuals
    adaptiveBehavior:    0.6,   // Reward for changing strategy based on context

    // ── Penalties ──
    collision:      -50.0,   // Hitting obstacle
    droneCollision: -30.0,   // Hitting another drone
    outOfBounds:    -10.0,   // Leaving the zone
    idlePenalty:    -0.1,    // Hovering without reason
    timePenalty:    -0.005   // Per-step time cost
});

// ─── REWARD SHAPER ───────────────────────────────────────────────────

export class RewardShaper {
    /**
     * @param {Object} config
     * @param {Object} config.weights - Override default weights
     * @param {number} config.cooperativeRatio - 0..1, how much cooperative vs individual (default 0.5)
     * @param {number} config.zoneSize - Half-zone in meters
     */
    constructor(config = {}) {
        this.weights = { ...DEFAULT_REWARD_WEIGHTS, ...(config.weights || {}) };
        this.cooperativeRatio = config.cooperativeRatio ?? 0.5;
        this.zoneSize = config.zoneSize ?? 40;

        // Per-agent tracking
        /** @type {Map<string, AgentRewardState>} */
        this.agentStates = new Map();
    }

    /**
     * Initialize tracking for a new agent.
     * @param {string} agentId
     */
    registerAgent(agentId) {
        this.agentStates.set(agentId, {
            prevPosition: null,
            prevCoverage: 0,
            prevFrontierDist: Infinity,
            prevGlobalCoverage: 0,
            cellsExploredAlone: 0,
            cellsOverlap: 0,
            cumulativeReward: 0,
            stepCount: 0,
            rewardComponents: {}
        });
    }

    /**
     * Set cooperative ratio (called when autonomy slider changes).
     * 0 = purely individual reward (each drone optimizes alone)
     * 1 = purely cooperative reward (team reward shared equally)
     * @param {number} ratio
     */
    setCooperativeRatio(ratio) {
        this.cooperativeRatio = Math.max(0, Math.min(1, ratio));
    }

    /**
     * Compute the total reward for an agent at this timestep.
     *
     * @param {string} agentId
     * @param {Object} params
     * @param {number} params.action - Action taken (AgentAction enum)
     * @param {Object} params.position - { x, y, z }
     * @param {Object} params.velocity - { x, z }
     * @param {number} params.battery - [0, 1]
     * @param {number} params.localCoverage - [0, 1] local area coverage
     * @param {number} params.globalCoverage - [0, 1] team coverage
     * @param {number} params.nearestObstacle - Distance to nearest obstacle (from perception)
     * @param {number} params.nearestDrone - Distance to nearest other drone
     * @param {number} params.frontierDist - Distance to nearest frontier
     * @param {number} params.newCellsThisStep - Number of new cells explored this step
     * @param {number} params.overlapCells - Cells explored that others already explored
     * @param {boolean} params.collision - Hit an obstacle?
     * @param {boolean} params.droneCollision - Hit another drone?
     * @param {boolean} params.outOfBounds - Left the zone?
     * @param {boolean} params.sharedMap - Shared map data this step?
     * @param {boolean} params.participatedConsensus - Participated in consensus?
     * @returns {{ total: number, components: Object, done: boolean }}
     */
    computeReward(agentId, params) {
        const w = this.weights;
        const state = this.agentStates.get(agentId);
        if (!state) {
            this.registerAgent(agentId);
            return this.computeReward(agentId, params);
        }

        const components = {};
        let individual = 0;
        let cooperative = 0;
        let done = false;

        // ════════════════════════════════════════════════════
        // INDIVIDUAL REWARD (per-drone)
        // ════════════════════════════════════════════════════

        // 1. Exploration: reward for visiting new cells
        if (params.newCellsThisStep > 0) {
            const exploreR = Math.min(3.0, params.newCellsThisStep * 0.5) * w.exploration;
            components.exploration = exploreR;
            individual += exploreR;
        }

        // 2. Frontier progress: getting closer to unexplored areas
        if (params.frontierDist < Infinity && state.prevFrontierDist < Infinity) {
            const progress = state.prevFrontierDist - params.frontierDist;
            if (progress > 0.5) {
                const frontierR = Math.min(1.0, progress * 0.3) * w.frontierProgress;
                components.frontierProgress = frontierR;
                individual += frontierR;
            }
        }
        state.prevFrontierDist = params.frontierDist;

        // 3. Safety: penalty for being too close to obstacles
        if (params.nearestObstacle < 2.0) {
            const dangerR = -(2.0 - params.nearestObstacle) * w.safety;
            components.safety = dangerR;
            individual += dangerR;
        }

        // 4. Efficiency: penalize oscillation (low speed means stuck)
        const speed = Math.sqrt(
            (params.velocity?.x ?? 0) ** 2 + (params.velocity?.z ?? 0) ** 2
        );
        if (speed < 0.3 && params.action !== AgentAction.HOVER && params.action !== AgentAction.SHARE_MAP) {
            components.efficiency = -0.2 * w.efficiency;
            individual += -0.2 * w.efficiency;
        }

        // 5. Battery conservation
        if (params.battery < 0.2) {
            const battR = -(0.2 - params.battery) * 5 * w.batteryConserve;
            components.batteryPenalty = battR;
            individual += battR;
        }

        // ════════════════════════════════════════════════════
        // COOPERATIVE REWARD (team-level)
        // ════════════════════════════════════════════════════

        // 6. Team coverage improvement
        if (params.globalCoverage > state.prevGlobalCoverage) {
            const coverageGain = (params.globalCoverage - state.prevGlobalCoverage) * 100;
            const teamR = Math.min(3.0, coverageGain * 2) * w.teamCoverage;
            components.teamCoverage = teamR;
            cooperative += teamR;
        }
        state.prevGlobalCoverage = params.globalCoverage;

        // 7. Non-overlap: reward for exploring unique areas
        if (params.newCellsThisStep > 0) {
            const uniqueRatio = params.overlapCells > 0 ?
                1.0 - (params.overlapCells / (params.newCellsThisStep + params.overlapCells)) : 1.0;
            const nonOverlapR = uniqueRatio * 0.5 * w.nonOverlap;
            components.nonOverlap = nonOverlapR;
            cooperative += nonOverlapR;
        }

        // 8. Inter-drone separation: not too close, not too far
        if (params.nearestDrone < 3.0) {
            // Too close — collision risk
            const sepR = -(3.0 - params.nearestDrone) * w.separation;
            components.separation = sepR;
            cooperative += sepR;
        } else if (params.nearestDrone > 5.0 && params.nearestDrone < 30.0) {
            // Good distance — mild reward
            components.separation = 0.1 * w.separation;
            cooperative += 0.1 * w.separation;
        }

        // 9. Communication reward
        if (params.sharedMap) {
            components.usefulShare = w.usefulShare;
            cooperative += w.usefulShare;
        }
        if (params.participatedConsensus) {
            components.consensusParticipation = w.consensusParticipation;
            cooperative += w.consensusParticipation;
        }

        // ════════════════════════════════════════════════════
        // TERMINAL PENALTIES
        // ════════════════════════════════════════════════════

        if (params.collision) {
            components.collision = w.collision;
            individual += w.collision;
            done = true;
        }
        if (params.droneCollision) {
            components.droneCollision = w.droneCollision;
            cooperative += w.droneCollision;
        }
        if (params.outOfBounds) {
            components.outOfBounds = w.outOfBounds;
            individual += w.outOfBounds;
        }

        // Idle penalty
        if (params.action === AgentAction.HOVER) {
            components.idlePenalty = w.idlePenalty;
            individual += w.idlePenalty;
        }

        // Time penalty
        components.timePenalty = w.timePenalty;
        individual += w.timePenalty;

        // ════════════════════════════════════════════════════
        // EMERGENCE REWARD (behaviors NOT explicitly programmed)
        // These rewards detect and reinforce spontaneous collective
        // patterns that transcend individual agent rules.
        // ════════════════════════════════════════════════════
        let emergence = 0;

        // 10. Role specialization: reward agents that consistently
        //     pick a unique action profile different from peers.
        //     Measured by action diversity across the team.
        if (params.teamActions && params.teamActions.length >= 2) {
            const uniqueActions = new Set(params.teamActions);
            const diversityRatio = uniqueActions.size / params.teamActions.length;
            if (diversityRatio > 0.6) {
                // Agents are doing different things → specialization
                const roleR = (diversityRatio - 0.5) * 2 * w.roleSpecialization;
                components.roleSpecialization = roleR;
                emergence += roleR;
            }
        }

        // 11. Spontaneous formation: reward when agents self-organize
        //     into an efficient spatial pattern (even spacing).
        //     Coefficient of variation of inter-drone distances → low = formation.
        if (params.interDroneDistances && params.interDroneDistances.length >= 2) {
            const dists = params.interDroneDistances;
            const mean = dists.reduce((a, b) => a + b, 0) / dists.length;
            if (mean > 1) {
                const variance = dists.reduce((s, d) => s + (d - mean) ** 2, 0) / dists.length;
                const cv = Math.sqrt(variance) / mean;
                // Low CV (< 0.3) = very even spacing = emergent formation
                if (cv < 0.5) {
                    const formationR = (0.5 - cv) * 2 * w.spontaneousFormation;
                    components.spontaneousFormation = formationR;
                    emergence += formationR;
                }
            }
        }

        // 12. Synergy bonus: reward when global coverage gain exceeds
        //     what independent agents would achieve (superlinear scaling).
        if (params.globalCoverage > 0 && params.newCellsThisStep !== undefined) {
            const N = params.teamActions?.length || 4;
            // Expected independent rate: each agent covers ~1 cell per step
            const expectedIndependentGain = 1.0 / (this.zoneSize * this.zoneSize);
            const actualGain = params.globalCoverage - (state.prevGlobalCoverage || 0);
            if (actualGain > expectedIndependentGain * 1.5) {
                // Team gained >50% more than independent expectation → synergy!
                const synergyR = Math.min(2.0, (actualGain / expectedIndependentGain - 1) * w.synergyBonus);
                components.synergyBonus = synergyR;
                emergence += synergyR;
            }
        }

        // 13. Adaptive behavior: reward for changing action type based
        //     on context (not repeating same action endlessly).
        if (!state._recentActions) state._recentActions = [];
        state._recentActions.push(params.action);
        if (state._recentActions.length > 10) state._recentActions.shift();
        if (state._recentActions.length >= 5) {
            const recent5 = state._recentActions.slice(-5);
            const uniqueRecent = new Set(recent5).size;
            if (uniqueRecent >= 3) {
                // Using at least 3 different actions in last 5 steps → adaptive
                const adaptR = (uniqueRecent / 5) * 0.3 * w.adaptiveBehavior;
                components.adaptiveBehavior = adaptR;
                emergence += adaptR;
            }
        }

        // ════════════════════════════════════════════════════
        // COMBINE
        // ════════════════════════════════════════════════════

        const alpha = this.cooperativeRatio;
        // Emergence reward is always fully included (not scaled by alpha)
        // because it rewards behaviors we want to see regardless of autonomy level
        const total = (1 - alpha) * individual + alpha * cooperative + emergence;

        // Update tracking
        state.cumulativeReward += total;
        state.stepCount++;
        state.rewardComponents = components;
        state.prevPosition = { ...params.position };

        return { total, components, done };
    }

    /**
     * Get cumulative stats for an agent.
     * @param {string} agentId
     * @returns {Object}
     */
    getAgentStats(agentId) {
        const state = this.agentStates.get(agentId);
        if (!state) return null;
        return {
            cumulativeReward: Math.round(state.cumulativeReward * 100) / 100,
            steps: state.stepCount,
            avgReward: state.stepCount > 0 ?
                Math.round((state.cumulativeReward / state.stepCount) * 1000) / 1000 : 0,
            lastComponents: state.rewardComponents
        };
    }

    /**
     * Reset episode tracking for all agents.
     */
    resetEpisode() {
        for (const [_, state] of this.agentStates) {
            state.prevPosition = null;
            state.prevCoverage = 0;
            state.prevFrontierDist = Infinity;
            state.prevGlobalCoverage = 0;
            state.cumulativeReward = 0;
            state.stepCount = 0;
        }
    }
}
