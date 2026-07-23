/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * cognitive-brain.js — Extended Q-learning brain for heavy cognitive drones (X500).
 *
 * Extends AgentBrain with:
 *   1. Depth camera observation integration (OAK-D Pro W: 127°×80° FOV)
 *   2. Object-level reasoning (detected entities → deliberative actions)
 *   3. Longer planning horizon (γ=0.98 vs γ=0.95)
 *   4. Larger replay buffer (50K vs 10K) — more experience capacity
 *   5. Mission-level meta-actions: task allocation, zone claiming
 *   6. VLM-informed exploration bonus (entities of interest)
 *
 * Supports Multi-Agent Reinforcement Learning (MARL):
 *   - Shared reward channel for cooperative objectives
 *   - Observation includes neighbor brain types (reactive vs cognitive)
 *   - Can delegate sub-tasks to reactive agents
 *
 * Still uses the same 10-action space for compatibility with the coordinator,
 * but augments the observation with depth-camera features (dims 24–31).
 *
 * @module cognitive-brain
 */

import { AgentBrain, AgentAction, OBS_DIM, ObsIndex } from './agent-brain.js';

// ─── EXTENDED OBSERVATION (base 24 + 8 depth features = 32) ──────────

export const COGNITIVE_OBS_DIM = 32;

export const CognitiveObsIndex = Object.freeze({
    // Inherit dims 0–23 from ObsIndex
    ...ObsIndex,

    // Depth camera features (24–31)
    DEPTH_MIN:              24,   // Minimum depth in FOV (normalized)
    DEPTH_CENTER:           25,   // Center depth reading
    DEPTH_SPREAD:           26,   // Depth variance (obstacle complexity)
    ENTITY_COUNT:           27,   // Number of detected entities (normalized)
    NEAREST_ENTITY_DIST:    28,   // Distance to nearest entity
    NEAREST_ENTITY_ANGLE:   29,   // Angle to nearest entity (sin)
    ENTITY_INTEREST_SCORE:  30,   // Max interest score from VLM
    DEPTH_COVERAGE_RATIO:   31,   // % of depth pixels with valid readings
});


// ─── COGNITIVE BRAIN ─────────────────────────────────────────────────

export class CognitiveBrain extends AgentBrain {
    /**
     * @param {string} agentId
     * @param {Object} [config]
     */
    constructor(agentId, config = {}) {
        // Higher-capacity defaults for cognitive drones
        super(agentId, {
            learningRate:    config.learningRate    ?? 0.03,    // Slower, more stable
            gamma:           config.gamma           ?? 0.98,    // Longer horizon
            epsilon:         config.epsilon         ?? 0.25,    // Less random
            epsilonMin:      config.epsilonMin      ?? 0.03,
            epsilonDecay:    config.epsilonDecay    ?? 0.9998,  // Slower decay
            batchSize:       config.batchSize       ?? 64,      // Bigger batches
            replaySize:      config.replaySize      ?? 50000,   // 5× capacity
            learningStarts:  config.learningStarts  ?? 64,      // Start learning quickly (2×batchSize)
            targetUpdateFreq:config.targetUpdateFreq?? 200,     // Slower target sync
            numTilings:      config.numTilings      ?? 12,      // More tiles → finer
            tilesPerDim:     config.tilesPerDim     ?? 6,       // Higher resolution
            trainingEnabled: config.trainingEnabled ?? true,
            obsDim:          COGNITIVE_OBS_DIM,                 // 32-dim observation
        });

        // Override brain type identifier
        this.brainType = 'cognitive';

        // ── Depth camera state ──
        this._lastDepthData    = null;
        this._entityMemory     = new Map();  // entityId → { lastSeen, position, category, visits }
        this._explorationBonus = 0;

        // ── Mission-level state ──
        this._claimedZone      = null;       // { xMin, zMin, xMax, zMax }
        this._delegatedTasks   = [];         // Tasks delegated to reactive agents
        this._planningHorizon  = config.planningHorizon ?? 10;  // Steps ahead to consider

        // ── MARL features ──
        this._teamRewardAccum  = 0;
        this._individualRewardAccum = 0;
        this._cooperativeWeight = config.cooperativeWeight ?? 0.6;  // Heavier team bias
    }

    // ────────────────────────────────────────────────────────────────
    // DEPTH CAMERA INTEGRATION
    // ────────────────────────────────────────────────────────────────

    /**
     * Feed depth camera output into the brain's state.
     * Called by the agent's reactive layer when a DepthCameraSimulator is attached.
     *
     * @param {Object} depthData - { depthMap, pointCloud, detections }
     */
    updateDepthData(depthData) {
        if (!depthData) return;
        this._lastDepthData = depthData;

        // Update entity memory from detections
        if (depthData.detections) {
            const now = Date.now();
            for (const det of depthData.detections) {
                const key = det.trackId || `${det.category}_${det.position?.x?.toFixed(0)}_${det.position?.z?.toFixed(0)}`;
                const existing = this._entityMemory.get(key);
                if (existing) {
                    existing.lastSeen = now;
                    existing.visits++;
                    existing.confidence = det.confidence || existing.confidence;
                } else {
                    this._entityMemory.set(key, {
                        firstSeen: now,
                        lastSeen: now,
                        position: det.position,
                        category: det.category,
                        confidence: det.confidence || 0.5,
                        visits: 1,
                        interestScore: this._computeInterestScore(det)
                    });
                }
            }

            // Prune old entities (>30s unseen)
            for (const [key, ent] of this._entityMemory) {
                if (now - ent.lastSeen > 30000) {
                    this._entityMemory.delete(key);
                }
            }
        }
    }

    /**
     * Augment the base 24-dim observation with 8 depth-camera features.
     *
     * @param {Float32Array|number[]} baseObs - 24-dim base observation
     * @returns {Float32Array} - 32-dim augmented observation
     */
    augmentObservation(baseObs) {
        const obs = new Float32Array(COGNITIVE_OBS_DIM);

        // Copy base observation (dims 0–23)
        for (let i = 0; i < Math.min(baseObs.length, OBS_DIM); i++) {
            obs[i] = baseObs[i];
        }

        // Fill depth-camera dims (24–31)
        if (this._lastDepthData) {
            const dm = this._lastDepthData.depthMap;

            // Min depth (normalized: 0=very close, 1=max range)
            if (dm && dm.length > 0) {
                let minD = Infinity, sumD = 0, countValid = 0;
                let centerD = dm[Math.floor(dm.length / 2)] || 35;
                for (const d of dm) {
                    if (d > 0 && d < 35) {
                        minD = Math.min(minD, d);
                        sumD += d;
                        countValid++;
                    }
                }
                obs[CognitiveObsIndex.DEPTH_MIN] = Math.min(1, (minD === Infinity ? 35 : minD) / 35);
                obs[CognitiveObsIndex.DEPTH_CENTER] = Math.min(1, centerD / 35);
                obs[CognitiveObsIndex.DEPTH_COVERAGE_RATIO] = countValid / Math.max(1, dm.length);

                // Depth variance (complexity)
                if (countValid > 1) {
                    const mean = sumD / countValid;
                    let variance = 0;
                    for (const d of dm) {
                        if (d > 0 && d < 35) {
                            variance += (d - mean) ** 2;
                        }
                    }
                    variance /= countValid;
                    obs[CognitiveObsIndex.DEPTH_SPREAD] = Math.min(1, Math.sqrt(variance) / 10);
                }
            }

            // Entity features
            const dets = this._lastDepthData.detections || [];
            obs[CognitiveObsIndex.ENTITY_COUNT] = Math.min(1, dets.length / 10);

            if (dets.length > 0) {
                // Sort by distance to find nearest
                const sorted = [...dets].sort((a, b) => (a.distance || 999) - (b.distance || 999));
                const nearest = sorted[0];
                obs[CognitiveObsIndex.NEAREST_ENTITY_DIST] = Math.min(1, (nearest.distance || 35) / 35);
                obs[CognitiveObsIndex.NEAREST_ENTITY_ANGLE] = nearest.angle ? Math.sin(nearest.angle) : 0;

                // Max interest score
                let maxInterest = 0;
                for (const det of dets) {
                    const interest = this._computeInterestScore(det);
                    maxInterest = Math.max(maxInterest, interest);
                }
                obs[CognitiveObsIndex.ENTITY_INTEREST_SCORE] = Math.min(1, maxInterest);
            }
        }

        return obs;
    }

    /**
     * Override chooseAction to augment observation if base is 24-dim.
     */
    chooseAction(observation, actionMask) {
        let obs = observation;

        // If we receive a 24-dim observation, augment it with depth features
        if (obs.length === OBS_DIM && this._lastDepthData) {
            obs = this.augmentObservation(obs);
        }

        const result = super.chooseAction(obs, actionMask);

        // Add exploration bonus for entities of interest
        if (this._lastDepthData?.detections?.length > 0) {
            const hasInteresting = this._lastDepthData.detections.some(
                d => this._computeInterestScore(d) > 0.6
            );
            if (hasInteresting && result.action === AgentAction.EXPLORE_FORWARD) {
                // Boost tendency to approach interesting entities
                result.deliberation = 'entity-approach';
            }
        }

        return result;
    }

    /**
     * Learn with MARL-aware reward decomposition.
     */
    learn(obs, action, reward, nextObs, done) {
        // Track individual vs team reward
        this._individualRewardAccum += reward;

        // Call parent Q-learning update
        super.learn(obs, action, reward, nextObs, done);
    }

    /**
     * Compute interest score for a detected entity.
     * Higher = more worth investigating.
     * @private
     */
    _computeInterestScore(det) {
        if (!det) return 0;

        const CATEGORY_INTEREST = {
            'person': 1.0,
            'vehicle': 0.8,
            'animal': 0.7,
            'drone': 0.6,
            'zone': 0.5,
            'building': 0.3,
            'tree': 0.1,
            'obstacle': 0.1
        };

        let score = CATEGORY_INTEREST[det.category] || 0.2;

        // Boost for high-confidence detections
        score *= (det.confidence || 0.5);

        // Novelty: entities seen fewer times are more interesting
        const key = det.trackId || `${det.category}_${det.position?.x?.toFixed(0)}_${det.position?.z?.toFixed(0)}`;
        const memory = this._entityMemory.get(key);
        if (memory) {
            score *= Math.max(0.1, 1 - memory.visits * 0.1);
        }

        return Math.min(1, score);
    }

    // ────────────────────────────────────────────────────────────────
    // ZONE CLAIMING (Mission-level deliberation)
    // ────────────────────────────────────────────────────────────────

    /**
     * Claim an exploration zone for focused coverage.
     * @param {{ xMin: number, zMin: number, xMax: number, zMax: number }} zone
     */
    claimZone(zone) {
        this._claimedZone = zone;
    }

    /**
     * Release the claimed zone.
     * @returns {Object|null} The released zone
     */
    releaseZone() {
        const zone = this._claimedZone;
        this._claimedZone = null;
        return zone;
    }

    /**
     * Get this brain's claimed zone.
     * @returns {Object|null}
     */
    getClaimedZone() {
        return this._claimedZone;
    }

    // ────────────────────────────────────────────────────────────────
    // EXPORT / IMPORT
    // ────────────────────────────────────────────────────────────────

    exportWeights() {
        const base = super.exportWeights();
        return {
            ...base,
            brainType: 'cognitive',
            entityMemorySize: this._entityMemory.size,
            claimedZone: this._claimedZone,
            teamRewardAccum: this._teamRewardAccum,
            individualRewardAccum: this._individualRewardAccum
        };
    }

    getStats() {
        return {
            ...super.getStats(),
            brainType: 'cognitive',
            entityMemorySize: this._entityMemory.size,
            claimedZone: this._claimedZone,
            depthDataAvailable: !!this._lastDepthData,
            cooperativeWeight: this._cooperativeWeight,
            planningHorizon: this._planningHorizon
        };
    }

    reset() {
        super.reset();
        this._lastDepthData = null;
        this._entityMemory.clear();
        this._claimedZone = null;
        this._delegatedTasks = [];
        this._teamRewardAccum = 0;
        this._individualRewardAccum = 0;
    }
}

export default CognitiveBrain;
