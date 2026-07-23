/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * agent-drone-registry.js — Maps drone profiles to agent intelligence types.
 *
 * Central registry that defines the "marriage" between:
 *   - Drone hardware profile (physics, sensors, visual mesh)
 *   - Agent brain type (reactive, cognitive, custom)
 *   - Sensor suite (Multi-Ranger for Crazyflie, OAK-D for X500)
 *   - Digital twin visual model
 *
 * Usage:
 *   const spec = AgentDroneRegistry.getSpec('X500');
 *   // → { profileId: 'X500', brainType: 'cognitive', brainClass: CognitiveBrain,
 *   //     sensors: ['depth-camera'], visualModel: 'x500', ... }
 *
 * @module agent-drone-registry
 */

import { ReactiveBrain }  from './reactive-brain.js';
import { CognitiveBrain } from './cognitive-brain.js';
import { AgentBrain }     from './agent-brain.js';

// ─── AGENT ROLES ─────────────────────────────────────────────────────

export const AgentRole = Object.freeze({
    REACTIVE:   'reactive',    // Rule-based, fast, lightweight (Crazyflie)
    COGNITIVE:  'cognitive',   // Q-learning + depth camera, deliberative (X500)
    HYBRID:     'hybrid',      // Q-learning, standard sensors (Mavic/Phantom)
});

// ─── SENSOR SUITES ───────────────────────────────────────────────────

export const SensorSuite = Object.freeze({
    MULTI_RANGER:  'multi-ranger',    // 4-5 ToF beams (Crazyflie)
    DEPTH_CAMERA:  'depth-camera',    // OAK-D stereo (X500)
    BASIC:         'basic',           // Raycasting only
});

// ─── DEFAULT BRAIN CONFIGS ───────────────────────────────────────────

const BRAIN_CONFIGS = {
    reactive: {
        wallFollowSide: 'left',
        jitterProb: 0.1
    },
    cognitive: {
        learningRate: 0.03,
        gamma: 0.98,
        epsilon: 0.25,
        epsilonMin: 0.03,
        epsilonDecay: 0.9998,
        batchSize: 64,
        replaySize: 50000,
        cooperativeWeight: 0.6,
        planningHorizon: 10
    },
    hybrid: {
        learningRate: 0.05,
        gamma: 0.95,
        epsilon: 0.3,
        epsilonMin: 0.05,
        epsilonDecay: 0.9995,
        batchSize: 32,
        replaySize: 10000
    }
};

// ─── BUILT-IN DRONE-AGENT SPECS ──────────────────────────────────────

const BUILT_IN_SPECS = new Map([
    ['CRAZYFLIE', {
        profileId:    'CRAZYFLIE',
        role:         AgentRole.REACTIVE,
        brainType:    'reactive',
        brainClass:   ReactiveBrain,
        brainConfig:  BRAIN_CONFIGS.reactive,
        sensors:      [SensorSuite.MULTI_RANGER],
        visualModel:  'crazyflie',
        description:  'Crazyflie 2.1 — Lightweight reactive scout. Rule-based wall-following, ' +
                      'Multi-Ranger ToF sensors. Fast 60Hz reactive layer, no learning.',
        capabilities: ['wall-following', 'collision-avoidance', 'frontier-seeking', 'swarm-separation'],
        limitations:  ['no-learning', 'short-range-sensors', 'low-payload', 'short-endurance'],
        teamRole:     'scout',       // In MARL: fast exploration, coverage filling
        colorHint:    0x00ff00,      // Green for reactive
    }],

    ['X500', {
        profileId:    'X500',
        role:         AgentRole.COGNITIVE,
        brainType:    'cognitive',
        brainClass:   CognitiveBrain,
        brainConfig:  BRAIN_CONFIGS.cognitive,
        sensors:      [SensorSuite.DEPTH_CAMERA],
        visualModel:  'x500',
        description:  'Holybro X500 V2 — Heavy cognitive agent. Q-learning with depth camera, ' +
                      'OAK-D Pro W stereo vision (127°×80° FOV), entity detection, zone claiming.',
        capabilities: ['q-learning', 'depth-perception', 'entity-detection', 'zone-claiming',
                       'deliberative-planning', 'task-delegation', 'experience-replay'],
        limitations:  ['heavy', 'slower-reactions', 'higher-power'],
        teamRole:     'coordinator', // In MARL: plans zones, delegates to reactives
        colorHint:    0xff4400,      // Orange for cognitive
    }],

    ['MAVIC', {
        profileId:    'MAVIC',
        role:         AgentRole.HYBRID,
        brainType:    'hybrid',
        brainClass:   AgentBrain,
        brainConfig:  BRAIN_CONFIGS.hybrid,
        sensors:      [SensorSuite.BASIC],
        visualModel:  'mavic',
        description:  'Mavic Pro — Medium hybrid agent. Standard Q-learning brain, ' +
                      'basic perception. Balanced speed and intelligence.',
        capabilities: ['q-learning', 'experience-replay', 'frontier-seeking'],
        limitations:  ['no-depth-camera', 'moderate-agility'],
        teamRole:     'explorer',
        colorHint:    0x0088ff,      // Blue for hybrid
    }],

    ['PHANTOM', {
        profileId:    'PHANTOM',
        role:         AgentRole.HYBRID,
        brainType:    'hybrid',
        brainClass:   AgentBrain,
        brainConfig:  BRAIN_CONFIGS.hybrid,
        sensors:      [SensorSuite.BASIC],
        visualModel:  'phantom',
        description:  'Phantom 4 — Heavy hybrid agent. Standard Q-learning, ' +
                      'stable platform for observation.',
        capabilities: ['q-learning', 'experience-replay', 'stable-hover'],
        limitations:  ['no-depth-camera', 'large', 'slow'],
        teamRole:     'observer',
        colorHint:    0x8800ff,      // Purple for hybrid
    }],

    ['S500', {
        profileId:    'S500',
        role:         AgentRole.COGNITIVE,
        brainType:    'cognitive',
        brainClass:   CognitiveBrain,
        brainConfig:  BRAIN_CONFIGS.cognitive,
        sensors:      [SensorSuite.DEPTH_CAMERA],
        visualModel:  's500',
        description:  'Holybro S500 — Medium-heavy patrol agent. Cognitive brain with ' +
                      'OAK-D Lite depth camera (73°×58° FOV). Endurance patrol and area monitoring.',
        capabilities: ['q-learning', 'depth-perception', 'entity-detection', 'area-patrol',
                       'deliberative-planning', 'experience-replay'],
        limitations:  ['heavy', 'lower-twr', 'slower-than-x500'],
        teamRole:     'patrol',
        colorHint:    0x2E86C1,      // Cyan-blue for patrol
    }],
]);


// ─── AGENT-DRONE REGISTRY ────────────────────────────────────────────

export class AgentDroneRegistry {
    static _customSpecs = new Map();

    /**
     * Get the full agent-drone specification for a profile.
     *
     * @param {string} profileId - 'CRAZYFLIE', 'X500', 'MAVIC', 'PHANTOM', or custom
     * @returns {Object} Full spec with brainClass, sensors, visualModel, etc.
     */
    static getSpec(profileId) {
        return this._customSpecs.get(profileId)
            || BUILT_IN_SPECS.get(profileId)
            || BUILT_IN_SPECS.get('CRAZYFLIE');  // Fallback
    }

    /**
     * Register a custom drone-agent spec.
     *
     * @param {string} profileId
     * @param {Object} spec - Must include { brainClass, role, sensors, visualModel }
     */
    static registerSpec(profileId, spec) {
        if (!spec.brainClass || !spec.role) {
            throw new Error(`AgentDroneRegistry: spec for '${profileId}' must include brainClass and role`);
        }
        this._customSpecs.set(profileId, {
            profileId,
            ...spec,
            brainConfig: spec.brainConfig || BRAIN_CONFIGS[spec.brainType] || {}
        });
        console.log(`[AgentDroneRegistry] Custom spec registered: ${profileId} → ${spec.role}`);
    }

    /**
     * Create the appropriate brain for a given profile.
     *
     * @param {string} profileId
     * @param {string} agentId
     * @param {Object} [configOverrides] - Override brain config
     * @returns {import('./brain-interface.js').BrainInterface}
     */
    static createBrain(profileId, agentId, configOverrides = {}) {
        const spec = this.getSpec(profileId);
        const BrainClass = spec.brainClass;
        const config = { ...spec.brainConfig, ...configOverrides };

        return new BrainClass(agentId, config);
    }

    /**
     * Check if a profile should have a depth camera attached.
     *
     * @param {string} profileId
     * @returns {boolean}
     */
    static hasDepthCamera(profileId) {
        const spec = this.getSpec(profileId);
        return spec.sensors.includes(SensorSuite.DEPTH_CAMERA);
    }

    /**
     * Get the agent role for a profile.
     *
     * @param {string} profileId
     * @returns {string} AgentRole value
     */
    static getRole(profileId) {
        return this.getSpec(profileId).role;
    }

    /**
     * List all registered specs (built-in + custom).
     *
     * @returns {Array<{profileId, role, brainType, teamRole, description}>}
     */
    static listSpecs() {
        const all = new Map([...BUILT_IN_SPECS, ...this._customSpecs]);
        return [...all.values()].map(s => ({
            profileId:   s.profileId,
            role:        s.role,
            brainType:   s.brainType,
            teamRole:    s.teamRole,
            description: s.description,
            sensors:     s.sensors,
            capabilities: s.capabilities,
            colorHint:   s.colorHint
        }));
    }

    /**
     * Generate a MARL fleet composition recommendation.
     * Returns suggested drone mix for effective multi-agent coverage.
     *
     * @param {number} totalDrones - Total fleet size
     * @param {string} [strategy='balanced'] - 'balanced' | 'swarm' | 'cognitive-heavy'
     * @returns {Array<{ profileId: string, count: number, role: string }>}
     */
    static recommendFleetComposition(totalDrones, strategy = 'balanced') {
        if (strategy === 'swarm') {
            // Mostly reactive with 1 coordinator
            return [
                { profileId: 'X500',      count: 1,               role: AgentRole.COGNITIVE },
                { profileId: 'CRAZYFLIE', count: totalDrones - 1,  role: AgentRole.REACTIVE },
            ];
        }

        if (strategy === 'cognitive-heavy') {
            const cognitive = Math.ceil(totalDrones * 0.6);
            return [
                { profileId: 'X500',      count: cognitive,             role: AgentRole.COGNITIVE },
                { profileId: 'CRAZYFLIE', count: totalDrones - cognitive, role: AgentRole.REACTIVE },
            ];
        }

        // Balanced: 1/3 cognitive, 2/3 reactive
        const cognitive = Math.max(1, Math.floor(totalDrones / 3));
        const reactive  = totalDrones - cognitive;
        return [
            { profileId: 'X500',      count: cognitive, role: AgentRole.COGNITIVE },
            { profileId: 'CRAZYFLIE', count: reactive,  role: AgentRole.REACTIVE },
        ];
    }
}

export default AgentDroneRegistry;
