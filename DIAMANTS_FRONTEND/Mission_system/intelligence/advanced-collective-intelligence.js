/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * AdvancedCollectiveIntelligence — public shell. See collective-intelligence.js.
 */

import { CollectiveIntelligence } from './collective-intelligence.js';

export class AdvancedCollectiveIntelligence extends CollectiveIntelligence {
    /**
     * Same shape the controller reads every frame, with empty values: no
     * leaders, no patterns, no effect. Returning a smaller object here used to
     * throw on `state.wahooSystem.emergentLeaders` and stop the update loop.
     */
    getAdvancedState() {
        return {
            active: false,
            metrics: { wahooEffectiveness: 0, emergenceLevel: 0, coherence: 0 },
            phase: 'IDLE',
            wahooSystem: { emergentLeaders: [], attractors: [], intensity: 0 },
            patterns: [],
            attractors: [],
            repulsors: [],
            leadership: null,
        };
    }
    stop() { this.active = false; }
}

export default AdvancedCollectiveIntelligence;
