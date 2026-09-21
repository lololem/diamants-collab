/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DroneIntelligenceManager — public shell
 * ========================================
 * The internal bridge to local language models (prompt building, decision
 * parsing, per-drone memory, action guards) lives in the private repository.
 *
 * This shell exposes the same surface, disabled. It is imported statically by
 * tools/integrated-controller.js.
 *
 * To plug in YOUR models instead, create
 * intelligence/model-providers/agent-models.json: the controller then swaps
 * this shell for neurosymbolic-bridge.js. See README, "Bring your own model".
 */

export class DroneIntelligenceManager {
    constructor() {
        this.connector = null;
        this.enabled = false;
    }

    async init() { return false; }
    registerDrone(_id, _type) { /* no-op */ }
    getBrain(_id) { return null; }
    async evaluate(_id, _state) { return null; }

    setEnabled(_id, _v) { /* no-op */ }
    setGlobalEnabled(_v) { /* no-op */ }
    setInfluence(_id, _v) { /* no-op */ }
    setGlobalInfluence(_v) { /* no-op */ }
    setModel(_id, _m) { /* no-op */ }
    setModelForType(_t, _m) { /* no-op */ }
    setOllamaUrl(_u) { /* no-op */ }

    reset(_id) { /* no-op */ }
    resetAll() { /* no-op */ }
    getStats() {
        return { requests: 0, cache: 0, failures: 0, latencyMs: null, activeBrains: 0 };
    }
}

export default DroneIntelligenceManager;
