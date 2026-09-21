/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * BrainInterface — the contract a decision maker implements.
 *
 * Write your own brain against this interface, or plug a model through
 * intelligence/agent-model-interface.js (see README, "Bring your own model").
 *
 * The learning agents of the research version — observation encoding, reward
 * shaping, training environment, weight transfer — are not published.
 */

export class BrainInterface {
    /** @param {Float32Array|number[]} observation @returns {{action:number, confidence:number}} */
    decide(observation) { throw new Error('BrainInterface.decide() is not implemented'); }
    reset() { /* no-op */ }
    getWeights() { return null; }
    setWeights(_w) { /* no-op */ }
}

export class ModelRegistry {
    constructor() { this.models = new Map(); }
    register(id, model) { this.models.set(id, model); return id; }
    get(id) { return this.models.get(id) || null; }
    listModels() { return [...this.models.keys()]; }
    deleteModel(id) { return this.models.delete(id); }
    exportRegistry() { return null; }
    importRegistry(_data, _opts) { /* no-op */ }
}

export default BrainInterface;
