/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * MultiAgentCoordinator — public shell.
 *
 * The trainable multi-agent layer (per-drone learning agents, the radio protocol
 * they learn to use, cooperative reward, episodes and weight transfer) lives in
 * the private repository.
 *
 * This shell keeps the same surface, inert. Nothing is lost for extension: the
 * fleet flies on the PID engine and its state machine, and a model you plug in
 * goes through intelligence/agent-model-interface.js.
 */

class EmptyRegistry {
    listModels() { return []; }
    deleteModel() { return false; }
    exportRegistry() { return null; }
    importRegistry() { /* no-op */ }
}

export class MultiAgentCoordinator {
    constructor(config = {}) {
        this.config = config;
        this.agents = new Map();
        this.registry = new EmptyRegistry();
        this.autonomyLevel = config.autonomyLevel ?? 100;
    }
    async initialize() { return false; }
    addAgent(_id, _opts) { return null; }
    removeAgent(_id) { return null; }
    getAgentIds() { return []; }
    getAgentCount() { return 0; }
    getFleetComposition() { return null; }
    swapAgentBrain(_id, _brain, _opts) { return false; }
    registerModel(_id, _model) { return null; }
    loadModel(_id) { return false; }
    tick(_dt) { /* no-op */ }
    setAutonomyLevel(v) { this.autonomyLevel = v; }
    setTrainingEnabled(_v) { /* no-op */ }
    startNewEpisode() { /* no-op */ }
    onEpisodeEnd(_fn) { /* no-op */ }
    exportAllWeights() { return null; }
    importAllWeights(_w) { /* no-op */ }
    createSnapshot(_name) { return null; }
    rollbackToSnapshot(_name) { return false; }
    listSnapshots() { return []; }
    startABTest() { return null; }
    getABTestResults() { return null; }
    startExperiment() { return null; }
    endExperiment() { return null; }
    listExperiments() { return []; }
    exportExperiments() { return null; }
    getMetrics() { return null; }
}

export default MultiAgentCoordinator;
