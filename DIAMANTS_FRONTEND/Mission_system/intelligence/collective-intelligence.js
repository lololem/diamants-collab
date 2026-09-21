/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * CollectiveIntelligence — public shell.
 *
 * The consensus map, emergent-pattern detection and dynamic attractors of the
 * research version are not published. Write your own against
 * intelligence/swarm-intelligence-interface.js — it is called every frame and its
 * output is merged with the flight command.
 */

export class CollectiveIntelligence {
    constructor(config = {}) { this.config = config; this.active = false; }
    initialize() { return false; }
    update(_droneStates, _dt) { return new Map(); }
    getState() { return { active: false, agents: 0, consensus: 0 }; }
    stop() { this.active = false; }
}

export default CollectiveIntelligence;
