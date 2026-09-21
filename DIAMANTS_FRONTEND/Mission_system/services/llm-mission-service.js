/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * LLMMissionService — public shell.
 *
 * Turning an operator sentence into a mission (system prompt, parsing, mission
 * planning, the model call behind it) lives in the private repository.
 *
 * What stays public is everything a contributor needs: beacons can be placed
 * and are tracked here, the swarm searches for them on its own, and a model you
 * plug in decides through intelligence/agent-model-interface.js.
 */

export const MissionType = Object.freeze({
    EXPLORE: 'explore',
    SEARCH_ZONE: 'search_zone',
    SEARCH_BEACON: 'search_beacon',
    PATROL: 'patrol',
    FORMATION: 'formation',
    RTL: 'rtl',
});

const NOT_PUBLIC = 'The mission interpreter is not part of the public build. '
    + 'Place beacons and launch the mission from the panel, or plug your own model — see README, "Bring your own model".';

export class LLMMissionService {
    constructor(config = {}) {
        this.config = config;
        this.beacons = new Map();
        this.activeMission = null;
        this._model = '';
        this._ollamaOnline = false;
    }

    /** @returns {Promise<{text:string, mission:null}>} */
    async processMessage(_text) {
        return { text: NOT_PUBLIC, mission: null };
    }

    registerBeacon(id, position) {
        this.beacons.set(id, { id, position, found: false });
        return id;
    }

    markBeaconFound(id, droneId) {
        const b = this.beacons.get(id);
        if (b) { b.found = true; b.foundBy = droneId; }
        return !!b;
    }

    getBeacons() { return [...this.beacons.values()]; }
    clearBeacons() { this.beacons.clear(); }
}

export default LLMMissionService;
