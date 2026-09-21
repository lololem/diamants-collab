/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * AgentDroneRegistry — which role a drone profile plays in the fleet.
 *
 * Deliberately simple and public: it decides who scouts and who confirms, and
 * which platform carries a depth camera. A profile it does not know gets the
 * generic entry, so your own profile works without touching this file.
 *
 * The learning agents that used these roles in the research build are not
 * published; see agent/multi-agent-coordinator.js.
 */

export const AgentRole = Object.freeze({
    REACTIVE: 'reactive',    // small, cheap, many - covers ground
    COGNITIVE: 'cognitive',  // carries the sensors, confirms what scouts report
    HYBRID: 'hybrid',
});

const SPECS = {
    CRAZYFLIE: { role: AgentRole.REACTIVE,  teamRole: 'scout',       sensors: ['multi-ranger'], depthCamera: false },
    X500:      { role: AgentRole.COGNITIVE, teamRole: 'coordinator', sensors: ['depth-camera'], depthCamera: true },
    S500:      { role: AgentRole.COGNITIVE, teamRole: 'relay',       sensors: ['depth-camera'], depthCamera: true },
    MAVIC:     { role: AgentRole.HYBRID,    teamRole: 'explorer',    sensors: ['basic'],        depthCamera: false },
    PHANTOM:   { role: AgentRole.HYBRID,    teamRole: 'observer',    sensors: ['basic'],        depthCamera: false },
};

/** Any profile the table does not name flies as a hybrid explorer. */
const GENERIC = { role: AgentRole.HYBRID, teamRole: 'explorer', sensors: ['basic'], depthCamera: false };

export class AgentDroneRegistry {
    /** @returns {{profileId:string, role:string, teamRole:string, sensors:string[], depthCamera:boolean}} */
    static getSpec(profileId) {
        const s = SPECS[profileId] || GENERIC;
        return { profileId, ...s };
    }

    static hasDepthCamera(profileId) {
        return (SPECS[profileId] || GENERIC).depthCamera;
    }

    static listSpecs() {
        return Object.keys(SPECS).map(id => this.getSpec(id));
    }

    /**
     * How to split a fleet of `count` drones between scouts and coordinators.
     * @param {number} count
     * @param {'balanced'|'swarm'|'cognitive-heavy'} [mode]
     * @returns {{profileId:string, count:number, role:string}[]}
     */
    static recommendFleetComposition(count, mode = 'balanced') {
        const n = Math.max(1, count | 0);
        let cognitive;
        if (mode === 'swarm') cognitive = 1;
        else if (mode === 'cognitive-heavy') cognitive = Math.ceil(n * 0.6);
        else cognitive = Math.max(1, Math.floor(n / 4));
        cognitive = Math.min(cognitive, n);
        const reactive = n - cognitive;

        const out = [];
        if (cognitive > 0) {
            const x500 = Math.ceil(cognitive / 2);
            out.push({ profileId: 'X500', count: x500, role: AgentRole.COGNITIVE });
            if (cognitive - x500 > 0) out.push({ profileId: 'S500', count: cognitive - x500, role: AgentRole.COGNITIVE });
        }
        if (reactive > 0) out.push({ profileId: 'CRAZYFLIE', count: reactive, role: AgentRole.REACTIVE });
        return out;
    }
}

export default AgentDroneRegistry;
