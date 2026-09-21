/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * CollaborativeScouting — public shell.
 *
 * The trace-based coverage behaviour (deposit and decay, consensus threshold,
 * voxel pathfinding) is not published. The flight engine still explores, spreads
 * the fleet and avoids obstacles on its own; coordination beyond that is yours to
 * write.
 */

export class CollaborativeScouting {
    constructor(config = {}) {
        this.config = config;
        this.missionState = {
            active: false,
            phase: 'IDLE',
            coverage: 0,
            progress: 0,
            // same shape the controller prints every few seconds
            collaborationMetrics: { coordinationIndex: 0, redundancy: 0, exchanges: 0 },
        };
    }
    startScoutingMission(_opts) { return false; }
    getMissionStatus() { return { ...this.missionState }; }
    update(_droneStates, _dt) { return new Map(); }
    stop() { this.missionState.active = false; }
}

export default CollaborativeScouting;
