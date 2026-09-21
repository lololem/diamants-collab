/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * SwarmCommManager — Stub public
 * ================================
 * The concrete implementation is not part of this distribution.
 *
 * Ce stub reproduit STRICTEMENT la surface d'API consommée par le reste du
 * code, avec des valeurs neutres. Il est importé statiquement par
 * physics/autonomous-flight-engine.js : sans lui, la compilation échoue et
 * plus rien ne vole.
 *
 * Contrat respecté (types de retour identiques à l'implémentation réelle) :
 *   getLocalCellsSet  -> Set        getCommNeighbors -> Array
 *   getPendingWaves   -> Array      getCommLabel     -> String
 *   isCommunicating   -> Boolean    getCommDetails   -> null
 *   getLocalKnowledge -> Number     getDirective     -> null
 *   getBeaconZones    -> Array
 *
 * Conséquence fonctionnelle : aucune communication inter-drones n'est
 * simulée. Les drones volent et explorent, mais ne partagent pas leur
 * connaissance de la carte. Le chemin de vol nominal reste intact — il ne
 * fait que lire des collections vides.
 */

export class SwarmCommManager {
    constructor() {
        // Le moteur de vol teste `_engine` pour savoir s'il doit s'attacher.
        this._engine = null;
    }

    attach(engine) {
        this._engine = engine || null;
    }

    reset() {
        // rien à réinitialiser
    }

    update(_dt) {
        // aucune propagation radio
    }

    // ── Lectures : collections vides, jamais null/undefined ──────────
    getLocalCellsSet(_droneId) { return new Set(); }
    // Set, et non Array : autonomous-flight-engine lit .has() et .size dessus.
    getCommNeighbors(_droneId) { return new Set(); }
    getPendingWaves() { return []; }
    getBeaconZones(_droneId) { return []; }
    getAllKnownBeaconZones() { return []; }
    getLocalKnowledge(_droneId) { return 0; }
    getVectorClock(_droneId) { return {}; }
    getInfoFlow(_droneId) { return { explored: 0, received: 0, ratio: 0 }; }
    getGlobalInfoFlowRatio() { return 0; }
    getLatestEvent(_droneId) { return null; }
    getDirective(_droneId) { return null; }
    // Objet structure, et non null : l'UI parcourt .peers et lit .infoFlow.
    getCommDetails(_droneId) {
        return {
            active: false,
            peers: [],
            ownKnowledge: 0,
            directive: null,
            infoFlow: { explored: 0, received: 0, ratio: 0 },
            beaconZones: 0,
            beaconZoneList: [],
        };
    }
    getCommLabel(_droneId) { return ''; }
    isCommunicating(_droneId) { return false; }
    getStats() {
        return { messages: 0, drops: 0, pairs: 0, avgLatencyMs: 0 };
    }

    // ── Écritures : absorbées sans effet ─────────────────────────────
    consumeDirective(_droneId) { return null; }
    reportBeaconFound(_droneId, _position, _beaconId) { /* no-op */ }
}

export default SwarmCommManager;
