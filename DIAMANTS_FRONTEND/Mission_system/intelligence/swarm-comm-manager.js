/**
 * SwarmCommManager — Stub public
 * ================================
 * L'implémentation réelle (radio simulée entre drones : portée, pertes de
 * paquets, latence, synchronisation de connaissance locale, horloges
 * vectorielles, directives) vit dans le dépôt privé.
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
    getCommNeighbors(_droneId) { return []; }
    getPendingWaves() { return []; }
    getBeaconZones(_droneId) { return []; }
    getAllKnownBeaconZones() { return []; }
    getLocalKnowledge(_droneId) { return 0; }
    getVectorClock(_droneId) { return {}; }
    getInfoFlow(_droneId) { return { explored: 0, received: 0, ratio: 0 }; }
    getGlobalInfoFlowRatio() { return 0; }
    getLatestEvent(_droneId) { return null; }
    getDirective(_droneId) { return null; }
    getCommDetails(_droneId) { return null; }
    getCommLabel(_droneId) { return ''; }
    isCommunicating(_droneId) { return false; }
    getStats() {
        return { messages: 0, drops: 0, pairs: 0, avgLatencyMs: 0 };
    }

    // ── Écritures : absorbées sans effet ─────────────────────────────
    consumeDirective(_droneId) { return null; }
    reportBeaconFound(_droneId, _beacon) { /* no-op */ }
}

export default SwarmCommManager;
