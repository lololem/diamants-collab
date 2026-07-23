/**
 * DroneIntelligenceManager — Stub public
 * ========================================
 * Le pont vers les modèles de langage locaux (construction de prompt,
 * analyse de décision, mémoire par drone, garde-fous d'action) vit dans le
 * dépôt privé.
 *
 * Ce stub expose la même surface, désactivée. Il est importé statiquement
 * par tools/integrated-controller.js.
 *
 * Conséquence : le panneau « Intelligence LLM » s'affiche mais aucun drone
 * n'interroge de modèle. Le vol autonome, lui, ne dépend pas du LLM.
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
