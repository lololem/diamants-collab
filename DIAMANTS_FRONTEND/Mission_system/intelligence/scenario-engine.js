/**
 * ScenarioEngine — Stub public
 * ==============================
 * Le catalogue de scénarios PoC et leur logique d'exécution (déclenchement,
 * vérification de preuve, observation de l'essaim) vivent dans le dépôt privé.
 *
 * Ce stub expose la même surface avec un catalogue VIDE. Il est importé
 * statiquement par ui/panel-controller.js : sans lui, la compilation échoue.
 *
 * Conséquence : le sélecteur « Scénarios PoC » de l'interface s'affiche mais
 * ne propose aucune entrée. Le reste de l'application est intact.
 */

export class ScenarioEngine {
    constructor() {
        this.scenarios = new Map();   // catalogue vide
        this._activeToggles = new Map();
        this._activeId = null;
        this._scenarios = this.scenarios;
        this._autoCheckInterval = null;
    }

    getAll() { return []; }
    get(_id) { return undefined; }
    isActive(_id) { return false; }
    getActiveScenarios() { return []; }

    async execute(_id) { return { ok: false, reason: 'scénarios indisponibles (stub public)' }; }
    async resetAndExecute(_id) { return this.execute(_id); }

    _startAutoCheck() { /* no-op */ }
    _stopAutoCheck() { /* no-op */ }
}

export default ScenarioEngine;
