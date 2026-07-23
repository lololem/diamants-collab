/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * ScenarioEngine — Stub public
 * ==============================
 * The concrete implementation is not part of this distribution.
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
