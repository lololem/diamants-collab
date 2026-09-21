/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * MARLTrainingPanel — public shell.
 *
 * The training console (episodes, reward curves, weight import and export) goes
 * with the learning stack, which is not published.
 */

export class MARLTrainingPanel {
    constructor() { this.visible = false; }
    show() { /* no-op */ }
    hide() { /* no-op */ }
    toggle() { /* no-op */ }
    update() { /* no-op */ }
    destroy() { /* no-op */ }
}

export default MARLTrainingPanel;
