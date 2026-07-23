/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Diagnostic & Repair stub
 * Placeholder to prevent Vite 404 errors.
 * Real diagnostics are handled by the IntegratedController.
 */
(function () {
    'use strict';

    // Expose a no-op diagnostic API on window so other scripts don't error
    window.DIAMANTS_DIAGNOSTIC = {
        check() { return { ok: true, issues: [] }; },
        repair() { return true; },
        status: 'ready',
    };
})();
