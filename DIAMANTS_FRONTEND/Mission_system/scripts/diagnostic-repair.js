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
