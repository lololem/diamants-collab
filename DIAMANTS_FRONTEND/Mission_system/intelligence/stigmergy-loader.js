/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Stigmergy Engine Loader
 * =====================================
 * Charge dynamiquement l'implémentation du moteur stigmergique.
 * 
 * Le code propriétaire (StigmergyEngine) vit dans diamants-private.
 * Ce loader permet au frontend public de l'utiliser SI disponible,
 * sans dépendance directe.
 *
 * Mécanisme (par ordre de priorité):
 *   1. Import dynamique de stigmergy-engine-private.js (symlink vers diamants-private)
 *   2. Cherche `window.DIAMANTS_STIGMERGY_ENGINE` (injection manuelle)
 *   3. Si rien trouvé, retourne null (l'engine utilise NoopSwarmIntelligence par défaut)
 *
 * Pour activer le moteur stigmergique depuis diamants-private:
 *   - Créer un symlink: ln -s /path/to/diamants-private/src/intelligence/stigmergy-engine.js stigmergy-engine-private.js
 *   - OU importer manuellement: import { StigmergyEngine } from '...'; window.DIAMANTS_STIGMERGY_ENGINE = StigmergyEngine;
 */

import { StigmergyInterface, PheromoneType } from './stigmergy-interface.js';

// Re-export for convenience
export { StigmergyInterface, PheromoneType };

/**
 * Default stigmergy configuration.
 * Can be overridden when calling loadStigmergyEngine(config).
 */
export const DEFAULT_STIGMERGY_CONFIG = {
    gridResolution: 1.0,    // meters per cell
    gridSize: 100,          // cells per side (100x100 = 10,000 cells for 100m arena)
    evaporationRate: 0.02,  // per second
    diffusionRate: 0.1,     // per tick
    maxIntensity: 100,

    // Pheromone weights for combined gradient
    weights: {
        exploration: -1.5,   // repulsion (avoid revisiting)
        interest: 3.0,       // strong attraction
        danger: -5.0,        // very strong repulsion
        rally: 2.0,          // medium attraction
    },

    // Deposit amounts
    deposits: {
        exploration: 5.0,
        interest: 20.0,
        danger: 40.0,
    },

    // Waypoint generation
    waypointDistance: 10.0,
    minGradientMagnitude: 0.1,
};

/**
 * Try to dynamically import the private stigmergy engine.
 * Returns the StigmergyEngine class or null.
 */
async function tryLoadPrivateEngine() {
    // Try 1: Distributed Swarm Engine (autonomous agents)
    try {
        const distModule = await import('./distributed-swarm-engine.js');
        if (distModule.DistributedSwarmEngine) {
            return { Engine: distModule.DistributedSwarmEngine, mode: 'distributed' };
        }
    } catch (err) {
        console.debug('🔍 Distributed engine not available:', err.message);
    }

    // Try 2: symlink stigmergy-engine-private.js → diamants-private
    try {
        const module = await import('./stigmergy-engine-private.js');
        if (module.StigmergyEngine) {
            return { Engine: module.StigmergyEngine, mode: 'centralized' };
        }
    } catch (err) {
        // File not found or symlink broken — that's OK, it's optional
        if (!err.message?.includes('Failed to fetch') && !err.message?.includes('Cannot find')) {
            console.debug('🔍 Private stigmergy module not available:', err.message);
        }
    }

    // Try 3: Direct import of stigmergy-engine.js (concrete implementation)
    try {
        const module = await import('./stigmergy-engine.js');
        if (module.StigmergyEngine) {
            return { Engine: module.StigmergyEngine, mode: 'centralized' };
        }
    } catch (err) {
        console.debug('🔍 Stigmergy engine not found:', err.message);
    }

    return null;
}

/**
 * Load the StigmergyEngine implementation if available.
 * This is an ASYNC function — use await or .then() to get the result.
 *
 * @param {object} config - Configuration to pass to the engine constructor
 * @returns {Promise<StigmergyInterface|null>} - Engine instance or null if not available
 */
export async function loadStigmergyEngine(config = {}) {
    const mergedConfig = { ...DEFAULT_STIGMERGY_CONFIG, ...config };

    // 1. Check for already-injected implementation (synchronous)
    if (window.DIAMANTS_STIGMERGY_ENGINE) {
        try {
            const EngineClass = window.DIAMANTS_STIGMERGY_ENGINE;
            const engine = new EngineClass(mergedConfig);
            console.log('🧠 StigmergyEngine loaded from window.DIAMANTS_STIGMERGY_ENGINE');
            return engine;
        } catch (err) {
            console.warn('⚠️ Failed to instantiate StigmergyEngine:', err);
        }
    }

    // 2. Try dynamic import (distributed first, then centralized)
    const result = await tryLoadPrivateEngine();
    if (result) {
        try {
            const engine = new result.Engine(mergedConfig);
            const modeIcon = result.mode === 'distributed' ? '🌐' : '🧠';
            console.log(`${modeIcon} StigmergyEngine loaded — mode: ${result.mode.toUpperCase()}`);
            return engine;
        } catch (err) {
            console.warn('⚠️ Failed to instantiate StigmergyEngine:', err);
        }
    }

    // 3. Check for async loader callback (alternative pattern)
    if (window.DIAMANTS_LOAD_STIGMERGY) {
        try {
            const engine = window.DIAMANTS_LOAD_STIGMERGY(mergedConfig);
            if (engine) {
                console.log('🧠 StigmergyEngine loaded via async loader callback');
                return engine;
            }
        } catch (err) {
            console.warn('⚠️ Async stigmergy loader failed:', err);
        }
    }

    // No implementation available
    console.log('ℹ️ No StigmergyEngine available — using NoopSwarmIntelligence');
    return null;
}

/**
 * Check if stigmergy implementation is available.
 * @returns {boolean}
 */
export function isStigmergyAvailable() {
    return !!(window.DIAMANTS_STIGMERGY_ENGINE || window.DIAMANTS_LOAD_STIGMERGY);
}

/**
 * Register a stigmergy engine implementation.
 * Call this from diamants-private to make the engine available.
 *
 * @param {typeof StigmergyInterface} EngineClass - The StigmergyEngine class
 */
export function registerStigmergyEngine(EngineClass) {
    window.DIAMANTS_STIGMERGY_ENGINE = EngineClass;
    console.log('✅ StigmergyEngine registered');

    // Dispatch event for late listeners
    try {
        window.dispatchEvent(new CustomEvent('diamants:stigmergy-available', {
            detail: { EngineClass }
        }));
    } catch (_) { /* ignore */ }
}
