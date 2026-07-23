/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Doctrine Importer
 * ==============================
 * Pont Autonome → CoLab (Gap #1 du ROADMAP_SMA)
 *
 * Charge un fichier JSON doctrine autonome et applique ses paramètres
 * au DoctrineManager du frontend collaboratif.
 *
 * Usage:
 *   import { importDoctrineFromFile, importDoctrineFromJSON } from './doctrine-importer.js';
 *   importDoctrineFromFile(doctrineManager);            // ouvre file picker
 *   importDoctrineFromJSON(doctrineManager, jsonObj);    // programmatique
 */

import { DOCTRINES, COURSES_OF_ACTION, FORMATION_TYPES } from './mission-doctrine.js';

// ─────────────────────────────────────────────
//  Reverse mappings (autonome → frontend)
// ─────────────────────────────────────────────

/**
 * Mapping behavior autonome → COA frontend
 */
const BEHAVIOR_TO_COA = {
    'grid_exploration':          'grid',
    'grid_exploration_with_ai':  'adaptive',
    'spiral_exploration':        'spiral',
    'radial_exploration':        'radial',
    'perimeter_patrol':          'perimeter',
    'thermal_grid_scan':         'grid',
    'grid_search':               'grid'
};

/**
 * Mapping mission type autonome → doctrine frontend
 */
const MISSION_TYPE_TO_DOCTRINE = {
    'exploration':  'stigmergy',
    'search':       'search',
    'patrol':       'formation',
    'surveillance': 'coverage',
    'fire':         'coverage',
    'incendie':     'coverage'
};


// ─────────────────────────────────────────────
//  Conversion NED → Frontend zone
// ─────────────────────────────────────────────

/**
 * Convertit une zone NED (n_min/n_max, e_min/e_max) en zone frontend (center + size).
 */
function nedToZone(nedZone) {
    return {
        centerX: (nedZone.e_min + nedZone.e_max) / 2,
        centerZ: (nedZone.n_min + nedZone.n_max) / 2,
        sizeX: nedZone.e_max - nedZone.e_min,
        sizeZ: nedZone.n_max - nedZone.n_min
    };
}


// ─────────────────────────────────────────────
//  Import logic
// ─────────────────────────────────────────────

/**
 * Valide qu'un objet JSON a la structure minimale d'une doctrine autonome.
 * @returns {{ valid: boolean, errors: string[] }}
 */
export function validateDoctrineJSON(json) {
    const errors = [];

    if (!json) {
        errors.push('JSON est null ou undefined');
        return { valid: false, errors };
    }

    // Required top-level keys
    const requiredKeys = ['doctrine', 'mission'];
    for (const key of requiredKeys) {
        if (!json[key]) {
            errors.push(`Section "${key}" manquante`);
        }
    }

    // mission.doctrine_states[]
    if (json.mission) {
        if (!json.mission.doctrine_states || !Array.isArray(json.mission.doctrine_states)) {
            errors.push('mission.doctrine_states[] manquant ou pas un tableau');
        } else if (json.mission.doctrine_states.length === 0) {
            errors.push('mission.doctrine_states[] est vide');
        } else {
            // Check each state has required fields
            json.mission.doctrine_states.forEach((s, i) => {
                if (!s.state) errors.push(`doctrine_states[${i}]: "state" manquant`);
                if (!s.behavior) errors.push(`doctrine_states[${i}]: "behavior" manquant`);
                if (!Array.isArray(s.transitions)) errors.push(`doctrine_states[${i}]: "transitions" manquant ou invalide`);
            });
        }

        // Zone
        if (json.mission.zone) {
            const z = json.mission.zone;
            if (z.n_min === undefined || z.n_max === undefined ||
                z.e_min === undefined || z.e_max === undefined) {
                errors.push('mission.zone incomplet (n_min/n_max/e_min/e_max requis)');
            }
        }
    }

    return { valid: errors.length === 0, errors };
}


/**
 * Importe un JSON doctrine autonome dans le DoctrineManager frontend.
 *
 * Ce qui est appliqué :
 * - Doctrine frontière (stigmergy/swarm/formation/coverage/search)
 * - COA (grid/spiral/perimeter/adaptive/...)
 * - Zone (center + size)
 * - Altitude
 * - Safety distance
 *
 * Ce qui est conservé pour info :
 * - FSM states (stockés dans .importedDoctrineStates)
 * - AI config (stocké dans .importedAIConfig)
 * - Safety config (stocké dans .importedSafetyConfig)
 *
 * @param {DoctrineManager} manager
 * @param {Object} json - Doctrine JSON autonome
 * @returns {{ success: boolean, applied: string[], warnings: string[] }}
 */
export function importDoctrineFromJSON(manager, json) {
    const validation = validateDoctrineJSON(json);
    if (!validation.valid) {
        console.error('❌ Doctrine JSON invalide:', validation.errors);
        return { success: false, applied: [], warnings: validation.errors };
    }

    const applied = [];
    const warnings = [];

    // ── 1. Déduire la doctrine frontend depuis le mission type ou les tags ──
    let doctrineId = null;

    // A. From colab_source (if re-importing an export)
    if (json.colab_source?.doctrine_id) {
        doctrineId = json.colab_source.doctrine_id;
    }

    // B. From mission.type
    if (!doctrineId && json.mission?.type) {
        doctrineId = MISSION_TYPE_TO_DOCTRINE[json.mission.type];
    }

    // C. From tags
    if (!doctrineId && json.doctrine?.tags) {
        const tags = json.doctrine.tags;
        for (const [tag, did] of Object.entries(MISSION_TYPE_TO_DOCTRINE)) {
            if (tags.includes(tag)) {
                doctrineId = did;
                break;
            }
        }
        // Direct doctrine id in tags
        if (!doctrineId) {
            for (const d of Object.values(DOCTRINES)) {
                if (tags.includes(d.id)) {
                    doctrineId = d.id;
                    break;
                }
            }
        }
    }

    if (doctrineId) {
        manager.setDoctrine(doctrineId);
        applied.push(`Doctrine: ${doctrineId}`);
    } else {
        warnings.push('Impossible de déterminer la doctrine frontend, stigmergy par défaut');
        manager.setDoctrine('stigmergy');
        applied.push('Doctrine: stigmergy (défaut)');
    }

    // ── 2. Déduire le COA depuis le behavior principal ──
    let coaId = null;

    // A. From colab_source
    if (json.colab_source?.coa_id) {
        coaId = json.colab_source.coa_id;
    }

    // B. From the main exploration state behavior
    if (!coaId && json.mission?.doctrine_states) {
        const mainState = json.mission.doctrine_states.find(s =>
            ['EXPLORE', 'SEARCH', 'PATROL', 'SURVEY'].includes(s.state)
        );
        if (mainState) {
            coaId = BEHAVIOR_TO_COA[mainState.behavior];
        }
    }

    if (coaId) {
        manager.setCOA(coaId);
        applied.push(`COA: ${coaId}`);
    } else {
        warnings.push('Impossible de déterminer le COA, adaptatif par défaut');
        manager.setCOA('adaptive');
        applied.push('COA: adaptive (défaut)');
    }

    // ── 3. Zone ──
    if (json.mission?.zone) {
        const frontendZone = nedToZone(json.mission.zone);
        manager.zoneParams.centerX = frontendZone.centerX;
        manager.zoneParams.centerZ = frontendZone.centerZ;
        manager.zoneParams.sizeX = frontendZone.sizeX;
        manager.zoneParams.sizeZ = frontendZone.sizeZ;
        applied.push(`Zone: ${frontendZone.sizeX}×${frontendZone.sizeZ}m`);
    }

    // ── 4. Altitude ──
    if (json.mission?.altitude_m) {
        manager.zoneParams.altitude = json.mission.altitude_m;
        applied.push(`Altitude: ${json.mission.altitude_m}m`);
    }

    // ── 5. Safety distance ──
    if (json.safety?.emergency_stop_m) {
        manager.zoneParams.safetyDistance = json.safety.emergency_stop_m;
        applied.push(`Safety distance: ${json.safety.emergency_stop_m}m`);
    }

    // ── 6. Formation (if formation doctrine) ──
    if (json.colab_source?.formation_id) {
        manager.setFormation(json.colab_source.formation_id);
        applied.push(`Formation: ${json.colab_source.formation_id}`);
    }

    // ── 7. Stocker les données non-mappées pour référence ──
    manager.importedDoctrine = json;
    manager.importedDoctrineStates = json.mission?.doctrine_states || [];
    manager.importedAIConfig = json.ai || null;
    manager.importedSafetyConfig = json.safety || null;
    manager.importedAt = new Date().toISOString();

    // ── 8. Appliquer visuellement ──
    manager.applyDoctrineParams();
    manager.applyCOAParams();
    manager.notifyListeners('doctrineImported', {
        name: json.doctrine?.name,
        states: json.mission?.doctrine_states?.length || 0,
        applied,
        warnings
    });

    // ── 9. Log résumé ──
    console.log('📥 Doctrine importée:', json.doctrine?.name);
    console.log('   ✅ Appliqué:', applied.join(', '));
    if (warnings.length > 0) {
        console.warn('   ⚠️ Warnings:', warnings.join(', '));
    }
    console.log(`   📊 FSM: ${json.mission?.doctrine_states?.length || 0} états chargés`);

    return { success: true, applied, warnings };
}


/**
 * Ouvre un file picker pour charger un fichier doctrine JSON.
 *
 * @param {DoctrineManager} manager
 * @returns {Promise<{ success: boolean, applied: string[], warnings: string[] }>}
 */
export function importDoctrineFromFile(manager) {
    return new Promise((resolve) => {
        const input = document.createElement('input');
        input.type = 'file';
        input.accept = '.json';
        input.style.display = 'none';

        input.addEventListener('change', async (e) => {
            const file = e.target.files[0];
            if (!file) {
                resolve({ success: false, applied: [], warnings: ['Aucun fichier sélectionné'] });
                return;
            }

            try {
                const text = await file.text();
                const json = JSON.parse(text);
                const result = importDoctrineFromJSON(manager, json);
                resolve(result);
            } catch (err) {
                console.error('❌ Erreur import doctrine:', err);
                resolve({
                    success: false,
                    applied: [],
                    warnings: [`Erreur de parsing: ${err.message}`]
                });
            } finally {
                document.body.removeChild(input);
            }
        });

        input.addEventListener('cancel', () => {
            resolve({ success: false, applied: [], warnings: ['Import annulé'] });
            document.body.removeChild(input);
        });

        document.body.appendChild(input);
        input.click();
    });
}


/**
 * Importe une doctrine depuis une URL (fetch).
 */
export async function importDoctrineFromURL(manager, url) {
    try {
        const response = await fetch(url);
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
        const json = await response.json();
        return importDoctrineFromJSON(manager, json);
    } catch (err) {
        console.error('❌ Erreur fetch doctrine:', err);
        return {
            success: false,
            applied: [],
            warnings: [`Erreur réseau: ${err.message}`]
        };
    }
}


export default importDoctrineFromJSON;
