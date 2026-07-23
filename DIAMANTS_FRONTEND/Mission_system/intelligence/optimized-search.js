/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Recherche optimisée — Stub public
 * ===================================
 * L'implémentation réelle (partitionnement, pivots, découpage par blocs)
 * vit dans le dépôt privé.
 *
 * `computeOptimalParams` est conservée à l'identique : c'est une formule
 * fermée, dérivée d'un résultat publié, sans valeur propriétaire. Elle est
 * importée statiquement par ui/optimal-search-ui.js.
 */

/**
 * Paramètres théoriques d'une recherche hiérarchique pour n éléments.
 * @param {number} n
 * @returns {{k:number, t:number, depth:number, blockSize:number, theoreticalSpeedup:number}}
 */
export function computeOptimalParams(n) {
    const logN = Math.log2(Math.max(n, 4));
    return {
        k: Math.max(2, Math.floor(Math.pow(logN, 1 / 3))),
        t: Math.max(2, Math.floor(Math.pow(logN, 2 / 3))),
        depth: Math.ceil(logN / Math.pow(logN, 2 / 3)),
        blockSize: Math.pow(2, Math.floor(Math.pow(logN, 2 / 3))),
        theoreticalSpeedup: Math.pow(logN, 1 / 3),
    };
}
