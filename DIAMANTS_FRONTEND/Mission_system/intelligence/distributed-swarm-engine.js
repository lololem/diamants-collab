/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DistributedSwarmEngine — Stub public
 * ======================================
 * L'implémentation réelle (un agent autonome par drone, grille et journal
 * locaux, échanges par radio simulée) vit dans le dépôt privé.
 *
 * Ce fichier n'exporte VOLONTAIREMENT aucune classe `DistributedSwarmEngine`.
 * stigmergy-loader.js l'essaie en premier, ne trouve rien, et poursuit vers
 * les options suivantes puis son repli. Sa seule raison d'être est de
 * permettre au bundler de résoudre l'import dynamique.
 */

// Aucun export nommé DistributedSwarmEngine => le loader poursuit.
export default null;
