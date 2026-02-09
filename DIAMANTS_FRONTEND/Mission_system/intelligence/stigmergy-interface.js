/**
 * DIAMANTS — Stigmergy Interface (Abstract)
 * =============================================
 * ⚠️  Post v0-origin — interface publique (diamants-collab)
 *
 * Contrat abstrait pour un moteur stigmergique intégré à
 * l'AutonomousFlightEngine via le slot SwarmIntelligenceInterface.
 *
 * La stigmergie est une forme de coordination indirecte :
 * les drones communiquent en modifiant leur environnement partagé
 * (grille de phéromones virtuelles) plutôt qu'en se parlant.
 *
 * Ce fichier définit :
 *   1. Les types de phéromones (PheromoneType)
 *   2. L'API de la grille (lecture/dépôt/évaporation)
 *   3. La classe abstraite StigmergyInterface (extends SwarmIntelligenceInterface)
 *
 * L'implémentation concrète (diffusion, gradient, optimisation)
 * vit dans diamants-private.
 *
 * Usage :
 *   import { StigmergyInterface, PheromoneType } from './stigmergy-interface.js';
 *
 *   class MonMoteur extends StigmergyInterface {
 *       readPheromone(x, z, type) { ... }
 *       depositPheromone(x, z, type, intensity) { ... }
 *       getPheromoneGradient(x, z, type) { ... }
 *       // ... etc
 *   }
 *
 *   engine.setSwarmIntelligence(new MonMoteur(config));
 */

import { SwarmIntelligenceInterface } from './swarm-intelligence-interface.js';

// ─── Pheromone Types ─────────────────────────────────────────────────
/**
 * Types de phéromones virtuelles.
 * Chaque type influence le comportement de l'essaim différemment.
 */
export const PheromoneType = Object.freeze({
    /** Déposée quand un drone explore une zone → répulsion (on y est déjà passé). */
    EXPLORATION: 'exploration',

    /** Déposée quand un drone détecte quelque chose d'intéressant → attraction. */
    INTEREST: 'interest',

    /** Zone dangereuse (obstacle détecté, crash évité) → forte répulsion. */
    DANGER: 'danger',

    /** Signal de ralliement (pour missions de rassemblement) → attraction. */
    RALLY: 'rally',

    /** Phéromone personnalisée — le moteur concret peut définir ses propres types. */
    CUSTOM: 'custom',
});


// ─── Stigmergy Interface (Abstract) ─────────────────────────────────
/**
 * Interface abstraite pour un moteur stigmergique.
 *
 * Hérite de SwarmIntelligenceInterface et ajoute les hooks spécifiques
 * à la stigmergie : lecture/écriture de phéromones, gradient, config.
 *
 * Le cycle par frame est :
 *   1. updateEnvironment(droneId, pos, obs) → appelle depositPheromone()
 *   2. computeNextWaypoint(droneId, state, neighbors) → lit getPheromoneGradient()
 *   3. modulateVelocity(droneId, vel, state) → ajoute des forces d'attraction/répulsion
 *   4. tick(dt) → évaporation + diffusion globales
 */
export class StigmergyInterface extends SwarmIntelligenceInterface {

    /**
     * @param {object} config
     * @param {number} config.gridResolution - Taille d'une cellule en mètres (défaut: 1.0)
     * @param {number} config.gridSize - Dimension de la grille en cellules (défaut: 100)
     * @param {number} config.evaporationRate - Taux d'évaporation par seconde [0,1] (défaut: 0.02)
     * @param {number} config.diffusionRate - Taux de diffusion vers les voisins [0,1] (défaut: 0.1)
     * @param {number} config.maxIntensity - Intensité max d'une cellule (défaut: 100)
     */
    constructor(config = {}) {
        super(config);
        this.gridResolution = config.gridResolution ?? 1.0;
        this.gridSize = config.gridSize ?? 100;
        this.evaporationRate = config.evaporationRate ?? 0.02;
        this.diffusionRate = config.diffusionRate ?? 0.1;
        this.maxIntensity = config.maxIntensity ?? 100;
    }

    // ─── Pheromone Grid API (abstract — must be overridden) ──────────

    /**
     * Lit la concentration d'une phéromone à une position donnée.
     * @param {number} x - Position monde X
     * @param {number} z - Position monde Z
     * @param {string} type - PheromoneType
     * @returns {number} Intensité [0, maxIntensity]
     */
    readPheromone(x, z, type) {
        return 0; // Override in implementation
    }

    /**
     * Dépose une phéromone à une position donnée.
     * @param {number} x - Position monde X
     * @param {number} z - Position monde Z
     * @param {string} type - PheromoneType
     * @param {number} intensity - Quantité à déposer (sera clampée à maxIntensity)
     */
    depositPheromone(x, z, type, intensity) {
        // Override in implementation
    }

    /**
     * Calcule le gradient d'une phéromone autour d'une position.
     * Le gradient pointe vers la concentration croissante.
     * @param {number} x - Position monde X
     * @param {number} z - Position monde Z
     * @param {string} type - PheromoneType
     * @returns {{ gx: number, gz: number, magnitude: number }}
     */
    getPheromoneGradient(x, z, type) {
        return { gx: 0, gz: 0, magnitude: 0 }; // Override in implementation
    }

    /**
     * Retourne une snapshot de la grille complète pour un type donné.
     * Utile pour debug / visualisation dans le frontend.
     * @param {string} type - PheromoneType
     * @returns {Float32Array|null} Grille 2D aplatie (row-major), ou null si non dispo.
     */
    getGrid(type) {
        return null; // Override in implementation
    }

    /**
     * Retourne la liste des types de phéromones actifs.
     * @returns {string[]}
     */
    getActiveTypes() {
        return []; // Override in implementation
    }

    // ─── Configuration API ───────────────────────────────────────────

    /**
     * Met à jour les paramètres de stigmergie à chaud.
     * @param {object} params - Sous-ensemble de la config du constructeur
     */
    updateParameters(params) {
        if (params.evaporationRate !== undefined) this.evaporationRate = params.evaporationRate;
        if (params.diffusionRate !== undefined) this.diffusionRate = params.diffusionRate;
        if (params.maxIntensity !== undefined) this.maxIntensity = params.maxIntensity;
    }

    // ─── Default getMetrics ──────────────────────────────────────────

    getMetrics() {
        return {
            name: 'stigmergy',
            enabled: this.enabled,
            gridResolution: this.gridResolution,
            gridSize: this.gridSize,
            evaporationRate: this.evaporationRate,
            diffusionRate: this.diffusionRate,
            activeTypes: this.getActiveTypes(),
        };
    }
}
