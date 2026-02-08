/**
 * DIAMANTS - Swarm Intelligence Interface (Abstract)
 * =====================================================
 * ⚠️  Post v0-origin — interface publique (diamants-collab)
 *
 * Contrat abstrait que toute implémentation d'intelligence d'essaim
 * doit respecter pour s'intégrer à l'AutonomousFlightEngine.
 *
 * L'implémentation concrète (stigmergie, RL, LLM...) vit dans
 * diamants-private. Ici on ne définit que la FORME du slot.
 *
 * Usage :
 *   class MonMoteurStigmergique extends SwarmIntelligenceInterface {
 *       computeNextWaypoint(droneId, state, neighbors) { ... }
 *       updateEnvironment(droneId, position, observation) { ... }
 *   }
 *
 *   engine.setSwarmIntelligence(new MonMoteurStigmergique());
 */

export class SwarmIntelligenceInterface {
    constructor(config = {}) {
        this.config = config;
        this.enabled = true;
    }

    /**
     * Initialise le système avec la liste des drones et l'environnement.
     * Appelé une fois au démarrage.
     * @param {Map<string, object>} drones - Map droneId → DroneFlightState
     * @param {Array<{center: THREE.Vector3, radius: number}>} obstacles
     */
    initialize(drones, obstacles) {
        // Override in subclass
    }

    /**
     * Calcule le prochain waypoint pour un drone.
     * L'engine appelle cette méthode AVANT son propre waypoint generator.
     * Si elle retourne null, l'engine utilise son exploration par défaut.
     *
     * @param {string} droneId
     * @param {object} state - DroneFlightState (position, velocity, heading, phase...)
     * @param {Array<object>} neighbors - États des drones voisins
     * @returns {THREE.Vector3|null} - Waypoint suggéré, ou null pour laisser l'engine décider
     */
    computeNextWaypoint(droneId, state, neighbors) {
        return null; // Default: let engine handle it
    }

    /**
     * Modifie le vecteur de vélocité commandé.
     * Appelé APRÈS le PID, AVANT le clamp de vitesse.
     * Permet d'injecter des forces stigmergiques (attraction/répulsion de phéromones).
     *
     * @param {string} droneId
     * @param {object} velocity - { x, y, z } vélocité commandée (mutable)
     * @param {object} state - DroneFlightState
     * @returns {object} - Vélocité modifiée { x, y, z }
     */
    modulateVelocity(droneId, velocity, state) {
        return velocity; // Default: no modulation
    }

    /**
     * Met à jour l'environnement avec une observation du drone.
     * Appelé à chaque frame pour chaque drone actif.
     * C'est ici qu'on dépose les phéromones / marques stigmergiques.
     *
     * @param {string} droneId
     * @param {THREE.Vector3} position
     * @param {object} observation - { speed, heading, phase, nearbyDrones, nearbyTrees }
     */
    updateEnvironment(droneId, position, observation) {
        // Override in subclass
    }

    /**
     * Tick global — appelé une fois par frame après tous les drones.
     * Pour l'évaporation des phéromones, la diffusion, les calculs globaux.
     *
     * @param {number} dt - Delta time en secondes
     */
    tick(dt) {
        // Override in subclass
    }

    /**
     * Retourne des métriques pour le debug / UI.
     * @returns {object}
     */
    getMetrics() {
        return {
            name: 'none',
            enabled: this.enabled,
        };
    }

    /**
     * Sérialise l'état interne (pour sauvegarde / transmission).
     * @returns {object}
     */
    serialize() {
        return {};
    }

    /**
     * Restaure un état précédemment sérialisé.
     * @param {object} data
     */
    deserialize(data) {
        // Override in subclass
    }
}

/**
 * No-op implementation — used when no intelligence module is loaded.
 * Ensures the engine never crashes even without a swarm brain.
 */
export class NoopSwarmIntelligence extends SwarmIntelligenceInterface {
    getMetrics() {
        return { name: 'noop', enabled: false };
    }
}
