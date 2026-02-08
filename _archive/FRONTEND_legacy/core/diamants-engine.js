/**
 * DIAMANTS ENGINE - Moteur de calcul central
 * Implémentation authentique de la formule I(t) = ∬|∇(φ+σ)|dΩ
 *
 * NOTE: Utilise la variable globale THREE fournie par le CDN (r128).
 * Ne pas importer 'three' en ESM ici (échec sans bundler).
 */
const THREE = (typeof window !== 'undefined' && window.THREE) ? window.THREE : undefined;

export class DiamantEngine {
    constructor() {
        this.initialized = false;

        // Paramètres DIAMANTS
        this.phiStrength = 1.5;      // Force d'attraction φ
        this.sigmaStrength = 1.2;    // Force de répulsion σ
        this.gradientStep = 0.01;    // Pas pour différentiation numérique
        this.emergenceThreshold = 0.7;

        // Cache pour optimisation
        this.fieldCache = new Map();
        this.cacheMaxAge = 50; // ms

        // Métriques globales
        this.globalMetrics = {
            totalIntelligence: 0,
            averageIntelligence: 0,
            emergenceLevel: 0,
            cohesionIndex: 0,
            timestamp: Date.now()
        };

        console.log('🧠 DiamantEngine initialisé - Moteur DIAMANTS actif');
    }

    async initialize() {
        this.initialized = true;
        console.log('✅ DiamantEngine prêt pour calculs I(t) = ∬|∇(φ+σ)|dΩ');
        return true;
    }

    /**
     * Calcul du potentiel attractif φ (harmonique)
     * @param {THREE.Vector3} position - Position du drone
     * @param {Array} neighbors - Drones voisins
     * @returns {number} Potentiel φ
     */
    calculatePhi(position, neighbors) {
        if (!neighbors || neighbors.length === 0) return 0;

        let phi = 0;
        neighbors.forEach(neighbor => {
            if (neighbor.position && neighbor.position.distanceTo) {
                const distance = position.distanceTo(neighbor.position);
                if (distance > 0.1) { // Éviter division par zéro
                    // Potentiel attractif 1/r (harmonique)
                    phi += this.phiStrength / distance;
                }
            }
        });

        return Math.min(phi, 10); // Limiter pour stabilité
    }

    /**
     * Calcul du potentiel répulsif σ (chaos/exploration)
     * @param {THREE.Vector3} position - Position du drone
     * @param {Array} neighbors - Drones voisins
     * @param {number} scoutingPriority - Priorité d'exploration
     * @returns {number} Potentiel σ
     */
    calculateSigma(position, neighbors, scoutingPriority = 0) {
        if (!neighbors || neighbors.length === 0) return scoutingPriority;

        let sigma = scoutingPriority;
        neighbors.forEach(neighbor => {
            if (neighbor.position && neighbor.position.distanceTo) {
                const distance = position.distanceTo(neighbor.position);
                if (distance > 0.1) {
                    // Potentiel répulsif exponentiel pour éviter collisions
                    sigma += this.sigmaStrength * Math.exp(-distance / 2);
                }
            }
        });

        return Math.min(sigma, 15); // Limiter pour stabilité
    }

    /**
     * Calcul du gradient ∇(φ+σ) par différentiation numérique
     * @param {THREE.Vector3} position - Position du drone
     * @param {Array} neighbors - Drones voisins
     * @param {number} h - Pas de différentiation
     * @returns {THREE.Vector3} Gradient ∇(φ+σ)
     */
    calculateGradient(position, neighbors, h = 0.01) {
        const cacheKey = `grad_${position.x.toFixed(2)}_${position.y.toFixed(2)}_${position.z.toFixed(2)}_${neighbors.length}`;
        const cached = this.fieldCache.get(cacheKey);

        if (cached && Date.now() - cached.timestamp < this.cacheMaxAge) {
            return cached.gradient.clone();
        }

        // Échantillonnage du potentiel total φ + σ
        const potential = (x, y, z) => {
            const testPos = new THREE.Vector3(x, y, z);
            return this.calculatePhi(testPos, neighbors) + this.calculateSigma(testPos, neighbors);
        };

        // Différentiation numérique
        const gradX = (potential(position.x + h, position.y, position.z) -
            potential(position.x - h, position.y, position.z)) / (2 * h);

        const gradY = (potential(position.x, position.y + h, position.z) -
            potential(position.x, position.y - h, position.z)) / (2 * h);

        const gradZ = (potential(position.x, position.y, position.z + h) -
            potential(position.x, position.y, position.z - h)) / (2 * h);

        const gradient = new THREE.Vector3(gradX, gradY, gradZ);

        // Vérification de validité
        if (!isFinite(gradX) || !isFinite(gradY) || !isFinite(gradZ)) {
            gradient.set(0, 0, 0);
        }

        // Cache du résultat
        this.fieldCache.set(cacheKey, {
            gradient: gradient.clone(),
            timestamp: Date.now()
        });

        return gradient;
    }

    /**
     * Calcul de l'intelligence I(t) = |∇(φ+σ)|
     * @param {THREE.Vector3} gradient - Gradient calculé
     * @param {number} wahooFactor - Facteur d'amplification
     * @param {number} scoutingPriority - Priorité d'exploration
     * @returns {number} Intelligence I(t)
     */
    calculateIntelligence(gradient, wahooFactor = 1.0, scoutingPriority = 0) {
        if (!gradient || !gradient.length) return 0;

        const rawIntelligence = gradient.length();
        const intelligence = rawIntelligence * wahooFactor * (1.0 + scoutingPriority * 0.5);

        return Math.min(intelligence, 20); // Limiter pour affichage
    }

    /**
     * Calcul de l'émergence (amplification non-linéaire)
     * @param {number} intelligence - Intelligence I(t)
     * @param {number} collaborationFactor - Facteur de collaboration
     * @returns {number} Niveau d'émergence
     */
    calculateEmergence(intelligence, collaborationFactor = 1.0) {
        if (!intelligence || intelligence === 0) return 0;

        const emergence = Math.pow(intelligence * collaborationFactor, 1.2) / 10;
        return Math.min(emergence, 2.0);
    }

    /**
     * Calcul des métriques globales de l'essaim
     * @param {Array} drones - Tous les drones de l'essaim
     * @returns {Object} Métriques globales
     */
    updateGlobalMetrics(drones) {
        if (!drones || drones.length === 0) {
            this.globalMetrics = {
                totalIntelligence: 0,
                averageIntelligence: 0,
                emergenceLevel: 0,
                cohesionIndex: 0,
                timestamp: Date.now()
            };
            return this.globalMetrics;
        }

        // Calcul intelligence totale
        const totalIntelligence = drones.reduce((sum, drone) => {
            return sum + (drone.intelligence || 0);
        }, 0);

        // Calcul émergence moyenne
        const totalEmergence = drones.reduce((sum, drone) => {
            return sum + (drone.emergence || 0);
        }, 0);

        // Calcul cohésion spatiale
        const cohesionIndex = this.calculateSpatialCohesion(drones);

        this.globalMetrics = {
            totalIntelligence: totalIntelligence,
            averageIntelligence: totalIntelligence / drones.length,
            emergenceLevel: totalEmergence / drones.length,
            cohesionIndex: cohesionIndex,
            droneCount: drones.length,
            timestamp: Date.now()
        };

        return this.globalMetrics;
    }

    /**
     * Calcul de la cohésion spatiale de l'essaim
     * @param {Array} drones - Drones de l'essaim
     * @returns {number} Index de cohésion (0-1)
     */
    calculateSpatialCohesion(drones) {
        if (drones.length < 2) return 0.5;

        // Centre de masse
        const centerOfMass = new THREE.Vector3();
        drones.forEach(drone => {
            if (drone.position) {
                centerOfMass.add(drone.position);
            }
        });
        centerOfMass.divideScalar(drones.length);

        // Distance moyenne au centre
        let totalDistance = 0;
        let validDrones = 0;

        drones.forEach(drone => {
            if (drone.position) {
                totalDistance += drone.position.distanceTo(centerOfMass);
                validDrones++;
            }
        });

        if (validDrones === 0) return 0.5;

        const avgDistance = totalDistance / validDrones;
        const maxExpectedDistance = 30; // Rayon maximal attendu

        // Cohésion inversée : plus la distance est faible, plus la cohésion est élevée
        return Math.max(0.1, 1.0 - Math.min(1.0, avgDistance / maxExpectedDistance));
    }

    /**
     * Obtenir les métriques pour l'interface utilisateur
     * @returns {Object} Métriques formatées pour l'UI
     */
    getMetricsSnapshot() {
        return {
            totalIntelligence: this.globalMetrics.totalIntelligence.toFixed(2),
            averageIntelligence: this.globalMetrics.averageIntelligence.toFixed(2),
            emergenceLevel: this.globalMetrics.emergenceLevel.toFixed(2),
            cohesionIndex: this.globalMetrics.cohesionIndex.toFixed(2),
            droneCount: this.globalMetrics.droneCount || 0,
            lastUpdate: new Date(this.globalMetrics.timestamp).toLocaleTimeString()
        };
    }

    /**
     * Nettoyage du cache pour optimisation
     */
    cleanCache() {
        const now = Date.now();
        for (const [key, value] of this.fieldCache.entries()) {
            if (now - value.timestamp > this.cacheMaxAge * 10) {
                this.fieldCache.delete(key);
            }
        }
    }

    /**
     * Reset complet du moteur
     */
    reset() {
        this.fieldCache.clear();
        this.globalMetrics = {
            totalIntelligence: 0,
            averageIntelligence: 0,
            emergenceLevel: 0,
            cohesionIndex: 0,
            timestamp: Date.now()
        };
        console.log('🔄 DiamantEngine réinitialisé');
    }

    /**
     * Méthode pour récupérer toutes les métriques formatées pour l'UI
     */
    getMetricsForUI(agents) {
        if (!agents || agents.length === 0) {
            return {
                totalIntelligence: 0,
                avgIntelligence: 0,
                emergence: 0,
                coherence: 0,
                fieldEnergy: 0,
                harmonics: new Array(15).fill(0)
            };
        }

        let totalIntelligence = 0;
        let fieldEnergy = 0;
        const harmonics = new Array(15).fill(0);

        // Calcul des métriques pour chaque agent
        agents.forEach((agent) => {
            const pos = agent.position || (agent.getPosition && agent.getPosition()) || new THREE.Vector3();
            const gradient = this.calculateGradient(pos, agents);
            const intelligence = this.calculateIntelligence(gradient);
            totalIntelligence += intelligence;

            const phi = this.calculatePhi(pos, agents);
            const sigma = this.calculateSigma(pos, agents);
            fieldEnergy += phi * phi + sigma * sigma;

            // Harmoniques (approximation spectrale)
            for (let h = 0; h < 15; h++) {
                harmonics[h] += Math.sin((h + 1) * phi) * Math.cos((h + 1) * sigma);
            }
        });

        const avgIntelligence = totalIntelligence / agents.length;
        // Cohésion spatiale (0..1). Plus élevé => groupe serré
        const coherence = this.calculateSpatialCohesion(agents);
        // Émergence collective à partir de l'intelligence moyenne et de la cohérence
        const emergence = this.calculateEmergence(avgIntelligence, 0.8 + 0.4 * coherence);

        return {
            totalIntelligence,
            avgIntelligence,
            emergence,
            coherence,
            fieldEnergy: fieldEnergy / agents.length,
            harmonics: harmonics.map(h => h / agents.length)
        };
    }
}

export default DiamantEngine;
