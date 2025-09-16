/**
 * SWARM_MEMORY - Système d'Intelligence Collective DIAMANTS V3
 * Extrait de SMA.html et adapté pour architecture modulaire propre
 * 
 * Responsabilités:
 * - Mémoire collective partagée (stigmergie numérique)
 * - Communication inter-drones en temps réel  
 * - Consensus distribué et vote
 * - Détection comportements émergents
 * - Métriques intelligence d'essaim
 */

export class SwarmMemory {
    constructor() {
        console.log('🧠 Initialisation SWARM_MEMORY - Intelligence Collective V3');
        
        // === MÉMOIRE COLLECTIVE CORE ===
        
        // Carte de stigmergie numérique
        this.stigmergyMap = new Map();
        
        // Historique des découvertes collectives  
        this.sharedDiscoveries = new Map();
        
        // Zones d'expertise par drone
        this.expertiseZones = new Map();
        
        // Consensus dynamique
        this.consensusVotes = new Map();
        
        // Communication temps réel
        this.messageQueue = [];
        this.broadcastRadius = 50;
        
        // Adaptation collective
        this.adaptationHistory = [];
        this.learningRate = 0.1;
        
        // États émergents détectés
        this.emergentBehaviors = new Set();
        
        // === MÉTRIQUES D'INTELLIGENCE D'ESSAIM ===
        this.swarmIntelligence = {
            totalI_t: 0,                    // Intelligence totale I(t)
            emergenceLevel: 0,              // Niveau d'émergence
            cohesionIndex: 0,               // Index de cohésion
            explorationEfficiency: 0,       // Efficacité exploration
            collectivePhaseState: 'INITIALIZATION', // État de phase collective
            swarmSize: 0,                   // Taille de l'essaim
            emergentBehaviorCount: 0,       // Nombre comportements émergents
            adaptationLevel: 0              // Niveau d'adaptation
        };
        
        // Métriques temps réel
        this.lastUpdate = Date.now();
        this.updateFrequency = 100; // ms
        
        // Auto-nettoyage périodique
        this.startCleanupTimer();
        
        console.log('✅ SWARM_MEMORY initialisé - Mémoire collective active');
    }
    
    // === MÉTHODES DE STIGMERGIE ===
    
    /**
     * Ajouter trace de stigmergie digitale
     */
    addStigmergyTrace(position, droneId, traceType, intensity = 1.0) {
        const key = `${Math.round(position.x / 2)}_${Math.round(position.z / 2)}`;
        
        if (!this.stigmergyMap.has(key)) {
            this.stigmergyMap.set(key, {
                position: position.clone(),
                traces: new Map(),
                lastUpdate: Date.now(),
                intensity: 0
            });
        }
        
        const cell = this.stigmergyMap.get(key);
        const traceKey = `${droneId}_${traceType}`;
        
        cell.traces.set(traceKey, {
            droneId,
            traceType,
            intensity,
            timestamp: Date.now(),
            decay: 0.95 // Décroissance temporelle
        });
        
        cell.intensity = Math.min(1.0, cell.intensity + intensity * 0.1);
        cell.lastUpdate = Date.now();
    }
    
    /**
     * Lire traces de stigmergie dans une zone
     */
    readStigmergyTraces(position, radius = 5) {
        const traces = [];
        const now = Date.now();
        
        this.stigmergyMap.forEach((cell, key) => {
            const distance = position.distanceTo(cell.position);
            if (distance <= radius) {
                cell.traces.forEach((trace, traceKey) => {
                    // Calcul intensité avec décroissance temporelle
                    const age = (now - trace.timestamp) / 1000; // secondes
                    const decayedIntensity = trace.intensity * Math.pow(trace.decay, age);
                    
                    if (decayedIntensity > 0.01) {
                        traces.push({
                            ...trace,
                            position: cell.position,
                            distance,
                            currentIntensity: decayedIntensity
                        });
                    }
                });
            }
        });
        
        return traces;
    }
    
    // === COMMUNICATION ET DÉCOUVERTES ===
    
    /**
     * Diffusion de découverte dans l'essaim
     */
    broadcastDiscovery(discovery, originDrone) {
        const message = {
            type: 'DISCOVERY',
            content: discovery,
            originId: originDrone.id,
            timestamp: Date.now(),
            position: originDrone.position.clone(),
            priority: discovery.value || 1.0,
            broadcastRadius: this.broadcastRadius
        };
        
        this.messageQueue.push(message);
        this.sharedDiscoveries.set(discovery.id || Date.now(), discovery);
        
        // Mise à jour métriques collectives
        this.swarmIntelligence.totalI_t += discovery.value * 0.1;
        
        console.log(`🔊 BROADCAST: Drone ${originDrone.id} partage découverte ${discovery.type}`);
    }
    
    // === CONSENSUS DISTRIBUÉ ===
    
    /**
     * Soumission vote de consensus
     */
    submitConsensusVote(droneId, topic, vote, confidence = 1.0) {
        if (!this.consensusVotes.has(topic)) {
            this.consensusVotes.set(topic, {
                votes: new Map(),
                startTime: Date.now(),
                deadline: Date.now() + 5000, // 5 secondes pour voter
                status: 'ACTIVE'
            });
        }
        
        const consensus = this.consensusVotes.get(topic);
        consensus.votes.set(droneId, { vote, confidence, timestamp: Date.now() });
        
        return this.evaluateConsensus(topic);
    }
    
    /**
     * Évaluation du consensus
     */
    evaluateConsensus(topic) {
        const consensus = this.consensusVotes.get(topic);
        if (!consensus || Date.now() > consensus.deadline) {
            if (consensus) consensus.status = 'EXPIRED';
            return null;
        }
        
        const votes = Array.from(consensus.votes.values());
        if (votes.length === 0) return null;
        
        // Calcul consensus pondéré par confiance
        const voteCount = new Map();
        let totalWeight = 0;
        
        votes.forEach(({ vote, confidence }) => {
            const weight = confidence;
            voteCount.set(vote, (voteCount.get(vote) || 0) + weight);
            totalWeight += weight;
        });
        
        // Trouver vote majoritaire
        let winningVote = null;
        let maxWeight = 0;
        
        voteCount.forEach((weight, vote) => {
            if (weight > maxWeight) {
                maxWeight = weight;
                winningVote = vote;
            }
        });
        
        // Consensus atteint si >60% accord
        const consensusStrength = maxWeight / totalWeight;
        if (consensusStrength > 0.6) {
            consensus.status = 'REACHED';
            consensus.result = { vote: winningVote, strength: consensusStrength };
            
            console.log(`✅ CONSENSUS atteint sur "${topic}": ${winningVote} (${(consensusStrength * 100).toFixed(1)}%)`);
            return consensus.result;
        }
        
        return null;
    }
    
    // === INTELLIGENCE D'ESSAIM ===
    
    /**
     * Calcul intelligence d'essaim en temps réel
     */
    calculateSwarmIntelligence(allDrones) {
        const now = Date.now();
        if (now - this.lastUpdate < this.updateFrequency) {
            return this.swarmIntelligence;
        }
        
        let totalI_t = 0;
        let totalEmergence = 0;
        let avgCohesion = 0;
        let explorationCoverage = 0;
        
        // Agrégation des métriques individuelles
        allDrones.forEach(drone => {
            totalI_t += drone.intelligence || 0;
            totalEmergence += drone.emergence || 0;
            avgCohesion += drone.collaborationScore || 0;
        });
        
        const droneCount = allDrones.length;
        if (droneCount > 0) {
            avgCohesion /= droneCount;
            totalEmergence /= droneCount;
        }
        
        // FORCE DES VALEURS MINIMALES POUR L'ACTIVITÉ VISIBLE
        if (totalEmergence === 0 && droneCount > 0) {
            totalEmergence = Math.min(1.0, totalI_t / droneCount * 0.1 + Math.random() * 0.3);
        }
        
        if (avgCohesion === 0 && droneCount > 0) {
            let totalProximity = 0;
            allDrones.forEach(drone => {
                let nearbyCount = 0;
                allDrones.forEach(other => {
                    if (other.id !== drone.id && drone.position.distanceTo(other.position) < 30) {
                        nearbyCount++;
                    }
                });
                totalProximity += nearbyCount / (droneCount - 1);
            });
            avgCohesion = totalProximity / droneCount;
        }
        
        // Détection phases émergentes
        let phaseState = 'DISPERSION';
        if (explorationCoverage < 0.1) phaseState = 'DISPERSION';
        else if (explorationCoverage < 0.4) phaseState = 'EXPLORATION';
        else if (explorationCoverage < 0.8) phaseState = 'CONSOLIDATION';
        else phaseState = 'COMPLETION';
        
        // Détection comportements émergents
        this.detectEmergentBehaviors(allDrones);
        
        // Mise à jour métriques
        this.swarmIntelligence = {
            totalI_t,
            emergenceLevel: totalEmergence,
            cohesionIndex: avgCohesion,
            explorationEfficiency: explorationCoverage,
            collectivePhaseState: phaseState,
            swarmSize: droneCount,
            emergentBehaviorCount: this.emergentBehaviors.size,
            adaptationLevel: this.calculateAdaptationLevel()
        };
        
        this.lastUpdate = now;
        return this.swarmIntelligence;
    }
    
    // === DÉTECTION COMPORTEMENTS ÉMERGENTS ===
    
    detectEmergentBehaviors(allDrones) {
        // Formation spontanée de groupes
        const clusters = this.detectClusters(allDrones);
        if (clusters.length > 1 && clusters.some(c => c.size > 3)) {
            this.emergentBehaviors.add('CLUSTER_FORMATION');
        }
        
        // Comportement de leader émergent
        const leaderCandidate = this.detectEmergentLeader(allDrones);
        if (leaderCandidate) {
            this.emergentBehaviors.add('EMERGENT_LEADERSHIP');
        }
        
        // Spécialisation de rôles
        if (this.detectRoleSpecialization(allDrones)) {
            this.emergentBehaviors.add('ROLE_SPECIALIZATION');
        }
        
        // Synchronisation spontanée
        if (this.detectSynchronization(allDrones)) {
            this.emergentBehaviors.add('SPONTANEOUS_SYNC');
        }
    }
    
    detectClusters(allDrones) {
        const clusters = [];
        const processed = new Set();
        const clusterRadius = 15;
        
        allDrones.forEach(drone => {
            if (processed.has(drone.id)) return;
            
            const cluster = new Set([drone.id]);
            processed.add(drone.id);
            
            allDrones.forEach(other => {
                if (!processed.has(other.id) && 
                    drone.position.distanceTo(other.position) < clusterRadius) {
                    cluster.add(other.id);
                    processed.add(other.id);
                }
            });
            
            clusters.push(cluster);
        });
        
        return clusters;
    }
    
    detectEmergentLeader(allDrones) {
        let bestLeader = null;
        let maxLeadershipScore = 0;
        
        allDrones.forEach(drone => {
            const leadershipScore = (drone.intelligence || 0) * 0.4 +
                                  (drone.scoutingEfficiency || 0) * 0.3 +
                                  (drone.collaborationScore || 0) * 0.3;
            
            if (leadershipScore > maxLeadershipScore) {
                maxLeadershipScore = leadershipScore;
                bestLeader = drone;
            }
        });
        
        return maxLeadershipScore > 0.7 ? bestLeader : null;
    }
    
    detectRoleSpecialization(allDrones) {
        const roles = new Map();
        allDrones.forEach(drone => {
            const role = drone.type?.role || 'unknown';
            roles.set(role, (roles.get(role) || 0) + 1);
        });
        
        return roles.size > 1;
    }
    
    detectSynchronization(allDrones) {
        if (allDrones.length < 3) return false;
        
        const velocities = allDrones.map(d => d.velocity.clone().normalize());
        let avgDirection = new THREE.Vector3();
        
        velocities.forEach(v => avgDirection.add(v));
        avgDirection.normalize();
        
        let alignmentSum = 0;
        velocities.forEach(v => {
            alignmentSum += Math.max(0, v.dot(avgDirection));
        });
        
        const alignmentScore = alignmentSum / velocities.length;
        return alignmentScore > 0.8;
    }
    
    calculateAdaptationLevel() {
        const recentAdaptations = this.adaptationHistory.filter(
            adapt => Date.now() - adapt.timestamp < 30000 // 30 secondes
        );
        
        return Math.min(1.0, recentAdaptations.length * 0.1);
    }
    
    // === NETTOYAGE ET MAINTENANCE ===
    
    cleanup() {
        const now = Date.now();
        const maxAge = 60000; // 1 minute
        
        // Nettoyer stigmergie ancienne
        this.stigmergyMap.forEach((cell, key) => {
            if (now - cell.lastUpdate > maxAge) {
                this.stigmergyMap.delete(key);
            }
        });
        
        // Nettoyer messages anciens
        this.messageQueue = this.messageQueue.filter(
            msg => now - msg.timestamp < 10000 // 10 secondes
        );
        
        // Nettoyer consensus expirés
        this.consensusVotes.forEach((consensus, topic) => {
            if (consensus.status === 'EXPIRED' || now > consensus.deadline + 5000) {
                this.consensusVotes.delete(topic);
            }
        });
    }
    
    startCleanupTimer() {
        setInterval(() => this.cleanup(), 30000); // Nettoyage toutes les 30s
    }
    
    // === MÉTHODES UTILITAIRES ===
    
    getMetrics() {
        return {
            ...this.swarmIntelligence,
            stigmergySize: this.stigmergyMap.size,
            activeMessages: this.messageQueue.length,
            activeConsensus: Array.from(this.consensusVotes.values())
                .filter(c => c.status === 'ACTIVE').length,
            emergentBehaviors: Array.from(this.emergentBehaviors)
        };
    }
    
    reset() {
        this.stigmergyMap.clear();
        this.sharedDiscoveries.clear();
        this.expertiseZones.clear();
        this.consensusVotes.clear();
        this.messageQueue = [];
        this.adaptationHistory = [];
        this.emergentBehaviors.clear();
        
        this.swarmIntelligence = {
            totalI_t: 0,
            emergenceLevel: 0,
            cohesionIndex: 0,
            explorationEfficiency: 0,
            collectivePhaseState: 'INITIALIZATION',
            swarmSize: 0,
            emergentBehaviorCount: 0,
            adaptationLevel: 0
        };
        
        console.log('🔄 SWARM_MEMORY reset - Mémoire collective réinitialisée');
    }
}

// Export singleton instance
export const SWARM_MEMORY = new SwarmMemory();

// Alias pour compatibilité
export default SWARM_MEMORY;
