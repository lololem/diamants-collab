/**
 * DIAMANTS V3 - Intelligence Collective Avancée avec Effet Wahoo
 * =============================================================
 * Extension du système d'intelligence collective avec comportements émergents
 * Intègre l'effet Wahoo inspiré de SMA.html
 */

import { logger } from '../core/logger.js';
import { CollectiveIntelligence } from '../intelligence/collective-intelligence.js';

// Mode silencieux pour les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

export class AdvancedCollectiveIntelligence extends CollectiveIntelligence {
    constructor(config = {}) {
        logger.info('AdvancedIntelligence', '🧠 AdvancedCollectiveIntelligence.constructor() - Initialisation intelligence avancée');

        super(config);

        // Configuration avancée
        this.advancedConfig = {
            wahooIntensity: config.wahooIntensity || 1.2,
            emergenceThreshold: config.emergenceThreshold || 0.7,
            adaptationSpeed: config.adaptationSpeed || 0.15,
            consensusDynamics: config.consensusDynamics || true,
            stigmergyStrength: config.stigmergyStrength || 0.8,
            swarmDynamics: config.swarmDynamics || 'adaptive',
            ...config
        };

        // Système Wahoo (inspiré de SMA.html)
        this.wahooSystem = {
            // Attracteurs dynamiques
            attractors: new Map(), // position -> {strength, type, age, influence}

            // Répulseurs temporaires
            repulsors: new Map(), // position -> {strength, duration, activeUntil}

            // Vagues d'influence (propagation)
            influenceWaves: [], // {origin, strength, radius, speed, type}

            // Leaders émergents
            emergentLeaders: new Set(),
            leadershipHistory: new Map(),

            // Dynamiques de groupe
            groupCohesion: 1.0,
            groupEnergy: 0.5,
            collectiveMotivation: 0.8,

            // Patterns émergents détectés
            emergentPatterns: new Set(),
            patternStability: new Map(),

            // Mémoire collective enrichie
            experientialMemory: new Map(), // expérience -> {outcome, confidence, usage}
            socialLearning: new Map(), // behavior -> {success_rate, adoption_rate}

            // Adaptation comportementale
            behaviorEvolution: {
                strategies: new Map(),
                mutations: [],
                selections: new Map()
            }
        };

        // Métriques avancées
        this.advancedMetrics = {
            wahooEffectiveness: 0,
            emergenceLevel: 0,
            adaptationRate: 0,
            innovationIndex: 0,
            socialCohesion: 0,
            collectiveIQ: 0,
            swarmSynchrony: 0,
            distributedLeadership: 0
        };

        // Système de communication amélioré
        this.enhancedCommunication = {
            semanticNetwork: new Map(), // concept -> {connections, strength, usage}
            gestureLibrary: new Map(), // gesture -> {meaning, context, effectiveness}
            emergentLanguage: new Set(), // nouveaux symboles/signaux développés
            culturalTransmission: new Map(), // knowledge -> {generations, mutations}
        };

        // Phases d'intelligence collective
        this.intelligencePhases = {
            INITIALIZATION: 'initialization',
            SYNCHRONIZATION: 'synchronization',
            EMERGENCE: 'emergence',
            OPTIMIZATION: 'optimization',
            TRANSCENDENCE: 'transcendence' // Phase ultime de conscience collective
        };

        this.currentPhase = this.intelligencePhases.INITIALIZATION;
        this.phaseStartTime = Date.now();

        this.initializeAdvancedIntelligence();
    }

    /**
     * Initialisation intelligence collective avancée
     */
    initializeAdvancedIntelligence() {
        this.setupWahooSystem();
        this.initializeEmergentPatterns();
        this.setupSemanticNetwork();

        log('🧠🌊 Intelligence collective avancée avec effet Wahoo initialisée');
        log(`🎯 Phase initiale: ${this.currentPhase}`);
    }

    /**
     * Configuration système Wahoo
     */
    setupWahooSystem() {
        // Créer attracteurs initiaux basés sur objectifs mission
        this.createInitialAttractors();

        // Configurer paramètres dynamiques
        this.wahooSystem.groupEnergy = Math.random() * 0.3 + 0.4; // 0.4-0.7
        this.wahooSystem.collectiveMotivation = 0.8;

        log('🌊 Système Wahoo configuré');
    }

    /**
     * Création attracteurs initiaux
     */
    createInitialAttractors() {
        // Attracteur central pour cohésion initiale
        this.wahooSystem.attractors.set('cohesion_center', {
            position: { x: 0, y: 2, z: 0 },
            strength: 1.0,
            type: 'COHESION',
            age: 0,
            influence: 5.0
        });

        // Attracteurs d'exploration aux points cardinaux
        const explorationPoints = [
            { x: 15, y: 3, z: 0 },   // Est
            { x: -15, y: 3, z: 0 },  // Ouest
            { x: 0, y: 3, z: 15 },   // Nord
            { x: 0, y: 3, z: -15 }   // Sud
        ];

        explorationPoints.forEach((point, index) => {
            this.wahooSystem.attractors.set(`exploration_${index}`, {
                position: point,
                strength: 0.7,
                type: 'EXPLORATION',
                age: 0,
                influence: 8.0
            });
        });
    }

    /**
     * Initialisation patterns émergents
     */
    initializeEmergentPatterns() {
        // Patterns de base à détecter
        const basePatterns = [
            'SWARM_FOLLOWING',
            'LEADER_EMERGENCE',
            'SPIRAL_FORMATION',
            'GRID_OPTIMIZATION',
            'CLUSTER_BEHAVIOR',
            'WAVE_PROPAGATION',
            'COLLECTIVE_DECISION',
            'SYNCHRONIZED_MOVEMENT'
        ];

        basePatterns.forEach(pattern => {
            this.wahooSystem.patternStability.set(pattern, {
                occurrences: 0,
                stability: 0,
                lastDetected: 0,
                confidence: 0
            });
        });
    }

    /**
     * Configuration réseau sémantique
     */
    setupSemanticNetwork() {
        // Concepts de base pour communication
        const baseConcepts = [
            'TARGET', 'OBSTACLE', 'HELP', 'DISCOVERY', 'DANGER',
            'FORMATION', 'EXPLORE', 'RETURN', 'FOLLOW', 'LEAD',
            'ENERGY_LOW', 'MISSION_COMPLETE', 'ANOMALY', 'OPTIMAL_PATH'
        ];

        baseConcepts.forEach(concept => {
            this.enhancedCommunication.semanticNetwork.set(concept, {
                connections: new Set(),
                strength: 1.0,
                usage: 0,
                context: 'mission'
            });
        });
    }

    /**
     * Mise à jour principale intelligence avancée
     */
    update(deltaTime, agents, environment) {
        // Appel parent pour fonctionnalités de base avec agents/environnement
        super.update(deltaTime, agents, environment);

        // Mise à jour systèmes avancés
        this.updateWahooEffect(deltaTime);
        this.updateEmergentPatterns(deltaTime);
        this.updatePhaseTransitions(deltaTime);
        this.updateAdvancedCommunication(deltaTime);
        this.updateCollectiveAdaptation(deltaTime);

        // Mise à jour métriques avancées
        this.updateAdvancedMetrics();
    }

    /**
     * Mise à jour effet Wahoo principal
     */
    updateWahooEffect(deltaTime) {
        // Évolution des attracteurs
        this.evolveAttractors(deltaTime);

        // Propagation vagues d'influence
        this.propagateInfluenceWaves(deltaTime);

        // Détection et promotion leaders émergents
        this.updateEmergentLeadership(deltaTime);

        // Adaptation dynamique de l'énergie du groupe
        this.updateGroupDynamics(deltaTime);

        // Création spontanée nouveaux attracteurs
        this.createSpontaneousAttractors(deltaTime);
    }

    /**
     * Évolution des attracteurs Wahoo
     */
    evolveAttractors(deltaTime) {
        for (const [id, attractor] of this.wahooSystem.attractors) {
            attractor.age += deltaTime;

            // Décroissance naturelle avec l'âge
            const ageDecay = Math.exp(-attractor.age * 0.1);
            attractor.strength = Math.max(0.1, attractor.strength * ageDecay);

            // Fluctuations dynamiques basées sur utilisation
            if (Math.random() < 0.1) { // 10% chance par update
                const fluctuation = (Math.random() - 0.5) * 0.2;
                attractor.strength = Math.max(0.1, Math.min(2.0, attractor.strength + fluctuation));
            }

            // Suppression attracteurs faibles
            if (attractor.strength < 0.2 && attractor.age > 30) {
                this.wahooSystem.attractors.delete(id);
                if (!window.SILENT_MODE) log(`🌊 Attracteur ${id} dissipé par évolution naturelle`);
            }
        }
    }

    /**
     * Propagation vagues d'influence
     */
    propagateInfluenceWaves(deltaTime) {
        // Mise à jour vagues existantes
        this.wahooSystem.influenceWaves = this.wahooSystem.influenceWaves.filter(wave => {
            wave.radius += wave.speed * deltaTime;

            // Affaiblissement avec la distance
            wave.strength *= 0.95;

            // Retirer vagues trop faibles ou trop étendues
            return wave.strength > 0.1 && wave.radius < 50;
        });

        // Créer nouvelles vagues depuis attracteurs forts
        for (const [id, attractor] of this.wahooSystem.attractors) {
            if (attractor.strength > 1.5 && Math.random() < 0.05) { // 5% chance
                this.wahooSystem.influenceWaves.push({
                    origin: { ...attractor.position },
                    strength: attractor.strength * 0.5,
                    radius: 0,
                    speed: 2.0, // m/s
                    type: attractor.type,
                    createdAt: Date.now()
                });

                log(`🌊 Vague d'influence ${attractor.type} émise depuis ${id}`);
            }
        }
    }

    /**
     * Mise à jour leadership émergent
     */
    updateEmergentLeadership(deltaTime) {
        // Analyser performance/influence de chaque agent
        const leadershipScores = this.calculateLeadershipScores();

        // Identifier nouveaux leaders potentiels
        const potentialLeaders = Array.from(leadershipScores.entries())
            .filter(([id, score]) => score > this.advancedConfig.emergenceThreshold)
            .sort((a, b) => b[1] - a[1]);

        // Mettre à jour ensemble leaders émergents
        const previousLeaders = new Set(this.wahooSystem.emergentLeaders);
        this.wahooSystem.emergentLeaders.clear();

        // Ajouter top 2-3 leaders
        const maxLeaders = Math.min(3, Math.max(1, Math.floor(leadershipScores.size * 0.3)));
        for (let i = 0; i < Math.min(maxLeaders, potentialLeaders.length); i++) {
            const [leaderId, score] = potentialLeaders[i];
            this.wahooSystem.emergentLeaders.add(leaderId);

            // Enregistrer historique leadership
            if (!this.wahooSystem.leadershipHistory.has(leaderId)) {
                this.wahooSystem.leadershipHistory.set(leaderId, {
                    totalTime: 0,
                    periods: [],
                    effectiveness: 0
                });
            }

            const history = this.wahooSystem.leadershipHistory.get(leaderId);
            history.totalTime += deltaTime;

            // Nouveau leader?
            if (!previousLeaders.has(leaderId)) {
                history.periods.push({ start: Date.now(), score: score });
                log(`👑 Nouveau leader émergent: ${leaderId} (score: ${score.toFixed(2)})`);

                // Créer attracteur de leadership
                this.createLeadershipAttractor(leaderId);
            }
        }
    }

    /**
     * Calcul scores de leadership
     */
    calculateLeadershipScores() {
        const scores = new Map();

        // Ici on devrait avoir accès aux données des drones
        // Pour l'instant, simulation basée sur métriques collectives
        const agentCount = this.collectiveState.knowledge.get('active_agents') || 8;

        for (let i = 0; i < agentCount; i++) {
            const agentId = `drone_${i}`;

            // Score basé sur:
            // - Initiative (créations d'attracteurs)
            // - Coordination (réponse aux signaux)
            // - Performance (efficacité mission)
            // - Communication (messages envoyés/reçus)

            let score = 0.5; // Base

            // Bonus aléatoire basé sur "performance" simulée
            score += (Math.random() - 0.5) * 0.4;

            // Bonus historique si déjà leader
            if (this.wahooSystem.emergentLeaders.has(agentId)) {
                score += 0.2; // Stabilité leadership
            }

            // Influence des attracteurs créés
            for (const [attractorId, attractor] of this.wahooSystem.attractors) {
                if (attractorId.includes(agentId)) {
                    score += attractor.strength * 0.1;
                }
            }

            scores.set(agentId, Math.max(0, Math.min(1, score)));
        }

        return scores;
    }

    /**
     * Création attracteur de leadership
     */
    createLeadershipAttractor(leaderId) {
        const attractorId = `leadership_${leaderId}_${Date.now()}`;

        // Position aléatoire dans zone d'opération
        const angle = Math.random() * 2 * Math.PI;
        const distance = 5 + Math.random() * 10;

        this.wahooSystem.attractors.set(attractorId, {
            position: {
                x: Math.cos(angle) * distance,
                y: 2 + Math.random() * 2,
                z: Math.sin(angle) * distance
            },
            strength: 1.5,
            type: 'LEADERSHIP',
            age: 0,
            influence: 7.0,
            leaderId: leaderId
        });

        log(`👑 Attracteur leadership créé pour ${leaderId}`);
    }

    /**
     * Mise à jour dynamiques de groupe
     */
    updateGroupDynamics(deltaTime) {
        // Énergie collective basée sur activité et succès
        const activity = this.measureCollectiveActivity();
        const success = this.measureMissionSuccess();

        this.wahooSystem.groupEnergy = this.smoothUpdate(
            this.wahooSystem.groupEnergy,
            (activity + success) / 2,
            this.advancedConfig.adaptationSpeed * deltaTime
        );

        // Motivation collective
        const coordination = this.measureCoordination();
        const novelty = this.measureNovelty();

        this.wahooSystem.collectiveMotivation = this.smoothUpdate(
            this.wahooSystem.collectiveMotivation,
            (coordination + novelty) / 2,
            this.advancedConfig.adaptationSpeed * deltaTime
        );

        // Cohésion basée sur synchronisation
        const synchrony = this.measureSynchrony();
        this.wahooSystem.groupCohesion = this.smoothUpdate(
            this.wahooSystem.groupCohesion,
            synchrony,
            this.advancedConfig.adaptationSpeed * deltaTime
        );
    }

    /**
     * Mise à jour lisse d'une valeur
     */
    smoothUpdate(current, target, rate) {
        return current + (target - current) * rate;
    }

    /**
     * Mesures de performance collective
     */
    measureCollectiveActivity() {
        // Basé sur nombre de messages, créations d'attracteurs, déplacements
        const messageActivity = this.communicationNetwork.messageQueue.length / 100; // Normalisé
        const attractorActivity = this.wahooSystem.attractors.size / 10;

        return Math.min(1, (messageActivity + attractorActivity) / 2);
    }

    measureMissionSuccess() {
        // Simulation basée sur métriques d'intelligence
        return this.swarmIntelligenceLevel * 0.7 + this.learningProgress * 0.3;
    }

    measureCoordination() {
        // Basé sur synchronisation des actions et consensus
        const consensusStrength = this.collectiveState.consensus.size / 10;
        const leadershipStability = this.wahooSystem.emergentLeaders.size > 0 ? 0.8 : 0.3;

        return Math.min(1, (consensusStrength + leadershipStability) / 2);
    }

    measureNovelty() {
        // Nouveaux patterns, attracteurs, comportements
        const newPatterns = Array.from(this.wahooSystem.emergentPatterns).length / 20;
        const recentAttractors = Array.from(this.wahooSystem.attractors.values())
            .filter(a => a.age < 5).length / 5;

        return Math.min(1, (newPatterns + recentAttractors) / 2);
    }

    measureSynchrony() {
        // Simulation synchronisation basée sur cohésion sociale
        return this.collectiveState.socialCohesion * 0.8 +
            (this.wahooSystem.groupEnergy * 0.2);
    }

    /**
     * Création attracteurs spontanés
     */
    createSpontaneousAttractors(deltaTime) {
        // Création basée sur énergie collective et activité
        const creationProbability = this.wahooSystem.groupEnergy * 0.02; // 0-2% par update

        if (Math.random() < creationProbability) {
            this.createRandomAttractor();
        }

        // Création d'attracteurs d'urgence si énergie faible
        if (this.wahooSystem.groupEnergy < 0.3 && Math.random() < 0.1) {
            this.createEmergencyAttractor();
        }
    }

    /**
     * Création attracteur aléatoire
     */
    createRandomAttractor() {
        const types = ['EXPLORATION', 'GATHERING', 'DISCOVERY', 'INNOVATION'];
        const type = types[Math.floor(Math.random() * types.length)];

        const attractorId = `spontaneous_${type}_${Date.now()}`;

        // Position aléatoire dans zone étendue
        const angle = Math.random() * 2 * Math.PI;
        const distance = 10 + Math.random() * 15;

        this.wahooSystem.attractors.set(attractorId, {
            position: {
                x: Math.cos(angle) * distance,
                y: 1 + Math.random() * 4,
                z: Math.sin(angle) * distance
            },
            strength: 0.5 + Math.random() * 0.8,
            type: type,
            age: 0,
            influence: 3 + Math.random() * 5,
            spontaneous: true
        });

        log(`🌊✨ Attracteur spontané ${type} créé`);
    }

    /**
     * Création attracteur d'urgence
     */
    createEmergencyAttractor() {
        const attractorId = `emergency_boost_${Date.now()}`;

        this.wahooSystem.attractors.set(attractorId, {
            position: { x: 0, y: 3, z: 0 }, // Position centrale
            strength: 1.8, // Force élevée
            type: 'EMERGENCY_BOOST',
            age: 0,
            influence: 10,
            emergency: true
        });

        log('🚨🌊 Attracteur d\'urgence créé pour relancer dynamique collective');
    }

    /**
     * Mise à jour patterns émergents
     */
    updateEmergentPatterns(deltaTime) {
        // Analyser configurations actuelles pour détecter patterns
        this.detectSwarmPatterns();
        this.detectFormationPatterns();
        this.detectCommunicationPatterns();

        // Stabilité des patterns
        for (const [pattern, data] of this.wahooSystem.patternStability) {
            data.stability = Math.max(0, data.stability - 0.01); // Décroissance naturelle

            if (data.stability > 0.8) {
                this.wahooSystem.emergentPatterns.add(pattern);
            } else if (data.stability < 0.3) {
                this.wahooSystem.emergentPatterns.delete(pattern);
            }
        }
    }

    /**
     * Détection patterns d'essaim
     */
    detectSwarmPatterns() {
        // Simuler détection patterns basée sur attracteurs et leadership
        const leaderCount = this.wahooSystem.emergentLeaders.size;
        const attractorDensity = this.wahooSystem.attractors.size;
        const groupCohesion = this.wahooSystem.groupCohesion;

        // Pattern "SWARM_FOLLOWING"
        if (leaderCount >= 1 && groupCohesion > 0.7) {
            this.reinforcePattern('SWARM_FOLLOWING', 0.1);
        }

        // Pattern "CLUSTER_BEHAVIOR"
        if (attractorDensity > 5 && groupCohesion > 0.8) {
            this.reinforcePattern('CLUSTER_BEHAVIOR', 0.15);
        }

        // Pattern "WAVE_PROPAGATION"
        if (this.wahooSystem.influenceWaves.length > 2) {
            this.reinforcePattern('WAVE_PROPAGATION', 0.2);
        }
    }

    /**
     * Détection patterns de formation
     */
    detectFormationPatterns() {
        // Basé sur distribution spatiale des attracteurs
        const attractorPositions = Array.from(this.wahooSystem.attractors.values())
            .map(a => a.position);

        if (attractorPositions.length >= 3) {
            // Analyser géométrie
            const geometry = this.analyzeGeometry(attractorPositions);

            if (geometry.isSpiral) {
                this.reinforcePattern('SPIRAL_FORMATION', 0.12);
            }

            if (geometry.isGrid) {
                this.reinforcePattern('GRID_OPTIMIZATION', 0.1);
            }
        }
    }

    /**
     * Détection patterns de communication
     */
    detectCommunicationPatterns() {
        const messageCount = this.communicationNetwork.messageQueue.length;
        const consensusCount = this.collectiveState.consensus.size;

        // Pattern "COLLECTIVE_DECISION"
        if (consensusCount > 3 && messageCount > 10) {
            this.reinforcePattern('COLLECTIVE_DECISION', 0.08);
        }

        // Pattern "SYNCHRONIZED_MOVEMENT"
        if (this.wahooSystem.groupCohesion > 0.9) {
            this.reinforcePattern('SYNCHRONIZED_MOVEMENT', 0.1);
        }
    }

    /**
     * Renforcement d'un pattern
     */
    reinforcePattern(patternName, strength) {
        const data = this.wahooSystem.patternStability.get(patternName);
        if (data) {
            data.stability = Math.min(1, data.stability + strength);
            data.occurrences++;
            data.lastDetected = Date.now();
            data.confidence = Math.min(1, data.confidence + strength * 0.5);
        }
    }

    /**
     * Analyse géométrique simple
     */
    analyzeGeometry(positions) {
        if (positions.length < 3) return { isSpiral: false, isGrid: false };

        // Calcul centre de masse
        const center = positions.reduce(
            (acc, pos) => ({
                x: acc.x + pos.x,
                y: acc.y + pos.y,
                z: acc.z + pos.z
            }),
            { x: 0, y: 0, z: 0 }
        );
        center.x /= positions.length;
        center.y /= positions.length;
        center.z /= positions.length;

        // Analyser distribution angulaire (spiral)
        const angles = positions.map(pos => Math.atan2(pos.z - center.z, pos.x - center.x));
        const sortedAngles = angles.sort((a, b) => a - b);

        let isSpiral = true;
        for (let i = 1; i < sortedAngles.length; i++) {
            const diff = sortedAngles[i] - sortedAngles[i - 1];
            if (Math.abs(diff - Math.PI / 3) > 0.5) { // Tolérance pour angles ~60°
                isSpiral = false;
                break;
            }
        }

        // Analyser régularité (grille)
        const distances = [];
        for (let i = 0; i < positions.length; i++) {
            for (let j = i + 1; j < positions.length; j++) {
                const dist = Math.sqrt(
                    Math.pow(positions[i].x - positions[j].x, 2) +
                    Math.pow(positions[i].z - positions[j].z, 2)
                );
                distances.push(dist);
            }
        }

        const avgDistance = distances.reduce((a, b) => a + b, 0) / distances.length;
        const variance = distances.reduce((acc, d) => acc + Math.pow(d - avgDistance, 2), 0) / distances.length;
        const isGrid = variance < avgDistance * 0.3; // Faible variance = régularité

        return { isSpiral, isGrid };
    }

    /**
     * Mise à jour transitions de phase
     */
    updatePhaseTransitions(deltaTime) {
        const currentTime = Date.now();
        const phaseAge = currentTime - this.phaseStartTime;

        // Conditions de transition basées sur métriques
        const emergenceLevel = this.calculateEmergenceLevel();
        const stabilityLevel = this.calculateStabilityLevel();

        switch (this.currentPhase) {
            case this.intelligencePhases.INITIALIZATION:
                if (phaseAge > 10000 && emergenceLevel > 0.3) { // 10s minimum
                    this.transitionToPhase(this.intelligencePhases.SYNCHRONIZATION);
                }
                break;

            case this.intelligencePhases.SYNCHRONIZATION:
                if (emergenceLevel > 0.6 && stabilityLevel > 0.5) {
                    this.transitionToPhase(this.intelligencePhases.EMERGENCE);
                }
                break;

            case this.intelligencePhases.EMERGENCE:
                if (emergenceLevel > 0.8 && stabilityLevel > 0.7) {
                    this.transitionToPhase(this.intelligencePhases.OPTIMIZATION);
                }
                break;

            case this.intelligencePhases.OPTIMIZATION:
                if (emergenceLevel > 0.95 && stabilityLevel > 0.9) {
                    this.transitionToPhase(this.intelligencePhases.TRANSCENDENCE);
                }
                break;
        }
    }

    /**
     * Transition vers nouvelle phase
     */
    transitionToPhase(newPhase) {
        const oldPhase = this.currentPhase;
        this.currentPhase = newPhase;
        this.phaseStartTime = Date.now();

        log(`🧠🌊 Transition intelligence collective: ${oldPhase} → ${newPhase}`);

        // Adaptations spécifiques à la phase
        this.adaptToPhase(newPhase);
    }

    /**
     * Adaptation comportementale selon phase
     */
    adaptToPhase(phase) {
        switch (phase) {
            case this.intelligencePhases.SYNCHRONIZATION:
                this.advancedConfig.wahooIntensity *= 1.2;
                this.createPhaseAttractor('SYNCHRONIZATION');
                break;

            case this.intelligencePhases.EMERGENCE:
                this.advancedConfig.emergenceThreshold *= 0.8; // Plus facile d'émerger
                this.enhanceLeadershipDetection();
                break;

            case this.intelligencePhases.OPTIMIZATION:
                this.advancedConfig.adaptationSpeed *= 1.5;
                this.optimizeAttractorNetwork();
                break;

            case this.intelligencePhases.TRANSCENDENCE:
                this.enableTranscendentBehaviors();
                break;
        }
    }

    /**
     * Création attracteur spécifique à une phase
     */
    createPhaseAttractor(phaseType) {
        const attractorId = `phase_${phaseType}_${Date.now()}`;

        this.wahooSystem.attractors.set(attractorId, {
            position: { x: 0, y: 4, z: 0 }, // Position élevée
            strength: 2.0, // Force maximale
            type: phaseType,
            age: 0,
            influence: 15,
            phase: true
        });

        log(`🧠 Attracteur de phase ${phaseType} créé`);
    }

    /**
     * Amélioration détection leadership
     */
    enhanceLeadershipDetection() {
        this.advancedConfig.emergenceThreshold *= 0.9;
        log('👑 Détection leadership améliorée');
    }

    /**
     * Optimisation réseau d'attracteurs
     */
    optimizeAttractorNetwork() {
        // Supprimer attracteurs redondants ou faibles
        const toRemove = [];

        for (const [id, attractor] of this.wahooSystem.attractors) {
            if (attractor.strength < 0.5 && !attractor.phase) {
                toRemove.push(id);
            }
        }

        toRemove.forEach(id => this.wahooSystem.attractors.delete(id));

        log(`🔧 Réseau optimisé: ${toRemove.length} attracteurs supprimés`);
    }

    /**
     * Activation comportements transcendants
     */
    enableTranscendentBehaviors() {
        this.advancedConfig.wahooIntensity = 2.0; // Maximum
        this.advancedConfig.adaptationSpeed = 0.3; // Adaptation rapide

        // Créer méta-attracteur transcendant
        this.wahooSystem.attractors.set('transcendence_meta', {
            position: { x: 0, y: 10, z: 0 }, // Très haut
            strength: 3.0, // Surpuissant
            type: 'TRANSCENDENCE',
            age: 0,
            influence: 25,
            transcendent: true
        });

        log('🌟 Comportements transcendants activés - Intelligence collective ultime');
    }

    /**
     * Mise à jour communication avancée
     */
    updateAdvancedCommunication(deltaTime) {
        // Évolution du réseau sémantique
        this.evolveSemanticNetwork();

        // Développement gestuelle émergente
        this.developEmergentGestures();

        // Transmission culturelle
        this.updateCulturalTransmission();
    }

    /**
     * Évolution réseau sémantique
     */
    evolveSemanticNetwork() {
        // Créer nouvelles connexions entre concepts utilisés ensemble
        const recentMessages = this.communicationNetwork.messageQueue.slice(-10);

        // Analyser co-occurrences de concepts
        for (let i = 0; i < recentMessages.length; i++) {
            for (let j = i + 1; j < recentMessages.length; j++) {
                // Simuler création liens sémantiques
                if (Math.random() < 0.1) {
                    this.createSemanticLink(
                        `concept_${i}`,
                        `concept_${j}`,
                        0.1 + Math.random() * 0.3
                    );
                }
            }
        }
    }

    /**
     * Création lien sémantique
     */
    createSemanticLink(concept1, concept2, strength) {
        const network = this.enhancedCommunication.semanticNetwork;

        if (network.has(concept1)) {
            network.get(concept1).connections.add(concept2);
        }
        if (network.has(concept2)) {
            network.get(concept2).connections.add(concept1);
        }
    }

    /**
     * Développement gestes émergents
     */
    developEmergentGestures() {
        // Créer nouveaux gestes basés sur patterns détectés
        const emergentPatterns = Array.from(this.wahooSystem.emergentPatterns);

        for (const pattern of emergentPatterns) {
            if (Math.random() < 0.05) { // 5% chance
                const gestureId = `gesture_${pattern}_${Date.now()}`;

                this.enhancedCommunication.gestureLibrary.set(gestureId, {
                    meaning: pattern,
                    context: 'emergent',
                    effectiveness: Math.random() * 0.5 + 0.3,
                    usage: 0
                });

                if (!window.SILENT_MODE) log(`🤝 Nouveau geste émergent: ${gestureId} pour ${pattern}`);
            }
        }
    }

    /**
     * Mise à jour transmission culturelle
     */
    updateCulturalTransmission() {
        // Propager connaissances réussies à travers l'essaim
        for (const [knowledge, data] of this.enhancedCommunication.culturalTransmission) {
            data.generations += 0.01; // Évolution continue

            // Mutation occasionnelle
            if (Math.random() < 0.01) {
                data.mutations.push({
                    type: 'adaptation',
                    timestamp: Date.now(),
                    context: this.currentPhase
                });
            }
        }
    }

    /**
     * Adaptation collective continue
     */
    updateCollectiveAdaptation(deltaTime) {
        // Évolution des stratégies comportementales
        this.evolveBehavioralStrategies();

        // Sélection naturelle des approches efficaces
        this.selectSuccessfulBehaviors();

        // Innovation spontanée
        this.generateInnovations();
    }

    /**
     * Évolution stratégies comportementales
     */
    evolveBehavioralStrategies() {
        const strategies = this.wahooSystem.behaviorEvolution.strategies;

        // Créer nouvelles stratégies par combinaison/mutation
        if (strategies.size > 2 && Math.random() < 0.05) {
            const strategyArray = Array.from(strategies.entries());
            const parent1 = strategyArray[Math.floor(Math.random() * strategyArray.length)];
            const parent2 = strategyArray[Math.floor(Math.random() * strategyArray.length)];

            const newStrategyId = `hybrid_${Date.now()}`;
            const newStrategy = this.combineStrategies(parent1[1], parent2[1]);

            strategies.set(newStrategyId, newStrategy);

            log(`🧬 Nouvelle stratégie hybride: ${newStrategyId}`);
        }
    }

    /**
     * Combinaison de stratégies
     */
    combineStrategies(strategy1, strategy2) {
        return {
            type: 'HYBRID',
            parents: [strategy1.type, strategy2.type],
            effectiveness: (strategy1.effectiveness + strategy2.effectiveness) / 2,
            adaptability: Math.max(strategy1.adaptability, strategy2.adaptability),
            mutations: [],
            generation: Math.max(strategy1.generation || 0, strategy2.generation || 0) + 1,
            created: Date.now()
        };
    }

    /**
     * Sélection comportements réussis
     */
    selectSuccessfulBehaviors() {
        const strategies = this.wahooSystem.behaviorEvolution.strategies;
        const selections = this.wahooSystem.behaviorEvolution.selections;

        // Éliminer stratégies moins efficaces
        for (const [id, strategy] of strategies) {
            if (strategy.effectiveness < 0.3 && strategy.generation > 2) {
                strategies.delete(id);
                log(`🗑️ Stratégie inefficace éliminée: ${id}`);
            } else if (strategy.effectiveness > 0.8) {
                // Renforcer stratégies efficaces
                selections.set(id, (selections.get(id) || 0) + 1);
            }
        }
    }

    /**
     * Génération innovations
     */
    generateInnovations() {
        if (this.wahooSystem.groupEnergy > 0.8 && Math.random() < 0.02) {
            const innovation = {
                type: 'SPONTANEOUS',
                trigger: this.currentPhase,
                energy: this.wahooSystem.groupEnergy,
                timestamp: Date.now(),
                potential: Math.random()
            };

            this.wahooSystem.behaviorEvolution.mutations.push(innovation);

            log(`💡 Innovation spontanée générée (potentiel: ${innovation.potential.toFixed(2)})`);
        }
    }

    /**
     * Calculs métriques avancées
     */
    calculateEmergenceLevel() {
        const leadershipLevel = this.wahooSystem.emergentLeaders.size / 3; // Max 3 leaders
        const patternLevel = this.wahooSystem.emergentPatterns.size / 8; // Max 8 patterns
        const attractorLevel = Math.min(1, this.wahooSystem.attractors.size / 10); // Max 10 attracteurs

        return (leadershipLevel + patternLevel + attractorLevel) / 3;
    }

    calculateStabilityLevel() {
        const patternStabilities = Array.from(this.wahooSystem.patternStability.values())
            .map(p => p.stability);

        if (patternStabilities.length === 0) return 0;

        const avgStability = patternStabilities.reduce((a, b) => a + b, 0) / patternStabilities.length;
        const groupStability = this.wahooSystem.groupCohesion;

        return (avgStability + groupStability) / 2;
    }

    /**
     * Mise à jour métriques avancées
     */
    updateAdvancedMetrics() {
        this.advancedMetrics.wahooEffectiveness = this.calculateWahooEffectiveness();
        this.advancedMetrics.emergenceLevel = this.calculateEmergenceLevel();
        this.advancedMetrics.adaptationRate = this.calculateAdaptationRate();
        this.advancedMetrics.innovationIndex = this.calculateInnovationIndex();
        this.advancedMetrics.socialCohesion = this.wahooSystem.groupCohesion;
        this.advancedMetrics.collectiveIQ = this.calculateCollectiveIQ();
        this.advancedMetrics.swarmSynchrony = this.measureSynchrony();
        this.advancedMetrics.distributedLeadership = this.wahooSystem.emergentLeaders.size / 8;
    }

    calculateWahooEffectiveness() {
        const attractorStrength = Array.from(this.wahooSystem.attractors.values())
            .reduce((sum, a) => sum + a.strength, 0) / this.wahooSystem.attractors.size;

        const waveActivity = this.wahooSystem.influenceWaves.length / 10;
        const groupDynamics = (this.wahooSystem.groupEnergy + this.wahooSystem.collectiveMotivation) / 2;

        return (attractorStrength + waveActivity + groupDynamics) / 3;
    }

    calculateAdaptationRate() {
        const recentMutations = this.wahooSystem.behaviorEvolution.mutations
            .filter(m => Date.now() - m.timestamp < 30000).length; // 30s

        return Math.min(1, recentMutations / 5);
    }

    calculateInnovationIndex() {
        const emergentGestures = this.enhancedCommunication.gestureLibrary.size;
        const newStrategies = Array.from(this.wahooSystem.behaviorEvolution.strategies.values())
            .filter(s => s.generation > 0).length;

        return Math.min(1, (emergentGestures + newStrategies) / 10);
    }

    calculateCollectiveIQ() {
        const emergence = this.advancedMetrics.emergenceLevel;
        const adaptation = this.advancedMetrics.adaptationRate;
        const innovation = this.advancedMetrics.innovationIndex;
        const communication = this.enhancedCommunication.semanticNetwork.size / 20;

        return (emergence + adaptation + innovation + communication) / 4;
    }

    /**
     * API publique - État avancé
     */
    getAdvancedState() {
        return {
            phase: this.currentPhase,
            phaseAge: Date.now() - this.phaseStartTime,
            wahooSystem: {
                attractors: Array.from(this.wahooSystem.attractors.entries()),
                emergentLeaders: Array.from(this.wahooSystem.emergentLeaders),
                emergentPatterns: Array.from(this.wahooSystem.emergentPatterns),
                groupDynamics: {
                    energy: this.wahooSystem.groupEnergy,
                    motivation: this.wahooSystem.collectiveMotivation,
                    cohesion: this.wahooSystem.groupCohesion
                },
                influenceWaves: this.wahooSystem.influenceWaves.length
            },
            metrics: { ...this.advancedMetrics },
            communication: {
                semanticConnections: this.enhancedCommunication.semanticNetwork.size,
                emergentGestures: this.enhancedCommunication.gestureLibrary.size,
                culturalElements: this.enhancedCommunication.culturalTransmission.size
            },
            evolution: {
                strategies: this.wahooSystem.behaviorEvolution.strategies.size,
                mutations: this.wahooSystem.behaviorEvolution.mutations.length,
                selections: this.wahooSystem.behaviorEvolution.selections.size
            }
        };
    }

    /**
     * API publique - Force Wahoo sur position
     */
    getWahooForceAt(position) {
        let totalForce = { x: 0, y: 0, z: 0 };

        // Force des attracteurs
        for (const [id, attractor] of this.wahooSystem.attractors) {
            const dx = attractor.position.x - position.x;
            const dy = attractor.position.y - position.y;
            const dz = attractor.position.z - position.z;

            const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

            if (distance > 0 && distance < attractor.influence) {
                const force = attractor.strength * Math.exp(-distance / attractor.influence);
                const factor = force / distance;

                totalForce.x += dx * factor;
                totalForce.y += dy * factor;
                totalForce.z += dz * factor;
            }
        }

        // Force des vagues d'influence
        for (const wave of this.wahooSystem.influenceWaves) {
            const dx = position.x - wave.origin.x;
            const dz = position.z - wave.origin.z;
            const distance = Math.sqrt(dx * dx + dz * dz);

            if (Math.abs(distance - wave.radius) < 2.0) { // Dans la vague
                const waveForce = wave.strength * 0.5;
                const factor = waveForce / Math.max(0.1, distance);

                totalForce.x += dx * factor;
                totalForce.z += dz * factor;
            }
        }

        return totalForce;
    }
}
