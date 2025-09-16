/**
 * DIAMANTS - Intelligence Comportementale Collective
 * ===================================================
 * Système d'intelligence distribué basé sur les formules DIAMANTS
 */

import { DiamantFormulas } from '../core/diamants-formulas.js';

// Mode silencieux global
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

export class CollectiveIntelligence {
    constructor(config = {}) {
        this.config = {
            swarmSize: config.swarmSize || 8,
            communicationRange: config.communicationRange || 5.0,
            decisionThreshold: config.decisionThreshold || 0.7,
            adaptationRate: config.adaptationRate || 0.1,
            ...config
        };

        this.diamantFormulas = new DiamantFormulas();

        // État collectif
        this.collectiveState = {
            knowledge: new Map(),           // Connaissance partagée
            consensus: new Map(),           // Consensus de groupe
            emergentPatterns: [],           // Motifs émergents détectés
            socialCohesion: 1.0,           // Cohésion sociale du groupe
            adaptationLevel: 0.5,          // Niveau d'adaptation
            leadershipStructure: new Map() // Structure de leadership dynamique
        };

        // Mémoire comportementale
        this.behaviorMemory = {
            successfulStrategies: [],
            failedAttempts: [],
            environmentLearning: new Map(),
            socialInteractions: []
        };

        // Communication inter-agents
        this.communicationNetwork = {
            activeConnections: new Map(),
            messageQueue: [],
            sharedSymbols: new Map(),
            stigmergyMarks: [] // Marquage stigmergique (comme les fourmis)
        };

        // Ajout des métriques manquantes de diamants-gazebo-meshes
        this.learningProgress = 0;
        this.innovationIndex = 0;
        this.coherenceIndex = 1.0;
        this.swarmIntelligenceLevel = 0;
        this.emergenceStrength = 0;
        this.redundancyLevel = 0;

        this.initializeIntelligence();
    }

    initializeIntelligence() {
        // Initialisation de l'intelligence collective
        this.setupKnowledgeBase();
        this.establishCommunicationProtocols();

        log('🧠 Intelligence collective initialisée');
    }

    // Boucle de mise à jour minimale pour intégration fluide
    // Évite les erreurs quand l'application appelle update(deltaTime)
    update(deltaTime = 0.016) {
        // Décroissance simple des marques stigmergiques et nettoyage de la file de messages
        const now = Date.now();
        if (Array.isArray(this.communicationNetwork.stigmergyMarks)) {
            this.communicationNetwork.stigmergyMarks = this.communicationNetwork.stigmergyMarks
                .map(mark => ({
                    ...mark,
                    strength: Math.max(0, mark.strength - (mark.decay || 0.1) * deltaTime)
                }))
                .filter(mark => mark.strength > 0.05);
        }

        // Traiter un petit lot de messages par tick (si nécessaire)
        if (Array.isArray(this.communicationNetwork.messageQueue) && this.communicationNetwork.messageQueue.length) {
            const batch = this.communicationNetwork.messageQueue.splice(0, 5);
            // Ici on pourrait dispatcher, pour l'instant on effectue un no-op sûr
            void batch;
        }

        // Légère adaptation continue pour montrer une activité
        this.collectiveState.adaptationLevel = Math.min(1.0, Math.max(0.0, this.collectiveState.adaptationLevel + (Math.random() - 0.5) * 0.001));
    }

    setupKnowledgeBase() {
        // Base de connaissances collective partagée
        this.collectiveState.knowledge.set('mission_types', [
            'exploration', 'search_rescue', 'formation_flight',
            'area_surveillance', 'resource_mapping'
        ]);

        this.collectiveState.knowledge.set('behavior_patterns', [
            'flocking', 'leader_follower', 'consensus_based',
            'emergence_driven', 'adaptive_swarm'
        ]);

        this.collectiveState.knowledge.set('environmental_factors', [
            'obstacles', 'wind_conditions', 'visibility',
            'terrain_difficulty', 'mission_urgency'
        ]);
    }

    establishCommunicationProtocols() {
        // Protocoles de communication inspirés des systèmes naturels
        this.communicationNetwork.sharedSymbols.set('danger', {
            priority: 'high',
            decay: 0.1, // Décroissance temporelle
            range: this.config.communicationRange * 1.5
        });

        this.communicationNetwork.sharedSymbols.set('resource_found', {
            priority: 'medium',
            decay: 0.05,
            range: this.config.communicationRange
        });

        this.communicationNetwork.sharedSymbols.set('formation_request', {
            priority: 'low',
            decay: 0.2,
            range: this.config.communicationRange * 0.8
        });
    }

    /**
     * Prise de décision collective basée sur DIAMANTS
     */
    makeCollectiveDecision(agents, situation) {
        const decisions = [];

        // 1. Collecte des opinions individuelles
        for (const agent of agents) {
            const opinion = this.getAgentOpinion(agent, situation);
            decisions.push(opinion);
        }

        // 2. Application des formules DIAMANTS pour consensus
        const consensusField = this.diamantFormulas.calculateHarmonique(7, { // H7 - Consensus
            agents: agents.map(a => a.position),
            opinions: decisions,
            threshold: this.config.decisionThreshold
        });

        // 3. Analyse des motifs émergents
        const emergentPattern = this.detectEmergentPattern(decisions, consensusField);

        // 4. Génération de la décision collective
        const collectiveDecision = this.synthesizeDecision(decisions, emergentPattern);

        // 5. Mise à jour de la mémoire collective
        this.updateCollectiveMemory(situation, collectiveDecision);

        return collectiveDecision;
    }

    getAgentOpinion(agent, situation) {
        // Opinion d'un agent basée sur son état et l'environnement
        const opinion = {
            agentId: agent.id,
            confidence: 0,
            preferredAction: null,
            reasoning: [],
            urgency: 0
        };

        // Analyse de la situation par l'agent
        if (situation.type === 'obstacle_detected') {
            opinion.preferredAction = this.evaluateObstacleResponse(agent, situation);
            opinion.confidence = 0.8;
            opinion.urgency = situation.distance < 2.0 ? 0.9 : 0.3;
        } else if (situation.type === 'mission_update') {
            opinion.preferredAction = this.evaluateMissionResponse(agent, situation);
            opinion.confidence = 0.6;
            opinion.urgency = 0.5;
        } else if (situation.type === 'agent_distress') {
            opinion.preferredAction = 'assist_agent';
            opinion.confidence = 0.9;
            opinion.urgency = 0.8;
        }

        // Ajout du raisonnement DIAMANTS
        const diamantInfluence = this.diamantFormulas.calculateHarmonique(3, { // H3 - Adaptation
            position: agent.position,
            situation: situation,
            experience: agent.experience || 0
        });

        opinion.confidence *= diamantInfluence.intensity;
        opinion.reasoning.push(`DIAMANTS H3: ${diamantInfluence.intensity.toFixed(2)}`);

        return opinion;
    }

    evaluateObstacleResponse(agent, situation) {
        const distance = situation.distance;
        const obstacleType = situation.obstacle.type;

        // Stratégies selon le type d'obstacle
        if (obstacleType === 'tree' && distance > 1.5) {
            return { action: 'navigate_around', direction: this.getBestDetourDirection(agent, situation) };
        } else if (obstacleType === 'rock' && distance > 1.0) {
            return { action: 'climb_over', altitude: situation.obstacle.height + 1.0 };
        } else {
            return { action: 'emergency_stop', hover: true };
        }
    }

    evaluateMissionResponse(agent, situation) {
        const missionType = situation.mission.type;
        const currentRole = agent.role || 'follower';

        switch (missionType) {
            case 'exploration':
                return {
                    action: 'spread_formation',
                    pattern: 'hexagonal',
                    spacing: 3.0
                };

            case 'search_rescue':
                return {
                    action: 'systematic_search',
                    pattern: 'boustrophedon',
                    altitude: 2.0
                };

            case 'formation_flight':
                return {
                    action: 'maintain_formation',
                    position: this.calculateFormationPosition(agent, situation)
                };

            default:
                return { action: 'await_instructions' };
        }
    }

    detectEmergentPattern(decisions, consensusField) {
        // Détection de motifs émergents dans les décisions
        const pattern = {
            type: 'unknown',
            strength: 0,
            participants: [],
            characteristics: {}
        };

        // Analyse des actions préférées
        const actionCounts = new Map();
        decisions.forEach(decision => {
            const action = decision.preferredAction?.action || 'none';
            actionCounts.set(action, (actionCounts.get(action) || 0) + 1);
        });

        // Détection de consensus émergent
        const totalDecisions = decisions.length;
        for (const [action, count] of actionCounts) {
            const ratio = count / totalDecisions;
            if (ratio > 0.6) {
                pattern.type = 'strong_consensus';
                pattern.strength = ratio;
                pattern.characteristics.dominantAction = action;
            } else if (ratio > 0.4) {
                pattern.type = 'weak_consensus';
                pattern.strength = ratio;
            }
        }

        // Détection de polarisation
        if (actionCounts.size === 2 && [...actionCounts.values()].every(v => v > totalDecisions * 0.3)) {
            pattern.type = 'polarization';
            pattern.strength = 0.8;
            pattern.characteristics.opposingActions = [...actionCounts.keys()];
        }

        // Application des formules DIAMANTS pour validation
        const emergenceValue = this.diamantFormulas.calculateHarmonique(12, { // H12 - Émergence
            decisions,
            consensusField,
            threshold: this.config.decisionThreshold
        });

        pattern.strength *= emergenceValue.intensity;

        // Enregistrement du motif
        this.collectiveState.emergentPatterns.push({
            ...pattern,
            timestamp: Date.now(),
            context: decisions
        });

        return pattern;
    }

    synthesizeDecision(decisions, emergentPattern) {
        const synthesis = {
            action: null,
            confidence: 0,
            participants: [],
            reasoning: ['Synthesis of collective intelligence'],
            alternatives: []
        };

        if (emergentPattern.type === 'strong_consensus') {
            // Consensus fort : adoption de l'action dominante
            synthesis.action = emergentPattern.characteristics.dominantAction;
            synthesis.confidence = emergentPattern.strength;
            synthesis.participants = decisions.map(d => d.agentId);

        } else if (emergentPattern.type === 'polarization') {
            // Polarisation : résolution par leader ou vote pondéré
            const leaderDecision = this.resolveByLeadership(decisions);
            if (leaderDecision) {
                synthesis.action = leaderDecision.preferredAction;
                synthesis.confidence = 0.6;
                synthesis.reasoning.push('Leadership resolution');
            } else {
                synthesis.action = this.resolveByWeightedVote(decisions);
                synthesis.confidence = 0.5;
                synthesis.reasoning.push('Weighted vote resolution');
            }

        } else {
            // Pas de consensus clair : stratégie adaptative
            synthesis.action = this.generateAdaptiveStrategy(decisions);
            synthesis.confidence = 0.4;
            synthesis.reasoning.push('Adaptive strategy generation');
        }

        return synthesis;
    }

    resolveByLeadership(decisions) {
        // Résolution par structure de leadership dynamique
        const leaders = [...this.collectiveState.leadershipStructure.entries()]
            .sort((a, b) => b[1].influence - a[1].influence);

        if (leaders.length > 0) {
            const topLeader = leaders[0][0];
            return decisions.find(d => d.agentId === topLeader);
        }

        return null;
    }

    resolveByWeightedVote(decisions) {
        // Vote pondéré basé sur la confiance et l'expérience
        const weightedActions = new Map();

        decisions.forEach(decision => {
            const action = decision.preferredAction?.action || 'none';
            const weight = decision.confidence * (decision.urgency + 0.5);

            weightedActions.set(action, (weightedActions.get(action) || 0) + weight);
        });

        // Sélection de l'action avec le poids maximum
        let maxWeight = 0;
        let selectedAction = null;

        for (const [action, weight] of weightedActions) {
            if (weight > maxWeight) {
                maxWeight = weight;
                selectedAction = action;
            }
        }

        return { action: selectedAction };
    }

    generateAdaptiveStrategy(decisions) {
        // Génération d'une stratégie adaptative basée sur l'historique
        const recentStrategies = this.behaviorMemory.successfulStrategies
            .slice(-5) // 5 dernières stratégies réussies
            .map(s => s.action);

        // Sélection d'une stratégie éprouvée ou exploration
        if (recentStrategies.length > 0 && Math.random() > 0.3) {
            // Exploitation : utiliser une stratégie connue
            const randomStrategy = recentStrategies[Math.floor(Math.random() * recentStrategies.length)];
            return { action: randomStrategy, type: 'exploitation' };
        } else {
            // Exploration : nouvelle stratégie
            return { action: 'explore_new_strategy', type: 'exploration' };
        }
    }

    /**
     * Communication et stigmergie
     */
    broadcastMessage(senderId, messageType, content, range = null) {
        const message = {
            id: this.generateMessageId(),
            senderId,
            type: messageType,
            content,
            timestamp: Date.now(),
            range: range || this.config.communicationRange,
            priority: this.communicationNetwork.sharedSymbols.get(messageType)?.priority || 'low'
        };

        this.communicationNetwork.messageQueue.push(message);

        // Application des formules DIAMANTS pour propagation
        const propagationField = this.diamantFormulas.calculateHarmonique(8, { // H8 - Communication
            source: senderId,
            message: content,
            network: this.communicationNetwork.activeConnections
        });

        message.propagationStrength = propagationField.intensity;

        return message;
    }

    addStigmergyMark(position, type, strength, agentId) {
        // Ajout de marquage stigmergique (comme les phéromones des fourmis)
        const mark = {
            id: this.generateMarkId(),
            position: position.clone(),
            type,
            strength,
            agentId,
            timestamp: Date.now(),
            decay: this.communicationNetwork.sharedSymbols.get(type)?.decay || 0.1
        };

        this.communicationNetwork.stigmergyMarks.push(mark);

        // Limitation du nombre de marques pour performance
        if (this.communicationNetwork.stigmergyMarks.length > 100) {
            this.communicationNetwork.stigmergyMarks.splice(0, 20);
        }

        return mark;
    }

    readStigmergyMarks(position, range = 3.0) {
        // Lecture des marques stigmergiques dans un rayon donné
        const nearbyMarks = this.communicationNetwork.stigmergyMarks
            .filter(mark => position.distanceTo(mark.position) <= range)
            .sort((a, b) => a.strength - b.strength);

        // Application de la décroissance temporelle
        const now = Date.now();
        return nearbyMarks.map(mark => ({
            ...mark,
            currentStrength: mark.strength * Math.exp(-mark.decay * (now - mark.timestamp) / 1000)
        })).filter(mark => mark.currentStrength > 0.1);
    }

    /**
     * Apprentissage et adaptation
     */
    updateCollectiveMemory(situation, decision) {
        // Mise à jour de la mémoire collective
        const memoryEntry = {
            situation: { ...situation },
            decision: { ...decision },
            timestamp: Date.now(),
            outcome: null // Sera mis à jour après évaluation
        };

        // Ajout à la mémoire selon le type de situation
        if (situation.type === 'obstacle_detected') {
            this.behaviorMemory.environmentLearning.set(
                this.hashSituation(situation),
                memoryEntry
            );
        } else if (situation.type === 'social_interaction') {
            this.behaviorMemory.socialInteractions.push(memoryEntry);
        }

        // Application de l'harmonique d'apprentissage DIAMANTS
        const learningField = this.diamantFormulas.calculateHarmonique(9, { // H9 - Apprentissage
            situation,
            decision,
            memory: this.behaviorMemory
        });

        memoryEntry.learningValue = learningField.intensity;

        // Adaptation du niveau d'adaptation collective
        this.collectiveState.adaptationLevel += learningField.intensity * this.config.adaptationRate;
        this.collectiveState.adaptationLevel = Math.min(1.0, this.collectiveState.adaptationLevel);
    }

    evaluateDecisionOutcome(decisionId, success, performance) {
        // Évaluation rétrospective des décisions
        const evaluation = {
            decisionId,
            success,
            performance,
            timestamp: Date.now()
        };

        if (success) {
            this.behaviorMemory.successfulStrategies.push(evaluation);
            // Limiter la taille de l'historique
            if (this.behaviorMemory.successfulStrategies.length > 50) {
                this.behaviorMemory.successfulStrategies.splice(0, 10);
            }
        } else {
            this.behaviorMemory.failedAttempts.push(evaluation);
            if (this.behaviorMemory.failedAttempts.length > 30) {
                this.behaviorMemory.failedAttempts.splice(0, 5);
            }
        }

        // Mise à jour de la cohésion sociale
        this.updateSocialCohesion(success, performance);
    }

    updateSocialCohesion(success, performance) {
        const impact = success ? performance * 0.1 : -0.05;
        this.collectiveState.socialCohesion += impact;
        this.collectiveState.socialCohesion = Math.max(0.2, Math.min(1.0, this.collectiveState.socialCohesion));
    }

    /**
     * Leadership dynamique
     */
    updateLeadershipStructure(agents, currentSituation) {
        // Garde contre entrées invalides
        if (!Array.isArray(agents) || agents.length === 0) {
            return;
        }

        // Mise à jour de la structure de leadership selon les performances
        for (const agent of agents) {
            const currentInfluence = this.collectiveState.leadershipStructure.get(agent.id) || {
                influence: 0.5,
                specialties: [],
                recentPerformance: []
            };

            // Évaluation de la performance récente
            const performance = this.evaluateAgentPerformance(agent, currentSituation);
            currentInfluence.recentPerformance.push(performance);

            // Garde seulement les 10 dernières performances
            if (currentInfluence.recentPerformance.length > 10) {
                currentInfluence.recentPerformance.shift();
            }

            // Calcul de l'influence moyenne
            const avgPerformance = currentInfluence.recentPerformance.reduce((a, b) => a + b, 0) /
                currentInfluence.recentPerformance.length;

            // Mise à jour de l'influence avec décroissance temporelle
            currentInfluence.influence = currentInfluence.influence * 0.95 + avgPerformance * 0.05;

            this.collectiveState.leadershipStructure.set(agent.id, currentInfluence);
        }

        // Application de l'harmonique de leadership DIAMANTS
        const leadershipField = this.diamantFormulas.calculateHarmonique(11, { // H11 - Leadership
            agents,
            leadershipStructure: this.collectiveState.leadershipStructure,
            situation: currentSituation
        });

        // Ajustement basé sur le champ DIAMANTS
        for (const [agentId, influence] of this.collectiveState.leadershipStructure) {
            influence.diamantBonus = leadershipField.intensity * 0.2;
        }
    }

    evaluateAgentPerformance(agent, situation) {
        // Évaluation simple de la performance d'un agent
        let performance = 0.5; // Base neutre

        // Facteurs positifs
        if (agent.missionSuccess) performance += 0.3;
        if (agent.communicationActive) performance += 0.1;
        if (agent.helpedOthers) performance += 0.2;

        // Facteurs négatifs
        if (agent.collisionCount > 0) performance -= 0.2;
        if (agent.emergencyStops > 0) performance -= 0.1;

        return Math.max(0, Math.min(1, performance));
    }

    /**
     * Utilitaires
     */
    generateMessageId() {
        return `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    }

    generateMarkId() {
        return `mark_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    }

    hashSituation(situation) {
        // Hash simple pour indexer les situations
        return `${situation.type}_${JSON.stringify(situation.obstacle || situation.mission || {})}`;
    }

    getBestDetourDirection(agent, situation) {
        // Calcul de la meilleure direction de contournement
        const obstacle = situation.obstacle;
        const toObstacle = new THREE.Vector3().subVectors(obstacle.position, agent.position);

        // Directions perpendiculaires
        const right = new THREE.Vector3(-toObstacle.z, 0, toObstacle.x).normalize();
        const left = new THREE.Vector3(toObstacle.z, 0, -toObstacle.x).normalize();

        // Choix de la direction la plus libre
        // (Simplification : préférence aléatoire, à améliorer avec détection d'obstacles)
        return Math.random() > 0.5 ? right : left;
    }

    calculateFormationPosition(agent, situation) {
        // Calcul de position en formation
        const formation = situation.mission.formation;
        const index = agent.formationIndex || 0;

        // Formations de base
        switch (formation.type) {
            case 'line':
                return new THREE.Vector3(index * 2, 0, 0);
            case 'wedge':
                return new THREE.Vector3(Math.abs(index) * 1.5, index * 1.5, 0);
            case 'circle':
                const angle = (index / formation.totalAgents) * Math.PI * 2;
                return new THREE.Vector3(
                    Math.cos(angle) * formation.radius,
                    Math.sin(angle) * formation.radius,
                    0
                );
            default:
                return new THREE.Vector3(0, 0, 0);
        }
    }

    /**
     * Interface publique
     */
    getCollectiveState() {
        return { ...this.collectiveState };
    }

    getSwarmMetrics() {
        return {
            socialCohesion: this.collectiveState.socialCohesion,
            adaptationLevel: this.collectiveState.adaptationLevel,
            activeMessages: this.communicationNetwork.messageQueue.length,
            stigmergyMarks: this.communicationNetwork.stigmergyMarks.length,
            emergentPatterns: this.collectiveState.emergentPatterns.length,
            leaderCount: this.collectiveState.leadershipStructure.size
        };
    }

    /**
     * Mise à jour principale
     */
    update(deltaTime, agents, environment) {
        // Normaliser les paramètres pour éviter les crashs si indéfinis
        const safeAgents = Array.isArray(agents) ? agents : [];

        // 1. Traitement des messages
        this.processMessageQueue(safeAgents);

        // 2. Décroissance des marques stigmergiques
        this.updateStigmergyMarks(deltaTime);

        // 3. Mise à jour du leadership
        if (safeAgents.length > 0) {
            this.updateLeadershipStructure(safeAgents, environment || {});
        }

        // 4. Nettoyage des motifs anciens
        this.cleanupOldPatterns();
    }

    processMessageQueue(agents) {
        // Traitement de la queue des messages
        const now = Date.now();
        this.communicationNetwork.messageQueue = this.communicationNetwork.messageQueue
            .filter(message => now - message.timestamp < 5000); // 5 secondes de TTL
    }

    updateStigmergyMarks(deltaTime) {
        // Mise à jour et décroissance des marques
        const now = Date.now();
        this.communicationNetwork.stigmergyMarks = this.communicationNetwork.stigmergyMarks
            .filter(mark => {
                const age = (now - mark.timestamp) / 1000;
                const strength = mark.strength * Math.exp(-mark.decay * age);
                return strength > 0.1; // Suppression des marques trop faibles
            });
    }

    cleanupOldPatterns() {
        // Nettoyage des anciens motifs émergents
        const cutoff = Date.now() - 60000; // 1 minute
        this.collectiveState.emergentPatterns = this.collectiveState.emergentPatterns
            .filter(pattern => pattern.timestamp > cutoff);
    }

    /**
     * Nettoyage
     */
    destroy() {
        // Nettoyage de l'intelligence collective
        this.collectiveState.knowledge.clear();
        this.collectiveState.consensus.clear();
        this.collectiveState.emergentPatterns = [];
        this.collectiveState.leadershipStructure.clear();

        this.behaviorMemory.successfulStrategies = [];
        this.behaviorMemory.failedAttempts = [];
        this.behaviorMemory.environmentLearning.clear();
        this.behaviorMemory.socialInteractions = [];

        this.communicationNetwork.activeConnections.clear();
        this.communicationNetwork.messageQueue = [];
        this.communicationNetwork.sharedSymbols.clear();
        this.communicationNetwork.stigmergyMarks = [];

        log('🧠 Intelligence collective détruite proprement');
    }

    // ===== MÉTHODES MANQUANTES DE DIAMANTS-GAZEBO-MESHES =====

    /**
     * Calcul du progrès d'apprentissage collectif
     */
    calculateLearningProgress(agents = []) {
        if (!agents || agents.length === 0) {
            this.learningProgress = 0;
            return this.learningProgress;
        }

        let activity = 0;
        agents.forEach(agent => {
            activity += agent.expertiseSystem ? (agent.expertiseSystem.experienceCounter || 0) : 0;
        });

        const norm = Math.min(1, activity / Math.max(1, agents.length * 50));
        this.learningProgress = this.learningProgress * 0.9 + norm * 0.1;
        return this.learningProgress;
    }

    /**
     * Évaluation du risque de redondance
     */
    assessRedundancyRisk(agents = []) {
        if (!agents || agents.length === 0) {
            this.redundancyLevel = 0;
            return this.redundancyLevel;
        }

        // Analyser la redondance comportementale
        const behaviorCounts = new Map();
        agents.forEach(agent => {
            const behavior = agent.currentBehavior || 'exploration';
            behaviorCounts.set(behavior, (behaviorCounts.get(behavior) || 0) + 1);
        });

        // Calculer l'indice de redondance
        let redundancy = 0;
        for (const [behavior, count] of behaviorCounts) {
            const ratio = count / agents.length;
            if (ratio > 0.7) { // Plus de 70% sur le même comportement
                redundancy += ratio * 0.8;
            }
        }

        this.redundancyLevel = Math.min(1, redundancy);
        return this.redundancyLevel;
    }

    /**
     * Adaptation automatique des paramètres
     */
    adaptParameters(agents = [], environmentData = {}) {
        if (!agents || agents.length === 0) return;

        // Adaptation basée sur la performance
        const avgPerformance = agents.reduce((sum, agent) =>
            sum + (agent.performance || 0.5), 0) / agents.length;

        if (avgPerformance < 0.3) {
            // Performance faible - augmenter exploration
            this.config.adaptationRate = Math.min(0.2, this.config.adaptationRate * 1.1);
        } else if (avgPerformance > 0.8) {
            // Performance élevée - stabiliser
            this.config.adaptationRate = Math.max(0.05, this.config.adaptationRate * 0.95);
        }

        log(`🔧 Paramètres adaptés: performance=${avgPerformance.toFixed(2)}, adaptation=${this.config.adaptationRate.toFixed(3)}`);
    }

    /**
     * Calcul de l'indice d'innovation
     */
    calculateInnovationIndex(agents = []) {
        if (!agents || agents.length === 0) {
            this.innovationIndex = 0;
            return this.innovationIndex;
        }

        let sum = 0;
        agents.forEach(agent => {
            sum += agent.innovation?.innovationScore || 0;
        });

        const avg = agents.length ? sum / agents.length : 0;
        this.innovationIndex = Math.max(0, Math.min(1, avg));
        return this.innovationIndex;
    }

    /**
     * Calcul de la cohérence du swarm
     */
    calculateCoherence(agents = []) {
        if (!agents || agents.length < 2) {
            this.coherenceIndex = 1.0;
            return this.coherenceIndex;
        }

        // Cohérence spatiale
        const center = { x: 0, y: 0, z: 0 };
        agents.forEach(agent => {
            center.x += agent.position.x;
            center.y += agent.position.y;
            center.z += agent.position.z;
        });
        center.x /= agents.length;
        center.y /= agents.length;
        center.z /= agents.length;

        let spatialVariance = 0;
        agents.forEach(agent => {
            const dx = agent.position.x - center.x;
            const dy = agent.position.y - center.y;
            const dz = agent.position.z - center.z;
            spatialVariance += dx * dx + dy * dy + dz * dz;
        });
        spatialVariance /= agents.length;
        const spatialCoherence = Math.max(0, 1 - spatialVariance / 10000);

        // Cohérence comportementale
        const behaviorCounts = new Map();
        agents.forEach(agent => {
            const behavior = agent.currentBehavior || 'exploration';
            behaviorCounts.set(behavior, (behaviorCounts.get(behavior) || 0) + 1);
        });

        const maxBehaviorCount = Math.max(...behaviorCounts.values());
        const behavioralCoherence = maxBehaviorCount / agents.length;

        this.coherenceIndex = (spatialCoherence + behavioralCoherence) / 2;
        return this.coherenceIndex;
    }

    /**
     * Calcul de l'intelligence collective
     */
    calculateSwarmIntelligence(agents = []) {
        if (!agents || agents.length === 0) {
            this.swarmIntelligenceLevel = 0;
            return this.swarmIntelligenceLevel;
        }

        let totalIndividualIntelligence = 0;
        let totalCollectiveBonus = 0;

        agents.forEach(agent => {
            totalIndividualIntelligence += agent.intelligence || 0;

            // Bonus d'intelligence collective basé sur les interactions
            const communicationBonus = (agent.communicationLevel || 0) * 0.1;
            const expertiseBonus = (agent.expertiseSystem?.getBestSpecialization() ? 0.2 : 0);
            const consensusBonus = (agent.lastConsensusParticipation || 0) * 0.15;

            totalCollectiveBonus += communicationBonus + expertiseBonus + consensusBonus;
        });

        const baseIntelligence = totalIndividualIntelligence / agents.length;
        const collectiveMultiplier = 1 + (totalCollectiveBonus / agents.length);

        this.swarmIntelligenceLevel = baseIntelligence * collectiveMultiplier;
        return this.swarmIntelligenceLevel;
    }

    /**
     * Obtenir le niveau d'intelligence du swarm
     */
    getSwarmIntelligenceLevel() {
        return this.swarmIntelligenceLevel;
    }
}

// Export par défaut pour compatibilité avec l'import dynamique
export default CollectiveIntelligence;
