/**
 * DIAMANTS - Système de Scouting Collaboratif
 * ==============================================
 * Intelligence collective pour exploration et couverture de terrain
 */

import { logger } from '../core/logger.js';
import { CollectiveIntelligence } from '../intelligence/collective-intelligence.js';
import { FlightBehaviors } from './flight-behaviors.js';

// Mode silencieux global - utilise les fonctions globales  
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;
// Force silence total pour ce module
if (window.SILENT_MODE) {
    var originalConsoleLog = console.log;
    console.log = (...args) => {}; // Silence total pour les behaviors
}

export class CollaborativeScouting {
    constructor(config = {}) {
        logger.info('CollaborativeScouting', '🔍 CollaborativeScouting.constructor() - Initialisation scouting collaboratif');

        this.config = {
            explorationRadius: config.explorationRadius || 25.0, // mètres
            coverageResolution: config.coverageResolution || 1.0, // grille 1m x 1m
            communicationRange: config.communicationRange || 10.0,
            consensusThreshold: config.consensusThreshold || 0.6,
            adaptationRate: config.adaptationRate || 0.1,
            stigmergyDecay: config.stigmergyDecay || 0.95,
            wahooEffectStrength: config.wahooEffectStrength || 1.2,
            missionArea: config.missionArea || { width: 100, height: 100 }, // Zone de mission par défaut
            ...config
        };

        // Intégration des systèmes existants
        this.collectiveIntelligence = new CollectiveIntelligence(this.config);
        this.flightBehaviors = new FlightBehaviors(this.config);

        // Carte de couverture globale (de Sample/SMA.html)
        this.coverageMap = new Map(); // key: "x_z" -> {coverage: 0-1, lastVisited: timestamp, quality: 0-1}

        // Zones d'intérêt découvertes
        this.pointsOfInterest = new Map(); // key: id -> {position, type, priority, discoveredBy, timestamp}

        // Assignations dynamiques des drones
        this.droneAssignments = new Map(); // droneId -> {zone, role, priority, target}

        // Patterns de scouting collaboratif
        this.scoutingPatterns = {
            GRID_SYSTEMATIC: 'grid_systematic',
            SPIRAL_CONVERGENT: 'spiral_convergent',
            ADAPTIVE_PRIORITY: 'adaptive_priority',
            SWARM_DISPERSION: 'swarm_dispersion',
            WAHOO_EFFECT: 'wahoo_effect' // Depuis SMA.html
        };

        // État actuel de la mission de scouting
        this.missionState = {
            pattern: this.scoutingPatterns.ADAPTIVE_PRIORITY,
            startTime: 0,
            totalArea: 0,
            coveredArea: 0,
            efficiency: 0,
            phase: 'INITIALIZATION', // INITIALIZATION, DEPLOYMENT, EXPLORATION, CONVERGENCE, COMPLETION
            objectives: [],
            discoveries: []
        };

        // Système de stigmergie (inspiré fourmis - de Sample)
        this.stigmergySystem = {
            traces: new Map(), // "x_z" -> {intensity, type, age, droneId}
            traceTypes: {
                EXPLORED: 'explored',
                INTERESTING: 'interesting',
                OBSTACLE: 'obstacle',
                SAFE_PATH: 'safe_path',
                WAHOO_TRACE: 'wahoo_trace'
            }
        };

        // Métriques de performance collaborative
        this.collaborationMetrics = {
            communicationEfficiency: 0,
            coordinationIndex: 0,
            redundancyLevel: 0,
            emergentBehaviors: 0,
            wahooEffectLevel: 0
        };

        // Traces de stigmergie simples (pour éviter les erreurs)
        this.stigmergyTraces = new Map();

        this.initializeScoutingSystem();
    }

    /**
     * Initialisation du système de scouting
     */
    initializeScoutingSystem() {
        this.setupCoverageGrid();
        this.defineMissionObjectives();
        this.initializeWahooEffect();

        if (!window.SILENT_MODE) {
            console.log('🔍 Système de scouting collaboratif initialisé');
            console.log(`📊 Zone d'exploration: ${this.config.explorationRadius}m radius`);
            console.log(`🎯 Résolution couverture: ${this.config.coverageResolution}m`);
        }
    }

    /**
     * Configuration de la grille de couverture
     */
    setupCoverageGrid() {
        const radius = this.config.explorationRadius;
        const resolution = this.config.coverageResolution;

        // Créer grille hexagonale pour optimiser la couverture
        for (let x = -radius; x <= radius; x += resolution) {
            for (let z = -radius; z <= radius; z += resolution) {
                const distance = Math.sqrt(x * x + z * z);
                if (distance <= radius) {
                    const key = `${Math.round(x)}_${Math.round(z)}`;
                    this.coverageMap.set(key, {
                        position: { x, z },
                        coverage: 0,
                        lastVisited: 0,
                        quality: 0,
                        priority: 1.0 - (distance / radius), // Priorité basée sur la distance
                        visitCount: 0
                    });
                }
            }
        }

        this.missionState.totalArea = this.coverageMap.size;
                if (!window.SILENT_MODE) {
            console.log(`📍 Grille de couverture: ${this.missionState.totalArea} cellules`);
        }
    }

    /**
     * Définition des objectifs de mission
     */
    defineMissionObjectives() {
        this.missionState.objectives = [
            {
                type: 'COVERAGE',
                target: 0.85, // 85% de couverture minimale
                priority: 1.0,
                status: 'ACTIVE'
            },
            {
                type: 'EFFICIENCY',
                target: 0.70, // 70% d'efficacité minimale
                priority: 0.8,
                status: 'ACTIVE'
            },
            {
                type: 'COLLABORATION',
                target: 0.75, // 75% de coordination entre drones
                priority: 0.9,
                status: 'ACTIVE'
            }
        ];
    }

    /**
     * Initialisation Effet Wahoo (depuis SMA.html)
     */
    initializeWahooEffect() {
        // L'effet Wahoo crée des dynamiques d'attraction/répulsion entre agents
        // inspiré des comportements de groupe des dauphins
        this.wahooEffect = {
            attractionCenters: [], // Zones d'attraction dynamiques
            repulsionFields: [],   // Champs de répulsion temporaires
            emergentLeaders: new Set(), // Drones émergents comme leaders
            groupCohesion: 1.0,
            dynamicAdaptation: 0.5
        };
    }

    /**
     * Démarrage mission de scouting
     */
    async startScoutingMission(droneIds, missionParams = {}) {
        if (!window.SILENT_MODE) {
            log(`🚀 Démarrage mission scouting avec ${droneIds.length} drones`);
        }

        this.missionState.startTime = Date.now();
        this.missionState.phase = 'DEPLOYMENT';

        // Initialiser physique pour chaque drone
        for (const droneId of droneIds) {
            const startPosition = this.generateStartPosition(droneId, droneIds.length);
            this.flightBehaviors.initializeDronePhysics(droneId, startPosition);

            // ⚠️ PAS DE DÉCOLLAGE AUTOMATIQUE - ATTENDRE COMMANDE UTILISATEUR
            // const ok = await this.flightBehaviors.performTakeoff(droneId, 2.0 + Math.random() * 2.0);

            // Assignation initiale uniquement - DRONE RESTE AU SOL jusqu'à commande utilisateur
            this.assignInitialRole(droneId);
            if (!window.SILENT_MODE) {
                log(`🚁 Drone ${droneId}: Prêt au sol - En attente commande utilisateur`);
            }
        }

        this.missionState.phase = 'STANDBY'; // État d'attente utilisateur au lieu de 'EXPLORATION'
        if (!window.SILENT_MODE) {
            log('✅ Tous les drones initialisés au sol - En attente commandes utilisateur');
        }

        return true;
    }

    /**
     * Génération position de départ
     */
    generateStartPosition(droneId, totalDrones) {
        // droneId peut être une chaîne (ex: "crazyflie_3"): extraire un index numérique robuste
        const idx = typeof droneId === 'number' ? droneId : (parseInt(String(droneId).match(/(\d+)/)?.[1] || '0', 10) || 0);
        const angle = (idx / totalDrones) * 2 * Math.PI;
        const radius = 12.0; // 12m du centre pour éviter les collisions (était 3m)

        return {
            x: Math.cos(angle) * radius,
            y: 0,
            z: Math.sin(angle) * radius
        };
    }

    /**
     * Assignation rôle initial pour un drone
     */
    assignInitialRole(droneId) {
        const roles = ['EXPLORER', 'MAPPER', 'SCOUT', 'COORDINATOR'];
        // droneId peut être une chaîne: calculer un index stable
        const idx = typeof droneId === 'number' ? droneId : (parseInt(String(droneId).match(/(\d+)/)?.[1] || '0', 10) || 0);
        const role = roles[idx % roles.length];

        this.droneAssignments.set(droneId, {
            role: role,
            zone: this.selectInitialZone(droneId),
            priority: 1.0,
            target: null,
            lastUpdate: Date.now()
        });

        if (!window.SILENT_MODE) {
            log(`📋 Drone ${droneId}: Rôle ${role} assigné`);
        }
    }

    /**
     * Sélection zone initiale d'exploration
     */
    selectInitialZone(droneId) {
        // Répartition hexagonale des zones
        const angles = [0, Math.PI / 3, 2 * Math.PI / 3, Math.PI, 4 * Math.PI / 3, 5 * Math.PI / 3];
        const idx = typeof droneId === 'number' ? droneId : (parseInt(String(droneId).match(/(\d+)/)?.[1] || '0', 10) || 0);
        const angle = angles[idx % angles.length];
        const distance = this.config.explorationRadius * 0.7;

        return {
            center: {
                x: Math.cos(angle) * distance,
                z: Math.sin(angle) * distance
            },
            radius: this.config.explorationRadius * 0.3
        };
    }

    /**
     * Mise à jour principale du système de scouting
     */
    update(deltaTime) {
        // Mise à jour des comportements de vol
        this.flightBehaviors.update(deltaTime);

        // Mise à jour intelligence collective
        this.collectiveIntelligence.update(deltaTime);

        // Système de coordination collaborative
        this.updateCollaboration(deltaTime);

        // Mise à jour de la couverture
        this.updateCoverageMapping();

        // Effet Wahoo dynamique
        this.updateWahooEffect(deltaTime);

        // Mise à jour des métriques
        this.updateMetrics();

        // Adaptation dynamique des stratégies
        this.adaptStrategies(deltaTime);
    }

    /**
     * Coordination collaborative entre drones
     */
    updateCollaboration(deltaTime) {
        const activeDrones = Array.from(this.droneAssignments.keys());

        for (const droneId of activeDrones) {
            const assignment = this.droneAssignments.get(droneId);
            const telemetry = this.flightBehaviors.getTelemetry(droneId);

            if (!telemetry) continue;

            // Communication avec drones proches
            this.processLocalCommunication(droneId, telemetry);

            // Mise à jour objectif selon rôle
            this.updateDroneObjective(droneId, assignment, telemetry);

            // Traces de stigmergie
            this.updateStigmergyTraces(droneId, telemetry);
        }
    }

    /**
     * Communication locale entre drones
     */
    processLocalCommunication(droneId, telemetry) {
        const nearbyDrones = this.findNearbyDrones(droneId, this.config.communicationRange);

        for (const nearbyId of nearbyDrones) {
            const nearbyTelemetry = this.flightBehaviors.getTelemetry(nearbyId);

            // Partage d'informations de couverture
            this.shareDiscoveries(droneId, nearbyId);

            // Coordination pour éviter redondance
            this.coordinateExploration(droneId, nearbyId);

            // Effet Wahoo entre drones proches
            this.applyWahooInteraction(droneId, nearbyId, telemetry, nearbyTelemetry);
        }
    }

    /**
     * Recherche drones à proximité
     */
    findNearbyDrones(droneId, range) {
        const targetTelemetry = this.flightBehaviors.getTelemetry(droneId);
        if (!targetTelemetry) return [];

        const nearby = [];

        for (const [otherId, assignment] of this.droneAssignments) {
            if (otherId === droneId) continue;

            const otherTelemetry = this.flightBehaviors.getTelemetry(otherId);
            if (!otherTelemetry) continue;

            const distance = Math.sqrt(
                Math.pow(targetTelemetry.position.x - otherTelemetry.position.x, 2) +
                Math.pow(targetTelemetry.position.z - otherTelemetry.position.z, 2)
            );

            if (distance <= range) {
                nearby.push(otherId);
            }
        }

        return nearby;
    }

    /**
     * Partage des découvertes entre drones
     */
    shareDiscoveries(droneId1, droneId2) {
        // Partage des points d'intérêt découverts
        for (const [poiId, poi] of this.pointsOfInterest) {
            if (poi.discoveredBy === droneId1) {
                // Diffuser la découverte au drone proche
                this.collectiveIntelligence.communicationNetwork.messageQueue.push({
                    from: droneId1,
                    to: droneId2,
                    type: 'POI_DISCOVERY',
                    data: poi,
                    timestamp: Date.now()
                });
            }
        }
    }

    /**
     * Coordination pour éviter exploration redondante
     */
    coordinateExploration(droneId1, droneId2) {
        const assignment1 = this.droneAssignments.get(droneId1);
        const assignment2 = this.droneAssignments.get(droneId2);

        // Vérifier chevauchement des zones
        const distance = Math.sqrt(
            Math.pow(assignment1.zone.center.x - assignment2.zone.center.x, 2) +
            Math.pow(assignment1.zone.center.z - assignment2.zone.center.z, 2)
        );

        const minDistance = assignment1.zone.radius + assignment2.zone.radius;

        if (distance < minDistance) {
            // Réassigner l'un des drones à une nouvelle zone
            this.reassignDroneZone(droneId2);
            if (!window.SILENT_MODE) {
                log(`🔄 Drone ${droneId2}: Réassignation pour éviter redondance`);
            }
        }
    }

    /**
     * Application Effet Wahoo entre drones
     */
    applyWahooInteraction(droneId1, droneId2, telemetry1, telemetry2) {
        const distance = Math.sqrt(
            Math.pow(telemetry1.position.x - telemetry2.position.x, 2) +
            Math.pow(telemetry1.position.z - telemetry2.position.z, 2)
        );

        // Force Wahoo (attraction/répulsion dynamique)
        const wahooStrength = this.config.wahooEffectStrength;
        const optimalDistance = 5.0; // Distance optimale entre drones

        let forceDirection = 1; // Attraction
        if (distance < optimalDistance * 0.5) {
            forceDirection = -1; // Répulsion si trop proches
        }

        const forceMagnitude = wahooStrength * Math.exp(-distance / optimalDistance);

        // Vecteur force
        const dx = telemetry2.position.x - telemetry1.position.x;
        const dz = telemetry2.position.z - telemetry1.position.z;
        const norm = Math.sqrt(dx * dx + dz * dz);

        if (norm > 0) {
            const physics1 = this.flightBehaviors.dronePhysics.get(droneId1);
            const physics2 = this.flightBehaviors.dronePhysics.get(droneId2);

            const forceX = (dx / norm) * forceMagnitude * forceDirection;
            const forceZ = (dz / norm) * forceMagnitude * forceDirection;

            // Appliquer forces Wahoo
            physics1.velocity.x += forceX * 0.1;
            physics1.velocity.z += forceZ * 0.1;

            physics2.velocity.x -= forceX * 0.1;
            physics2.velocity.z -= forceZ * 0.1;
        }
    }

    /**
     * Mise à jour objectif individuel d'un drone
     */
    updateDroneObjective(droneId, assignment, telemetry) {
        switch (assignment.role) {
            case 'EXPLORER':
                this.updateExplorerObjective(droneId, assignment, telemetry);
                break;
            case 'MAPPER':
                this.updateMapperObjective(droneId, assignment, telemetry);
                break;
            case 'SCOUT':
                this.updateScoutObjective(droneId, assignment, telemetry);
                break;
            case 'COORDINATOR':
                this.updateCoordinatorObjective(droneId, assignment, telemetry);
                break;
        }
    }

    /**
     * Objectif pour drone EXPLORER
     */
    updateExplorerObjective(droneId, assignment, telemetry) {
        // Chercher zone non explorée la plus proche
        const unexplored = this.findUnexploredAreas(telemetry.position);

        if (unexplored.length > 0) {
            const target = unexplored[0]; // Prendre la plus proche
            assignment.target = target.position;

            // Naviguer vers la cible
            this.navigateToTarget(droneId, target.position);
        }
    }

    /**
     * Objectif pour drone MAPPER
     */
    updateMapperObjective(droneId, assignment, telemetry) {
        // Se concentrer sur cartographie détaillée de zones partiellement explorées
        const partiallyMapped = this.findPartiallyMappedAreas(telemetry.position);

        if (partiallyMapped.length > 0) {
            const target = partiallyMapped[0];
            assignment.target = target.position;
            this.navigateToTarget(droneId, target.position);
        }
    }

    /**
     * Objectif pour drone SCOUT
     */
    updateScoutObjective(droneId, assignment, telemetry) {
        // Reconnaissance rapide de points d'intérêt potentiels
        const suspiciousAreas = this.identifySuspiciousAreas(telemetry.position);

        if (suspiciousAreas.length > 0) {
            const target = suspiciousAreas[0];
            assignment.target = target.position;
            this.navigateToTarget(droneId, target.position);
        }
    }

    /**
     * Objectif pour drone COORDINATOR
     */
    updateCoordinatorObjective(droneId, assignment, telemetry) {
        // Maintenir position centrale pour optimiser communication
        const swarmCenter = this.calculateSwarmCenter();
        assignment.target = swarmCenter;
        this.navigateToTarget(droneId, swarmCenter);
    }

    /**
     * Navigation vers cible
     */
    navigateToTarget(droneId, target) {
        const physics = this.flightBehaviors.dronePhysics.get(droneId);
        if (!physics) return;

        const dx = target.x - physics.position.x;
        const dz = target.z - physics.position.z;
        const distance = Math.sqrt(dx * dx + dz * dz);

        if (distance > 0.5) { // Seuil d'arrivée 50cm
            const speed = Math.min(this.flightBehaviors.config.maxVelocity, distance * 0.5);

            physics.velocity.x = (dx / distance) * speed;
            physics.velocity.z = (dz / distance) * speed;
        }
    }

    /**
     * Mise à jour cartographie de couverture
     */
    updateCoverageMapping() {
        for (const [droneId, assignment] of this.droneAssignments) {
            const telemetry = this.flightBehaviors.getTelemetry(droneId);
            if (!telemetry) continue;

            // Marquer cellules comme visitées dans rayon de détection
            const detectionRadius = 2.0; // mètres
            this.markAreaAsCovered(telemetry.position, detectionRadius, droneId);
        }

        // Calculer pourcentage de couverture
        const coveredCells = Array.from(this.coverageMap.values())
            .filter(cell => cell.coverage > 0.5).length;

        this.missionState.coveredArea = coveredCells;
        this.missionState.efficiency = coveredCells / this.missionState.totalArea;
    }

    /**
     * Marquer zone comme couverte
     */
    markAreaAsCovered(position, radius, droneId) {
        const resolution = this.config.coverageResolution;

        for (let x = position.x - radius; x <= position.x + radius; x += resolution) {
            for (let z = position.z - radius; z <= position.z + radius; z += resolution) {
                const distance = Math.sqrt(
                    Math.pow(x - position.x, 2) +
                    Math.pow(z - position.z, 2)
                );

                if (distance <= radius) {
                    const key = `${Math.round(x)}_${Math.round(z)}`;
                    const cell = this.coverageMap.get(key);

                    if (cell) {
                        cell.coverage = Math.min(1.0, cell.coverage + 0.2);
                        cell.lastVisited = Date.now();
                        cell.visitCount++;

                        // Ajouter trace stigmergie
                        this.addStigmergyTrace(position, this.stigmergySystem.traceTypes.EXPLORED, droneId);
                    }
                }
            }
        }
    }

    /**
     * Ajouter trace stigmergie
     */
    addStigmergyTrace(position, type, droneId) {
        const key = `${Math.round(position.x)}_${Math.round(position.z)}`;

        this.stigmergySystem.traces.set(key, {
            position: { ...position },
            type: type,
            intensity: 1.0,
            age: 0,
            droneId: droneId,
            timestamp: Date.now()
        });
    }

    /**
     * Mise à jour effet Wahoo
     */
    updateWahooEffect(deltaTime) {
        // Décroissance des traces stigmergie
        for (const [key, trace] of this.stigmergySystem.traces) {
            trace.age += deltaTime;
            trace.intensity *= this.config.stigmergyDecay;

            if (trace.intensity < 0.1) {
                this.stigmergySystem.traces.delete(key);
            }
        }

        // Détection leaders émergents
        this.detectEmergentLeaders();

        // Mise à jour métriques Wahoo
        this.collaborationMetrics.wahooEffectLevel = this.calculateWahooLevel();
    }

    /**
     * Détection leaders émergents
     */
    detectEmergentLeaders() {
        const dronePerformances = new Map();

        for (const [droneId, assignment] of this.droneAssignments) {
            const telemetry = this.flightBehaviors.getTelemetry(droneId);
            if (!telemetry) continue;

            // Score basé sur couverture et coordination
            const coverageScore = this.calculateDroneCoverageScore(droneId);
            const coordinationScore = this.calculateCoordinationScore(droneId);

            const totalScore = coverageScore * 0.6 + coordinationScore * 0.4;
            dronePerformances.set(droneId, totalScore);
        }

        // Identifier top performers comme leaders émergents
        const sortedDrones = Array.from(dronePerformances.entries())
            .sort((a, b) => b[1] - a[1]);

        this.wahooEffect.emergentLeaders.clear();
        for (let i = 0; i < Math.min(2, sortedDrones.length); i++) {
            this.wahooEffect.emergentLeaders.add(sortedDrones[i][0]);
        }
    }

    /**
     * Recherche zones non explorées
     */
    findUnexploredAreas(position) {
        return Array.from(this.coverageMap.values())
            .filter(cell => cell.coverage < 0.1)
            .sort((a, b) => {
                const distA = Math.sqrt(Math.pow(a.position.x - position.x, 2) + Math.pow(a.position.z - position.z, 2));
                const distB = Math.sqrt(Math.pow(b.position.x - position.x, 2) + Math.pow(b.position.z - position.z, 2));
                return distA - distB;
            });
    }

    /**
     * Recherche zones partiellement cartographiées
     */
    findPartiallyMappedAreas(position) {
        return Array.from(this.coverageMap.values())
            .filter(cell => cell.coverage > 0.1 && cell.coverage < 0.8)
            .sort((a, b) => b.priority - a.priority);
    }

    /**
     * Identification zones suspectes
     */
    identifySuspiciousAreas(position) {
        return Array.from(this.pointsOfInterest.values())
            .filter(poi => poi.type === 'ANOMALY' || poi.priority > 0.8)
            .sort((a, b) => b.priority - a.priority);
    }

    /**
     * Calcul centre de l'essaim
     */
    calculateSwarmCenter() {
        const activeDrones = Array.from(this.droneAssignments.keys());
        let sumX = 0, sumZ = 0, count = 0;

        for (const droneId of activeDrones) {
            const telemetry = this.flightBehaviors.getTelemetry(droneId);
            if (telemetry) {
                sumX += telemetry.position.x;
                sumZ += telemetry.position.z;
                count++;
            }
        }

        return count > 0 ? { x: sumX / count, z: sumZ / count } : { x: 0, z: 0 };
    }

    /**
     * Réassignation zone pour un drone
     */
    reassignDroneZone(droneId) {
        const unexplored = this.findUnexploredAreas({ x: 0, z: 0 });

        if (unexplored.length > 0) {
            const newZone = unexplored[Math.floor(Math.random() * Math.min(3, unexplored.length))];
            const assignment = this.droneAssignments.get(droneId);

            assignment.zone = {
                center: newZone.position,
                radius: this.config.explorationRadius * 0.3
            };
            assignment.lastUpdate = Date.now();
        }
    }

    /**
     * Adaptation dynamique des stratégies
     */
    adaptStrategies(deltaTime) {
        // Analyser performance actuelle
        const efficiency = this.missionState.efficiency;
        const wahooLevel = this.collaborationMetrics.wahooEffectLevel;

        // Adapter pattern selon performance
        if (efficiency < 0.3 && wahooLevel < 0.4) {
            this.missionState.pattern = this.scoutingPatterns.WAHOO_EFFECT;
        } else if (efficiency > 0.7) {
            this.missionState.pattern = this.scoutingPatterns.ADAPTIVE_PRIORITY;
        }

        // Adapter paramètres Wahoo
        if (this.collaborationMetrics.coordinationIndex < 0.5) {
            this.config.wahooEffectStrength *= 1.05; // Augmenter effet
        }
    }

    /**
     * Mise à jour métriques
     */
    updateMetrics() {
        this.collaborationMetrics.communicationEfficiency = this.calculateCommunicationEfficiency();
        this.collaborationMetrics.coordinationIndex = this.calculateCoordinationIndex();
        this.collaborationMetrics.redundancyLevel = this.calculateRedundancyLevel();
        this.collaborationMetrics.emergentBehaviors = this.wahooEffect.emergentLeaders.size;
    }

    /**
     * Calculs de métriques spécialisées
     */
    calculateCommunicationEfficiency() {
        const totalMessages = this.collectiveIntelligence.communicationNetwork.messageQueue.length;
        const activeDrones = this.droneAssignments.size;
        return activeDrones > 0 ? Math.min(1.0, totalMessages / (activeDrones * 10)) : 0;
    }

    calculateCoordinationIndex() {
        // Basé sur l'évitement de redondance et optimisation des trajectoires
        const redundancy = this.calculateRedundancyLevel();
        return Math.max(0, 1.0 - redundancy);
    }

    calculateRedundancyLevel() {
        // Analyser chevauchement des zones d'assignation
        const assignments = Array.from(this.droneAssignments.values());
        let overlaps = 0;
        let total = 0;

        for (let i = 0; i < assignments.length; i++) {
            for (let j = i + 1; j < assignments.length; j++) {
                const dist = Math.sqrt(
                    Math.pow(assignments[i].zone.center.x - assignments[j].zone.center.x, 2) +
                    Math.pow(assignments[i].zone.center.z - assignments[j].zone.center.z, 2)
                );
                const minDist = assignments[i].zone.radius + assignments[j].zone.radius;

                if (dist < minDist) overlaps++;
                total++;
            }
        }

        return total > 0 ? overlaps / total : 0;
    }

    calculateWahooLevel() {
        const leaders = this.wahooEffect.emergentLeaders.size;
        const groupCohesion = this.wahooEffect.groupCohesion;
        return (leaders / 2 + groupCohesion) / 2;
    }

    calculateDroneCoverageScore(droneId) {
        // Score basé sur zones couvertes par ce drone
        let score = 0;
        for (const [key, cell] of this.coverageMap) {
            if (cell.lastVisited > 0) {
                score += cell.coverage;
            }
        }
        return score / this.coverageMap.size;
    }

    calculateCoordinationScore(droneId) {
        // Score basé sur communication et évitement redondance
        const nearbyDrones = this.findNearbyDrones(droneId, this.config.communicationRange);
        return nearbyDrones.length / Math.max(1, this.droneAssignments.size - 1);
    }

    /**
     * Mise à jour des traces de stigmergie pour un drone
     */
    updateStigmergyTraces(droneId, telemetry) {
        if (!telemetry || !telemetry.position) return;

        // Créer une trace de stigmergie simple
        const key = `${Math.floor(telemetry.position.x / 5)}_${Math.floor(telemetry.position.z / 5)}`;
        
        // Pour l'instant, juste enregistrer le passage sans système complexe
        if (!this.stigmergyTraces) {
            this.stigmergyTraces = new Map();
        }
        
        const trace = this.stigmergyTraces.get(key) || { 
            visits: 0, 
            lastVisit: 0, 
            density: 0,
            drones: new Set()
        };
        
        trace.visits++;
        trace.lastVisit = Date.now();
        trace.drones.add(droneId);
        trace.density = trace.visits / trace.drones.size;
        
        this.stigmergyTraces.set(key, trace);
    }

    /**
     * Mise à jour de l'objectif d'un drone selon son assignment
     */
    updateDroneObjective(droneId, assignment, telemetry) {
        if (!assignment || !telemetry) return;

        // Mettre à jour l'objectif selon le rôle
        switch (assignment.role) {
            case 'SCOUT':
                this.updateScoutObjective(droneId, assignment, telemetry);
                break;
            case 'LEADER':
                this.updateLeaderObjective(droneId, assignment, telemetry);
                break;
            case 'FOLLOWER':
                this.updateFollowerObjective(droneId, assignment, telemetry);
                break;
            default:
                // Objectif par défaut: exploration
                assignment.targetArea = this.getNextExplorationArea(droneId);
                break;
        }
    }

    /**
     * Objectif pour drone scout
     */
    updateScoutObjective(droneId, assignment, telemetry) {
        // Scout explore les zones non couvertes
        if (!assignment.targetArea || this.isAreaCovered(assignment.targetArea)) {
            assignment.targetArea = this.getNextExplorationArea(droneId);
        }
    }

    /**
     * Objectif pour drone leader
     */
    updateLeaderObjective(droneId, assignment, telemetry) {
        // Leader coordonne et guide les followers
        assignment.targetArea = this.getBestLeaderPosition(droneId, telemetry);
    }

    /**
     * Objectif pour drone follower  
     */
    updateFollowerObjective(droneId, assignment, telemetry) {
        // Follower suit son leader assigné
        const leaderId = assignment.leaderId;
        if (leaderId) {
            const leaderTelemetry = this.flightBehaviors.getTelemetry(leaderId);
            if (leaderTelemetry) {
                assignment.targetArea = {
                    x: leaderTelemetry.position.x + (Math.random() - 0.5) * 10,
                    z: leaderTelemetry.position.z + (Math.random() - 0.5) * 10
                };
            }
        }
    }

    /**
     * Vérifier si une zone est suffisamment couverte
     */
    isAreaCovered(area) {
        if (!area) return false;
        
        const key = `${Math.floor(area.x / 10)}_${Math.floor(area.z / 10)}`;
        const cell = this.coverageMap.get(key);
        return cell && cell.coverage > 0.7;
    }

    /**
     * Obtenir la prochaine zone à explorer
     */
    getNextExplorationArea(droneId) {
        // Vérification de sécurité
        if (!this.config.missionArea) {
            warn('⚠️ missionArea non définie, utilisation valeurs par défaut');
            this.config.missionArea = { width: 100, height: 100 };
        }
        
        // Simple: choisir une zone aléatoire dans les limites
        return {
            x: (Math.random() - 0.5) * this.config.missionArea.width,
            z: (Math.random() - 0.5) * this.config.missionArea.height
        };
    }

    /**
     * Obtenir la meilleure position pour un leader
     */
    getBestLeaderPosition(droneId, telemetry) {
        // Position centrale pour coordonner les followers
        return {
            x: telemetry.position.x + (Math.random() - 0.5) * 5,
            z: telemetry.position.z + (Math.random() - 0.5) * 5
        };
    }

    /**
     * API publique - État de la mission
     */
    getMissionStatus() {
        return {
            phase: this.missionState.phase,
            progress: this.missionState.efficiency,
            coverage: `${(this.missionState.efficiency * 100).toFixed(1)}%`,
            activeDrones: this.droneAssignments.size,
            discoveries: this.pointsOfInterest.size,
            collaborationMetrics: { ...this.collaborationMetrics },
            elapsedTime: Date.now() - this.missionState.startTime,
            wahooEffectActive: this.missionState.pattern === this.scoutingPatterns.WAHOO_EFFECT
        };
    }

    /**
     * API publique - Données de couverture
     */
    getCoverageData() {
        return {
            totalCells: this.missionState.totalArea,
            coveredCells: this.missionState.coveredArea,
            efficiency: this.missionState.efficiency,
            coverageMap: Array.from(this.coverageMap.entries()),
            stigmergyTraces: Array.from(this.stigmergySystem.traces.entries())
        };
    }

    /**
     * API publique - Données collaborative
     */
    getCollaborativeData() {
        return {
            droneAssignments: Array.from(this.droneAssignments.entries()),
            pointsOfInterest: Array.from(this.pointsOfInterest.entries()),
            emergentLeaders: Array.from(this.wahooEffect.emergentLeaders),
            communicationNetwork: this.collectiveIntelligence.communicationNetwork,
            wahooEffect: { ...this.wahooEffect }
        };
    }
}
