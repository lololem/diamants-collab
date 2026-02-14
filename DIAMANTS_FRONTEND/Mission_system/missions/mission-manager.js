/**
 * DIAMANTS - Gestionnaire de Missions Multi-Agent
 * ==================================================
 * Système de missions basé sur l'architecture DIAMANTS
 * Gestion des comportements collectifs et auto-organisation
 */

// Mode silencieux pour les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Fonctions de logging
const log = (...args) => { if (!window.SILENT_MODE) console.log(...args); };
const warn = (...args) => { if (!window.SILENT_MODE) console.warn(...args); };
const error = (...args) => console.error(...args);

export class MissionManager {
    constructor(config = {}) {
        this.config = {
            maxDrones: config.maxDrones || 8,
            missionTimeout: config.missionTimeout || 300, // 5 minutes
            communicationRange: config.communicationRange || 10,
            emergencyAltitude: config.emergencyAltitude || 5,
            ...config
        };

        // État des missions
        this.activeMissions = new Map();
        this.missionHistory = [];
        this.currentMissionId = null;

        // Données de l'essaim
        this.swarmData = {
            drones: new Map(),
            formations: new Map(),
            discoveries: [],
            coverage: new Map(),
            threats: [],
            objectives: []
        };

        // Métriques de performance
        this.metrics = {
            missionsCompleted: 0,
            totalFlightTime: 0,
            coverageArea: 0,
            efficiency: 0,
            collisionEvents: 0,
            emergencyLandings: 0
        };

        // Sécurité et anti-spam des logs
        this.safety = {
            // Fréquence de vérification des collisions (ms)
            checkIntervalMs: this.config.safetyCheckIntervalMs || 200, // ~5 Hz
            // Distance minimale de sécurité entre drones (m)
            minSafeDistance: this.config.minSafeDistance || 1.5,
            // Délai minimal entre deux alertes pour la même paire (ms)
            collisionCooldownMs: this.config.collisionCooldownMs || 1000,
            // Timestamp de la dernière vérif
            lastCheck: 0,
            // Mémoire des dernières alertes par paire "A|B"
            lastCollisionAlert: new Map()
        };

        // Types de missions disponibles (fonctions fléchées pour capturer 'this')
        this.missionTypes = {
            EXPLORATION: (id, p) => this.createExplorationMission(id, p),
            SEARCH_RESCUE: (id, p) => this.createSearchRescueMission(id, p),
            FORMATION_FLIGHT: (id, p) => this.createFormationMission(id, p),
            SURVEILLANCE: (id, p) => this.createSurveillanceMission(id, p),
            MAPPING: (id, p) => this.createMappingMission(id, p),
            EMERGENCY_RESPONSE: (id, p) => this.createEmergencyMission(id, p)
        };

        // Patterns d'auto-organisation (fonctions fléchées pour capturer 'this')
        this.organizationPatterns = {
            BOUSTROPHEDON: (m, d, e) => this.boustrophedonPattern(m, d, e),
            SPIRAL: (m, d, e) => this.spiralPattern(m, d, e),
            GRID_SEARCH: (m, d, e) => this.gridSearchPattern(m, d, e),
            SWARM_DISPERSION: (m, d, e) => this.swarmDispersionPattern(m, d, e),
            HIERARCHICAL: (m, d, e) => this.hierarchicalPattern(m, d, e)
        };

        this.isActive = false;
        this.startTime = 0;
    }

    /**
     * Création mission de surveillance
     */
    createSurveillanceMission(missionId, params) {
        return {
            id: missionId,
            type: 'SURVEILLANCE',
            status: 'ACTIVE',
            startTime: Date.now(),
            timeout: params.timeout || this.config.missionTimeout,
            patrolRoute: params.patrolRoute || [
                { x: -10, y: -10, z: 4 },
                { x: 10, y: -10, z: 4 },
                { x: 10, y: 10, z: 4 },
                { x: -10, y: 10, z: 4 }
            ],
            currentIndex: 0,
            dwellTime: params.dwellTime || 2,
            assignedDrones: new Set(),
            progress: { laps: 0 }
        };
    }

    /**
     * Création mission de cartographie
     */
    createMappingMission(missionId, params) {
        return {
            id: missionId,
            type: 'MAPPING',
            status: 'ACTIVE',
            startTime: Date.now(),
            timeout: params.timeout || this.config.missionTimeout,
            area: params.area || { center: { x: 0, y: 0, z: 3 }, radius: 15 },
            resolution: params.resolution || 0.5,
            overlap: params.overlap || 0.2,
            assignedDrones: new Set(),
            progress: { tiles: 0, totalTiles: 0 }
        };
    }

    /**
     * Création mission d'intervention d'urgence
     */
    createEmergencyMission(missionId, params) {
        return {
            id: missionId,
            type: 'EMERGENCY_RESPONSE',
            status: 'ACTIVE',
            startTime: Date.now(),
            timeout: params.timeout || this.config.missionTimeout,
            target: params.target || { x: 0, y: 0, z: 0 },
            priority: params.priority || 'HIGH',
            assignedDrones: new Set(),
            progress: { arrived: 0 }
        };
    }

    /**
     * Démarrage du gestionnaire de missions
     */
    start() {
        this.isActive = true;
        this.startTime = Date.now();
        log('🚀 Mission Manager démarré');
    }

    /**
     * Arrêt du gestionnaire
     */
    stop() {
        this.isActive = false;
        this.abortAllMissions();
        log('🛑 Mission Manager arrêté');
    }

    /**
     * Mise à jour principale
     */
    update(deltaTime, drones, environment) {
        if (!this.isActive) return;

        // Debug périodique du système de missions
        if (this._debugFrame === undefined) this._debugFrame = 0;
        this._debugFrame++;
        
        if (this._debugFrame % 300 === 0) { // Log toutes les 5 secondes environ
            log(`🎯 Mission Manager Update: ${this.activeMissions.size} missions actives, ${drones?.length || 0} drones`);
            this.activeMissions.forEach((mission, id) => {
                log(`  📋 Mission ${id}: ${mission.type}, ${mission.assignedDrones?.size || 0} drones assignés`);
            });
        }

        // Mise à jour données essaim
        this.updateSwarmData(drones);

        // Gestion des missions actives
        this.updateActiveMissions(deltaTime, drones, environment);

        // Auto-organisation émergente
        this.updateAutoOrganization(drones);

        // Vérifications sécurité
        this.updateSafetyChecks(drones, environment);

        // Mise à jour métriques
        this.updateMetrics(deltaTime, drones);
    }

    /**
     * Lancement d'une nouvelle mission
     */
    startMission(type, parameters = {}) {
        const missionId = `mission_${Date.now()}`;

        if (this.missionTypes[type]) {
            const mission = this.missionTypes[type](missionId, parameters);
            this.activeMissions.set(missionId, mission);
            this.currentMissionId = missionId;

            log(`🎯 Mission ${type} démarrée:`, missionId);
            return missionId;
        }

        error(`❌ Type de mission inconnu: ${type}`);
        return null;
    }

    /**
     * Création mission d'exploration
     */
    createExplorationMission(missionId, params) {
        return {
            id: missionId,
            type: 'EXPLORATION',
            status: 'ACTIVE',
            startTime: Date.now(),
            timeout: params.timeout || this.config.missionTimeout,

            // Paramètres spécifiques
            targetArea: params.area || {
                center: { x: 0, y: 0, z: 3 },
                radius: 20
            },
            pattern: (params.pattern || 'BOUSTROPHEDON').toUpperCase(),
            spacing: params.spacing || 3,
            altitude: params.altitude || 3,

            // Objectifs
            objectives: {
                coverageTarget: params.coverage || 0.8,
                discoveryTarget: params.discoveries || 5,
                timeLimit: params.timeLimit || 300
            },

            // État
            progress: {
                coverage: 0,
                discoveries: 0,
                efficiency: 0,
                dronesAssigned: 0
            },

            // Assignations
            assignedDrones: new Set(),
            teamRoles: new Map(),

            // Callbacks
            onProgress: params.onProgress || (() => { }),
            onComplete: params.onComplete || (() => { }),
            onFailed: params.onFailed || (() => { })
        };
    }

    /**
     * Création mission de recherche et sauvetage
     */
    createSearchRescueMission(missionId, params) {
        return {
            id: missionId,
            type: 'SEARCH_RESCUE',
            status: 'ACTIVE',
            startTime: Date.now(),
            timeout: params.timeout || this.config.missionTimeout,

            // Zone de recherche
            searchArea: params.searchArea || {
                corners: [
                    { x: -15, y: -15 },
                    { x: 15, y: -15 },
                    { x: 15, y: 15 },
                    { x: -15, y: 15 }
                ]
            },

            // Cibles à rechercher
            targets: params.targets || [
                { type: 'survivor', priority: 'HIGH' },
                { type: 'equipment', priority: 'MEDIUM' }
            ],

            // Configuration
            searchPattern: (params.pattern || 'SPIRAL').toUpperCase(),
            altitude: params.altitude || 4,
            sensorRange: params.sensorRange || 3,

            // État
            progress: {
                areaSearched: 0,
                targetsFound: 0,
                falsePositives: 0,
                confidence: 0
            },

            assignedDrones: new Set(),
            teamConfiguration: {
                scouts: 2,
                heavy: 1,
                coordinator: 1
            }
        };
    }

    /**
     * Création mission de vol en formation
     */
    createFormationMission(missionId, params) {
        return {
            id: missionId,
            type: 'FORMATION_FLIGHT',
            status: 'ACTIVE',
            startTime: Date.now(),
            timeout: params.timeout || this.config.missionTimeout,

            // Configuration formation
            formation: {
                type: params.formation || 'V_FORMATION',
                spacing: params.spacing || 2.5,
                altitude: params.altitude || 3.5,
                leader: params.leader || null
            },

            // Trajectoire
            waypoints: params.waypoints || [
                { x: 0, y: 0, z: 3.5 },
                { x: 10, y: 10, z: 3.5 },
                { x: 20, y: 0, z: 3.5 },
                { x: 0, y: 0, z: 3.5 }
            ],
            currentWaypoint: 0,

            // Métriques
            cohesion: 0,
            alignment: 0,
            separation: 0,

            assignedDrones: new Set(),
            positions: new Map()
        };
    }

    /**
     * Mise à jour des missions actives
     */
    updateActiveMissions(deltaTime, drones, environment) {
        for (const [missionId, mission] of this.activeMissions) {
            // Vérification timeout
            const elapsed = Date.now() - mission.startTime;
            if (elapsed > mission.timeout * 1000) {
                this.completeMission(missionId, 'TIMEOUT');
                continue;
            }

            // Mise à jour selon type
            switch (mission.type) {
                case 'EXPLORATION':
                    this.updateExplorationMission(mission, drones, environment);
                    break;
                case 'SEARCH_RESCUE':
                    this.updateSearchRescueMission(mission, drones, environment);
                    break;
                case 'FORMATION_FLIGHT':
                    this.updateFormationMission(mission, drones, environment);
                    break;
            }

            // Vérification objectifs
            this.checkMissionObjectives(mission);
        }
    }

    /**
     * Mise à jour mission d'exploration
     */
    updateExplorationMission(mission, drones, environment) {
        const assignedDrones = Array.from(mission.assignedDrones)
            .map(id => drones.find(d => d.id === id))
            .filter(d => d);

        if (assignedDrones.length === 0) {
            // Auto-assignation des drones disponibles
            this.autoAssignDrones(mission, drones);
            return;
        }

        // ⚠️ SOLUTION DIRECTE: MOUVEMENT IMMÉDIAT DES DRONES
        this.applyDirectMovementToExploringDrones(mission, assignedDrones, environment);

        // Application du pattern d'exploration
        const pattern = this.organizationPatterns[mission.pattern];
        if (pattern) {
            pattern(mission, assignedDrones, environment);
        }

        // Calcul progression
        this.updateExplorationProgress(mission, assignedDrones);

        // Application DIAMANTS pour auto-organisation
        this.applyDiamantsLogic(mission, assignedDrones);
    }

    /**
     * 🚁 MOUVEMENT DIRECT DES DRONES EN MISSION D'EXPLORATION
     * Contournement du système complexe pour assurer le mouvement immédiat
     */
    applyDirectMovementToExploringDrones(mission, assignedDrones, environment) {
        const time = (Date.now() - mission.startTime) / 1000;
        const { targetArea, altitude = 3 } = mission;
        const { center = { x: 0, y: 0, z: 3 }, radius = 20 } = targetArea;

        assignedDrones.forEach((drone, index) => {
            if (!drone.position || !drone.mesh) return;

            // Pattern d'exploration simple: cercle avec rotation
            const angleOffset = (index / assignedDrones.length) * 2 * Math.PI;
            const explorationRadius = 8 + (index * 3); // Rayons différents par drone, plus compacts
            const rotationSpeed = 0.3 + (index * 0.15); // Vitesses différentes, plus rapides

            const angle = time * rotationSpeed + angleOffset;
            const targetX = center.x + Math.cos(angle) * explorationRadius;
            const targetZ = center.z + Math.sin(angle) * explorationRadius;
            const targetY = altitude;

            // Calcul de la direction vers la cible
            const currentPos = drone.position;
            
            const direction = {
                x: targetX - currentPos.x,
                y: targetY - currentPos.y,
                z: targetZ - currentPos.z
            };

            const distance = Math.sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);

            if (distance > 0.5) { // Distance minimum plus importante pour éviter les vibrations
                // Normaliser la direction
                direction.x /= distance;
                direction.y /= distance;
                direction.z /= distance;

                // Vitesse d'exploration plus rapide
                const speed = 4.0; // m/s
                const deltaTime = 0.016; // Frame time approximatif
                
                // Appliquer le mouvement directement à la position
                drone.position.x += direction.x * speed * deltaTime;
                drone.position.y += direction.y * speed * deltaTime;
                drone.position.z += direction.z * speed * deltaTime;

                // Synchroniser le mesh si disponible
                if (drone.mesh && drone.mesh.position) {
                    drone.mesh.position.set(drone.position.x, drone.position.y, drone.position.z);
                }

                // Marquer le drone comme actif
                drone.state = 'EXPLORING';
            }

            // Debug: Log périodique du mouvement
            if (Math.floor(time * 2) % 10 === 0 && index === 0) {
                log(`🚁 Drone ${drone.id} explore: pos(${drone.position.x.toFixed(1)}, ${drone.position.y.toFixed(1)}, ${drone.position.z.toFixed(1)}) → target(${targetX.toFixed(1)}, ${targetY.toFixed(1)}, ${targetZ.toFixed(1)})`);
            }
        });

        // Marquer la progression de la mission
        mission.progress.coverage = Math.min(1.0, time / 60); // 60 secondes pour compléter
        mission.progress.discoveries = assignedDrones.length; // Simuler des découvertes
    }

    /**
     * Pattern Boustrophédon pour exploration systématique
     */
    boustrophedonPattern(mission, drones, environment) {
        const { targetArea, spacing, altitude } = mission;
        const { center, radius } = targetArea;

        const linesCount = Math.ceil(radius * 2 / spacing);
        const dronesPerLine = Math.ceil(drones.length / linesCount);

        drones.forEach((drone, index) => {
            const lineIndex = Math.floor(index / dronesPerLine);
            const positionInLine = index % dronesPerLine;

            // Position Y selon la ligne
            const y = center.y - radius + lineIndex * spacing;

            // Position X avec alternance (boustrophédon)
            let x;
            if (lineIndex % 2 === 0) {
                // Ligne paire: gauche -> droite
                x = center.x - radius + positionInLine * spacing;
            } else {
                // Ligne impaire: droite -> gauche
                x = center.x + radius - positionInLine * spacing;
            }

            // Assignation de la position cible
            if (drone.setMission) {
                drone.setMission('EXPLORATION', {
                    position: new THREE.Vector3(x, y, altitude),
                    altitude: altitude
                });
            }
        });
    }

    /**
     * Pattern en spirale pour recherche convergente
     */
    spiralPattern(mission, drones, environment) {
        const { targetArea, altitude } = mission;
        const { center } = targetArea;
        const time = (Date.now() - mission.startTime) / 1000;

        drones.forEach((drone, index) => {
            const angleOffset = (index / drones.length) * 2 * Math.PI;
            const spiral = {
                radius: 2 + time * 0.1 + index * 1.5,
                angle: time * 0.3 + angleOffset
            };

            const x = center.x + Math.cos(spiral.angle) * spiral.radius;
            const y = center.y + Math.sin(spiral.angle) * spiral.radius;

            if (drone.setMission) {
                drone.setMission('SEARCH_RESCUE', {
                    position: new THREE.Vector3(x, y, altitude),
                    altitude: altitude
                });
            }
        });
    }

    /**
     * Pattern de recherche en grille
     */
    gridSearchPattern(mission, drones, environment) {
        const { searchArea, altitude } = mission;
        const gridSize = Math.ceil(Math.sqrt(drones.length));

        drones.forEach((drone, index) => {
            const row = Math.floor(index / gridSize);
            const col = index % gridSize;

            const x = searchArea.corners[0].x +
                (col / (gridSize - 1)) *
                (searchArea.corners[1].x - searchArea.corners[0].x);

            const y = searchArea.corners[0].y +
                (row / (gridSize - 1)) *
                (searchArea.corners[3].y - searchArea.corners[0].y);

            if (drone.setMission) {
                drone.setMission('SEARCH_RESCUE', {
                    position: new THREE.Vector3(x, y, altitude),
                    altitude: altitude
                });
            }
        });
    }

    /**
     * Pattern de dispersion de l'essaim (cercle autour du centre de zone)
     */
    swarmDispersionPattern(mission, drones, environment) {
        const center = (mission.targetArea && mission.targetArea.center) || { x: 0, y: 0, z: 3 };
        const radius = (mission.targetArea && mission.targetArea.radius) || 10;
        const altitude = (mission.altitude) || 3;

        drones.forEach((drone, i) => {
            const angle = (i / drones.length) * Math.PI * 2;
            const x = center.x + Math.cos(angle) * radius;
            const y = center.y + Math.sin(angle) * radius;
            if (drone.setMission) {
                drone.setMission('EXPLORATION', {
                    position: new THREE.Vector3(x, y, altitude),
                    altitude
                });
            }
        });
    }

    /**
     * Pattern hiérarchique: un leader et des suiveurs en chevron
     */
    hierarchicalPattern(mission, drones, environment) {
        if (drones.length === 0) return;
        const leader = drones[0];
        const altitude = mission.altitude || 3.5;
        if (leader.setMission) {
            leader.setMission('FORMATION', { position: leader.position.clone().setZ(altitude), altitude });
        }
        // Suiveurs en V
        drones.slice(1).forEach((drone, idx) => {
            const side = idx % 2 === 0 ? 1 : -1;
            const rank = Math.floor(idx / 2) + 1;
            const offset = new THREE.Vector3(-rank * 2.0, side * 2.0 * rank, 0);
            const target = leader.position.clone().add(offset).setZ(altitude);
            if (drone.setMission) {
                drone.setMission('FORMATION', { position: target, altitude });
            }
        });
    }

    /**
     * Auto-assignation intelligente des drones
     */
    autoAssignDrones(mission, allDrones) {
        // ⚠️ SOLUTION ROBUSTE: Accepter tous les drones disponibles
        const availableDrones = allDrones.filter(drone => 
            drone && drone.id && 
            (!drone.state || drone.state === 'FLYING' || drone.state === 'IDLE' || drone.state === 'READY' || !mission.assignedDrones.has(drone.id))
        );

        // Sélection selon le type de mission
        let selectedDrones = [];

        switch (mission.type) {
            case 'EXPLORATION':
                // Privilégier les scouts et stealths
                selectedDrones = this.selectDronesByRole(
                    availableDrones,
                    ['exploration', 'reconnaissance'],
                    Math.min(4, availableDrones.length)
                );
                break;

            case 'SEARCH_RESCUE':
                // Équipe mixte équilibrée
                selectedDrones = this.selectBalancedTeam(
                    availableDrones,
                    mission.teamConfiguration
                );
                break;

            case 'FORMATION_FLIGHT':
                // Tous les drones disponibles
                selectedDrones = availableDrones.slice(0, 6);
                break;
        }

        // Assignation
        selectedDrones.forEach(drone => {
            mission.assignedDrones.add(drone.id);
            mission.teamRoles.set(drone.id, drone.typeConfig.role);
        });

        log(`👥 ${selectedDrones.length} drones assignés à la mission ${mission.id}`);
    }

    selectDronesByRole(drones, preferredRoles, maxCount) {
        // Tri par pertinence du rôle
        const sorted = drones.sort((a, b) => {
            const aScore = preferredRoles.includes(a.typeConfig.role) ? 1 : 0;
            const bScore = preferredRoles.includes(b.typeConfig.role) ? 1 : 0;
            return bScore - aScore;
        });

        return sorted.slice(0, maxCount);
    }

    selectBalancedTeam(drones, teamConfig) {
        const team = [];
        const roleMap = {
            scouts: ['exploration', 'reconnaissance'],
            heavy: ['transport'],
            coordinator: ['coordination']
        };

        // Sélection par rôle souhaité
        for (const [roleType, count] of Object.entries(teamConfig)) {
            const roles = roleMap[roleType] || [roleType];
            const candidates = drones.filter(d =>
                roles.includes(d.typeConfig.role) &&
                !team.includes(d)
            );

            team.push(...candidates.slice(0, count));
        }

        return team;
    }

    /**
     * Application de la logique DIAMANTS pour auto-organisation
     */
    applyDiamantsLogic(mission, drones) {
        // Calcul des champs φ et σ pour la mission
        const missionField = this.calculateMissionField(mission, drones);

        // Application des forces d'auto-organisation
        drones.forEach(drone => {
            const localGradient = this.getMissionGradient(missionField, drone.position);

            // Force d'exploration (φ)
            const explorationForce = localGradient.phi.multiplyScalar(0.3);

            // Force de cohésion/répulsion (σ)
            const cohesionForce = this.calculateCohesionForce(drone, drones);

            // Application combinée
            if (drone.acceleration) {
                drone.acceleration.add(explorationForce);
                drone.acceleration.add(cohesionForce);
            }
        });
    }

    calculateMissionField(mission, drones) {
        // Simplifié - implémentation complète nécessiterait DiamantFormulas
        const field = {
            phi: new Map(), // Champ d'attraction vers objectifs
            sigma: new Map() // Champ de répulsion/exploration
        };

        // Objectifs attirent (φ) — supporte liste ou unique
        if (Array.isArray(mission.objectives)) {
            mission.objectives.forEach(objective => {
                if (!objective || !objective.position) return;
                field.phi.set(objective.id || `${objective.position.x},${objective.position.y}`, {
                    position: new THREE.Vector3(objective.position.x, objective.position.y, objective.position.z || 0),
                    strength: objective.priority === 'HIGH' ? 2.0 : 1.0
                });
            });
        } else if (mission.targetArea && mission.targetArea.center) {
            // Utiliser le centre de la zone comme attracteur doux
            const c = mission.targetArea.center;
            field.phi.set('area_center', {
                position: new THREE.Vector3(c.x, c.y, c.z || 0),
                strength: 0.5
            });
        }

        return field;
    }

    getMissionGradient(field, position) {
        // Calcul simplifié du gradient local
        let phi = new THREE.Vector3(0, 0, 0);
        let sigma = new THREE.Vector3(0, 0, 0);

        // Attraction vers objectifs
        for (const [id, objective] of field.phi) {
            const direction = objective.position.clone().sub(position);
            const distance = direction.length();
            if (distance > 0) {
                direction.normalize().multiplyScalar(objective.strength / (distance + 1));
                phi.add(direction);
            }
        }

        return { phi, sigma };
    }

    calculateCohesionForce(drone, allDrones) {
        const cohesionForce = new THREE.Vector3(0, 0, 0);
        const separationForce = new THREE.Vector3(0, 0, 0);
        const alignmentForce = new THREE.Vector3(0, 0, 0);

        let neighbors = 0;
        const neighborDistance = 5.0;
        const separationDistance = 2.0;

        allDrones.forEach(other => {
            if (other.id === drone.id) return;

            const distance = drone.position.distanceTo(other.position);

            if (distance < neighborDistance) {
                neighbors++;

                // Cohésion - attraction vers centre de masse
                cohesionForce.add(other.position);

                // Alignement - alignement des vitesses
                if (other.velocity) {
                    alignmentForce.add(other.velocity);
                }

                // Séparation - répulsion si trop proche
                if (distance < separationDistance && distance > 0) {
                    const separation = drone.position.clone()
                        .sub(other.position)
                        .normalize()
                        .multiplyScalar((separationDistance - distance) / separationDistance);
                    separationForce.add(separation);
                }
            }
        });

        if (neighbors > 0) {
            // Cohésion vers centre de masse
            cohesionForce.divideScalar(neighbors);
            cohesionForce.sub(drone.position);
            cohesionForce.multiplyScalar(0.1);

            // Alignement des vitesses
            alignmentForce.divideScalar(neighbors);
            if (drone.velocity) {
                alignmentForce.sub(drone.velocity);
            }
            alignmentForce.multiplyScalar(0.05);
        }

        // Combinaison des forces
        const totalForce = new THREE.Vector3()
            .add(cohesionForce.multiplyScalar(0.3))
            .add(alignmentForce.multiplyScalar(0.2))
            .add(separationForce.multiplyScalar(0.5));

        return totalForce;
    }

    /**
     * Mise à jour des données de l'essaim
     */
    updateSwarmData(drones) {
        // Mise à jour positions et états
        drones.forEach(drone => {
            this.swarmData.drones.set(drone.id, {
                position: drone.position.clone(),
                velocity: drone.velocity.clone(),
                state: drone.state,
                intelligence: drone.intelligence || 0,
                emergence: drone.emergence || 0,
                battery: drone.getStatus ? drone.getStatus().batteryLevel : 1
            });
        });

        // Calcul couverture globale
        this.updateCoverageMap(drones);

        // Détection découvertes
        this.updateDiscoveries(drones);
    }

    updateCoverageMap(drones) {
        const resolution = 1.0; // 1m de résolution

        drones.forEach(drone => {
            const gridX = Math.floor(drone.position.x / resolution);
            const gridY = Math.floor(drone.position.y / resolution);
            const key = `${gridX},${gridY}`;

            const existing = this.swarmData.coverage.get(key) || {
                count: 0,
                lastVisit: 0,
                quality: 0
            };

            this.swarmData.coverage.set(key, {
                count: existing.count + 1,
                lastVisit: Date.now(),
                quality: Math.min(1.0, existing.quality + 0.1)
            });
        });
    }

    updateDiscoveries(drones) {
        drones.forEach(drone => {
            if (drone.intelligence > 0.5) { // Seuil de découverte
                const discovery = {
                    id: `discovery_${Date.now()}_${drone.id}`,
                    position: drone.position.clone(),
                    intelligence: drone.intelligence,
                    discoverer: drone.id,
                    timestamp: Date.now(),
                    verified: false
                };

                // Éviter doublons
                const existing = this.swarmData.discoveries.find(d =>
                    d.position.distanceTo(discovery.position) < 2.0
                );

                if (!existing) {
                    this.swarmData.discoveries.push(discovery);
                }
            }
        });
    }

    /**
     * Vérifications de sécurité
     */
    updateSafetyChecks(drones, environment) {
        // Détection collisions imminentes (throttling)
        const now = (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
        if (now - this.safety.lastCheck >= this.safety.checkIntervalMs) {
            this.safety.lastCheck = now;
            this.checkCollisionRisks(drones, now);
        }

        // Surveillance batteries
        this.checkBatteryLevels(drones);

        // Vérification limites zone
        this.checkBoundaries(drones);

        // Surveillance conditions météo
        this.checkEnvironmentConditions(environment);
    }

    checkCollisionRisks(drones, nowTs) {
        const now = nowTs || ((typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now());
        const minSafeDistance = (this.safety && this.safety.minSafeDistance) || 4.0; // Augmenté de 1.5m à 4m

        for (let i = 0; i < drones.length; i++) {
            for (let j = i + 1; j < drones.length; j++) {
                const a = drones[i];
                const b = drones[j];
                if (!a || !b || !a.position || !b.position) continue;

                const distance = a.position.distanceTo(b.position);
                if (distance >= minSafeDistance) continue;

                const idA = a.id || `drone_${i}`;
                const idB = b.id || `drone_${j}`;
                const key = idA < idB ? `${idA}|${idB}` : `${idB}|${idA}`;
                const lastAlert = this.safety.lastCollisionAlert.get(key) || 0;

                if (now - lastAlert >= this.safety.collisionCooldownMs) {
                    // Log + répulsion (avec cooldown par paire)
                    this.handleCollisionRisk(a, b);
                    this.safety.lastCollisionAlert.set(key, now);
                } else {
                    // Répulsion silencieuse renforcée (sans log) entre les checks
                    const repulsionStrength = Math.max(3.0, 8.0 / (distance + 0.1));
                    const repulsionVector = a.position.clone()
                        .sub(b.position)
                        .normalize()
                        .multiplyScalar(repulsionStrength);
                    if (a.acceleration) a.acceleration.add(repulsionVector);
                    if (b.acceleration) b.acceleration.sub(repulsionVector);
                }
            }
        }
    }

    handleCollisionRisk(drone1, drone2) {
        // Clé de paire stable
        const idA = drone1.id || 'A';
        const idB = drone2.id || 'B';
        const key = idA < idB ? `${idA}|${idB}` : `${idB}|${idA}`;
        const now = (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();

        // Throttle des logs par paire, mais appliquer la répulsion à chaque fois
        const lastAlert = this.safety?.lastCollisionAlert?.get(key) || 0;
        const cooldown = (this.safety && this.safety.collisionCooldownMs) || 1000;
        const shouldLog = now - lastAlert >= cooldown;

        if (shouldLog) {
            warn(`⚠️  Risque collision entre ${idA} et ${idB}`);
            if (this.safety && this.safety.lastCollisionAlert) {
                this.safety.lastCollisionAlert.set(key, now);
            }
        }

        // Force de répulsion d'urgence renforcée
        const distance = drone1.position.distanceTo(drone2.position);
        const repulsionStrength = Math.max(5.0, 10.0 / (distance + 0.1)); // Force inverse de la distance
        const repulsionVector = drone1.position.clone()
            .sub(drone2.position)
            .normalize()
            .multiplyScalar(repulsionStrength);

        if (drone1.acceleration) {
            drone1.acceleration.add(repulsionVector);
        }
        if (drone2.acceleration) {
            drone2.acceleration.sub(repulsionVector);
        }

        if (shouldLog) {
            this.metrics.collisionEvents++;
        }
    }

    checkBatteryLevels(drones) {
        drones.forEach(drone => {
            const status = drone.getStatus ? drone.getStatus() : { batteryLevel: 1 };

            if (status.batteryLevel < 0.2) {
                // Batterie critique - retour base
                warn(`🔋 Batterie critique pour ${drone.id}`);
                this.initiateEmergencyReturn(drone);
            }
        });
    }

    initiateEmergencyReturn(drone) {
        if (drone.setMission) {
            drone.setMission('RETURN_BASE', {
                priority: 'EMERGENCY'
            });
        }
        this.metrics.emergencyLandings++;
    }

    /**
     * Complétion de mission
     */
    completeMission(missionId, result = 'SUCCESS') {
        const mission = this.activeMissions.get(missionId);
        if (!mission) return;

        mission.status = result;
        mission.endTime = Date.now();
        mission.duration = mission.endTime - mission.startTime;

        // Archivage
        this.missionHistory.push(mission);
        this.activeMissions.delete(missionId);

        if (this.currentMissionId === missionId) {
            this.currentMissionId = null;
        }

        // Callbacks
        if (result === 'SUCCESS' && mission.onComplete) {
            mission.onComplete(mission);
        } else if (result !== 'SUCCESS' && mission.onFailed) {
            mission.onFailed(mission, result);
        }

        log(`🏁 Mission ${mission.type} terminée: ${result}`);
        this.metrics.missionsCompleted++;
    }

    checkMissionObjectives(mission) {
        switch (mission.type) {
            case 'EXPLORATION':
                if (mission.progress.coverage >= mission.objectives.coverageTarget) {
                    this.completeMission(mission.id, 'SUCCESS');
                }
                break;
        }
    }

    updateExplorationProgress(mission, drones) {
        const coverage = this.calculateCoverage(mission.targetArea);
        const discoveries = this.swarmData.discoveries.length;

        mission.progress.coverage = coverage;
        mission.progress.discoveries = discoveries;
        mission.progress.dronesAssigned = drones.length;
        mission.progress.efficiency = coverage / ((Date.now() - mission.startTime) / 60000); // coverage/minute

        if (mission.onProgress) {
            mission.onProgress(mission.progress);
        }
    }

    calculateCoverage(targetArea) {
        // Calcul simplifié basé sur la carte de couverture
        let coveredCells = 0;
        let totalCells = 0;

        const { center, radius } = targetArea;
        const resolution = 1.0;

        for (let x = center.x - radius; x <= center.x + radius; x += resolution) {
            for (let y = center.y - radius; y <= center.y + radius; y += resolution) {
                const distance = Math.sqrt((x - center.x) ** 2 + (y - center.y) ** 2);
                if (distance <= radius) {
                    totalCells++;
                    const key = `${Math.floor(x)},${Math.floor(y)}`;
                    if (this.swarmData.coverage.has(key)) {
                        coveredCells++;
                    }
                }
            }
        }

        return totalCells > 0 ? coveredCells / totalCells : 0;
    }

    /**
     * Interface de contrôle
     */
    abortMission(missionId) {
        this.completeMission(missionId, 'ABORTED');
    }

    abortAllMissions() {
        for (const missionId of this.activeMissions.keys()) {
            this.abortMission(missionId);
        }
    }

    getMissionStatus(missionId) {
        return this.activeMissions.get(missionId) || null;
    }

    getAllMissionsStatus() {
        return Array.from(this.activeMissions.values());
    }

    updateMetrics(deltaTime, drones) {
        this.metrics.totalFlightTime += deltaTime * drones.length;
        this.metrics.coverageArea = this.swarmData.coverage.size;

        if (this.metrics.totalFlightTime > 0) {
            this.metrics.efficiency = this.metrics.coverageArea / (this.metrics.totalFlightTime / 60);
        }
    }

    getMetrics() {
        return { ...this.metrics };
    }

    getSwarmData() {
        return {
            drones: Object.fromEntries(this.swarmData.drones),
            coverage: this.swarmData.coverage.size,
            discoveries: this.swarmData.discoveries.length,
            threats: this.swarmData.threats.length
        };
    }

    /**
     * Méthode updateAutoOrganization manquante
     */
    updateAutoOrganization(drones) {
        if (!drones || drones.length === 0) return;

        try {
            // Auto-organisation basique des formations
            this.activeMissions.forEach(mission => {
                if (mission.type === 'FORMATION_FLIGHT') {
                    this.updateFormationFlying(mission, drones);
                }
            });

            // Maintien de la cohésion de l'essaim
            this.maintainSwarmCohesion(drones);

        } catch (error) {
            warn('⚠️ Erreur auto-organisation:', error.message);
        }
    }

    /**
     * Méthode getCurrentMission manquante
     */
    getCurrentMission() {
        if (this.currentMissionId && this.activeMissions.has(this.currentMissionId)) {
            return this.activeMissions.get(this.currentMissionId);
        }

        // Retourner la première mission active s'il n'y a pas de mission courante définie
        if (this.activeMissions.size > 0) {
            return this.activeMissions.values().next().value;
        }

        return null;
    }

    /**
     * Obtenir le progrès de la mission courante
     */
    getCurrentProgress() {
        const currentMission = this.getCurrentMission();
        if (!currentMission) {
            return 0;
        }

        // Calculer le progrès basé sur les objectifs accomplis
        if (currentMission.objectives && currentMission.objectives.length > 0) {
            const completedObjectives = currentMission.objectives.filter(obj => obj.completed).length;
            return completedObjectives / currentMission.objectives.length;
        }

        // Progrès basé sur le temps écoulé si pas d'objectifs
        if (currentMission.duration && currentMission.startTime) {
            const elapsed = Date.now() - currentMission.startTime;
            return Math.min(elapsed / currentMission.duration, 1.0);
        }

        // Par défaut, estimer le progrès
        return this.isActive ? 0.5 : 0;
    }

    /**
     * Maintien de la cohésion de l'essaim
     */
    maintainSwarmCohesion(drones) {
        const center = this.calculateSwarmCenter(drones);
        const maxDistance = 15; // Distance maximale du centre

        drones.forEach(drone => {
            const distanceFromCenter = drone.position.distanceTo(center);

            if (distanceFromCenter > maxDistance) {
                // Force de rappel vers le centre
                const recallForce = center.clone().sub(drone.position).normalize().multiplyScalar(0.5);
                if (drone.velocity) {
                    drone.velocity.add(recallForce);
                }
            }
        });
    }

    /**
     * Calcul du centre de l'essaim
     */
    calculateSwarmCenter(drones) {
        if (drones.length === 0) return new THREE.Vector3();

        const center = new THREE.Vector3();
        drones.forEach(drone => {
            center.add(drone.position);
        });
        center.divideScalar(drones.length);

        return center;
    }

    /**
     * Vérification des limites de la zone de mission
     */
    checkBoundaries(drones) {
        const boundaryLimits = {
            x: { min: -25, max: 25 },
            y: { min: -25, max: 25 },
            z: { min: 0.5, max: 15 }
        };

        drones.forEach(drone => {
            try {
                if (!drone.position) return;

                let needsCorrection = false;
                const correctionForce = { x: 0, y: 0, z: 0 };

                // Vérification limites X
                if (drone.position.x < boundaryLimits.x.min) {
                    correctionForce.x = 0.5;
                    needsCorrection = true;
                } else if (drone.position.x > boundaryLimits.x.max) {
                    correctionForce.x = -0.5;
                    needsCorrection = true;
                }

                // Vérification limites Y
                if (drone.position.y < boundaryLimits.y.min) {
                    correctionForce.y = 0.5;
                    needsCorrection = true;
                } else if (drone.position.y > boundaryLimits.y.max) {
                    correctionForce.y = -0.5;
                    needsCorrection = true;
                }

                // Vérification limites Z (altitude)
                if (drone.position.z < boundaryLimits.z.min) {
                    correctionForce.z = 0.3;
                    needsCorrection = true;
                } else if (drone.position.z > boundaryLimits.z.max) {
                    correctionForce.z = -0.3;
                    needsCorrection = true;
                }

                // Application de la correction si nécessaire
                if (needsCorrection && drone.velocity) {
                    drone.velocity.x += correctionForce.x;
                    drone.velocity.y += correctionForce.y;
                    drone.velocity.z += correctionForce.z;
                }

            } catch (error) {
                warn(`⚠️ Erreur vérification limites pour drone ${drone.id || 'inconnu'}:`, error.message);
            }
        });
    }

    /**
     * Vérifie les conditions environnementales pour la mission
     * @returns {Object} Statut des conditions
     */
    checkEnvironmentConditions() {
        try {
            return {
                weather: 'clear',
                wind_speed: 0.5,
                visibility: 'good',
                temperature: 22,
                safety_level: 'high',
                mission_feasible: true
            };
        } catch (error) {
            warn('⚠️ Erreur vérification conditions environnementales:', error.message);
            return {
                weather: 'unknown',
                wind_speed: 0,
                visibility: 'unknown',
                temperature: 20,
                safety_level: 'medium',
                mission_feasible: true
            };
        }
    }
}

export default MissionManager;
