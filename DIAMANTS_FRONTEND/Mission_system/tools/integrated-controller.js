/**
 * DIAMANTS - Contrôleur Principal Intégré
 * ==========================================
 * Système unifié intégrant toutes les fonctionnalités migrées
 */

// Sécurité: ES6 modules n'ont pas accès aux variables globales automatiquement
// Récupérer les fonctions de logging depuis window
const log = window.log || ((...args) => console.log(...args));
const warn = window.warn || ((...args) => console.warn(...args));
const error = window.error || ((...args) => console.error(...args));

// Mode silencieux global - mettre à false pour voir les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Note: tools/ is a sibling of core/, drones/, missions/, etc. Use ../ imports.
import { logger } from '../core/logger.js';
import { DiamantFormulas } from '../core/diamants-formulas.js';
import { AuthenticCrazyflie } from '../drones/authentic-crazyflie.js';
import { MissionManager } from '../missions/mission-manager.js';
import { AuthenticProvencalEnvironment } from '../environment/authentic-provencal-environment.js';
import { DiamantUI } from '../ui/diamant-ui.js';

// Nouveaux modules migrés
import { FlightBehaviors } from '../behaviors/flight-behaviors.js';
import { CollaborativeScouting } from '../behaviors/collaborative-scouting.js';
import { AdvancedCollectiveIntelligence } from '../intelligence/advanced-collective-intelligence.js';
import { CrazyflieRosController } from '../controllers/crazyflie-ros-controller.js';
import { CrazyflieVisual } from '../drones/crazyflie-visual.js';
import { CrazyflieVisualEnhancements } from '../visual/crazyflie-visual-enhancements.js';

// Modules supplémentaires "morts"
import { DronePhysics } from '../physics/drone-physics.js';
import { PIDController } from '../physics/pid-controller.js';
import { RealisticFlightDynamics } from '../physics/realistic-flight-dynamics.js';
import { GLSLGrassField } from '../environment/glsl-grass-field.js';
// SAMPLE_MODE import removed - sample files moved to DEMO/ directory

export class IntegratedDiamantsController {
    constructor(scene, config = {}) {
        logger.info('Controller', '🎯 IntegratedDiamantsController.constructor() - Début initialisation');
        this.scene = scene;
        this.config = {
            droneCount: config.droneCount || 8,
            enableRealisticFlight: config.enableRealisticFlight !== false,
            enableCollaborativeScouting: config.enableCollaborativeScouting !== false,
            enableWahooEffect: config.enableWahooEffect !== false,
            enableAdvancedIntelligence: config.enableAdvancedIntelligence !== false,
            missionType: config.missionType || 'COLLABORATIVE_EXPLORATION',
            ...config
        };
        
        // Systèmes principaux
        this.diamantFormulas = new DiamantFormulas(this.config);
        this.missionManager = new MissionManager(this.config);
        this.environment = null;
        this.ui = null;

        // Nouveaux systèmes migrés
        this.flightBehaviors = null;
        this.collaborativeScouting = null;
        this.advancedIntelligence = null;
        this.rosController = null;
        this.visualEnhancements = null;
        this.dronePhysics = null;
        this.realisticFlightDynamics = null; // NOUVEAU: Système dynamique vol réaliste
        this.advancedIntelligence = null;
        
        // Contrôleurs avancés
        this.rosController = null;
        this.visualEnhancements = null;
        
        // Modules supplémentaires
        this.dronePhysics = null;
        this.pidController = null;
        this.grassFieldBasic = null;
        this.sampleMode = SAMPLE_MODE;

        // Collections
        this.drones = [];
        this.activeMissions = new Map();

        // État système
        this.isInitialized = false;
        this.isRunning = false;
        this.startTime = 0;

        // Métriques globales
        this.metrics = {
            totalFlightTime: 0,
            coveragePercentage: 0,
            collaborationEfficiency: 0,
            wahooEffectLevel: 0,
            emergenceLevel: 0,
            missionSuccess: 0
        };

        this.initializeSystem();
    }

    /**
     * Initialisation complète du système
     */
    async initializeSystem() {
        logger.info('Controller', '🚀 Initialisation système DIAMANTS intégré...');

        try {
            // 1. Créer environnement
            await this.initializeEnvironment();

            // 2. Initialiser systèmes avancés
            if (this.config.enableAdvancedIntelligence) {
                this.advancedIntelligence = new AdvancedCollectiveIntelligence(this.config);
                logger.info('Controller', '🧠 Intelligence collective avancée initialisée');
            }

            // 3. Initialiser comportements de vol
            if (this.config.enableRealisticFlight) {
                this.flightBehaviors = new FlightBehaviors(this.config);
                log('✅ FlightBehaviors initialisé:', this.flightBehaviors);
                logger.info('Controller', '✈️ Comportements de vol réalistes initialisés');
            } else {
                log('⚠️ enableRealisticFlight désactivé, pas de FlightBehaviors');
            }

            // 4. Initialiser scouting collaboratif
            if (this.config.enableCollaborativeScouting) {
                this.collaborativeScouting = new CollaborativeScouting(this.config);
                logger.info('Controller', '🔍 Système de scouting collaboratif initialisé');
            }

            // 5. Initialiser contrôleur ROS
            if (this.config.enableRosController) {
                this.rosController = new CrazyflieRosController({
                    isActive: true,
                    wsUrl: this.config.rosWsUrl || 'ws://localhost:9001',
                    maxReconnectDelayMs: this.config.rosMaxReconnectDelayMs || 30000
                });
                logger.info('Controller', '🌐 Contrôleur ROS initialisé');
            }

            // 6. Initialiser améliorations visuelles
            if (this.config.enableVisualEnhancements) {
                this.visualEnhancements = new CrazyflieVisualEnhancements({
                    scene: this.scene
                });
                logger.info('Controller', '🎨 Améliorations visuelles initialisées');
            }

            // 7. Initialiser physique avancée
            if (this.config.enablePhysicsEngine) {
                this.dronePhysics = new DronePhysics({
                    gravity: -9.81,
                    airDensity: 1.225,
                    dragCoefficient: 0.1
                });
                this.pidController = new PIDController({
                    kp: 2.0, ki: 0.5, kd: 0.1
                });
                logger.info('Controller', '⚡ Moteur physique avancé initialisé');
            }

            // 7.5. NOUVEAU: Initialiser dynamique de vol réaliste - TEMPORAIREMENT DÉSACTIVÉ POUR DÉBOGAGE
            if (false) { // DÉSACTIVÉ - causait NaN positions
                this.realisticFlightDynamics = new RealisticFlightDynamics({
                    platformHeight: 8.5,
                    maxVelocity: 8.0,
                    dragCoefficient: 0.25
                });
                logger.info('Controller', '🚁 Dynamique de vol réaliste initialisée');
            } else {
                this.realisticFlightDynamics = null;
                logger.warning('Controller', '⚠️ Dynamique de vol réaliste DÉSACTIVÉE pour débogage');
            }

            // 8. Initialiser herbe basique en fallback
            if (this.config.enableGrassFieldBasic) {
                this.grassFieldBasic = new GLSLGrassField();
                logger.info('Controller', '🌱 Champ d\'herbe basique initialisé');
            }

            // 7. Créer drones
            await this.createDroneFleet();

            // 9. Initialiser interface avec tous les modules
            this.ui = new DiamantUI({
                scene: this.scene,
                drones: this.drones,
                missionManager: this.missionManager,
                showAdvancedMetrics: true,
                
                // Passer tous les modules pour contrôle via UI
                controller: this,
                modules: {
                    flightBehaviors: this.flightBehaviors,
                    collaborativeScouting: this.collaborativeScouting,
                    advancedIntelligence: this.advancedIntelligence,
                    rosController: this.rosController,
                    visualEnhancements: this.visualEnhancements,
                    dronePhysics: this.dronePhysics,
                    pidController: this.pidController,
                    grassFieldBasic: this.grassFieldBasic
                },
                
                // Configuration des toggles
                moduleToggles: {
                    enablePhysicsEngine: this.config.enablePhysicsEngine,
                    enableRosController: this.config.enableRosController,
                    enableVisualEnhancements: this.config.enableVisualEnhancements,
                    enableGrassFieldBasic: this.config.enableGrassFieldBasic
                }
            });

            // 9. Démarrer le gestionnaire de missions puis lancer la mission initiale
            try {
                this.missionManager.start && this.missionManager.start();
            } catch (_) { /* noop */ }

            // 10. ✅ MISSION AUTOMATIQUE ACTIVÉE - Démarrer scouting collaboratif
            await this.startInitialMission();
            log('🚀 Mission de scouting collaboratif démarrée automatiquement');

            this.isInitialized = true;
            this.isRunning = true;
            this.startTime = Date.now();

            logger.info('Controller', '✅ Système DIAMANTS complètement initialisé');
            // Notify listeners that the engine/controller is ready
            try { if (typeof window !== 'undefined') window.dispatchEvent(new CustomEvent('diamants:engine-ready', { detail: { drones: this.drones.length } })); } catch (_) {}
            this.printSystemStatus();

        } catch (error) {
            console.error('❌ Erreur lors de la mise à jour:', error);
            throw error;
        }
    }

    /**
     * Initialisation environnement
     */
    async initializeEnvironment() {
        this.environment = new AuthenticProvencalEnvironment(this.scene, {
            lightweight: true,
            terrainSize: { x: 70, y: 70 },
            structuredLayout: true,
            visibilityBubbles: true,
            terrainAmplitude: 1.2,
            terrainDetail: 1.25,
            maxTrees: 160,
            forestDensity: 0.75,
            minTreeSpacing: 2.4
        });

        logger.info('Controller', '🌲 Environnement provençal créé');
    }

    /**
     * Création flotte de drones
     */
    async createDroneFleet() {
        logger.info('Controller', `🚁 Création flotte de ${this.config.droneCount} drones...`);

        for (let i = 0; i < this.config.droneCount; i++) {
            const droneId = `crazyflie_${i}`;

            // Position initiale en cercle sur la plateforme
            const angle = (i / this.config.droneCount) * 2 * Math.PI;
            const radius = 8.0; // Même rayon que dans main.js
            const platformHeight = 8.5; // Hauteur de la plateforme + offset
            const startPosition = {
                x: Math.cos(angle) * radius,
                y: platformHeight + 0.5, // 0.5m au-dessus de la plateforme pour éviter collisions
                z: Math.sin(angle) * radius
            };

            // Créer drone Crazyflie authentique avec le bon ordre des paramètres
            const drone = new AuthenticCrazyflie({
                id: droneId,
                position: startPosition,
                type: 'SCOUT',
                scene: this.scene,
                enableAdvancedBehaviors: this.config.enableRealisticFlight,
                collaborativeMode: this.config.enableCollaborativeScouting
            });

            // NOUVEAU: Initialiser physique réaliste pour chaque drone - DÉSACTIVÉ pour débogage
            if (this.realisticFlightDynamics) {
                this.realisticFlightDynamics.initializeDrone(droneId, new THREE.Vector3(
                    startPosition.x,
                    startPosition.y,
                    startPosition.z
                ));
            }

            // Initialiser physique de vol si activée (ancienne méthode)
            if (this.flightBehaviors) {
                this.flightBehaviors.initializeDronePhysics(droneId, startPosition);
            }

            this.drones.push(drone);
            logger.debug('Controller', `✅ Drone ${droneId} créé à position (${startPosition.x.toFixed(1)}, ${startPosition.z.toFixed(1)})`);

            // Vérifier que le drone a bien un mesh et qu'il est dans la scène
            if (drone.mesh) {
                logger.debug('Controller', `🎯 Drone ${droneId} mesh créé, position:`, drone.mesh.position);
                if (this.scene.children.includes(drone.mesh)) {
                    logger.debug('Controller', `✅ Drone ${droneId} ajouté à la scène`);
                } else {
                    logger.warning('Controller', `⚠️ Drone ${droneId} mesh non trouvé dans la scène`);
                }
            } else {
                logger.error('Controller', `❌ Drone ${droneId} n'a pas de mesh!`);
            }
        }

        // Connecter les drones aux systèmes avancés
        this.connectDronesToSystems();

        logger.info('Controller', `🎯 TOTAL: ${this.drones.length} drones créés et configurés`);
        
        // Vérification des positions des drones du contrôleur intégré
        log('🔍 Contrôleur intégré - Positions des drones:');
        this.drones.forEach((drone, index) => {
            log(`  Drone intégré ${index + 1}: x=${drone.position.x.toFixed(2)}, y=${drone.position.y.toFixed(2)}, z=${drone.position.z.toFixed(2)}`);
        });
    }

    /**
     * Connexion drones aux systèmes avancés
     */
    connectDronesToSystems() {
        const droneIds = this.drones.map(d => d.id);

        // Connecter au système d'intelligence avancée
        if (this.advancedIntelligence) {
            // Les drones peuvent maintenant accéder aux attracteurs Wahoo
            this.drones.forEach(drone => {
                drone.advancedIntelligence = this.advancedIntelligence;
            });
        }

        // Le MissionManager utilise les drones via la méthode update()
        // Pas besoin d'enregistrement explicite - il reçoit les drones en paramètre

        log(`🔗 ${this.drones.length} drones connectés aux systèmes avancés`);
    }

    /**
     * Démarrage mission initiale
     */
    async startInitialMission() {
        log('🎯 Démarrage mission collaborative initiale...');

        if (this.config.enableCollaborativeScouting) {
            // Mission de scouting collaboratif
            const droneIds = this.drones.map(d => d.id);

            await this.collaborativeScouting.startScoutingMission(droneIds, {
                explorationRadius: 25.0,
                coverageTarget: 0.85,
                enableWahooEffect: this.config.enableWahooEffect
            });

            log('🔍 Mission de scouting collaboratif démarrée');
        } else {
            // Mission classique de formation
            const missionParams = {
                type: 'FORMATION_FLIGHT',
                duration: 300, // 5 minutes
                formation: 'DIAMOND'
            };

            const missionId = await this.missionManager.startMission(missionParams);
            this.activeMissions.set(missionId, missionParams);

            log(`🎯 Mission classique ${missionId} démarrée`);
        }
    }

    /**
     * Boucle de mise à jour principale
     */
    update(deltaTime) {
        if (!this.isRunning) return;

        try {
            // 1. NOUVEAU: Mettre à jour dynamique de vol réaliste en priorité
            if (this.realisticFlightDynamics) {
                this.realisticFlightDynamics.update(deltaTime);
            }

            // 2. Mise à jour systèmes de base
            this.diamantFormulas.update && this.diamantFormulas.update(deltaTime);
            this.missionManager.update && this.missionManager.update(deltaTime, this.drones, this.environment);

            // 3. Mise à jour systèmes avancés
            if (this.advancedIntelligence) {
                // Pass agents and environment as required by AdvancedCollectiveIntelligence API
                this.advancedIntelligence.update(deltaTime, this.drones, this.environment);
            }

            if (this.flightBehaviors) {
                // Vérifier si au moins un drone n'est pas IDLE avant de mettre à jour FlightBehaviors
                const activeDroneIds = this.drones.filter(drone => drone.state !== 'IDLE').map(drone => drone.id);
                
                // DEBUG: Log des états des drones
                if (this._debugFrame === undefined) this._debugFrame = 0;
                this._debugFrame++;
                if (this._debugFrame % 120 === 0) { // Log toutes les 2 secondes environ
                    const droneStates = this.drones.map(drone => `${drone.id}:${drone.state}`).join(', ');
                    log(`🔍 États drones: [${droneStates}] → ${activeDroneIds.length} actifs: [${activeDroneIds.join(', ')}]`);
                }
                
                if (activeDroneIds.length > 0) {
                    this.flightBehaviors.update(deltaTime, activeDroneIds);
                }
            }

            if (this.collaborativeScouting) {
                this.collaborativeScouting.update(deltaTime);
            }

            // 4. Mise à jour drones avec dynamique réaliste
            this.updateDronesBehaviors(deltaTime);

            // 5. Mise à jour interface
            if (this.ui && this.ui.update) {
                this.ui.update(deltaTime);
            }

            // 6. Mise à jour métriques
            this.updateMetrics();

        } catch (error) {
            console.error('❌ Erreur lors de la mise à jour:', error);
        }
    }

    /**
     * Mise à jour comportements des drones avec physique réaliste
     */
    updateDronesBehaviors(deltaTime) {
        // Mettre à jour physique réaliste - DÉSACTIVÉ pour débogage
        if (this.realisticFlightDynamics) {
            this.realisticFlightDynamics.update(deltaTime);
        }
        
        this.drones.forEach(drone => {
            // 1. Obtenir état physique réaliste en priorité - DÉSACTIVÉ pour débogage
            let realisticState = null;
            if (this.realisticFlightDynamics) {
                realisticState = this.realisticFlightDynamics.getDroneState(drone.id);
                
                // Diagnostic pour résoudre problème NaN
                if (!realisticState && Math.random() < 0.005) { // Log rare
                    console.warn(`⚠️ DIAGNOSTIC: Drone ${drone.id} pas dans système réaliste`);
                    console.log('🔧 Drones dans système réaliste:', Array.from(this.realisticFlightDynamics.drones.keys()));
                }
            }
            
            // 2. Récupérer télémétrie ancienne méthode (fallback)
            let telemetry = null;
            if (this.flightBehaviors) {
                telemetry = this.flightBehaviors.getTelemetry(drone.id);
            }

            // 3. Utiliser physique réaliste si disponible, sinon fallback télémétrie
            const activeState = realisticState || telemetry;
            
            if (activeState) {
                // Appliquer forces Wahoo si intelligence avancée active ET drone non IDLE
                if (this.advancedIntelligence && activeState && drone.state !== 'IDLE') {
                    const wahooForce = this.advancedIntelligence.getWahooForceAt(activeState.position);

                    // Appliquer la force au système physique réaliste
                    if (realisticState) {
                        realisticState.velocity.x += wahooForce.x * deltaTime * 0.1;
                        realisticState.velocity.y += wahooForce.y * deltaTime * 0.1;
                        realisticState.velocity.z += wahooForce.z * deltaTime * 0.1;
                    }
                    // Fallback: appliquer aux anciennes physics
                    else if (this.flightBehaviors) {
                        const physics = this.flightBehaviors.dronePhysics.get(drone.id);
                        if (physics) {
                            physics.velocity.x += wahooForce.x * deltaTime * 0.1;
                            physics.velocity.y += wahooForce.y * deltaTime * 0.1;
                            physics.velocity.z += wahooForce.z * deltaTime * 0.1;
                        }
                    }
                }

                // 4. Mise à jour position visuelle du drone
                if (drone.mesh) {
                    // PRIORITÉ: Position physique réaliste
                    if (realisticState) {
                        drone.mesh.position.set(
                            realisticState.position.x,
                            realisticState.position.y,
                            realisticState.position.z
                        );
                        
                        // Synchroniser état interne drone
                        try {
                            if (drone.position) {
                                drone.position.set(
                                    realisticState.position.x,
                                    realisticState.position.y,
                                    realisticState.position.z
                                );
                            }
                            if (drone.velocity) {
                                drone.velocity.set(
                                    realisticState.velocity.x,
                                    realisticState.velocity.y,
                                    realisticState.velocity.z
                                );
                            }
                        } catch (_) { /* safe */ }
                    }
                    // FALLBACK: Ancienne télémétrie
                    else if (telemetry) {
                        drone.mesh.position.set(
                            telemetry.position.x,
                            telemetry.position.y,
                            telemetry.position.z
                        );
                        
                        try {
                            if (drone.position) {
                                drone.position.set(
                                    telemetry.position.x,
                                    telemetry.position.y,
                                    telemetry.position.z
                                );
                            }
                            if (drone.velocity) {
                                drone.velocity.set(
                                    telemetry.velocity.x,
                                    telemetry.velocity.y,
                                    telemetry.velocity.z
                                );
                            }
                        } catch (_) { /* safe */ }
                    }

                    // Animation hélices basée sur vitesse
                    const velocityState = realisticState || telemetry;
                    const speed = Math.sqrt(
                        velocityState.velocity.x ** 2 +
                        velocityState.velocity.y ** 2 +
                        velocityState.velocity.z ** 2
                    );
                    
                    if (drone.animateRotors) {
                        drone.animateRotors(speed * 100); // Vitesse des hélices proportionnelle
                    }
                }
            }

            // 5. Mise à jour état drone avec les autres drones pour la collision
            if (drone.update) {
                drone.update(deltaTime, this.drones); // Passer la liste des drones pour la collision
            }
        });
    }

    /**
     * Mise à jour métriques globales
     */
    updateMetrics() {
        // Métriques de vol
        if (this.flightBehaviors) {
            const flightMetrics = this.flightBehaviors.getPerformanceMetrics();
            this.metrics.totalFlightTime = flightMetrics.totalFlightTime;
        }

        // Métriques de scouting
        if (this.collaborativeScouting) {
            const scoutingStatus = this.collaborativeScouting.getMissionStatus();
            this.metrics.coveragePercentage = scoutingStatus.progress * 100;
            this.metrics.collaborationEfficiency = scoutingStatus.collaborationMetrics.coordinationIndex;
        }

        // Métriques d'intelligence avancée
        if (this.advancedIntelligence) {
            const advancedState = this.advancedIntelligence.getAdvancedState();
            this.metrics.wahooEffectLevel = advancedState.metrics.wahooEffectiveness;
            this.metrics.emergenceLevel = advancedState.metrics.emergenceLevel;
        }

        // Métriques de mission
        this.metrics.missionSuccess = this.calculateMissionSuccess();
    }

    /**
     * Calcul succès mission
     */
    calculateMissionSuccess() {
        let successScore = 0;

        // Score basé sur couverture
        if (this.metrics.coveragePercentage > 80) {
            successScore += 0.4;
        } else if (this.metrics.coveragePercentage > 60) {
            successScore += 0.2;
        }

        // Score basé sur collaboration
        if (this.metrics.collaborationEfficiency > 0.7) {
            successScore += 0.3;
        } else if (this.metrics.collaborationEfficiency > 0.5) {
            successScore += 0.15;
        }

        // Score basé sur émergence
        if (this.metrics.emergenceLevel > 0.6) {
            successScore += 0.3;
        } else if (this.metrics.emergenceLevel > 0.4) {
            successScore += 0.15;
        }

        return Math.min(1.0, successScore);
    }

    /**
     * Commandes de contrôle
     */
    
    /**
     * Démarrage MANUEL des missions - appelée depuis le control panel
     */
    async startMissionManual() {
        log('🎯 Démarrage MANUEL de la mission depuis le control panel...');
        
        if (this.missionStarted) {
            log('⚠️ Mission déjà démarrée - ignoré');
            return;
        }
        
        await this.startInitialMission();
        this.missionStarted = true;
        log('✅ Mission démarrée manuellement');
    }

    async takeoffAllDrones() {
        log('🚁 Décollage de tous les drones...');

        const takeoffPromises = this.drones.map(async (drone, index) => {
            const PLATFORM_HEIGHT = 8.5; // Hauteur de la plateforme
            const altitude = PLATFORM_HEIGHT + 1.0 + (index * 0.2); // 1m au-dessus plateforme + échelonnement léger

            if (this.flightBehaviors) {
                return this.flightBehaviors.performTakeoff(drone.id, altitude);
            } else if (drone.takeoff) {
                return drone.takeoff(altitude);
            }
        });

        await Promise.all(takeoffPromises);
        log('✅ Tous les drones ont décollé à altitude opérationnelle au-dessus de la plateforme');
    }

    async landAllDrones() {
        log('🛬 Atterrissage de tous les drones...');

        const landingPromises = this.drones.map(async drone => {
            if (this.flightBehaviors) {
                const physics = this.flightBehaviors.dronePhysics.get(drone.id);
                if (physics) {
                    return this.flightBehaviors.smoothAltitudeChange(drone.id, 0, 3000);
                }
            } else if (drone.land) {
                return drone.land();
            }
        });

        await Promise.all(landingPromises);
        log('✅ Tous les drones ont atterri');
    }

    emergencyStop() {
        log('🚨 ARRÊT D\'URGENCE');

        this.isRunning = false;

        // NOUVEAU: Correction altitude d'urgence avec physique réaliste
        this.drones.forEach(drone => {
            if (this.realisticFlightDynamics) {
                this.realisticFlightDynamics.emergencyAltitudeCorrection(drone.id);
            }
            
            if (this.flightBehaviors) {
                this.flightBehaviors.emergencyLanding(drone.id);
            } else if (drone.emergencyStop) {
                drone.emergencyStop();
            }
        });

        // Arrêt des systèmes
        if (this.collaborativeScouting) {
            this.collaborativeScouting.missionState.phase = 'EMERGENCY_STOPPED';
        }
    }

    /**
     * Affichage statut système
     */
    printSystemStatus() {
        log('\n📊 === STATUT SYSTÈME DIAMANTS ===');
        log(`🚁 Drones actifs: ${this.drones.length}/${this.config.droneCount}`);
        log(`🧠 Intelligence avancée: ${this.advancedIntelligence ? '✅' : '❌'}`);
        log(`✈️ Vol réaliste: ${this.flightBehaviors ? '✅' : '❌'}`);
        log(`� Dynamique réaliste: ${this.realisticFlightDynamics ? '✅' : '❌'}`);
        log(`�🔍 Scouting collaboratif: ${this.collaborativeScouting ? '✅' : '❌'}`);
        log(`🌊 Effet Wahoo: ${this.config.enableWahooEffect ? '✅' : '❌'}`);

        // Statistiques physique réaliste
        if (this.realisticFlightDynamics) {
            const stats = this.realisticFlightDynamics.getSystemStats();
            log(`📐 Altitude moyenne: ${stats.averageAltitude.toFixed(1)}m`);
            log(`🚨 Freins d'urgence: ${stats.emergencyBrakes}`);
            log(`⚠️ Drones en danger: ${stats.unsafeDrones}`);
        }

        if (this.advancedIntelligence) {
            const state = this.advancedIntelligence.getAdvancedState();
            log(`🎯 Phase intelligence: ${state.phase}`);
            log(`👑 Leaders émergents: ${state.wahooSystem.emergentLeaders.length}`);
            log(`🌊 Attracteurs actifs: ${state.wahooSystem.attractors.length}`);
        }

        if (this.collaborativeScouting) {
            const status = this.collaborativeScouting.getMissionStatus();
            log(`📈 Couverture: ${status.coverage}`);
            log(`🤝 Efficacité collaboration: ${(status.collaborationMetrics.coordinationIndex * 100).toFixed(1)}%`);
        }

        log('=====================================\n');
    }

    /**
     * API publique - Obtenir état complet
     */
    getSystemState() {
        return {
            isInitialized: this.isInitialized,
            isRunning: this.isRunning,
            uptime: Date.now() - this.startTime,
            droneCount: this.drones.length,
            metrics: { ...this.metrics },

            // États des sous-systèmes
            flightBehaviors: this.flightBehaviors ? this.flightBehaviors.getPerformanceMetrics() : null,
            collaborativeScouting: this.collaborativeScouting ? this.collaborativeScouting.getMissionStatus() : null,
            advancedIntelligence: this.advancedIntelligence ? this.advancedIntelligence.getAdvancedState() : null,

            // Configuration active
            config: { ...this.config }
        };
    }

    /**
     * API publique - Statistiques détaillées
     */
    getDetailedStats() {
        const state = this.getSystemState();

        return {
            system: {
                uptime: state.uptime,
                phase: state.advancedIntelligence?.phase || 'BASIC',
                performance: state.metrics.missionSuccess
            },

            fleet: {
                totalDrones: state.droneCount,
                activeDrones: state.flightBehaviors?.activeDrones || state.droneCount,
                averageBattery: state.flightBehaviors?.averageBattery || 100,
                totalFlightTime: state.metrics.totalFlightTime
            },

            mission: {
                type: this.config.missionType,
                coverage: state.metrics.coveragePercentage,
                efficiency: state.metrics.collaborationEfficiency,
                success: state.metrics.missionSuccess
            },

            intelligence: {
                wahooLevel: state.metrics.wahooEffectLevel,
                emergence: state.metrics.emergenceLevel,
                leaders: state.advancedIntelligence?.wahooSystem.emergentLeaders.length || 0,
                attractors: state.advancedIntelligence?.wahooSystem.attractors.length || 0
            }
        };
    }

    /**
     * API publique - Activer/Désactiver un module
     */
    toggleModule(moduleName, enabled) {
        logger.info('Controller', `🔄 Basculement module ${moduleName}: ${enabled ? 'ON' : 'OFF'}`);
        
        try {
            // Modules existants - mise à jour de la config et réactivation si nécessaire
            switch (moduleName) {
                case 'advancedIntelligence':
                    this.config.enableAdvancedIntelligence = enabled;
                    if (enabled && !this.advancedIntelligence) {
                        this.advancedIntelligence = new AdvancedCollectiveIntelligence(this.config);
                    } else if (!enabled && this.advancedIntelligence) {
                        if (this.advancedIntelligence.stop) this.advancedIntelligence.stop();
                        this.advancedIntelligence = null;
                    }
                    break;

                case 'collaborativeScouting':
                    this.config.enableCollaborativeScouting = enabled;
                    if (enabled && !this.collaborativeScouting) {
                        this.collaborativeScouting = new CollaborativeScouting(this.config);
                    } else if (!enabled && this.collaborativeScouting) {
                        if (this.collaborativeScouting.stop) this.collaborativeScouting.stop();
                        this.collaborativeScouting = null;
                    }
                    break;

                case 'flightBehaviors':
                case 'realisticFlight':
                    this.config.enableRealisticFlight = enabled;
                    if (enabled && !this.flightBehaviors) {
                        this.flightBehaviors = new FlightBehaviors(this.config);
                    } else if (!enabled && this.flightBehaviors) {
                        if (this.flightBehaviors.stop) this.flightBehaviors.stop();
                        this.flightBehaviors = null;
                    }
                    break;

                case 'dronePhysics':
                    if (enabled && !this.dronePhysics) {
                        this.dronePhysics = new DronePhysics(this.scene, this.config);
                        if (this.dronePhysics.isActive !== undefined) this.dronePhysics.isActive = true;
                    } else if (!enabled && this.dronePhysics) {
                        if (this.dronePhysics.isActive !== undefined) this.dronePhysics.isActive = false;
                        if (this.dronePhysics.stop) this.dronePhysics.stop();
                    }
                    break;

                case 'pidController':
                    if (enabled && !this.pidController) {
                        this.pidController = new PIDController(this.config);
                        if (this.pidController.isActive !== undefined) this.pidController.isActive = true;
                    } else if (!enabled && this.pidController) {
                        if (this.pidController.isActive !== undefined) this.pidController.isActive = false;
                        if (this.pidController.stop) this.pidController.stop();
                    }
                    break;

                case 'grassFieldBasic':
                case 'glslGrassField':
                    if (enabled && !this.grassFieldBasic) {
                        this.grassFieldBasic = new GLSLGrassField(this.scene, this.config);
                        if (this.grassFieldBasic.isActive !== undefined) this.grassFieldBasic.isActive = true;
                    } else if (!enabled && this.grassFieldBasic) {
                        if (this.grassFieldBasic.isActive !== undefined) this.grassFieldBasic.isActive = false;
                        if (this.grassFieldBasic.stop) this.grassFieldBasic.stop();
                    }
                    break;

                case 'rosController':
                case 'rosIntegrator':
                    if (enabled && !this.rosController) {
                        this.rosController = new CrazyflieRosController({
                            isActive: true,
                            wsUrl: this.config.rosWsUrl || 'ws://localhost:9001',
                            maxReconnectDelayMs: this.config.rosMaxReconnectDelayMs || 30000
                        });
                        if (this.rosController.isActive !== undefined) this.rosController.isActive = true;
                    } else if (!enabled && this.rosController) {
                        if (this.rosController.isActive !== undefined) this.rosController.isActive = false;
                        if (this.rosController.disconnect) this.rosController.disconnect();
                        this.rosController = null;
                    }
                    break;

                case 'visualEnhancements':
                    if (enabled && !this.visualEnhancements) {
                        this.visualEnhancements = new CrazyflieVisualEnhancements(this.scene, this.config);
                        if (this.visualEnhancements.isActive !== undefined) this.visualEnhancements.isActive = true;
                    } else if (!enabled && this.visualEnhancements) {
                        if (this.visualEnhancements.isActive !== undefined) this.visualEnhancements.isActive = false;
                        if (this.visualEnhancements.dispose) this.visualEnhancements.dispose();
                        this.visualEnhancements = null;
                    }
                    break;

                // Modules qui n'existent pas encore mais sont répertoriés dans l'UI
                case 'swarmIntelligence':
                case 'adaptivePerformanceEngine':
                case 'networkOptimizer':
                case 'crossPlatformOptimizer':
                case 'rlNeuralNetwork':
                case 'quantumNeuralNetwork':
                case 'flockingBehaviors':
                case 'weatherEngine':
                case 'terrainSystem':
                case 'lightingSystem':
                case 'edgeAIProcessor':
                case 'realtimeMesaIntegrator':
                case 'cloudFormation':
                case 'airTrafficSystem':
                    logger.warn('Controller', `⚠️ Module ${moduleName} pas encore implémenté, configuration sauvée`);
                    // Sauvegarder dans config pour plus tard
                    this.config[`enable${moduleName.charAt(0).toUpperCase()}${moduleName.slice(1)}`] = enabled;
                    break;

                default:
                    logger.warn('Controller', `❌ Module inconnu: ${moduleName}`);
                    return false;
            }

            logger.success('Controller', `✅ Module ${moduleName} ${enabled ? 'activé' : 'désactivé'}`);
            return true;

        } catch (error) {
            logger.error('Controller', `❌ Erreur basculement ${moduleName}:`, error);
            return false;
        }
    }

    /**
     * API publique - Obtenir métriques de performance 
     */
    getPerformanceMetrics() {
        return {
            cpu: this.getCPUUsage(),
            memory: this.getMemoryUsage(),
            fps: this.getFPS(),
            activeModules: this.getActiveModulesCount(),
            uptime: Date.now() - this.startTime
        };
    }

    getCPUUsage() {
        // Simulation d'usage CPU basé sur le nombre de modules actifs
        const activeModules = this.getActiveModulesCount();
        return Math.min(95, 20 + (activeModules * 8) + Math.random() * 10);
    }

    getMemoryUsage() {
        // Simulation d'usage mémoire
        const baseMemory = 150;
        const moduleMemory = this.getActiveModulesCount() * 25;
        const droneMemory = this.drones.length * 5;
        return baseMemory + moduleMemory + droneMemory + Math.random() * 50;
    }

    getFPS() {
        // Simulation du FPS basée sur la charge système
        const activeModules = this.getActiveModulesCount();
        const baseFPS = 60;
        const fpsDrop = activeModules * 2;
        return Math.max(15, baseFPS - fpsDrop + Math.random() * 5);
    }

    getActiveModulesCount() {
        let count = 0;
        if (this.advancedIntelligence) count++;
        if (this.collaborativeScouting) count++;
        if (this.flightBehaviors) count++;
        if (this.dronePhysics && this.dronePhysics.isActive !== false) count++;
        if (this.pidController && this.pidController.isActive !== false) count++;
        if (this.grassFieldBasic && this.grassFieldBasic.isActive !== false) count++;
        if (this.rosController && this.rosController.isActive !== false) count++;
        if (this.visualEnhancements && this.visualEnhancements.isActive !== false) count++;
        return count;
    }
}
