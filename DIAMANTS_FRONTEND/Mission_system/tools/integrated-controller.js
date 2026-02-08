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
import { AutonomousFlightEngine, DRONE_PROFILES } from '../physics/autonomous-flight-engine.js';
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
        this.sampleMode = false; // SAMPLE_MODE import removed - sample files in DEMO/

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
                    wsUrl: this.config.rosWsUrl || 'ws://localhost:8765',
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
        // NOTE: main.js already creates the primary AuthenticProvencalEnvironment with
        // forest, terrain, etc. This second instance is lightweight — NO forest, NO terrain
        // to avoid doubling trees/objects in the scene.
        this.environment = new AuthenticProvencalEnvironment(this.scene, {
            lightweight: true,
            terrainSize: { x: 200, y: 200 },
            structuredLayout: true,
            visibilityBubbles: true,
            terrainAmplitude: 0.0,  // Flat terrain to match Gazebo ground plane
            terrainDetail: 1.0,
            enableForest: false,    // Forest already created by main.js
            enableTerrain: false,   // Terrain already created by main.js
            enableSkybox: false,    // Skybox already created by main.js
            enableGrass: false,     // Grass already created by main.js
            enableFallenLeaves: false,
            enableForestWood: false,
            maxTrees: 0,
            forestDensity: 0,
            minTreeSpacing: 16.0
        });

        logger.info('Controller', '🌲 Environnement provençal créé');
    }

    /**
     * Création flotte de drones
     */
    async createDroneFleet() {
        logger.info('Controller', `🚁 Création flotte de ${this.config.droneCount} drones mixtes...`);

        // ── Create AutonomousFlightEngine ──
        this.autonomousFlightEngine = new AutonomousFlightEngine({
            explorationBounds: 50,
        });

        // ── Mixed drone type assignment ──
        // First 4 = Crazyflie, next 2 = Mavic, last 2 = Phantom
        const typeAssignment = [];
        for (let i = 0; i < this.config.droneCount; i++) {
            if (i < 4)       typeAssignment.push('CRAZYFLIE');
            else if (i < 6)  typeAssignment.push('MAVIC');
            else             typeAssignment.push('PHANTOM');
        }

        for (let i = 0; i < this.config.droneCount; i++) {
            // 1-indexed, zero-padded to match backend IDs (crazyflie_01 .. crazyflie_08)
            const droneId = `crazyflie_${String(i + 1).padStart(2, '0')}`;
            const profileName = typeAssignment[i];
            const profile = DRONE_PROFILES[profileName];

            // Initial position — drones on the enlarged heliport (radius 5.5m)
            const angle = (i / this.config.droneCount) * 2 * Math.PI;
            const spawnRadius = 5.5; // Matches the drone markers on platform
            const startPosition = {
                x: Math.cos(angle) * spawnRadius,
                y: 0.02,  // Ground level
                z: Math.sin(angle) * spawnRadius
            };

            // Créer drone Crazyflie authentique
            const drone = new AuthenticCrazyflie({
                id: droneId,
                position: startPosition,
                type: 'SCOUT',
                scene: this.scene,
                enableAdvancedBehaviors: this.config.enableRealisticFlight,
                collaborativeMode: this.config.enableCollaborativeScouting
            });

            // Tag with drone profile for UI display
            drone.droneProfile = profileName;
            drone.profileLabel = profile.label;

            // Override visual scale based on drone type
            if (drone.mesh) {
                try { drone.mesh.scale.setScalar(profile.scale); } catch(_) {}
            }

            // Register with autonomous flight engine
            const startVec = new THREE.Vector3(startPosition.x, startPosition.y, startPosition.z);
            this.autonomousFlightEngine.registerDrone(droneId, profileName, startVec);

            // Initialiser physique de vol si activée (ancienne méthode)
            if (this.flightBehaviors) {
                this.flightBehaviors.initializeDronePhysics(droneId, startPosition);
            }

            this.drones.push(drone);
            logger.debug('Controller', `✅ Drone ${droneId} [${profile.label}] créé à (${startPosition.x.toFixed(1)}, ${startPosition.z.toFixed(1)})`);
        }

        // Register trees from the environment
        this._registerEnvironmentTrees();

        // Connecter les drones aux systèmes avancés
        this.connectDronesToSystems();

        logger.info('Controller', `🎯 TOTAL: ${this.drones.length} drones créés (mixed fleet)`);
        
        // Vérification des positions
        log('🔍 Fleet positions:');
        this.drones.forEach((drone, index) => {
            const profile = typeAssignment[index];
            log(`  ${drone.id} [${profile}]: x=${drone.position.x.toFixed(2)}, y=${drone.position.y.toFixed(2)}, z=${drone.position.z.toFixed(2)}`);
        });
    }

    /**
     * Register trees from the environment as collision obstacles
     */
    _registerEnvironmentTrees() {
        // Look for trees in the scene
        const env = window.environment || this.environment;
        if (!env) return;

        let treeCount = 0;
        // Search for tree objects in the scene
        this.scene.traverse((obj) => {
            if (obj.name && (obj.name.includes('tree') || obj.name.includes('Tree') ||
                obj.name.includes('arbre') || obj.name.includes('Arbre') ||
                obj.name.includes('Pin') || obj.name.includes('Ch') ||
                obj.name.includes('Olivier'))) {
                const pos = new THREE.Vector3();
                obj.getWorldPosition(pos);
                // Estimate trunk radius from bounding box or default
                let radius = 2.0; // default trunk exclusion
                try {
                    const box = new THREE.Box3().setFromObject(obj);
                    const size = new THREE.Vector3();
                    box.getSize(size);
                    radius = Math.max(size.x, size.z) * 0.5; // horizontal extent
                    radius = Math.max(1.5, Math.min(radius, 5.0)); // clamp 1.5-5m
                } catch(_) {}
                this.autonomousFlightEngine.registerTree(pos, radius);
                treeCount++;
            }
        });
        log(`🌲 Registered ${treeCount} trees as collision obstacles`);
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
            // 1. Update AutonomousFlightEngine (PID + collision avoidance)
            if (this.autonomousFlightEngine) {
                this.autonomousFlightEngine.update(deltaTime);
            }

            // 1b. Legacy realistic flight dynamics (disabled)
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
     * Mise à jour comportements des drones — AutonomousFlightEngine prioritaire
     */
    updateDronesBehaviors(deltaTime) {
        // Ground level — no platform clamping, backend is authoritative
        const GROUND_LEVEL = 0.0;
        
        this.drones.forEach(drone => {
            // Déterminer si le drone est piloté par le backend (position vient du ROS)
            const isBackendDriven = drone.rosData && drone.rosData.position
                && (Date.now() - (drone.rosData.lastUpdate || 0)) < 5000;

            // ──────────────────────────────────────────────────
            // CAS 1: Drone piloté par le backend (position via ROS)
            // ──────────────────────────────────────────────────
            if (isBackendDriven && drone.mesh) {
                const bp = drone.rosData.position;
                let targetY = Math.max(GROUND_LEVEL, bp.y);

                // Smooth lerp towards backend position
                const lerpFactor = Math.min(deltaTime * 5.0, 1.0);
                drone.mesh.position.x += (bp.x - drone.mesh.position.x) * lerpFactor;
                drone.mesh.position.y += (targetY - drone.mesh.position.y) * lerpFactor;
                drone.mesh.position.z += (bp.z - drone.mesh.position.z) * lerpFactor;

                try {
                    if (drone.position) drone.position.copy(drone.mesh.position);
                    if (drone.velocity) drone.velocity.set(0, 0, 0);
                } catch (_) { /* safe */ }

                if (drone.state === 'IDLE') drone.state = 'FLYING';

                // Propeller animation from backend
                if (drone.rosData.propeller_speeds && drone.motors) {
                    const speeds = drone.rosData.propeller_speeds;
                    for (let i = 0; i < Math.min(4, speeds.length); i++) {
                        drone.motors[i].rpm = speeds[i];
                        drone.motors[i].omega = (speeds[i] / 60) * 2 * Math.PI;
                    }
                } else if (drone.motors) {
                    for (let i = 0; i < 4; i++) {
                        drone.motors[i].rpm = 14200;
                        drone.motors[i].omega = (14200 / 60) * 2 * Math.PI;
                    }
                }

                if (drone.updateVisuals) drone.updateVisuals(deltaTime);
                if (drone.rosData.battery !== undefined) drone.battery = drone.rosData.battery;
                if (drone.rosData.status) drone.backendStatus = drone.rosData.status;
                return; // Backend mode — no autonomous physics
            }

            // ──────────────────────────────────────────────────
            // CAS 2: Autonomous flight via AutonomousFlightEngine
            // Fast, fluid, dynamic exploration with PID + collision avoidance
            // ──────────────────────────────────────────────────
            if (this.autonomousFlightEngine) {
                const flightState = this.autonomousFlightEngine.getDroneState(drone.id);
                if (flightState) {
                    this.autonomousFlightEngine.applyToDrone(drone, flightState);
                    if (drone.updateVisuals) drone.updateVisuals(deltaTime);
                    return;
                }
            }

            // CAS 3: Fallback — simple update
            if (drone.update) {
                drone.update(deltaTime, this.drones);
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
            // Gazebo drones fly at ~0.5m — no elevated platform
            const altitude = 0.5 + (index * 0.1); // Slight stagger

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
                            wsUrl: this.config.rosWsUrl || 'ws://localhost:8765',
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
        // Real browser performance data (no fake simulation)
        return typeof performance !== 'undefined' && performance.measureUserAgentSpecificMemory
            ? 0 : 0; // Placeholder — backend should provide real telemetry
    }

    getMemoryUsage() {
        // Real browser memory if available
        if (performance && performance.memory) {
            return Math.round(performance.memory.usedJSHeapSize / (1024 * 1024));
        }
        return 0;
    }

    getFPS() {
        // Return 0 — real FPS should be measured by the render loop, not faked
        return 0;
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
