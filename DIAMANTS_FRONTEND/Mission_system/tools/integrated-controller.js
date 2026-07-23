/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Contrôleur Principal Intégré
 * ==========================================
 * Fallback 3-tier : Backend ROS2 → AutonomousFlightEngine → simple update.
 *
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
import { TerrainEnvironment } from '../environment/terrain-environment.js';
import { DiamantUI } from '../ui/diamant-ui.js';

// Nouveaux modules migrés
import { FlightBehaviors } from '../behaviors/flight-behaviors.js';
import { CollaborativeScouting } from '../behaviors/collaborative-scouting.js';
import { AdvancedCollectiveIntelligence } from '../intelligence/advanced-collective-intelligence.js';
import { CrazyflieRosController } from '../controllers/crazyflie-ros-controller.js';
import { CrazyflieVisualEnhancements } from '../visual/crazyflie-visual-enhancements.js';

// Modules supplémentaires "morts"
import { DronePhysics } from '../physics/drone-physics.js';
import { PIDController } from '../physics/pid-controller.js';
import { RealisticFlightDynamics } from '../physics/realistic-flight-dynamics.js';
import { GLSLGrassField } from '../environment/glsl-grass-field.js';
import { AutonomousFlightEngine } from '../physics/autonomous-flight-engine.js';
import { DRONE_PROFILES, DronePhysicsRegistry } from '../physics/drone-physics-registry.js';
import { loadStigmergyEngine, isStigmergyAvailable } from '../intelligence/stigmergy-loader.js';
import { FieldVisualizer3D, HarmonicsHUD } from '../core/field-visualizer-3d.js';
import { FlowParticles } from '../core/flow-particles.js';
import { DroneIntelligenceManager } from '../intelligence/drone-intelligence.js';
import { MultiAgentCoordinator } from '../agent/multi-agent-coordinator.js';
import { DroneVisualFactory } from '../drones/drone-visual-factory.js';
import { DepthCameraSimulator } from '../sensors/depth-camera-simulator.js';
import { AgentDroneRegistry, AgentRole } from '../agent/agent-drone-registry.js';
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
        
        // NOUVEAU: Visualisation 3D des champs DIAMANTS
        this.fieldVisualizer = null;
        this.harmonicsHUD = null;
        this.flowParticles = null;  // ← Essaim fluide

        // Collections
        this.drones = [];
        this.activeMissions = new Map();

        // Autonomy level (0 = centralisé, 100 = distribué)
        this.autonomyLevel = 100;

        // NOUVEAU: Multi-agent trainable coordinator
        this.multiAgentCoordinator = null;
        this._stigmergyEngine = null;  // Keep reference for fallback

        // NOUVEAU: LLM/Agent intelligence for drones
        this.droneIntelligenceManager = null;

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
            // 0. Pre-load authentic 3D drone models (GLB) before anything else
            try {
                await DroneVisualFactory.preloadModels();
                logger.info('Controller', '🎨 Authentic 3D drone models preloaded');
            } catch (e) {
                logger.warn('Controller', '⚠️ Model preload failed, using procedural fallback:', e.message);
            }

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

            // 7.1. Injecter la scène dans l'AutonomousFlightEngine (Multi-Ranger + CommWaveRenderer)
            // MUST happen here because the constructor's initializeSystem() is async but
            // not awaited, so main.js setupVoxelizer() runs before createDroneFleet() finishes.
            if (this.autonomousFlightEngine && this.scene) {
                this.autonomousFlightEngine.setScene(this.scene);
                logger.info('Controller', '📡 Scene injectée → Multi-Ranger + CommWaveRenderer initialisés');
            }

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

            // 9b. NOUVEAU: Initialiser visualisation 3D des champs DIAMANTS
            try {
                // Visualisation statique des champs (désactivée par défaut)
                this.fieldVisualizer = new FieldVisualizer3D(this.scene, this.diamantFormulas, {
                    particleSize: 0.5,
                    opacity: 0.85,
                    colorScale: 'viridis',
                    threshold: 0.005,
                    maxParticles: 5000,
                    updateInterval: 50,
                    offset: { x: -25, y: 0, z: -25 },
                    mode: 'psi'
                });
                this.fieldVisualizer.setVisible(false);  // Désactivé cosmétique
                
                // ✨ ESSAIM FLUIDE - Particules de flux (désactivé cosmétique)
                // Pour réactiver: window.diamantsSystem?.integratedController?.flowParticles?.setVisible(true)
                this.flowParticles = new FlowParticles(this.scene, this.diamantFormulas, {
                    particleCount: 3000,
                    particleSize: 0.2,
                    speed: 3.0,
                    lifetime: 6.0,
                    colorStart: 0x00ffcc,   // Cyan brillant
                    colorEnd: 0xff44ff,     // Magenta
                    domainSize: { x: 50, y: 12, z: 50 },
                    offset: { x: -25, y: 0, z: -25 }
                });
                this.flowParticles.setVisible(false); // Désactivé cosmétique
                
                this.harmonicsHUD = new HarmonicsHUD(this.diamantFormulas);
                this.harmonicsHUD.setVisible(false); // Masqué au démarrage — activé via bouton ◆
                logger.info('Controller', '🎨 Visualisation 3D DIAMANTS + FlowParticles initialisées');
            } catch (vizError) {
                console.warn('⚠️ Visualisation 3D non disponible:', vizError.message);
                console.error(vizError);
            }

            // 10. Mission NON auto-démarrée — attendre le bouton Launch
            // (anciennement: await this.startInitialMission())
            this.missionStarted = false;
            log('⏸️ Système prêt — appuyez sur Launch pour démarrer la mission');

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
        // NOTE: main.js already creates the primary TerrainEnvironment with
        // forest, terrain, etc. This second instance is lightweight — NO forest, NO terrain
        // to avoid doubling trees/objects in the scene.
        this.environment = new TerrainEnvironment(this.scene, {
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
            explorationBounds: 80,
        });

        // ── Connect DoctrineManager to the flight engine ──
        try {
            const dm = window.DIAMANTS_DOCTRINE || window.doctrineManager;
            if (dm) {
                this.autonomousFlightEngine.setDoctrineManager(dm);
            } else {
                console.warn('[Controller] DoctrineManager non trouvé — sera connecté plus tard');
                // Retry once after a short delay (DOM might not be ready yet)
                setTimeout(() => {
                    const dm2 = window.DIAMANTS_DOCTRINE || window.doctrineManager;
                    if (dm2 && this.autonomousFlightEngine) {
                        this.autonomousFlightEngine.setDoctrineManager(dm2);
                    }
                }, 1000);
            }
        } catch (e) { /* safe */ }

        // ── Load Stigmergy Engine (if available from diamants-private) ──
        const stigmergyEngine = await loadStigmergyEngine({
            gridSize: 100,
            gridResolution: 1.0,
            evaporationRate: 0.02,
        });
        if (stigmergyEngine) {
            this.autonomousFlightEngine.setSwarmIntelligence(stigmergyEngine);
            this._stigmergyEngine = stigmergyEngine;
            logger.info('Controller', '🧠 Stigmergy engine loaded and attached');
        }

        // ── Initialize Multi-Agent Trainable Coordinator ──
        try {
            this.multiAgentCoordinator = new MultiAgentCoordinator({
                communicationRange: this.config.communicationRange || 15,
                trainingEnabled: true,
                cooperativeRatio: 0.5
            });
            logger.info('Controller', '🤖 Multi-agent trainable coordinator created');
        } catch (e) {
            logger.warn('Controller', `Multi-agent coordinator init failed: ${e.message}`);
        }

        // ── Initialize Drone Intelligence Manager (LLM brains) — non-blocking ──
        try {
            this.droneIntelligenceManager = new DroneIntelligenceManager();
            // Don't await — Ollama ping can take 2s+ if not running
            this.droneIntelligenceManager.init().then(() => {
                this.autonomousFlightEngine?.setIntelligenceManager(this.droneIntelligenceManager);
                logger.info('Controller', '🧠 Drone Intelligence Manager initialized (Ollama LLM)');
            }).catch(e => {
                console.warn('[Controller] DroneIntelligence init failed (Ollama may not be running):', e.message);
                this.droneIntelligenceManager = null;
            });
        } catch (e) {
            console.warn('[Controller] DroneIntelligence creation failed:', e.message);
            this.droneIntelligenceManager = null;
        }

        // ── Fleet config drone type mapping ──
        // Map fleet_config.json types → DRONE_PROFILES keys
        // All non-Crazyflie types use the authentic X500 DAE model
        const FLEET_TYPE_TO_PROFILE = {
            'crazyflie': 'CRAZYFLIE',
            'scout': 'CRAZYFLIE',      // Scout uses Crazyflie physics profile
            'heavy': 'X500',           // Heavy → authentic X500 DAE
            'leader': 'X500',          // Leader → authentic X500 DAE
            'x500': 'X500',            // X500 cognitive drone
            's500': 'S500',            // S500 patrol drone (PX4 4014_s500)
            'cognitive': 'X500',       // Alias for cognitive agents
            'patrol': 'S500',          // Alias for patrol agents
            'reactive': 'CRAZYFLIE',   // Alias for reactive agents
        };

        // Build type assignments from fleet config
        // If no fleet config, use AgentDroneRegistry recommendation (mixed fleet)
        const fleetDrones = window.FLEET_CONFIG?.drones || [];
        const typeAssignment = [];

        if (fleetDrones.length > 0) {
            // Use explicit fleet config
            for (let i = 0; i < this.config.droneCount; i++) {
                const droneType = fleetDrones[i]?.type || 'crazyflie';
                typeAssignment.push(FLEET_TYPE_TO_PROFILE[droneType] || 'CRAZYFLIE');
            }
        } else {
            // Auto-compose mixed fleet: X500 cognitive + Crazyflie reactive
            const composition = AgentDroneRegistry.recommendFleetComposition(
                this.config.droneCount, 'balanced'
            );
            for (const { profileId, count } of composition) {
                for (let i = 0; i < count; i++) {
                    typeAssignment.push(profileId);
                }
            }
            log(`🧬 Auto-composed fleet: ${composition.map(c => `${c.count}×${c.profileId}`).join(' + ')}`);
        }

        // ── Spawn positions: type-aware layout (CF inner, X500/S500 outer) ──
        const cfIndices = [], x500Indices = [];
        for (let i = 0; i < this.config.droneCount; i++) {
            if (typeAssignment[i] === 'X500' || typeAssignment[i] === 'S500') x500Indices.push(i);
            else cfIndices.push(i);
        }

        const cfCount = cfIndices.length;
        const x500Count = x500Indices.length;
        const x500Scale = DRONE_PROFILES['X500']?.scale || 10;

        // Compute platform radius based on mixed fleet
        let platformRadius;
        if (x500Count === 0) {
            platformRadius = this.config.droneCount <= 8 ? 8.0
                : 8.0 * Math.sqrt(this.config.droneCount / 8);
        } else {
            // X500 outer ring needs spacing proportional to visual size
            const x500Spacing = x500Scale * 0.5;  // ~5m at scale 10
            const minRing = x500Count <= 4 ? 8 : 12;
            const x500RingRadius = Math.max(minRing, (x500Count * x500Spacing) / (2 * Math.PI));
            platformRadius = x500RingRadius + 3;
        }

        // Build per-drone positions: Crazyflies on inner rings, X500 on outer ring
        const spawnPositions = new Array(this.config.droneCount);

        // Inner rings for Crazyflies — wider ring for proper landing separation
        if (cfCount > 0) {
            const cfMaxR = Math.max(6, platformRadius * 0.55);
            const cfPositions = this._computeRingPositions(cfCount, cfMaxR);
            cfIndices.forEach((idx, i) => { spawnPositions[idx] = cfPositions[i]; });
        }

        // Outer ring(s) for X500
        if (x500Count > 0) {
            const x500R = platformRadius * 0.75;
            const x500Positions = [];
            for (let i = 0; i < x500Count; i++) {
                const angle = (i / x500Count) * 2 * Math.PI;
                x500Positions.push({ x: Math.cos(angle) * x500R, z: Math.sin(angle) * x500R });
            }
            x500Indices.forEach((idx, i) => { spawnPositions[idx] = x500Positions[i]; });
        }

        log(`🏗️ Platform: radius=${platformRadius.toFixed(1)}m, ${cfCount}×CF inner, ${x500Count}×X500/S500 outer`);

        // ── PARALLEL DRONE CREATION — build all mesh promises at once ──
        const droneSlots = [];
        for (let i = 0; i < this.config.droneCount; i++) {
            const profileName = typeAssignment[i];
            const spec = AgentDroneRegistry.getSpec(profileName);
            const prefix = profileName === 'X500' ? 'x500' : profileName === 'S500' ? 's500' : 'crazyflie';
            const droneId = `${prefix}_${String(i + 1).padStart(2, '0')}`;
            const profile = DRONE_PROFILES[profileName];
            const PLATFORM_SURFACE_Y = 0.15;
            const spawnY = (profileName === 'X500' || profileName === 'S500')
                ? PLATFORM_SURFACE_Y + (profile.scale || 10) * 0.22
                : PLATFORM_SURFACE_Y + 0.05;
            const startPosition = { x: spawnPositions[i].x, y: spawnY, z: spawnPositions[i].z };
            droneSlots.push({ i, profileName, spec, droneId, profile, startPosition });
        }

        // Fire all X500/S500 mesh creations in parallel
        const meshPromises = droneSlots.map(slot => {
            if (slot.profileName === 'CRAZYFLIE' || slot.profile.model === 'crazyflie') return Promise.resolve(null);
            return DroneVisualFactory.createAsync(slot.profile, this.scene).catch(e => { console.error(`❌ Mesh ${slot.droneId}:`, e); return null; });
        });
        const meshResults = await Promise.all(meshPromises);

        for (let si = 0; si < droneSlots.length; si++) {
          const { profileName, spec, droneId, profile, startPosition } = droneSlots[si];
          try {
            let drone;
            if (profileName === 'CRAZYFLIE' || profile.model === 'crazyflie') {
                drone = new AuthenticCrazyflie({
                    id: droneId,
                    position: startPosition,
                    type: spec.teamRole?.toUpperCase() || 'SCOUT',
                    scene: this.scene,
                    enableAdvancedBehaviors: this.config.enableRealisticFlight,
                    collaborativeMode: this.config.enableCollaborativeScouting
                });
                if (drone.mesh) {
                    try { drone.mesh.scale.setScalar(profile.scale); } catch(_) {}
                }
            } else {
                const meshResult = meshResults[si];
                if (!meshResult) { console.error(`❌ Drone ${droneId} mesh failed`); continue; }
                const { mesh, fovCone } = meshResult;
                mesh.position.set(startPosition.x, startPosition.y, startPosition.z);
                this.scene.add(mesh);

                drone = {
                    id: droneId,
                    mesh,
                    position: new THREE.Vector3(startPosition.x, startPosition.y, startPosition.z),
                    velocity: new THREE.Vector3(0, 0, 0),
                    state: 'IDLE',
                    type: spec.teamRole?.toUpperCase() || profileName,
                    fovCone,
                    update(dt) {
                        // Update position
                        if (this.mesh && this.position) this.mesh.position.copy(this.position);
                        this._spinProps(dt);
                    },
                    updateVisuals(dt) {
                        // Called by AutonomousFlightEngine path (CAS 2)
                        this._spinProps(dt);
                    },
                    _spinProps(dt) {
                        // Spin propellers — based on real motor specs:
                        // X500: 2216 KV920 + 4S → hover ~10000 RPM ≈ 1047 rad/s, max ~15000 RPM ≈ 1570 rad/s
                        // CF 2.1: 7×16mm coreless → hover ~16000 RPM ≈ 1675 rad/s, max ~21000 RPM ≈ 2200 rad/s
                        const props = this.mesh?.userData?.propellers;
                        const dirs = this.mesh?.userData?.propDirections;
                        if (props) {
                            const vy = this.velocity?.y || 0;
                            const y = this.position?.y || 0;
                            const hSpeed = Math.sqrt((this.velocity?.x || 0) ** 2 + (this.velocity?.z || 0) ** 2);
                            let speed;
                            if (y < 0.5) {
                                speed = 200; // idle on ground — slow visible spin
                            } else if (vy > 0.3) {
                                speed = 1500 + vy * 100; // climbing / takeoff — near max power
                            } else if (vy < -0.3) {
                                speed = 800; // descending — reduced but still fast
                            } else {
                                speed = 1200 + hSpeed * 30; // hover / cruise — nominal RPM
                            }
                            for (let i = 0; i < props.length; i++) {
                                const dir = dirs?.[i] ?? 1;
                                props[i].rotation.y += dt * speed * dir;
                            }
                        }
                    },
                    getPosition() { return this.position; },
                    destroy() { if (this.mesh?.parent) this.mesh.parent.remove(this.mesh); },

                    // ── 3D status label (same API as AuthenticCrazyflie) ──
                    _labelSprite: null,
                    _labelCanvas: null,
                    _labelLastDraw: 0,
                    _initLabel() {
                        if (!THREE || !this.mesh) return;
                        const canvas = document.createElement('canvas');
                        canvas.width = 640; canvas.height = 520;
                        const texture = new THREE.CanvasTexture(canvas);
                        texture.minFilter = THREE.LinearFilter;
                        const mat = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
                        this._labelSprite = new THREE.Sprite(mat);
                        const ps = this.mesh.scale?.x || 1;
                        this._labelSprite.position.set(0, 4.2 / ps, 0);
                        this._labelSprite.scale.set(6.0 / ps, 4.9 / ps, 1);
                        this._labelSprite.renderOrder = 999;
                        this.mesh.add(this._labelSprite);
                        this._labelCanvas = canvas;
                    },
                    updateStatusText(status) {
                        if (!this._labelSprite && this.mesh) this._initLabel();
                        if (!this._labelSprite || !this._labelCanvas) return;
                        const canvas = this._labelCanvas;
                        const ctx = canvas.getContext('2d');
                        const W = canvas.width, H = canvas.height;
                        ctx.clearRect(0, 0, W, H);
                        // Background
                        const r = 14;
                        ctx.fillStyle = 'rgba(8, 12, 20, 0.82)';
                        ctx.strokeStyle = 'rgba(80, 200, 255, 0.45)';
                        ctx.lineWidth = 3;
                        ctx.beginPath();
                        ctx.moveTo(r, 0); ctx.lineTo(W - r, 0);
                        ctx.quadraticCurveTo(W, 0, W, r); ctx.lineTo(W, H - r);
                        ctx.quadraticCurveTo(W, H, W - r, H); ctx.lineTo(r, H);
                        ctx.quadraticCurveTo(0, H, 0, H - r); ctx.lineTo(0, r);
                        ctx.quadraticCurveTo(0, 0, r, 0);
                        ctx.fill(); ctx.stroke();
                        ctx.textAlign = 'center';
                        if (typeof status === 'string') {
                            ctx.font = 'bold 30px monospace';
                            ctx.fillStyle = '#ffaa00';
                            ctx.textBaseline = 'middle';
                            ctx.fillText(`${this.id.toUpperCase()}: ${status}`, W / 2, H / 2);
                        } else {
                            const info = status;
                            const pColors = { IDLE:'#888', TAKEOFF:'#22ccff', EXPLORE:'#00ff88', HOVER:'#ffcc00', LAND:'#ff6644', LANDED:'#777' };
                            const pIcons  = { IDLE:'\u23F8', TAKEOFF:'\u2B06', EXPLORE:'\u2692', HOVER:'\u2B55', LAND:'\u2B07', LANDED:'\u2B1B' };
                            const phase = info.phase || 'IDLE';
                            const pC = pColors[phase] || '#fff';
                            // Line 1 — ID + Phase + LLM brain indicator
                            ctx.font = 'bold 38px monospace'; ctx.textBaseline = 'top'; ctx.fillStyle = pC;
                            const brainIcon = info.reasoning ? ' \uD83E\uDDE0' : '';
                            ctx.fillText(`${this.id.toUpperCase()}  ${pIcons[phase]||''} ${phase}${brainIcon}`, W/2, 12);
                            ctx.fillStyle = pC; ctx.fillRect(W*0.15, 50, W*0.7, 3);
                            // Line 2 — Doctrine / COA
                            ctx.font = 'bold 28px monospace'; ctx.fillStyle = '#88ccff';
                            ctx.fillText(`${info.doctrine||'\u2014'}  \u2502  ${info.coa||'\u2014'}`, W/2, 62);
                            // Line 3 — Autonomy mode badge + action
                            const a = info.autonomy ?? 100;
                            const aMode = info.autonomyMode || (a>=90?'DISTRIBUÉ':a>=75?'SEMI-AUTO':a>=50?'HYBRIDE':a>=25?'GUIDÉ':'CENTRAL');
                            const bY=102;
                            const bColors={'CENTRAL':'#00BFFF','GUIDÉ':'#33bbdd','HYBRIDE':'#66DDAA','SEMI-AUTO':'#88dd88','AUTONOME':'#aaee66','DISTRIBUÉ':'#00FF88'};
                            const bC=bColors[aMode]||'#00FF88';
                            ctx.font='bold 22px monospace'; const bW2=ctx.measureText(aMode).width+16||80;
                            ctx.fillStyle='rgba(0,0,0,0.5)'; ctx.beginPath(); ctx.roundRect(24,bY-4,bW2+8,22,4); ctx.fill();
                            ctx.fillStyle=bC; ctx.textAlign='left';
                            ctx.fillText(aMode, 30, bY+12);
                            if (info.action) {
                                ctx.textAlign='right'; ctx.fillStyle='#ffdd66'; ctx.font='bold 24px monospace';
                                const t = info.action.length>16 ? info.action.slice(0,16)+'\u2026' : info.action;
                                ctx.fillText(t, W-20, bY+14);
                            }
                            // Line 4 — Decision + WP
                            ctx.textAlign='center'; ctx.font='22px monospace'; ctx.fillStyle='#aac';
                            const wp = info.waypointsVisited!==undefined ? `WP:${info.waypointsVisited}` : '';
                            const dec = info.decision || '';
                            const l4 = [dec, wp].filter(Boolean).join('  \u2502  ');
                            if (l4) ctx.fillText(l4, W/2, 140);
                            // Line 5 — LLM reasoning
                            if (info.reasoning) {
                                ctx.font='bold 18px monospace'; ctx.fillStyle='#dbf';
                                const rt = info.reasoning.length>55 ? info.reasoning.slice(0,55)+'\u2026' : info.reasoning;
                                ctx.fillText(rt, W/2, 168);
                            }
                            // Line 5b — Beacon field awareness
                            const bZones = info.beaconZones || [];
                            if (bZones.length > 0) {
                                const bY5 = info.reasoning ? 190 : 168;
                                ctx.font='bold 17px monospace'; ctx.fillStyle='#ff9933';
                                ctx.fillText(`\uD83C\uDFAF ${bZones.length} zone${bZones.length>1?'s':''} balise${bZones.length>1?'s':''} connue${bZones.length>1?'s':''}`, W/2, bY5);
                            }
                            // Line 6+ — Rich Inter-drone COMMUNICATION panel
                            const cd = info.commDetails;
                            const commY = (bZones.length > 0 ? (info.reasoning ? 212 : 190) : (info.reasoning ? 196 : 168));
                            if (cd && cd.active && cd.peers && cd.peers.length > 0) {
                                // Bright border glow when actively communicating
                                ctx.strokeStyle='rgba(0,255,200,0.55)'; ctx.lineWidth=2;
                                ctx.beginPath();
                                const r2=10;
                                ctx.moveTo(r2,0); ctx.lineTo(W-r2,0);
                                ctx.quadraticCurveTo(W,0,W,r2); ctx.lineTo(W,H-r2);
                                ctx.quadraticCurveTo(W,H,W-r2,H); ctx.lineTo(r2,H);
                                ctx.quadraticCurveTo(0,H,0,H-r2); ctx.lineTo(0,r2);
                                ctx.quadraticCurveTo(0,0,r2,0);
                                ctx.stroke();
                                // Section header: COMM + own knowledge
                                ctx.textAlign='left';
                                ctx.font='bold 19px monospace'; ctx.fillStyle='#00ffcc';
                                ctx.fillText(`\uD83D\uDCE1 COMM  [${cd.peers.length} peer${cd.peers.length>1?'s':''}]`, 14, commY+2);
                                ctx.textAlign='right'; ctx.fillStyle='#557766'; ctx.font='15px monospace';
                                ctx.fillText(`\uD83D\uDDFA ${cd.ownKnowledge||0} cells  flow:${Math.round((cd.infoFlow||0)*100)}%`, W-14, commY+2);
                                // Peer rows (max 4 displayed)
                                const maxPeers = Math.min(cd.peers.length, 4);
                                for (let pi = 0; pi < maxPeers; pi++) {
                                    const p = cd.peers[pi];
                                    const rowY = commY + 22 + pi * 20;
                                    // Type-based colors
                                    const tColors = { MAP_SYNC:'#00ffd0', HEARTBEAT:'#4488aa', DIRECTIVE:'#ff8800' };
                                    const tIcons  = { MAP_SYNC:'\u21C4', HEARTBEAT:'\uD83D\uDC93', DIRECTIVE:'\u26A1' };
                                    const tC = tColors[p.type] || '#aaa';
                                    const tI = tIcons[p.type] || '\u2022';
                                    // Peer icon + shortId
                                    ctx.textAlign='left'; ctx.font='bold 17px monospace'; ctx.fillStyle=tC;
                                    ctx.fillText(`${p.icon||'\uD83D\uDC1C'} ${p.shortId}`, 14, rowY);
                                    // Type badge
                                    ctx.font='15px monospace';
                                    const badge = p.type === 'MAP_SYNC' ? 'SYNC' : p.type === 'HEARTBEAT' ? 'HB' : 'DIR';
                                    ctx.fillText(`${tI} ${badge}`, 130, rowY);
                                    // Data exchanged
                                    ctx.textAlign='center'; ctx.fillStyle='#8899aa'; ctx.font='15px monospace';
                                    if (p.type === 'MAP_SYNC' && p.cells > 0) {
                                        ctx.fillText(`${p.cells} cells`, W/2+30, rowY);
                                    } else if (p.type === 'HEARTBEAT') {
                                        ctx.fillText(`\u2208range`, W/2+30, rowY);
                                    }
                                    // Peer knowledge
                                    ctx.textAlign='right'; ctx.fillStyle='#667788'; ctx.font='14px monospace';
                                    if (p.peerKnowledge !== undefined) {
                                        ctx.fillText(`peer:${p.peerKnowledge}`, W-14, rowY);
                                    }
                                }
                                // Overflow indicator
                                if (cd.peers.length > 4) {
                                    ctx.textAlign='center'; ctx.font='14px monospace'; ctx.fillStyle='#556677';
                                    ctx.fillText(`+${cd.peers.length - 4} more`, W/2, commY + 22 + maxPeers * 20);
                                }
                                // Directive line
                                if (cd.directive) {
                                    const dY = commY + 26 + maxPeers * 20;
                                    ctx.textAlign='left'; ctx.font='bold 17px monospace'; ctx.fillStyle='#ff8800';
                                    const dirStr = `${cd.directive.from||'?'}→${cd.directive.target||'?'}`;
                                    const dTxt = dirStr.length > 40 ? dirStr.slice(0,40)+'\u2026' : dirStr;
                                    ctx.fillText(`\u26A1 ${dTxt}`, 14, dY);
                                }
                            } else if (info.comm) {
                                // Fallback: simple comm text when no commDetails available
                                ctx.textAlign='center';
                                ctx.font='bold 20px monospace';
                                const typeIcon = info.droneType==='CRAZYFLIE' ? '\uD83D\uDC1C' : '\uD83E\uDDE0';
                                ctx.fillStyle = info.commActive ? '#00ffcc' : '#66aa88';
                                const cTxt = info.comm.length>42 ? info.comm.slice(0,42)+'\u2026' : info.comm;
                                ctx.fillText(`${typeIcon} ${cTxt}`, W/2, commY);
                                if (info.localKnowledge > 0) {
                                    ctx.font='15px monospace';
                                    // Knowledge delta: how much this drone DOESN'T know vs global truth
                                    const delta = info.knowledgeDelta || 0;
                                    const deltaColor = delta > 20 ? '#ff6644' : delta > 5 ? '#ffaa44' : '#44ff88';
                                    ctx.fillStyle = deltaColor;
                                    const neighbors = info.commNeighborCount || 0;
                                    ctx.fillText(`🗺 ${info.localKnowledge}/${info.globalKnowledge||'?'} cells  Δ${delta}  📡${neighbors}`, W/2, commY+18);
                                }
                            }
                        }
                        this._labelSprite.material.map.needsUpdate = true;
                    }
                };
            }

            // Tag with profile + role metadata
            drone.droneProfile = profileName;
            drone.profileLabel = profile.label;
            drone.agentRole = spec.role;
            drone.brainType = spec.brainType;

            // Register with autonomous flight engine
            const startVec = new THREE.Vector3(startPosition.x, startPosition.y, startPosition.z);
            this.autonomousFlightEngine.registerDrone(droneId, profileName, startVec);

            // Register with LLM Intelligence Manager
            if (this.droneIntelligenceManager) {
                this.droneIntelligenceManager.registerDrone(droneId, profileName);
            }

            // Initialiser physique de vol si activée (ancienne méthode)
            if (this.flightBehaviors) {
                this.flightBehaviors.initializeDronePhysics(droneId, startPosition);
            }

            this.drones.push(drone);
            logger.debug('Controller', `✅ Drone ${droneId} [${profile.label}] créé à (${startPosition.x.toFixed(1)}, ${startPosition.z.toFixed(1)})`);
          } catch (droneErr) {
            console.error(`❌ Drone #${si} creation failed:`, droneErr);
          }
        }

        // ── FLEET CREATION DIAGNOSTIC ──
        const cfCreated = this.drones.filter(d => d.id?.startsWith('crazyflie')).length;
        const x500Created = this.drones.filter(d => d.id?.startsWith('x500') || d.id?.startsWith('s500')).length;
        const inScene = this.drones.filter(d => d.mesh?.parent).length;
        console.log(`🚁 FLEET DIAGNOSTIC: ${this.drones.length} drones (${cfCreated}×CF + ${x500Created}×X500/S500), ${inScene} in scene`);
        if (inScene < this.drones.length) {
            console.warn(`⚠️ ${this.drones.length - inScene} drones NOT in scene!`);
            this.drones.forEach(d => {
                if (!d.mesh?.parent) console.warn(`  ❌ ${d.id}: mesh.parent=${d.mesh?.parent}`);
            });
        }

        // Register trees from the environment (async — waits for env.ready)
        await this._registerEnvironmentTrees();

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
     * Distribute n drones across concentric rings on a platform
     * Returns array of {x, z} positions
     */
    _computeRingPositions(n, platformRadius) {
        const positions = [];
        if (n <= 8) {
            // Single ring at 85% radius — wider spread for proper landing separation
            const r = platformRadius * 0.85;
            for (let i = 0; i < n; i++) {
                const angle = (i / n) * 2 * Math.PI;
                positions.push({ x: Math.cos(angle) * r, z: Math.sin(angle) * r });
            }
            return positions;
        }
        // Multiple concentric rings
        const numRings = n <= 16 ? 2 : n <= 32 ? 3 : Math.min(5, Math.ceil(n / 10));
        const minR = platformRadius * 0.22;
        const maxR = platformRadius * 0.75;
        // Compute ring radii and distribute proportionally to circumference
        const rings = [];
        let totalCirc = 0;
        for (let r = 0; r < numRings; r++) {
            const radius = numRings === 1 ? maxR : minR + (maxR - minR) * r / (numRings - 1);
            const circ = 2 * Math.PI * radius;
            rings.push({ radius, circ });
            totalCirc += circ;
        }
        let assigned = 0;
        for (let r = 0; r < numRings; r++) {
            const count = r === numRings - 1
                ? n - assigned
                : Math.max(3, Math.round(n * rings[r].circ / totalCirc));
            const angleOffset = r * 0.3; // Stagger rings for visual clarity
            for (let i = 0; i < count; i++) {
                const angle = (i / count) * 2 * Math.PI + angleOffset;
                positions.push({
                    x: Math.cos(angle) * rings[r].radius,
                    z: Math.sin(angle) * rings[r].radius
                });
            }
            assigned += count;
        }
        return positions;
    }

    /**
     * Register trees from the environment as collision obstacles
     */
    async _registerEnvironmentTrees() {
        const env = window.environment || this.environment;
        if (!env) return;

        // Wait for environment to finish generating all trees
        if (env.ready) {
            try { await env.ready; } catch(e) { console.warn('[Trees] env.ready rejected:', e); }
        }

        let treeCount = 0;

        // ─── METHOD 1: Use environment.trees[] directly (best source of truth) ───
        // TerrainEnvironment stores all EZ-Tree Group objects in this.trees[]
        // Each tree is a THREE.Group with .branchesMesh and .leavesMesh children
        if (env.trees && env.trees.length > 0) {
            for (const treeObj of env.trees) {
                try {
                    // Force world matrix update so bounding box is accurate
                    treeObj.updateWorldMatrix(true, true);
                    
                    const pos = new THREE.Vector3();
                    treeObj.getWorldPosition(pos);
                    
                    // Compute bounding box from the full tree (branches + leaves)
                    const box = new THREE.Box3().setFromObject(treeObj);
                    const size = new THREE.Vector3();
                    box.getSize(size);
                    
                    // Canopy radius = half the max horizontal extent
                    let radius = Math.max(size.x, size.z) * 0.5;
                    radius = Math.max(2.0, Math.min(radius, 15.0)); // clamp 2-15m
                    
                    let height = size.y;
                    height = Math.max(3.0, Math.min(height, 30.0)); // clamp 3-30m
                    
                    // Use the base position (ground level), not center of bounding box
                    const basePos = new THREE.Vector3(pos.x, box.min.y, pos.z);
                    
                    this.autonomousFlightEngine.registerTree(basePos, radius, height);
                    treeCount++;
                    if (treeCount <= 3) {
                        log(`   🌳 Sample tree ${treeObj.name}: pos=(${basePos.x.toFixed(1)},${basePos.z.toFixed(1)}) r=${radius.toFixed(1)}m h=${height.toFixed(1)}m`);
                    }
                } catch(e) {
                    console.warn(`[Trees] Failed to register tree:`, e.message);
                }
            }
            log(`🌲 Registered ${treeCount} trees from environment.trees[] (direct)`);
            return;
        }

        // ─── METHOD 2: Fallback — scene traverse by name ───
        // Match ALL possible tree names (French species, English, EZ-Tree names)
        const TREE_NAME_PATTERNS = [
            'tree', 'Tree', 'arbre', 'Arbre',
            'Pin', 'Chêne', 'Chene', 'Ch',       // French species
            'Olivier', 'Cyprès', 'Cypres',         // French species
            'Oak', 'Pine', 'Cypress', 'Olive',     // English
            'Forest'                       // Forest group fallback
        ];
        
        const registered = new Set(); // prevent double-registration
        
        this.scene.traverse((obj) => {
            // Skip if already registered (parent group vs child mesh)
            if (registered.has(obj.uuid)) return;
            
            if (obj.name && TREE_NAME_PATTERNS.some(p => obj.name.includes(p))) {
                // If this is the forest GROUP, register its direct children instead
                if (obj.name === 'Forest') {
                    return; // children will be visited by traverse
                }
                
                // Skip internal mesh children (branchesMesh, leavesMesh)
                // We want the parent Group that IS the tree
                if (obj.parent && registered.has(obj.parent.uuid)) return;
                
                try {
                    obj.updateWorldMatrix(true, true);
                    const pos = new THREE.Vector3();
                    obj.getWorldPosition(pos);
                    
                    const box = new THREE.Box3().setFromObject(obj);
                    const size = new THREE.Vector3();
                    box.getSize(size);
                    
                    let radius = Math.max(size.x, size.z) * 0.5;
                    radius = Math.max(2.0, Math.min(radius, 15.0));
                    
                    let height = size.y;
                    height = Math.max(3.0, Math.min(height, 30.0));
                    
                    const basePos = new THREE.Vector3(pos.x, box.min.y, pos.z);
                    
                    this.autonomousFlightEngine.registerTree(basePos, radius, height);
                    registered.add(obj.uuid);
                    treeCount++;
                } catch(_) {}
            }
        });
        log(`🌲 Registered ${treeCount} trees via scene traverse (fallback)`);
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

        // ── Initialize Multi-Agent Coordinator with known drones ──
        if (this.multiAgentCoordinator && this.autonomousFlightEngine) {
            try {
                this.multiAgentCoordinator.initialize(
                    this.autonomousFlightEngine.drones,
                    this.autonomousFlightEngine.treeBounds || []
                );
                log(`🤖 Multi-agent coordinator: ${this.multiAgentCoordinator.agents.size} agents initialized`);
            } catch (e) {
                console.warn('[Controller] Multi-agent coordinator init failed:', e.message);
            }
        }
    }

    // ────────────────────────────────────────────────────────────────
    // HOT DRONE INJECTION / REMOVAL (Digital Twin Management)
    // ────────────────────────────────────────────────────────────────

    /**
     * Spawn a new drone into the scene at runtime.
     * Creates the 3D mesh (profile-appropriate), registers with the flight engine,
     * creates the RL agent, and optionally attaches a depth camera.
     *
     * @param {Object} options
     * @param {string} options.droneId - Unique ID (e.g. 'x500_01')
     * @param {string} options.profileId - Physics profile ID ('CRAZYFLIE','X500','MAVIC','PHANTOM')
     * @param {Object} options.position - {x, y, z} spawn position
     * @param {Object} [options.agentWeights] - Pre-trained RL weights to import
     * @param {boolean} [options.enableDepthCamera=false] - Attach OAK-D simulator
     * @returns {Object} { drone, agent, depthCamera } or null on failure
     */
    async spawnDrone(options = {}) {
        const {
            droneId,
            profileId = 'CRAZYFLIE',
            position = { x: 0, y: 0.15, z: 0 },
            agentWeights = null,
            enableDepthCamera = false
        } = options;

        if (!droneId) {
            console.warn('[Controller] spawnDrone: droneId required');
            return null;
        }

        // Check for duplicate
        if (this.drones.find(d => d.id === droneId)) {
            console.warn(`[Controller] Drone ${droneId} already exists`);
            return null;
        }

        const profile = DRONE_PROFILES[profileId] || DRONE_PROFILES['CRAZYFLIE'];
        const modelId = profile.model || 'generic';

        // 1. Create 3D mesh via factory
        let drone;
        if (modelId === 'crazyflie') {
            // Use existing AuthenticCrazyflie for backwards compatibility
            drone = new AuthenticCrazyflie({
                id: droneId,
                position,
                type: 'SCOUT',
                scene: this.scene,
                enableAdvancedBehaviors: this.config.enableRealisticFlight,
                collaborativeMode: this.config.enableCollaborativeScouting
            });
        } else {
            // Use the visual factory for non-Crazyflie types
            const { mesh, fovCone } = await DroneVisualFactory.createAsync(profile, this.scene);
            mesh.position.set(position.x, position.y, position.z);

            // Wrap in a compatible drone-like object
            drone = {
                id: droneId,
                mesh,
                position: new THREE.Vector3(position.x, position.y, position.z),
                velocity: new THREE.Vector3(0, 0, 0),
                type: profileId,
                droneProfile: profileId,
                profileLabel: profile.label,
                fovCone,
                update(dt) {
                    if (this.mesh && this.position) {
                        this.mesh.position.copy(this.position);
                    }
                    this._spinProps(dt);
                },
                updateVisuals(dt) {
                    this._spinProps(dt);
                },
                _spinProps(dt) {
                    // Spin propellers — based on real motor specs:
                    // X500: 2216 KV920 + 4S → hover ~10000 RPM ≈ 1047 rad/s, max ~15000 RPM ≈ 1570 rad/s
                    // CF 2.1: 7×16mm coreless → hover ~16000 RPM ≈ 1675 rad/s, max ~21000 RPM ≈ 2200 rad/s
                    const props = this.mesh?.userData?.propellers;
                    const dirs = this.mesh?.userData?.propDirections;
                    if (props) {
                        const vy = this.velocity?.y || 0;
                        const y = this.position?.y || 0;
                        const hSpeed = Math.sqrt((this.velocity?.x || 0) ** 2 + (this.velocity?.z || 0) ** 2);
                        let speed;
                        if (y < 0.5) {
                            speed = 200; // idle on ground — slow visible spin
                        } else if (vy > 0.3) {
                            speed = 1500 + vy * 100; // climbing / takeoff — near max power
                        } else if (vy < -0.3) {
                            speed = 800; // descending — reduced but still fast
                        } else {
                            speed = 1200 + hSpeed * 30; // hover / cruise — nominal RPM
                        }
                        for (let i = 0; i < props.length; i++) {
                            const dir = dirs?.[i] ?? 1;
                            props[i].rotation.y += dt * speed * dir;
                        }
                    }
                },
                getPosition() { return this.position; },
                destroy() {
                    if (this.mesh?.parent) this.mesh.parent.remove(this.mesh);
                }
            };

            // Add to scene
            this.scene.add(mesh);
        }

        drone.droneProfile = profileId;
        drone.profileLabel = profile.label;

        // 2. Register with the flight engine (physics)
        if (this.autonomousFlightEngine) {
            const startVec = new THREE.Vector3(position.x, position.y, position.z);
            this.autonomousFlightEngine.registerDrone(droneId, profileId, startVec);
        }

        // 3. Create RL agent with profile-appropriate brain
        let agent = null;
        if (this.multiAgentCoordinator) {
            agent = this.multiAgentCoordinator.addAgent(droneId, {
                profileId  // Pass profile so coordinator assigns right brain
            }, agentWeights);
        }

        // Tag drone with agent metadata
        const spec = AgentDroneRegistry.getSpec(profileId);
        drone.agentRole = spec.role;
        drone.brainType = spec.brainType;

        // 4. Optionally attach depth camera (OAK-D)
        let depthCamera = null;
        const rawProfile = profile._raw;
        if (enableDepthCamera || AgentDroneRegistry.hasDepthCamera(profileId) || rawProfile?.sensors?.camera) {
            depthCamera = new DepthCameraSimulator(droneId, {
                hFov: rawProfile?.sensors?.hFov || 127,
                vFov: rawProfile?.sensors?.vFov || 80,
                maxDepth: rawProfile?.sensors?.depthRange?.[1] || 35,
                rayCount: 64
            });
            drone.depthCamera = depthCamera;
        }

        // Add to drones array
        this.drones.push(drone);

        log(`✅ Drone ${droneId} [${profile.label}] spawned at (${position.x.toFixed(1)}, ${position.y.toFixed(1)}, ${position.z.toFixed(1)}) — ${modelId} mesh, depth=${!!depthCamera}`);

        return { drone, agent, depthCamera };
    }

    /**
     * Remove a drone from the scene at runtime.
     * Cleans up 3D mesh, flight engine registration, and RL agent.
     *
     * @param {string} droneId
     * @returns {Object|null} Exported agent weights, or null if not found
     */
    despawnDrone(droneId) {
        const idx = this.drones.findIndex(d => d.id === droneId);
        if (idx === -1) {
            console.warn(`[Controller] Drone ${droneId} not found`);
            return null;
        }

        const drone = this.drones[idx];

        // 1. Remove from RL coordinator (exports weights before removal)
        let weights = null;
        if (this.multiAgentCoordinator) {
            weights = this.multiAgentCoordinator.removeAgent(droneId);
        }

        // 2. Remove from flight engine
        if (this.autonomousFlightEngine?.drones) {
            this.autonomousFlightEngine.drones.delete(droneId);
        }

        // 3. Remove 3D mesh from scene
        if (drone.destroy) {
            drone.destroy();
        } else if (drone.mesh?.parent) {
            drone.mesh.parent.remove(drone.mesh);
        }

        // 4. Remove from drones array
        this.drones.splice(idx, 1);

        log(`❌ Drone ${droneId} despawned (${this.drones.length} remaining)`);
        return weights;
    }

    /**
     * List available drone profiles for injection.
     * @returns {Array<{id, label, category, mass, maxSpeed}>}
     */
    listDroneProfiles() {
        const registry = DronePhysicsRegistry.getInstance();
        return registry.listProfiles().map(id => {
            const p = registry.getProfile(id);
            return {
                id: p.id,
                label: p.label,
                category: p.category,
                mass: p.mass,
                maxSpeed: p.maxSpeed,
                model: p.model,
                hasSensors: !!p._raw?.sensors
            };
        });
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
     * Physics/AI subsystems run at max 30Hz; visuals run every frame.
     */
    update(deltaTime) {
        if (!this.isRunning) return;

        // ══════════════════════════════════════════════════════════════════
        // PERFORMANCE-CRITICAL: Only essential systems run in the loop.
        // Everything else is DISABLED to prevent main-thread freeze.
        //
        // ACTIVE:  AutonomousFlightEngine (10Hz), drone sync, metrics (0.5Hz)
        // DISABLED: MARL, DIAMANTS PDE, realisticFlightDynamics, flightBehaviors,
        //           collaborativeScouting, advancedIntelligence, missionManager,
        //           fieldVisualizer, flowParticles, harmonicsHUD
        // ══════════════════════════════════════════════════════════════════

        // ── Physics at 10 Hz (was 30Hz — ÷3 CPU) ──
        this._physicsAccum = (this._physicsAccum || 0) + deltaTime;
        const PHYSICS_STEP = 0.1; // 10 Hz
        if (this._physicsAccum < PHYSICS_STEP) {
            // Between physics ticks: only sync mesh positions (visual smoothness)
            this.updateDronesBehaviors(deltaTime);
            return;
        }
        const physicsDt = this._physicsAccum;
        this._physicsAccum = 0;

        try {
            // 1. AutonomousFlightEngine — the ONLY physics system
            if (this.autonomousFlightEngine) {
                this.autonomousFlightEngine.update(physicsDt);
            }

            // 2. Sync drone visuals (mesh position, propellers, labels)
            this.updateDronesBehaviors(deltaTime);

            // 3. Metrics — ultra rare (0.5 Hz = every 2 seconds)
            this._metricsFrame = (this._metricsFrame || 0) + 1;
            if (this._metricsFrame % 20 === 0) {
                this.updateMetrics();
            }

            // 3b. MARL Coordinator — federated learning at 1 Hz
            if (this.multiAgentCoordinator && this._metricsFrame % 10 === 0) {
                try {
                    this.multiAgentCoordinator.tick(physicsDt * 10);
                } catch (_) {}
            }

            // 3c. Broadcast drone-positions for minimaps (1 Hz fallback if coordinator missed it)
            if (this._metricsFrame % 10 === 0 && this.autonomousFlightEngine) {
                try {
                    const positions = {};
                    for (const [id, st] of this.autonomousFlightEngine.drones) {
                        if (st.position) {
                            positions[id] = {
                                position: { x: st.position.x, y: st.position.y, z: st.position.z },
                                type: st.profileId || 'crazyflie',
                                source: 'engine',
                            };
                        }
                    }
                    window.dispatchEvent(new CustomEvent('diamants:drone-positions', { detail: positions }));
                } catch (_) {}
            }

            // 4. Debug log — every 30 seconds
            if (this._metricsFrame % 300 === 0 && this.autonomousFlightEngine) {
                const phases = [];
                for (const [id, st] of this.autonomousFlightEngine.drones) {
                    phases.push(`${id}:${st.phase}`);
                }
                console.log(`[STATE] ${phases.join(', ')}`);
            }

        } catch (error) {
            console.error('❌ Update error:', error.message);
        }
    }

    /**
     * Mise à jour comportements des drones — AutonomousFlightEngine prioritaire
     * autonomyLevel est propagé à l'engine qui gère formation/hybride/distribué
     */
    updateDronesBehaviors(deltaTime) {
        // Ground level = platform surface height (PLATFORM_HEIGHT = 0.15)
        const GROUND_LEVEL = 0.15;

        // ── Sync autonomy from UI slider (window global) → controller → engine ──
        // DiamantsUIController writes to window.DIAMANTS_AUTONOMY_LEVEL on slider change.
        // We read it here so the integrated controller stays in sync.
        if (typeof window !== 'undefined' && window.DIAMANTS_AUTONOMY_LEVEL !== undefined) {
            const uiLevel = Math.max(0, Math.min(100, window.DIAMANTS_AUTONOMY_LEVEL));
            if (uiLevel !== this.autonomyLevel) {
                this.autonomyLevel = uiLevel;
            }
        }

        // Propagate autonomy to flight engine (the engine handles waypoint strategy)
        if (this.autonomousFlightEngine && this.autonomousFlightEngine.autonomyLevel !== this.autonomyLevel) {
            this.autonomousFlightEngine.setAutonomyLevel(this.autonomyLevel);
        }
        
        this.drones.forEach((drone) => {
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
            // CAS 2: AutonomousFlightEngine
            // L'engine gère internement l'autonomyLevel:
            //   0%  → formation serrée (waypoints coordonnés)
            //   50% → sous-groupes hybrides
            //  100% → exploration individuelle distribuée
            // ──────────────────────────────────────────────────
            if (this.autonomousFlightEngine) {
                const flightState = this.autonomousFlightEngine.getDroneState(drone.id);
                if (flightState) {
                    this.autonomousFlightEngine.applyToDrone(drone, flightState);
                    // Flag so AuthenticCrazyflie.updateVisuals() skips overwriting
                    // the rotation/position that applyToDrone just set
                    drone._engineControlled = true;
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

        // Métriques de scouting — primary source: flight engine visitedCells
        if (this.autonomousFlightEngine) {
            const visited = this.autonomousFlightEngine.visitedCells?.size || 0;
            const bounds = this.autonomousFlightEngine.explorationBounds || 60;
            const cs = this.autonomousFlightEngine.cellSize || 3;
            const totalCells = Math.pow(2 * bounds / cs, 2);
            this.metrics.coveragePercentage = Math.min(100, (visited / totalCells) * 100);
        } else if (this.collaborativeScouting) {
            const scoutingStatus = this.collaborativeScouting.getMissionStatus();
            this.metrics.coveragePercentage = scoutingStatus.progress * 100;
            this.metrics.collaborationEfficiency = scoutingStatus.collaborationMetrics.coordinationIndex;
        }

        // Métriques d'intelligence avancée (wahoo uniquement, PAS d'émergence)
        if (this.advancedIntelligence) {
            const advancedState = this.advancedIntelligence.getAdvancedState();
            this.metrics.wahooEffectLevel = advancedState.metrics.wahooEffectiveness;
            // emergenceLevel N'EST PLUS lu depuis advancedIntelligence
        }

        // Métriques de mission
        this.metrics.missionSuccess = this.calculateMissionSuccess();

        // Métriques DIAMANTS — SEULE source de vérité pour émergence/cohérence
        // (EI + TE scientifique, Klein & Hoel 2020 / Schreiber 2000)
        //
        // NOTE: diamantFormulas.update() (PDE fields) est désactivé du loop principal
        // pour performance. Mais les swarmMetrics (Vicsek, Reynolds, EI+TE) sont
        // légers et doivent tourner pour alimenter le panneau Intelligence Collective.
        if (this.diamantFormulas) {
            // Mettre à jour _activeAgentCount (normalement fait par update())
            const drones = this.drones || [];
            this.diamantFormulas._activeAgentCount = drones.filter(a => {
                const s = (a.state || 'IDLE').toUpperCase();
                return s !== 'IDLE' && s !== 'LANDED';
            }).length;

            // Calcul léger des métriques scientifiques d'essaim
            // (Vicsek alignment, Reynolds cohesion/separation, EI+TE emergence)
            try {
                this.diamantFormulas.updateSwarmMetrics(drones);
            } catch (e) {
                if (!this._swarmMetricsErrorLogged) {
                    console.warn('⚠️ updateSwarmMetrics error (safe):', e.message);
                    this._swarmMetricsErrorLogged = true;
                }
            }

            // Synchroniser emergence_factor et coherence_level depuis swarmMetrics
            this.diamantFormulas.emergence_factor = this.diamantFormulas._activeAgentCount > 0
                ? this.diamantFormulas.swarmMetrics.emergence : 0;
            this.diamantFormulas.coherence_level = this.diamantFormulas._activeAgentCount > 0
                ? this.diamantFormulas.swarmMetrics.alignment : 0;

            this.metrics.diamantsValue = this.diamantFormulas.diamants_value || 0;
            this.metrics.emergenceLevel = this.diamantFormulas.emergence_factor ?? 0;
            this.metrics.coherenceLevel = this.diamantFormulas.coherence_level ?? 0;
        } else {
            this.metrics.emergenceLevel = 0;
        }
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
     * Emit custom events consumed by OrchestrationConsole tabs.
     * Called every ~2Hz from update() alongside updateMetrics().
     * THROTTLED: drone-positions uses change detection to avoid log saturation.
     */
    _emitOrchestrationEvents() {
        if (typeof window === 'undefined') return;
        const engine = this.autonomousFlightEngine;

        // ── DRONES: position summary for "Drones" tab ──
        // Only emit when a drone phase changed OR position moved >1m (avoids flooding)
        if (this.drones.length > 0) {
            const positions = {};
            let changed = false;
            if (!this._lastDronePhases) this._lastDronePhases = {};
            if (!this._lastDronePositions) this._lastDronePositions = {};
            this.drones.forEach(drone => {
                const p = drone.mesh?.position || drone.position || { x: 0, y: 0, z: 0 };
                const engineState = engine?.getDroneState(drone.id);
                const phase = engineState?.phase || drone.state || 'UNKNOWN';
                positions[drone.id] = {
                    position: { x: p.x, y: p.y, z: p.z },
                    state: phase,
                    source: 'engine',   // Tag so backend listener in main.js ignores these
                };
                // Detect meaningful changes
                const prevPhase = this._lastDronePhases[drone.id];
                const prevPos = this._lastDronePositions[drone.id];
                if (prevPhase !== phase) {
                    changed = true;
                } else if (prevPos) {
                    const dx = p.x - prevPos.x, dz = p.z - prevPos.z;
                    if (dx * dx + dz * dz > 1.0) changed = true; // >1m movement
                } else {
                    changed = true; // first time
                }
                this._lastDronePhases[drone.id] = phase;
                this._lastDronePositions[drone.id] = { x: p.x, y: p.y, z: p.z };
            });
            if (changed) {
                window.dispatchEvent(new CustomEvent('diamants:drone-positions', { detail: positions }));
            }
        }

        // ── SLAM: coverage/visited cells update for "SLAM" tab ──
        if (engine) {
            const visited = engine.visitedCells?.size || 0;
            const prevVisited = this._lastVisitedCount || 0;
            if (visited !== prevVisited) {
                this._lastVisitedCount = visited;
                window.dispatchEvent(new CustomEvent('diamants:slam-map', {
                    detail: { cells: { length: visited }, coverage: this.metrics.coveragePercentage || 0 }
                }));
            }
        }

        // ── MISSION: mission state for "Mission" tab ──
        {
            const flyingCount = this.drones.filter(d => {
                const st = engine?.getDroneState(d.id);
                return st?.phase === 'EXPLORE' || st?.phase === 'TAKEOFF' || st?.phase === 'HOVER';
            }).length;
            const coverage = this.metrics.coveragePercentage || 0;
            const success = this.metrics.missionSuccess || 0;
            const phase = flyingCount === 0 ? 'IDLE' : (this.missionStarted ? 'ACTIVE' : 'STANDBY');

            // Only emit when something changed
            const mKey = `${phase}:${flyingCount}:${coverage.toFixed(0)}`;
            if (mKey !== this._lastMissionKey) {
                this._lastMissionKey = mKey;
                window.dispatchEvent(new CustomEvent('diamants:mission-status', {
                    detail: { phase, flyingDrones: flyingCount, total: this.drones.length, coverage: coverage.toFixed(1), success: (success * 100).toFixed(0) }
                }));
            }
        }

        // ── SWARM: intelligence metrics for "Swarm" tab ──
        if (engine) {
            const dm = engine.doctrineManager || window.DIAMANTS_DOCTRINE;
            const doctrine = dm?.currentDoctrine?.name || 'N/A';
            const coa = dm?.currentCOA?.name || 'N/A';
            const autonomy = engine.autonomyLevel;
            const mode = engine.useOrganicMode ? 'Organic' : 'Swarm Intelligence';
            const emergence = this.metrics.emergenceLevel || 0;
            const coherence = this.metrics.coherenceLevel || 0;

            const sKey = `${doctrine}:${coa}:${autonomy}:${mode}:${emergence.toFixed(2)}`;
            if (sKey !== this._lastSwarmKey) {
                this._lastSwarmKey = sKey;
                window.dispatchEvent(new CustomEvent('diamants:swarm-update', {
                    detail: { formation: `${doctrine} / ${coa}`, mode, autonomy, coherence, emergence }
                }));
            }
        }
    }

    /**
     * Commandes de contrôle
     */
    
    /**
     * Démarrage MANUEL des missions - appelée depuis le control panel
     */
    async startMissionManual() {
        log('🎯 Démarrage MANUEL de la mission depuis le control panel...');
        
        // Réactiver le contrôleur si précédemment stoppé
        this.isRunning = true;
        
        // Si les drones sont au sol (IDLE/LANDED), les faire décoller d'abord
        if (this.autonomousFlightEngine) {
            let needsTakeoff = false;
            for (const [id, state] of this.autonomousFlightEngine.drones) {
                if (state.phase === 'IDLE' || state.phase === 'LANDED') {
                    needsTakeoff = true;
                    break;
                }
            }
            
            if (needsTakeoff) {
                // Use each drone profile's native cruiseAlt (Crazyflie=3m, X500=10m)
                this.drones.forEach((drone) => {
                    this.autonomousFlightEngine.takeoff(drone.id);
                });
                // Wait for drones to reach cruise altitude before exploring
                // X500 needs ~4s to climb 10m at 3m/s, Crazyflie ~2s for 3m
                setTimeout(() => {
                    this.autonomousFlightEngine.startExploration();
                }, 4000);
            } else {
                // Drones déjà en l'air (HOVER/TAKEOFF) → lancer l'exploration
                this.autonomousFlightEngine.startExploration();
            }
        }
        
        this.missionStarted = true;
        log('✅ Mission démarrée manuellement');

        // Notify orchestration console
        if (typeof window !== 'undefined') {
            window.dispatchEvent(new CustomEvent('diamants:mission-status', {
                detail: { phase: 'LAUNCH', status: 'Takeoff initiated', total: this.drones.length }
            }));
        }
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

        // Keep isRunning = true so the physics loop continues to animate the landing
        // S-curve. The drones need active physics ticks to descend smoothly.
        this.isRunning = true;
        this.missionStarted = false; // Permettre un re-Launch

        // Notify orchestration console
        if (typeof window !== 'undefined') {
            window.dispatchEvent(new CustomEvent('diamants:mission-status', {
                detail: { phase: 'EMERGENCY_STOP', status: 'All drones landing' }
            }));
        }

        // Synchroniser avec AutonomousFlightEngine
        this.drones.forEach((drone, idx) => {
            const droneId = drone.id || `crazyflie_${String(idx + 1).padStart(2, '0')}`;
            
            // Mettre l'engine en mode LAND — the physics loop drives the S-curve descent
            if (this.autonomousFlightEngine) {
                this.autonomousFlightEngine.land(droneId);
            }
            // Note: realisticFlightDynamics and flightBehaviors operate on separate
            // state Maps that don't drive rendering. Removed to prevent confusion.
        });

        // Clean per-drone stale state so re-Launch starts fresh
        // (visitedCells + pheromones preserved — drones don't re-explore known areas)
        if (this.autonomousFlightEngine) {
            const engine = this.autonomousFlightEngine;
            // Clear formation drift, waypoint caches, per-drone memory
            engine._formationCenter.set(0, 0, 0);
            engine._formationPhase = 0;
            engine._formationAdvanceTimer = 0;
            engine._formationDirty = true;
            engine._doctrineWaypoints.clear();
            engine._pathWaypoints.clear();
            for (const [id, state] of engine.drones) {
                state._recentCells = [];
                state._recentCellSet.clear();
                state._stallTimer = 0;
                state._escaleCount = 0;
                state._oscillationCount = 0;
                state._oscillationTimer = 0;
                state._lastVelSign = null;
                state._wpInitialDist = null;
                // Territory preserved (don't force re-dispersion on resume)
            }
        }

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
