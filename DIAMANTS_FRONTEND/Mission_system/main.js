/**
 * DIAMANTS - Point d'entrée principal avec Vite et EZ-Tree authentique
 * =======================================================================
 * ⚠️  v0-origin  (commit 47cec8ee — tag v0-origin)
 * Version de référence : drones scootent, exploration fluide, 100% frontend.
 * Restaurer : git checkout v0-origin -- main.js
 *
 * Cache bust: 2025-09-13-14:55
 */

// Mode silencieux pour les logs - TRUE = moins de logs, FALSE = tous les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;
window.SILENT_MODE = true; // RÉACTIVÉ pour stopper le déluge

// OVERRIDE SÉLECTIF - Désactiver seulement les logs non-critiques
if (window.SILENT_MODE) {
    const originalLog = console.log;
    const originalInfo = console.info;
    
    // Override sélectif qui garde les logs 3D critiques
    console.log = (...args) => {
        let text = '';
        try { text = args.map(String).join(' '); } catch { originalLog(...args); return; }
        // Garder les logs critiques pour le 3D et les erreurs
        if (text.includes('DAE') || text.includes('THREE') || text.includes('ERROR') || 
            text.includes('ERREUR') || text.includes('✅') || text.includes('❌') ||
            text.includes('Moteur') || text.includes('hélice') || text.includes('propeller')) {
            originalLog(...args);
        }
        // Bloquer tous les autres logs
    };
    
    console.info = (...args) => {
        let text = '';
        try { text = args.map(String).join(' '); } catch { originalInfo(...args); return; }
        if (text.includes('DAE') || text.includes('THREE') || text.includes('ERROR')) {
            originalInfo(...args);
        }
    };
}

// DIAMANTS - Chargement principal
const log = window.log || console.log.bind(console);
const warn = window.warn || console.warn.bind(console);
const error = window.error || console.error.bind(console);
log('🔥 MAIN.JS STARTING - DIAMANTS Loading...');
log('📍 main.js file loaded and executing');

// Attendre que THREE.js soit disponible globalement
async function waitForTHREE() {
    return new Promise((resolve, reject) => {
        if (window.THREE && window.THREE_READY) {
            log('✅ THREE.js déjà prêt !');
            resolve(window.THREE);
            return;
        }
        
        // Écouter l'événement threeReady
        window.addEventListener('threeReady', (event) => {
            log('✅ THREE.js maintenant prêt !');
            resolve(event.detail.THREE);
        });
        
        // Timeout de sécurité
        setTimeout(() => {
            if (!window.THREE) {
                reject(new Error('Timeout: THREE.js non disponible après 10 secondes'));
            }
        }, 10000);
    });
}

// Import conditionnel pour rester compatible avec les modules
let THREE, OrbitControls, EffectComposer, RenderPass, OutputPass;

// Initialisation asynchrone
async function initializeTHREE() {
    try {
        // Attendre le bootstrap THREE.js
        THREE = await waitForTHREE();
        log('✅ THREE.js bootstrap terminé, récupération des composants...');
        
        // Utiliser les composants globaux ou importer si nécessaire
        if (window.OrbitControls) {
            OrbitControls = window.OrbitControls;
        } else {
            const OrbitControlsModule = await import('three/addons/controls/OrbitControls.js');
            OrbitControls = OrbitControlsModule.OrbitControls;
        }
        
        // Import des composants post-processing
        try {
            const EffectComposerModule = await import('three/addons/postprocessing/EffectComposer.js');
            const RenderPassModule = await import('three/addons/postprocessing/RenderPass.js');
            const OutputPassModule = await import('three/addons/postprocessing/OutputPass.js');
            
            EffectComposer = EffectComposerModule.EffectComposer;
            RenderPass = RenderPassModule.RenderPass;
            OutputPass = OutputPassModule.OutputPass;
        } catch (postProcessError) {
            warn('⚠️ Post-processing non disponible:', postProcessError);
        }
        
        log('✅ Tous les composants THREE.js chargés');
        return true;
    } catch (error) {
        console.error('🚨 Erreur initialisation THREE.js:', error);
        return false;
    }
}

import { AuthenticProvencalEnvironment } from './environment/authentic-provencal-environment.js';
import { AuthenticCrazyflie } from './drones/authentic-crazyflie.js';
// ROS/WebSocket bridge removed — simulation-only build (public vitrine)
import { IntegratedDiamantsController } from './tools/integrated-controller.js';
import { OrchestrationConsole } from './ui/orchestration-console.js';
import { MetricsUIConnector } from './tools/metrics-ui-connector.js';
import { BenchmarkRunner, runQuickBenchmark } from './tools/benchmark-runner.js';
import { BenchmarkChartGenerator } from './tools/benchmark-charts.js';
import { initDiamantsUI } from './ui/diamants-ui-controller.js';
import { ExplorationMinimap } from './ui/exploration-minimap.js';
import { initPanelController } from './ui/panel-controller.js';
import { initDoctrineManager } from './missions/mission-doctrine.js';
import { ButtonTestSuite, TEST_DATASETS } from './tests/button-test-suite.js';

class DiamantsMissionSystem {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.composer = null;
        this.controls = null;
        this.environment = null;
        this.drones = [];
        this.droneMarkers = [];
        this.followDroneIndex = -1; // Index du drone suivi (-1 = aucun)
        this.integratedController = null; // NOUVEAU: Contrôleur intégré
        this.metricsUI = null; // Connecteur métriques UI
        this.benchmark = null; // Runner benchmark
        this.explorationMinimap = null; // Minimap d'exploration
        // Backend-less WebSim can publish/sub ROS topics via rosbridge
        this.roswebEnabled = false; // ROS bridge removed — simulation-only build
        this.ros = null;
        // Visual safety defaults: clearer visuals, full manual control
        this.autoFocus = false; // off by default to avoid camera auto-move
        this.usePostProcessing = false; // no bloom/glow post-processing
        this.visualSafeMode = true; // no fog, no tone mapping halos
        this._focusSmoothing = { center: null, radius: 10 }; // center sera initialisé plus tard
        this.clock = null; // clock sera initialisé plus tard
        
        // Orchestration console — instantiated early so it captures all events
        this.orchestrationConsole = new OrchestrationConsole();

        log('🚀 Initialisation DIAMANTS Mission System V3 avec EZ-Tree');
        this.init();
    }

    async init() {
        // Attendre que THREE.js soit prêt
        log('⏳ Attente de THREE.js...');
        const threeReady = await initializeTHREE();
        if (!threeReady) {
            console.error('🛑 Impossible d\'initialiser THREE.js. Arrêt.');
            return;
        }
        
        // Maintenant que THREE.js est disponible, initialiser les objets THREE
        this._focusSmoothing.center = new THREE.Vector3();
        this.clock = new THREE.Clock();
        
        await this.setupRenderer();
        // If renderer couldn't be created (WebGL blocked/unavailable), stop early
        if (!this.renderer) {
            console.error('🛑 WebGL renderer unavailable. Aborting further initialization.');
            return;
        }
        await this.setupScene();
        await this.setupCamera();
        await this.setupControls();
        await this.setupPostProcessing();
        await this.setupEnvironment();
        await this.setupIntegratedController(); // NOUVEAU: Contrôleur intégré
        if (this.roswebEnabled) await this.setupRosWeb();
        
        this.setupEventListeners();
        this.animate();
        
        log('✅ DIAMANTS Mission System V3 initialisé avec succès');
        
        // Activer immédiatement le debug des collisions
        setTimeout(() => {
            const collisionDebugToggle = document.getElementById('collision_debug_toggle');
            const collisionVisualsToggle = document.getElementById('collision_visuals_toggle');
            
            if (collisionDebugToggle && window.toggleCollisionDebug) {
                collisionDebugToggle.checked = true;
                window.toggleCollisionDebug();
                log('🔍 Debug des collisions activé automatiquement');
            }
            
            if (collisionVisualsToggle && window.toggleCollisionVisuals) {
                collisionVisualsToggle.checked = true;
                window.toggleCollisionVisuals();
                log('🔍 Visualisation des collisions activée automatiquement');
            }
        }, 500); // Délai pour s'assurer que tous les éléments sont prêts
    }

    async setupRenderer() {
        const container = document.getElementById('canvas_container') || document.body;

        // Try multiple WebGL context creation strategies
        const canvas = document.createElement('canvas');
        
        // Strategy 1: Conservative approach
        const safeAttribs = {
            alpha: false,
            depth: true,
            stencil: false,
            antialias: false,
            desynchronized: false, // More conservative
            preserveDrawingBuffer: false,
            premultipliedAlpha: false,
            powerPreference: 'default',
            failIfMajorPerformanceCaveat: false
        };

        // Strategy 2: Minimal approach
        const minimalAttribs = {
            alpha: false,
            antialias: false,
            powerPreference: 'default'
        };

        let gl = null;
        
        // Try WebGL2 first with safe attributes
        try {
            gl = canvas.getContext('webgl2', safeAttribs);
            log('✅ WebGL2 context created with safe attributes');
        } catch (e) { 
            warn('WebGL2 safe failed:', e.message);
        }
        
        // Try WebGL2 with minimal attributes
        if (!gl) {
            try {
                gl = canvas.getContext('webgl2', minimalAttribs);
                log('✅ WebGL2 context created with minimal attributes');
            } catch (e) { 
                warn('WebGL2 minimal failed:', e.message);
            }
        }
        
        // Try WebGL1 with safe attributes
        if (!gl) {
            try {
                gl = canvas.getContext('webgl', safeAttribs);
                log('✅ WebGL1 context created with safe attributes');
            } catch (e) { 
                warn('WebGL1 safe failed:', e.message);
            }
        }
        
        // Try WebGL1 with minimal attributes
        if (!gl) {
            try {
                gl = canvas.getContext('webgl', minimalAttribs);
                log('✅ WebGL1 context created with minimal attributes');
            } catch (e) { 
                warn('WebGL1 minimal failed:', e.message);
            }
        }
        
        // Try experimental WebGL as last resort
        if (!gl) {
            try {
                gl = canvas.getContext('experimental-webgl', minimalAttribs);
                log('✅ Experimental WebGL context created');
            } catch (e) { 
                warn('Experimental WebGL failed:', e.message);
            }
        }

        if (!gl) {
            console.error('🛑 All WebGL context creation attempts failed');
            this.showWebglBlockedOverlay();
            this.renderer = null;
            return;
        }

        // Try to create THREE.js renderer with the obtained context
        try {
            this.renderer = new THREE.WebGLRenderer({ 
                canvas, 
                context: gl, 
                antialias: false, 
                powerPreference: 'default',
                precision: 'mediump' // More conservative precision
            });
            log('✅ THREE.js WebGL renderer created successfully');
        } catch (e) {
            console.error('🛑 THREE.js renderer creation failed:', e.message);
            this.showWebglBlockedOverlay();
            this.renderer = null;
            return;
        }

        // Safer defaults to reduce memory pressure and avoid context loss
        this.renderer.setClearColor(0x87CEEB, 1.0); // Couleur ciel méditerranéen
        const width = Math.max(1, container.clientWidth || window.innerWidth);
        const height = Math.max(1, container.clientHeight || window.innerHeight);
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(1); // keep low to avoid VRAM spikes

        // Keep shadows off initially to avoid heavy allocations; can be toggled later
        this.renderer.shadowMap.enabled = false;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

        // Tone mapping: off — no halos. Toggle visualSafeMode=false to re-enable.
        if (this.visualSafeMode) {
            this.renderer.toneMapping = THREE.NoToneMapping;
            this.renderer.toneMappingExposure = 1.0;
        } else {
            this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
            this.renderer.toneMappingExposure = 1.2;
        }

        // Gamma correction
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;

        // Attach and handle context loss/restoration to avoid the browser blocking the page
    const onContextLost = (e) => {
            try { e.preventDefault(); } catch (_) {}
            warn('⚠️ WebGL context LOST - pausing rendering to avoid browser blocking');
            this._contextLost = true;
        };
    const onContextRestored = () => {
            console.info('✅ WebGL context RESTORED - resetting sizes and resuming');
            this._contextLost = false;
            try {
                const w = Math.max(1, container.clientWidth || window.innerWidth);
                const h = Math.max(1, container.clientHeight || window.innerHeight);
                this.renderer.setSize(w, h);
            } catch (_) {}
        };
    this._onWebglContextLost = onContextLost;
    this._onWebglContextRestored = onContextRestored;
    this.renderer.domElement.addEventListener('webglcontextlost', this._onWebglContextLost, false);
    this.renderer.domElement.addEventListener('webglcontextrestored', this._onWebglContextRestored, false);

        container.appendChild(this.renderer.domElement);
        log('🎨 Renderer configuré (safe defaults)');
        // Expose THREE globally for modules expecting window.THREE (e.g., AuthenticCrazyflie)
        try { window.THREE = THREE; } catch (_) { /* ignore if not in browser */ }
    }

    showWebglBlockedOverlay() {
        // Enhanced WebGL error overlay with comprehensive solutions
        const existing = document.getElementById('webgl-error-overlay');
        if (existing) return;
        const overlay = document.createElement('div');
        overlay.id = 'webgl-error-overlay';
        overlay.style.cssText = `
            position: fixed; inset: 0; display: flex; align-items: center; justify-content: center;
            background: rgba(0,0,0,0.95); color: #fff; z-index: 10000; font-family: system-ui, sans-serif;
            padding: 20px; overflow-y: auto;
        `;
        overlay.innerHTML = `
            <div style="max-width: 600px; background: #2d2d2d; padding: 30px; border-radius: 10px; box-shadow: 0 10px 30px rgba(0,0,0,0.5);">
                <div style="font-size: 24px; margin-bottom: 20px; color: #ff6b6b; text-align: center;">⚠️ WebGL Non Disponible</div>
                <div style="margin-bottom: 20px; opacity: 0.9; line-height: 1.5;">
                    WebGL est bloqué ou indisponible. Toutes les stratégies de récupération ont échoué.
                </div>
                
                <div style="margin: 25px 0;">
                    <div style="font-size: 18px; color: #4ecdc4; margin-bottom: 10px;">🔄 Solutions Rapides :</div>
                    <ul style="margin: 0; padding-left: 20px; line-height: 1.6;">
                        <li><strong>Redémarrer le navigateur</strong> (fermer complètement et rouvrir)</li>
                        <li><strong>Actualiser la page</strong> (Ctrl+F5 ou Cmd+Shift+R)</li>
                        <li><strong>Fermer d'autres onglets 3D/jeux</strong> pour libérer WebGL</li>
                        <li><strong>Vider le cache</strong> : Paramètres → Confidentialité → Vider les données</li>
                    </ul>
                </div>
                
                <div style="margin: 25px 0;">
                    <div style="font-size: 18px; color: #4ecdc4; margin-bottom: 10px;">⚙️ Solutions Avancées :</div>
                    <ul style="margin: 0; padding-left: 20px; line-height: 1.6;">
                        <li><strong>Chrome :</strong> Aller à <code style="background: #1e1e1e; padding: 2px 6px; border-radius: 3px; color: #ffd700;">chrome://flags/</code> et activer "WebGL"</li>
                        <li><strong>Firefox :</strong> Aller à <code style="background: #1e1e1e; padding: 2px 6px; border-radius: 3px; color: #ffd700;">about:config</code> et vérifier "webgl.disabled"</li>
                        <li><strong>Hardware acceleration :</strong> Activer dans les paramètres du navigateur</li>
                        <li><strong>Mode incognito :</strong> Essayer pour désactiver les extensions</li>
                    </ul>
                </div>
                
                <div style="margin: 25px 0;">
                    <div style="font-size: 18px; color: #4ecdc4; margin-bottom: 10px;">🛠️ Si le problème persiste :</div>
                    <ul style="margin: 0; padding-left: 20px; line-height: 1.6;">
                        <li>Mettre à jour les drivers graphiques</li>
                        <li>Désactiver temporairement l'antivirus/extensions</li>
                        <li>Tester avec un autre navigateur (Chrome, Firefox, Edge)</li>
                        <li>Redémarrer l'ordinateur</li>
                    </ul>
                </div>
                
                <div style="text-align: center; margin-top: 30px;">
                    <button onclick="window.location.reload()" style="
                        padding: 12px 24px; background: #4CAF50; color: white; border: none; 
                        border-radius: 5px; cursor: pointer; font-size: 16px; margin-right: 10px;
                        transition: all 0.2s ease;
                    " onmouseover="this.style.background='#45a049'; this.style.transform='translateY(-1px)'" 
                       onmouseout="this.style.background='#4CAF50'; this.style.transform='translateY(0)'">
                        🔄 Réessayer
                    </button>
                    <button onclick="window.open('chrome://flags/#enable-webgl', '_blank')" style="
                        padding: 12px 24px; background: #2196F3; color: white; border: none; 
                        border-radius: 5px; cursor: pointer; font-size: 16px;
                        transition: all 0.2s ease;
                    " onmouseover="this.style.background='#1976D2'; this.style.transform='translateY(-1px)'" 
                       onmouseout="this.style.background='#2196F3'; this.style.transform='translateY(0)'">
                        ⚙️ Paramètres WebGL
                    </button>
                </div>
            </div>`;
        try { document.body.appendChild(overlay); } catch (_) { /* ignore */ }
    }

    async setupRosWeb() {
        // Network bridge removed — simulation-only build (public vitrine)
        log('ℹ️ Mode simulation locale — pas de bridge réseau');
    }

    async setupScene() {
        this.scene = new THREE.Scene();
        this.scene.name = 'DiamantsMissionScene';
        // Aide visuelle minimale pour mieux percevoir le mouvement
        try {
            const hemi = new THREE.HemisphereLight(0xffffff, 0x334466, 0.6);
            const dir = new THREE.DirectionalLight(0xffffff, 0.6);
            dir.position.set(50, 80, 40);
            dir.castShadow = true;
            this.scene.add(hemi);
            this.scene.add(dir);
        } catch (_) { /* safe */ }
        log('🎬 Scène créée');
    }

    async setupCamera() {
        const container = this.renderer.domElement.parentElement;
        const aspect = container.clientWidth / container.clientHeight;
        
        this.camera = new THREE.PerspectiveCamera(60, aspect, 0.01, 1000);
        // Elevated view: above tree canopy, looking down at the arena
        this.camera.position.set(20, 18, 20);
        this.camera.lookAt(0, 2, 0);
        
        log('📷 Caméra configurée');
    }

    async setupControls() {
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.enablePan = true;
        this.controls.enableZoom = true;
        this.controls.enableRotate = true;
        
        // Limites réalistes pour la caméra
        this.controls.minDistance = 1;
        this.controls.maxDistance = 100;
        this.controls.minPolarAngle = Math.PI / 6; // Pas trop bas
        this.controls.maxPolarAngle = Math.PI / 2 + 0.1; // Légèrement sous l'horizon
        
        // Centrer les contrôles sur l'altitude des drones (0.5m)
        this.controls.target.set(0, 0.5, 0);
        
        // DÉSACTIVER toute animation automatique
        this.controls.autoRotate = false;
        this.controls.autoRotateSpeed = 0;
        
        this.controls.update();
        
        log('🎮 Contrôles configurés (autoRotate DÉSACTIVÉ)');
    }

    async setupPostProcessing() {
        const container = this.renderer.domElement.parentElement;
        
        this.composer = new EffectComposer(this.renderer);
        
        // Pass de rendu principal
        const renderPass = new RenderPass(this.scene, this.camera);
        this.composer.addPass(renderPass);
        
        // Pass de sortie (SMAA antialiasing supprimé)
        const outputPass = new OutputPass();
        this.composer.addPass(outputPass);
        
    log('🎭 Post-processing configuré (désactivé par défaut)');
    }

    async setupEnvironment() {
        log('🌍 Création environnement provençal authentique...');
        
        // Afficher un indicateur de chargement
        this.showLoadingIndicator();
        
        try {
            this.environment = new AuthenticProvencalEnvironment(this.scene, {
                terrainSize: { x: 200, y: 200 },
                forestDensity: 0.4,
                maxTrees: 60,
                minTreeSpacing: 16.0,
                enableSkybox: true,
                enableTerrain: true,
                enableForest: true,          // Forest AROUND the arena
                enableFallenLeaves: true,
                enableForestWood: true,
                enableUndergrowth: true,
                grassCount: 60000,
                leavesCount: 2000,
                arenaExclusionRadius: 12,    // No trees inside 12m radius — clear zone around heliport
            });
            
            // NOUVEAU: Exposer l'environnement globalement pour le système de collision
            window.environment = this.environment;
            log('✅ Environnement exposé globalement pour système de collision');
            
            // Attendre que l'environnement soit prêt
            await new Promise(resolve => setTimeout(resolve, 1000));
            // Pas de brume en safe mode (halos cosmétiques désactivés)
            this.scene.fog = null;
            // Ensuite, déployer une petite flotte de Crazyflie bien visibles
            this.setupDrones();
            // Exposer un pont de compatibilité global pour l'UI (window.DIAMANTS)
            this.exposeGlobalBridge();
            
            // Initialiser la minimap d'exploration
            try {
                this.explorationMinimap = new ExplorationMinimap('minimap_canvas');
                log('🗺️ ExplorationMinimap initialisée');
            } catch (e) {
                warn('⚠️ Erreur init ExplorationMinimap:', e);
            }
            
        } catch (error) {
            console.error('❌ Erreur création environnement:', error);
        } finally {
            this.hideLoadingIndicator();
        }
        
        log('✅ Environnement provençal prêt');
    }

    async setupIntegratedController() {
        log('🎛️ Initialisation contrôleur intégré...');
        
        try {
            // Configuration du contrôleur intégré avec tous les modules avancés
            // NOTE: le contrôleur attend (scene, config). On passe donc la scène en 1er arg
            this.integratedController = new IntegratedDiamantsController(this.scene, {
                camera: this.camera,
                renderer: this.renderer,
                
                // Activer tous les modules avancés
                enableCollaborativeScouting: true,
                enableAdvancedIntelligence: true,
                enableMissionManagement: true,
                enablePhysicsEngine: true,
                enableVisualEnhancements: true,
                // Disable ROS by default; allow page to override via window.CONFIG
                enableRosController: (typeof window !== 'undefined' && window.CONFIG && typeof window.CONFIG.enableRosController === 'boolean')
                    ? window.CONFIG.enableRosController
                    : false,
                enableGrassFieldBasic: false, // En fallback seulement
                
                // Configuration des comportements
                explorationRadius: 30.0,
                communicationRange: 15.0,
                wahooIntensity: 1.5,
                emergenceThreshold: 0.8,
                
                // Paramètres de performance
                droneCount: 8, // Default fleet size
                maxConcurrentDrones: 8,
                updateFrequency: 30, // Hz
                
                // Mode développement
                debugMode: true,
                verbose: true
            });
            
            // Mettre à jour le pont global pour l'UI principale
            try {
                window.DIAMANTS = window.DIAMANTS || {};
                // Expose system instance for diagnostic access
                window.DIAMANTS.system = this;
                // Exposer l'accès au MissionManager attendu par l'UI
                window.DIAMANTS.getMissionManager = () => this.integratedController?.missionManager || null;
                // Exposer le contrôleur pour outils debug
                window.DIAMANTS.controller = this.integratedController;
                // Exposer métriques et drones pour le panneau legacy
                window.DIAMANTS.metrics = this.integratedController?.metrics || {};
                window.DIAMANTS.drones = this.drones;
                // Exposer un arrêt d'urgence unifié
                window.DIAMANTS.emergencyStop = () => {
                    try {
                        this.integratedController?.emergencyStop?.();
                    } catch (_) {}
                };

                // Helpers caméra/viewport utilisés par l'UI historique
                window.DIAMANTS.resetCamera = () => {
                    try {
                        this.camera.position.set(12, 12, 12);
                        this.controls.target.set(0, 0.5, 0);
                        this.controls.update();
                    } catch (_) { /* noop */ }
                };
                window.DIAMANTS.topView = () => {
                    try {
                        const { center, radius } = this.computeFleetBounds();
                        this.camera.position.set(center.x, Math.max(radius * 2.2, 25), center.z);
                        this.controls.target.copy(center);
                        this.controls.update();
                    } catch (_) { /* noop */ }
                };
                window.DIAMANTS.toggleFollowMode = () => {
                    // Mode suiveur non implémenté: simple stub pour compatibilité UI
                    log('ℹ️ Follow mode non implémenté dans cette build');
                };
                window.DIAMANTS.zoomToSwarm = () => {
                    try {
                        const { center, radius } = this.computeFleetBounds();
                        const dist = Math.max(radius * 2.5, 30);
                        this.camera.position.set(center.x + dist, center.y + dist * 0.6, center.z + dist);
                        this.controls.target.copy(center);
                        this.controls.update();
                    } catch (_) { /* noop */ }
                };
            } catch (_) { /* ignore */ }

            log('✅ Contrôleur intégré initialisé avec succès');
            
            // === NOUVEAU: Initialiser le connecteur métriques UI ===
            this.metricsUI = new MetricsUIConnector(this);
            this.metricsUI.start(500); // Mise à jour toutes les 500ms
            log('📊 MetricsUIConnector démarré');
            
            // === NOUVEAU: Initialiser le système de benchmark ===
            this.benchmark = new BenchmarkRunner(this);
            window.benchmark = this.benchmark;
            window.runQuickBenchmark = (counts, dur) => runQuickBenchmark(this, counts, dur);
            window.benchmarkCharts = new BenchmarkChartGenerator();
            
            // Export CSV/JSON pour le benchmark
            window.benchmarkExportCSV = () => {
                const csv = this.benchmark.exportCSV();
                console.log('📊 CSV Export:\n', csv);
                // Download
                const blob = new Blob([csv], { type: 'text/csv' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url; a.download = 'diamants_benchmark.csv'; a.click();
            };
            window.benchmarkExportJSON = () => {
                const json = this.benchmark.exportJSON();
                const blob = new Blob([json], { type: 'application/json' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url; a.download = 'diamants_benchmark.json'; a.click();
            };
            log('🧪 Système benchmark initialisé');
            
            // === NOUVEAU: Initialiser le nouveau UI Controller (index-v2.html) ===
            if (document.getElementById('panel-mission')) {
                this.uiController = initDiamantsUI(this);
                log('🎨 DiamantsUIController initialisé');
            }
            
            // Expose engine + core objects globally for diagnostics and legacy tooling
            try {
                window.engineInstance = this.integratedController;
                window.scene = this.scene;
                window.camera = this.camera;
                window.renderer = this.renderer;
                window.drones = this.drones;
            } catch (_) { /* noop */ }
            
        } catch (error) {
            console.error('❌ Erreur initialisation contrôleur intégré:', error);
        }
    }

    // Pont de compatibilité pour les UIs qui attendent window.DIAMANTS
    exposeGlobalBridge() {
        try {
            const self = this;
            window.DIAMANTS = window.DIAMANTS || {};
            // Référencer le tableau (mise à jour en direct)
            window.DIAMANTS.drones = this.drones;
            // Miroir des métriques si le contrôleur intégré est présent
            Object.defineProperty(window.DIAMANTS, 'metrics', {
                get() { return self.integratedController?.metrics || {}; }
            });
            // Stubs utiles pour les menus debug de l'index
            if (!window.DIAMANTS.getSystemStats) {
                window.DIAMANTS.getSystemStats = () => ({
                    drones: self.drones.length,
                    scene: !!self.scene,
                    renderer: !!self.renderer,
                    fps: self?.composer ? undefined : undefined
                });
            }
            if (!window.DIAMANTS.emergencyStop) {
                window.DIAMANTS.emergencyStop = () => {
                    self.drones.forEach(d => d?.emergencyStop?.());
                };
            }
            if (!window.DIAMANTS.resetSimulation) {
                window.DIAMANTS.resetSimulation = () => {
                    // Remettre une altitude sûre
                    self.drones.forEach((d, i) => {
                        if (d?.position) {
                            d.position.y = Math.max(d.position.y, 2.0);
                            if (d.mesh) d.mesh.position.y = d.position.y;
                        }
                    });
                };
            }
            // Émettre un signal d'init si besoin
            try { window.dispatchEvent(new CustomEvent('diamants:initialized', { detail: { drones: self.drones.length } })); } catch(_) {}
            log('🔗 Pont global window.DIAMANTS exposé (compat UI)');
            
            // Initialiser le Panel Controller et Doctrine Manager
            try {
                initDoctrineManager();
                initPanelController();
                log('✅ Panel Controller et Doctrine Manager initialisés');
                
                // Exposer la suite de tests
                window.ButtonTestSuite = ButtonTestSuite;
                window.TEST_DATASETS = TEST_DATASETS;
                
                // Auto-run tests en mode développement (après 2s pour laisser tout charger)
                if (window.location.hostname === 'localhost') {
                    setTimeout(() => {
                        console.log('\\n🧪 AUTO-TEST DÉMARRÉ (mode dev)...');
                        const results = window.runButtonTests();
                        if (results.failed > 0) {
                            console.warn(`⚠️ ${results.failed} tests échoués - voir détails ci-dessus`);
                        } else {
                            console.log('✅ Tous les tests passent !');
                        }
                        
                        // Diagnostic état drones post-init
                        const sys = window.diamantsSystem;
                        if (sys?.integratedController) {
                            const ic = sys.integratedController;
                            const engine = ic.autonomousFlightEngine;
                            console.log('\\n🔍 DIAGNOSTIC POST-INIT:');
                            console.log(`   isRunning: ${ic.isRunning}`);
                            console.log(`   missionStarted: ${ic.missionStarted}`);
                            ic.drones.forEach((drone, i) => {
                                const flightState = engine?.getDroneState(drone.id);
                                console.log(`   ${drone.id}: state=${drone.state}, enginePhase=${flightState?.phase}, pos.y=${drone.position?.y?.toFixed(2)}`);
                            });
                        }
                    }, 4000);
                }
            } catch (e) {
                warn('⚠️ Erreur init Panel/Doctrine:', e);
            }
        } catch (_) { /* noop */ }
    }

    setupDrones() {
        // Les drones sont créés et gérés par l'IntegratedController
        // Ne pas créer de drones supplémentaires ici
        log('🚁 Les drones seront créés par l\'IntegratedController');
        
        // Initialiser un tableau vide qui sera rempli par l'IntegratedController
        this.drones = [];
    }

    // Calcule le centre et le rayon englobant de la flotte pour cadrer la caméra
    // (implémentation unique — Box3-based, voir plus bas)
    // Déplacé pour éviter doublon

    showLoadingIndicator() {
        const indicator = document.createElement('div');
        indicator.id = 'loading-indicator';
        indicator.style.cssText = `
            position: fixed;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 20px;
            border-radius: 10px;
            font-family: monospace;
            font-size: 16px;
            z-index: 1000;
        `;
        indicator.innerHTML = '🌲 Génération environnement provençal authentique...<br>Utilisation EZ-Tree';
        document.body.appendChild(indicator);
    }

    hideLoadingIndicator() {
        const indicator = document.getElementById('loading-indicator');
        if (indicator) {
            indicator.remove();
        }
    }

    setupEventListeners() {
        // Redimensionnement
    this._onResize = () => this.onWindowResize();
    window.addEventListener('resize', this._onResize);
        
        // Raccourcis clavier
    this._onKeyDown = (event) => this.onKeyDown(event);
    window.addEventListener('keydown', this._onKeyDown);
        
        log('🎧 Event listeners configurés');
    }

    onWindowResize() {
    if (!this.renderer) return; // renderer unavailable
        const container = this.renderer.domElement.parentElement;
        const width = container.clientWidth;
        const height = container.clientHeight;
        
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        
        this.renderer.setSize(width, height);
        if (this.composer) this.composer.setSize(width, height);
    }

    onKeyDown(event) {
        switch (event.code) {
            case 'KeyF':
                // Plein écran
                if (document.fullscreenElement) {
                    document.exitFullscreen();
                } else {
                    document.documentElement.requestFullscreen();
                }
                break;
                
            case 'KeyR':
                // Reset caméra - centré sur les drones
                this.camera.position.set(12, 12, 12);
                this.controls.target.set(0, 0.5, 0);
                this.controls.update();
                break;
            case 'KeyA':
                // Toggle auto-focus camera framing - FORCÉ DÉSACTIVÉ
                log('🚫 AutoFocus DÉSACTIVÉ DÉFINITIVEMENT - contrôle manuel uniquement');
                this.autoFocus = false; // Force toujours false
                break;
            case 'KeyP':
                // Toggle post-processing
                this.usePostProcessing = !this.usePostProcessing;
                log(`🎭 Post-processing ${this.usePostProcessing ? 'activé' : 'désactivé'}`);
                break;
            case 'KeyG':
                // Fog désactivée (cosmétique)
                log('🌫️ Brume désactivée (cosmétique)');
                break;
                
            case 'KeyL':
                // VERROUILLER la caméra en position actuelle
                this.lockCamera();
                break;
                
            case 'KeyH':
                // Aide
                log(`
🎮 DIAMANTS Mission System V3 - Contrôles:
F: Plein écran
R: Reset caméra  
A: Toggle auto-focus caméra
P: Toggle post-processing
G: Toggle brume (fog)
H: Cette aide
Souris: Navigation 3D
                `);
                break;
        }
    }

    animate() {
        this._rafId = requestAnimationFrame(() => this.animate());
        
        // Delta temps pour les mises à jour physiques des drones
        const delta = this.clock.getDelta();
        this.elapsedTime = (this.elapsedTime || 0) + delta;

        // Mettre à jour contrôles - SANS ANIMATION AUTOMATIQUE
        this.controls.update();
        
        // VERROUILLAGE DE SÉCURITÉ - Empêcher tout mouvement automatique
        if (this.isLocked) {
            this.camera.position.copy(this.lockedPosition);
            this.controls.target.copy(this.lockedTarget);
        } else {
            // FORCER la stabilité - pas de mouvement automatique
            // Le target reste fixe sauf interaction utilisateur
            if (!this.controls.isUserInteracting) {
                // Maintenir le target stable si l'utilisateur n'interagit pas
                // this.controls.target.set(0, 6, 0); // Commenté pour éviter de forcer
            }
        }
        
        // Mettre à jour environnement
        if (this.environment && this.environment.update) {
            this.environment.update(this.elapsedTime, this.camera);
        }

        // NOUVEAU: Mettre à jour le contrôleur intégré
        if (this.integratedController) {
            this.integratedController.update(delta);
            
            // Synchroniser les drones du contrôleur intégré avec this.drones pour la compatibilité
            if (this.integratedController.drones && this.integratedController.drones.length > 0) {
                this.drones = this.integratedController.drones;
                // Mettre à jour window.drones pour que le système de collision puisse les trouver
                window.drones = this.drones;
                // Deferred ROS subscriptions: register per-drone topics once drones exist
                if (this._subscribeDronesToRos) this._subscribeDronesToRos();

                // ── Update Mission Control UI panel ──
                // Update drone count display
                const droneCountEl = document.getElementById('drone_count');
                if (droneCountEl) {
                    const flyingCount = this.drones.filter(d => d.state && d.state !== 'IDLE').length;
                    droneCountEl.textContent = `${flyingCount}/${this.drones.length}`;
                }
                // Update intelligence/emergence from backend metrics
                const intelEl = document.getElementById('total_intelligence_display');
                const emergeEl = document.getElementById('emergence_display');
                if (intelEl && this.integratedController.metrics) {
                    intelEl.textContent = (this.integratedController.metrics.collaborationEfficiency * 100 || 0).toFixed(0);
                }
                if (emergeEl && this.integratedController.metrics) {
                    emergeEl.textContent = (this.integratedController.metrics.emergenceLevel * 100 || 0).toFixed(0);
                }
                
                // Mettre à jour la minimap d'exploration avec les positions des drones
                if (window.DIAMANTS_MINIMAP) {
                    this.drones.forEach((drone, idx) => {
                        if (drone.position) {
                            window.DIAMANTS_MINIMAP.updateDronePosition(idx, drone.position);
                        }
                    });
                }
            }
        }
        
        // Mise à jour réaliste des drones (vol, RPM, hélices, exploration)
        // Note: Les drones sont maintenant gérés principalement par l'IntegratedController
        // La boucle de mise à jour individuelle des drones est supprimée pour éviter les conflits

        // Plus de marqueurs à lier - les drones sont visibles par eux-mêmes

        // AUTOFOCUS COMPLÈTEMENT DÉSACTIVÉ - PAS DE MOUVEMENT AUTOMATIQUE
        // L'utilisateur contrôle manuellement la caméra avec la souris
        // SAUF si autoFollow est activé manuellement par le bouton Follow
        if (this.autoFollow && this.drones.length > 0 && this.followDroneIndex >= 0) {
            const drone = this.drones[this.followDroneIndex];
            if (drone && drone.position) {
                const target = drone.mesh ? drone.mesh.position : drone.position;
                // Suivre le drone individuel (interpolation douce)
                this.controls.target.lerp(target, 0.08);
                
                // Rapprocher la caméra du drone (distance ~5m, légèrement au-dessus)
                const idealOffset = new THREE.Vector3(3, 2.5, 3);
                const idealPos = target.clone().add(idealOffset);
                this.camera.position.lerp(idealPos, 0.05);
                
                this.controls.update();
            }
        }
        
        // Rendu
    if (!this.renderer || this._contextLost) {
            // Skip rendering while context is lost/unavailable
        } else if (this.usePostProcessing && this.composer) {
            try { this.composer.render(); } catch (e) { /* ignore transient context errors */ }
        } else {
            try { this.renderer.render(this.scene, this.camera); } catch (e) { /* ignore transient context errors */ }
        }

        // Positions are computed locally by the PID flight engine.
    }

    computeFleetBounds() {
        // Calcule le centre et le rayon de la flotte de drones pour l'autofocus
        if (this.drones.length === 0) {
            return { center: new THREE.Vector3(0, 0.5, 0), radius: 5 };
        }

        const bounds = new THREE.Box3();
        this.drones.forEach(drone => {
            if (drone.position) {
                bounds.expandByPoint(new THREE.Vector3(drone.position.x, drone.position.y, drone.position.z));
            }
        });

        const center = new THREE.Vector3();
        bounds.getCenter(center);
        const radius = Math.max(bounds.min.distanceTo(bounds.max) / 2, 8);

        return { center, radius };
    }

    lockCamera() {
        // Verrouille la caméra en position actuelle
        this.isLocked = true;
        this.lockedPosition = this.camera.position.clone();
        this.lockedTarget = this.controls.target.clone();
        log('🔒 Caméra VERROUILLÉE en position actuelle');
    }

    unlockCamera() {
        // Déverrouille la caméra
        this.isLocked = false;
        log('🔓 Caméra DÉVERROUILLÉE - contrôles libres');
    }

    dispose() {
        // Nettoyage complet
        if (this.environment) {
            this.environment.dispose();
        }
        
        if (this.composer) {
            this.composer.dispose();
        }
        
        // Annuler la boucle d'animation si active
        try {
            if (this._rafId) {
                cancelAnimationFrame(this._rafId);
                this._rafId = null;
            }
        } catch (_) {}

        // Retirer les écouteurs globaux
        try {
            if (this._onResize) window.removeEventListener('resize', this._onResize);
            if (this._onKeyDown) window.removeEventListener('keydown', this._onKeyDown);
        } catch (_) {}

        // Libérer le contexte WebGL de manière agressive pour éviter les blocages lors des rechargements/HMR
        if (this.renderer) {
            try {
                // Détacher les listeners de contexte
                if (this._onWebglContextLost) this.renderer.domElement.removeEventListener('webglcontextlost', this._onWebglContextLost, false);
                if (this._onWebglContextRestored) this.renderer.domElement.removeEventListener('webglcontextrestored', this._onWebglContextRestored, false);
            } catch (_) {}

            try { this.renderer.dispose(); } catch (_) {}
            try { this.renderer.forceContextLoss && this.renderer.forceContextLoss(); } catch (_) {}
            try {
                const ext = this.renderer.getContext && this.renderer.getContext().getExtension && this.renderer.getContext().getExtension('WEBGL_lose_context');
                if (ext && ext.loseContext) ext.loseContext();
            } catch (_) {}
            try { this.renderer.domElement && this.renderer.domElement.remove(); } catch (_) {}
            this.renderer = null;
        }
        
        log('🧹 DIAMANTS Mission System nettoyé');
    }
}

// Initialisation quand le DOM est prêt
document.addEventListener('DOMContentLoaded', () => {
    try {
        // Avoid duplicate systems (e.g., during HMR) that could create multiple WebGL contexts
        if (window.diamantsSystem && typeof window.diamantsSystem.dispose === 'function') {
            try { window.diamantsSystem.dispose(); } catch (_) {}
        }
        window.diamantsSystem = new DiamantsMissionSystem();
        log('🎯 DIAMANTS Mission System V3 démarré avec EZ-Tree authentique');
    } catch (error) {
        console.error('💥 Erreur fatale lors du démarrage:', error);
    }
});

// ══════════════════════════════════════════════════════════════════════════
// NOTE: Les fonctions window.launchMission, window.emergencyLand, 
// window.resetSwarm, window.toggleDebugPanels sont définies dans
// panel-controller.js via exposeGlobalFunctions() avec DoctrineManager intégré.
// NE PAS les redéfinir ici car cela écraserait la version complète.
// ══════════════════════════════════════════════════════════════════════════

window.toggleDebugLogs = function() {
    const logsPanel = document.getElementById('debug-logs');
    const btn = document.getElementById('btn-toggle-logs');
    if (logsPanel) {
        const isVisible = logsPanel.style.display !== 'none';
        logsPanel.style.display = isVisible ? 'none' : 'block';
        btn.textContent = isVisible ? '📋 Show Logs' : '📋 Hide Logs';
    }
};

window.clearDebugLogs = function() {
    const logsContent = document.getElementById('debug-logs-content');
    if (logsContent) {
        logsContent.innerHTML = '';
    }
    log('🧹 Logs nettoyés');
};

// 🔲 Fonction pour activer/désactiver la visualisation des collisions
window.toggleCollisionDebug = function() {
    if (window.environment && window.environment.collisionDetection) {
        const collision = window.environment.collisionDetection;
        const isDebugEnabled = collision.toggleDebugMode();
        const btn = document.getElementById('btn-collision-debug');
        
        if (btn) {
            btn.textContent = isDebugEnabled ? '🔲 Hide Collisions' : '🔲 Show Collisions';
        }
        
        log(isDebugEnabled ? 
            '🔲 Visualisation des collisions activée' : 
            '🔲 Visualisation des collisions désactivée'
        );
    } else {
        log('❌ Système de collision non disponible');
    }
};

// window.toggleDebugPanels définie dans panel-controller.js

// Nettoyage avant fermeture
window.addEventListener('beforeunload', () => {
    if (window.diamantsSystem) {
        window.diamantsSystem.dispose();
    }
});

// Cache busting comments
// Sat Sep 13 03:25:27 PM CEST 2025: Debug collision update
// Sat Sep 13 03:26:48 PM CEST 2025: Emergency altitude fix
// Sat Sep 13 03:40:57 PM CEST 2025: PID system implementation completed
// Sat Sep 13 03:42:33 PM CEST 2025: PID system syntax fix
// Sat Sep 13 03:43:28 PM CEST 2025: Syntax error fixed
// Sat Sep 13 03:54:01 PM CEST 2025: Système de vol simple et stable
// Sat Sep 13 03:56:54 PM CEST 2025: Collision plateforme et bounding boxes corrigées
// Sat Sep 13 03:57:29 PM CEST 2025: Zones de sécurité rouges et collision plateforme
// Sat Sep 13 04:00:26 PM CEST 2025: DEBUG - Correction décollage et bounding boxes
// Sat Sep 13 04:09:42 PM CEST 2025: CORRECTION PATTERN - Décollage forcé 10m
