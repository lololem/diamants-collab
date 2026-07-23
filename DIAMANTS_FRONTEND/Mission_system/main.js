/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
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
        // Fast path: check first arg type before expensive serialization
        const first = args[0];
        if (typeof first === 'string') {
            if (first.includes('DAE') || first.includes('THREE') || first.includes('ERROR') || 
                first.includes('ERREUR') || first.includes('✅') || first.includes('❌') ||
                first.includes('Moteur') || first.includes('hélice') || first.includes('propeller') ||
                first.includes('[STATE-MACHINE]') || first.includes('[FLEET]') || first.includes('[DRONE-LOG]')) {
                originalLog(...args);
            }
            return; // String first arg but no match → skip without serializing rest
        }
        // Non-string first arg: serialize only if needed
        let text = '';
        try { text = args.map(String).join(' '); } catch { originalLog(...args); return; }
        if (text.includes('DAE') || text.includes('THREE') || text.includes('ERROR') || 
            text.includes('ERREUR') || text.includes('✅') || text.includes('❌') ||
            text.includes('Moteur') || text.includes('hélice') || text.includes('propeller') ||
            text.includes('[STATE-MACHINE]') || text.includes('[FLEET]') || text.includes('[DRONE-LOG]')) {
            originalLog(...args);
        }
    };
    
    console.info = (...args) => {
        const first = args[0];
        if (typeof first === 'string') {
            if (first.includes('DAE') || first.includes('THREE') || first.includes('ERROR')) {
                originalInfo(...args);
            }
            return;
        }
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

import { TerrainEnvironment } from './environment/terrain-environment.js';
import { shaderQualityManager } from './shaders/shader-quality-manager.js';
import { AuthenticCrazyflie } from './drones/authentic-crazyflie.js';
import { RosWebBridge, makeOdometry, yawToQuat } from './net/ros-bridge-simple.js';
import { IntegratedDiamantsController } from './tools/integrated-controller.js';
import { OrchestrationConsole } from './ui/orchestration-console.js';
import { OptimalSearchUI } from './ui/optimal-search-ui.js';
import { EnvironmentVoxelizer, CellState } from './intelligence/environment-voxelizer.js';
import { createPathfinderFromVoxelGrid } from './intelligence/drone-pathfinder.js';
import { MetricsUIConnector } from './tools/metrics-ui-connector.js';
import { BenchmarkRunner, runQuickBenchmark } from './tools/benchmark-runner.js';
import { BenchmarkChartGenerator } from './tools/benchmark-charts.js';
import { initDiamantsUI } from './ui/diamants-ui-controller.js';
import { ExplorationMinimap } from './ui/exploration-minimap.js';
import { PerceptionMinimap } from './ui/perception-minimap.js';
import { DiscoveryMinimap } from './ui/discovery-minimap.js';
import { SitacMinimap } from './ui/sitac-minimap.js';
import { initPanelController } from './ui/panel-controller.js';
import { initDoctrineManager } from './missions/mission-doctrine.js';
import { loadFleetConfig } from './core/config.js';
import { CasController } from './core/cas-controller.js';
import { FractalHypervision } from './core/fractal-hypervision.js';
import { FractalHypervisionPanel } from './ui/fractal-hypervision-panel.js';
import { SwarmCoordinationPanel } from './ui/swarm-coordination-panel.js';
import { FederatedLearningMinimap } from './ui/federated-learning-minimap.js';
import { SwarmZoneOverlay } from './ui/swarm-zone-overlay.js';
import { MARLTrainingPanel } from './ui/marl-training-panel.js';
import { CommPanel } from './ui/comm-panel.js';
import { LLMChatPanel } from './ui/llm-chat-panel.js';
import { FollowDroneFab } from './ui/follow-drone-fab.js';
import { DroneSelectPanel } from './ui/drone-select-panel.js';
import { DroneVisualFactory } from './drones/drone-visual-factory.js';
import { ViewPersistence } from './services/view-persistence.js';

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
        this.voxelizer = null; // Système de voxelisation 3D pour pathfinding Dijkstra
        this.pathfinder = null; // Pathfinder Dijkstra pour navigation
        this.benchmark = null; // Runner benchmark
        this.explorationMinimap = null; // Minimap d'exploration
        this.perceptionMinimap = null; // Minimap de perception SLAM
        this.discoveryMinimap = null;  // Minimap de découverte (pixel coverage)
        this.sitacMinimap = null;      // Minimap SITAC (détection d'objets)
        // Backend-less WebSim can publish/sub ROS topics via rosbridge
        this.roswebEnabled = true; // toggle to enable ROS-in-the-browser
        this.ros = null;
        // Visual safety defaults: clearer visuals, full manual control
        this.autoFocus = false; // off by default to avoid camera auto-move
        this.usePostProcessing = false; // no bloom/glow post-processing
        this.visualSafeMode = true; // no fog, no tone mapping halos
        this._focusSmoothing = { center: null, radius: 10 }; // center sera initialisé plus tard
        this.clock = null; // clock sera initialisé plus tard
        
        // Orchestration console — DISABLED (removed from UI)
        this.orchestrationConsole = null;
        
        // Swarm RL Coordination Panel — [B] to toggle
        this.swarmCoordinationPanel = new SwarmCoordinationPanel();
        
        // MARL Training Panel — [T] to toggle
        this.marlTrainingPanel = new MARLTrainingPanel();
        
        // LLM Chat Panel — [C] to toggle
        this.llmChatPanel = new LLMChatPanel();
        
        // Follow Drone FAB — bottom-right floating button
        this.followDroneFab = new FollowDroneFab();
        
        // CAS / FPS HUD
        this.casController = null;
        this.fractalHypervision = null;
        this.fractalPanel = null;
        
        // Drone Select Panel — tap on drone to see info
        this.droneSelectPanel = new DroneSelectPanel();
        
        // Optimal Search UI — Ctrl+K to open
        this.optimalSearchUI = new OptimalSearchUI({
            onConfigSelect: (config) => {
                log(`🎯 Configuration optimale appliquée: ${config.doctrine.name} + ${config.coa.name}`);
                // orchestrationConsole removed
            }
        });

        log('🚀 Initialisation DIAMANTS Mission System V3 avec EZ-Tree');
        this.init();
    }

    async init() {
        // Attendre que THREE.js soit prêt
        log('⏳ Attente de THREE.js...');
        const threeReady = await initializeTHREE();
        if (!threeReady) {
            const msg = 'THREE.js non initialisé (timeout ou erreur bootstrap)';
            console.error('🛑 Impossible d\'initialiser THREE.js. Arrêt.');
            showFatalInitError('Initialisation 3D échouée', msg);
            return;
        }
        
        // Maintenant que THREE.js est disponible, initialiser les objets THREE
        this._focusSmoothing.center = new THREE.Vector3();
        this.clock = new THREE.Clock();
        
        await this.setupRenderer();
        // If renderer couldn't be created (WebGL blocked/unavailable), stop early
        if (!this.renderer) {
            const msg = 'WebGL indisponible — vérifiez pilote GPU ou essayez un autre navigateur';
            console.error('🛑 WebGL renderer unavailable. Aborting further initialization.');
            showFatalInitError('Rendu WebGL impossible', msg);
            return;
        }
        await this.setupScene();
        await this.setupCamera();
        await this.setupControls();
        await this.setupPostProcessing();
        
        // Load fleet config + environment in parallel (env uses fallback droneCount if config not ready)
        // Also start preloading drone 3D models (DAE/STL/GLB) in parallel to avoid sequential delay
        const fleetPromise = loadFleetConfig('http://localhost:8000');
        const dronePreloadPromise = DroneVisualFactory.preloadModels().catch(e => {
            warn('⚠️ Early drone model preload failed (will retry):', e.message);
        });
        await this.setupEnvironment();
        // Ensure fleet config is settled (should be done by now with 2s timeout)
        await fleetPromise.catch(() => {});
        // Ensure drone models are ready before creating controller
        await dronePreloadPromise;
        await this.setupIntegratedController();
        if (this.roswebEnabled) await this.setupRosWeb();
        
        this.casController = new CasController(this);
        this.casController.mount();

        this.fractalHypervision = new FractalHypervision(this);
        this.fractalHypervision.mount();
        this.fractalPanel = new FractalHypervisionPanel(this.fractalHypervision);
        this.fractalPanel.mount();
        
        this.setupEventListeners();
        this.animate();

        // View persistence — restore saved UI state (minimaps, panels, sidebar)
        this.viewPersistence = new ViewPersistence();
        // Defer init so DOM-dependent inline scripts (initMinimapDetach) run first
        setTimeout(() => {
            this.viewPersistence.init();
            window.DIAMANTS = window.DIAMANTS || {};
            window.DIAMANTS.viewPersistence = this.viewPersistence;
        }, 600);
        
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
        
        // Strategy 1: High-quality approach
        const safeAttribs = {
            alpha: false,
            depth: true,
            stencil: false,
            antialias: true,
            desynchronized: false, // false to avoid tearing/flickering artifacts
            preserveDrawingBuffer: false,
            premultipliedAlpha: false,
            powerPreference: 'high-performance',
            failIfMajorPerformanceCaveat: false
        };

        // Strategy 2: Minimal approach
        const minimalAttribs = {
            alpha: false,
            antialias: true,
            powerPreference: 'high-performance'
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
                antialias: true, 
                powerPreference: 'high-performance',
                // precision removed — let Three.js auto-detect (Firefox mediump fallback)
            });
            log('✅ THREE.js WebGL renderer created successfully');
            // ── DIAGNOSTIC GPU ── quel renderer sert réellement ? Si "SwiftShader",
            // "llvmpipe" ou "Software" apparaît => rendu LOGICIEL (pas le GPU) et le
            // FPS sera bas quoi qu'on optimise (failIfMajorPerformanceCaveat:false
            // laisse Chrome retomber silencieusement sur le CPU).
            try {
                const _dbg = gl.getExtension('WEBGL_debug_renderer_info');
                const _vendor = _dbg ? gl.getParameter(_dbg.UNMASKED_VENDOR_WEBGL) : 'n/a';
                const _rend = _dbg ? gl.getParameter(_dbg.UNMASKED_RENDERER_WEBGL) : 'n/a';
                const _soft = /swiftshader|llvmpipe|software|basic render/i.test(String(_rend));
                console.log(`%c[GPU] ${_soft ? '⚠️ RENDU LOGICIEL (pas le GPU !)' : '✅ accéléré matériellement'}\n  vendor   = ${_vendor}\n  renderer = ${_rend}`,
                    `font-weight:bold;color:${_soft ? '#c00' : '#0a0'}`);
                window.__DIAMANTS_GPU = { vendor: _vendor, renderer: _rend, software: _soft };
            } catch (_) { /* extension indisponible */ }
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
        // Use native pixel ratio for crisp rendering (capped at 2 to avoid VRAM issues on 4K displays)
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(1); // cap fill-rate (HiDPI = 4x pixels = FPS killer)

        // Enable shadows for better visual quality
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

        // Tone mapping: ACES for cinematic lighting
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.6;

        // Gamma correction
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;

        // Attach and handle context loss/restoration to avoid the browser blocking the page
    const onContextLost = (e) => {
            try { e.preventDefault(); } catch (_) {}
            warn('⚠️ WebGL context LOST - pausing rendering to avoid browser blocking');
            this._contextLost = true;
        };
    const onContextRestored = () => {
            console.info('✅ WebGL context RESTORED - rebuilding renderer');
            this._contextLost = false;
            try {
                const w = Math.max(1, container.clientWidth || window.innerWidth);
                const h = Math.max(1, container.clientHeight || window.innerHeight);
                this.renderer.setSize(w, h);
                // Force re-compile all materials after context restore
                if (this.scene) {
                    this.scene.traverse((obj) => {
                        if (obj.material) {
                            const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
                            mats.forEach(m => { m.needsUpdate = true; });
                        }
                        if (obj.geometry) obj.geometry.dispose();
                    });
                }
                // Force a render frame to push everything back to GPU
                if (this.renderer && this.scene && this.camera) {
                    this.renderer.render(this.scene, this.camera);
                }
                console.info('✅ WebGL context recovery complete');
            } catch (err) {
                console.error('❌ WebGL recovery failed — reloading page in 2s', err);
                setTimeout(() => window.location.reload(), 2000);
            }
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
            background: rgba(0,0,0,0.95); color: #fff; z-index: 9000; font-family: system-ui, sans-serif;
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
        try {
            this.ros = new RosWebBridge({ silent: window.SILENT_MODE });
            await this.ros.connect().catch(() => {});
            
            log(`🔌 ROS2 WebSocket: ${this.ros.connected ? 'Connected' : 'Offline'} — ${this.ros.url}`);
            
            // ── ID mapper ────────────────────────────────────────────
            // Backend IDs (crazyflie_01, 1-indexed) ↔ Frontend IDs (crazyflie_0, 0-indexed)
            this._findLocalDrone = (backendId) => {
                if (!this.drones?.length) return null;
                // 1) Exact match
                let d = this.drones.find(dr => dr.id === backendId);
                if (d) return d;
                const m = backendId.match(/^(.+?)_0*(\d+)$/);
                if (m) {
                    const num = parseInt(m[2], 10);
                    // 2) Backend 1-indexed → frontend 0-indexed (priority!)
                    d = this.drones.find(dr => dr.id === `${m[1]}_${num - 1}`);
                    if (d) return d;
                    // 3) Strip leading zeros only: crazyflie_01 → crazyflie_1
                    d = this.drones.find(dr => dr.id === `${m[1]}_${m[2]}`);
                    if (d) return d;
                }
                // 4) Positional fallback
                const idx = parseInt(backendId.replace(/\D/g, ''), 10);
                if (!isNaN(idx) && idx > 0 && idx <= this.drones.length) return this.drones[idx - 1];
                return null;
            };

            // ── Backend-driven position handler ──────────────────────
            // Listen for the global CustomEvent emitted by RosWebBridge
            // whenever it receives drone positions from the backend.
            // This works regardless of whether per-drone subs are set up.
            this._backendPosCount = 0;
            this._lastBackendLog = 0;
            window.addEventListener('diamants:drone-positions', (evt) => {
                const drones = evt.detail;
                if (!drones || typeof drones !== 'object') return;
                let matched = 0;
                for (const [backendId, info] of Object.entries(drones)) {
                    // Skip positions emitted by the frontend engine — these are for
                    // the orchestration console only, NOT for overriding mesh positions.
                    // Only real backend/ROS data should drive CAS 1 (backend mode).
                    if (info?.source === 'engine') continue;
                    // Accept both nested {position:{x,y,z}} and flat {x,y,z}
                    const pos = info?.position || (info?.x !== undefined ? { x: info.x, y: info.y, z: info.z || 0 } : null);
                    if (!pos) continue;
                    const drone = this._findLocalDrone(backendId);
                    if (!drone) continue;
                    matched++;
                    try {
                        // Set the target for smooth lerp interpolation
                        drone.targetPosition.set(pos.x, pos.y, pos.z);
                        if (drone.state === 'IDLE') drone.state = 'FLYING';

                        // Tag with rosData for UI/debugging
                        if (!drone.rosData) drone.rosData = {};
                        drone.rosData.lastUpdate = Date.now();
                        drone.rosData.position = { x: pos.x, y: pos.y, z: pos.z };
                        drone.rosData.source = info.source || 'backend';
                        // Forward backend battery & status
                        if (info.battery !== undefined) drone.rosData.battery = info.battery;
                        if (info.status) drone.rosData.status = info.status;
                        if (info.mode) drone.rosData.mode = info.mode;
                        if (info.sysid !== undefined) drone.rosData.sysid = info.sysid;
                        if (info.armed !== undefined) drone.rosData.armed = info.armed;
                        if (info.engine) drone.rosData.engine = info.engine;
                    } catch (_) { /* safe */ }
                }
                this._backendPosCount++;
                const now = Date.now();
                if (now - this._lastBackendLog > 5000) {
                    this._lastBackendLog = now;
                    // Only log when there are actual matches, or on first emission
                    if (matched > 0 || this._backendPosCount <= 1) {
                        console.debug(`📡 Backend positions: ${this._backendPosCount} updates, ${matched}/${Object.keys(drones).length} drones matched`);
                    }
                }
            });
            
            // ── Backend propeller speeds handler ────────────────────
            window.addEventListener('diamants:propeller-speeds', (evt) => {
                const data = evt.detail;
                if (!data || typeof data !== 'object') return;
                for (const [droneId, speeds] of Object.entries(data)) {
                    const drone = this._findLocalDrone(droneId);
                    if (!drone) continue;
                    if (!drone.rosData) drone.rosData = {};
                    drone.rosData.propeller_speeds = speeds; // [rpm1, rpm2, rpm3, rpm4]
                    drone.rosData.lastUpdate = Date.now();
                }
            });

            // ── Autonomy level change → forward to WebSocket bridge + engine ──
            window.addEventListener('diamants:autonomy-change', (evt) => {
                const { level, mode } = evt.detail || {};
                if (this.ros?.connected) {
                    this.ros._send({
                        type: 'set_parameter',
                        data: { key: 'autonomy_level', value: level, mode }
                    });
                }
                // Store on system for integratedController access
                if (this.integratedController) {
                    this.integratedController.autonomyLevel = level;
                    // Propagate directly to flight engine for immediate effect
                    if (this.integratedController.autonomousFlightEngine) {
                        this.integratedController.autonomousFlightEngine.setAutonomyLevel(level);
                    }
                    // Propagate to multi-agent coordinator (metrics/training only — not waypoint driver)
                    if (this.integratedController.multiAgentCoordinator) {
                        this.integratedController.multiAgentCoordinator.setAutonomyLevel(level);
                    }
                }
            });

            // ── Deferred per-drone subscriptions ─────────────────────
            this._rosSubscribedDrones = new Set();
            this._subscribeDronesToRos = () => {
                if (!this.ros || !this.drones?.length) return;
                this.drones.forEach((d) => {
                    if (this._rosSubscribedDrones.has(d.id)) return;
                    this._rosSubscribedDrones.add(d.id);
                    const ns = `/${d.id}`;
                    log(`📡 ROS2 subscriptions for ${d.id}`);
                    this.ros.subscribe(`${ns}/cmd/target_pose`, 'geometry_msgs/PoseStamped', (msg) => {
                        try {
                            const p = msg.pose.position;
                            d.targetPosition.set(p.x, p.y, p.z);
                            d.targetAltitude = p.y;
                            if (d.state === 'IDLE') d.state = 'TAKEOFF';
                        } catch (_) {}
                    });
                    this.ros.subscribe(`${ns}/cmd/land`, 'std_msgs/Empty', () => {
                        try { d.state = 'LANDING'; } catch (_) {}
                    });
                    this.ros.subscribe(`${ns}/cmd/takeoff`, 'std_msgs/Float32', (msg) => {
                        try { d.takeoff(msg.data || 2.0); } catch (_) {}
                    });
                });
            };
            this._subscribeDronesToRos();
            
            log('🌐 ROS-Web bridge prêt');
        } catch (e) {
            warn('ROS-Web non disponible:', e.message);
        }
    }

    async setupScene() {
        this.scene = new THREE.Scene();
        this.scene.name = 'DiamantsMissionScene';
        // Lighting setup for PBR drone materials (carbon fiber, metals, plastics)
        try {
            // Hemisphere: sky white above, dark ground below — matches backup values
            const hemi = new THREE.HemisphereLight(0xffffff, 0x334466, 0.6);
            this.scene.add(hemi);

            // Key light: single directional sun — intensity 0.6 (backup value)
            const dir = new THREE.DirectionalLight(0xffffff, 0.6);
            dir.position.set(50, 80, 40);
            // Ombres ON, mais avec une shadow camera DIMENSIONNÉE. Sans ça Three.js
            // utilise un frustum orthographique par défaut de 10×10 unités : on payait
            // la passe d'ombre pour une zone utile de 10 m seulement (mesuré : coût réel
            // noyé dans le coût fixe de présentation, donc autant les rendre utiles).
            dir.castShadow = true;
            const _shadowSpan = 90; // couvre la zone d'exploration, pas 10 m
            dir.shadow.camera.left = -_shadowSpan;
            dir.shadow.camera.right = _shadowSpan;
            dir.shadow.camera.top = _shadowSpan;
            dir.shadow.camera.bottom = -_shadowSpan;
            dir.shadow.camera.near = 1;
            dir.shadow.camera.far = 300;
            dir.shadow.bias = -0.0005;
            dir.shadow.camera.updateProjectionMatrix();
            this.scene.add(dir);
        } catch (_) { /* safe */ }
        log('🎬 Scène créée');
    }

    async setupCamera() {
        const container = this.renderer.domElement.parentElement;
        const aspect = container.clientWidth / container.clientHeight;
        
        this.camera = new THREE.PerspectiveCamera(60, aspect, 0.01, 1000);
        // Vue initiale : vue proche de la piste à hauteur d'homme, entre les arbres
        this.camera.position.set(18, 4, 22);
        this.camera.lookAt(0, 1, 0);
        
        // Disable camera restore — always start with clean view
        this._cameraRestorePending = false;
        this._lastCameraSave = 0;
        // Clear any old saved camera state that might be in the trees
        try { localStorage.removeItem('diamants_camera'); } catch(_) {}
        
        log('📷 Caméra configurée');
    }

    /** Save camera position + target to localStorage */
    _saveCameraState() {
        if (!this.camera || !this.controls) return;
        const p = this.camera.position;
        const t = this.controls.target;
        try {
            localStorage.setItem('diamants_camera', JSON.stringify({
                px: p.x, py: p.y, pz: p.z,
                tx: t.x, ty: t.y, tz: t.z
            }));
        } catch (_) { /* quota exceeded — ignore */ }
    }

    /** Load camera state from localStorage */
    _loadCameraState() {
        try {
            const raw = localStorage.getItem('diamants_camera');
            if (!raw) return null;
            const s = JSON.parse(raw);
            if (typeof s.px === 'number' && typeof s.tx === 'number') return s;
        } catch (_) { /* corrupted — ignore */ }
        return null;
    }

    async setupControls() {
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.12;
        this.controls.enablePan = true;
        this.controls.enableZoom = true;
        this.controls.zoomSpeed = 0.8;
        this.controls.enableRotate = true;
        this.controls.touches = { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN };

        // Auto-exit follow mode & auto-rotate when user interacts with OrbitControls (drag/rotate)
        this.controls.addEventListener('start', () => {
            if (this.autoFollow) {
                this.autoFollow = false;
                this.followDroneIndex = -1;
                log('📷 Follow mode désactivé (interaction souris)');
            }
            if (this.controls.autoRotate) {
                this.controls.autoRotate = false;
                this.controls.autoRotateSpeed = 0;
                const btn = document.getElementById('auto-rotate-btn');
                if (btn) {
                    btn.style.background = 'rgba(0, 15, 35, 0.88)';
                    btn.style.color = 'rgba(0, 210, 255, 0.7)';
                    btn.style.borderColor = 'rgba(0, 200, 255, 0.35)';
                }
            }
        });
        
        this.controls.rotateSpeed = 0.6;
        this.controls.panSpeed = 1.2;
        this.controls.screenSpacePanning = false; // Pan sur le plan du sol (XZ), pas en screen-space

        // Supprimer le menu contextuel sur le canvas pour ne pas interrompre le pan clic droit
        this.renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());
        
        // ── Zoom scroll normalisé par frame ──
        // Chrome Linux smooth-scroll envoie 10-50 events par tick molette.
        // On accumule le deltaY brut et on applique un zoom proportionnel dans animate().
        this._zoomAccum = 0;
        this.renderer.domElement.addEventListener('wheel', (e) => {
            e.preventDefault();
            // Normaliser: deltaMode 1 (lignes) → ×40, deltaMode 2 (pages) → ×800
            let dy = e.deltaY;
            if (e.deltaMode === 1) dy *= 40;
            else if (e.deltaMode === 2) dy *= 800;
            this._zoomAccum += dy;
            // Exit follow mode on scroll
            if (this.autoFollow) {
                this.autoFollow = false;
                this.followDroneIndex = -1;
            }
        }, { passive: false });

        // Exit follow mode on touch interaction
        this.renderer.domElement.addEventListener('touchstart', (e) => {
            if (this.autoFollow && e.touches.length >= 1) {
                this.autoFollow = false;
                this.followDroneIndex = -1;
            }
        }, { passive: true });
        
        // Limites pour la caméra (zone 120×120m)
        this.controls.minDistance = 3;
        this.controls.maxDistance = 80;
        this.controls.minPolarAngle = Math.PI / 8;       // ~22.5° from top
        this.controls.maxPolarAngle = Math.PI / 2 - 0.05; // ~87° — NEVER below horizon
        
        // Centrer les contrôles sur la piste (à hauteur des drones)
        this.controls.target.set(0, 1, 0);
        
        // DÉSACTIVER toute animation automatique
        this.controls.autoRotate = false;
        this.controls.autoRotateSpeed = 0;
        
        this.controls.update();
        
        log('🎮 Contrôles configurés (zoom throttlé 5%/frame)');
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
        log('🌍 Initialisation de l\'environnement 3D...');

        // Auto-detect optimal quality based on GPU + device before creating grass
        let detectedQuality = 'medium';
        try {
            detectedQuality = await shaderQualityManager.autoDetectOnInit(this.renderer);
            log(`🎯 Qualité graphique auto-détectée: ${detectedQuality}`);
        } catch (e) {
            warn('⚠️ Auto-détection qualité échouée:', e);
        }

        // ── Adapt renderer settings to detected quality ──
        const qualityProfiles = {
            low:    { pixelRatio: 1, shadows: false, shadowMapSize: 512,  maxTrees: 35, grassCount: 15000, leavesCount: 500,  exposure: 1.6 },
            medium: { pixelRatio: 1, shadows: true,  shadowMapSize: 1024, maxTrees: 60, grassCount: 35000, leavesCount: 1200, exposure: 1.6 },
            high:   { pixelRatio: 1, shadows: true, shadowMapSize: 1536, maxTrees: 85, grassCount: 60000, leavesCount: 2000, exposure: 1.6 },
            ultra:  { pixelRatio: 1, shadows: true, shadowMapSize: 2048, maxTrees: 110, grassCount: 90000, leavesCount: 3000, exposure: 1.6 },
        };
        const qp = qualityProfiles[detectedQuality] || qualityProfiles.medium;
        if (!this.renderer) {
            showFatalInitError('Rendu WebGL perdu', 'Le renderer a été libéré avant setupEnvironment.');
            return;
        }
        this.renderer.setPixelRatio(qp.pixelRatio);
        this.renderer.shadowMap.enabled = qp.shadows;
        this.renderer.toneMappingExposure = qp.exposure;
        log(`🖥️ Profil rendu appliqué (${detectedQuality}): shadows=${qp.shadows}, pixelRatio=${qp.pixelRatio}, trees=${qp.maxTrees}`);

        // Afficher un indicateur de chargement
        this.showLoadingIndicator();
        
        try {
            this.environment = new TerrainEnvironment(this.scene, {
                terrainSize: { x: 200, y: 200 },
                forestDensity: 0.4,
                maxTrees: qp.maxTrees,
                minTreeSpacing: 11.0,
                enableSkybox: true,
                enableTerrain: true,
                enableForest: true,          // Forest AROUND the arena
                enableFallenLeaves: qp.shadows,
                enableForestWood: qp.shadows,
                enableUndergrowth: qp.shadows,
                grassCount: qp.grassCount,
                leavesCount: qp.leavesCount,
                shadowMapSize: qp.shadowMapSize,
                droneCount: window.FLEET_CONFIG?.numDrones || 8,
            });
            
            // NOUVEAU: Exposer l'environnement globalement pour le système de collision
            window.environment = this.environment;
            log('✅ Environnement exposé globalement pour système de collision');
            
            // ── PERSPECTIVE ATMOSPHÉRIQUE ──
            // Sans fog, le lointain garde la saturation du premier plan : l'œil lit un
            // aplat vert sans profondeur. Le fog linéaire fait fondre progressivement
            // le terrain et les arbres vers l'horizon => vraie sensation de distance.
            // near/far larges : aucun effet sur l'arène, l'estompage démarre au loin.
            //
            // La teinte était un bleu ciel (0x9fc3dd), réglée d'après un ciel 0x87CEEB
            // que ce projet ne rend jamais : le skybox a un horizon CHAUD
            // (skyColorLow [1.0, 0.8, 0.6]) sous un voile [0.9, 0.85, 0.8]. Le fog
            // bleuissait donc la ligne d'arbres à contre-emploi du ciel derrière elle.
            // Sage clair désaturé : s'accorde à la fois au voile chaud du skybox et au
            // fog du champ d'herbe (#aebb92), sans virer au blanc qui délave tout.
            this.scene.fog = new THREE.Fog(0xbcc4a8, 55, 340);
            // Ensuite, déployer une petite flotte de Crazyflie bien visibles
            this.setupDrones();
            // Exposer un pont de compatibilité global pour l'UI (window.DIAMANTS)
            this.exposeGlobalBridge();
            
            // Initialiser les minimaps
            try {
                this.explorationMinimap = new ExplorationMinimap('minimap_canvas');
                log('🗺️ ExplorationMinimap initialisée');
            } catch (e) {
                warn('⚠️ Erreur init ExplorationMinimap:', e);
            }
            try {
                this.perceptionMinimap = new PerceptionMinimap('perception_canvas');
                log('🔬 PerceptionMinimap (SLAM) initialisée');
            } catch (e) {
                warn('⚠️ Erreur init PerceptionMinimap:', e);
            }
            try {
                this.discoveryMinimap = new DiscoveryMinimap('discovery_canvas');
                log('🟩 DiscoveryMinimap (pixel coverage) initialisée');
            } catch (e) {
                warn('⚠️ Erreur init DiscoveryMinimap:', e);
            }
            try {
                this.sitacMinimap = new SitacMinimap('sitac_canvas');
                log('🎯 SitacMinimap (détection objets) initialisée');
            } catch (e) {
                warn('⚠️ Erreur init SitacMinimap:', e);
            }
            try {
                this.federatedMinimap = new FederatedLearningMinimap('federated_canvas');
                log('🧠 FederatedLearningMinimap initialisée');
            } catch (e) {
                warn('⚠️ Erreur init FederatedLearningMinimap:', e);
            }
            try {
                this.commPanel = new CommPanel('comm_canvas');
                log('📡 CommPanel (communication P2P) initialisé');
            } catch (e) {
                warn('⚠️ Erreur init CommPanel:', e);
            }
            try {
                this.swarmZoneOverlay = new SwarmZoneOverlay(this.scene);
                log('🔮 SwarmZoneOverlay initialisé');
            } catch (e) {
                warn('⚠️ Erreur init SwarmZoneOverlay:', e);
            }

            // Click-to-expand system for minimaps
            this._initMinimapExpander();
            
        } catch (error) {
            console.error('❌ Erreur création environnement:', error);
        } finally {
            this.hideLoadingIndicator();
        }
        
        log('✅ Environnement 3D prêt');
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
                droneCount: window.FLEET_CONFIG?.numDrones || 8,
                maxConcurrentDrones: window.FLEET_CONFIG?.numDrones || 8,
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
                window.DIAMANTS.viewPersistence = this.viewPersistence;
                // Exposer l'accès au MissionManager attendu par l'UI
                window.DIAMANTS.getMissionManager = () => this.integratedController?.missionManager || null;
                // Exposer le contrôleur pour outils debug
                window.DIAMANTS.controller = this.integratedController;
                // Expose flight engine via dynamic getter (engine is created asynchronously
                // inside initializeSystem → createDroneFleet, so a static assignment would
                // capture null). Same pattern as the metrics getter below.
                Object.defineProperty(window.DIAMANTS, 'flightEngine', {
                    get: () => this.integratedController?.autonomousFlightEngine || null,
                    configurable: true,
                    enumerable: true
                });
                // Métriques exposées via defineProperty (getter dynamique) plus bas — pas d'affectation directe ici
                window.DIAMANTS.drones = this.drones;
                // Exposer un arrêt d'urgence unifié
                window.DIAMANTS.emergencyStop = () => {
                    try {
                        this.integratedController?.emergencyStop?.();
                    } catch (_) {}
                };

                // ─── Exploration Algorithm Toggle (Voronoi vs Legacy) ────
                window.DIAMANTS.setAlgorithm = (mode) => {
                    const si = this.integratedController?.autonomousFlightEngine?.swarmIntelligence;
                    if (si?.setAlgorithmMode) {
                        si.setAlgorithmMode(mode);
                        return `Algorithm → ${mode.toUpperCase()}`;
                    }
                    return 'Swarm intelligence not available';
                };
                window.DIAMANTS.getCoverageHistory = () => {
                    const si = this.integratedController?.autonomousFlightEngine?.swarmIntelligence;
                    return si?.getCoverageHistory?.() || null;
                };
                window.DIAMANTS.getSwarmMetrics = () => {
                    const si = this.integratedController?.autonomousFlightEngine?.swarmIntelligence;
                    return si?.getMetrics?.() || null;
                };

                // ── Multi-Agent Trainable System API ──
                window.DIAMANTS.getAgentMetrics = () => {
                    return this.integratedController?.multiAgentCoordinator?.getMetrics?.() || null;
                };
                window.DIAMANTS.exportAgentWeights = () => {
                    return this.integratedController?.multiAgentCoordinator?.exportAllWeights?.() || null;
                };
                window.DIAMANTS.importAgentWeights = (data) => {
                    this.integratedController?.multiAgentCoordinator?.importAllWeights?.(data);
                };
                window.DIAMANTS.setTrainingEnabled = (enabled) => {
                    this.integratedController?.multiAgentCoordinator?.setTrainingEnabled?.(enabled);
                };
                window.DIAMANTS.startNewEpisode = () => {
                    this.integratedController?.multiAgentCoordinator?.startNewEpisode?.();
                };

                // ── MLOps API ──
                const coord = () => this.integratedController?.multiAgentCoordinator;

                // Hot agent injection / removal
                window.DIAMANTS.addAgent = (droneId, config, weights) => {
                    return coord()?.addAgent?.(droneId, config, weights) || null;
                };
                window.DIAMANTS.removeAgent = (droneId) => {
                    return coord()?.removeAgent?.(droneId) || null;
                };
                window.DIAMANTS.getAgentIds = () => {
                    return coord()?.getAgentIds?.() || [];
                };
                window.DIAMANTS.getAgentCount = () => {
                    return coord()?.getAgentCount?.() || 0;
                };

                // Brain swapping
                window.DIAMANTS.swapBrain = (droneId, newBrain, transfer) => {
                    return coord()?.swapAgentBrain?.(droneId, newBrain, transfer) || false;
                };

                // Model registry
                window.DIAMANTS.registerModel = (name, metadata) => {
                    return coord()?.registerModel?.(name, metadata) || null;
                };
                window.DIAMANTS.loadModel = (name) => {
                    return coord()?.loadModel?.(name) || false;
                };
                window.DIAMANTS.listModels = () => {
                    return coord()?.registry?.listModels?.() || [];
                };
                window.DIAMANTS.deleteModel = (name) => {
                    return coord()?.registry?.deleteModel?.(name) || false;
                };

                // Snapshots & rollback
                window.DIAMANTS.createSnapshot = (metadata) => {
                    return coord()?.createSnapshot?.(metadata) || null;
                };
                window.DIAMANTS.rollbackToSnapshot = (index) => {
                    return coord()?.rollbackToSnapshot?.(index) || false;
                };
                window.DIAMANTS.listSnapshots = () => {
                    return coord()?.listSnapshots?.() || [];
                };

                // A/B testing
                window.DIAMANTS.startABTest = (name, modelA, modelB, split) => {
                    return coord()?.startABTest?.(name, modelA, modelB, split) || null;
                };
                window.DIAMANTS.getABTestResults = (name) => {
                    return coord()?.getABTestResults?.(name) || null;
                };

                // Experiment tracking
                window.DIAMANTS.startExperiment = (name, hyperparams) => {
                    return coord()?.startExperiment?.(name, hyperparams) || null;
                };
                window.DIAMANTS.endExperiment = () => {
                    return coord()?.endExperiment?.() || null;
                };
                window.DIAMANTS.listExperiments = () => {
                    return coord()?.listExperiments?.() || [];
                };
                window.DIAMANTS.exportExperiments = (id) => {
                    return coord()?.exportExperiments?.(id) || null;
                };

                // Full registry export/import
                window.DIAMANTS.exportRegistry = () => {
                    return coord()?.registry?.exportRegistry?.() || null;
                };
                window.DIAMANTS.importRegistry = (data, merge) => {
                    coord()?.registry?.importRegistry?.(data, merge);
                };

                // ── Digital Twin: Drone injection / removal ──────────────────
                const ctrl = () => this.integratedController;

                window.DIAMANTS.spawnDrone = (droneId, profileId, position, opts) => {
                    return ctrl()?.spawnDrone?.({ droneId, profileId, position, ...opts }) || null;
                };
                window.DIAMANTS.despawnDrone = (droneId) => {
                    return ctrl()?.despawnDrone?.(droneId) || null;
                };
                window.DIAMANTS.listDroneProfiles = () => {
                    return ctrl()?.listDroneProfiles?.() || [];
                };

                // ── MARL: Fleet composition & agent roles ────────────────────
                window.DIAMANTS.getFleetComposition = () => {
                    return coord()?.getFleetComposition?.() || null;
                };
                window.DIAMANTS.getAgentRole = (droneId) => {
                    const agent = coord()?.agents?.get(droneId);
                    return agent ? {
                        role: agent._agentRole,
                        profileId: agent._profileId,
                        brainType: agent.getBrainType()
                    } : null;
                };
                window.DIAMANTS.recommendFleet = (total, strategy) => {
                    // Lazy import to avoid circular deps at module level
                    try {
                        const { AgentDroneRegistry } = ctrl()?.constructor
                            ? { AgentDroneRegistry: null } : {};
                        // Direct inline recommendation
                        const composition = [];
                        if (strategy === 'swarm') {
                            composition.push({ profileId: 'X500', count: 1, role: 'cognitive' });
                            composition.push({ profileId: 'CRAZYFLIE', count: total - 1, role: 'reactive' });
                        } else if (strategy === 'cognitive-heavy') {
                            const cognitive = Math.ceil(total * 0.6);
                            composition.push({ profileId: 'X500', count: cognitive, role: 'cognitive' });
                            composition.push({ profileId: 'CRAZYFLIE', count: total - cognitive, role: 'reactive' });
                        } else {
                            // balanced
                            const cognitive = Math.max(1, Math.floor(total / 3));
                            composition.push({ profileId: 'X500', count: cognitive, role: 'cognitive' });
                            composition.push({ profileId: 'CRAZYFLIE', count: total - cognitive, role: 'reactive' });
                        }
                        return composition;
                    } catch(_) { return []; }
                };
                window.DIAMANTS.listAgentSpecs = () => {
                    try {
                        // Return available drone-agent specs
                        return [
                            { profileId: 'CRAZYFLIE', role: 'reactive', brainType: 'reactive', teamRole: 'scout', sensors: ['multi-ranger'] },
                            { profileId: 'X500', role: 'cognitive', brainType: 'cognitive', teamRole: 'coordinator', sensors: ['depth-camera'] },
                            { profileId: 'MAVIC', role: 'hybrid', brainType: 'hybrid', teamRole: 'explorer', sensors: ['basic'] },
                            { profileId: 'PHANTOM', role: 'hybrid', brainType: 'hybrid', teamRole: 'observer', sensors: ['basic'] },
                        ];
                    } catch(_) { return []; }
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
                    // Delegate to panel-controller's handler
                    if (window.toggleFollowMode) {
                        window.toggleFollowMode();
                    } else {
                        // Fallback inline toggle
                        this.autoFollow = !this.autoFollow;
                        if (!this.autoFollow) {
                            this.followDroneIndex = -1;
                            try {
                                const { center, radius } = this.computeFleetBounds();
                                const d = Math.max(radius * 2.5, 30);
                                this.camera.position.set(center.x + d * 0.7, center.y + d * 0.5, center.z + d * 0.7);
                                this.controls.target.copy(center);
                                this.controls.update();
                            } catch (_) {}
                        } else {
                            this.followDroneIndex = 0;
                        }
                    }
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

            } catch (e) { console.warn('[DIAMANTS] API bridge init error:', e); }

            // ── LLM Chat & Beacon API (separate try to avoid being swallowed) ──
            try {
                window.DIAMANTS.llmChat = this.llmChatPanel;
                // Live getter — beaconSystem is initialized later via initBeaconSystem()
                Object.defineProperty(window.DIAMANTS, 'beacons', {
                    get: () => this.llmChatPanel?.beaconSystem || null,
                    configurable: true
                });
                window.DIAMANTS.sendMission = (text) => {
                    return this.llmChatPanel?.missionService?.processMessage?.(text) || null;
                };
                window.DIAMANTS.placeBeacon = (x, z, label) => {
                    return this.llmChatPanel?.beaconSystem?.placeBeacon?.(x, z, label) || null;
                };
                window.DIAMANTS.getBeacons = () => {
                    return this.llmChatPanel?.beaconSystem?.beacons || [];
                };
                window.DIAMANTS.clearBeacons = () => {
                    this.llmChatPanel?.beaconSystem?.clearAll?.();
                    this.llmChatPanel?.missionService?.beacons?.clear?.();
                    window.dispatchEvent(new CustomEvent('diamants:beacons-updated'));
                };
                window.DIAMANTS.enterBeaconPlacement = () => {
                    this.llmChatPanel?.beaconSystem?.enterPlacementMode?.();
                };
                window.DIAMANTS.exitBeaconPlacement = () => {
                    this.llmChatPanel?.beaconSystem?.exitPlacementMode?.();
                };
                window.DIAMANTS.placeRandomBeacons = (count = 3) => {
                    const bs = this.llmChatPanel?.beaconSystem;
                    if (!bs) return;
                    for (let i = 0; i < count; i++) {
                        const x = (Math.random() - 0.5) * 60;
                        const z = -5 - Math.random() * 45;
                        bs.placeBeacon(x, z, `Balise-${Date.now()}-${i}`);
                    }
                    window.dispatchEvent(new CustomEvent('diamants:beacons-updated'));
                };
            } catch (e) { console.warn('[DIAMANTS] LLM Chat API init error:', e); }

            log('✅ Contrôleur intégré initialisé avec succès');
            
            // === NOUVEAU: Initialiser le connecteur métriques UI ===
            this.metricsUI = new MetricsUIConnector(this);
            this.metricsUI.start(500); // Mise à jour toutes les 500ms
            log('📊 MetricsUIConnector démarré');
            
            // === NOUVEAU: Initialiser le système de voxelisation 3D + Pathfinder Dijkstra ===
            this.setupVoxelizer();
            log('🧊 Système Voxelizer + Dijkstra Pathfinder initialisé');
            
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
            
            // === NOUVEAU: Initialiser le UI Controller (index-v2.html sidebar OU index.html slider) ===
            if (document.getElementById('panel-mission') || document.getElementById('autonomy-slider')) {
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
            
            // === LLM Chat Panel: connect beacon system to the 3D scene ===
            try {
                if (this.llmChatPanel && this.scene) {
                    this.llmChatPanel.initBeaconSystem(this.scene);
                    log('🤖 LLM Chat Panel + Beacon System initialisé');
                }
            } catch (e) { console.warn('LLM Chat Panel init error:', e); }
            
        } catch (error) {
            console.error('❌ Erreur initialisation contrôleur intégré:', error);
        }
    }

    /**
     * Initialiser le système de voxelisation 3D et le pathfinder Dijkstra
     * Permet la navigation avec évitement d'obstacles
     */
    setupVoxelizer() {
        try {
            // Configuration de la zone de vol (basée sur terrain provençal)
            const zoneConfig = {
                minX: -50,
                maxX: 50,
                minZ: -50,
                maxZ: 50,
                minY: 0,
                maxY: 20,
                resolution: 0.5  // 50cm par voxel
            };
            
            // Créer le voxelizer avec visualisation Three.js
            this.voxelizer = new EnvironmentVoxelizer(this.scene, zoneConfig);
            
            // Créer le pathfinder Dijkstra connecté à la grille voxel
            this.pathfinder = createPathfinderFromVoxelGrid(this.voxelizer.grid);
            
            // Ajouter obstacles depuis l'environnement (arbres, bâtiments...)
            this._populateVoxelObstacles();
            
            // Exposer globalement pour accès debug/UI
            window.DIAMANTS = window.DIAMANTS || {};
            window.DIAMANTS.voxelizer = this.voxelizer;
            window.DIAMANTS.pathfinder = this.pathfinder;
            
            // Commande de visualisation
            window.DIAMANTS.showVoxels = (options = {}) => {
                if (this.voxelizer?.visualizer) {
                    this.voxelizer.visualizer.options = { 
                        ...this.voxelizer.visualizer.options, 
                        ...options 
                    };
                    this.voxelizer.refresh();
                }
            };
            
            // Commande de pathfinding
            window.DIAMANTS.findPath = (start, goal) => {
                const path = this.pathfinder?.findPath(start, goal);
                if (path) {
                    this.voxelizer?.grid?.markPath(path);
                    this.voxelizer?.refresh();
                }
                return path;
            };
            
            // Stats
            window.DIAMANTS.getVoxelStats = () => this.voxelizer?.getStats();
            
            // Injecter le pathfinder dans CollaborativeScouting
            if (this.integratedController?.collaborativeScouting) {
                this.integratedController.collaborativeScouting.setPathfinder(
                    this.pathfinder, 
                    this.voxelizer
                );
                log('🔗 Pathfinder Dijkstra connecté au CollaborativeScouting');
            }
            
            // NOUVEAU: Injecter le pathfinder dans AutonomousFlightEngine
            if (this.integratedController?.autonomousFlightEngine) {
                this.integratedController.autonomousFlightEngine.setPathfinder(
                    this.pathfinder,
                    this.voxelizer
                );
                log('🔗 Pathfinder Dijkstra connecté à AutonomousFlightEngine');
                
                // Injecter la fonction de hauteur terrain pour collision réaliste
                if (this.environment?.getHeightAt) {
                    const heightFn = this.environment.getHeightAt.bind(this.environment);
                    this.integratedController.autonomousFlightEngine.setTerrainHeightFunction(heightFn);
                    log('🏔️ Fonction hauteur terrain connectée à AutonomousFlightEngine');
                    
                    // Aussi injecter dans le moteur stigmergique s'il est actif
                    const si = this.integratedController.autonomousFlightEngine.swarmIntelligence;
                    if (si && typeof si.setTerrainHeightFunction === 'function') {
                        si.setTerrainHeightFunction(heightFn);
                        log('🏔️ Fonction hauteur terrain connectée au moteur stigmergique');
                    }
                }
                
                // NOUVEAU: Injecter la scène pour les capteurs Multi-Ranger
                if (this.scene) {
                    this.integratedController.autonomousFlightEngine.setScene(this.scene);
                    log('📡 Capteurs Multi-Ranger initialisés avec la scène');
                }
            }
            
            log(`🧊 Voxelizer: ${this.voxelizer.getStats().totalCells} voxels (${zoneConfig.resolution}m res)`);
            
        } catch (error) {
            console.error('❌ Erreur initialisation voxelizer:', error);
        }
    }
    
    /**
     * Peupler les obstacles dans la grille voxel depuis l'environnement
     */
    _populateVoxelObstacles() {
        if (!this.voxelizer || !this.environment) return;
        
        // NOUVEAU: Ajouter le terrain comme obstacle (sol + relief)
        if (this.environment.getHeightAt) {
            this.voxelizer.populateTerrain(
                this.environment.getHeightAt.bind(this.environment),
                0.3 // 30cm de marge au-dessus du terrain
            );
            log('🏔️ Terrain ajouté comme obstacles voxel');
        }
        
        // Ajouter les arbres comme obstacles cylindriques
        if (this.environment.trees && this.environment.trees.length > 0) {
            for (const treeObj of this.environment.trees) {
                if (treeObj?.position) {
                    // Estimer rayon et hauteur de l'arbre depuis bounding box
                    let radius = 1.5;
                    let height = 10;
                    try {
                        if (treeObj.geometry?.boundingBox) {
                            const box = treeObj.geometry.boundingBox;
                            radius = Math.max((box.max.x - box.min.x) / 2, (box.max.z - box.min.z) / 2) * 0.6;
                            height = box.max.y - box.min.y;
                        }
                    } catch (e) {}
                    
                    const baseY = this.environment.getHeightAt 
                        ? this.environment.getHeightAt(treeObj.position.x, treeObj.position.z) 
                        : 0;
                    
                    this.voxelizer.addObstacle('cylinder', 
                        { x: treeObj.position.x, y: baseY, z: treeObj.position.z },
                        Math.max(1.0, Math.min(radius, 4.0)),  // Clamp 1-4m
                        Math.max(3.0, Math.min(height, 20.0))   // Clamp 3-20m
                    );
                }
            }
            log(`🌲 ${this.environment.trees.length} arbres ajoutés comme obstacles voxel`);
        }
        
        // Ajouter quelques obstacles de test si pas d'environnement
        if (!this.environment.trees || this.environment.trees.length === 0) {
            // Obstacles de démonstration
            this.voxelizer.addObstacle('cylinder', { x: 10, y: 0, z: 5 }, 2, 8);
            this.voxelizer.addObstacle('cylinder', { x: -15, y: 0, z: 8 }, 1.5, 6);
            this.voxelizer.addObstacle('sphere', { x: -5, y: 5, z: -10 }, 3);
            log('🧪 Obstacles de démonstration ajoutés');
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
                get() { return self.integratedController?.metrics || {}; },
                configurable: true
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
                
                if (import.meta.env.DEV) {
                    import('./tests/button-test-suite.js').then((mod) => {
                        window.ButtonTestSuite = mod.ButtonTestSuite;
                        window.TEST_DATASETS = mod.TEST_DATASETS;
                        log('🧪 ButtonTestSuite chargé (dev only)');
                    }).catch((e) => warn('ButtonTestSuite dev load failed:', e));
                }
                
                // Auto-tests désactivés — utilisez runButtonTests() manuellement dans la console
                if (false && window.location.hostname === 'localhost') {
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
        indicator.innerHTML = 'Chargement de l\'environnement 3D…';
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
    window.addEventListener('orientationchange', () => { setTimeout(() => this.onWindowResize(), 150); });
        
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
        if (this.controls) this.controls.update();
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
                // Reset caméra - centré sur les drones (also exits follow mode)
                this.autoFollow = false;
                this.followDroneIndex = -1;
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
                
            case 'KeyV':
                this.fractalPanel?.toggle();
                break;

            case 'ArrowDown':
                if (this.fractalPanel?.visible) {
                    this.fractalHypervision?.drillDown();
                    this.fractalPanel.render();
                    event.preventDefault();
                }
                break;

            case 'ArrowUp':
                if (this.fractalPanel?.visible) {
                    this.fractalHypervision?.drillUp();
                    this.fractalPanel.render();
                    event.preventDefault();
                }
                break;
                
            case 'KeyH':
                // Aide
                log(`
🎮 DIAMANTS Mission System V3 - Contrôles:
V: Hypervision fractale (L0→L5, couche ROS)
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

        // ── PROFILER DE FRAME (diagnostic FPS) ── mesure où part le temps.
        // Logge un résumé toutes les 2 s : CPU (contrôleur/env) vs GPU (rendu).
        this._prof = this._prof || { ctrl: 0, env: 0, render: 0, frames: 0, last: performance.now() };
        const _pf = this._prof;
        const _t0 = performance.now();

        // ── Dolly zoom: scroll = avancer/reculer dans la scène ──
        // Déplace la caméra ET le point d'orbite le long de l'axe de vue.
        // Permet de naviguer librement partout dans la scène, pas juste orbiter un point.
        if (this._zoomAccum !== 0) {
            const raw = this._zoomAccum;
            this._zoomAccum = 0;
            // Convertir pixels → vitesse de déplacement
            // Un tick molette ≈ 120px → speed ≈ 0.07 × dist
            const speed = raw * -0.0006; // négatif car deltaY+ = scroll down = reculer
            const offset = this.camera.position.clone().sub(this.controls.target);
            const dist = offset.length();
            // Direction de vue (de la caméra vers le target), projetée au sol (XZ)
            const viewDir = this.controls.target.clone().sub(this.camera.position);
            viewDir.y = 0; // Rester à la même hauteur
            viewDir.normalize();
            // Distance de déplacement proportionnelle à la distance actuelle + un minimum
            const moveAmount = speed * Math.max(dist * 0.5, 3);
            // Clamp le déplacement à ±15 unités par frame
            const clampedMove = Math.max(-15, Math.min(15, moveAmount));
            const moveVec = viewDir.multiplyScalar(clampedMove);
            // Déplacer caméra ET target ensemble (dolly, pas orbit zoom)
            this.camera.position.add(moveVec);
            this.controls.target.add(moveVec);
        }

        // ── Clamp camera + target within ops zone (120m from origin) ──
        const CAM_BOUNDS = 120;
        const cx = this.camera.position.x, cz = this.camera.position.z;
        const tx = this.controls.target.x, tz = this.controls.target.z;
        if (Math.abs(cx) > CAM_BOUNDS || Math.abs(cz) > CAM_BOUNDS ||
            Math.abs(tx) > CAM_BOUNDS || Math.abs(tz) > CAM_BOUNDS) {
            this.camera.position.x = Math.max(-CAM_BOUNDS, Math.min(CAM_BOUNDS, cx));
            this.camera.position.z = Math.max(-CAM_BOUNDS, Math.min(CAM_BOUNDS, cz));
            this.controls.target.x = Math.max(-CAM_BOUNDS, Math.min(CAM_BOUNDS, tx));
            this.controls.target.z = Math.max(-CAM_BOUNDS, Math.min(CAM_BOUNDS, tz));
        }
        // controls.update() moved to end of animate — single call per frame for smooth damping

        // ── Camera persistence: restore on first frame, save periodically ──
        // Camera restore disabled — always start with a clean view of the helipad
        // (Previous localStorage restore would often place camera inside trees)
        if (false && this._cameraRestorePending) {
            this._cameraRestorePending = false;
            const saved = this._loadCameraState();
            if (saved) {
                this.camera.position.set(saved.px, saved.py, saved.pz);
                this.controls.target.set(saved.tx, saved.ty, saved.tz);
                this.controls.update();
                log('📷 Caméra restaurée depuis la session précédente');
            }
        }
        // Throttled save every 2 seconds
        const now = performance.now();
        if (now - this._lastCameraSave > 2000) {
            this._lastCameraSave = now;
            this._saveCameraState();
        }
        
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
        
        // Grass + terrain: update every frame (GPU-driven animation, cheap)
        if (this.environment) {
            if (this.environment.grassField && this.camera) {
                this.environment.grassField.update(this.elapsedTime, this.camera.position);
            }
            if (this.environment.updateTerrainFollow && this.camera) {
                this.environment.updateTerrainFollow(this.camera.position);
            }
            if (this.environment.updateTreeChunks && this.camera) {
                this.environment.updateTreeChunks(this.camera.position);
            }
        }
        // Heavy environment updates (trees, leaves, undergrowth, skybox) — throttled to 10 Hz
        this._envAccum = (this._envAccum || 0) + delta;
        if (this._envAccum >= 0.1 && this.environment) {
            if (this.environment.updateSkybox) this.environment.updateSkybox(this.elapsedTime);
            if (this.environment.fallenLeaves) this.environment.fallenLeaves.animate(this.elapsedTime);
            if (this.environment.forestWood) this.environment.forestWood.update(this.elapsedTime, this.camera);
            if (this.environment.undergrowth) this.environment.undergrowth.update(this.elapsedTime);
            if (this.environment.trees) this.environment.trees.forEach(t => t.update?.(this.elapsedTime));
            if (this.environment.qualityPanel) this.environment.qualityPanel.updatePerformanceDisplay();
            this._envAccum = 0;
        }

        // swarmZoneOverlay DISABLED — allocates/destroys geometries every frame
        // if (this.swarmZoneOverlay) { this.swarmZoneOverlay.update(); }

        _pf.env += performance.now() - _t0; // fin des màj environnement
        const _tCtrl = performance.now();

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
                if (intelEl && this.integratedController.diamantFormulas) {
                    // Composite intelligence: coverage + alignment + emergence
                    const sm = this.integratedController.diamantFormulas.swarmMetrics || {};
                    const coverage = (this.integratedController.metrics?.coveragePercentage || 0) / 100;
                    const alignment = sm.alignment || 0;
                    const emergence = sm.emergence || 0;
                    const flyingCount = this.drones.filter(d => d.state && d.state !== 'IDLE' && d.state !== 'LANDED').length;
                    const activeRatio = flyingCount / Math.max(1, this.drones.length);
                    const intel = (0.35 * coverage + 0.35 * alignment + 0.30 * emergence) * 100 * activeRatio;
                    intelEl.textContent = Math.min(100, intel).toFixed(0);
                }
                if (emergeEl) {
                    // Emergence: scientific metric from computeScientificEmergence()
                    const sm = this.integratedController.diamantFormulas?.swarmMetrics || {};
                    const emergenceVal = sm.emergence || 0;
                    emergeEl.textContent = (emergenceVal * 100).toFixed(0);
                }
                
                // Mettre à jour les minimaps — throttled to 5 Hz (they have their own rAF loops)
                this._mmAccum = (this._mmAccum || 0) + delta;
                if (this._mmAccum >= 0.2) {
                    this._updateMinimaps();
                    this._mmAccum = 0;
                }
                
                // LLM Chat Panel: beacon proximity check (throttled with minimaps at 5 Hz)
                if (this.llmChatPanel) {
                    this.llmChatPanel.tick(this.drones);
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
                
                // Reculer la caméra pour voir le drone + son label/panneau texte au-dessus
                const idealOffset = new THREE.Vector3(6, 5, 6);
                const idealPos = target.clone().add(idealOffset);
                this.camera.position.lerp(idealPos, 0.05);
            }
        }
        // Always keep OrbitControls in sync so mouse interaction works in free mode
        if (this.controls) this.controls.update();

        // ── HARD CAMERA GROUND CLAMP — user must NEVER see under the terrain ──
        if (this.camera) {
            const MIN_CAM_Y = 0.5; // absolute minimum camera height
            let groundY = 0;
            if (this.environment?.getHeightAt) {
                try {
                    groundY = this.environment.getHeightAt(this.camera.position.x, this.camera.position.z) || 0;
                } catch (_) {}
            }
            const minY = Math.max(MIN_CAM_Y, groundY + 0.4);
            if (this.camera.position.y < minY) {
                this.camera.position.y = minY;
            }
            // Also clamp the orbit target above ground
            if (this.controls?.target) {
                let targetGroundY = 0;
                if (this.environment?.getHeightAt) {
                    try {
                        targetGroundY = this.environment.getHeightAt(this.controls.target.x, this.controls.target.z) || 0;
                    } catch (_) {}
                }
                if (this.controls.target.y < targetGroundY + 0.2) {
                    this.controls.target.y = targetGroundY + 0.2;
                }
            }
        }
        
        _pf.ctrl += performance.now() - _tCtrl; // fin contrôleur/drones
        const _tRender = performance.now();

        // Rendu
    if (!this.renderer || this._contextLost) {
            // Skip rendering while context is lost/unavailable
        } else if (this.usePostProcessing && this.composer) {
            try { this.composer.render(); } catch (e) { /* ignore transient context errors */ }
        } else {
            try { this.renderer.render(this.scene, this.camera); } catch (e) { /* ignore transient context errors */ }
        }

        _pf.render += performance.now() - _tRender;
        _pf.frames++;
        if (performance.now() - _pf.last >= 2000) {
            const n = Math.max(1, _pf.frames);
            const tot = (_pf.env + _pf.ctrl + _pf.render) / n;
            const realFrameMs = 2000 / n;              // temps réel entre 2 frames
            const gap = realFrameMs - tot;             // temps passé HORS de notre code
            console.log(
                `%c[PERF] ${(_pf.frames / 2).toFixed(0)} fps | env ${(_pf.env / n).toFixed(1)}ms | ` +
                `controller ${(_pf.ctrl / n).toFixed(1)}ms | render ${(_pf.render / n).toFixed(1)}ms | ` +
                `draws ${this.renderer?.info?.render?.calls} | tris ${Math.round((this.renderer?.info?.render?.triangles || 0) / 1000)}k | ` +
                `objets ${(() => { let objs = 0; this.scene?.traverse(() => objs++); return objs; })()} | ` +
                `notre code ${tot.toFixed(1)}ms | frame réelle ${realFrameMs.toFixed(1)}ms | ` +
                `HORS CODE ${gap.toFixed(1)}ms (${(100 * gap / realFrameMs).toFixed(0)}%)`,
                'font-weight:bold;color:#0aa'
            );
            _pf.env = _pf.ctrl = _pf.render = 0; _pf.frames = 0; _pf.last = performance.now();
        }

        this.casController?.tick(delta);
        this.fractalHypervision?.tick(delta);

        // NOTE: Frontend does NOT publish odometry. All positions come from the
        // ROS2 backend (Gazebo → PositionBroadcaster → WebSocket Bridge → here).
        // The frontend is a pure viewer, never a simulator.
    }

    // ─── Minimap expand/collapse ────────────────────────────────────
    _initMinimapExpander() {
        const lightbox = document.getElementById('minimap-lightbox');
        const lbCanvas = document.getElementById('minimap-lightbox-canvas');
        const lbHeader = document.getElementById('minimap-lightbox-header');
        if (!lightbox || !lbCanvas || !lbHeader) return;

        const lbCtx = lbCanvas.getContext('2d');
        let activeSourceCanvas = null;
        let rafId = null;

        /** Resize a minimap's canvas + offscreen buffers to given dimensions. */
        const resizeMinimapBuffers = (canvas, w, h) => {
            canvas.width = w;
            canvas.height = h;
            const inst = canvas._minimapInstance;
            if (!inst) return;
            // Offscreen buffer variants
            if (inst._offscreen) { inst._offscreen.width = w; inst._offscreen.height = h; }
            if (inst._off)       { inst._off.width = w;       inst._off.height = h; }
            // PerceptionMinimap pixel buffer
            if (inst._imgData && inst.ctx) {
                inst._imgData = inst.ctx.createImageData(w, h);
            }
        };

        const closeLightbox = () => {
            // Restore source canvas to original minimap resolution
            if (activeSourceCanvas) {
                resizeMinimapBuffers(activeSourceCanvas, 280, 200);
            }
            lightbox.classList.remove('active');
            activeSourceCanvas = null;
            if (rafId) { cancelAnimationFrame(rafId); rafId = null; }
        };

        // Mirror source canvas → lightbox canvas at ~30fps
        const mirrorLoop = () => {
            if (!activeSourceCanvas || !lightbox.classList.contains('active')) return;
            lbCtx.clearRect(0, 0, lbCanvas.width, lbCanvas.height);
            lbCtx.imageSmoothingEnabled = false;
            lbCtx.drawImage(activeSourceCanvas, 0, 0, lbCanvas.width, lbCanvas.height);
            rafId = requestAnimationFrame(mirrorLoop);
        };

        const openLightbox = (sourceCanvas, headerText, borderColor) => {
            activeSourceCanvas = sourceCanvas;
            const titleSpan = document.getElementById('minimap-lightbox-title');
            if (titleSpan) titleSpan.textContent = headerText;
            else lbHeader.textContent = headerText;
            lbHeader.style.background = '';
            // Copy the header gradient from the panel
            const panel = sourceCanvas.closest('.minimap-panel');
            if (panel) {
                const origHeader = panel.querySelector('.minimap-header');
                if (origHeader) {
                    lbHeader.style.background = getComputedStyle(origHeader).background;
                }
            }
            // Size: fill most of the viewport while keeping aspect ratio
            const vw = Math.min(window.innerWidth * 0.85, 1200);
            const vh = Math.min(window.innerHeight * 0.78, 800);
            const aspect = 280 / 200;
            let w, h;
            if (vw / vh > aspect) {
                h = vh; w = h * aspect;
            } else {
                w = vw; h = w / aspect;
            }
            w = Math.round(w);
            h = Math.round(h);

            lbCanvas.width = w;
            lbCanvas.height = h;
            lbCanvas.style.width = w + 'px';
            lbCanvas.style.height = h + 'px';

            // Resize source canvas to lightbox resolution for crisp rendering
            resizeMinimapBuffers(sourceCanvas, w, h);

            if (borderColor) {
                lightbox.querySelector('#minimap-lightbox-inner').style.border = `2px solid ${borderColor}`;
            }

            lightbox.classList.add('active');
            mirrorLoop();
        };

        // Double-click header → open lightbox  (single click reserved for detach button)
        const container = document.getElementById('minimaps-container');
        if (!container) return;

        container.querySelectorAll('.minimap-panel').forEach(panel => {
            const header = panel.querySelector('.minimap-header');
            const canvas = panel.querySelector('canvas');
            if (!header || !canvas) return;

            const borderColor = getComputedStyle(panel).borderColor;

            header.addEventListener('dblclick', (e) => {
                // Don't open lightbox if user double-clicked the detach button
                if (e.target.closest('.minimap-detach-btn')) return;
                if (lightbox.classList.contains('active')) {
                    closeLightbox();
                } else {
                    openLightbox(canvas, header.textContent, borderColor);
                }
            });
        });

        // Close on click backdrop, header, or close button
        lightbox.addEventListener('click', (e) => {
            if (e.target === lightbox || e.target === lbHeader || e.target.id === 'minimap-lightbox-close') closeLightbox();
        });
        // Expose closeLightbox for inline onclick
        window._closeLightbox = closeLightbox;
        // Close on Escape
        document.addEventListener('keydown', (e) => {
            if (e.code === 'Escape' && lightbox.classList.contains('active')) closeLightbox();
        });

        log('🔍 Minimap lightbox initialisé (clic header pour vue agrandie)');
    }

    _syncAllMinimapCanvases() {
        // No-op — original canvases are never resized now
    }

    // ─── Minimap update helper ───────────────────────────────────────
    _updateMinimaps() {
        const flightEngine = this.integratedController?.autonomousFlightEngine;
        if (!flightEngine) return;

        // Access the flight engine's internal drone states (Map<id, DroneFlightState>)
        const droneStates = flightEngine.drones; // Map
        if (!droneStates || droneStates.size === 0) return;

        // Expose stigmergy engine instance for the exploration minimap to read pheromone data
        // Re-check every frame: the real StigmergyEngine may replace the NoopSwarmIntelligence
        // after async loading. Only expose engines that have getGrid() (not Noop).
        const swarmImpl = flightEngine.swarmIntelligence;
        if (swarmImpl && typeof swarmImpl.getGrid === 'function') {
            if (window.DIAMANTS_STIGMERGY_INSTANCE !== swarmImpl) {
                window.DIAMANTS_STIGMERGY_INSTANCE = swarmImpl;
            }
        }

        // Expose the engine's permanent visited-cells Set for flicker-free minimap coverage
        // (the journal expires entries after 2min → flicker; visitedCells is permanent)
        if (!window.DIAMANTS_VISITED_CELLS) {
            window.DIAMANTS_VISITED_CELLS = flightEngine.visitedCells;
            window.DIAMANTS_CELL_SIZE = flightEngine.cellSize;
        }

        for (const [id, state] of droneStates) {
            if (!state.position) continue;

            // ── Exploration minimap: position + heading + current waypoint
            // Hide waypoint markers at high autonomy (no orchestrator)
            if (window.DIAMANTS_MINIMAP) {
                const autoLvl = window.DIAMANTS_AUTONOMY_LEVEL ?? 0;
                const wp = (autoLvl < 75 && state.waypoint)
                    ? { x: state.waypoint.x, z: state.waypoint.z }
                    : null;
                window.DIAMANTS_MINIMAP.updateDronePosition(
                    id, state.position, state.heading || 0, wp
                );
            }

            // ── Perception minimap: position + heading + sensor ranges
            if (window.DIAMANTS_PERCEPTION && state.sensorRanges) {
                window.DIAMANTS_PERCEPTION.updateFromSensor(
                    id, state.position, state.heading || 0, state.sensorRanges
                );
            }

            // ── Discovery minimap: position only (pixel fog-of-war)
            if (window.DIAMANTS_DISCOVERY) {
                window.DIAMANTS_DISCOVERY.updateDronePosition(id, state.position);
            }

            // ── SITAC minimap: position + heading + waypoint (tactical)
            // Hide waypoint markers at high autonomy (no orchestrator)
            if (window.DIAMANTS_SITAC) {
                const autoLvl2 = window.DIAMANTS_AUTONOMY_LEVEL ?? 0;
                const wp = (autoLvl2 < 75 && state.waypoint)
                    ? { x: state.waypoint.x, z: state.waypoint.z }
                    : null;
                window.DIAMANTS_SITAC.updateDronePosition(
                    id, state.position, state.heading || 0, wp
                );
            }
        }
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

function showFatalInitError(title, detail) {
    const existing = document.getElementById('diamants-fatal-error');
    if (existing) existing.remove();
    const box = document.createElement('div');
    box.id = 'diamants-fatal-error';
    box.style.cssText = `
        position: fixed; inset: 0; z-index: 99999; display: flex; align-items: center; justify-content: center;
        background: rgba(5, 10, 25, 0.92); font-family: system-ui, sans-serif; padding: 24px;
    `;
    box.innerHTML = `
        <div style="max-width: 520px; background: #1a1a2e; color: #e2e8f0; padding: 24px; border-radius: 12px;
            border: 2px solid #ff4757; text-align: center; box-shadow: 0 8px 32px rgba(0,0,0,0.5);">
            <h2 style="margin: 0 0 12px; color: #ff6b6b;">🛑 ${title}</h2>
            <pre style="text-align: left; background: #0f172a; padding: 12px; border-radius: 8px; font-size: 12px;
                overflow: auto; max-height: 200px; white-space: pre-wrap; word-break: break-word;">${detail}</pre>
            <button onclick="location.reload()" style="margin-top: 16px; padding: 10px 20px; background: #00ccff;
                color: #1a1a2e; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">🔄 Recharger</button>
        </div>
    `;
    document.body.appendChild(box);
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
        const msg = error?.message || String(error);
        console.error('💥 Erreur fatale lors du démarrage:', msg, error?.stack || '');
        showFatalInitError('DIAMANTS ne démarre pas', msg);
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
