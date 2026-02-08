/**
 * DIAMANTS - Point d'entrée principal avec Vite et EZ-Tree authentique
 * =======================================================================
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
        const text = args.join(' ');
        // Garder les logs critiques pour le 3D et les erreurs
        if (text.includes('DAE') || text.includes('THREE') || text.includes('ERROR') || 
            text.includes('ERREUR') || text.includes('✅') || text.includes('❌') ||
            text.includes('Moteur') || text.includes('hélice') || text.includes('propeller')) {
            originalLog(...args);
        }
        // Bloquer tous les autres logs
    };
    
    console.info = (...args) => {
        const text = args.join(' ');
        if (text.includes('DAE') || text.includes('THREE') || text.includes('ERROR')) {
            originalInfo(...args);
        }
    };
}

// DIAMANTS - Chargement principal
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
import { RosWebBridge, makeOdometry, yawToQuat } from './net/ros-bridge-simple.js';
import { IntegratedDiamantsController } from './tools/integrated-controller.js';
import { OrchestrationConsole } from './ui/orchestration-console.js';

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
        this.integratedController = null; // NOUVEAU: Contrôleur intégré
        // Backend-less WebSim can publish/sub ROS topics via rosbridge
        this.roswebEnabled = true; // toggle to enable ROS-in-the-browser
        this.ros = null;
        // Visual safety defaults: clearer visuals, full manual control
        this.autoFocus = false; // off by default to avoid camera auto-move
        this.usePostProcessing = false; // off by default to avoid halo/double tonemapping
        this.visualSafeMode = true; // disables fog + heavy tone mapping
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

        // Tone mapping: keep off in safe mode to avoid halos; can be toggled later
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
                        drone.rosData.source = 'backend';
                        // Forward backend battery & status
                        if (info.battery !== undefined) drone.rosData.battery = info.battery;
                        if (info.status) drone.rosData.status = info.status;
                    } catch (_) { /* safe */ }
                }
                this._backendPosCount++;
                const now = Date.now();
                if (now - this._lastBackendLog > 5000) {
                    this._lastBackendLog = now;
                    console.warn(`📡 Backend positions: ${this._backendPosCount} updates, ${matched}/${Object.keys(drones).length} drones matched, ${this.drones?.length || 0} local drones`);
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
            // En mode visuel sûr, désactiver la brume pour éviter les halos
            if (this.visualSafeMode) {
                try { this.scene.fog = null; } catch (_) { /* noop */ }
            }
            // Ensuite, déployer une petite flotte de Crazyflie bien visibles
            this.setupDrones();
            // Exposer un pont de compatibilité global pour l'UI (window.DIAMANTS)
            this.exposeGlobalBridge();
            
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
                droneCount: 8, // Must match backend (8 Crazyflie in Gazebo)
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
        } catch (_) { /* noop */ }
    }

    setupDrones() {
        // Les drones sont créés et gérés par l'IntegratedController
        // Ne pas créer de drones supplémentaires ici
        log('🚁 Les drones seront créés par l\'IntegratedController');
        
        // Initialiser un tableau vide qui sera rempli par l'IntegratedController
        this.drones = [];
    }

    // Calcule le centre et le rayon englobant de la flotte pour cadrer automatiquement la caméra
    computeFleetBounds() {
        if (!this.drones.length) return { center: new THREE.Vector3(0, 0.5, 0), radius: 5 };
        const center = new THREE.Vector3();
        let count = 0;
        this.drones.forEach(d => {
            if (d && d.position) { center.add(d.position); count++; }
        });
        if (count > 0) center.multiplyScalar(1 / count);
        let radius = 5;
        this.drones.forEach(d => {
            if (d && d.position) radius = Math.max(radius, d.position.distanceTo(center));
        });
        return { center, radius };
    }

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
                // Toggle fog (brume)
                if (this.scene.fog) {
                    this.scene.fog = null;
                    log('🌫️ Brume désactivée');
                } else {
                    this.scene.fog = new THREE.FogExp2(0xE6E6FA, 0.0008);
                    log('🌫️ Brume activée');
                }
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
            }
        }
        
        // Mise à jour réaliste des drones (vol, RPM, hélices, exploration)
        // Note: Les drones sont maintenant gérés principalement par l'IntegratedController
        // La boucle de mise à jour individuelle des drones est supprimée pour éviter les conflits

        // Plus de marqueurs à lier - les drones sont visibles par eux-mêmes

        // AUTOFOCUS COMPLÈTEMENT DÉSACTIVÉ - PAS DE MOUVEMENT AUTOMATIQUE
        // L'utilisateur contrôle manuellement la caméra avec la souris
        if (false) { // Force désactivation - même si this.autoFocus devient true accidentellement
            const { center, radius } = this.computeFleetBounds();
            // Code d'autofocus désactivé
        }
        
        // Rendu
    if (!this.renderer || this._contextLost) {
            // Skip rendering while context is lost/unavailable
        } else if (this.usePostProcessing && this.composer) {
            try { this.composer.render(); } catch (e) { /* ignore transient context errors */ }
        } else {
            try { this.renderer.render(this.scene, this.camera); } catch (e) { /* ignore transient context errors */ }
        }

        // NOTE: Frontend does NOT publish odometry. All positions come from the
        // ROS2 backend (Gazebo → PositionBroadcaster → WebSocket Bridge → here).
        // The frontend is a pure viewer, never a simulator.
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

// Mission lanceur — sends real mission commands to the backend via WebSocket
window.launchMission = function() {
    log('🚀 LANCEMENT MISSION — envoi commande au backend ROS2');
    
    const system = window.diamantsSystem;
    if (!system || !system.ros) {
        log('⚠️ WebSocket bridge non disponible, tentative de connexion...');
    }

    // Send mission start command via WebSocket bridge
    try {
        if (system && system.ros && system.ros.ws && system.ros.ws.readyState === WebSocket.OPEN) {
            system.ros.ws.send(JSON.stringify({
                type: 'mission_command',
                data: { action: 'start', mission_type: 'exploration' },
            }));
            log('✅ Commande mission:start envoyée au backend');
        } else {
            log('⚠️ WebSocket non connecté — la mission est déjà active côté backend');
        }
    } catch (e) {
        log('⚠️ Envoi commande mission échoué:', e.message);
    }

    // Also notify the integrated controller
    if (system && system.integratedController) {
        try { system.integratedController.startMissionManual(); } catch (_) {}
    }
};

window.emergencyLand = function() {
    log('🛑 Arrêt d\'urgence — envoi commande emergency au backend');
    
    const system = window.diamantsSystem;
    try {
        if (system && system.ros && system.ros.ws && system.ros.ws.readyState === WebSocket.OPEN) {
            system.ros.ws.send(JSON.stringify({
                type: 'mission_command',
                data: { action: 'emergency' },
            }));
            log('✅ Commande emergency envoyée');
        }
    } catch (e) { log('⚠️ Erreur envoi emergency:', e.message); }

    // Also trigger local emergency
    if (system && system.integratedController) {
        try { system.integratedController.emergencyStop(); } catch (_) {}
    }
};

window.resetSwarm = function() {
    log('🔄 Reset essaim — envoi commande return_home au backend');
    
    const system = window.diamantsSystem;
    try {
        if (system && system.ros && system.ros.ws && system.ros.ws.readyState === WebSocket.OPEN) {
            system.ros.ws.send(JSON.stringify({
                type: 'mission_command',
                data: { action: 'return_home' },
            }));
            log('✅ Commande return_home envoyée');
        }
    } catch (e) { log('⚠️ Erreur envoi reset:', e.message); }
};

// changePattern défini dans drone-panel-controller.js

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

window.toggleDebugPanels = function() {
    log('🛠️ Toggle debug panels');
    alert('🛠️ Debug panels toggled');
};

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
