/**
 * DIAMANTS - Interface Utilisateur Complète
 * ============================================
 * Interface de contrôle DIAMANTS
 */

// Mode silencieux global
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

export class DiamantUI {
    constructor(config = {}) {
        this.config = {
            updateInterval: config.updateInterval || 100, // ms
            theme: config.theme || 'dark',
            showAdvancedMetrics: config.showAdvancedMetrics || false,
            ...config
        };

        this.elements = {};
        this.metrics = {};
        this.isInitialized = false;
        this.updateCallbacks = new Map();

        // Mini-map tactique
        this.minimap = {
            canvas: null,
            ctx: null,
            visible: true,
            scale: 4,
            width: 200,
            height: 200
        };

        this.initializeUI();
    }

    initializeUI() {
        try {
            log('🎨 Début initialisation interface...');

            // Version simplifiée pour éviter les blocages
            this.createSimpleInterface();
            this.setupBasicEventHandlers();

            this.isInitialized = true;
            log('🎨 Interface DIAMANTS initialisée');
        } catch (error) {
            error('❌ Erreur lors de l\'initialisation UI:', error);
            this.createFallbackInterface();
        }
    }

    // Compatibilité avec l'appel "ui.create()" dans l'application
    // L'UI est déjà créée au constructeur; on expose donc un no-op sûr.
    create() {
        return this;
    }

    createSimpleInterface() {
        // Interface minimale pour éviter les conflits
        const overlayUI = document.createElement('div');
        overlayUI.id = 'diamants-ui-overlay';
        overlayUI.style.cssText = `
            position: fixed;
            top: 10px;
            left: 10px;
            background: rgba(0, 20, 40, 0.9);
            border: 1px solid #00AAFF;
            border-radius: 8px;
            padding: 15px;
            color: #00FFCC;
            font-family: monospace;
            z-index: 100;
            backdrop-filter: blur(5px);
            max-width: 350px;
            max-height: 80vh;
            overflow-y: auto;
        `;

        overlayUI.innerHTML = `
            <h3>🚁 DIAMANTS Control</h3>
            
            <div id="system-metrics">
                <div>Status: <span id="sim-status">Active</span></div>
                <div>Drones: <span id="drone-count">0</span></div>
                <div>Intelligence: <span id="intelligence-level">0.0</span></div>
            </div>
            
            <div style="margin-top: 10px;">
                <button id="pause-btn" class="ui-btn">⏸️ Pause</button>
                <button id="reset-btn" class="ui-btn">🔄 Reset</button>
                <button id="toggle-logs" class="ui-btn">🔇 Logs: Silent</button>
            </div>
            
            <!-- Contrôles des modules avancés -->
            <div class="module-section">
                <h4>🧠 AI & Behaviors</h4>
                <label><input type="checkbox" id="toggle-advanced-ai" checked> Advanced Intelligence</label><br>
                <label><input type="checkbox" id="toggle-collaborative" checked> Collaborative Scouting</label><br>
                <label><input type="checkbox" id="toggle-flight-behaviors" checked> Flight Behaviors</label><br>
            </div>
            
            <div class="module-section">
                <h4>⚡ Physics & Control</h4>
                <label><input type="checkbox" id="toggle-physics" checked> Physics Engine</label><br>
                <label><input type="checkbox" id="toggle-pid" checked> PID Controller</label><br>
                <label><input type="checkbox" id="toggle-ros" checked> ROS Controller</label><br>
            </div>
            
            <div class="module-section">
                <h4>🎨 Visual & Environment</h4>
                <label><input type="checkbox" id="toggle-visual-enhancements" checked> Visual Enhancements</label><br>
                <label><input type="checkbox" id="toggle-grass-basic" > Basic Grass Field</label><br>
            </div>
            
            <div class="module-section">
                <h4>📊 System Info</h4>
                <div id="module-status">Loading...</div>
                <button id="toggle-advanced-panel" class="ui-btn">🔧 Advanced Panel</button>
            </div>
        `;

        // Ajouter les styles pour les boutons
        const style = document.createElement('style');
        style.textContent = `
            .ui-btn {
                background: #0066AA;
                border: 1px solid #00AAFF;
                color: white;
                padding: 5px 10px;
                margin: 2px;
                border-radius: 3px;
                cursor: pointer;
                font-size: 12px;
            }
            .ui-btn:hover {
                background: #0088CC;
            }
            .module-section {
                margin-top: 10px;
                padding-top: 8px;
                border-top: 1px solid #004466;
            }
            .module-section h4 {
                margin: 0 0 5px 0;
                color: #00AAFF;
                font-size: 13px;
            }
            .module-section label {
                display: block;
                font-size: 11px;
                margin: 3px 0;
                cursor: pointer;
            }
            .module-section input[type="checkbox"] {
                margin-right: 5px;
            }
            #module-status {
                font-size: 10px;
                color: #00FFAA;
                background: rgba(0, 50, 50, 0.5);
                padding: 5px;
                margin: 5px 0;
                border-radius: 3px;
            }
            .ui-btn:hover {
                background: #0088CC;
            }
        `;
        document.head.appendChild(style);

        document.body.appendChild(overlayUI);
        this.elements.overlay = overlayUI;

        // Créer la mini-map tactique
        this.createTacticalMinimap();
    }

    createFallbackInterface() {
        log('🛠️ Interface de secours activée');
        // Interface de secours minimale
        const fallback = document.createElement('div');
        fallback.style.cssText = `
            position: fixed;
            bottom: 10px;
            right: 10px;
            background: rgba(255, 0, 0, 0.8);
            color: white;
            padding: 10px;
            border-radius: 5px;
            z-index: 200;
        `;
        fallback.textContent = 'UI simplifiée - DIAMANTS actif';
        document.body.appendChild(fallback);
    }

    // Boucle de mise à jour minimale appelée par l'application
    update(deltaTime = 0.016) {
        try {
            // Rafraîchir quelques métriques visibles si présentes
            const countEl = document.getElementById('drone-count');
            if (countEl && Array.isArray(this.config.drones)) {
                countEl.textContent = String(this.config.drones.length);
            }

            const intelEl = document.getElementById('intelligence-level');
            if (intelEl && this.config.metrics && typeof this.config.metrics.avgIntelligence === 'number') {
                intelEl.textContent = this.config.metrics.avgIntelligence.toFixed(2);
            }

            // Mettre à jour le statut des modules
            this.updateModuleStatus();

            // Minimap très simple (si créée hors de ce module)
            const canvas = document.getElementById('minimap_canvas');
            if (canvas && canvas.getContext) {
                const ctx = canvas.getContext('2d');
                if (ctx) {
                    // Effacer
                    ctx.fillStyle = '#87CEEB';
                    ctx.fillRect(0, 0, canvas.width, canvas.height);
                    // Dessiner les drones
                    if (Array.isArray(this.config.drones)) {
                        ctx.fillStyle = '#003355';
                        ctx.fillRect(0, canvas.height - 20, canvas.width, 20);
                        for (const d of this.config.drones) {
                            const p = d.getPosition ? d.getPosition() : d.position;
                            if (!p) continue;
                            const x = Math.floor(canvas.width / 2 + p.x * 3);
                            const y = Math.floor(canvas.height / 2 + p.z * 3);
                            ctx.fillStyle = '#00FFFF';
                            ctx.fillRect(x - 2, y - 2, 4, 4);
                        }
                    }
                }
            }

            // Mettre à jour le panneau avancé s'il est ouvert
            if (document.getElementById('advanced-panel')) {
                this.updateAdvancedPanel();
            }

        } catch (e) {
            // Ne jamais casser la boucle principale pour l'UI
        }
    }

    setupBasicEventHandlers() {
        // Gestionnaires d'événements simplifiés
        const pauseBtn = document.getElementById('pause-btn');
        const resetBtn = document.getElementById('reset-btn');

        if (pauseBtn) {
            pauseBtn.addEventListener('click', () => {
                console.log('[UI-BTN] Pause/Resume clicked');
                const system = window.diamantsSystem;
                if (system?.integratedController) {
                    const engine = system.integratedController.autonomousFlightEngine;
                    if (engine) {
                        engine.paused = !engine.paused;
                        console.log(`[UI-BTN] Simulation ${engine.paused ? 'PAUSED' : 'RESUMED'}`);
                        pauseBtn.textContent = engine.paused ? '▶️ Resume' : '⏸ Pause';
                    }
                } else {
                    console.warn('[UI-BTN] Pause: système non initialisé');
                }
            });
        }

        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                console.log('[UI-BTN] Reset clicked');
                if (typeof window.resetMission === 'function') {
                    window.resetMission();
                } else if (typeof window.resetSwarm === 'function') {
                    window.resetSwarm();
                } else {
                    console.warn('[UI-BTN] Reset: aucune fonction disponible');
                }
            });
        }

        // Toggle pour les logs
        const toggleLogsBtn = document.getElementById('toggle-logs');
        if (toggleLogsBtn) {
            this.updateLogButtonText(toggleLogsBtn);
            toggleLogsBtn.addEventListener('click', () => {
                window.SILENT_MODE = !window.SILENT_MODE;
                this.updateLogButtonText(toggleLogsBtn);
                
                // Feedback immédiat
                if (!window.SILENT_MODE) {
                    console.log('🔊 Logs activés - Mode verbeux');
                } else {
                    console.log('🔇 Logs silencieux activés');
                }
            });
        }

        // Gestionnaires pour les modules avancés
        this.setupModuleToggles();
        this.setupAdvancedPanel();
    }

    setupModuleToggles() {
        // Toggles des modules AI & Behaviors
        const toggleAdvancedAI = document.getElementById('toggle-advanced-ai');
        const toggleCollaborative = document.getElementById('toggle-collaborative');
        const toggleFlightBehaviors = document.getElementById('toggle-flight-behaviors');

        // Toggles Physics & Control
        const togglePhysics = document.getElementById('toggle-physics');
        const togglePID = document.getElementById('toggle-pid');
        const toggleROS = document.getElementById('toggle-ros');

        // Toggles Visual & Environment
        const toggleVisualEnhancements = document.getElementById('toggle-visual-enhancements');
        const toggleGrassBasic = document.getElementById('toggle-grass-basic');

        // Connecter les toggles aux modules via le contrôleur
        if (toggleAdvancedAI && this.config.controller) {
            toggleAdvancedAI.addEventListener('change', (e) => {
                this.toggleModule('advancedIntelligence', e.target.checked);
            });
        }

        if (toggleCollaborative && this.config.controller) {
            toggleCollaborative.addEventListener('change', (e) => {
                this.toggleModule('collaborativeScouting', e.target.checked);
            });
        }

        if (toggleFlightBehaviors && this.config.controller) {
            toggleFlightBehaviors.addEventListener('change', (e) => {
                this.toggleModule('flightBehaviors', e.target.checked);
            });
        }

        if (togglePhysics && this.config.controller) {
            togglePhysics.addEventListener('change', (e) => {
                this.toggleModule('dronePhysics', e.target.checked);
            });
        }

        if (togglePID && this.config.controller) {
            togglePID.addEventListener('change', (e) => {
                this.toggleModule('pidController', e.target.checked);
            });
        }

        if (toggleROS && this.config.controller) {
            toggleROS.addEventListener('change', (e) => {
                this.toggleModule('rosController', e.target.checked);
            });
        }

        if (toggleVisualEnhancements && this.config.controller) {
            toggleVisualEnhancements.addEventListener('change', (e) => {
                this.toggleModule('visualEnhancements', e.target.checked);
            });
        }

        if (toggleGrassBasic && this.config.controller) {
            toggleGrassBasic.addEventListener('change', (e) => {
                this.toggleModule('grassFieldBasic', e.target.checked);
            });
        }
    }

    setupAdvancedPanel() {
        const advancedBtn = document.getElementById('toggle-advanced-panel');
        if (advancedBtn) {
            advancedBtn.addEventListener('click', () => {
                this.showAdvancedPanel();
            });
        }
    }

    toggleModule(moduleName, enabled) {
        if (!this.config.controller) return;

        try {
            // Utiliser l'API officielle du contrôleur si disponible
            if (typeof this.config.controller.toggleModule === 'function') {
                this.config.controller.toggleModule(moduleName, enabled);
                log(`📋 Module ${moduleName}: ${enabled ? 'Activé' : 'Désactivé'}`);
            } else {
                // Fallback: essayer d'appeler setEnabled sur le module ou basculer la config
                const module = this.config.controller[moduleName];
                if (module && typeof module.setEnabled === 'function') {
                    module.setEnabled(enabled);
                    console.log(`📋 Module ${moduleName}: ${enabled ? 'Activé' : 'Désactivé'}`);
                } else if (this.config.controller.config) {
                    const configKey = `enable${moduleName.charAt(0).toUpperCase() + moduleName.slice(1)}`;
                    this.config.controller.config[configKey] = enabled;
                    console.log(`📋 Config ${configKey}: ${enabled}`);
                }
            }
            
            this.updateModuleStatus();
        } catch (err) {
            console.error(`❌ Erreur toggle module ${moduleName}:`, err);
        }
    }

    showAdvancedPanel() {
        // Créer panneau avancé en overlay
        if (document.getElementById('advanced-panel')) {
            document.getElementById('advanced-panel').remove();
            return;
        }

        const advancedPanel = document.createElement('div');
        advancedPanel.id = 'advanced-panel';
        advancedPanel.style.cssText = `
            position: fixed;
            top: 50px;
            right: 50px;
            width: 400px;
            height: 500px;
            background: rgba(0, 30, 60, 0.95);
            border: 2px solid #00AAFF;
            border-radius: 10px;
            padding: 20px;
            color: #00FFCC;
            font-family: monospace;
            z-index: 200;
            backdrop-filter: blur(10px);
            overflow-y: auto;
        `;

        advancedPanel.innerHTML = `
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 15px;">
                <h3>🔧 DIAMANTS Advanced Control</h3>
                <button id="close-advanced-panel" style="background: #AA0000; border: none; color: white; padding: 5px 10px; border-radius: 3px; cursor: pointer;">✖</button>
            </div>
            
            <div id="advanced-module-details"></div>
            <div id="advanced-performance-metrics"></div>
            <div id="advanced-system-logs"></div>
        `;

        document.body.appendChild(advancedPanel);

        // Gestionnaire de fermeture
        document.getElementById('close-advanced-panel').addEventListener('click', () => {
            advancedPanel.remove();
        });

        this.updateAdvancedPanel();
    }

    updateModuleStatus() {
        // Mettre à jour le statut des modules dans l'interface
        if (!this.config.controller) return;

        // Mettre à jour les checkboxes des modules
        const modules = [
            'swarmIntelligence', 'dronePhysics', 'adaptivePerformanceEngine', 'networkOptimizer',
            'crossPlatformOptimizer', 'rlNeuralNetwork', 'quantumNeuralNetwork', 
            'pidController', 'flockingBehaviors', 'glslGrassField', 'rosIntegrator',
            'weatherEngine', 'terrainSystem', 'lightingSystem', 'edgeAIProcessor',
            'realtimeMesaIntegrator', 'cloudFormation', 'airTrafficSystem'
        ];

        modules.forEach(moduleName => {
            const checkbox = document.querySelector(`input[data-module="${moduleName}"]`);
            if (checkbox && this.config.controller[moduleName]) {
                const moduleInstance = this.config.controller[moduleName];
                const isActive = moduleInstance.isActive !== undefined ? 
                    moduleInstance.isActive : 
                    (moduleInstance.enabled !== undefined ? moduleInstance.enabled : true);
                checkbox.checked = isActive;
            }
        });

        // Mettre à jour l'affichage général du statut
        const statusDiv = document.getElementById('module-status');
        if (statusDiv && this.config.modules) {
            const activeModules = [];
            Object.keys(this.config.modules).forEach(key => {
                if (this.config.modules[key]) {
                    activeModules.push(key);
                }
            });

            statusDiv.innerHTML = `Active: ${activeModules.length}<br>Modules: ${activeModules.slice(0, 3).join(', ')}${activeModules.length > 3 ? '...' : ''}`;
        }
    }

    updateAdvancedPanel() {
        // Mise à jour du panneau avancé avec détails complets des modules
        const detailsDiv = document.getElementById('advanced-module-details');
        if (!detailsDiv || !this.config.controller) return;

        const modules = [
            { name: 'swarmIntelligence', label: 'Intelligence de Essaim', category: 'AI' },
            { name: 'dronePhysics', label: 'Physique des Drones', category: 'Physics' },
            { name: 'adaptivePerformanceEngine', label: 'Moteur de Performance Adaptatif', category: 'Performance' },
            { name: 'networkOptimizer', label: 'Optimiseur Réseau', category: 'Network' },
            { name: 'crossPlatformOptimizer', label: 'Optimiseur Multi-Plateforme', category: 'Performance' },
            { name: 'rlNeuralNetwork', label: 'Réseau de Neurones RL', category: 'AI' },
            { name: 'quantumNeuralNetwork', label: 'Réseau de Neurones Quantique', category: 'AI' },
            { name: 'pidController', label: 'Contrôleur PID', category: 'Control' },
            { name: 'flockingBehaviors', label: 'Comportements de Vol en Formation', category: 'Behavior' },
            { name: 'glslGrassField', label: 'Champ d\'Herbe GLSL', category: 'Visual' },
            { name: 'rosIntegrator', label: 'Intégrateur ROS', category: 'Integration' },
            { name: 'weatherEngine', label: 'Moteur Météorologique', category: 'Environment' },
            { name: 'terrainSystem', label: 'Système de Terrain', category: 'Environment' },
            { name: 'lightingSystem', label: 'Système d\'Éclairage', category: 'Visual' },
            { name: 'edgeAIProcessor', label: 'Processeur IA Edge', category: 'AI' },
            { name: 'realtimeMesaIntegrator', label: 'Intégrateur Mesa Temps Réel', category: 'Integration' },
            { name: 'cloudFormation', label: 'Formation de Nuages', category: 'Environment' },
            { name: 'airTrafficSystem', label: 'Système de Trafic Aérien', category: 'Control' }
        ];

        let moduleDetails = '<h4>📊 Status Détaillé des Modules</h4>';
        
        // Grouper par catégorie
        const categories = {};
        modules.forEach(module => {
            if (!categories[module.category]) {
                categories[module.category] = [];
            }
            categories[module.category].push(module);
        });

        Object.keys(categories).forEach(category => {
            moduleDetails += `<div class="module-category"><strong>${category}:</strong></div>`;
            
            categories[category].forEach(module => {
                const moduleInstance = this.config.controller[module.name];
                let status, statusClass, details;
                
                if (moduleInstance) {
                    const isActive = moduleInstance.isActive !== undefined ? 
                        moduleInstance.isActive : 
                        (moduleInstance.enabled !== undefined ? moduleInstance.enabled : true);
                    
                    status = isActive ? '✅ Actif' : '⏸️ Inactif';
                    statusClass = isActive ? 'status-active' : 'status-inactive';
                    
                    // Ajouter des détails si disponibles
                    details = '';
                    if (moduleInstance.getStatus && typeof moduleInstance.getStatus === 'function') {
                        try {
                            const moduleStatus = moduleInstance.getStatus();
                            details = ` (${JSON.stringify(moduleStatus).slice(0, 50)})`;
                        } catch (e) {
                            details = ' (status error)';
                        }
                    }
                } else {
                    status = '❌ Non Disponible';
                    statusClass = 'status-unavailable';
                    details = '';
                }
                
                moduleDetails += `
                    <div class="module-status-item ${statusClass}">
                        <span class="module-name">${module.label}:</span>
                        <span class="status">${status}</span>
                        <span class="status-details">${details}</span>
                    </div>
                `;
            });
        });

        detailsDiv.innerHTML = moduleDetails + `
            <style>
                .module-category {
                    margin-top: 10px;
                    color: #00AAFF;
                    border-bottom: 1px solid #00AAFF;
                    padding-bottom: 2px;
                }
                .module-status-item {
                    margin: 2px 0;
                    padding: 2px 0;
                    font-size: 12px;
                    display: flex;
                    justify-content: space-between;
                }
                .status-active { color: #00FF88; }
                .status-inactive { color: #FFAA00; }
                .status-unavailable { color: #FF4444; }
                .status-details { 
                    font-size: 10px; 
                    opacity: 0.7; 
                    max-width: 100px; 
                    overflow: hidden; 
                    text-overflow: ellipsis; 
                }
            </style>
        `;

        // Mettre à jour les métriques de performance si le div existe
        const performanceDiv = document.getElementById('advanced-performance-metrics');
        if (performanceDiv && this.config.controller.getPerformanceMetrics) {
            try {
                const metrics = this.config.controller.getPerformanceMetrics();
                performanceDiv.innerHTML = `
                    <h4>⚡ Métriques de Performance</h4>
                    <div>CPU: ${metrics.cpu || 'N/A'}%</div>
                    <div>Mémoire: ${metrics.memory || 'N/A'} MB</div>
                    <div>FPS: ${metrics.fps || 'N/A'}</div>
                    <div>Modules Actifs: ${metrics.activeModules || 0}</div>
                `;
            } catch (e) {
                performanceDiv.innerHTML = '<h4>⚡ Métriques de Performance</h4><div>Non disponibles</div>';
            }
        }
    }

    createMainInterface() {
        // Interface principale
        const mainUI = document.createElement('div');
        mainUI.id = 'diamants-ui';
        mainUI.className = 'diamants-interface';

        mainUI.innerHTML = `
            <div class="ui-header">
                <h2>🚁 DIAMANTS - Intelligence Collective</h2>
                <div class="system-status" id="system-status">
                    <span class="status-indicator" id="status-indicator">●</span>
                    <span id="status-text">Initialisation...</span>
                </div>
            </div>
            
            <div class="ui-body">
                <div class="left-panel">
                    <div id="mission-control"></div>
                    <div id="swarm-control"></div>
                    <div id="diamants-metrics"></div>
                </div>
                
                <div class="center-panel">
                    <div id="main-display">
                        <!-- Zone de rendu 3D -->
                    </div>
                    <div id="controls-overlay"></div>
                </div>
                
                <div class="right-panel">
                    <div id="drone-status"></div>
                    <div id="environment-info"></div>
                    <div id="emergency-controls"></div>
                </div>
            </div>
            
            <div class="ui-footer">
                <div id="formula-display"></div>
                <div id="performance-metrics"></div>
            </div>
        `;

        // Styles CSS intégrés
        const styles = `
            <style>
                .diamants-interface {
                    position: fixed;
                    top: 0;
                    left: 0;
                    width: 100%;
                    height: 100%;
                    font-family: 'Roboto Mono', 'Courier New', monospace;
                    background: linear-gradient(135deg, #0a0a0a, #1a1a2e);
                    color: #00FFCC;
                    overflow: hidden;
                    z-index: 1000;
                }
                
                .ui-header {
                    height: 60px;
                    background: rgba(0, 20, 40, 0.9);
                    backdrop-filter: blur(10px);
                    border-bottom: 2px solid #00AAFF;
                    display: flex;
                    align-items: center;
                    justify-content: space-between;
                    padding: 0 20px;
                }
                
                .ui-header h2 {
                    margin: 0;
                    color: #00FFCC;
                    text-shadow: 0 0 10px #00FFCC50;
                }
                
                .system-status {
                    display: flex;
                    align-items: center;
                    gap: 10px;
                }
                
                .status-indicator {
                    font-size: 20px;
                    color: #00FF00;
                    text-shadow: 0 0 10px #00FF0080;
                    animation: pulse 2s infinite;
                }
                
                @keyframes pulse {
                    0%, 100% { opacity: 1; }
                    50% { opacity: 0.5; }
                }
                
                .ui-body {
                    height: calc(100% - 120px);
                    display: flex;
                }
                
                .left-panel, .right-panel {
                    width: 300px;
                    background: rgba(0, 20, 40, 0.8);
                    backdrop-filter: blur(5px);
                    border: 1px solid #00AAFF40;
                    padding: 15px;
                    overflow-y: auto;
                }
                
                .center-panel {
                    flex: 1;
                    position: relative;
                    background: linear-gradient(45deg, #001122, #002244);
                }
                
                .ui-footer {
                    height: 60px;
                    background: rgba(0, 20, 40, 0.9);
                    border-top: 2px solid #00AAFF;
                    display: flex;
                    align-items: center;
                    justify-content: space-between;
                    padding: 0 20px;
                }
                
                .panel-section {
                    background: rgba(0, 40, 80, 0.3);
                    border: 1px solid #00AAFF30;
                    border-radius: 8px;
                    padding: 15px;
                    margin-bottom: 15px;
                }
                
                .panel-section h3 {
                    margin: 0 0 15px 0;
                    color: #00FFAA;
                    font-size: 16px;
                    border-bottom: 1px solid #00FFAA40;
                    padding-bottom: 5px;
                }
                
                .metric-item {
                    display: flex;
                    justify-content: space-between;
                    margin-bottom: 8px;
                    font-size: 14px;
                }
                
                .metric-label {
                    color: #AAFFCC;
                }
                
                .metric-value {
                    color: #00FFFF;
                    font-weight: bold;
                    text-shadow: 0 0 5px #00FFFF50;
                }
                
                .control-button {
                    background: linear-gradient(45deg, #0066AA, #0088CC);
                    border: 1px solid #00AAFF;
                    color: white;
                    padding: 10px 15px;
                    margin: 5px;
                    border-radius: 5px;
                    cursor: pointer;
                    font-family: inherit;
                    font-size: 13px;
                    transition: all 0.3s ease;
                }
                
                .control-button:hover {
                    background: linear-gradient(45deg, #0088CC, #00AAFF);
                    box-shadow: 0 0 15px #00AAFF50;
                    transform: translateY(-2px);
                }
                
                .control-button.emergency {
                    background: linear-gradient(45deg, #AA0000, #CC2200);
                    border-color: #FF4400;
                }
                
                .control-button.emergency:hover {
                    background: linear-gradient(45deg, #CC2200, #FF4400);
                    box-shadow: 0 0 15px #FF440050;
                }
                
                .progress-bar {
                    width: 100%;
                    height: 20px;
                    background: rgba(0, 0, 0, 0.5);
                    border-radius: 10px;
                    overflow: hidden;
                    margin: 5px 0;
                }
                
                .progress-fill {
                    height: 100%;
                    background: linear-gradient(90deg, #00AA88, #00FFCC);
                    border-radius: 10px;
                    transition: width 0.5s ease;
                    box-shadow: 0 0 10px #00FFCC50;
                }
                
                .drone-item {
                    background: rgba(0, 60, 120, 0.2);
                    border: 1px solid #0088FF40;
                    border-radius: 5px;
                    padding: 10px;
                    margin-bottom: 8px;
                }
                
                .drone-id {
                    font-weight: bold;
                    color: #00FFFF;
                }
                
                .drone-state {
                    font-size: 12px;
                    color: #AAFFCC;
                }
                
                .formula-display {
                    font-size: 18px;
                    color: #FFAA00;
                    text-shadow: 0 0 10px #FFAA0050;
                }
                
                .controls-overlay {
                    position: absolute;
                    bottom: 20px;
                    left: 20px;
                    background: rgba(0, 20, 40, 0.9);
                    border-radius: 10px;
                    padding: 15px;
                    border: 1px solid #00AAFF40;
                }
                
                .minimap {
                    position: absolute;
                    top: 20px;
                    right: 20px;
                    width: 200px;
                    height: 150px;
                    background: rgba(0, 20, 40, 0.9);
                    border: 2px solid #00AAFF;
                    border-radius: 8px;
                }
                
                .alert-message {
                    position: absolute;
                    top: 50%;
                    left: 50%;
                    transform: translate(-50%, -50%);
                    background: rgba(255, 0, 0, 0.9);
                    color: white;
                    padding: 20px;
                    border-radius: 10px;
                    font-size: 18px;
                    z-index: 2000;
                    display: none;
                }
                
                input[type="range"] {
                    width: 100%;
                    margin: 10px 0;
                    accent-color: #00AAFF;
                }
                
                select {
                    background: rgba(0, 40, 80, 0.8);
                    border: 1px solid #00AAFF;
                    color: #00FFCC;
                    padding: 8px;
                    border-radius: 4px;
                    width: 100%;
                    margin: 5px 0;
                }
            </style>
        `;

        document.head.insertAdjacentHTML('beforeend', styles);
        document.body.appendChild(mainUI);

        this.elements.main = mainUI;
    }

    createControlPanels() {
        // Panel de contrôle de mission
        const missionControl = document.getElementById('mission-control');
        missionControl.innerHTML = `
            <div class="panel-section">
                <h3>🎯 Contrôle Mission</h3>
                
                <div class="metric-item">
                    <span class="metric-label">Mission Active:</span>
                    <span class="metric-value" id="active-mission">Aucune</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Progression:</span>
                    <span class="metric-value" id="mission-progress">0%</span>
                </div>
                
                <div class="progress-bar">
                    <div class="progress-fill" id="mission-progress-bar" style="width: 0%"></div>
                </div>
                
                <select id="mission-type">
                    <option value="EXPLORATION">🔍 Exploration</option>
                    <option value="SEARCH_RESCUE">🚑 Recherche & Sauvetage</option>
                    <option value="FORMATION_FLIGHT">✈️  Vol en Formation</option>
                    <option value="SURVEILLANCE">👁️ Surveillance</option>
                    <option value="MAPPING">🗺️ Cartographie</option>
                </select>
                
                <button class="control-button" id="start-mission">🚀 Démarrer Mission</button>
                <button class="control-button" id="abort-mission">⏹️ Arrêter Mission</button>
            </div>
        `;

        // Panel de contrôle essaim
        const swarmControl = document.getElementById('swarm-control');
        swarmControl.innerHTML = `
            <div class="panel-section">
                <h3>🤖 Contrôle Essaim</h3>
                
                <div class="metric-item">
                    <span class="metric-label">Drones Actifs:</span>
                    <span class="metric-value" id="active-drones">0</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Formation:</span>
                    <span class="metric-value" id="current-formation">Libre</span>
                </div>
                
                <select id="drone-type">
                    <option value="SCOUT">🔍 Scout</option>
                    <option value="HEAVY">🚚 Heavy</option>
                    <option value="STEALTH">👻 Stealth</option>
                    <option value="LEADER">👑 Leader</option>
                </select>
                
                <button class="control-button" id="add-drone">➕ Ajouter Drone</button>
                <button class="control-button" id="formation-toggle">🔄 Formation</button>
                <button class="control-button emergency" id="emergency-land">🛑 Atterrissage d'Urgence</button>
            </div>
        `;
    }

    createMetricsDisplay() {
        const metricsPanel = document.getElementById('diamants-metrics');
        metricsPanel.innerHTML = `
            <div class="panel-section">
                <h3>📊 Métriques DIAMANTS</h3>
                
                <div class="metric-item">
                    <span class="metric-label">Intelligence I(t):</span>
                    <span class="metric-value" id="intelligence-value">0.00</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Gradient |∇|:</span>
                    <span class="metric-value" id="gradient-magnitude">0.00</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Phi (φ):</span>
                    <span class="metric-value" id="phi-value">0.00</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Sigma (σ):</span>
                    <span class="metric-value" id="sigma-value">0.00</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Émergence:</span>
                    <span class="metric-value" id="emergence-value">0.00</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Cohérence:</span>
                    <span class="metric-value" id="coherence-value">0.00</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Couverture:</span>
                    <span class="metric-value" id="coverage-value">0%</span>
                </div>
                
                <div class="progress-bar">
                    <div class="progress-fill" id="coverage-progress" style="width: 0%"></div>
                </div>
            </div>
            
            <div class="panel-section" id="advanced-metrics" style="display: none;">
                <h3>🔬 Métriques Avancées</h3>
                
                <div class="metric-item">
                    <span class="metric-label">H1 (Externe):</span>
                    <span class="metric-value" id="h1-value">0.00</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">H2 (Interne):</span>
                    <span class="metric-value" id="h2-value">0.00</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">H3 (Énergie):</span>
                    <span class="metric-value" id="h3-value">0.00</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">H13 (Cohérence):</span>
                    <span class="metric-value" id="h13-value">0.00</span>
                </div>
                
                <button class="control-button" id="toggle-advanced">📈 Harmoniques</button>
            </div>
        `;
    }

    createMissionControl() {
        const controlsOverlay = document.getElementById('controls-overlay');
        controlsOverlay.innerHTML = `
            <div class="controls-overlay">
                <h4>🎮 Contrôles</h4>
                <div style="font-size: 12px; line-height: 1.4;">
                    <div><strong>Souris:</strong> Orbit/Pan/Zoom</div>
                    <div><strong>WASD:</strong> Déplacement caméra</div>
                    <div><strong>QE:</strong> Montée/Descente</div>
                    <div><strong>Espace:</strong> Reset caméra</div>
                    <div><strong>R:</strong> Reset simulation</div>
                </div>
                
                <div style="margin-top: 10px;">
                    <label>Vitesse Simulation:</label>
                    <input type="range" id="sim-speed" min="0.1" max="3" step="0.1" value="1">
                    <span id="sim-speed-value">1.0x</span>
                </div>
            </div>
        `;
    }

    createVisualizationControls() {
        const rightPanel = document.getElementById('drone-status');
        rightPanel.innerHTML = `
            <div class="panel-section">
                <h3>🚁 État des Drones</h3>
                <div id="drone-list"></div>
            </div>
            
            <div class="panel-section">
                <h3>🌍 Environnement</h3>
                
                <div class="metric-item">
                    <span class="metric-label">Obstacles:</span>
                    <span class="metric-value" id="obstacle-count">0</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Zone Explorée:</span>
                    <span class="metric-value" id="explored-area">0 m²</span>
                </div>
                
                <div class="metric-item">
                    <span class="metric-label">Découvertes:</span>
                    <span class="metric-value" id="discoveries-count">0</span>
                </div>
                
                <label>Visualisation:</label>
                <select id="visualization-mode">
                    <option value="normal">🎨 Normal</option>
                    <option value="field">⚡ Champs DIAMANTS</option>
                    <option value="gradient">📊 Gradients</option>
                    <option value="intelligence">🧠 Intelligence</option>
                    <option value="coverage">🗺️ Couverture</option>
                </select>
                
                <button class="control-button" id="toggle-minimap">🗺️ Minimap</button>
                <button class="control-button" id="toggle-trails">✨ Trajectoires</button>
            </div>
        `;

        // Création de la minimap
        const centerPanel = document.getElementById('center-panel');
        if (centerPanel) {
            const minimap = document.createElement('div');
            minimap.className = 'minimap';
            minimap.id = 'minimap';
            minimap.innerHTML = `
                <canvas id="minimap-canvas" width="196" height="146"></canvas>
            `;
            centerPanel.appendChild(minimap);
        }
    }

    setupEventHandlers() {
        // Gestionnaires d'événements pour les contrôles

        // Contrôles de mission
        const startMissionBtn = document.getElementById('start-mission');
        if (startMissionBtn) {
            startMissionBtn.addEventListener('click', () => {
                this.triggerCallback('startMission');
            });
        }

        const abortMissionBtn = document.getElementById('abort-mission');
        if (abortMissionBtn) {
            abortMissionBtn.addEventListener('click', () => {
                this.triggerCallback('abortMission');
            });
        }

        // Contrôles d'essaim
        const addDroneBtn = document.getElementById('add-drone');
        if (addDroneBtn) {
            addDroneBtn.addEventListener('click', () => {
                const droneType = document.getElementById('drone-type').value;
                this.triggerCallback('addDrone', { type: droneType });
            });
        }

        const emergencyLandBtn = document.getElementById('emergency-land');
        if (emergencyLandBtn) {
            emergencyLandBtn.addEventListener('click', () => {
                this.triggerCallback('emergencyLand');
            });
        }

        // Contrôles de visualisation
        const visualizationMode = document.getElementById('visualization-mode');
        if (visualizationMode) {
            visualizationMode.addEventListener('change', (e) => {
                this.triggerCallback('changeVisualization', { mode: e.target.value });
            });
        }

        const simSpeed = document.getElementById('sim-speed');
        if (simSpeed) {
            simSpeed.addEventListener('input', (e) => {
                const speed = parseFloat(e.target.value);
                document.getElementById('sim-speed-value').textContent = speed.toFixed(1) + 'x';
                this.triggerCallback('changeSimSpeed', { speed });
            });
        }

        // Toggle pour métriques avancées
        const toggleAdvanced = document.getElementById('toggle-advanced');
        if (toggleAdvanced) {
            toggleAdvanced.addEventListener('click', () => {
                const advancedPanel = document.getElementById('advanced-metrics');
                const isVisible = advancedPanel.style.display !== 'none';
                advancedPanel.style.display = isVisible ? 'none' : 'block';
                toggleAdvanced.textContent = isVisible ? '📈 Afficher Harmoniques' : '📉 Masquer Harmoniques';
            });
        }

        // Raccourcis clavier
        document.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;

            switch (e.key.toLowerCase()) {
                case 'r':
                    this.triggerCallback('resetSimulation');
                    break;
                case ' ':
                    e.preventDefault();
                    this.triggerCallback('resetCamera');
                    break;
                case 'escape':
                    this.triggerCallback('emergencyLand');
                    break;
            }
        });
    }

    /**
     * Mise à jour des métriques DIAMANTS
     */
    updateDiamantsMetrics(metrics) {
        this.metrics = { ...this.metrics, ...metrics };

        // Métriques principales
        this.updateElement('intelligence-value', metrics.diamants?.toFixed(3) || '0.000');
        this.updateElement('gradient-magnitude', metrics.gradient_mean?.toFixed(3) || '0.000');
        this.updateElement('phi-value', metrics.phi_mean?.toFixed(3) || '0.000');
        this.updateElement('sigma-value', metrics.sigma_mean?.toFixed(3) || '0.000');
        this.updateElement('emergence-value', metrics.emergence?.toFixed(3) || '0.000');
        this.updateElement('coherence-value', metrics.coherence?.toFixed(3) || '0.000');

        // Harmoniques avancées
        if (metrics.harmonics) {
            this.updateElement('h1-value', metrics.harmonics[0]?.toFixed(3) || '0.000');
            this.updateElement('h2-value', metrics.harmonics[1]?.toFixed(3) || '0.000');
            this.updateElement('h3-value', metrics.harmonics[2]?.toFixed(3) || '0.000');
            this.updateElement('h13-value', metrics.harmonics[12]?.toFixed(3) || '0.000');
        }
    }

    /**
     * Mise à jour du statut des drones
     */
    updateDroneStatus(drones) {
        const droneList = document.getElementById('drone-list');
        if (!droneList) return;

        droneList.innerHTML = '';

        drones.forEach(drone => {
            const status = drone.getStatus ? drone.getStatus() : {};
            const droneElement = document.createElement('div');
            droneElement.className = 'drone-item';

            const stateColor = {
                'IDLE': '#888888',
                'TAKEOFF': '#00FF00',
                'FLYING': '#0088FF',
                'LANDING': '#FFAA00',
                'EMERGENCY': '#FF0000'
            }[status.state] || '#888888';

            droneElement.innerHTML = `
                <div class="drone-id" style="color: ${stateColor}">
                    ${drone.id} (${drone.type})
                </div>
                <div class="drone-state">
                    État: ${status.state || 'UNKNOWN'}<br>
                    Position: (${status.position?.[0]?.toFixed(1) || '0'}, 
                              ${status.position?.[1]?.toFixed(1) || '0'}, 
                              ${status.position?.[2]?.toFixed(1) || '0'})<br>
                    Intelligence: ${status.intelligence?.toFixed(3) || '0.000'}<br>
                    Batterie: ${((status.batteryLevel || 0) * 100).toFixed(0)}%
                </div>
            `;

            droneList.appendChild(droneElement);
        });

        this.updateElement('active-drones', drones.length.toString());
    }

    /**
     * Mise à jour du statut des missions
     */
    updateMissionStatus(missionManager) {
        if (!missionManager) return;

        const activeMissions = missionManager.getAllMissionsStatus();
        const currentMission = activeMissions[0];

        if (currentMission) {
            this.updateElement('active-mission', currentMission.type);

            const progress = currentMission.progress?.coverage || 0;
            this.updateElement('mission-progress', (progress * 100).toFixed(1) + '%');

            const progressBar = document.getElementById('mission-progress-bar');
            if (progressBar) {
                progressBar.style.width = (progress * 100) + '%';
            }
        } else {
            this.updateElement('active-mission', 'Aucune');
            this.updateElement('mission-progress', '0%');

            const progressBar = document.getElementById('mission-progress-bar');
            if (progressBar) {
                progressBar.style.width = '0%';
            }
        }

        // Mise à jour des métriques d'environnement
        const swarmData = missionManager.getSwarmData();
        this.updateElement('explored-area', (swarmData.coverage * 1).toFixed(0) + ' m²');
        this.updateElement('discoveries-count', swarmData.discoveries.toString());
    }

    /**
     * Mise à jour de la couverture
     */
    updateCoverage(coveragePercent) {
        this.updateElement('coverage-value', (coveragePercent * 100).toFixed(1) + '%');

        const coverageProgress = document.getElementById('coverage-progress');
        if (coverageProgress) {
            coverageProgress.style.width = (coveragePercent * 100) + '%';
        }
    }

    /**
     * Affichage d'alertes
     */
    showAlert(message, type = 'info', duration = 3000) {
        let alertElement = document.getElementById('alert-message');

        if (!alertElement) {
            alertElement = document.createElement('div');
            alertElement.id = 'alert-message';
            alertElement.className = 'alert-message';
            document.body.appendChild(alertElement);
        }

        const colors = {
            'info': 'rgba(0, 150, 255, 0.9)',
            'warning': 'rgba(255, 150, 0, 0.9)',
            'error': 'rgba(255, 50, 50, 0.9)',
            'success': 'rgba(0, 255, 100, 0.9)'
        };

        alertElement.style.background = colors[type] || colors.info;
        alertElement.textContent = message;
        alertElement.style.display = 'block';

        setTimeout(() => {
            alertElement.style.display = 'none';
        }, duration);
    }

    /**
     * Mise à jour du statut système
     */
    updateSystemStatus(status, text) {
        const indicator = document.getElementById('status-indicator');
        const statusText = document.getElementById('status-text');

        if (indicator && statusText) {
            const colors = {
                'ready': '#00FF00',
                'active': '#00AAFF',
                'warning': '#FFAA00',
                'error': '#FF0000',
                'offline': '#888888'
            };

            indicator.style.color = colors[status] || colors.offline;
            statusText.textContent = text || status.toUpperCase();
        }
    }

    /**
     * Mise à jour de la formule DIAMANTS
     */
    updateFormula(formula = "I(t) = ∬Ω |∇(φ+σ)| dΩ") {
        const formulaDisplay = document.getElementById('formula-display');
        if (formulaDisplay) {
            formulaDisplay.innerHTML = `
                <div class="formula-display">
                    <strong>${formula}</strong> - Intelligence Émergente DIAMANTS
                </div>
            `;
        }
    }

    /**
     * Gestionnaire de callbacks
     */
    onCallback(event, callback) {
        this.updateCallbacks.set(event, callback);
    }

    triggerCallback(event, data = null) {
        const callback = this.updateCallbacks.get(event);
        if (callback) {
            callback(data);
        }
    }

    /**
     * Méthodes utilitaires
     */
    updateElement(id, value) {
        const element = document.getElementById(id);
        if (element) {
            element.textContent = value;
        }
    }

    setMissionType(type) {
        const select = document.getElementById('mission-type');
        if (select) {
            select.value = type;
        }
    }

    setVisualizationMode(mode) {
        const select = document.getElementById('visualization-mode');
        if (select) {
            select.value = mode;
        }
    }

    /**
     * Mise à jour de l'interface
     */
    update(drones, metrics, mission) {
        try {
            if (!this.isInitialized || !this.elements.main) return;

            // Mise à jour des métriques uniquement si disponibles
            if (metrics && this.elements.status) {
                this.elements.status.innerHTML = `
                    Drones: ${drones ? drones.length : 0} | 
                    Intelligence: ${metrics.intelligence ? metrics.intelligence.toFixed(2) : '0.00'} |
                    Mission: ${mission ? mission.type || 'Aucune' : 'Aucune'}
                `;
            }
        } catch (error) {
            warn('Erreur UI update:', error);
        }
    }

    /**
     * Destruction de l'interface
     */
    destroy() {
        if (this.elements.main) {
            this.elements.main.remove();
        }
        this.updateCallbacks.clear();
        this.isInitialized = false;

        // Nettoyer la mini-map
        if (this.minimap.canvas) {
            this.minimap.canvas.remove();
        }

        log('🗑️ Interface DIAMANTS UI détruite');
    }

    /**
     * Création de la mini-map tactique
     */
    createTacticalMinimap() {
        try {
            const minimapContainer = document.createElement('div');
            minimapContainer.id = 'tactical-minimap';
            minimapContainer.style.cssText = `
                position: fixed;
                top: 10px;
                right: 10px;
                width: ${this.minimap.width + 20}px;
                height: ${this.minimap.height + 60}px;
                background: rgba(10, 20, 40, 0.9);
                border: 2px solid #00ff88;
                border-radius: 8px;
                padding: 10px;
                z-index: 200;
                backdrop-filter: blur(10px);
            `;

            const title = document.createElement('div');
            title.textContent = '🗺️ Mission Tactical Map';
            title.style.cssText = `
                color: #00ff88;
                font-family: 'Roboto Mono', monospace;
                font-size: 12px;
                font-weight: bold;
                margin-bottom: 5px;
                text-align: center;
            `;

            this.minimap.canvas = document.createElement('canvas');
            this.minimap.canvas.width = this.minimap.width;
            this.minimap.canvas.height = this.minimap.height;
            this.minimap.ctx = this.minimap.canvas.getContext('2d');

            this.minimap.canvas.style.cssText = `
                border: 1px solid #00ff88;
                border-radius: 4px;
            `;

            const info = document.createElement('div');
            info.id = 'minimap-info';
            info.style.cssText = `
                color: #00ffcc;
                font-family: monospace;
                font-size: 10px;
                margin-top: 5px;
                text-align: center;
            `;
            info.innerHTML = 'Coverage: 0% | Targets: 0/0';

            minimapContainer.appendChild(title);
            minimapContainer.appendChild(this.minimap.canvas);
            minimapContainer.appendChild(info);

            document.body.appendChild(minimapContainer);
            this.elements.minimapContainer = minimapContainer;
            this.elements.minimapInfo = info;

            log('🗺️ Mini-map tactique créée');
        } catch (error) {
            warn('⚠️ Erreur création mini-map:', error.message);
        }
    }

    /**
     * Mise à jour de la mini-map tactique
     */
    updateTacticalMinimap(drones = [], targets = [], explorationMap = null) {
        if (!this.minimap.canvas || !this.minimap.ctx || !this.minimap.visible) return;

        try {
            const ctx = this.minimap.ctx;
            const width = this.minimap.canvas.width;
            const height = this.minimap.canvas.height;
            const scale = this.minimap.scale;
            const centerX = width / 2;
            const centerY = height / 2;

            // Fond style mission tactique
            const gradient = ctx.createRadialGradient(centerX, centerY, 0, centerX, centerY, Math.max(width, height) / 2);
            gradient.addColorStop(0, '#1a1a2e');
            gradient.addColorStop(0.5, '#16213e');
            gradient.addColorStop(1, '#0f3460');
            ctx.fillStyle = gradient;
            ctx.fillRect(0, 0, width, height);

            // Bordure de mission
            ctx.strokeStyle = '#00ff88';
            ctx.lineWidth = 2;
            ctx.strokeRect(2, 2, width - 4, height - 4);

            // Grille tactique
            ctx.strokeStyle = 'rgba(0, 255, 136, 0.3)';
            ctx.lineWidth = 1;
            ctx.beginPath();

            for (let i = -25; i <= 25; i += 5) {
                const x = centerX + i * scale / 2;
                const y = centerY + i * scale / 2;
                if (x >= 0 && x <= width) {
                    ctx.moveTo(x, 0);
                    ctx.lineTo(x, height);
                }
                if (y >= 0 && y <= height) {
                    ctx.moveTo(0, y);
                    ctx.lineTo(width, y);
                }
            }
            ctx.stroke();

            // Axes de référence
            ctx.strokeStyle = '#00ff88';
            ctx.lineWidth = 1;
            ctx.setLineDash([5, 5]);
            ctx.beginPath();
            ctx.moveTo(centerX, 0);
            ctx.lineTo(centerX, height);
            ctx.moveTo(0, centerY);
            ctx.lineTo(width, centerY);
            ctx.stroke();
            ctx.setLineDash([]);

            // Drones avec informations tactiques
            if (drones && drones.length > 0) {
                drones.forEach((drone) => {
                    if (!drone.position) return;

                    const x = centerX + drone.position.x * scale;
                    const y = centerY - drone.position.z * scale;

                    // Couleur selon rôle et état
                    let droneColor = '#00aa00'; // Par défaut vert
                    if (drone.state === 'EMERGENCY') {
                        droneColor = '#ff0000';
                    } else if (drone.type === 'LEADER') {
                        droneColor = '#ffd700'; // Or pour les leaders
                    } else if (drone.type === 'SCOUT') {
                        droneColor = '#00ffff'; // Cyan pour les scouts
                    } else if (drone.type === 'HEAVY') {
                        droneColor = '#ff8800'; // Orange pour les lourds
                    }

                    // Corps du drone
                    ctx.fillStyle = droneColor;
                    ctx.beginPath();
                    ctx.arc(x, y, 3, 0, 2 * Math.PI);
                    ctx.fill();

                    // Bordure selon état
                    ctx.strokeStyle = drone.state === 'EMERGENCY' ? '#ffffff' : '#000000';
                    ctx.lineWidth = 1;
                    ctx.stroke();

                    // ID du drone
                    ctx.fillStyle = '#ffffff';
                    ctx.font = 'bold 8px Arial';
                    ctx.textAlign = 'center';
                    ctx.fillText(drone.id?.toString() || '?', x, y + 2);
                });
            }

            // Cibles de mission
            if (targets && targets.length > 0) {
                targets.forEach((target, idx) => {
                    if (!target.position) return;

                    const x = centerX + target.position.x * scale;
                    const y = centerY - target.position.z * scale;

                    if (target.discovered) {
                        ctx.fillStyle = '#00ff00';
                        ctx.beginPath();
                        ctx.arc(x, y, 2, 0, 2 * Math.PI);
                        ctx.fill();
                    } else {
                        ctx.fillStyle = target.type === 'emergency' ? '#ff6b6b' :
                            target.type === 'resource' ? '#feca57' : '#48dbfb';
                        ctx.beginPath();
                        ctx.arc(x, y, 1.5, 0, 2 * Math.PI);
                        ctx.fill();
                    }
                });
            }

            // Informations de mission
            if (this.elements.minimapInfo) {
                const discovered = targets ? targets.filter(t => t.discovered).length : 0;
                const total = targets ? targets.length : 0;
                const coverage = explorationMap ? this.calculateCoverage(explorationMap) : 0;

                this.elements.minimapInfo.innerHTML =
                    `Coverage: ${coverage.toFixed(1)}% | Targets: ${discovered}/${total}`;
            }

        } catch (error) {
            warn('⚠️ Erreur mise à jour mini-map:', error.message);
        }
    }

    /**
     * Calcul du pourcentage de couverture
     */
    calculateCoverage(explorationMap) {
        if (!explorationMap || !Array.isArray(explorationMap)) return 0;

        let totalCells = 0;
        let coveredCells = 0;

        for (let i = 0; i < explorationMap.length; i++) {
            if (Array.isArray(explorationMap[i])) {
                for (let j = 0; j < explorationMap[i].length; j++) {
                    totalCells++;
                    if (explorationMap[i][j] > 0) {
                        coveredCells++;
                    }
                }
            }
        }

        return totalCells > 0 ? (coveredCells / totalCells) * 100 : 0;
    }

    /**
     * Mise à jour du texte du bouton de logs
     */
    updateLogButtonText(button) {
        if (window.SILENT_MODE) {
            button.textContent = '🔇 Logs: Silent';
            button.style.backgroundColor = '#2a4a5a';
        } else {
            button.textContent = '🔊 Logs: Verbose';
            button.style.backgroundColor = '#5a4a2a';
        }
    }
}

export default DiamantUI;
