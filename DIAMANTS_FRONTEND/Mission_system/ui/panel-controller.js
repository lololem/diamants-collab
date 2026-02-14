/**
 * DIAMANTS - Panel Controller Robuste
 * =====================================
 * Câblage complet de tous les boutons et fonctions du panneau de contrôle
 * Avec validation et feedback utilisateur
 */

import { getDoctrineManager, DOCTRINES, COURSES_OF_ACTION } from '../missions/mission-doctrine.js';

const log = (...args) => console.log('[PanelCtrl]', ...args);
const warn = (...args) => console.warn('[PanelCtrl]', ...args);

/**
 * Panel Controller - Gère toutes les interactions UI
 */
export class PanelController {
    constructor() {
        this.isInitialized = false;
        this.doctrineManager = null;
        this.feedbackTimeout = null;
        
        // État des boutons
        this.buttonStates = new Map();
        
        // Fonctions globales câblées
        this.globalFunctions = {};
        
        this.init();
    }
    
    async init() {
        // Attendre que le DOM soit prêt
        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', () => this.setup());
        } else {
            this.setup();
        }
    }
    
    setup() {
        log('🔧 Initialisation Panel Controller...');
        
        // Initialiser le gestionnaire de doctrine
        this.doctrineManager = getDoctrineManager();
        
        // Câbler tous les boutons
        this.wireAllButtons();
        
        // Câbler les selects
        this.wireSelects();
        
        // Câbler les sliders
        this.wireSliders();
        
        // Exposer les fonctions globales
        this.exposeGlobalFunctions();
        
        // Valider l'état initial
        this.validateState();
        
        this.isInitialized = true;
        log('✅ Panel Controller initialisé');
        
        // Feedback visuel
        this.showFeedback('Système prêt', 'success');
    }
    
    /**
     * Câbler TOUS les boutons
     */
    wireAllButtons() {
        // === MISSION CONTROL ===
        this.bindButton('launchMission', () => this.handleLaunch(), 'Launch Mission');
        this.bindButton('emergencyLand', () => this.handleStop(), 'Emergency Stop');
        this.bindButton('takeoffAllDrones', () => this.handleTakeoff(), 'Takeoff All');
        this.bindButton('landAllDrones', () => this.handleLand(), 'Land All');
        
        // === CAMERA CONTROLS ===
        this.bindButton('resetCamera', () => this.handleResetCamera(), 'Reset Camera');
        this.bindButton('topView', () => this.handleTopView(), 'Top View');
        this.bindButton('zoomToSwarm', () => this.handleZoomToSwarm(), 'Zoom to Swarm');
        this.bindButton('toggleFollowMode', () => this.handleToggleFollow(), 'Toggle Follow');
        
        // === DISPLAY CONTROLS ===
        this.bindButton('toggleMinimap', () => this.handleToggleMinimap(), 'Toggle Minimap');
        this.bindButton('toggleDebugPanels', () => this.handleToggleDebug(), 'Toggle Debug');
        this.bindButton('togglePanel', () => this.handleTogglePanel(), 'Toggle Panel');
        
        // === BENCHMARK ===
        this.bindButtonByQuery('[onclick*="runQuickBenchmark"]', () => this.handleBenchmark(30), 'Quick Benchmark');
        
        log(`📎 ${this.buttonStates.size} boutons câblés`);
    }
    
    /**
     * Liaison générique d'un bouton par onclick
     */
    bindButton(fnName, handler, description) {
        // Trouver tous les boutons qui appellent cette fonction
        const buttons = document.querySelectorAll(`[onclick*="${fnName}"]`);
        
        buttons.forEach(btn => {
            // Supprimer l'ancien onclick
            const oldOnclick = btn.getAttribute('onclick');
            btn.removeAttribute('onclick');
            
            // Ajouter data-action pour les tests
            btn.setAttribute('data-action', fnName);
            
            // Ajouter le nouveau handler
            btn.addEventListener('click', (e) => {
                e.preventDefault();
                this.executeWithFeedback(handler, description, btn);
            });
            
            // Marquer comme câblé
            this.buttonStates.set(fnName, {
                elements: buttons,
                handler,
                description
            });
        });
        
        // Exposer aussi comme fonction globale
        window[fnName] = () => this.executeWithFeedback(handler, description);
    }
    
    /**
     * Liaison par query selector
     */
    bindButtonByQuery(selector, handler, description) {
        const buttons = document.querySelectorAll(selector);
        buttons.forEach(btn => {
            btn.removeAttribute('onclick');
            btn.addEventListener('click', (e) => {
                e.preventDefault();
                this.executeWithFeedback(handler, description, btn);
            });
        });
    }
    
    /**
     * Exécuter avec feedback visuel
     */
    executeWithFeedback(handler, description, btnElement = null) {
        try {
            // Feedback bouton
            if (btnElement) {
                btnElement.classList.add('btn-active');
                setTimeout(() => btnElement.classList.remove('btn-active'), 200);
            }
            
            // Exécuter
            const result = handler();
            
            // Feedback réussite
            if (result !== false) {
                log(`✅ ${description}`);
            }
            
            return result;
        } catch (error) {
            warn(`❌ Erreur ${description}:`, error.message);
            this.showFeedback(`Erreur: ${error.message}`, 'error');
            return false;
        }
    }
    
    /**
     * Câbler les selects
     */
    wireSelects() {
        // Mission Type -> Doctrine
        const missionType = document.getElementById('mission_type');
        if (missionType) {
            missionType.addEventListener('change', (e) => {
                const doctrine = e.target.value;
                this.doctrineManager?.setDoctrine(doctrine);
                this.showFeedback(`Doctrine: ${doctrine}`, 'info');
            });
            log('📎 Select mission_type câblé');
        }
        
        // Mode Select -> COA
        const modeSelect = document.getElementById('mode_select');
        if (modeSelect) {
            modeSelect.addEventListener('change', (e) => {
                const coa = e.target.value;
                this.doctrineManager?.setCOA(coa);
                this.showFeedback(`Pattern: ${coa}`, 'info');
            });
            log('📎 Select mode_select câblé');
        }
    }
    
    /**
     * Câbler les sliders
     */
    wireSliders() {
        // Altitude
        const altitude = document.getElementById('altitude_slider');
        if (altitude) {
            altitude.addEventListener('input', (e) => {
                const val = parseFloat(e.target.value);
                const display = document.getElementById('altitude_display');
                if (display) display.textContent = `${val.toFixed(1)}m`;
                
                // Appliquer aux drones
                this.applyAltitude(val);
            });
            log('📎 Slider altitude câblé');
        }
        
        // Safety distance
        const safety = document.getElementById('safety_distance');
        if (safety) {
            safety.addEventListener('input', (e) => {
                const val = parseFloat(e.target.value);
                const display = document.getElementById('safety_distance_value');
                if (display) display.textContent = `${val.toFixed(1)}m`;
                
                this.applySafetyDistance(val);
            });
            log('📎 Slider safety_distance câblé');
        }
    }
    
    /**
     * Exposer les fonctions globales
     */
    exposeGlobalFunctions() {
        // Fonctions principales
        window.launchMission = () => this.handleLaunch();
        window.emergencyLand = () => this.handleStop();
        window.takeoffAllDrones = () => this.handleTakeoff();
        window.landAllDrones = () => this.handleLand();
        window.resetSwarm = () => this.handleReset();
        window.resetMission = () => this.handleReset();
        
        // Caméra
        window.resetCamera = () => this.handleResetCamera();
        window.topView = () => this.handleTopView();
        window.zoomToSwarm = () => this.handleZoomToSwarm();
        window.toggleFollowMode = () => this.handleToggleFollow();
        
        // Display
        window.toggleMinimap = () => this.handleToggleMinimap();
        window.toggleDebugPanels = () => this.handleToggleDebug();
        window.togglePanel = () => this.handleTogglePanel();
        
        // Modes et patterns
        window.applyMode = () => this.handleApplyMode();
        window.changePattern = () => this.handleChangePattern();
        window.setFormation = (type) => this.handleSetFormation(type);
        
        // Altitude et paramètres
        window.updateAltitudeDisplay = (val) => this.updateAltitudeDisplay(val);
        window.updateSafetyDistance = (val) => this.applySafetyDistance(parseFloat(val));
        
        // Self-test
        window.panelSelfTest = () => this.selfTest();
        
        // Exposer le controller lui-même
        window.panelController = this;
        
        log('🌐 Fonctions globales exposées');
    }
    
    // ==========================================
    // HANDLERS - Mission Control
    // ==========================================
    
    handleLaunch() {
        console.log('[BTN-TEST] handleLaunch() called');
        log('🚀 LAUNCH MISSION');
        
        const system = window.diamantsSystem;
        if (!system?.integratedController) {
            console.warn('[BTN-TEST] handleLaunch: système non prêt');
            this.showFeedback('Système non initialisé — patientez', 'warning');
            return false;
        }
        
        // Démarrer le timer minimap
        if (window.DIAMANTS_MINIMAP) {
            window.DIAMANTS_MINIMAP.startExploration();
        }
        
        // Démarrer la doctrine
        this.doctrineManager?.startMission();
        
        // Controller intégré
        if (system?.integratedController) {
            try {
                system.integratedController.startMissionManual();
            } catch (e) {
                warn('IntegratedController error:', e);
            }
        }
        
        // WebSocket command
        this.sendWebSocketCommand('mission_command', { action: 'start', mission_type: 'exploration' });
        
        this.showFeedback('Mission lancée', 'success');
        return true;
    }
    
    handleStop() {
        console.log('[BTN-TEST] handleStop() called');
        log('⏹ EMERGENCY STOP');
        
        const system = window.diamantsSystem;
        if (!system?.integratedController) {
            console.warn('[BTN-TEST] handleStop: système non prêt');
            this.showFeedback('Système non initialisé — patientez', 'warning');
        }
        
        // Arrêter minimap
        if (window.DIAMANTS_MINIMAP) {
            window.DIAMANTS_MINIMAP.stopExploration();
        }
        
        // Arrêter doctrine
        this.doctrineManager?.stopMission();
        
        // Controller
        if (system?.integratedController) {
            try {
                system.integratedController.emergencyStop();
            } catch (e) {}
        }
        
        // WebSocket
        this.sendWebSocketCommand('mission_command', { action: 'emergency_land' });
        
        this.showFeedback('Arrêt d\'urgence', 'warning');
        return true;
    }
    
    handleTakeoff() {
        console.log('[BTN-TEST] handleTakeoff() called');
        log('🛫 TAKEOFF ALL');
        
        const system = window.diamantsSystem;
        if (!system?.integratedController?.autonomousFlightEngine) {
            this.showFeedback('Système non prêt pour le décollage', 'warning');
            return false;
        }
        
        const controller = system.integratedController;
        const engine = controller.autonomousFlightEngine;
        const altitude = this.doctrineManager?.zoneParams?.altitude || 3.0;
        
        // Activer le controller pour que engine.update() tourne
        controller.isRunning = true;
        
        // Takeoff chaque drone via l'engine
        let count = 0;
        controller.drones.forEach((drone, idx) => {
            const droneId = drone.id || `crazyflie_${String(idx + 1).padStart(2, '0')}`;
            const state = engine.drones?.get(droneId);
            if (state) {
                engine.takeoff(droneId, altitude);
                count++;
            } else {
                console.warn(`[TAKEOFF] Drone ${droneId} non trouvé dans l'engine`);
            }
        });
        
        this.showFeedback(`Décollage ${count} drones à ${altitude}m`, 'success');
        this.sendWebSocketCommand('mission_command', { action: 'takeoff_all' });
        return true;
    }
    
    handleLand() {
        console.log('[BTN-TEST] handleLand() called');
        log('🛬 LAND ALL');
        
        const system = window.diamantsSystem;
        if (!system?.integratedController?.autonomousFlightEngine) {
            this.showFeedback('Système non prêt pour l\'atterrissage', 'warning');
            return false;
        }
        
        const controller = system.integratedController;
        const engine = controller.autonomousFlightEngine;
        
        // S'assurer que le controller tourne pour que l'atterrissage s'anime
        controller.isRunning = true;
        // Permettre un re-Launch après atterrissage
        controller.missionStarted = false;
        
        let count = 0;
        controller.drones.forEach((drone, idx) => {
            const droneId = drone.id || `crazyflie_${String(idx + 1).padStart(2, '0')}`;
            const state = engine.drones?.get(droneId);
            if (state) {
                engine.land(droneId);
                count++;
            } else {
                console.warn(`[LAND] Drone ${droneId} non trouvé dans l'engine`);
            }
        });
        
        this.showFeedback(`Atterrissage de ${count} drones`, 'info');
        this.sendWebSocketCommand('mission_command', { action: 'land_all' });
        return true;
    }
    
    handleReset() {
        console.log('[BTN-TEST] handleReset() called');
        log('🔄 RESET SWARM');
        
        // Reset minimap
        if (window.DIAMANTS_MINIMAP) {
            window.DIAMANTS_MINIMAP.reset();
        }
        
        // Reset drone positions + engine state
        const system = window.diamantsSystem;
        if (system?.integratedController) {
            const engine = system.integratedController.autonomousFlightEngine;
            
            // 1. Stop la mission en cours
            system.integratedController.isRunning = false;
            system.integratedController.missionStarted = false;
            
            // 2. Réinitialiser tous les drones (position + engine)
            system.integratedController.drones.forEach((drone, idx) => {
                const droneId = drone.id || `crazyflie_${String(idx + 1).padStart(2, '0')}`;
                
                // Retour position initiale — sur la surface de la plateforme
                const angle = (idx / system.integratedController.drones.length) * 2 * Math.PI;
                const radius = 5.5;
                const PLATFORM_SURFACE_Y = 0.15; // Platform top surface
                const spawnX = Math.cos(angle) * radius;
                const spawnZ = Math.sin(angle) * radius;
                
                drone.position.set(spawnX, PLATFORM_SURFACE_Y, spawnZ);
                drone.velocity?.set(0, 0, 0);
                if (drone.mesh) {
                    drone.mesh.position.copy(drone.position);
                }
                
                // Réinitialiser l'état dans le moteur de vol autonomous
                if (engine) {
                    const state = engine.drones?.get(droneId);
                    if (state) {
                        state.phase = 'IDLE';
                        state.position.set(spawnX, PLATFORM_SURFACE_Y, spawnZ);
                        state.velocity.set(0, 0, 0);
                        state.smoothVelocity.set(0, 0, 0);
                        state.waypoint = null;
                        state.waypointTimer = 0;
                        state.waypointsVisited = 0;
                        state.lastWaypointTime = 0;
                    }
                }
            });
        }
        
        this.sendWebSocketCommand('mission_command', { action: 'return_home' });
        this.showFeedback('Essaim réinitialisé', 'info');
        return true;
    }
    
    // ==========================================
    // HANDLERS - Camera
    // ==========================================
    
    handleResetCamera() {
        console.log('[BTN-TEST] handleResetCamera() called');
        const system = window.diamantsSystem;
        if (system?.camera && system?.controls) {
            system.camera.position.set(40, 25, 40);
            system.controls.target.set(0, 5, 0);
            system.controls.update();
            this.showFeedback('Caméra réinitialisée', 'info');
        } else {
            this.showFeedback('Caméra non prête', 'warning');
            return false;
        }
        return true;
    }
    
    handleTopView() {
        console.log('[BTN-TEST] handleTopView() called');
        const system = window.diamantsSystem;
        if (system?.camera && system?.controls) {
            // Vue centrée sur l'essaim si disponible
            let targetX = 0, targetZ = 0;
            if (system.drones?.length > 0) {
                system.drones.forEach(d => { targetX += d.position.x; targetZ += d.position.z; });
                targetX /= system.drones.length;
                targetZ /= system.drones.length;
            }
            system.camera.position.set(targetX, 60, targetZ + 0.1);
            system.controls.target.set(targetX, 0, targetZ);
            system.controls.update();
            this.showFeedback('Vue de dessus', 'info');
        } else {
            this.showFeedback('Caméra non prête', 'warning');
            return false;
        }
        return true;
    }
    
    handleZoomToSwarm() {
        console.log('[BTN-TEST] handleZoomToSwarm() called');
        const system = window.diamantsSystem;
        if (system?.integratedController?.drones?.length > 0) {
            const drones = system.integratedController.drones;
            let avgX = 0, avgY = 0, avgZ = 0;
            
            drones.forEach(d => {
                avgX += d.position.x;
                avgY += d.position.y;
                avgZ += d.position.z;
            });
            
            avgX /= drones.length;
            avgY /= drones.length;
            avgZ /= drones.length;
            
            system.camera.position.set(avgX + 20, avgY + 15, avgZ + 20);
            system.controls.target.set(avgX, avgY, avgZ);
            system.controls.update();
            this.showFeedback('Vue essaim', 'info');
        } else {
            this.showFeedback('Aucun drone détecté', 'warning');
            return false;
        }
        return true;
    }
    
    handleToggleFollow() {
        console.log('[BTN-TEST] handleToggleFollow() called');
        const system = window.diamantsSystem;
        if (!system) {
            this.showFeedback('Système non prêt', 'warning');
            return true;
        }
        
        const droneCount = system.drones?.length || 0;
        if (droneCount === 0) {
            this.showFeedback('Aucun drone disponible', 'warning');
            return true;
        }
        
        // Si follow n'est pas actif, l'activer sur le premier drone
        if (!system.autoFollow) {
            system.autoFollow = true;
            system.followDroneIndex = 0;
        } else {
            // Déjà en mode follow — passer au drone suivant
            system.followDroneIndex = (system.followDroneIndex + 1) % droneCount;
            
            // Si on a fait le tour complet, désactiver le follow
            if (system.followDroneIndex === 0) {
                system.autoFollow = false;
                system.followDroneIndex = -1;
                this.showFeedback('Follow: OFF', 'info');
                return true;
            }
        }
        
        const drone = system.drones[system.followDroneIndex];
        const droneId = drone?.id || `Drone ${system.followDroneIndex + 1}`;
        this.showFeedback(`Follow: ${droneId} (${system.followDroneIndex + 1}/${droneCount})`, 'success');
        return true;
    }
    
    // ==========================================
    // HANDLERS - Display
    // ==========================================
    
    handleToggleMinimap() {
        console.log('[BTN-TEST] handleToggleMinimap() called');
        const minimap = document.getElementById('minimap');
        if (minimap) {
            const isHidden = minimap.style.display === 'none' || getComputedStyle(minimap).display === 'none';
            minimap.style.display = isHidden ? 'block' : 'none';
            this.showFeedback(`Minimap: ${isHidden ? 'ON' : 'OFF'}`, 'info');
        }
        return true;
    }
    
    handleToggleDebug() {
        console.log('[BTN-TEST] handleToggleDebug() called');
        
        // Toggle l'orchestration console (le vrai panneau debug)
        const system = window.diamantsSystem;
        if (system?.orchestrationConsole) {
            system.orchestrationConsole.toggle();
            this.showFeedback('Console debug toggled', 'info');
            return true;
        }
        
        // Fallback: toggle les panneaux debug si présents
        const debugPanels = document.querySelectorAll('.debug-panel, #debug-overlay');
        if (debugPanels.length > 0) {
            debugPanels.forEach(p => {
                p.style.display = p.style.display === 'none' ? 'block' : 'none';
            });
            return true;
        }
        
        this.showFeedback('Pas de panneau debug disponible', 'warning');
        return true;
    }
    
    handleTogglePanel() {
        console.log('[BTN-TEST] handleTogglePanel() called');
        const panel = document.getElementById('ros_interface');
        const canvas = document.getElementById('canvas_container');
        const button = document.getElementById('toggle_panel');
        
        if (!panel) return false;
        
        const isHidden = panel.style.display === 'none';
        
        panel.style.display = isHidden ? 'block' : 'none';
        if (canvas) canvas.classList.toggle('panel-visible', isHidden);
        if (button) {
            button.classList.toggle('panel-visible', isHidden);
            button.innerHTML = isHidden ? '◄' : '►';
        }
        
        return true;
    }
    
    // ==========================================
    // HANDLERS - Modes et Patterns
    // ==========================================
    
    handleApplyMode() {
        const select = document.getElementById('mode_select');
        if (select) {
            const mode = select.value;
            this.doctrineManager?.setCOA(mode);
            this.showFeedback(`Pattern: ${mode}`, 'success');
        }
        return true;
    }
    
    handleChangePattern() {
        const patterns = ['adaptive', 'grid', 'boustrophedon', 'spiral', 'swarm'];
        const select = document.getElementById('mode_select');
        
        if (select) {
            const currentIndex = patterns.indexOf(select.value);
            const nextIndex = (currentIndex + 1) % patterns.length;
            select.value = patterns[nextIndex];
            this.doctrineManager?.setCOA(patterns[nextIndex]);
            this.showFeedback(`Pattern: ${patterns[nextIndex]}`, 'info');
        }
        return true;
    }
    
    handleSetFormation(type) {
        this.doctrineManager?.setFormation(type);
        this.showFeedback(`Formation: ${type}`, 'info');
        return true;
    }
    
    // ==========================================
    // UTILITAIRES
    // ==========================================
    
    applyAltitude(value) {
        if (this.doctrineManager) {
            this.doctrineManager.zoneParams.altitude = value;
        }
        
        // Appliquer aux drones en vol
        const system = window.diamantsSystem;
        if (system?.integratedController?.autonomousFlightEngine) {
            const engine = system.integratedController.autonomousFlightEngine;
            if (engine.drones) {
                system.integratedController.drones.forEach((drone, idx) => {
                    const droneId = drone.id || `crazyflie_${String(idx + 1).padStart(2, '0')}`;
                    const state = engine.drones.get(droneId);
                    if (state && state.mode !== 'IDLE' && state.mode !== 'LANDING') {
                        state.targetAltitude = value;
                    }
                });
            }
        }
    }
    
    updateAltitudeDisplay(value) {
        const display = document.getElementById('altitude_display');
        if (display) display.textContent = `${parseFloat(value).toFixed(1)}m`;
        this.applyAltitude(parseFloat(value));
    }
    
    applySafetyDistance(value) {
        if (this.doctrineManager) {
            this.doctrineManager.zoneParams.safetyDistance = value;
        }
        
        // Mettre à jour le collision system
        const system = window.diamantsSystem;
        if (system?.integratedController?.autonomousFlightEngine) {
            system.integratedController.autonomousFlightEngine.collisionRadius = value;
        }
    }
    
    sendWebSocketCommand(type, data) {
        const system = window.diamantsSystem;
        if (system?.ros?.ws?.readyState === WebSocket.OPEN) {
            try {
                system.ros.ws.send(JSON.stringify({ type, data }));
                log(`📡 WS: ${type}`, data);
            } catch (e) {
                warn('WebSocket send error:', e);
            }
        }
    }
    
    showFeedback(message, type = 'info') {
        // Clear previous
        if (this.feedbackTimeout) {
            clearTimeout(this.feedbackTimeout);
        }
        
        // Créer ou réutiliser l'élément feedback
        let feedback = document.getElementById('panel-feedback');
        if (!feedback) {
            feedback = document.createElement('div');
            feedback.id = 'panel-feedback';
            feedback.style.cssText = `
                position: fixed;
                top: 20px;
                left: 50%;
                transform: translateX(-50%);
                padding: 10px 20px;
                border-radius: 6px;
                font-size: 12px;
                font-weight: bold;
                z-index: 10000;
                transition: opacity 0.3s;
                pointer-events: none;
            `;
            document.body.appendChild(feedback);
        }
        
        // Style selon type
        const styles = {
            success: { bg: '#059669', color: '#fff' },
            error: { bg: '#dc2626', color: '#fff' },
            warning: { bg: '#d97706', color: '#fff' },
            info: { bg: '#0284c7', color: '#fff' }
        };
        
        const style = styles[type] || styles.info;
        feedback.style.backgroundColor = style.bg;
        feedback.style.color = style.color;
        feedback.textContent = message;
        feedback.style.opacity = '1';
        
        // Auto-hide
        this.feedbackTimeout = setTimeout(() => {
            feedback.style.opacity = '0';
        }, 2000);
    }
    
    validateState() {
        const issues = [];
        
        // Vérifier les éléments UI
        const requiredElements = [
            'mission_type', 'mode_select', 'altitude_slider', 
            'safety_distance', 'minimap', 'ros_interface'
        ];
        
        requiredElements.forEach(id => {
            if (!document.getElementById(id)) {
                issues.push(`Element manquant: #${id}`);
            }
        });
        
        // Vérifier le système
        if (!window.diamantsSystem) {
            issues.push('diamantsSystem non initialisé');
        }
        
        if (issues.length > 0) {
            warn('⚠️ Problèmes détectés:', issues);
        } else {
            log('✅ Validation OK');
        }
        
        return issues;
    }
    
    /**
     * Run self-test
     */
    selfTest() {
        log('🧪 SELF-TEST Panel Controller');
        const results = {
            passed: 0,
            failed: 0,
            tests: []
        };
        
        // Test 1: Boutons câblés
        const test1 = this.buttonStates.size > 0;
        results.tests.push({ name: 'Boutons câblés', passed: test1 });
        test1 ? results.passed++ : results.failed++;
        
        // Test 2: Fonctions globales
        const globalFns = ['launchMission', 'emergencyLand', 'takeoffAllDrones', 'resetCamera', 'resetMission', 'landAllDrones', 'toggleMinimap'];
        const test2 = globalFns.every(fn => typeof window[fn] === 'function');
        results.tests.push({ name: 'Fonctions globales', passed: test2, detail: globalFns.filter(fn => typeof window[fn] !== 'function') });
        test2 ? results.passed++ : results.failed++;
        
        // Test 3: DoctrineManager
        const test3 = this.doctrineManager !== null;
        results.tests.push({ name: 'DoctrineManager', passed: test3 });
        test3 ? results.passed++ : results.failed++;
        
        // Test 4: Minimap
        const test4 = window.DIAMANTS_MINIMAP !== undefined;
        results.tests.push({ name: 'Exploration Minimap', passed: test4 });
        test4 ? results.passed++ : results.failed++;
        
        // Test 5: Éléments UI
        const test5 = document.getElementById('mission_type') !== null;
        results.tests.push({ name: 'UI Elements', passed: test5 });
        test5 ? results.passed++ : results.failed++;
        
        // Afficher résultats
        log('📊 Résultats:');
        results.tests.forEach(t => {
            log(`   ${t.passed ? '✅' : '❌'} ${t.name}`);
        });
        log(`   Total: ${results.passed}/${results.passed + results.failed}`);
        
        return results;
    }
}

// Auto-init supprimé — main.js appelle initPanelController() directement
let panelController = null;

export function initPanelController() {
    if (!panelController) {
        panelController = new PanelController();
        window.DIAMANTS_PANEL = panelController;
    }
    return panelController;
}

export default PanelController;
