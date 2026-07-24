/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Panel Controller Robuste
 * =====================================
 * Câblage complet de tous les boutons et fonctions du panneau de contrôle
 * Avec validation et feedback utilisateur
 */

import { getDoctrineManager, DOCTRINES, COURSES_OF_ACTION } from '../missions/mission-doctrine.js';
import { ScenarioEngine } from '../intelligence/scenario-engine.js';

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
        
        // Écouter le changement de mode (SIM / RÉEL / SITL)
        this.setupModeBadge();
        
        // Valider l'état initial
        this.validateState();
        
        this.isInitialized = true;
        log('✅ Panel Controller initialisé');
        
        // Start LLM stats updater once engine is ready
        window.addEventListener('diamants:engine-ready', () => {
            this.startLLMStatsUpdater();
        });
        // Fallback: auto-ping Ollama even if engine never fires ready event
        setTimeout(() => this.handlePingOllama(), 1000);
        setTimeout(() => { if (!this._llmStatsInterval) this.startLLMStatsUpdater(); }, 5000);

        // Feedback visuel
        this.showFeedback('Système prêt', 'success');

        // Sidebar resize handle
        this._initSidebarResize();
    }

    /**
     * Sidebar drag-to-resize — poignée droite du panneau latéral
     */
    _initSidebarResize() {
        const handle = document.getElementById('sidebar-resize-handle');
        const panel = document.getElementById('ros_interface');
        if (!handle || !panel) return;

        const MIN_W = 220;
        const MAX_W_VW = 0.55; // 55% of viewport
        const MAX_W_ABS = 1200;
        const BASE_W = 300; // reference width where scale = 1.0

        let isDragging = false;
        let startX = 0;
        let startWidth = 0;

        const getMaxWidth = () => Math.min(window.innerWidth * MAX_W_VW, MAX_W_ABS);

        const updateScale = (width) => {
            document.documentElement.style.setProperty('--sidebar-width', width + 'px');
            // Scale: linear from 0.73 at 220px → 1.0 at 300px → proportional beyond
            const scale = width / BASE_W;
            document.documentElement.style.setProperty('--sidebar-scale', scale.toFixed(3));
            // Update derived variables
            document.documentElement.style.setProperty('--sidebar-pad', `${Math.round(8 * scale)}px`);
            document.documentElement.style.setProperty('--sidebar-gap', `${Math.round(6 * scale)}px`);
        };

        const onMouseDown = (e) => {
            isDragging = true;
            startX = e.clientX;
            startWidth = panel.getBoundingClientRect().width;
            handle.classList.add('dragging');
            document.body.style.cursor = 'ew-resize';
            document.body.style.userSelect = 'none';
            e.preventDefault();
        };

        const onMouseMove = (e) => {
            if (!isDragging) return;
            const delta = e.clientX - startX;
            const maxW = getMaxWidth();
            const newWidth = Math.max(MIN_W, Math.min(maxW, startWidth + delta));
            updateScale(newWidth);
        };

        const onMouseUp = () => {
            if (!isDragging) return;
            isDragging = false;
            handle.classList.remove('dragging');
            document.body.style.cursor = '';
            document.body.style.userSelect = '';
        };

        handle.addEventListener('mousedown', onMouseDown);
        document.addEventListener('mousemove', onMouseMove);
        document.addEventListener('mouseup', onMouseUp);

        log('↔️ Sidebar resize initialisé (max: 55vw / 1200px)');
    }
    
    /**
     * Mode badge — affiche SIM / RÉEL / SITL dans le panneau status
     */
    setupModeBadge() {
        const BADGE_STYLES = {
            simulation: { text: 'SIM',  bg: '#0d3b66', border: '#2196F3', color: '#64B5F6' },
            sitl:       { text: 'SITL', bg: '#3e2723', border: '#FF9800', color: '#FFB74D' },
            real:       { text: 'RÉEL', bg: '#1b5e20', border: '#4CAF50', color: '#81C784' },
            unknown:    { text: '—',    bg: '#1a1a2e', border: '#555',    color: '#888'    },
        };

        const apply = (mode) => {
            const badge = document.getElementById('mode_badge');
            if (!badge) return;
            const s = BADGE_STYLES[mode] || BADGE_STYLES.unknown;
            badge.textContent = s.text;
            badge.style.background = s.bg;
            badge.style.borderColor = s.border;
            badge.style.color = s.color;
            log(`Mode badge → ${s.text}`);
        };

        // Listen for mode-change event from RosWebBridge
        window.addEventListener('diamants:mode-change', (e) => apply(e.detail?.mode));

        // If bridge already detected mode before panel init
        try {
            const ros = window.diamantsSystem?.ros;
            if (ros?.mode && ros.mode !== 'unknown') apply(ros.mode);
        } catch (_) { /* safe */ }

        // Standalone : badge CAS quand pas de backend ROS
        window.addEventListener('diamants:cas-changed', (e) => {
            const ros = window.diamantsSystem?.ros;
            if (ros?.connected) return;
            const level = e.detail?.level ?? window.DIAMANTS?.casLevel ?? 2;
            const labels = { 1: 'CAS-1', 2: 'CAS-2', 3: 'CAS-3' };
            const colors = {
                1: { bg: '#1b5e20', border: '#4CAF50', color: '#81C784' },
                2: { bg: '#0d3b66', border: '#38bdf8', color: '#7dd3fc' },
                3: { bg: '#7c2d12', border: '#f97316', color: '#fdba74' },
            };
            const badge = document.getElementById('mode_badge');
            if (!badge) return;
            const c = colors[level] || colors[2];
            badge.textContent = labels[level] || 'CAS-2';
            badge.style.background = c.bg;
            badge.style.borderColor = c.border;
            badge.style.color = c.color;
        });
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
        this.bindButton('toggleDiamantsHUD', () => this.handleToggleDiamantsHUD(), 'Toggle DIAMANTS');
        this.bindButton('toggleDroneLabels', () => this.handleToggleLabels(), 'Toggle Labels');
        this.bindButton('toggleCommWaves', () => this.handleToggleCommWaves(), 'Toggle Comm Waves');
        this.bindButton('togglePanel', () => this.handleTogglePanel(), 'Toggle Panel');
        
        // === BENCHMARK ===
        this.bindButtonByQuery('[onclick*="runQuickBenchmark"]', () => this.handleBenchmark(30), 'Quick Benchmark');
        
        // === BEACON MANAGEMENT ===
        this._wireBeaconButtons();
        
        // === SCÉNARIOS DISTRIBUÉS (indépendant du contrôle de mission) ===
        try {
            this.scenarioEngine = new ScenarioEngine();
            this._wireScenarioPanel();
            // Expose for LLM system prompt scenario awareness
            window.DIAMANTS = window.DIAMANTS || {};
            window.DIAMANTS.scenarioEngine = this.scenarioEngine;
        } catch (e) {
            console.error('[PanelCtrl] Failed to init ScenarioEngine:', e);
        }
        
        log(`📎 ${this.buttonStates.size} boutons câblés`);
    }
    
    /**
     * Wire beacon management buttons + badge updater
     */
    _wireBeaconButtons() {
        const btnClick = document.getElementById('btn-place-beacon-click');
        const btnRandom = document.getElementById('btn-place-beacon-random');
        const btn3 = document.getElementById('btn-place-beacon-3');
        const btnClear = document.getElementById('btn-clear-beacons');
        const badge = document.getElementById('beacon-count-badge');

        // ── Click-to-place toggle button ──
        if (btnClick) {
            const setActive = (active) => {
                if (active) {
                    btnClick.style.background = 'linear-gradient(45deg, #10b981, #34d399)';
                    btnClick.style.boxShadow = '0 0 8px #10b981';
                    btnClick.textContent = '📌 Placer… (clic scène)';
                } else {
                    btnClick.style.background = 'linear-gradient(45deg, #065f46, #10b981)';
                    btnClick.style.boxShadow = 'none';
                    btnClick.textContent = '📌 Placer';
                }
            };

            btnClick.addEventListener('click', () => {
                // Resolve beaconSystem dynamically (it's initialized after panel-controller)
                const bs = window.DIAMANTS?.beacons;
                if (!bs) {
                    console.warn('[Panel] BeaconSystem not ready yet');
                    return;
                }
                if (bs.placementMode) {
                    bs.exitPlacementMode();
                    document.getElementById('beacon-placement-hud')?.classList.remove('active');
                } else {
                    bs.enterPlacementMode();
                    document.getElementById('beacon-placement-hud')?.classList.add('active');
                    // Close le panneau LLM pour permettre l'interaction avec la scène 3D
                    document.getElementById('llm-chat-panel')?.style.setProperty('display', 'none');
                }
            });

            // Sync button state when beacon-mode toggles (including auto-exit after placement)
            window.addEventListener('diamants:beacon-mode', (e) => {
                setActive(e.detail?.active);
                // Also sync HUD
                const hud = document.getElementById('beacon-placement-hud');
                if (hud) {
                    if (e.detail?.active) hud.classList.add('active');
                    else hud.classList.remove('active');
                }
            });
        }

        // ── Random placement buttons ──
        if (btnRandom) {
            btnRandom.addEventListener('click', () => {
                this.executeWithFeedback(() => {
                    window.DIAMANTS?.placeRandomBeacons?.(1);
                }, 'Place 1 Beacon Random', btnRandom);
            });
        }
        if (btn3) {
            btn3.addEventListener('click', () => {
                this.executeWithFeedback(() => {
                    window.DIAMANTS?.placeRandomBeacons?.(3);
                }, 'Place 3 Beacons', btn3);
            });
        }
        if (btnClear) {
            btnClear.addEventListener('click', () => {
                this.executeWithFeedback(() => {
                    window.DIAMANTS?.clearBeacons?.();
                }, 'Clear All Beacons', btnClear);
            });
        }

        // Update beacon count badge on any beacon event
        const updateBadge = () => {
            if (!badge) return;
            const bs = window.DIAMANTS?.beacons;
            const count = bs?.beacons?.size || 0;
            badge.textContent = count;
            badge.style.color = count > 0 ? '#fbbf24' : '#a78bfa';
        };

        window.addEventListener('diamants:beacon-placed', updateBadge);
        window.addEventListener('diamants:beacon-found', updateBadge);
        window.addEventListener('diamants:beacons-updated', updateBadge);

        // Also expose globally
        window.placeRandomBeacons = (n = 3) => window.DIAMANTS?.placeRandomBeacons?.(n);
        window.clearBeacons = () => window.DIAMANTS?.clearBeacons?.();
    }
    
    /**
     * Wire Scénarios panel — driven by scenarios.json config.
     * Completely independent from mission control (no LAUNCH/TAKEOFF interaction).
     */
    _wireScenarioPanel() {
        const selectEl = document.getElementById('scenario-select');
        const launchBtn = document.getElementById('btn-scenario-launch');
        const statusEl  = document.getElementById('scenario-status');
        const infoDiv   = document.getElementById('scenario-info');
        const descEl    = document.getElementById('scenario-desc');
        const objEl     = document.getElementById('scenario-objective');
        const obsEl     = document.getElementById('scenario-observe');

        if (!selectEl || !launchBtn) {
            warn('Scenario panel elements not found');
            return;
        }

        // Populate dropdown from JSON config — grouped by category (M2MC champs)
        const CATEGORY_LABELS = {
            informationnel: '── Champ Informationnel ──',
            cognitif:       '── Champ Cognitive ──',
            physique:       '── Champ Physique ──',
            transverse:     '── Transverse ──',
        };
        const allScenarios = this.scenarioEngine.getAll();
        const seenCategories = new Set();
        for (const s of allScenarios) {
            const cat = s.category || 'autre';
            if (!seenCategories.has(cat)) {
                seenCategories.add(cat);
                const optGroup = document.createElement('optgroup');
                optGroup.label = CATEGORY_LABELS[cat] || `── ${cat} ──`;
                for (const sc of allScenarios.filter(x => x.category === cat)) {
                    const opt = document.createElement('option');
                    opt.value = sc.id;
                    opt.textContent = sc.name;
                    optGroup.appendChild(opt);
                }
                selectEl.appendChild(optGroup);
            }
        }

        // ── Selection handler: show info + enable launch button ──
        selectEl.addEventListener('change', () => {
            const scenario = this.scenarioEngine.get(selectEl.value);
            if (!scenario) return;

            // Show description panel
            if (infoDiv) infoDiv.style.display = 'block';
            if (descEl)  descEl.textContent = scenario.description;
            if (objEl)   objEl.innerHTML = `<em>${scenario.objective}</em>`;
            if (obsEl) {
                obsEl.innerHTML = '<strong>Observer :</strong><br>' +
                    scenario.observe.map(o => `  • ${o}`).join('<br>');
            }

            // Enable launch button with scenario color
            const isActive = this.scenarioEngine.isActive(scenario.id);
            const grad = isActive
                ? (scenario.color.activeGradient || scenario.color.gradient)
                : scenario.color.gradient;
            launchBtn.disabled = false;
            launchBtn.style.opacity = '1';
            launchBtn.style.cursor = 'pointer';
            launchBtn.style.background = `linear-gradient(45deg, ${grad[0]}, ${grad[1]})`;

            if (scenario.toggle && isActive) {
                launchBtn.textContent = `■ Arrêter: ${scenario.name}`;
                launchBtn.style.boxShadow = `0 0 8px ${scenario.color.glow || grad[1]}`;
            } else {
                launchBtn.textContent = `▶ Lancer: ${scenario.name}`;
                launchBtn.style.boxShadow = 'none';
            }
        });

        // ── Launch / toggle handler ──
        launchBtn.addEventListener('click', () => {
            const scenarioId = selectEl.value;
            if (!scenarioId) {
                console.error('[PanelCtrl] No scenario selected');
                return;
            }

            console.log('[PanelCtrl] Executing scenario:', scenarioId);
            const scenario = this.scenarioEngine.get(scenarioId);
            if (!scenario) return;

            // If scenario is already running (toggle active) → just toggle OFF
            // Otherwise → full reset + re-launch
            const isCurrentlyActive = scenario.toggle && this.scenarioEngine.isActive(scenarioId);
            const result = isCurrentlyActive
                ? this.scenarioEngine.execute(scenarioId)   // toggle OFF only
                : this.scenarioEngine.resetAndExecute(scenarioId); // reset + launch

            console.log('[PanelCtrl] Scenario result:', result);

            if (!result.ok) {
                if (statusEl) {
                    statusEl.textContent = `${result.message}`;
                    statusEl.style.color = '#ff4444';
                }
                if (descEl) {
                    descEl.textContent = result.message;
                    descEl.style.color = '#ff4444';
                }
                if (infoDiv) infoDiv.style.display = 'block';
                console.error('[PanelCtrl] Scenario failed:', result.message);
                warn(result.message);
                return;
            }

            // Update status badge
            if (statusEl) {
                statusEl.textContent = result.message;
                statusEl.style.color = scenario.color.status || '#ff6644';
            }

            // Update button appearance for toggle state
            if (scenario.toggle) {
                const isNowActive = result.active;
                const grad = isNowActive
                    ? (scenario.color.activeGradient || scenario.color.gradient)
                    : scenario.color.gradient;
                launchBtn.style.background = `linear-gradient(45deg, ${grad[0]}, ${grad[1]})`;

                if (isNowActive) {
                    launchBtn.textContent = `■ Arrêter: ${scenario.name}`;
                    launchBtn.style.boxShadow = `0 0 8px ${scenario.color.glow || grad[1]}`;
                    // Show restore info
                    if (scenario.restore && descEl) {
                        descEl.textContent = scenario.restore;
                    }
                } else {
                    launchBtn.textContent = `▶ Lancer: ${scenario.name}`;
                    launchBtn.style.boxShadow = 'none';
                    // Restore original description
                    if (descEl) descEl.textContent = scenario.description;
                    // Auto-clear status after 8s
                    setTimeout(() => {
                        if (statusEl) { statusEl.textContent = '—'; statusEl.style.color = '#888'; }
                    }, 8000);
                }
            } else {
                // One-shot: show result then auto-clear
                if (descEl && scenario.description) {
                    descEl.textContent = scenario.description;
                }
                setTimeout(() => {
                    if (statusEl) { statusEl.textContent = '—'; statusEl.style.color = '#888'; }
                }, 12000);
            }

            log(`Scenario ${scenarioId}: ${result.message}`);
        });

        log(`Scenario panel wired with ${this.scenarioEngine.scenarios.size} scenarios`);

        // ── Mirror to mobile scenario panel ──
        this._wireMobileScenarioPanel();
    }

    /**
     * Wire mobile scenario panel — mirrors the sidebar scenario dropdown + launch.
     */
    _wireMobileScenarioPanel() {
        const mobileSelect = document.getElementById('mobile-scenario-select');
        const mobileLaunch = document.getElementById('mobile-btn-scenario-launch');
        const mobileInfo   = document.getElementById('mobile-scenario-info');
        if (!mobileSelect || !mobileLaunch) return;

        // Ensure native picker can open on mobile — stop any parent touch handler
        mobileSelect.addEventListener('touchstart', (e) => e.stopPropagation(), { capture: true, passive: false });
        mobileSelect.addEventListener('mousedown', (e) => e.stopPropagation());

        // Populate mobile dropdown — use FLAT options (no optgroups) for max mobile compat
        const CATEGORY_ICONS = {
            informationnel: 'ℹ️',
            cognitif:       '🧠',
            physique:       '⚡',
            transverse:     '🎯',
        };
        const allScenarios = this.scenarioEngine.getAll();
        let lastCat = '';
        for (const sc of allScenarios) {
            const cat = sc.category || 'autre';
            // Add a disabled separator option when category changes
            if (cat !== lastCat) {
                lastCat = cat;
                const sep = document.createElement('option');
                sep.disabled = true;
                sep.textContent = `── ${(CATEGORY_ICONS[cat] || '📋')} ${cat.charAt(0).toUpperCase() + cat.slice(1)} ──`;
                mobileSelect.appendChild(sep);
            }
            const opt = document.createElement('option');
            opt.value = sc.id;
            opt.textContent = `${sc.name}`;
            mobileSelect.appendChild(opt);
        }

        log(`[Mobile] Scenario select populated with ${allScenarios.length} scenarios (flat)`);

        // Selection handler
        mobileSelect.addEventListener('change', () => {
            const scenario = this.scenarioEngine.get(mobileSelect.value);
            if (!scenario) return;
            if (mobileInfo) {
                mobileInfo.style.display = 'block';
                mobileInfo.innerHTML = `<strong>${scenario.name}</strong><br>${scenario.description}`;
            }
            mobileLaunch.disabled = false;
            mobileLaunch.style.opacity = '1';
            mobileLaunch.style.cursor = 'pointer';
            const grad = scenario.color?.gradient || ['#374151', '#4b5563'];
            mobileLaunch.style.background = `linear-gradient(135deg, ${grad[0]}, ${grad[1]})`;
            mobileLaunch.style.borderColor = scenario.color?.status || 'rgba(255,100,50,0.5)';

            const isActive = this.scenarioEngine.isActive(scenario.id);
            mobileLaunch.textContent = (scenario.toggle && isActive)
                ? `■ Arrêter: ${scenario.name}`
                : `▶ Lancer: ${scenario.name}`;

            // Also sync sidebar dropdown
            const sidebarSelect = document.getElementById('scenario-select');
            if (sidebarSelect) {
                sidebarSelect.value = mobileSelect.value;
                sidebarSelect.dispatchEvent(new Event('change'));
            }
        });

        // Launch handler — click + touchend for mobile reliability
        const doLaunch = () => {
            const scenarioId = mobileSelect.value;
            if (!scenarioId) return;
            const scenario = this.scenarioEngine.get(scenarioId);
            if (!scenario) return;

            const isCurrentlyActive = scenario.toggle && this.scenarioEngine.isActive(scenarioId);
            const result = isCurrentlyActive
                ? this.scenarioEngine.execute(scenarioId)
                : this.scenarioEngine.resetAndExecute(scenarioId);

            if (result.ok) {
                this.showFeedback(result.message, 'success');
                const isNowActive = result.active;
                mobileLaunch.textContent = (scenario.toggle && isNowActive)
                    ? `■ Arrêter: ${scenario.name}`
                    : `▶ Lancer: ${scenario.name}`;
            } else {
                this.showFeedback(result.message, 'error');
            }
        };
        let launchHandled = 0;
        const guardLaunch = () => { const now = Date.now(); if (now - launchHandled < 400) return false; launchHandled = now; return true; };
        mobileLaunch.addEventListener('click', () => { if (guardLaunch()) doLaunch(); });
        mobileLaunch.addEventListener('touchend', (e) => { e.preventDefault(); if (guardLaunch()) doLaunch(); });
        mobileLaunch.addEventListener('touchstart', (e) => e.stopPropagation(), { capture: true, passive: false });

        // Wire mobile beacon placement button
        const mobileBeaconBtn = document.getElementById('mobile-btn-place-beacon');
        if (mobileBeaconBtn) {
            mobileBeaconBtn.addEventListener('click', () => {
                const bs = window.DIAMANTS?.beacons;
                if (!bs) { this.showFeedback('Beacon system not ready', 'error'); return; }
                // Close mission sheet and enter placement mode
                document.getElementById('mobile-mission-sheet')?.classList.remove('active');
                bs.enterPlacementMode();
                document.getElementById('beacon-placement-hud')?.classList.add('active');
            });
        }

        // Beacon placement HUD auto-close on placement or mode exit
        window.addEventListener('diamants:beacon-mode', (e) => {
            const hud = document.getElementById('beacon-placement-hud');
            if (hud && !e.detail?.active) hud.classList.remove('active');
        });
        window.addEventListener('diamants:beacon-placed', () => {
            // Update mobile badge
            const bs = window.DIAMANTS?.beacons;
            const count = bs?.beacons?.size || 0;
            const mobileBadge = document.getElementById('mobile-beacon-count');
            if (mobileBadge) mobileBadge.textContent = count;
        });
        window.addEventListener('diamants:beacons-updated', () => {
            const bs = window.DIAMANTS?.beacons;
            const count = bs?.beacons?.size || 0;
            const mobileBadge = document.getElementById('mobile-beacon-count');
            if (mobileBadge) mobileBadge.textContent = count;
        });
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
        
        // Zone exploration (bounds X / Y)
        const boundsX = document.getElementById('bounds_x');
        const boundsY = document.getElementById('bounds_y');
        const applyBounds = () => {
            const bx = boundsX ? parseFloat(boundsX.value) || 80 : 80;
            const by = boundsY ? parseFloat(boundsY.value) || 80 : 80;
            this.applyExplorationBounds(bx, by);
        };
        if (boundsX) {
            boundsX.addEventListener('change', applyBounds);
            boundsX.addEventListener('input', applyBounds);
            log('📎 Input bounds_x câblé');
        }
        if (boundsY) {
            boundsY.addEventListener('change', applyBounds);
            boundsY.addEventListener('input', applyBounds);
            log('📎 Input bounds_y câblé');
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
        window.returnToHome = () => this.handleReturnToHome();
        window.resetSwarm = () => this.handleReset();
        window.resetMission = () => this.handleReset();
        window.resetDronesOnly = () => this.handleResetDronesOnly();
        
        // Caméra
        window.resetCamera = () => this.handleResetCamera();
        window.topView = () => this.handleTopView();
        window.zoomToSwarm = () => this.handleZoomToSwarm();
        window.toggleFollowMode = () => this.handleToggleFollow();
        
        // Display
        window.toggleMinimap = () => this.handleToggleMinimap();
        window.toggleDebugPanels = () => this.handleToggleDebug();
        window.toggleDiamantsHUD = () => this.handleToggleDiamantsHUD();
        window.toggleDroneLabels = () => this.handleToggleLabels();
        window.toggleCommWaves = () => this.handleToggleCommWaves();
        window.toggleCommPanel = () => this.handleToggleCommPanel();
        window.togglePanel = () => this.handleTogglePanel();
        
        // Modes et patterns
        window.applyMode = () => this.handleApplyMode();
        window.changePattern = () => this.handleChangePattern();
        window.setFormation = (type) => this.handleSetFormation(type);
        
        // Altitude et paramètres
        window.updateAltitudeDisplay = (val) => this.updateAltitudeDisplay(val);
        window.updateSafetyDistance = (val) => this.applySafetyDistance(parseFloat(val));
        
        // Intelligence LLM
        window.toggleLLM = () => this.handleToggleLLM();
        window.setLLMModel = (type, model) => this.handleSetLLMModel(type, model);
        window.setLLMInfluence = (val) => this.handleSetLLMInfluence(val);
        window.setOllamaUrl = (url) => this.handleSetOllamaUrl(url);
        window.pingOllama = () => this.handlePingOllama();
        window.resetLLMBrains = () => this.handleResetLLMBrains();

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
        if (window.DIAMANTS_DISCOVERY) {
            window.DIAMANTS_DISCOVERY.startExploration();
        }
        if (window.DIAMANTS_PERCEPTION) {
            window.DIAMANTS_PERCEPTION.reset();
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
            return false;
        }
        
        // Arrêter minimap
        if (window.DIAMANTS_MINIMAP) {
            window.DIAMANTS_MINIMAP.stopExploration();
        }
        if (window.DIAMANTS_DISCOVERY) {
            window.DIAMANTS_DISCOVERY.stopExploration();
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
            this.showFeedback('Système non prêt pour le takeoff', 'warning');
            return false;
        }
        
        const controller = system.integratedController;
        const engine = controller.autonomousFlightEngine;
        const altitude = this.doctrineManager?.zoneParams?.altitude || 3.0;
        
        // Activer le controller pour que engine.update() tourne
        controller.isRunning = true;
        
        // Takeoff chaque drone via l'engine
        let count = 0;
        (controller.drones || []).forEach((drone, idx) => {
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
            this.showFeedback('Système non prêt pour l\'landing', 'warning');
            return false;
        }
        
        const controller = system.integratedController;
        const engine = controller.autonomousFlightEngine;
        
        // S'assurer que le controller tourne pour que l'landing s'anime
        controller.isRunning = true;
        // Permettre un re-Launch après landing
        controller.missionStarted = false;
        
        let count = 0;
        (controller.drones || []).forEach((drone, idx) => {
            const droneId = drone.id || `crazyflie_${String(idx + 1).padStart(2, '0')}`;
            const state = engine.drones?.get(droneId);
            if (state) {
                engine.land(droneId);
                count++;
            } else {
                console.warn(`[LAND] Drone ${droneId} non trouvé dans l'engine`);
            }
        });
        
        this.showFeedback(`Atterrissage sur place de ${count} drones`, 'info');
        this.sendWebSocketCommand('mission_command', { action: 'land_all' });
        return true;
    }

    handleReturnToHome() {
        console.log('[BTN-TEST] handleReturnToHome() called');
        log('🏠 RETURN TO HOME');

        const system = window.diamantsSystem;
        if (!system?.integratedController?.autonomousFlightEngine) {
            this.showFeedback('Système non prêt', 'warning');
            return false;
        }

        const controller = system.integratedController;
        const engine = controller.autonomousFlightEngine;
        controller.isRunning = true;

        let count = 0;
        (controller.drones || []).forEach((drone, idx) => {
            const droneId = drone.id || `crazyflie_${String(idx + 1).padStart(2, '0')}`;
            const state = engine.drones?.get(droneId);
            if (state && state.phase !== 'IDLE' && state.phase !== 'LANDED') {
                engine.returnToHome(droneId);
                count++;
            }
        });

        this.showFeedback(`RTH: ${count} drones retournent à la base`, 'info');
        this.sendWebSocketCommand('mission_command', { action: 'return_home' });
        return true;
    }
    
    handleReset() {
        console.log('[BTN-TEST] handleReset() called');
        log('🔄 RESET SWARM');
        
        // Reset ALL minimaps (exploration heatmap, SLAM perception, fog-of-war discovery)
        if (window.DIAMANTS_MINIMAP)    window.DIAMANTS_MINIMAP.reset();
        if (window.DIAMANTS_PERCEPTION) window.DIAMANTS_PERCEPTION.reset();
        if (window.DIAMANTS_DISCOVERY)  window.DIAMANTS_DISCOVERY.reset();

        // Reset LLM brains (clear stale memory + reasoning)
        try {
            const mgr = window.diamantsSystem?.integratedController?.droneIntelligenceManager;
            if (mgr) mgr.resetAll();
        } catch (_) { /* safe */ }

        // Reset drone positions + engine state
        const system = window.diamantsSystem;
        if (system?.integratedController) {
            const engine = system.integratedController.autonomousFlightEngine;
            
            // 1. Stop la mission en cours
            system.integratedController.isRunning = false;
            system.integratedController.missionStarted = false;
            
            // 2. Full engine reset (coverage, per-drone memory, formation,
            //    pheromones, stall counters, territory — everything)
            if (engine && typeof engine.resetAll === 'function') {
                engine.resetAll();
            }

            // 3. Réinitialiser positions physiques des drones
            //    Use DUAL-RING layout matching createDroneFleet():
            //    Crazyflies on inner ring, X500/S500 on outer ring
            const ctrl = system.integratedController;
            const n = ctrl.drones.length;
            const PLATFORM_SURFACE_Y = 0.15;

            // Separate drone types (same logic as createDroneFleet)
            const cfIndices = [];
            const x500Indices = [];
            ctrl.drones.forEach((drone, idx) => {
                const id = drone.id || '';
                if (id.startsWith('x500') || id.startsWith('s500')) {
                    x500Indices.push(idx);
                } else {
                    cfIndices.push(idx);
                }
            });

            const cfCount = cfIndices.length;
            const x500Count = x500Indices.length;

            // Compute platform radius (same formula as createDroneFleet)
            let platformRadius;
            if (x500Count === 0) {
                platformRadius = n <= 8 ? 8.0 : 8.0 * Math.sqrt(n / 8);
            } else {
                const x500Scale = ctrl.drones[x500Indices[0]]?.mesh?.scale?.x || 10;
                const x500Spacing = x500Scale * 0.5;
                const minRing = x500Count <= 4 ? 8 : 12;
                const x500RingRadius = Math.max(minRing, (x500Count * x500Spacing) / (2 * Math.PI));
                platformRadius = x500RingRadius + 3;
            }

            // Inner rings for Crazyflies — match createDroneFleet() values exactly
            const spawnPositions = new Array(n);
            if (cfCount > 0) {
                const cfMaxR = Math.max(6, platformRadius * 0.55);
                const cfPositions = ctrl._computeRingPositions(cfCount, cfMaxR);
                cfIndices.forEach((idx, i) => { spawnPositions[idx] = cfPositions[i]; });
            }
            // Outer ring for X500/S500
            if (x500Count > 0) {
                const x500R = platformRadius * 0.75;
                x500Indices.forEach((idx, i) => {
                    const angle = (i / x500Count) * 2 * Math.PI;
                    spawnPositions[idx] = { x: Math.cos(angle) * x500R, z: Math.sin(angle) * x500R };
                });
            }

            ctrl.drones.forEach((drone, idx) => {
                const droneId = drone.id || `crazyflie_${String(idx + 1).padStart(2, '0')}`;
                
                const spawnX = spawnPositions[idx].x;
                const spawnZ = spawnPositions[idx].z;
                
                // X500/S500 need higher spawn Y (landing gear clearance at scale)
                const isLargeDrone = droneId.startsWith('x500') || droneId.startsWith('s500');
                const meshScale = drone.mesh?.scale?.x || 10;
                const spawnY = isLargeDrone
                    ? PLATFORM_SURFACE_Y + meshScale * 0.22
                    : PLATFORM_SURFACE_Y + 0.05;
                
                drone.position.set(spawnX, spawnY, spawnZ);
                drone.velocity?.set(0, 0, 0);
                // Clear stale rosData so drones don't enter backend mode (CAS 1)
                // after being fed by engine-emitted drone-positions events
                drone.rosData = null;
                drone._engineControlled = false;
                if (drone.mesh) {
                    drone.mesh.position.copy(drone.position);
                    // Reset visual rotation (heading + pitch/roll)
                    drone.mesh.rotation.set(0, 0, 0);
                }
                
                // Sync engine drone positions with visual spawn
                if (engine) {
                    const state = engine.drones?.get(droneId);
                    if (state) {
                        state.position.set(spawnX, spawnY, spawnZ);
                        state.heading = 0;
                        state._displayHeading = 0;
                    }
                }
            });

            // Clean visual artifacts (comm lines, intelligence sphere, particles)
            this._cleanVisualArtifacts(system);

            // Clear cached change-detection keys so events re-fire after reset
            ctrl._lastDronePhases = {};
            ctrl._lastDronePositions = {};
            ctrl._lastMissionKey = null;
            ctrl._lastSwarmKey = null;
            ctrl._lastVisitedCount = 0;
        }

        // Reset camera to initial view
        this.handleResetCamera();

        this.sendWebSocketCommand('mission_command', { action: 'return_home' });
        this.showFeedback('Essaim réinitialisé', 'info');
        return true;
    }

    /**
     * Reset drones only (positions, engine, minimaps, LLM) WITHOUT resetting the camera.
     * Used by ScenarioEngine to avoid camera jumps during scenario re-launches.
     */
    handleResetDronesOnly() {
        console.log('[BTN-TEST] handleResetDronesOnly() called');
        log('RESET DRONES (camera preserved)');

        if (window.DIAMANTS_MINIMAP)    window.DIAMANTS_MINIMAP.reset();
        if (window.DIAMANTS_PERCEPTION) window.DIAMANTS_PERCEPTION.reset();
        if (window.DIAMANTS_DISCOVERY)  window.DIAMANTS_DISCOVERY.reset();

        try {
            const mgr = window.diamantsSystem?.integratedController?.droneIntelligenceManager;
            if (mgr) mgr.resetAll();
        } catch (_) { /* safe */ }

        const system = window.diamantsSystem;
        if (system?.integratedController) {
            const engine = system.integratedController.autonomousFlightEngine;
            system.integratedController.isRunning = false;
            system.integratedController.missionStarted = false;
            if (engine && typeof engine.resetAll === 'function') engine.resetAll();

            const ctrl = system.integratedController;
            const PLATFORM_SURFACE_Y = 0.15;
            const cfIndices = [];
            const x500Indices = [];
            ctrl.drones.forEach((drone, idx) => {
                const id = drone.id || '';
                if (id.startsWith('x500') || id.startsWith('s500')) x500Indices.push(idx);
                else cfIndices.push(idx);
            });

            let platformRadius;
            if (x500Indices.length === 0) {
                platformRadius = ctrl.drones.length <= 8 ? 8.0 : 8.0 * Math.sqrt(ctrl.drones.length / 8);
            } else {
                const x500Scale = ctrl.drones[x500Indices[0]]?.mesh?.scale?.x || 10;
                const minRing = x500Indices.length <= 4 ? 8 : 12;
                const x500RingRadius = Math.max(minRing, (x500Indices.length * x500Scale * 0.5) / (2 * Math.PI));
                platformRadius = x500RingRadius + 3;
            }

            const spawnPositions = new Array(ctrl.drones.length);
            if (cfIndices.length > 0) {
                const cfMaxR = Math.max(6, platformRadius * 0.55);
                const cfPositions = ctrl._computeRingPositions(cfIndices.length, cfMaxR);
                cfIndices.forEach((idx, i) => { spawnPositions[idx] = cfPositions[i]; });
            }
            if (x500Indices.length > 0) {
                const x500R = platformRadius * 0.75;
                x500Indices.forEach((idx, i) => {
                    const angle = (i / x500Indices.length) * 2 * Math.PI;
                    spawnPositions[idx] = { x: Math.cos(angle) * x500R, z: Math.sin(angle) * x500R };
                });
            }

            ctrl.drones.forEach((drone, idx) => {
                const droneId = drone.id || `crazyflie_${String(idx + 1).padStart(2, '0')}`;
                const spawnX = spawnPositions[idx].x;
                const spawnZ = spawnPositions[idx].z;
                const isLargeDrone = droneId.startsWith('x500') || droneId.startsWith('s500');
                const meshScale = drone.mesh?.scale?.x || 10;
                const spawnY = isLargeDrone ? PLATFORM_SURFACE_Y + meshScale * 0.22 : PLATFORM_SURFACE_Y + 0.05;
                drone.position.set(spawnX, spawnY, spawnZ);
                drone.velocity?.set(0, 0, 0);
                drone.rosData = null;
                drone._engineControlled = false;
                if (drone.mesh) { drone.mesh.position.copy(drone.position); drone.mesh.rotation.set(0, 0, 0); }
                if (engine) {
                    const state = engine.drones?.get(droneId);
                    if (state) { state.position.set(spawnX, spawnY, spawnZ); state.heading = 0; state._displayHeading = 0; }
                }
            });

            // Clean visual artifacts (comm lines, intelligence sphere, particles)
            this._cleanVisualArtifacts(system);

            ctrl._lastDronePhases = {};
            ctrl._lastDronePositions = {};
            ctrl._lastMissionKey = null;
            ctrl._lastSwarmKey = null;
            ctrl._lastVisitedCount = 0;
        }

        // NO camera reset here — intentional
        this.sendWebSocketCommand('mission_command', { action: 'return_home' });
        return true;
    }
    
    // ==========================================
    // HANDLERS - Camera
    // ==========================================
    
    handleResetCamera() {
        console.log('[BTN-TEST] handleResetCamera() called');
        const system = window.diamantsSystem;
        if (system?.camera && system?.controls) {
            // Exit follow mode first so the lerp doesn't override
            if (system.autoFollow) this._exitFollowMode(system);
            // Match initial camera from setupCamera() — ground-level human-eye view
            system.camera.position.set(18, 4, 22);
            system.controls.target.set(0, 1, 0);
            system.controls.autoRotate = false;
            system.controls.autoRotateSpeed = 0;
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
            // Exit follow mode first so the lerp doesn't override
            if (system.autoFollow) this._exitFollowMode(system);
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
        // Exit follow mode first so the lerp doesn't override
        if (system?.autoFollow) this._exitFollowMode(system);
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
            system.controls?.update();
            this.showFeedback('Vue essaim', 'info');
        } else if (!system?.camera || !system?.controls) {
            this.showFeedback('Caméra non disponible', 'warning');
            return false;
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
        
        if (system.autoFollow) {
            // Already following — next drone, or OFF if we've cycled through all
            const nextIndex = (system.followDroneIndex + 1) % droneCount;
            if (nextIndex === 0) {
                // Full cycle → exit follow mode and restore free camera
                this._exitFollowMode(system);
                return true;
            }
            system.followDroneIndex = nextIndex;
        } else {
            // Activate follow on first drone
            system.autoFollow = true;
            system.followDroneIndex = 0;
        }
        
        const drone = system.drones[system.followDroneIndex];
        const droneId = drone?.id || `Drone ${system.followDroneIndex + 1}`;
        this.showFeedback(`Follow: ${droneId} (${system.followDroneIndex + 1}/${droneCount})`, 'success');
        return true;
    }

    /**
     * Exit follow mode and smoothly restore the camera to fleet overview.
     */
    _exitFollowMode(system) {
        system.autoFollow = false;
        system.followDroneIndex = -1;

        // Restore camera to fleet overview (zoom-to-swarm)
        try {
            const { center, radius } = system.computeFleetBounds();
            const dist = Math.max(radius * 2.5, 30);
            system.camera.position.set(
                center.x + dist * 0.7,
                center.y + dist * 0.5,
                center.z + dist * 0.7
            );
            system.controls.target.copy(center);
            system.controls.update();
        } catch (_) { /* noop */ }

        this.showFeedback('Follow: OFF — Caméra libre', 'info');
    }

    /**
     * Remove visual artifacts from the 3D scene (comm lines, spheres, particles).
     */
    _cleanVisualArtifacts(system) {
        const scene = system?.scene;
        if (!scene) return;
        // Remove swarm connection lines and intelligence sphere
        const toRemove = scene.children.filter(c =>
            c.userData.isSwarmConnection || c.userData.isIntelligenceSphere
        );
        toRemove.forEach(obj => {
            scene.remove(obj);
            if (obj.geometry) obj.geometry.dispose();
            if (obj.material) obj.material.dispose();
        });
        // Reset visual enhancements (particles, sounds)
        const ve = system.integratedController?.visualEnhancements;
        if (ve) {
            ve.dispose();
            ve.particleSystems = new Map();
            ve.soundEffects = new Map();
        }
        // Hide all active comm wave beams/pulses
        const cwr = system.integratedController?.autonomousFlightEngine?._commWaveRenderer;
        if (cwr) {
            cwr.setEnabled(false);
            cwr.setEnabled(true);
        }
    }
    
    // ==========================================
    // HANDLERS - Display
    // ==========================================
    
    handleToggleMinimap() {
        console.log('[BTN-TEST] handleToggleMinimap() called');
        if (window.minimapManager) {
            window.minimapManager.toggleAll();
            this.showFeedback('Minimaps toggled', 'info');
        } else {
            const container = document.getElementById('minimaps-container');
            if (container) {
                const isHidden = container.style.display === 'none' || getComputedStyle(container).display === 'none';
                container.style.display = isHidden ? 'flex' : 'none';
                this.showFeedback(`Minimaps: ${isHidden ? 'ON' : 'OFF'}`, 'info');
            }
        }
        return true;
    }
    
    handleToggleDiamantsHUD() {
        console.log('[BTN-TEST] handleToggleDiamantsHUD() called');
        const system = window.diamantsSystem;
        const hud = system?.integratedController?.harmonicsHUD;
        if (hud?.element) {
            const el = hud.element;
            const isHidden = el.style.display === 'none';
            hud.setVisible(isHidden);
            // Update button state
            const btn = document.getElementById('btn-toggle-diamants');
            if (btn) btn.style.opacity = isHidden ? '1' : '0.5';
            this.showFeedback(`DIAMANTS HUD: ${isHidden ? 'ON' : 'OFF'}`, 'info');
        } else {
            this.showFeedback('DIAMANTS HUD non disponible', 'warning');
        }
        return true;
    }

    handleToggleLabels() {
        console.log('[BTN-TEST] handleToggleLabels() called');
        const system = window.diamantsSystem;
        const controller = system?.integratedController;
        if (!controller) {
            this.showFeedback('Contrôleur non disponible', 'warning');
            return true;
        }
        // Toggle visibility on all drones' _labelSprite
        const drones = controller.drones || [];
        // Determine current state from first drone that has a label
        let currentlyVisible = true;
        for (const drone of drones) {
            if (drone._labelSprite) {
                currentlyVisible = drone._labelSprite.visible;
                break;
            }
        }
        const newVisible = !currentlyVisible;
        for (const drone of drones) {
            if (drone._labelSprite) {
                drone._labelSprite.visible = newVisible;
            }
        }
        // Update button visual
        const btn = document.getElementById('btn-toggle-labels');
        if (btn) btn.style.opacity = newVisible ? '1' : '0.5';
        this.showFeedback(`Labels 3D: ${newVisible ? 'ON' : 'OFF'}`, 'info');
        return true;
    }

    handleToggleCommWaves() {
        console.log('[BTN-TEST] handleToggleCommWaves() called');
        const system = window.diamantsSystem;
        const engine = system?.integratedController?.flightEngine;
        const renderer = engine?._commWaveRenderer;
        if (renderer) {
            const newState = !renderer.isEnabled();
            renderer.setEnabled(newState);
            const btn = document.getElementById('btn-toggle-comm');
            if (btn) btn.style.opacity = newState ? '1' : '0.5';
            this.showFeedback(`Comm Waves: ${newState ? 'ON' : 'OFF'}`, 'info');
        } else {
            // Fallback: toggle the global directly
            const current = window.DIAMANTS_SHOW_COMM_WAVES !== false;
            window.DIAMANTS_SHOW_COMM_WAVES = !current;
            const btn = document.getElementById('btn-toggle-comm');
            if (btn) btn.style.opacity = !current ? '1' : '0.5';
            this.showFeedback(`Comm Waves: ${!current ? 'ON' : 'OFF'}`, 'info');
        }
        return true;
    }

    handleToggleCommPanel() {
        const panel = document.getElementById('comm-panel-map');
        if (panel) {
            const wasHidden = panel.classList.contains('minimap-hidden');
            panel.classList.toggle('minimap-hidden');
            const btn = document.getElementById('btn-toggle-comm-panel');
            if (btn) btn.style.opacity = wasHidden ? '1' : '0.5';
            this.showFeedback(`Comm Panel: ${wasHidden ? 'ON' : 'OFF'}`, 'info');
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
        const backdrop = document.getElementById('mobile-sidebar-backdrop');
        const mobile = window.innerWidth <= 768;
        
        if (!panel) return false;
        
        const isHidden = panel.style.display === 'none' || !panel.style.display;
        
        panel.style.display = isHidden ? 'block' : 'none';
        if (canvas) canvas.classList.toggle('panel-visible', isHidden);
        if (button) {
            button.classList.toggle('panel-visible', isHidden);
            button.innerHTML = isHidden ? (mobile ? '✕' : '◄') : (mobile ? '☰' : '►');
        }
        // Mobile backdrop
        if (backdrop && mobile) {
            backdrop.classList.toggle('active', isHidden);
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
        
        // Appliquer aux drones en vol — set _cruiseAlt (read by engine as effective altitude)
        const system = window.diamantsSystem;
        if (system?.integratedController?.autonomousFlightEngine) {
            const engine = system.integratedController.autonomousFlightEngine;
            if (engine.drones) {
                engine.drones.forEach((state) => {
                    if (state.phase !== 'IDLE' && state.phase !== 'LANDING' && state.phase !== 'LANDED') {
                        state._cruiseAlt = value;
                        state.profile.cruiseAlt = value;
                    }
                });
            }
        }
        log(`🎯 Altitude appliquée: ${value}m`);
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
        
        // Mettre à jour la distance de sécurité dans le moteur de vol
        const system = window.diamantsSystem;
        if (system?.integratedController?.autonomousFlightEngine) {
            system.integratedController.autonomousFlightEngine.safetyDistanceOverride = value;
        }
        log(`🛡️ Distance sécurité appliquée: ${value}m`);
    }
    
    applyExplorationBounds(bx, by) {
        const system = window.diamantsSystem;
        if (system?.integratedController?.autonomousFlightEngine) {
            const engine = system.integratedController.autonomousFlightEngine;
            // halfBounds = demi-zone (le moteur clamp les waypoints à ±halfBounds)
            engine.halfBounds = { x: bx / 2, z: by / 2 };
            engine.explorationBounds = Math.max(bx, by) / 2;
        }
        // Belt-and-suspenders: update BOTH doctrineManager and window.DIAMANTS_DOCTRINE
        if (this.doctrineManager) {
            this.doctrineManager.zoneParams.sizeX = bx;
            this.doctrineManager.zoneParams.sizeZ = by;
        }
        if (window.DIAMANTS_DOCTRINE && window.DIAMANTS_DOCTRINE !== this.doctrineManager) {
            window.DIAMANTS_DOCTRINE.zoneParams.sizeX = bx;
            window.DIAMANTS_DOCTRINE.zoneParams.sizeZ = by;
        }
        // Force immediate minimap update (don't wait for next render frame)
        const maxZone = Math.max(bx, by);
        if (window.DIAMANTS_MINIMAP) {
            window.DIAMANTS_MINIMAP.config.zoneSize = maxZone;
        }
        if (window.DIAMANTS_DISCOVERY) {
            window.DIAMANTS_DISCOVERY.config.zoneSize = maxZone;
        }
        log(`📐 Zone exploration appliquée: ${bx}×${by}m (minimaps → ${maxZone}m)`);
    }
    
    sendWebSocketCommand(type, data) {
        const system = window.diamantsSystem;
        if (system?.ros?.ws?.readyState === WebSocket.OPEN) {
            try {
                system.ros.ws.send(JSON.stringify({ type, data }));
                log(`📡 WS: ${type}`, data);
                return true;
            } catch (e) {
                warn('WebSocket send error:', e);
                this.showFeedback(`Erreur WebSocket — ${type}`, 'error');
                return false;
            }
        }

        const casLevel = window.DIAMANTS?.casLevel ?? 2;
        const now = Date.now();
        if (!this._lastWsNoopToast || now - this._lastWsNoopToast > 8000) {
            this._lastWsNoopToast = now;
            if (casLevel >= 2) {
                this.showFeedback('Mode autonome (CAS-2) — exécution locale, backend offline', 'info');
            } else {
                this.showFeedback(`Backend indisponible — ${type} non transmis`, 'warning');
            }
        }
        return false;
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
                z-index: 5000;
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
        const globalFns = ['launchMission', 'emergencyLand', 'takeoffAllDrones', 'resetCamera', 'resetMission', 'landAllDrones', 'toggleMinimap', 'toggleLLM'];
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

        // Test 6: Intelligence LLM
        const mgr = this._getIntelligenceManager();
        const test6 = mgr !== null;
        results.tests.push({ name: 'Intelligence Manager', passed: test6 });
        test6 ? results.passed++ : results.failed++;
        
        // Afficher résultats
        log('📊 Résultats:');
        results.tests.forEach(t => {
            log(`   ${t.passed ? '✅' : '❌'} ${t.name}`);
        });
        log(`   Total: ${results.passed}/${results.passed + results.failed}`);
        
        return results;
    }

    // ==========================================
    // HANDLERS - Intelligence LLM
    // ==========================================

    /** @private Get the DroneIntelligenceManager from the system */
    _getIntelligenceManager() {
        return window.diamantsSystem?.integratedController?.droneIntelligenceManager || null;
    }

    /** Toggle LLM on/off globally */
    handleToggleLLM() {
        const mgr = this._getIntelligenceManager();
        if (!mgr) {
            console.warn('[LLM] Intelligence Manager not available');
            return;
        }
        const nowEnabled = !mgr.globalEnabled;
        mgr.setGlobalEnabled(nowEnabled);
        // Track user intent: prevent auto-enable from overriding a deliberate OFF
        this._userToggledLLMOff = !nowEnabled;
        
        const btn = document.getElementById('btn-toggle-llm');
        if (btn) {
            btn.textContent = nowEnabled ? '🧠 LLM ON' : '🧠 LLM OFF';
            btn.style.background = nowEnabled
                ? 'linear-gradient(45deg, #1a4a2a, #2a8a3a)'
                : 'linear-gradient(45deg, #2a1a4a, #5a2a8a)';
        }
        console.log(`[LLM] Global LLM ${nowEnabled ? 'ENABLED' : 'DISABLED'}`);
        this._updateLLMStatusDot();
    }

    /** Set model for a drone type */
    handleSetLLMModel(droneType, model) {
        const mgr = this._getIntelligenceManager();
        if (!mgr) return;
        mgr.setModelForType(droneType, model);
        console.log(`[LLM] Model for ${droneType} → ${model}`);
    }

    /** Set global LLM influence weight */
    handleSetLLMInfluence(value) {
        const mgr = this._getIntelligenceManager();
        if (!mgr) return;
        mgr.setGlobalInfluence(value);
        const display = document.getElementById('llm-influence-value');
        if (display) display.textContent = value.toFixed(2);
        console.log(`[LLM] Global influence → ${value.toFixed(2)}`);
    }

    /** Change Ollama server URL */
    handleSetOllamaUrl(url) {
        const mgr = this._getIntelligenceManager();
        if (!mgr?.connector) return;
        const cleanUrl = url.replace(/\/$/, '');
        mgr.connector.updateConfig({ baseUrl: cleanUrl });
        console.log(`[LLM] Ollama URL → ${cleanUrl}`);
    }

    /** Ping Ollama server to check connectivity */
    async handlePingOllama() {
        const btn = document.getElementById('btn-ping-ollama');
        if (btn) { btn.textContent = '⏳ Ping…'; btn.style.background = 'linear-gradient(45deg, #333, #555)'; }

        let ok = false;
        const mgr = this._getIntelligenceManager();
        // Try via connector first, fallback to direct fetch
        if (mgr?.connector) {
            try {
                const result = await mgr.connector.ping();
                ok = result && result.ok === true;
            } catch (e) {
                console.error('[LLM] Ping via connector failed:', e.message);
            }
        }
        if (!ok) {
            // Fallback: direct fetch (works even before manager is ready)
            const url = document.getElementById('ollama-url')?.value || 'http://localhost:11434';
            try {
                const ctrl = new AbortController();
                const timer = setTimeout(() => ctrl.abort(), 3000);
                const res = await fetch(`${url}/api/tags`, { signal: ctrl.signal });
                clearTimeout(timer);
                ok = res.ok;
            } catch (e) {
                console.warn('[LLM] Ollama ping (direct) failed:', e.message);
            }
        }
        console.log(`[LLM] Ollama ping: ${ok ? '✅ connected' : '❌ unreachable'}`);
        this._updateOllamaStatus(ok);
        // Auto-enable LLM when Ollama is reachable — but only if user hasn't manually toggled OFF
        if (ok && !this._userToggledLLMOff) {
            const mgr = this._getIntelligenceManager();
            if (mgr && !mgr.globalEnabled) {
                mgr.setGlobalEnabled(true);
                const btn = document.getElementById('btn-toggle-llm');
                if (btn) {
                    btn.textContent = '🧠 LLM ON';
                    btn.style.background = 'linear-gradient(45deg, #1a4a2a, #2a8a3a)';
                }
                console.log('[LLM] ✅ Auto-enabled LLM (Ollama detected)');
            }
        }
    }

    /** Reset all LLM brains (clear memory, re-enable failed ones) */
    handleResetLLMBrains() {
        const mgr = this._getIntelligenceManager();
        if (!mgr) return;
        for (const [id, brain] of mgr.brains) {
            brain.reset();
            // Respect global toggle: only re-enable if global is ON AND profile allows it
            brain.setEnabled(mgr.globalEnabled && brain.profile?.enableLLM !== false);
        }
        // Clear cached LLM results in engine
        const engine = window.diamantsSystem?.integratedController?.autonomousFlightEngine;
        if (engine?._llmResults) engine._llmResults.clear();
        console.log('[LLM] All brains reset');
    }

    /** Update ALL Ollama status indicators (dot + button + header) */
    _updateOllamaStatus(ok) {
        if (ok === true) this._lastPingOk = true;
        else if (ok === false) this._lastPingOk = false;

        const isUp = this._lastPingOk === true || this._getIntelligenceManager()?.connector?.isAvailable;
        const isDown = this._lastPingOk === false;
        // Demo mode: Ollama offline but drones are running — show demo status
        const hasDrones = (window.diamantsSystem?.integratedController?.drones?.length || 0) > 0;
        const isDemoMode = !isUp && hasDrones;

        // 1) Status dot
        const dot = document.getElementById('llm-status-dot');
        if (dot) {
            dot.style.background = isUp ? '#0f0' : isDemoMode ? '#fa0' : isDown ? '#f00' : '#555';
            dot.title = isUp ? 'Ollama: connecté ✅' : isDemoMode ? 'Mode démo: IA simulée 🟡' : isDown ? 'Ollama: injoignable ❌' : 'Ollama: inconnu';
        }

        // 2) Ping button — clear visual feedback
        const btn = document.getElementById('btn-ping-ollama');
        if (btn) {
            if (isUp) {
                btn.textContent = '✅ Ollama OK';
                btn.style.background = 'linear-gradient(45deg, #0a4a0a, #1a8a1a)';
            } else if (isDemoMode) {
                btn.textContent = '🟡 Mode Démo';
                btn.style.background = 'linear-gradient(45deg, #4a3a0a, #8a6a1a)';
            } else if (isDown) {
                btn.textContent = '❌ Offline';
                btn.style.background = 'linear-gradient(45deg, #4a0a0a, #8a1a1a)';
            } else {
                btn.textContent = '🔌 Ping';
                btn.style.background = 'linear-gradient(45deg, #333, #555)';
            }
        }

        // 3) Section header — single status emoji (use ID to avoid duplicates)
        let headerStatus = document.getElementById('llm-header-status-icon');
        if (!headerStatus) {
            const h3 = document.querySelector('#intelligence-content')?.parentElement?.querySelector('h3');
            if (h3) {
                headerStatus = document.createElement('span');
                headerStatus.id = 'llm-header-status-icon';
                headerStatus.style.cssText = 'margin-left: 6px; font-size: 10px;';
                h3.appendChild(headerStatus);
            }
        }
        if (headerStatus) {
            headerStatus.textContent = isUp ? '🟢' : isDemoMode ? '🟡' : isDown ? '🔴' : '';
        }
    }

    /** @deprecated Use _updateOllamaStatus instead */
    _updateLLMStatusDot(forceState) { this._updateOllamaStatus(forceState); }

    /** Start a periodic LLM stats updater */
    startLLMStatsUpdater() {
        if (this._llmStatsInterval) return;
        // Auto-ping Ollama on startup so the dot turns green immediately
        setTimeout(() => this.handlePingOllama(), 500);

        // Listen for LLM decision events to populate the live feed
        this._llmDecisions = [];
        window.addEventListener('diamants:llm-decision', (e) => {
            this._addLLMDecisionToFeed(e.detail);
        });

        this._llmStatsInterval = setInterval(() => {
            const mgr = this._getIntelligenceManager();
            if (mgr) {
                const stats = mgr.getStats();
                const conn = stats.connector || {};
                const el = (id) => document.getElementById(id);
                if (el('llm-stat-requests')) el('llm-stat-requests').textContent = conn.requests || 0;
                if (el('llm-stat-cache')) el('llm-stat-cache').textContent = conn.cacheHits || 0;
                if (el('llm-stat-failures')) el('llm-stat-failures').textContent = conn.failures || 0;
                if (el('llm-stat-latency')) el('llm-stat-latency').textContent = conn.avgLatencyMs ? `${conn.avgLatencyMs}ms` : '-';
            }
            this._updateLLMStatusDot();
            this._updateBrainsSummary();
        }, 2000);

        // ── Demo LLM decisions when Ollama is not available ──
        this._startDemoDecisions();
    }

    /** Generate LLM decision entries from real drone perception/decision/action state */
    _startDemoDecisions() {
        // ── Helper: compass direction from heading (radians) ──
        const headingToCompass = (heading) => {
            const dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
            const deg = ((heading || 0) * 180 / Math.PI + 360) % 360;
            return dirs[Math.round(deg / 45) % 8];
        };

        // ── Helper: sensor direction name ──
        const SENSOR_DIRS = ['avant', 'arrière', 'gauche', 'droite', 'dessus'];

        // ── Helper: get real peer drone name ──
        const getPeerName = (excludeId) => {
            const ctrl = window.diamantsSystem?.integratedController;
            const drones = ctrl?.drones || [];
            if (drones.length < 2) return 'x500_leader';
            const others = drones.filter(d => d.id !== excludeId);
            // Pair déterministe (pas de Math.random) — mode démo/offline
            return others[0]?.id || 'x500_leader';
        };

        // ── Build reasoning from actual drone state ──
        const buildRealReasoning = (droneState, drone) => {
            if (!droneState) return null;

            const phase = droneState.phase || 'IDLE';
            const pos = droneState.position;
            const vel = droneState.velocity;
            const heading = droneState.heading;
            const compass = headingToCompass(heading);
            const alt = pos?.y?.toFixed(1) || '0.0';
            const speed = vel ? Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z).toFixed(1) : '0.0';
            const speed2D = vel ? Math.sqrt(vel.x * vel.x + vel.z * vel.z).toFixed(1) : '0.0';
            const action = droneState._lastAction || phase;
            const decision = droneState._lastDecision || '';
            const sensors = droneState.sensorRanges || [3.5, 3.5, 3.5, 3.5, 3.5];
            const alerts = droneState.sensorAlerts || { warning: [], critical: [] };
            const attitude = droneState.attitude || { roll: 0, pitch: 0 };
            const waypointsVisited = droneState.waypointsVisited || 0;
            const wpDist = droneState.waypoint ? Math.sqrt(
                (pos.x - droneState.waypoint.x) ** 2 + (pos.z - droneState.waypoint.z) ** 2
            ).toFixed(1) : '—';
            const coverage = window.diamantsSystem?.integratedController?.intelligenceCollective?.metrics?.coverage;
            const covPct = coverage ? Math.floor(coverage * 100) : null;
            const minSensor = Math.min(...sensors.slice(0, 4));
            const minSensorIdx = sensors.indexOf(minSensor);
            const minSensorDir = SENSOR_DIRS[minSensorIdx] || 'avant';
            const cruiseAlt = droneState._cruiseAlt || droneState.profile?.cruiseAlt || 3;

            // ── Perception layer: what the drone sees ──
            let perception;
            if (alerts.critical?.length > 0) {
                perception = `CRITIQUE: obstacle ${minSensorDir} à ${minSensor.toFixed(1)}m — capteur ToF alerte critique`;
            } else if (alerts.warning?.length > 0) {
                perception = `Obstacle détecté ${minSensorDir} à ${minSensor.toFixed(1)}m — zone de sécurité entamée`;
            } else if (minSensor < 2.0) {
                perception = `Proximité ${minSensorDir}: ${minSensor.toFixed(1)}m — surveillance active`;
            } else {
                // No obstacle - describe exploration perception
                const sensorStatus = sensors.slice(0, 4).every(s => s > 3.0) ? 'dégagé 360°' : `min ${minSensor.toFixed(1)}m ${minSensorDir}`;
                perception = `Environnement ${sensorStatus}, altitude ${alt}m AGL`;
            }

            // ── Decision layer: what the drone decided ──
            let decisionText;
            switch (phase) {
                case 'TAKEOFF':
                    decisionText = `Montée vers ${cruiseAlt}m — vitesse verticale ${vel?.y?.toFixed(1) || '0.8'}m/s, stabilisation PID`;
                    break;
                case 'HOVER':
                    decisionText = `Maintien position [${pos?.x?.toFixed(0) || 0}, ${pos?.z?.toFixed(0) || 0}] — hover alt ${alt}m`;
                    break;
                case 'EXPLORE': {
                    if (droneState._returningToBase) {
                        const home = droneState.homePosition;
                        const distHome = home ? Math.sqrt((pos.x - home.x) ** 2 + (pos.z - home.z) ** 2).toFixed(0) : '?';
                        decisionText = `Retour base — distance ${distHome}m, cap ${compass}`;
                    } else if (action.includes('ESCAPE')) {
                        decisionText = `Anti-blocage: déviation d'urgence, vitesse 2D = ${speed2D}m/s`;
                    } else if (action.includes('BEACON')) {
                        decisionText = `Convergence balise — ${decision}`;
                    } else if (action.includes('MISSION')) {
                        decisionText = `Exécution mission LLM — waypoint à ${wpDist}m, cap ${compass}`;
                    } else if (action.includes('FORMATION')) {
                        decisionText = `Mode formation — repositionnement d=${wpDist}m, cap ${compass}`;
                    } else if (action.includes('SWARM')) {
                        decisionText = `Intelligence essaim — secteur Voronoï, WP à ${wpDist}m`;
                    } else if (action.includes('REDIR')) {
                        decisionText = `Redirection reçue — ordre cognitif, cap ${compass}`;
                    } else {
                        decisionText = `Exploration ${compass} — WP à ${wpDist}m, ${waypointsVisited} points visités`;
                        if (covPct !== null) decisionText += `, couverture ${covPct}%`;
                    }
                    break;
                }
                case 'LAND':
                    decisionText = `Atterrissage en cours — alt ${alt}m, descent contrôlée`;
                    break;
                case 'LANDED':
                    decisionText = `Au sol — moteurs coupés, ${waypointsVisited} waypoints complétés`;
                    break;
                case 'IDLE': {
                    // Get current scenario/mission context for richer decisions
                    const scenEng = window.DIAMANTS?.scenarioEngine;
                    const activeScId = scenEng ? [...(scenEng.scenarios?.entries() || [])].find(([k,v]) => scenEng.isActive(k))?.[0] : null;
                    const activeScen = activeScId ? scenEng.get(activeScId) : null;
                    const docMgr = window.diamantsSystem?.integratedController?.doctrineManager
                        || window.DIAMANTS?.doctrineManager;
                    const doctrine = docMgr?.currentDoctrine?.name || 'Exploration';
                    const coa = docMgr?.currentCOA?.name || 'Adaptatif';

                    const idleVariants = [
                        `Pré-vol: calibration IMU + magnétomètre, attente GPS fix (— sat)`,
                        `Initialisation capteurs ToF + barométrique, auto-check ESC —/4`,
                        `Diagnostic système: batterie —%, moteurs OK, liaison radio — dBm`,
                        `Mode standby — doctrine «${doctrine}» / COA «${coa}» chargées, prêt au takeoff`,
                        `Standby ordre de mission — algorithme Voronoï pré-calculé, ${drones.length} drones dans l'essaim`,
                        `Pre-flight check: nominal motor temperature, wind estimate —m/s`,
                    ];
                    if (activeScen) {
                        idleVariants.push(`Scénario «${activeScen.name}» actif — configuration en cours, attente synchronisation essaim`);
                        idleVariants.push(`Briefing scénario: ${activeScen.description?.slice(0, 80) || activeScen.name} — prêt`);
                    }
                    decisionText = idleVariants[Math.floor(Math.random() * idleVariants.length)];
                    break;
                }
                default:
                    decisionText = `Phase ${phase} — ${action} ${decision}`;
            }

            // ── Action layer: what the drone is physically doing ──
            let actionText;
            const rollDeg = (attitude.roll * 180 / Math.PI).toFixed(1);
            const pitchDeg = (attitude.pitch * 180 / Math.PI).toFixed(1);
            if (phase === 'IDLE' || phase === 'LANDED') {
                const idleActions = [
                    `Au sol pos [${pos?.x?.toFixed(0) || 0}, ${pos?.z?.toFixed(0) || 0}] — moteurs arrêtés, télémétrie active`,
                    `Stationnaire on ground — vérification liens essaim, latency —ms`,
                    `Slot base [${pos?.x?.toFixed(0) || 0}, ${pos?.z?.toFixed(0) || 0}] — prêt takeoff, batt —%`,
                    `Initial position — 3D GPS fix, HDOP —, data compression ok`,
                ];
                actionText = idleActions[Math.floor(Math.random() * idleActions.length)];
            } else if (alerts.critical?.length > 0) {
                actionText = `Évitement urgence: force latérale ${droneState.sensorAlerts?.obstacleVector ? 'active' : 'appliquée'}, roll=${rollDeg}°`;
            } else if (parseFloat(speed2D) > 2.0) {
                actionText = `Vol cap ${compass} à ${speed2D}m/s, alt ${alt}m, attitude [R:${rollDeg}° P:${pitchDeg}°]`;
            } else if (parseFloat(speed2D) > 0.3) {
                actionText = `Approche lente ${compass} à ${speed2D}m/s, stabilisation fine`;
            } else {
                actionText = `Stationnaire [${pos?.x?.toFixed(0) || 0}, ${alt}, ${pos?.z?.toFixed(0) || 0}], vit=${speed}m/s`;
            }

            // ── Compose full OODA-style reasoning ──
            const reasoning = `📡 ${perception} → 🧠 ${decisionText} → ⚡ ${actionText}`;

            // Determine actual action keyword for feed display
            const PHASE_ACTIONS = {
                IDLE: ['pre-flight', 'calibration', 'diagnostics', 'standby', 'sensor init'],
                LANDED: ['on ground', 'post-mission', 'report'],
                TAKEOFF: ['climb', 'takeoff'],
                HOVER: ['hover', 'hold pos'],
                LAND: ['landing', 'descent'],
            };
            let feedAction;
            if (PHASE_ACTIONS[phase]) {
                const arr = PHASE_ACTIONS[phase];
                feedAction = arr[Math.floor(Math.random() * arr.length)];
            } else {
                feedAction = action.replace(/[🔄📨🎯🛡️💀]/g, '').trim().toLowerCase() || phase.toLowerCase();
                if (feedAction.length > 20) feedAction = feedAction.slice(0, 20);
            }

            // Confidence based on sensor clarity and speed stability
            let confidence = 0.82;
            if (alerts.critical?.length > 0) confidence = 0.45;
            else if (alerts.warning?.length > 0) confidence = 0.62;
            else if (phase === 'EXPLORE' && parseFloat(speed2D) > 1.0) confidence = 0.88;
            else if (phase === 'HOVER') confidence = 0.90;
            else if (phase === 'TAKEOFF') confidence = 0.75;
            else if (phase === 'LAND') confidence = 0.70;
            else if (phase === 'IDLE') confidence = 0.92;   // pre-flight = high confidence
            else if (phase === 'LANDED') confidence = 0.85;
            // Confidence déterministe (jitter aléatoire retiré — mode démo/offline)

            // Determine direction from actual heading, not position
            const direction = compass;

            return { phase, feedAction, direction, reasoning, confidence };
        };

        let demoCount = 0;
        // Track last emission per drone to avoid repetition
        const lastEmitted = new Map();

        const emit = () => {
            // Skip if real Ollama is actually connected
            const mgr = this._getIntelligenceManager();
            if (mgr?.connector?._connected && mgr?.connector?._requestCount > 0) return;

            const engine = window.diamantsSystem?.integratedController?.autonomousFlightEngine;
            const ctrl = window.diamantsSystem?.integratedController;
            const drones = ctrl?.drones || [];

            // Fallback synthetic drone names when fleet not yet loaded
            const SYNTH_DRONES = ['x500_leader', 'x500_explorer_1', 'x500_explorer_2', 's500_scout_1', 'crazyflie_1', 'crazyflie_2', 'crazyflie_3'];
            const useSynthetic = drones.length === 0;

            // Pick a drone, preferring ones not recently emitted
            let droneIdx;
            const now = Date.now();
            let droneId, drone, droneState;

            if (useSynthetic) {
                // No fleet yet — use synthetic drone names
                const synthCandidates = SYNTH_DRONES.map((id, i) => ({
                    idx: i, id, lastT: lastEmitted.get(id) || 0
                })).sort((a, b) => a.lastT - b.lastT);
                const pick = synthCandidates[Math.floor(Math.random() * Math.max(1, Math.floor(synthCandidates.length / 2)))];
                droneId = pick.id;
                drone = null;
                droneState = null;
            } else {
                const candidates = drones.map((d, i) => ({
                    idx: i,
                    id: d.id,
                    lastT: lastEmitted.get(d.id) || 0
                })).sort((a, b) => a.lastT - b.lastT);
                const halfLen = Math.max(1, Math.floor(candidates.length / 2));
                droneIdx = candidates[Math.floor(Math.random() * halfLen)].idx;
                drone = drones[droneIdx];
                droneId = drone?.id || `drone_${droneIdx}`;
                droneState = engine?.drones?.get?.(droneId);
            }

            let result;
            if (droneState) {
                result = buildRealReasoning(droneState, drone);
            }
            // Fallback: generate a basic decision even without full engine state
            if (!result) {
                const pos = drone?.mesh?.position || drone?.position;
                const alt = pos?.y?.toFixed(1) || '0.0';
                const phases = ['IDLE', 'LANDED', 'HOVER', 'TAKEOFF', 'EXPLORE'];
                const phase = droneState?.phase || (pos?.y > 2 ? 'EXPLORE' : 'LANDED');
                const compassDirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
                const direction = compassDirs[Math.floor(Math.random() * 8)];
                const fallbackActions = {
                    IDLE: ['pre-flight', 'calibration', 'system diagnosticss', 'sensor init'],
                    LANDED: ['on ground', 'post-mission', 'sensor report', 'motor check'],
                    HOVER: ['hover', 'hold position', 'area scan'],
                    TAKEOFF: ['gradual climb', 'takeoff', 'altitude check'],
                    EXPLORE: ['exploration', 'corridor scan', 'autonomous navigation', 'avoidance']
                };
                const actions = fallbackActions[phase] || fallbackActions.LANDED;
                const feedAction = actions[Math.floor(Math.random() * actions.length)];
                const perceptions = [
                    `Alt ${alt}m, nominal sensors, light wind`,
                    `360° scan: no nearby obstacle, good visibility`,
                    `Battery OK, GPS fix 3D, — satellites`,
                    `Clear zone, fleet distance —m`,
                    `Stable telemetry, latency —ms`
                ];
                const decisions = [
                    `Maintien position sûre, monitoring continu`,
                    `Prêt pour prochaine phase mission`,
                    `Évaluation corridor ${direction}: praticable`,
                    `Coordination essaim: posture ${phase.toLowerCase()}`,
                    `Optimisation couverture zone en cours`
                ];
                const actionDescs = [
                    `${feedAction}, alt ${alt}m, cap ${direction}`,
                    `Exécution ${feedAction}, stabilisation active`,
                    `Mode ${phase.toLowerCase()}: ${feedAction}`
                ];
                const pick = arr => arr[Math.floor(Math.random() * arr.length)];
                const reasoning = `📡 ${pick(perceptions)} → 🧠 ${pick(decisions)} → ⚡ ${pick(actionDescs)}`;
                // Confidence déterministe neutre (pas de Math.random) — mode démo/offline
                const confidence = 0.65;
                result = { phase, feedAction, direction, reasoning, confidence };
            }

            // Mode démo/offline — libellé explicite (Ollama/LLM non connecté)
            const model = 'demo/offline';

            window.dispatchEvent(new CustomEvent('diamants:llm-decision', {
                detail: {
                    droneId,
                    action: result.feedAction,
                    direction: result.direction,
                    reasoning: result.reasoning,
                    confidence: Math.max(0.30, Math.min(0.95, result.confidence)),
                    phase: result.phase,
                    model,
                    _demo: true
                }
            }));
            lastEmitted.set(droneId, now);
            demoCount++;
        };

        // First batch after a short delay (fast start for demo mode)
        setTimeout(() => {
            emit();
            setTimeout(emit, 600);
            setTimeout(emit, 1200);
            setTimeout(emit, 2000);
        }, 1500);

        // Periodic decisions every 2-5 seconds
        const scheduleNext = () => {
            const delay = 2000 + Math.random() * 3000;
            this._demoDecisionTimer = setTimeout(() => {
                emit();
                scheduleNext();
            }, delay);
        };
        setTimeout(scheduleNext, 3500);
    }

    /** Add a decision entry to the LLM live feed */
    _addLLMDecisionToFeed(detail) {
        const feed = document.getElementById('llm-decision-feed');
        if (!feed) return;
        // Remove placeholder
        if (feed.children.length === 1 && feed.children[0].style?.textAlign === 'center') {
            feed.innerHTML = '';
        }
        const now = new Date();
        const ts = `${now.getHours().toString().padStart(2,'0')}:${now.getMinutes().toString().padStart(2,'0')}:${now.getSeconds().toString().padStart(2,'0')}`;
        const d = detail;
        // Color by drone type
        const typeColors = { crazyflie: '#0f8', x500: '#0af', s500: '#f80' };
        const typeKey = Object.keys(typeColors).find(k => d.droneId?.toLowerCase().includes(k)) || 'crazyflie';
        const color = typeColors[typeKey];
        // Confidence bar
        const conf = Math.round((d.confidence || 0) * 100);
        const confColor = conf > 70 ? '#0f0' : conf > 40 ? '#fa0' : '#f44';
        const row = document.createElement('div');
        row.style.cssText = `padding: 3px 4px; border-bottom: 1px solid #222; animation: fadeIn 0.3s; cursor: pointer;`;
        row.title = 'Cliquer pour voir le raisonnement complet';
        row.innerHTML = `
            <div style="display: flex; justify-content: space-between; align-items: center;">
                <span style="color: ${color}; font-weight: bold;">${d.droneId || '?'}</span>
                <span style="color: #666;">${ts}</span>
            </div>
            <div style="color: #c9f; margin: 2px 0;">🧠 ${d.action || 'think'} → <span style="color: #ff0;">${d.direction || '—'}</span></div>
            <div style="color: #aaa; font-style: italic; font-size: 8px;">${(d.reasoning || '').slice(0, 60)}</div>
            <div style="display: flex; align-items: center; gap: 4px; margin-top: 2px;">
                <div style="flex: 1; background: #1a1a2e; height: 4px; border-radius: 2px;"><div style="width: ${conf}%; background: ${confColor}; height: 4px; border-radius: 2px;"></div></div>
                <span style="color: ${confColor}; font-size: 8px;">${conf}%</span>
            </div>`;

        // Click to expand: show full reasoning in overlay
        row.addEventListener('click', () => this._showExpandedDecision(d, ts, color, conf, confColor));

        feed.insertBefore(row, feed.firstChild);
        // Keep max 30 entries
        while (feed.children.length > 30) feed.removeChild(feed.lastChild);

        // Show overlay toast on 3D view
        this._showDecisionToast(d, color, conf, confColor);
    }

    /** Show a brief toast notification on the 3D view for LLM decisions */
    _showDecisionToast(d, color, conf, confColor) {
        // Get or create toast container
        let container = document.getElementById('llm-decision-toasts');
        if (!container) {
            container = document.createElement('div');
            container.id = 'llm-decision-toasts';
            container.style.cssText = `
                position: fixed; top: 60px; left: 10px; z-index: 200;
                display: flex; flex-direction: column; gap: 4px;
                pointer-events: none; max-height: 40vh; overflow: hidden;
            `;
            document.body.appendChild(container);
        }

        const toast = document.createElement('div');
        toast.style.cssText = `
            background: rgba(5, 10, 25, 0.88); border: 1px solid ${color}44;
            border-left: 3px solid ${color}; border-radius: 6px;
            padding: 5px 10px; font-size: 10px; font-family: monospace;
            color: #ccc; backdrop-filter: blur(6px); -webkit-backdrop-filter: blur(6px);
            max-width: 260px; animation: fadeIn 0.3s, fadeOut 0.5s 4.5s forwards;
            pointer-events: auto; cursor: pointer;
        `;
        const shortReason = (d.reasoning || '').slice(0, 45);
        toast.innerHTML = `<span style="color:${color};font-weight:bold;">${d.droneId}</span> <span style="color:#c9f;">🧠 ${d.action||'think'}</span> → <span style="color:#ff0;">${d.direction||'—'}</span> <span style="color:${confColor};">${conf}%</span><br><span style="color:#888;font-size:9px;font-style:italic;">${shortReason}</span>`;

        toast.addEventListener('click', () => {
            const now = new Date();
            const ts2 = `${now.getHours().toString().padStart(2,'0')}:${now.getMinutes().toString().padStart(2,'0')}:${now.getSeconds().toString().padStart(2,'0')}`;
            this._showExpandedDecision(d, ts2, color, conf, confColor);
        });

        container.appendChild(toast);
        // Remove after 5 seconds
        setTimeout(() => { if (toast.parentNode) toast.remove(); }, 5000);
        // Keep max 5 toasts
        while (container.children.length > 5) container.removeChild(container.firstChild);
    }

    /** Show expanded decision modal with full LLM reasoning */
    _showExpandedDecision(d, ts, color, conf, confColor) {
        // Remove any existing overlay
        const existing = document.getElementById('llm-decision-overlay');
        if (existing) existing.remove();

        const overlay = document.createElement('div');
        overlay.id = 'llm-decision-overlay';
        overlay.style.cssText = `
            position: fixed; inset: 0; width: 100vw; height: 100vh;
            background: rgba(0,0,0,0.92); z-index: 99999;
            display: flex; align-items: center; justify-content: center;
            padding: 12px; box-sizing: border-box;
            -webkit-overflow-scrolling: touch;
        `;
        overlay.addEventListener('click', (e) => {
            if (e.target === overlay) overlay.remove();
        });

        const phaseColors = { EXPLORE:'#00ff88', HOVER:'#ffcc00', TAKEOFF:'#22ccff', LAND:'#ff6644', IDLE:'#888' };
        const pC = phaseColors[(d.phase||'').toUpperCase()] || '#fff';

        // Detect landscape
        const isLandscape = window.innerWidth > window.innerHeight;

        const card = document.createElement('div');
        card.style.cssText = `
            background: linear-gradient(135deg, #0a0e1a 0%, #121830 100%);
            border: 1px solid rgba(80,200,255,0.4); border-radius: 14px;
            padding: ${isLandscape ? '12px 16px' : '16px 18px'};
            width: 100%; max-width: ${isLandscape ? '520px' : '400px'};
            max-height: ${isLandscape ? '90vh' : '85vh'}; overflow-y: auto;
            box-shadow: 0 12px 40px rgba(0,100,255,0.3);
            font-family: 'Inter', -apple-system, sans-serif;
            box-sizing: border-box;
        `;
        card.addEventListener('click', (e) => e.stopPropagation());

        const fs = isLandscape ? 11 : 13; // base font size

        card.innerHTML = `
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; gap: 6px;">
                <div style="min-width: 0; overflow: hidden;">
                    <span style="color: ${color}; font-size: ${fs + 2}px; font-weight: bold;">🧠 ${d.droneId || '?'}</span>
                    <span style="color: ${pC}; font-size: ${fs - 2}px; margin-left: 6px; background: ${pC}22; padding: 2px 6px; border-radius: 6px;">${d.phase || 'EXPLORE'}</span>
                </div>
                <div style="color: #666; font-size: ${fs - 3}px; flex-shrink: 0;">${ts}</div>
            </div>
            <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; margin-bottom: 10px;">
                <div style="background: rgba(100,100,255,0.1); border-radius: 8px; padding: ${isLandscape ? '6px' : '8px'}; text-align: center; min-width: 0; overflow: hidden;">
                    <div style="color: #888; font-size: ${fs - 4}px; text-transform: uppercase;">Action</div>
                    <div style="color: #c9f; font-size: ${fs}px; font-weight: bold; margin-top: 3px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap;">${d.action || 'EXPLORE'}</div>
                </div>
                <div style="background: rgba(100,100,255,0.1); border-radius: 8px; padding: ${isLandscape ? '6px' : '8px'}; text-align: center; min-width: 0;">
                    <div style="color: #888; font-size: ${fs - 4}px; text-transform: uppercase;">Direction</div>
                    <div style="color: #ff0; font-size: ${fs}px; font-weight: bold; margin-top: 3px;">${d.direction || '—'}</div>
                </div>
                <div style="background: rgba(100,100,255,0.1); border-radius: 8px; padding: ${isLandscape ? '6px' : '8px'}; text-align: center; min-width: 0;">
                    <div style="color: #888; font-size: ${fs - 4}px; text-transform: uppercase;">Confidence</div>
                    <div style="color: ${confColor}; font-size: ${fs}px; font-weight: bold; margin-top: 3px;">${conf}%</div>
                    <div style="background: #1a1a2e; height: 4px; border-radius: 2px; margin-top: 3px;">
                        <div style="width: ${conf}%; background: ${confColor}; height: 4px; border-radius: 2px;"></div>
                    </div>
                </div>
            </div>
            <div style="margin-bottom: 10px;">
                <div style="color: #88aacc; font-size: ${fs - 3}px; text-transform: uppercase; margin-bottom: 3px;">🧠 Full reasoning</div>
                <div style="background: rgba(0,0,0,0.4); border: 1px solid #333; border-radius: 8px; padding: 10px; color: #ddd; font-size: ${fs - 1}px; line-height: 1.5; white-space: pre-wrap; word-break: break-word; overflow-wrap: break-word;">${d.reasoning || 'No reasoning available.'}</div>
            </div>
            <div style="display: flex; gap: 6px; color: #667; flex-wrap: wrap; margin-bottom: 8px;">
                <span style="font-size: ${fs - 4}px;">Model: ${d.model || 'mistral-nemo'}</span>
                <span style="font-size: ${fs - 4}px;">·</span>
                <span style="font-size: ${fs - 4}px;">${d.droneId?.includes('x500') ? 'Cognitive X500' : d.droneId?.includes('s500') ? 'Cognitive S500' : 'Reactive Crazyflie'}</span>
            </div>
            <button style="
                width: 100%; padding: 11px;
                background: linear-gradient(45deg, #2a3a5a, #3a5a8a); border: none; border-radius: 8px;
                color: #fff; font-size: ${fs + 1}px; cursor: pointer; touch-action: manipulation;
                -webkit-tap-highlight-color: rgba(100,200,255,0.2);
            " onclick="this.closest('#llm-decision-overlay').remove()">✕ Close</button>
        `;

        overlay.appendChild(card);
        document.body.appendChild(overlay);
    }

    /** Update the brains summary line */
    _updateBrainsSummary() {
        const el = document.getElementById('llm-brains-summary');
        if (!el) return;
        const ctrl = window.diamantsSystem?.integratedController;
        const droneCount = ctrl?.drones?.length || 0;
        const mgr = this._getIntelligenceManager();
        const ollamaUp = this._lastPingOk === true || mgr?.connector?.isAvailable;

        if (!mgr || (!ollamaUp && droneCount > 0)) {
            // No Ollama manager OR Ollama offline with drones → show demo simulation
            if (droneCount > 0) {
                el.innerHTML = `🧠 <span style="color:#0ff">${droneCount}/${droneCount}</span> cerveaux actifs | 🟡 DÉMO (offline) | 💭 —`;
            } else {
                el.innerHTML = '🧠 Initialisation cerveaux IA…';
            }
            return;
        }
        let active = 0, total = 0, thinking = 0;
        for (const [id, b] of mgr.brains) {
            total++;
            if (b.enabled) active++;
            if (b._lastReasoning) thinking++;
        }
        const globalState = mgr.globalEnabled ? '🟢 ON' : '🔴 OFF';
        el.innerHTML = `🧠 <span style="color:#0ff">${active}/${total}</span> cerveaux actifs | ${globalState} | 💭 ${thinking} pensent`;
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
