/**
 * DIAMANTS — UI Controller
 * ==========================
 * Contrôleur principal de l'interface utilisateur.
 * Gère la navigation, les interactions et le câblage avec le système.
 */

export class DiamantsUIController {
    constructor(diamantsSystem) {
        this.system = diamantsSystem;
        this.currentPanel = 'mission';
        this.sidebarCollapsed = false;
        this.autonomyLevel = 100; // 0 = centralisé, 100 = distribué
        
        // Cache des éléments DOM
        this.elements = {};
        
        // Configuration des panels
        this.panels = {
            mission: {
                icon: '🎯',
                label: 'Mission',
                component: 'panel-mission'
            },
            drones: {
                icon: '🚁',
                label: 'Drones',
                component: 'panel-drones'
            },
            analytics: {
                icon: '📊',
                label: 'Analytics',
                component: 'panel-analytics'
            },
            benchmark: {
                icon: '🧪',
                label: 'Benchmark',
                component: 'panel-benchmark'
            },
            config: {
                icon: '⚙️',
                label: 'Config',
                component: 'panel-config'
            }
        };

        this.init();
    }

    /**
     * Initialisation
     */
    init() {
        this.cacheElements();
        this.bindEvents();
        this.setupNavigation();
        this.startUpdateLoop();
        
        console.log('🎨 DiamantsUIController initialisé');
    }

    /**
     * Cache les références DOM
     */
    cacheElements() {
        // Header
        this.elements.statusIndicator = document.getElementById('status-indicator');
        this.elements.droneCount = document.getElementById('header-drone-count');
        this.elements.intelligence = document.getElementById('header-intelligence');
        this.elements.coverage = document.getElementById('header-coverage');
        
        // Navigation
        this.elements.navItems = document.querySelectorAll('.nav-item');
        this.elements.panels = document.querySelectorAll('.panel');
        
        // Mission Panel
        this.elements.missionType = document.getElementById('mission-type');
        this.elements.missionPattern = document.getElementById('mission-pattern');
        
        // Metrics
        this.elements.metricIntel = document.getElementById('metric-intelligence');
        this.elements.metricEmergence = document.getElementById('metric-emergence');
        this.elements.metricCohesion = document.getElementById('metric-cohesion');
        this.elements.metricCoord = document.getElementById('metric-coordination');
        this.elements.metricCoverage = document.getElementById('metric-coverage');
        this.elements.coverageBar = document.getElementById('coverage-progress');
        
        // Drone list
        this.elements.droneList = document.getElementById('drone-list');
    }

    /**
     * Bindage des événements
     */
    bindEvents() {
        // Navigation
        this.elements.navItems?.forEach(item => {
            item.addEventListener('click', (e) => {
                const panel = e.currentTarget.dataset.panel;
                this.switchPanel(panel);
            });
        });

        // Mission Controls
        this.bindButton('btn-launch', () => this.launchMission());
        this.bindButton('btn-emergency', () => this.emergencyLand());
        this.bindButton('btn-emergency-panel', () => this.emergencyLand());
        this.bindButton('btn-reset', () => this.resetSwarm());
        this.bindButton('btn-takeoff', () => this.takeoffAll());
        this.bindButton('btn-land', () => this.landAll());
        this.bindButton('btn-pattern', () => this.cyclePattern());
        
        // Camera
        this.bindButton('btn-camera-reset', () => this.resetCamera());
        this.bindButton('btn-camera-top', () => this.topView());
        this.bindButton('btn-camera-follow', () => this.toggleFollow());
        this.bindButton('btn-camera-fit', () => this.fitAll());
        
        // Benchmark
        this.bindButton('btn-benchmark-quick', () => this.runBenchmark(30));
        this.bindButton('btn-benchmark-full', () => this.runBenchmark(60));
        this.bindButton('btn-export-csv', () => window.benchmarkExportCSV?.());
        this.bindButton('btn-export-json', () => window.benchmarkExportJSON?.());
        
        // Config
        this.bindButton('btn-show-logs', () => this.toggleLogs());
        this.bindButton('btn-show-console', () => this.toggleConsole());
        
        // Sliders
        this.bindSlider('altitude-slider', 'altitude-value', (v) => this.setAltitude(v));
        this.bindSlider('safety-slider', 'safety-value', (v) => this.setSafetyDistance(v));
        
        // Autonomy slider (custom binding)
        this.setupAutonomySlider();
        
        // Toggles
        this.bindToggle('toggle-minimap', (v) => this.toggleMinimap(v));
        this.bindToggle('toggle-grid', (v) => this.toggleGrid(v));
        this.bindToggle('toggle-trails', (v) => this.toggleTrails(v));
    }

    bindButton(id, handler) {
        const el = document.getElementById(id);
        if (el) el.addEventListener('click', handler);
    }

    bindSlider(sliderId, valueId, handler) {
        const slider = document.getElementById(sliderId);
        const value = document.getElementById(valueId);
        if (slider) {
            slider.addEventListener('input', (e) => {
                const v = parseFloat(e.target.value);
                if (value) value.textContent = v.toFixed(1) + (sliderId.includes('altitude') ? 'm' : 'm');
                handler(v);
            });
        }
    }

    bindToggle(toggleId, handler) {
        const toggle = document.getElementById(toggleId);
        if (toggle) {
            toggle.addEventListener('change', (e) => {
                handler(e.target.checked);
            });
        }
    }

    /**
     * Setup navigation tabs
     */
    setupNavigation() {
        this.switchPanel(this.currentPanel);
    }

    /**
     * Switch entre les panels
     */
    switchPanel(panelId) {
        this.currentPanel = panelId;
        
        // Update nav items
        this.elements.navItems?.forEach(item => {
            item.classList.toggle('active', item.dataset.panel === panelId);
        });
        
        // Update panels
        this.elements.panels?.forEach(panel => {
            panel.classList.toggle('active', panel.id === `panel-${panelId}`);
        });
    }

    /**
     * Boucle de mise à jour UI
     */
    startUpdateLoop() {
        setInterval(() => this.updateUI(), 500);
    }

    updateUI() {
        const controller = this.system?.integratedController;
        const drones = this.system?.drones || [];
        const metrics = controller?.metrics || {};

        // Header status
        this.updateElement('header-drone-count', `${drones.filter(d => d.state !== 'IDLE').length}/${drones.length}`);
        this.updateElement('header-intelligence', ((metrics.collaborationEfficiency || 0) * 100).toFixed(0));
        this.updateElement('header-coverage', `${(metrics.coveragePercentage || 0).toFixed(0)}%`);

        // Metrics panel
        const totalIntel = this.calculateTotalIntelligence(metrics, drones);
        this.updateElement('metric-intelligence', totalIntel.toFixed(0));
        // Emergence: show phase state instead of raw percentage
        const eLevel = metrics.emergenceLevel || 0;
        const ePhase = eLevel < 0.01 ? 'CENTRALISÉ'
                     : eLevel < 0.15 ? 'TRANSITION'
                     : 'AUTO-ORG';
        this.updateElement('metric-emergence', ePhase);
        const elEm = document.getElementById('metric-emergence');
        if (elEm) {
            elEm.style.color = eLevel < 0.01 ? '#888' : eLevel < 0.15 ? '#FFAA00' : '#00FF88';
        }
        this.updateElement('metric-cohesion', this.calculateCohesion(drones).toFixed(0));
        this.updateElement('metric-coordination', ((metrics.collaborationEfficiency || 0) * 100).toFixed(0));
        this.updateElement('metric-coverage', `${(metrics.coveragePercentage || 0).toFixed(0)}%`);

        // Progress bars
        const coverageBar = document.getElementById('coverage-progress');
        if (coverageBar) coverageBar.style.width = `${metrics.coveragePercentage || 0}%`;

        // Drone list
        this.updateDroneList(drones);
    }

    updateElement(id, value) {
        const el = document.getElementById(id);
        if (el) el.textContent = value;
    }

    calculateTotalIntelligence(metrics, drones) {
        const coverage = (metrics.coveragePercentage || 0) / 100;
        const coordination = metrics.collaborationEfficiency || 0;
        const emergence = metrics.emergenceLevel || 0;
        const activeRatio = drones.filter(d => d.state && d.state !== 'IDLE').length / Math.max(1, drones.length);
        return (0.4 * coverage + 0.3 * coordination + 0.3 * emergence) * 100 * activeRatio;
    }

    calculateCohesion(drones) {
        if (drones.length < 2) return 0;
        
        let cx = 0, cy = 0, cz = 0, count = 0;
        drones.forEach(d => {
            if (d.position) {
                cx += d.position.x; cy += d.position.y; cz += d.position.z;
                count++;
            }
        });
        if (count === 0) return 0;
        cx /= count; cy /= count; cz /= count;
        
        let sumSqDist = 0;
        drones.forEach(d => {
            if (d.position) {
                const dist = Math.sqrt(
                    Math.pow(d.position.x - cx, 2) +
                    Math.pow(d.position.y - cy, 2) +
                    Math.pow(d.position.z - cz, 2)
                );
                sumSqDist += dist * dist;
            }
        });
        
        const stdDev = Math.sqrt(sumSqDist / count);
        return Math.max(0, 100 - Math.abs(stdDev - 3.0) * 15);
    }

    updateDroneList(drones) {
        const list = document.getElementById('drone-list');
        if (!list) return;

        if (drones.length === 0) {
            list.innerHTML = '<div class="text-muted" style="padding: 12px; text-align: center;">En attente de connexion...</div>';
            return;
        }

        list.innerHTML = drones.map((d, i) => {
            const isReal = d.rosData?.source === 'real' || d.rosData?.source === 'mavlink';
            const isAgent = d.rosData?.source === 'agent';
            let sourceTag = '';
            let itemClass = '';
            if (isReal) {
                sourceTag = '<span class="drone-source-tag real">RÉEL</span>';
                itemClass = 'real-agent';
            } else if (isAgent) {
                const engineLabel = d.rosData?.engine ? ` ${d.rosData.engine.toUpperCase()}` : '';
                sourceTag = `<span class="drone-source-tag agent">AGENT${engineLabel}</span>`;
                itemClass = 'agent-driven';
            }
            const batteryStr = d.rosData?.battery != null ? ` 🔋${Math.round(d.rosData.battery)}%` : '';
            return `
            <div class="drone-item ${itemClass}" data-drone="${i}">
                <span class="drone-status-dot ${this.getDroneStatusClass(d)}"></span>
                <div class="drone-info">
                    <div class="drone-name">CF${(i + 1).toString().padStart(2, '0')} ${sourceTag}</div>
                    <div class="drone-state">${d.state || 'IDLE'} • ${d.position ? `z:${d.position.y.toFixed(1)}m` : '--'}${batteryStr}</div>
                </div>
            </div>`;
        }).join('');
    }

    getDroneStatusClass(drone) {
        if (!drone.state || drone.state === 'IDLE') return 'idle';
        if (drone.state === 'EMERGENCY' || drone.state === 'ERROR') return 'error';
        if (drone.state === 'LANDING') return 'warning';
        return 'active';
    }

    // ═══════════════════════════════════════════════════════════
    // ACTIONS
    // ═══════════════════════════════════════════════════════════

    launchMission() {
        console.log('🚀 Launch Mission');
        window.launchMission?.();
    }

    emergencyLand() {
        console.log('🛑 Emergency Land');
        window.emergencyLand?.();
    }

    resetSwarm() {
        console.log('🔄 Reset Swarm');
        window.resetSwarm?.();
    }

    takeoffAll() {
        console.log('🛫 Takeoff All');
        window.takeoffAllDrones?.();
    }

    landAll() {
        console.log('🛬 Land All');
        window.landAllDrones?.();
    }

    resetCamera() {
        window.DIAMANTS?.resetCamera?.();
    }

    topView() {
        window.DIAMANTS?.topView?.();
    }

    toggleFollow() {
        // Toggle follow mode
        console.log('👁 Toggle follow mode');
    }

    fitAll() {
        window.DIAMANTS?.zoomToSwarm?.();
    }

    cyclePattern() {
        const patterns = ['grid', 'boustrophedon', 'spiral', 'swarm'];
        const select = document.getElementById('mission-pattern');
        if (select) {
            const currentIndex = patterns.indexOf(select.value);
            const nextIndex = (currentIndex + 1) % patterns.length;
            select.value = patterns[nextIndex];
        }
        window.changePattern?.();
    }

    setAltitude(value) {
        console.log(`🛫 Altitude: ${value}m`);
        // Set target altitude for all drones
    }

    setSafetyDistance(value) {
        console.log(`⚠️ Safety distance: ${value}m`);
        // Update safety distance
    }

    // ═══════════════════════════════════════════════════════════
    // AUTONOMY SLIDER
    // ═══════════════════════════════════════════════════════════

    /* ── Autonomy step definitions ── */
    static AUTONOMY_STEPS = [
        { step: 0, level:   0, icon: '🔗', mode: 'Centralisé',      short: 'CENTRAL',  desc: 'Tous les drones forment une seule unité coordonnée.' },
        { step: 1, level:  25, icon: '📐', mode: 'Guidé',           short: 'GUIDÉ',    desc: 'Formation unifiée avec micro-ajustements individuels.' },
        { step: 2, level:  50, icon: '🔀', mode: 'Hybride',         short: 'HYBRIDE',  desc: 'Objectifs de groupe + exploration locale autonome.' },
        { step: 3, level:  75, icon: '🧠', mode: 'Semi-autonome',   short: 'SEMI-AUTO', desc: 'Exploration indépendante, coordination par stigmergie.' },
        { step: 4, level:  90, icon: '🤖', mode: 'Autonome',        short: 'AUTONOME', desc: 'Agents indépendants, partage d\'information minimal.' },
        { step: 5, level: 100, icon: '⚡', mode: 'Distribué total', short: 'DISTRIBUÉ', desc: 'Agents 100% autonomes — aucun contrôle central.' },
    ];

    setupAutonomySlider() {
        const slider = document.getElementById('autonomy-slider');
        if (!slider) return;

        // Slider input → snap to step
        slider.addEventListener('input', (e) => {
            this.setAutonomyStep(parseInt(e.target.value, 10));
        });

        // Click on tick labels → jump to that step
        const ticks = document.getElementById('autonomy-ticks');
        if (ticks) {
            ticks.addEventListener('click', (e) => {
                const span = e.target.closest('[data-step]');
                if (!span) return;
                const step = parseInt(span.dataset.step, 10);
                slider.value = step;
                this.setAutonomyStep(step);
            });
        }

        // Initialize — full autonomy by default (step 5)
        this.setAutonomyStep(5);
    }

    /** Map a step index (0-5) → internal autonomy level (0-100) */
    setAutonomyStep(stepIdx) {
        const steps = this.constructor.AUTONOMY_STEPS;
        const def = steps[stepIdx] || steps[steps.length - 1];
        this.autonomyLevel = def.level;
        this.autonomyStep = def.step;
        this.autonomyMode = def.short;

        // Expose globally so bridge / engine can read it
        window.DIAMANTS_AUTONOMY_LEVEL = def.level;
        window.DIAMANTS_AUTONOMY_MODE  = def.short;

        // Sync slider position
        const slider = document.getElementById('autonomy-slider');
        if (slider) slider.value = def.step;

        // Highlight active tick label
        const tickSpans = document.querySelectorAll('#autonomy-ticks span');
        tickSpans.forEach(s => s.classList.toggle('active', parseInt(s.dataset.step, 10) === def.step));

        // Update thumb color to match current step
        const colors = ['#00BFFF','#33bbdd','#66DDAA','#88dd88','#aaee66','#00FF88'];
        const c = colors[def.step] || '#00FF88';
        if (slider) {
            slider.style.setProperty('--thumb-color', c);
            // Dynamic thumb glow via inline style override
            const css = `#autonomy-slider::-webkit-slider-thumb { border-color: ${c}; box-shadow: 0 0 10px ${c}88; }`;
            let styleEl = document.getElementById('autonomy-thumb-style');
            if (!styleEl) { styleEl = document.createElement('style'); styleEl.id = 'autonomy-thumb-style'; document.head.appendChild(styleEl); }
            styleEl.textContent = css + `\n#autonomy-slider::-moz-range-thumb { border-color: ${c}; box-shadow: 0 0 10px ${c}88; }`;
        }

        // Update info area
        const modeEl = document.getElementById('autonomy-mode');
        const iconEl = document.getElementById('autonomy-icon');
        const descEl = document.getElementById('autonomy-description');

        if (iconEl) iconEl.textContent = def.icon;
        if (modeEl) {
            modeEl.textContent = `${def.icon} ${def.mode}`;
            modeEl.style.color = c;
        }
        if (descEl) descEl.textContent = def.desc;

        // Dispatch event for other components to react
        try {
            window.dispatchEvent(new CustomEvent('diamants:autonomy-change', {
                detail: { level: def.level, mode: def.mode, step: def.step }
            }));
        } catch (_) { /* safe */ }

        console.log(`🎛️ Autonomie: ${def.icon} ${def.mode} (step ${def.step}, level ${def.level})`);
    }

    /** Backward-compatible — accepts 0-100 level and snaps to nearest step */
    setAutonomyLevel(level) {
        const steps = this.constructor.AUTONOMY_STEPS;
        let best = steps[steps.length - 1];
        for (const s of steps) {
            if (Math.abs(s.level - level) < Math.abs(best.level - level)) best = s;
        }
        this.setAutonomyStep(best.step);
    }

    toggleMinimap(visible) {
        const minimap = document.getElementById('minimap-container');
        if (minimap) minimap.style.display = visible ? 'block' : 'none';
    }

    toggleGrid(visible) {
        // Toggle grid visibility in scene
        console.log('Grid:', visible);
    }

    toggleTrails(visible) {
        // Toggle drone trails
        console.log('Trails:', visible);
    }

    toggleLogs() {
        const logs = document.getElementById('debug-logs');
        if (logs) logs.style.display = logs.style.display === 'none' ? 'block' : 'none';
    }

    toggleConsole() {
        // Toggle console panel
    }

    async runBenchmark(duration) {
        console.log(`🧪 Running benchmark (${duration}s)`);
        if (window.runQuickBenchmark) {
            const results = await window.runQuickBenchmark([1, 2, 4, 8], duration);
            window.benchmarkCharts?.generateAllCharts(results);
        }
    }

    // ═══════════════════════════════════════════════════════════
    // DISPOSE
    // ═══════════════════════════════════════════════════════════

    dispose() {
        // Cleanup
    }
}

// Global init function
export function initDiamantsUI(system) {
    window.diamantsUI = new DiamantsUIController(system);
    return window.diamantsUI;
}

// Auto-expose
if (typeof window !== 'undefined') {
    window.DiamantsUIController = DiamantsUIController;
    window.initDiamantsUI = initDiamantsUI;
}
