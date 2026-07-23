/**
 * DIAMANTS - Mission Doctrine System
 * ===================================
 * Système de doctrines et courses of action (COA) pour contrôle de mission
 * 
 * Hiérarchie:
 * - DOCTRINE: Stratégie globale (comment l'essaim se comporte)
 * - COA (Course of Action): Tactique de déplacement/exploration
 * - MODE: Paramètres spécifiques du pattern
 * 
 * Bridge CoLab ↔ Autonome:
 * - exportDoctrine() → JSON compatible DoctrineLoader (système autonome)
 * - importDoctrine() → charge un JSON doctrine autonome dans le frontend
 */

// Bridge imports (lazy — loaded only when export/import is called)
let _exportModule = null;
let _importModule = null;

// Doctrines disponibles
export const DOCTRINES = {
    EXPLORATION: {
        id: 'exploration',
        name: 'Exploration',
        icon: '🌿',
        description: 'Exploration naturelle avec phéromones (évaporation, diffusion, dépôt)',
        params: {
            usePheromones: true,
            useGradients: true,
            attractionStrength: 0.7,
            repulsionStrength: 0.5,
            explorationBias: 0.6
        }
    },
    SWARM: {
        id: 'swarm',
        name: 'Essaim',
        icon: '🐝',
        description: 'Comportement collectif émergent',
        params: {
            usePheromones: false,
            useGradients: true,
            cohesion: 0.7,
            separation: 0.8,
            alignment: 0.5
        }
    },
    FORMATION: {
        id: 'formation', 
        name: 'Formation',
        icon: '🔢',
        description: 'Vol en formation rigide',
        params: {
            usePheromones: false,
            useGradients: false,
            maintainFormation: true,
            followLeader: true,
            formationType: 'grid'
        }
    },
    COVERAGE: {
        id: 'coverage',
        name: 'Couverture',
        icon: '📐',
        description: 'Couverture systématique de zone',
        params: {
            usePheromones: true,
            useGradients: false,
            prioritizeCoverage: true,
            avoidRedundancy: true
        }
    },
    SEARCH: {
        id: 'search',
        name: 'Recherche',
        icon: '🔍',
        description: 'Recherche et sauvetage',
        params: {
            usePheromones: true,
            useGradients: true,
            targetSeek: true,
            spiralExpand: true
        }
    }
};

// Courses of Action (patterns de déplacement)
export const COURSES_OF_ACTION = {
    ADAPTIVE: {
        id: 'adaptive',
        name: 'Adaptatif',
        icon: '🌟',
        description: 'S\'adapte selon contexte (stigmergie)',
        waypointGenerator: 'gradient',
        params: {
            useContext: true,
            dynamicSpacing: true,
            velocityAdaptive: true
        }
    },
    STIGMERGY: {
        id: 'stigmergy',
        name: 'Stigmergie',
        icon: '🧠',
        description: 'Navigation par phéromones — évite les zones déjà explorées',
        waypointGenerator: 'pheromone_gradient',
        params: {
            usePheromones: true,
            gradientWeight: 8.0,
            repulsionFromExplored: true,
            attractToFrontier: true
        }
    },
    GRID: {
        id: 'grid',
        name: 'Grille',
        icon: '⊞',
        description: 'Exploration en grille systématique',
        waypointGenerator: 'grid',
        params: {
            cellSize: 5,
            overlap: 0.2,
            rowByRow: true
        }
    },
    BOUSTROPHEDON: {
        id: 'boustrophedon',
        name: 'Boustrophedon',
        icon: '↔',
        description: 'Zigzag comme un champ labouré',
        waypointGenerator: 'boustrophedon',
        params: {
            lineSpacing: 4,
            turnRadius: 2,
            alternateLanes: true
        }
    },
    SPIRAL: {
        id: 'spiral',
        name: 'Spirale',
        icon: '🌀',
        description: 'Expansion en spirale depuis le centre',
        waypointGenerator: 'spiral',
        params: {
            startRadius: 3,
            expansionRate: 1.5,
            clockwise: true
        }
    },
    RADIAL: {
        id: 'radial',
        name: 'Radial',
        icon: '☀',
        description: 'Exploration radiale depuis un point',
        waypointGenerator: 'radial',
        params: {
            sectors: 8,
            maxRadius: 40,
            returnToCenter: true
        }
    },
    PERIMETER: {
        id: 'perimeter',
        name: 'Périmètre',
        icon: '⬜',
        description: 'Patrouille du périmètre de zone',
        waypointGenerator: 'perimeter',
        params: {
            marginDistance: 3,
            patrolSpeed: 'medium',
            bidirectional: true
        }
    }
};

// Formation types pour doctrine FORMATION
export const FORMATION_TYPES = {
    LINE: { id: 'line', name: 'Ligne', spacing: 3 },
    COLUMN: { id: 'column', name: 'Colonne', spacing: 3 },
    GRID: { id: 'grid', name: 'Grille', spacing: 4 },
    CIRCLE: { id: 'circle', name: 'Cercle', radius: 8 },
    TRIANGLE: { id: 'triangle', name: 'Triangle', spacing: 5 },
    VEE: { id: 'vee', name: 'V', angle: 45, spacing: 4 },
    DIAMOND: { id: 'diamond', name: 'Losange', spacing: 5 }
};

/**
 * Gestionnaire de Doctrine
 * Contrôle la stratégie et la tactique de l'essaim
 */
export class DoctrineManager {
    constructor() {
        this.currentDoctrine = DOCTRINES.EXPLORATION;
        this.currentCOA = COURSES_OF_ACTION.ADAPTIVE;
        this.currentFormation = FORMATION_TYPES.GRID;
        
        this.listeners = new Set();
        this.missionState = {
            isActive: false,
            startTime: null,
            coverage: 0,
            dronesActive: 0
        };
        
        // Paramètres de zone
        this.zoneParams = {
            centerX: 0,
            centerZ: 0,
            sizeX: 120,
            sizeZ: 120,
            altitude: 3.0,
            safetyDistance: 3.0
        };
        
        this.init();
    }
    
    init() {
        // Exposer globalement (browser only)
        if (typeof window !== 'undefined') {
            window.DIAMANTS_DOCTRINE = this;
        }
        
        // Écouter les changements d'UI
        if (typeof document !== 'undefined') {
            this.bindUIElements();
        }
        
        console.log('📋 DoctrineManager initialisé');
        console.log(`   Doctrine: ${this.currentDoctrine.name}`);
        console.log(`   COA: ${this.currentCOA.name}`);
    }
    
    /**
     * Connecter les éléments UI
     */
    bindUIElements() {
        // Select doctrine (mission_type)
        const missionTypeSelect = document.getElementById('mission_type');
        if (missionTypeSelect) {
            missionTypeSelect.addEventListener('change', (e) => {
                this.setDoctrine(e.target.value);
            });
        }
        
        // Select COA (mode_select)
        const modeSelect = document.getElementById('mode_select');
        if (modeSelect) {
            modeSelect.addEventListener('change', (e) => {
                this.setCOA(e.target.value);
            });
        }
        
        // Altitude slider
        const altitudeSlider = document.getElementById('altitude_slider');
        if (altitudeSlider) {
            altitudeSlider.addEventListener('input', (e) => {
                this.zoneParams.altitude = parseFloat(e.target.value);
                this.updateDisplay('altitude_display', `${this.zoneParams.altitude.toFixed(1)}m`);
                this.notifyListeners('altitude', this.zoneParams.altitude);
            });
        }
        
        // Safety distance
        const safetySlider = document.getElementById('safety_distance');
        if (safetySlider) {
            safetySlider.addEventListener('input', (e) => {
                this.zoneParams.safetyDistance = parseFloat(e.target.value);
                this.updateDisplay('safety_distance_value', `${this.zoneParams.safetyDistance.toFixed(1)}m`);
                this.notifyListeners('safety', this.zoneParams.safetyDistance);
            });
        }
        
        // Zone bounds
        const boundsX = document.getElementById('bounds_x');
        const boundsY = document.getElementById('bounds_y');
        if (boundsX) {
            boundsX.addEventListener('change', (e) => {
                this.zoneParams.sizeX = parseFloat(e.target.value);
                this.notifyListeners('bounds', this.zoneParams);
            });
        }
        if (boundsY) {
            boundsY.addEventListener('change', (e) => {
                this.zoneParams.sizeZ = parseFloat(e.target.value);
                this.notifyListeners('bounds', this.zoneParams);
            });
        }
    }
    
    /**
     * Définir la doctrine active
     */
    setDoctrine(doctrineId) {
        const doctrine = Object.values(DOCTRINES).find(d => d.id === doctrineId);
        if (!doctrine) {
            console.warn(`⚠️ Doctrine inconnue: ${doctrineId}`);
            return false;
        }
        
        this.currentDoctrine = doctrine;
        console.log(`📋 Doctrine changée: ${doctrine.icon} ${doctrine.name}`);
        
        // Appliquer les paramètres
        this.applyDoctrineParams();
        this.notifyListeners('doctrine', doctrine);

        // Forcer le rafraîchissement visuel de l'UI
        this._updateDoctrineUI(doctrine);
        
        return true;
    }
    
    /**
     * Définir le COA actif
     */
    setCOA(coaId) {
        const coa = Object.values(COURSES_OF_ACTION).find(c => c.id === coaId);
        if (!coa) {
            console.warn(`⚠️ COA inconnu: ${coaId}`);
            return false;
        }
        
        this.currentCOA = coa;
        console.log(`🎯 COA changé: ${coa.icon} ${coa.name}`);
        
        // Appliquer les paramètres
        this.applyCOAParams();
        this.notifyListeners('coa', coa);

        // Forcer le rafraîchissement visuel de l'UI
        this._updateCOAUI(coa);
        
        return true;
    }
    
    /**
     * Définir la formation
     */
    setFormation(formationId) {
        const formation = Object.values(FORMATION_TYPES).find(f => f.id === formationId);
        if (!formation) {
            console.warn(`⚠️ Formation inconnue: ${formationId}`);
            return false;
        }
        
        this.currentFormation = formation;
        console.log(`🔢 Formation changée: ${formation.name}`);
        this.notifyListeners('formation', formation);
        
        return true;
    }
    
    /**
     * Appliquer les paramètres de doctrine au système
     */
    applyDoctrineParams() {
        const params = this.currentDoctrine.params;
        const controller = window.diamantsSystem?.integratedController;
        
        if (!controller) return;
        
        // Activer/désactiver stigmergie (use INSTANCE not CLASS)
        const stigInstance = window.DIAMANTS_STIGMERGY_INSTANCE;
        // Pheromones needed if EITHER doctrine OR current COA requires them
        const needsPheromones = params.usePheromones
                             || this.currentCOA?.params?.usePheromones === true
                             || this.currentCOA?.id === 'stigmergy';
        if (stigInstance) {
            if (typeof stigInstance.setActive === 'function') {
                stigInstance.setActive(needsPheromones);
            } else {
                stigInstance.enabled = needsPheromones;
            }
            // Toggle lightweightMode: OFF when pheromones needed, ON otherwise
            if (typeof stigInstance.setLightweightMode === 'function') {
                stigInstance.setLightweightMode(!needsPheromones);
            }
            console.log(`🧠 Phéromones: ${needsPheromones ? 'ON (full)' : 'OFF (lightweight)'} (doctrine=${this.currentDoctrine?.name}, COA=${this.currentCOA?.name})`);
        }
        
        // Configurer le contrôleur
        if (controller.config) {
            controller.config.enableStigmergy = params.usePheromones;
            controller.config.enableGradients = params.useGradients;
        }
        
        // Flush engine waypoints to force re-generation with new doctrine
        const engine = controller.autonomousFlightEngine;
        if (engine && typeof engine.flushDoctrineWaypoints === 'function') {
            engine.flushDoctrineWaypoints();
        }
        
        // Mettre à jour les métriques UI
        const stigmergyEl = document.getElementById('stigmergy_value');
        if (stigmergyEl) {
            stigmergyEl.textContent = params.usePheromones ? 'ON' : 'OFF';
            stigmergyEl.style.color = params.usePheromones ? '#00FF88' : '#666';
        }
    }
    
    /**
     * Appliquer les paramètres de COA
     */
    applyCOAParams() {
        const coa = this.currentCOA;
        const controller = window.diamantsSystem?.integratedController;
        
        if (!controller) return;
        
        // Store pattern on the engine for fallback waypoint generation
        if (controller.autonomousFlightEngine) {
            controller.autonomousFlightEngine.explorationPattern = coa.waypointGenerator;
        }
        
        // Toggle lightweightMode based on COA OR doctorate pheromone requirement
        // If EITHER the COA or the current doctrine needs pheromones → keep them active
        const stigInstance = window.DIAMANTS_STIGMERGY_INSTANCE;
        const needsPheromones = coa.params?.usePheromones === true
                             || coa.id === 'stigmergy'
                             || this.currentDoctrine?.params?.usePheromones === true;
        if (stigInstance && typeof stigInstance.setLightweightMode === 'function') {
            stigInstance.setLightweightMode(!needsPheromones);
            console.log(`🎯 COA ${coa.name}: pheromones ${needsPheromones ? 'ACTIVE' : 'lightweight'} (doctrine=${this.currentDoctrine?.name})`);
        }
        
        // Flush waypoints so drones pick up the new COA immediately
        const engine = controller.autonomousFlightEngine;
        if (engine && typeof engine.flushDoctrineWaypoints === 'function') {
            engine.flushDoctrineWaypoints();
        }
        
        // Mettre à jour le générateur de waypoints
        if (controller.flightBehaviors && typeof controller.flightBehaviors.setExplorationPattern === 'function') {
            controller.flightBehaviors.setExplorationPattern(coa.waypointGenerator);
        }
        
        // Mettre à jour l'affichage pattern
        const patternEl = document.getElementById('currentPattern');
        if (patternEl) {
            patternEl.textContent = coa.name;
        }
    }
    
    /**
     * Démarrer une mission avec la doctrine actuelle
     */
    startMission() {
        if (this.missionState.isActive) {
            console.log('⚠️ Mission déjà active');
            return;
        }
        
        this.missionState.isActive = true;
        this.missionState.startTime = Date.now();
        this.missionState.coverage = 0;
        
        console.log(`🚀 Mission démarrée avec:`);
        console.log(`   📋 Doctrine: ${this.currentDoctrine.name}`);
        console.log(`   🎯 COA: ${this.currentCOA.name}`);
        console.log(`   📐 Zone: ${this.zoneParams.sizeX}x${this.zoneParams.sizeZ}m`);
        console.log(`   🛫 Altitude: ${this.zoneParams.altitude}m`);
        
        // Appliquer les paramètres
        this.applyDoctrineParams();
        this.applyCOAParams();
        
        // Notifier les systèmes
        this.notifyListeners('missionStart', {
            doctrine: this.currentDoctrine,
            coa: this.currentCOA,
            zone: this.zoneParams
        });
        
        // Démarrer le timer minimap
        if (window.DIAMANTS_MINIMAP) {
            window.DIAMANTS_MINIMAP.startExploration();
        }
    }
    
    /**
     * Arrêter la mission
     */
    stopMission() {
        if (!this.missionState.isActive) return;
        
        const elapsed = Date.now() - this.missionState.startTime;
        const minutes = Math.floor(elapsed / 60000);
        const seconds = Math.floor((elapsed % 60000) / 1000);
        
        console.log(`⏹ Mission arrêtée après ${minutes}:${seconds.toString().padStart(2, '0')}`);
        console.log(`   📊 Couverture finale: ${this.missionState.coverage.toFixed(1)}%`);
        
        this.missionState.isActive = false;
        
        this.notifyListeners('missionStop', {
            elapsed,
            coverage: this.missionState.coverage
        });
        
        // Arrêter le timer minimap
        if (window.DIAMANTS_MINIMAP) {
            window.DIAMANTS_MINIMAP.stopExploration();
        }
    }
    
    /**
     * Mettre à jour la couverture
     */
    updateCoverage(percentage) {
        this.missionState.coverage = percentage;
    }
    
    /**
     * Ajouter un listener
     */
    addListener(callback) {
        this.listeners.add(callback);
    }
    
    /**
     * Notifier les listeners
     */
    notifyListeners(event, data) {
        this.listeners.forEach(cb => {
            try {
                cb(event, data);
            } catch (e) {
                console.error('Listener error:', e);
            }
        });
    }
    
    /**
     * Helper pour mettre à jour l'affichage
     */
    updateDisplay(elementId, value) {
        const el = document.getElementById(elementId);
        if (el) el.textContent = value;
    }

    /**
     * Update UI to reflect current doctrine (feedback badge)
     */
    _updateDoctrineUI(doctrine) {
        const el = document.getElementById('mission_type');
        if (el && el.value !== doctrine.id) el.value = doctrine.id;
        // Show active indicator
        const badge = document.getElementById('doctrine_active_badge');
        if (badge) badge.textContent = `${doctrine.icon} ${doctrine.name}`;
    }

    /**
     * Update UI to reflect current COA (feedback badge)
     */
    _updateCOAUI(coa) {
        const el = document.getElementById('mode_select');
        if (el && el.value !== coa.id) el.value = coa.id;
        const patternEl = document.getElementById('currentPattern');
        if (patternEl) patternEl.textContent = `${coa.icon} ${coa.name}`;
    }
    
    /**
     * Obtenir l'état actuel
     */
    getState() {
        return {
            doctrine: this.currentDoctrine,
            coa: this.currentCOA,
            formation: this.currentFormation,
            zone: this.zoneParams,
            mission: this.missionState
        };
    }

    // ─────────────────────────────────────────────
    //  Bridge CoLab ↔ Autonome (Gap #1)
    // ─────────────────────────────────────────────

    /**
     * Exporte la doctrine actuelle en JSON compatible DoctrineLoader (système autonome).
     * @param {Object} [options] - { name, mode: 'SIM'|'REAL', platform, connection, ai, safety }
     * @returns {Object} JSON doctrine autonome
     */
    async exportDoctrineJSON(options = {}) {
        if (!_exportModule) {
            _exportModule = await import('./doctrine-exporter.js');
        }
        return _exportModule.exportDoctrine(this, options);
    }

    /**
     * Exporte et télécharge un fichier JSON doctrine.
     * @param {string} [filename] - Nom du fichier (auto-généré si absent)
     * @param {Object} [options] - Options d'export
     */
    async downloadDoctrineJSON(filename, options = {}) {
        if (!_exportModule) {
            _exportModule = await import('./doctrine-exporter.js');
        }
        return _exportModule.downloadDoctrineJSON(this, filename, options);
    }

    /**
     * Importe un JSON doctrine autonome dans le frontend.
     * @param {Object} json - Doctrine JSON autonome
     * @returns {{ success, applied, warnings }}
     */
    async importDoctrineJSON(json) {
        if (!_importModule) {
            _importModule = await import('./doctrine-importer.js');
        }
        return _importModule.importDoctrineFromJSON(this, json);
    }

    /**
     * Ouvre un file picker pour charger un fichier doctrine JSON.
     * @returns {Promise<{ success, applied, warnings }>}
     */
    async importDoctrineFromFile() {
        if (!_importModule) {
            _importModule = await import('./doctrine-importer.js');
        }
        return _importModule.importDoctrineFromFile(this);
    }

    /**
     * Importe depuis une URL (ex: API backend, GitHub raw, etc.).
     * @param {string} url
     * @returns {Promise<{ success, applied, warnings }>}
     */
    async importDoctrineFromURL(url) {
        if (!_importModule) {
            _importModule = await import('./doctrine-importer.js');
        }
        return _importModule.importDoctrineFromURL(this, url);
    }
    
    /**
     * Générer les waypoints selon le COA actuel
     */
    generateWaypoints(droneId, currentPosition) {
        const coa = this.currentCOA;
        const zone = this.zoneParams;
        const waypoints = [];
        
        switch (coa.waypointGenerator) {
            case 'grid':
                return this._generateGridWaypoints(droneId, currentPosition);
            case 'boustrophedon':
                return this._generateBoustrophedonWaypoints(droneId, currentPosition);
            case 'spiral':
                return this._generateSpiralWaypoints(droneId, currentPosition);
            case 'radial':
                return this._generateRadialWaypoints(droneId, currentPosition);
            case 'perimeter':
                return this._generatePerimeterWaypoints(droneId, currentPosition);
            case 'gradient':
            default:
                // Gradient/Adaptive - utilise stigmergie
                return this._generateAdaptiveWaypoints(droneId, currentPosition);
        }
    }
    
    _generateGridWaypoints(droneId, pos) {
        const { sizeX, sizeZ, altitude } = this.zoneParams;
        const cellSize = this.currentCOA.params.cellSize || 5;
        const waypoints = [];
        
        const droneIndex = parseInt(droneId.split('_')[1]) - 1 || 0;
        const totalDrones = 8;
        const totalRows = Math.ceil(sizeZ / cellSize);
        // Distribute rows among drones
        const rowsPerDrone = Math.max(1, Math.ceil(totalRows / totalDrones));
        const startRow = droneIndex * rowsPerDrone;
        const endRow = Math.min(startRow + rowsPerDrone, totalRows);
        
        for (let r = startRow; r < endRow; r++) {
            const z = -sizeZ/2 + r * cellSize;
            // Alternate direction for coverage
            if (r % 2 === 0) {
                for (let x = -sizeX/2; x <= sizeX/2; x += cellSize) {
                    waypoints.push({ x, y: altitude, z });
                }
            } else {
                for (let x = sizeX/2; x >= -sizeX/2; x -= cellSize) {
                    waypoints.push({ x, y: altitude, z });
                }
            }
        }
        
        return waypoints;
    }
    
    _generateBoustrophedonWaypoints(droneId, pos) {
        const { sizeX, sizeZ, altitude } = this.zoneParams;
        const spacing = this.currentCOA.params.lineSpacing || 4;
        const waypoints = [];
        
        const droneIndex = parseInt(droneId.split('_')[1]) - 1 || 0;
        const totalDrones = 8;
        const totalLines = Math.ceil(sizeZ / spacing);
        const linesPerDrone = Math.max(1, Math.ceil(totalLines / totalDrones));
        const startLine = droneIndex * linesPerDrone;
        const endLine = Math.min(startLine + linesPerDrone, totalLines);
        
        let direction = (startLine % 2 === 0) ? 1 : -1;
        
        for (let line = startLine; line < endLine; line++) {
            const z = -sizeZ/2 + line * spacing;
            // Start and end of the zig-zag
            const xStart = direction > 0 ? -sizeX/2 : sizeX/2;
            const xEnd = direction > 0 ? sizeX/2 : -sizeX/2;
            const xStep = direction > 0 ? spacing : -spacing;
            
            for (let x = xStart; direction > 0 ? x <= xEnd : x >= xEnd; x += xStep) {
                waypoints.push({ x, y: altitude, z });
            }
            direction *= -1;
        }
        
        return waypoints;
    }
    
    _generateSpiralWaypoints(droneId, pos) {
        const { sizeX, sizeZ, altitude, centerX, centerZ } = this.zoneParams;
        const { startRadius, expansionRate } = this.currentCOA.params;
        const waypoints = [];
        
        const droneIndex = parseInt(droneId.split('_')[1]) - 1 || 0;
        const totalDrones = 8;
        // Each drone starts at a different angular offset
        const angleOffset = (droneIndex / totalDrones) * 2 * Math.PI;
        
        const maxRadius = Math.min(sizeX, sizeZ) / 2;
        let radius = startRadius;
        let angle = angleOffset;
        const angleStep = 0.4; // Tighter spiral
        const radiusStep = expansionRate * 0.15;
        
        while (radius < maxRadius) {
            const x = centerX + Math.cos(angle) * radius;
            const z = centerZ + Math.sin(angle) * radius;
            waypoints.push({ x, y: altitude, z });
            
            angle += angleStep;
            radius += radiusStep;
        }
        
        return waypoints;
    }
    
    _generateRadialWaypoints(droneId, pos) {
        const { sizeX, sizeZ, altitude, centerX, centerZ } = this.zoneParams;
        const { sectors, maxRadius } = this.currentCOA.params;
        const waypoints = [];
        
        const droneIndex = parseInt(droneId.split('_')[1]) - 1 || 0;
        // Each drone covers its own angular sector
        const sectorAngle = (droneIndex / sectors) * 2 * Math.PI;
        const sectorWidth = (2 * Math.PI) / sectors;
        
        // Go outward
        for (let r = 5; r <= maxRadius; r += 5) {
            const x = centerX + Math.cos(sectorAngle) * r;
            const z = centerZ + Math.sin(sectorAngle) * r;
            waypoints.push({ x, y: altitude, z });
        }
        // Sweep the arc at max radius
        const arcSteps = 6;
        for (let i = 0; i <= arcSteps; i++) {
            const a = sectorAngle + (i / arcSteps) * sectorWidth;
            const x = centerX + Math.cos(a) * maxRadius;
            const z = centerZ + Math.sin(a) * maxRadius;
            waypoints.push({ x, y: altitude, z });
        }
        // Come back inward on adjacent angle
        const returnAngle = sectorAngle + sectorWidth * 0.5;
        for (let r = maxRadius; r >= 5; r -= 5) {
            const x = centerX + Math.cos(returnAngle) * r;
            const z = centerZ + Math.sin(returnAngle) * r;
            waypoints.push({ x, y: altitude, z });
        }
        
        return waypoints;
    }
    
    _generatePerimeterWaypoints(droneId, pos) {
        const { sizeX, sizeZ, altitude, centerX, centerZ } = this.zoneParams;
        const margin = this.currentCOA.params.marginDistance || 3;
        const waypoints = [];
        
        const droneIndex = parseInt(droneId.split('_')[1]) - 1 || 0;
        const halfX = sizeX / 2 - margin;
        const halfZ = sizeZ / 2 - margin;
        
        // Full perimeter as a series of points along edges
        const edgeStep = 5; // Point every 5m
        const perimeterPoints = [];
        
        // Top edge (left to right)
        for (let x = -halfX; x <= halfX; x += edgeStep) {
            perimeterPoints.push({ x: centerX + x, z: centerZ - halfZ });
        }
        // Right edge (top to bottom)
        for (let z = -halfZ; z <= halfZ; z += edgeStep) {
            perimeterPoints.push({ x: centerX + halfX, z: centerZ + z });
        }
        // Bottom edge (right to left)
        for (let x = halfX; x >= -halfX; x -= edgeStep) {
            perimeterPoints.push({ x: centerX + x, z: centerZ + halfZ });
        }
        // Left edge (bottom to top)
        for (let z = halfZ; z >= -halfZ; z -= edgeStep) {
            perimeterPoints.push({ x: centerX - halfX, z: centerZ + z });
        }
        
        // Each drone starts at a different point on the perimeter
        const offset = Math.floor((droneIndex / 8) * perimeterPoints.length);
        // Alternate direction for half the drones
        if (droneIndex % 2 === 0) {
            for (let i = 0; i < perimeterPoints.length; i++) {
                const p = perimeterPoints[(i + offset) % perimeterPoints.length];
                waypoints.push({ x: p.x, y: altitude, z: p.z });
            }
        } else {
            for (let i = perimeterPoints.length - 1; i >= 0; i--) {
                const p = perimeterPoints[(i + offset) % perimeterPoints.length];
                waypoints.push({ x: p.x, y: altitude, z: p.z });
            }
        }
        
        return waypoints;
    }
    
    _generateAdaptiveWaypoints(droneId, pos) {
        // Utiliser la stigmergie (INSTANCE, pas la classe)
        const engine = window.DIAMANTS_STIGMERGY_INSTANCE;
        if (engine && engine.enabled) {
            const wp = engine.computeNextWaypoint ? 
                engine.computeNextWaypoint(droneId, { position: pos }) : null;
            if (wp) return [wp];
        }
        
        // Exploration semi-aléatoire guidée par couverture
        const { sizeX, sizeZ, altitude } = this.zoneParams;
        const waypoints = [];
        const droneIndex = parseInt(droneId.split('_')[1]) - 1 || 0;
        
        // Each drone gets a batch of 5 semi-random waypoints in its quadrant
        const quadrantX = (droneIndex % 4 < 2) ? -1 : 1;
        const quadrantZ = (droneIndex % 2 === 0) ? -1 : 1;
        
        for (let i = 0; i < 5; i++) {
            const spreadX = 0.15 + Math.random() * 0.35; // 15-50% of half-zone
            const spreadZ = 0.15 + Math.random() * 0.35;
            waypoints.push({
                x: quadrantX * spreadX * sizeX + (Math.random() - 0.5) * sizeX * 0.3,
                y: altitude + (Math.random() - 0.5) * 1.5,
                z: quadrantZ * spreadZ * sizeZ + (Math.random() - 0.5) * sizeZ * 0.3
            });
        }
        
        return waypoints;
    }
}

// Auto-initialisation
let doctrineManager = null;

export function initDoctrineManager() {
    if (!doctrineManager) {
        doctrineManager = new DoctrineManager();
        // Exposer à window pour les tests (browser only)
        if (typeof window !== 'undefined') {
            window.doctrineManager = doctrineManager;
        }
    }
    return doctrineManager;
}

export function getDoctrineManager() {
    return doctrineManager || initDoctrineManager();
}

// Init au chargement du DOM (browser only)
if (typeof document !== 'undefined') {
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initDoctrineManager);
    } else {
        initDoctrineManager();
    }
}

export default DoctrineManager;
