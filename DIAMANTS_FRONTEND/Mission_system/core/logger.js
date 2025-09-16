/**
 * DIAMANTS V3 - Système de Logging Hiérarchique
 * =============================================
 * Logger centralisé avec niveaux de log et filtrage
 */

// Mode silencieux global - utilisation de window pour éviter les conflits
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

export class DiamantLogger {
    static instance = null;

    static LOG_LEVELS = {
        FATAL: 0,
        ERROR: 1,
        WARNING: 2,
        INFO: 3,
        DEBUG: 4,
        TRACE: 5
    };

    static LEVEL_NAMES = ['FATAL', 'ERROR', 'WARNING', 'INFO', 'DEBUG', 'TRACE'];
    static LEVEL_COLORS = {
        FATAL: '#FF0000',
        ERROR: '#FF4444',
        WARNING: '#FFAA00',
        INFO: '#00AA88',
        DEBUG: '#0088FF',
        TRACE: '#888888'
    };

    static LEVEL_EMOJIS = {
        FATAL: '💀',
        ERROR: '❌',
        WARNING: '⚠️',
        INFO: '📊',
        DEBUG: '🔧',
        TRACE: '🔍'
    };

    constructor() {
        this.currentLevel = DiamantLogger.LOG_LEVELS.INFO;
        this.filters = new Set(); // Filtres par module/classe
        this.history = []; // Historique des logs
        this.maxHistory = 1000;
        this.startTime = Date.now();

        // Configuration UI
        this.setupUI();

        this.log('🎯 DiamantLogger initialisé - Niveau:', DiamantLogger.LEVEL_NAMES[this.currentLevel]);
    }

    // Fonctions d'aide pour le mode silencieux
    log(...args) {
        if (!window.SILENT_MODE) console.log(...args);
    }

    warn(...args) {
        if (!window.SILENT_MODE) console.warn(...args);
    }

    error(...args) {
        if (!window.SILENT_MODE) console.error(...args);
    }

    static getInstance() {
        if (!DiamantLogger.instance) {
            DiamantLogger.instance = new DiamantLogger();
        }
        return DiamantLogger.instance;
    }

    setupUI() {
        // Créer interface de contrôle des logs
        const logControl = document.createElement('div');
        logControl.id = 'diamant-log-control';
        logControl.style.cssText = `
            position: fixed;
            top: 10px;
            right: 10px;
            background: rgba(0,0,0,0.8);
            color: white;
            padding: 10px;
            border-radius: 5px;
            font-family: monospace;
            font-size: 12px;
            z-index: 10000;
            min-width: 200px;
            display: none; /* hidden by default */
        `;
        this.logControl = logControl;

        const title = document.createElement('div');
        title.textContent = '🎯 DIAMANTS Logger';
        title.style.fontWeight = 'bold';
        title.style.marginBottom = '5px';
        logControl.appendChild(title);

        // Sélecteur de niveau
        const levelSelect = document.createElement('select');
        levelSelect.style.cssText = 'width: 100%; margin: 5px 0;';
        DiamantLogger.LEVEL_NAMES.forEach((name, index) => {
            const option = document.createElement('option');
            option.value = index;
            option.textContent = `${DiamantLogger.LEVEL_EMOJIS[name]} ${name}`;
            if (index === this.currentLevel) option.selected = true;
            levelSelect.appendChild(option);
        });
        levelSelect.addEventListener('change', (e) => {
            this.setLevel(parseInt(e.target.value));
        });
        logControl.appendChild(levelSelect);

        // Boutons de contrôle
        const clearBtn = document.createElement('button');
        clearBtn.textContent = '🗑️ Clear';
        clearBtn.style.cssText = 'margin: 2px; padding: 2px 5px;';
        clearBtn.addEventListener('click', () => this.clear());
        logControl.appendChild(clearBtn);

        const exportBtn = document.createElement('button');
        exportBtn.textContent = '💾 Export';
        exportBtn.style.cssText = 'margin: 2px; padding: 2px 5px;';
        exportBtn.addEventListener('click', () => this.exportLogs());
        logControl.appendChild(exportBtn);

        // Compteurs
        this.counters = document.createElement('div');
        this.counters.style.cssText = 'margin-top: 5px; font-size: 10px;';
        logControl.appendChild(this.counters);

        document.body.appendChild(logControl);
        this.updateCounters();
    }

    // ——— Visibility controls ———
    showUI() {
        if (this.logControl) this.logControl.style.display = 'block';
    }
    hideUI() {
        if (this.logControl) this.logControl.style.display = 'none';
    }
    toggleUI() {
        if (!this.logControl) return;
        this.logControl.style.display = (this.logControl.style.display === 'none' || this.logControl.style.display === '')
            ? 'block'
            : 'none';
    }

    setLevel(level) {
        this.currentLevel = level;
        const levelName = DiamantLogger.LEVEL_NAMES[level];
        this.log(`🎯 Logger niveau changé: ${levelName}`);
        this.updateCounters();
    }

    shouldLog(level) {
        return level <= this.currentLevel;
    }

    log(level, module, message, ...args) {
        if (!this.shouldLog(level)) return;

        const timestamp = Date.now();
        const elapsed = timestamp - this.startTime;
        const levelName = DiamantLogger.LEVEL_NAMES[level];
        const emoji = DiamantLogger.LEVEL_EMOJIS[levelName];
        const color = DiamantLogger.LEVEL_COLORS[levelName];

        // Format du message
        const formattedTime = `+${(elapsed / 1000).toFixed(3)}s`;
        const prefix = `${emoji} [${levelName}] [${formattedTime}] [${module}]`;
        const fullMessage = `${prefix} ${message}`;

        // Log dans la console avec style
        const style = `color: ${color}; font-weight: ${level <= 1 ? 'bold' : 'normal'};`;
        this.log(`%c${fullMessage}`, style, ...args);

        // Stocker dans l'historique
        const logEntry = {
            timestamp,
            elapsed,
            level,
            levelName,
            module,
            message,
            args: args.length > 0 ? args : null
        };

        this.history.push(logEntry);
        if (this.history.length > this.maxHistory) {
            this.history.shift();
        }

        this.updateCounters();
    }

    updateCounters() {
        if (!this.counters) return;

        const counts = {};
        DiamantLogger.LEVEL_NAMES.forEach(name => counts[name] = 0);

        this.history.forEach(entry => {
            if (entry.levelName) counts[entry.levelName]++;
        });

        this.counters.innerHTML = DiamantLogger.LEVEL_NAMES.map(name =>
            `${DiamantLogger.LEVEL_EMOJIS[name]} ${counts[name]}`
        ).join(' | ');
    }

    clear() {
        this.history = [];
        console.clear();
        this.log('🎯 Logs effacés');
        this.updateCounters();
    }

    exportLogs() {
        const data = {
            timestamp: new Date().toISOString(),
            level: DiamantLogger.LEVEL_NAMES[this.currentLevel],
            entries: this.history
        };

        const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `diamants-logs-${Date.now()}.json`;
        a.click();
        URL.revokeObjectURL(url);

        this.info('Logger', 'Logs exportés');
    }

    // Méthodes de convenance
    fatal(module, message, ...args) { this.log(DiamantLogger.LOG_LEVELS.FATAL, module, message, ...args); }
    error(module, message, ...args) { this.log(DiamantLogger.LOG_LEVELS.ERROR, module, message, ...args); }
    warning(module, message, ...args) { this.log(DiamantLogger.LOG_LEVELS.WARNING, module, message, ...args); }
    info(module, message, ...args) { this.log(DiamantLogger.LOG_LEVELS.INFO, module, message, ...args); }
    debug(module, message, ...args) { this.log(DiamantLogger.LOG_LEVELS.DEBUG, module, message, ...args); }
    trace(module, message, ...args) { this.log(DiamantLogger.LOG_LEVELS.TRACE, module, message, ...args); }

    // Logs spécialisés
    logButtonClick(button, action) {
        this.info('UI', `🖱️ Bouton cliqué: ${button} → ${action}`);
    }

    logMeshLoading(droneId, status, details = '') {
        const level = status === 'success' ? 'info' : status === 'error' ? 'error' : 'debug';
        this[level]('Mesh', `🎭 Drone ${droneId}: ${status} ${details}`);
    }

    logFlightState(droneId, oldState, newState) {
        this.info('Flight', `🚁 Drone ${droneId}: ${oldState} → ${newState}`);
    }

    logMission(action, details) {
        this.info('Mission', `🎯 ${action}: ${details}`);
    }

    logCollision(droneId1, droneId2, distance) {
        this.warning('Safety', `⚠️ Collision évitée: ${droneId1} ↔ ${droneId2} (${distance.toFixed(2)}m)`);
    }

    logPerformance(module, metric, value, unit = '') {
        this.debug('Perf', `📊 ${module}.${metric}: ${value}${unit}`);
    }
}

// Export global pour faciliter l'usage
export const logger = DiamantLogger.getInstance();

// Raccourcis globaux
window.diamantLog = {
    fatal: (module, msg, ...args) => logger.fatal(module, msg, ...args),
    error: (module, msg, ...args) => logger.error(module, msg, ...args),
    warning: (module, msg, ...args) => logger.warning(module, msg, ...args),
    info: (module, msg, ...args) => logger.info(module, msg, ...args),
    debug: (module, msg, ...args) => logger.debug(module, msg, ...args),
    trace: (module, msg, ...args) => logger.trace(module, msg, ...args),
    setLevel: (level) => logger.setLevel(level),
    clear: () => logger.clear(),
    export: () => logger.exportLogs(),
    show: () => logger.showUI(),
    hide: () => logger.hideUI(),
    toggle: () => logger.toggleUI()
};
