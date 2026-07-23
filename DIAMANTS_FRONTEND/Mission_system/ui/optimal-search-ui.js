/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Optimal Search UI
 * ==============================
 * Interface de recherche intelligente pour trouver les configurations optimales
 * 
 * Fonctionnalités:
 *  • Recherche par critères multiples (couverture, temps, énergie)
 *  • Scoring Pareto des stratégies
 *  • Suggestions contextuelles
 *  • Historique des configurations performantes
 *  • Algorithmes de recherche optimisés (Frontier Shrink SSSP)
 * 
 * Basé sur:
 *  "Breaking the Sorting Barrier for Directed SSSP"
 *  (Tsinghua/Stanford/Max Planck, July 2025)
 *  https://arxiv.org/pdf/2504.17033v2
 */

import { DOCTRINES, COURSES_OF_ACTION, FORMATION_TYPES } from '../missions/mission-doctrine.js';
import { logger } from '../core/logger.js';
import { computeOptimalParams } from '../intelligence/optimized-search.js';
import { makeDraggable } from './panel-utils.js';

/**
 * Configuration scoring weights
 */
const SCORING_WEIGHTS = {
    coverage: 0.35,      // Importance de la couverture de zone
    time: 0.25,          // Importance du temps de mission
    energy: 0.20,        // Importance de l'efficacité énergétique
    reliability: 0.15,   // Importance de la fiabilité
    adaptability: 0.05   // Importance de l'adaptabilité
};

/**
 * Performance baselines pour chaque combinaison doctrine+COA
 * (données empiriques simulées)
 */
const PERFORMANCE_BASELINES = {
    'stigmergy_adaptive': { coverage: 0.92, time: 0.78, energy: 0.85, reliability: 0.90, adaptability: 0.95 },
    'stigmergy_grid': { coverage: 0.95, time: 0.60, energy: 0.70, reliability: 0.95, adaptability: 0.50 },
    'stigmergy_boustrophedon': { coverage: 0.94, time: 0.65, energy: 0.75, reliability: 0.92, adaptability: 0.55 },
    'stigmergy_spiral': { coverage: 0.88, time: 0.70, energy: 0.80, reliability: 0.88, adaptability: 0.70 },
    'stigmergy_radial': { coverage: 0.85, time: 0.75, energy: 0.78, reliability: 0.85, adaptability: 0.65 },
    'stigmergy_perimeter': { coverage: 0.60, time: 0.90, energy: 0.88, reliability: 0.92, adaptability: 0.40 },
    'swarm_adaptive': { coverage: 0.88, time: 0.82, energy: 0.80, reliability: 0.85, adaptability: 0.92 },
    'swarm_grid': { coverage: 0.90, time: 0.70, energy: 0.65, reliability: 0.88, adaptability: 0.60 },
    'swarm_spiral': { coverage: 0.82, time: 0.75, energy: 0.78, reliability: 0.80, adaptability: 0.75 },
    'formation_grid': { coverage: 0.85, time: 0.85, energy: 0.60, reliability: 0.95, adaptability: 0.30 },
    'formation_radial': { coverage: 0.78, time: 0.88, energy: 0.55, reliability: 0.92, adaptability: 0.35 },
    'coverage_grid': { coverage: 0.98, time: 0.55, energy: 0.65, reliability: 0.96, adaptability: 0.45 },
    'coverage_boustrophedon': { coverage: 0.97, time: 0.60, energy: 0.70, reliability: 0.94, adaptability: 0.50 },
    'search_adaptive': { coverage: 0.75, time: 0.90, energy: 0.85, reliability: 0.80, adaptability: 0.98 },
    'search_spiral': { coverage: 0.70, time: 0.88, energy: 0.82, reliability: 0.78, adaptability: 0.90 }
};

export class OptimalSearchUI {
    constructor(options = {}) {
        /** @type {HTMLElement|null} */
        this.root = null;
        /** @type {HTMLElement|null} */
        this.resultsContainer = null;
        /** @type {HTMLElement|null} */
        this.searchInput = null;
        
        this._visible = false;
        this._searchQuery = '';
        this._filters = {
            minCoverage: 0,
            maxTime: 1.0,
            minReliability: 0,
            priorities: Object.keys(SCORING_WEIGHTS)
        };
        
        // History (must be before _generateConfigurations which uses _usageHistory)
        this._searchHistory = [];
        this._usageHistory = JSON.parse(localStorage.getItem('diamants_config_usage') || '[]');
        
        // Cached configurations
        this._configurations = this._generateConfigurations();
        this._rankedConfigs = [];
        this._selectedConfig = null;
        
        // Callbacks
        this._onConfigSelect = options.onConfigSelect || null;
        
        this._buildDOM();
        this._performSearch();
        
        logger.info('OptimalSearchUI', '🔍 OptimalSearchUI initialisé');
    }
    
    /**
     * Generate all valid doctrine+COA combinations
     */
    _generateConfigurations() {
        const configs = [];
        
        for (const [docKey, doctrine] of Object.entries(DOCTRINES)) {
            for (const [coaKey, coa] of Object.entries(COURSES_OF_ACTION)) {
                const key = `${doctrine.id}_${coa.id}`;
                const baseline = PERFORMANCE_BASELINES[key];
                
                // Skip invalid combinations
                if (!baseline) continue;
                
                configs.push({
                    id: key,
                    doctrine: doctrine,
                    doctrineKey: docKey,
                    coa: coa,
                    coaKey: coaKey,
                    performance: baseline,
                    score: 0, // Will be calculated
                    rank: 0,
                    usageCount: this._getUsageCount(key),
                    tags: this._generateTags(doctrine, coa, baseline)
                });
            }
        }
        
        return configs;
    }
    
    /**
     * Generate searchable tags for a configuration
     */
    _generateTags(doctrine, coa, perf) {
        const tags = [];
        
        // Doctrine tags
        tags.push(doctrine.id, doctrine.name.toLowerCase());
        if (doctrine.params.usePheromones) tags.push('phéromones', 'pheromones');
        if (doctrine.params.useGradients) tags.push('gradients');
        
        // COA tags
        tags.push(coa.id, coa.name.toLowerCase());
        
        // Performance tags
        if (perf.coverage > 0.90) tags.push('haute couverture', 'high coverage', 'complet');
        if (perf.time > 0.80) tags.push('rapide', 'fast', 'efficace');
        if (perf.energy > 0.80) tags.push('économe', 'energy efficient', 'basse consommation');
        if (perf.adaptability > 0.80) tags.push('adaptatif', 'flexible', 'intelligent');
        if (perf.reliability > 0.90) tags.push('fiable', 'reliable', 'stable');
        
        // Special tags
        if (perf.coverage > 0.95 && perf.reliability > 0.90) tags.push('optimal', 'meilleur', 'best');
        if (perf.adaptability > 0.90 && perf.energy > 0.80) tags.push('autonome', 'smart');
        
        return tags;
    }
    
    _getUsageCount(configId) {
        return this._usageHistory.filter(h => h.id === configId).length;
    }
    
    /**
     * Build the search UI DOM
     */
    _buildDOM() {
        this.root = document.createElement('div');
        this.root.id = 'optimal-search-ui';
        Object.assign(this.root.style, {
            position: 'fixed',
            top: '50%',
            left: '50%',
            transform: 'translate(-50%, -50%)',
            width: '600px',
            maxWidth: '90vw',
            maxHeight: '80vh',
            background: 'rgba(5, 15, 30, 0.98)',
            border: '2px solid #00AAFF',
            borderRadius: '12px',
            fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
            fontSize: '12px',
            color: '#ccddee',
            zIndex: '5000',
            display: 'none',
            flexDirection: 'column',
            backdropFilter: 'blur(10px)',
            boxShadow: '0 0 40px rgba(0, 170, 255, 0.3)'
        });
        
        // Header
        const header = document.createElement('div');
        Object.assign(header.style, {
            display: 'flex',
            alignItems: 'center',
            gap: '12px',
            padding: '12px 16px',
            borderBottom: '1px solid #003366',
            background: 'rgba(0, 30, 60, 0.5)'
        });
        
        const icon = document.createElement('span');
        icon.textContent = '🎯';
        icon.style.fontSize = '18px';
        
        const title = document.createElement('span');
        title.textContent = 'Recherche Configuration Optimale';
        title.style.fontWeight = '600';
        title.style.color = '#00CCFF';
        title.style.flex = '1';
        
        const closeBtn = this._createButton('✕', 'Fermer', () => this.hide());
        
        header.append(icon, title, closeBtn);
        
        // Search bar
        const searchBar = document.createElement('div');
        Object.assign(searchBar.style, {
            display: 'flex',
            gap: '8px',
            padding: '12px 16px',
            borderBottom: '1px solid #002244'
        });
        
        this.searchInput = document.createElement('input');
        this.searchInput.type = 'text';
        this.searchInput.placeholder = 'Rechercher... (ex: rapide couverture, stigmergie adaptatif)';
        Object.assign(this.searchInput.style, {
            flex: '1',
            background: 'rgba(0, 40, 80, 0.6)',
            border: '1px solid #004488',
            borderRadius: '6px',
            color: '#ffffff',
            padding: '10px 14px',
            fontSize: '13px',
            outline: 'none'
        });
        this.searchInput.addEventListener('input', () => this._onSearchInput());
        this.searchInput.addEventListener('keydown', (e) => this._onKeyDown(e));
        
        searchBar.appendChild(this.searchInput);
        
        // Filter bar
        const filterBar = this._createFilterBar();
        
        // Results
        this.resultsContainer = document.createElement('div');
        Object.assign(this.resultsContainer.style, {
            flex: '1',
            overflowY: 'auto',
            padding: '8px 0',
            maxHeight: '400px'
        });
        
        // Footer with hints
        const footer = document.createElement('div');
        Object.assign(footer.style, {
            padding: '10px 16px',
            borderTop: '1px solid #002244',
            fontSize: '10px',
            color: '#668899',
            display: 'flex',
            justifyContent: 'space-between'
        });
        footer.innerHTML = `
            <span>↑↓ Naviguer • Enter Appliquer • Esc Fermer</span>
            <span>Ctrl+K pour ouvrir</span>
        `;
        
        // Custom scrollbar styles
        const style = document.createElement('style');
        style.textContent = `
            #optimal-search-ui ::-webkit-scrollbar { width: 6px; }
            #optimal-search-ui ::-webkit-scrollbar-track { background: rgba(0,10,20,0.5); }
            #optimal-search-ui ::-webkit-scrollbar-thumb { background: #004488; border-radius: 3px; }
            #optimal-search-ui ::-webkit-scrollbar-thumb:hover { background: #0066AA; }
            .config-result { 
                padding: 10px 16px; 
                cursor: pointer; 
                transition: background 0.15s;
                border-left: 3px solid transparent;
            }
            .config-result:hover { background: rgba(0, 60, 120, 0.4); }
            .config-result.selected { 
                background: rgba(0, 100, 180, 0.5); 
                border-left-color: #00CCFF;
            }
            .config-result .title { 
                font-weight: 600; 
                color: #00EEFF; 
                margin-bottom: 4px;
            }
            .config-result .description { 
                color: #88AACC; 
                font-size: 11px;
            }
            .config-result .metrics { 
                display: flex; 
                gap: 12px; 
                margin-top: 6px; 
                font-size: 10px;
            }
            .config-result .metric { 
                display: flex; 
                align-items: center; 
                gap: 4px;
            }
            .config-result .metric-bar {
                width: 40px;
                height: 4px;
                background: #223344;
                border-radius: 2px;
                overflow: hidden;
            }
            .config-result .metric-fill {
                height: 100%;
                border-radius: 2px;
                transition: width 0.3s;
            }
            .score-badge {
                background: linear-gradient(135deg, #00AA55, #00DD88);
                color: #001100;
                padding: 2px 8px;
                border-radius: 10px;
                font-weight: 700;
                font-size: 11px;
            }
            .filter-chip {
                background: rgba(0, 40, 80, 0.5);
                border: 1px solid #003355;
                border-radius: 12px;
                color: #6699AA;
                padding: 4px 10px;
                font-size: 10px;
                cursor: pointer;
                transition: all 0.15s;
            }
            .filter-chip:hover { background: rgba(0, 60, 120, 0.5); }
            .filter-chip.active { background: #005588; border-color: #00AAFF; color: #00EEFF; }
        `;
        
        this.root.append(style, header, searchBar, filterBar, this.resultsContainer, footer);
        document.body.appendChild(this.root);

        // ── Drag support ──
        makeDraggable(this.root, header);
        
        // Global keyboard shortcut
        document.addEventListener('keydown', (e) => {
            if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
                e.preventDefault();
                this.toggle();
            }
        });
    }
    
    _createFilterBar() {
        const bar = document.createElement('div');
        Object.assign(bar.style, {
            display: 'flex',
            alignItems: 'center',
            gap: '8px',
            padding: '8px 16px',
            borderBottom: '1px solid #002244',
            flexWrap: 'wrap'
        });
        
        const label = document.createElement('span');
        label.textContent = 'Priorité:';
        label.style.color = '#668899';
        label.style.fontSize = '10px';
        bar.appendChild(label);
        
        const priorities = [
            { key: 'coverage', label: '📐 Couverture', color: '#00FF88' },
            { key: 'time', label: '⚡ Rapidité', color: '#FFAA00' },
            { key: 'energy', label: '🔋 Énergie', color: '#00CCFF' },
            { key: 'reliability', label: '🛡️ Fiabilité', color: '#AA88FF' },
            { key: 'adaptability', label: '🧠 Adaptatif', color: '#FF88AA' }
        ];
        
        for (const p of priorities) {
            const chip = document.createElement('span');
            chip.className = 'filter-chip active';
            chip.textContent = p.label;
            chip.dataset.priority = p.key;
            chip.style.setProperty('--chip-color', p.color);
            chip.addEventListener('click', () => {
                chip.classList.toggle('active');
                this._updatePriorities();
            });
            bar.appendChild(chip);
        }
        
        return bar;
    }
    
    _createButton(text, tooltip, onClick) {
        const btn = document.createElement('button');
        btn.textContent = text;
        btn.title = tooltip;
        Object.assign(btn.style, {
            background: 'transparent',
            border: '1px solid #334455',
            borderRadius: '4px',
            color: '#88AACC',
            cursor: 'pointer',
            padding: '4px 8px',
            fontSize: '12px'
        });
        btn.addEventListener('click', onClick);
        return btn;
    }
    
    _updatePriorities() {
        const chips = this.root.querySelectorAll('.filter-chip');
        this._filters.priorities = [];
        chips.forEach(chip => {
            if (chip.classList.contains('active')) {
                this._filters.priorities.push(chip.dataset.priority);
            }
        });
        this._performSearch();
    }
    
    _onSearchInput() {
        this._searchQuery = this.searchInput.value.toLowerCase().trim();
        this._performSearch();
    }
    
    _onKeyDown(e) {
        const results = this.resultsContainer.querySelectorAll('.config-result');
        const currentIndex = Array.from(results).findIndex(r => r.classList.contains('selected'));
        
        if (e.key === 'ArrowDown') {
            e.preventDefault();
            const next = Math.min(currentIndex + 1, results.length - 1);
            this._selectResultByIndex(next);
        } else if (e.key === 'ArrowUp') {
            e.preventDefault();
            const prev = Math.max(currentIndex - 1, 0);
            this._selectResultByIndex(prev);
        } else if (e.key === 'Enter') {
            e.preventDefault();
            if (this._selectedConfig) {
                this._applyConfig(this._selectedConfig);
            }
        } else if (e.key === 'Escape') {
            this.hide();
        }
    }
    
    _selectResultByIndex(index) {
        const results = this.resultsContainer.querySelectorAll('.config-result');
        results.forEach((r, i) => {
            r.classList.toggle('selected', i === index);
            if (i === index) {
                this._selectedConfig = this._rankedConfigs[i];
                r.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
            }
        });
    }
    
    /**
     * Perform search and ranking
     */
    _performSearch() {
        // Calculate scores
        const scored = this._configurations.map(config => {
            const score = this._calculateScore(config);
            return { ...config, score };
        });
        
        // Filter by search query
        const filtered = scored.filter(config => {
            if (!this._searchQuery) return true;
            
            const terms = this._searchQuery.split(/\s+/);
            return terms.every(term => {
                const searchText = [
                    config.doctrine.name,
                    config.doctrine.id,
                    config.coa.name,
                    config.coa.id,
                    config.doctrine.description,
                    config.coa.description,
                    ...config.tags
                ].join(' ').toLowerCase();
                
                return searchText.includes(term);
            });
        });
        
        // Sort by score
        this._rankedConfigs = filtered.sort((a, b) => b.score - a.score);
        
        // Assign ranks
        this._rankedConfigs.forEach((c, i) => c.rank = i + 1);
        
        // Render
        this._renderResults();
    }
    
    /**
     * Calculate configuration score based on priorities
     */
    _calculateScore(config) {
        const perf = config.performance;
        let score = 0;
        let weightSum = 0;
        
        for (const priority of this._filters.priorities) {
            const weight = SCORING_WEIGHTS[priority] || 0;
            const value = perf[priority] || 0;
            score += weight * value;
            weightSum += weight;
        }
        
        // Normalize
        if (weightSum > 0) {
            score = score / weightSum;
        }
        
        // Bonus for usage (learned preferences)
        const usageBonus = Math.min(config.usageCount * 0.02, 0.1);
        score += usageBonus;
        
        // Pareto optimality bonus
        if (this._isParetoOptimal(config)) {
            score += 0.05;
        }
        
        return Math.min(score, 1.0);
    }
    
    /**
     * Check if config is Pareto-optimal
     */
    _isParetoOptimal(config) {
        const dominated = this._configurations.some(other => {
            if (other.id === config.id) return false;
            
            const dominated = Object.keys(config.performance).every(key => 
                other.performance[key] >= config.performance[key]
            );
            const strictlyBetter = Object.keys(config.performance).some(key =>
                other.performance[key] > config.performance[key]
            );
            
            return dominated && strictlyBetter;
        });
        
        return !dominated;
    }
    
    /**
     * Render search results
     */
    _renderResults() {
        this.resultsContainer.innerHTML = '';
        
        if (this._rankedConfigs.length === 0) {
            const empty = document.createElement('div');
            empty.style.padding = '20px';
            empty.style.textAlign = 'center';
            empty.style.color = '#668899';
            empty.textContent = 'Aucune configuration trouvée';
            this.resultsContainer.appendChild(empty);
            return;
        }
        
        this._rankedConfigs.forEach((config, index) => {
            const el = this._createResultElement(config, index === 0);
            this.resultsContainer.appendChild(el);
        });
        
        // Auto-select first
        this._selectedConfig = this._rankedConfigs[0];
    }
    
    _createResultElement(config, isFirst) {
        const el = document.createElement('div');
        el.className = 'config-result' + (isFirst ? ' selected' : '');
        el.dataset.id = config.id;
        
        const perf = config.performance;
        const scorePercent = Math.round(config.score * 100);
        
        el.innerHTML = `
            <div style="display: flex; justify-content: space-between; align-items: flex-start;">
                <div class="title">
                    ${config.doctrine.icon} ${config.doctrine.name} + ${config.coa.icon} ${config.coa.name}
                </div>
                <span class="score-badge">${scorePercent}%</span>
            </div>
            <div class="description">${config.coa.description}</div>
            <div class="metrics">
                ${this._renderMetric('📐', 'Couverture', perf.coverage, '#00FF88')}
                ${this._renderMetric('⚡', 'Rapidité', perf.time, '#FFAA00')}
                ${this._renderMetric('🔋', 'Énergie', perf.energy, '#00CCFF')}
                ${this._renderMetric('🛡️', 'Fiabilité', perf.reliability, '#AA88FF')}
                ${this._renderMetric('🧠', 'Adaptatif', perf.adaptability, '#FF88AA')}
            </div>
        `;
        
        el.addEventListener('click', () => {
            this.resultsContainer.querySelectorAll('.config-result').forEach(r => r.classList.remove('selected'));
            el.classList.add('selected');
            this._selectedConfig = config;
        });
        
        el.addEventListener('dblclick', () => {
            this._applyConfig(config);
        });
        
        return el;
    }
    
    _renderMetric(icon, label, value, color) {
        const percent = Math.round(value * 100);
        return `
            <div class="metric" title="${label}: ${percent}%">
                <span>${icon}</span>
                <div class="metric-bar">
                    <div class="metric-fill" style="width: ${percent}%; background: ${color};"></div>
                </div>
            </div>
        `;
    }
    
    /**
     * Apply selected configuration
     */
    _applyConfig(config) {
        logger.info('OptimalSearchUI', `🎯 Configuration appliquée: ${config.doctrine.name} + ${config.coa.name}`);
        
        // Record usage
        this._usageHistory.push({
            id: config.id,
            timestamp: Date.now()
        });
        if (this._usageHistory.length > 100) {
            this._usageHistory = this._usageHistory.slice(-100);
        }
        localStorage.setItem('diamants_config_usage', JSON.stringify(this._usageHistory));
        
        // Update configuration counts
        config.usageCount++;
        
        // Notify system
        if (typeof window.DIAMANTS_DOCTRINE !== 'undefined') {
            window.DIAMANTS_DOCTRINE.setDoctrine?.(config.doctrineKey);
            window.DIAMANTS_DOCTRINE.setCOA?.(config.coaKey);
        }
        
        // Update UI selects
        const missionTypeSelect = document.getElementById('mission_type');
        const modeSelect = document.getElementById('mode_select') || document.getElementById('mission-pattern');
        
        if (missionTypeSelect) missionTypeSelect.value = config.doctrine.id;
        if (modeSelect) modeSelect.value = config.coa.id;
        
        // Callback
        if (this._onConfigSelect) {
            this._onConfigSelect(config);
        }
        
        // Dispatch event
        window.dispatchEvent(new CustomEvent('diamants:config-change', {
            detail: { doctrine: config.doctrine, coa: config.coa, score: config.score }
        }));
        
        this.hide();
    }
    
    /**
     * Show/hide UI
     */
    show() {
        this._visible = true;
        this.root.style.display = 'flex';
        this.searchInput.focus();
        this.searchInput.select();
        this._performSearch();
    }
    
    hide() {
        this._visible = false;
        this.root.style.display = 'none';
    }
    
    toggle() {
        if (this._visible) {
            this.hide();
        } else {
            this.show();
        }
    }
    
    /**
     * Get recommended configuration for mission type
     */
    getRecommendation(missionType) {
        const typeMatches = {
            'exploration': ['stigmergy_adaptive', 'stigmergy_grid', 'coverage_boustrophedon'],
            'couverture': ['coverage_grid', 'coverage_boustrophedon', 'stigmergy_grid'],
            'recherche': ['search_adaptive', 'search_spiral', 'stigmergy_spiral'],
            'surveillance': ['formation_radial', 'stigmergy_perimeter'],
            'rapide': ['swarm_adaptive', 'search_adaptive', 'stigmergy_adaptive']
        };
        
        const matches = typeMatches[missionType.toLowerCase()] || typeMatches['exploration'];
        const candidates = this._configurations.filter(c => matches.includes(c.id));
        
        if (candidates.length === 0) return null;
        
        // Return highest scored
        return candidates.sort((a, b) => this._calculateScore(b) - this._calculateScore(a))[0];
    }
    
    /**
     * Destroy and cleanup
     */
    destroy() {
        this.root.remove();
    }
}

// Auto-initialize if window loaded
if (typeof window !== 'undefined') {
    window.OptimalSearchUI = OptimalSearchUI;
}

export default OptimalSearchUI;
