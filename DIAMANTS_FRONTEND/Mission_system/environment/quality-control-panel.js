/**
 * DIAMANTS V3 - Quality Control Panel
 * Panneau de contrôle de la qualité graphique
 */
import { shaderQualityManager, QUALITY_LEVELS } from '../shaders/shader-quality-manager.js';

// Sécurité: ES6 modules n'ont pas accès aux variables globales automatiquement
// Récupérer les fonctions de logging depuis window
const log = window.log || ((...args) => console.log(...args));
const warn = window.warn || ((...args) => console.warn(...args));
const error = window.error || ((...args) => console.error(...args));

// Mode silencieux pour les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

export class QualityControlPanel {
    constructor(container) {
        this.container = container;
        this.panel = null;
        this.grassField = null;
        this.callbacks = {
            onQualityChange: null,
            onAutoToggle: null
        };
        
        // Système de mise à jour FPS
        this.frameCount = 0;
        this.lastFPSUpdate = performance.now();
        this.currentFPS = 0;
        this.updateInterval = null;
        
        this.init();
    }

    init() {
        this.createPanel();
        
        // Délai pour s'assurer que le DOM est prêt
        setTimeout(() => {
            this.setupEventListeners();
            this.updateDisplay();
            this.startMetricsUpdating();
        }, 100);
    }

    createPanel() {
        // Vérifier s'il y a déjà un panneau existant
        const existingPanel = this.container.querySelector('.quality-control-panel');
        if (existingPanel) {
            console.warn('⚠️ Panneau de qualité déjà existant, suppression...');
            existingPanel.remove();
        }

        this.panel = document.createElement('div');
        this.panel.className = 'quality-control-panel';
        this.panel.innerHTML = `
            <div class="quality-panel-header">
                <h3>🎮 Qualité Graphique</h3>
                <button class="panel-toggle">-</button>
            </div>
            <div class="quality-panel-content">
                <!-- Sélection de qualité -->
                <div class="quality-section">
                    <label>Niveau de qualité:</label>
                    <select class="quality-selector">
                        <option value="${QUALITY_LEVELS.LOW}">🟢 Faible - Performance</option>
                        <option value="${QUALITY_LEVELS.MEDIUM}">🟡 Moyenne - Équilibré</option>
                        <option value="${QUALITY_LEVELS.HIGH}">🟠 Élevée - Qualité</option>
                        <option value="${QUALITY_LEVELS.ULTRA}">🔴 Ultra - Maximum</option>
                        <option value="${QUALITY_LEVELS.DEV}">🔧 Développement</option>
                        <option value="${QUALITY_LEVELS.EXPERIMENTAL}">⚗️ Expérimental</option>
                    </select>
                </div>

                <!-- Qualité automatique -->
                <div class="auto-quality-section">
                    <label class="checkbox-label">
                        <input type="checkbox" class="auto-quality-toggle">
                        🤖 Qualité automatique
                    </label>
                    <small>Ajuste automatiquement selon les performances</small>
                </div>

                <!-- Métriques de performance -->
                <div class="performance-metrics">
                    <h4>📊 Performances</h4>
                    <div class="metric">
                        <span>FPS:</span>
                        <span class="fps-counter">--</span>
                    </div>
                    <div class="metric">
                        <span>Brins d'herbe:</span>
                        <span class="grass-count">--</span>
                    </div>
                    <div class="metric">
                        <span>Chunks actifs:</span>
                        <span class="chunks-count">--</span>
                    </div>
                    <div class="metric">
                        <span>Qualité active:</span>
                        <span class="current-quality">Medium</span>
                    </div>
                </div>

                <!-- Informations sur la qualité -->
                <div class="quality-info">
                    <h4>ℹ️ Informations</h4>
                    <div class="quality-description">
                        Équilibre performance/qualité - recommandé
                    </div>
                    <div class="quality-features">
                        <div class="feature">🌬️ Vent: <span class="wind-status">Oui</span></div>
                        <div class="feature">🌫️ Ombres: <span class="shadows-status">Oui</span></div>
                        <div class="feature">🔍 Debug: <span class="debug-status">Non</span></div>
                        <div class="feature">📊 Densité: <span class="density-status">1.0</span></div>
                        <div class="feature">🎨 Shader: <span class="shader-status">stable</span></div>
                    </div>
                </div>

                <!-- Détection automatique -->
                <div class="detection-section">
                    <button class="detect-button">🔍 Détecter Qualité Optimale</button>
                </div>
            </div>
        `;

        // Styles CSS
        this.addStyles();
        
        this.container.appendChild(this.panel);
    }

    addStyles() {
        const style = document.createElement('style');
        style.textContent = `
            .quality-control-panel {
                position: fixed;
                top: 20px;
                right: 20px;
                width: 320px;
                max-height: 80vh;
                background: rgba(0, 0, 0, 0.9);
                border: 2px solid #00ff00;
                border-radius: 8px;
                color: white;
                font-family: 'Courier New', monospace;
                font-size: 13px;
                z-index: 99999;
                backdrop-filter: blur(10px);
                box-shadow: 0 0 20px rgba(0, 255, 0, 0.3);
            }

            .quality-panel-header {
                padding: 12px;
                border-bottom: 2px solid #00ff00;
                display: flex;
                justify-content: space-between;
                align-items: center;
                background: rgba(0, 255, 0, 0.1);
                border-radius: 8px 8px 0 0;
            }

            .quality-panel-header h3 {
                margin: 0;
                font-size: 14px;
            }

            .panel-toggle {
                background: none;
                border: none;
                color: white;
                cursor: pointer;
                font-size: 16px;
            }

            .quality-panel-content {
                padding: 15px;
                max-height: 400px;
                overflow-y: auto;
            }

            .quality-panel-content.collapsed {
                display: none !important;
                height: 0 !important;
                overflow: hidden !important;
                padding: 0 !important;
                opacity: 0 !important;
            }

            .quality-section, 
            .auto-quality-section, 
            .performance-metrics, 
            .quality-info,
            .detection-section {
                margin-bottom: 15px;
                padding: 10px;
                background: rgba(255, 255, 255, 0.05);
                border-radius: 4px;
            }

            .quality-section label,
            .auto-quality-section label {
                display: block;
                margin-bottom: 5px;
                font-weight: bold;
            }

            .quality-selector {
                width: 100%;
                padding: 5px;
                background: rgba(0, 0, 0, 0.7);
                border: 1px solid #666;
                border-radius: 4px;
                color: white;
            }

            .checkbox-label {
                display: flex !important;
                align-items: center;
                gap: 8px;
                cursor: pointer;
            }

            .auto-quality-toggle {
                margin: 0;
            }

            .performance-metrics h4,
            .quality-info h4 {
                margin: 0 0 10px 0;
                font-size: 12px;
                color: #aaa;
            }

            .metric {
                display: flex;
                justify-content: space-between;
                margin-bottom: 5px;
            }

            .fps-counter {
                font-weight: bold;
                color: #4CAF50;
            }

            .fps-counter.low {
                color: #F44336;
            }

            .fps-counter.medium {
                color: #FF9800;
            }

            .quality-description {
                font-style: italic;
                color: #bbb;
                margin-bottom: 10px;
            }

            .quality-features {
                display: grid;
                gap: 5px;
            }

            .feature {
                display: flex;
                justify-content: space-between;
                align-items: center;
            }

            .detect-button {
                width: 100%;
                padding: 8px;
                background: linear-gradient(45deg, #4CAF50, #45a049);
                border: none;
                border-radius: 4px;
                color: white;
                cursor: pointer;
                font-size: 12px;
                transition: background 0.3s;
            }

            .detect-button:hover {
                background: linear-gradient(45deg, #45a049, #4CAF50);
            }

            .detect-button:disabled {
                background: #666;
                cursor: not-allowed;
            }

            /* Scrollbar personnalisée */
            .quality-panel-content::-webkit-scrollbar {
                width: 6px;
            }

            .quality-panel-content::-webkit-scrollbar-track {
                background: rgba(255, 255, 255, 0.1);
            }

            .quality-panel-content::-webkit-scrollbar-thumb {
                background: rgba(255, 255, 255, 0.3);
                border-radius: 3px;
            }
        `;
        document.head.appendChild(style);
    }

    setupEventListeners() {
        const qualitySelector = this.panel.querySelector('.quality-selector');
        const autoToggle = this.panel.querySelector('.auto-quality-toggle');
        const panelToggle = this.panel.querySelector('.panel-toggle');
        const detectButton = this.panel.querySelector('.detect-button');
        const panelContent = this.panel.querySelector('.quality-panel-content');

        // Debug: vérifier que les éléments sont trouvés
        if (!window.SILENT_MODE) {
            console.log('🔍 Debug setupEventListeners:');
            console.log('panelToggle:', panelToggle);
            console.log('panelContent:', panelContent);
            console.log('panelToggle text:', panelToggle ? panelToggle.textContent : 'null');
        }

        // Debug: vérifier que tous les éléments sont trouvés
        console.log('Elements trouvés:');
        console.log('qualitySelector:', !!qualitySelector);
        console.log('autoToggle:', !!autoToggle);
        console.log('panelToggle:', !!panelToggle);
        console.log('detectButton:', !!detectButton);
        console.log('panelContent:', !!panelContent);

        // Sélection de qualité
        qualitySelector.addEventListener('change', async (e) => {
            const raw = String(e.target.value || '').toLowerCase();
            // Map raw to known levels
            const quality = Object.values(QUALITY_LEVELS).includes(raw) ? raw : QUALITY_LEVELS.MEDIUM;
            log(`🎮 Tentative de changement vers: ${quality}`);
            qualitySelector.disabled = true;
            
            // Indicateur visuel de changement
            this.panel.style.borderColor = '#ff6600';
            this.panel.querySelector('.quality-panel-header').style.background = 'rgba(255, 102, 0, 0.2)';
            
            try {
                log(`🎨 Changement vers qualité: ${quality}`);
                log(`🔍 shaderQualityManager disponible:`, !!shaderQualityManager);
                log(`🔍 setQuality disponible:`, !!shaderQualityManager.setQuality);
                
                // D'abord changer dans le shaderQualityManager
                if (shaderQualityManager && shaderQualityManager.setQuality) {
                    await shaderQualityManager.setQuality(quality);
                    log(`✅ shaderQualityManager mis à jour`);
                } else {
                    warn(`⚠️ shaderQualityManager non disponible`);
                }
                
                // Puis notifier le champ d'herbe si disponible
                if (this.grassField && this.grassField.setQuality) {
                    await this.grassField.setQuality(quality);
                    log(`✅ grassField mis à jour`);
                } else if (this.grassField && this.grassField.updateQuality) {
                    await this.grassField.updateQuality();
                    log(`✅ grassField rafraîchi`);
                } else {
                    warn(`⚠️ grassField non disponible ou sans setQuality`);
                }
                
                // Mettre à jour l'affichage
                await this.updateDisplay();
                
                // Notifier via callback
                if (this.callbacks.onQualityChange) {
                    this.callbacks.onQualityChange(quality);
                }
                
                log(`✅ Qualité changée vers: ${quality}`);
                
                // Retour à la normale
                this.panel.style.borderColor = '#00ff00';
                this.panel.querySelector('.quality-panel-header').style.background = 'rgba(0, 255, 0, 0.1)';
                
            } catch (error) {
                error('❌ Erreur lors du changement de qualité:', error);
                // Indicateur d'erreur
                this.panel.style.borderColor = '#ff0000';
                setTimeout(() => {
                    this.panel.style.borderColor = '#00ff00';
                    this.panel.querySelector('.quality-panel-header').style.background = 'rgba(0, 255, 0, 0.1)';
                }, 2000);
            } finally {
                qualitySelector.disabled = false;
            }
        });

        // Toggle qualité automatique
        autoToggle.addEventListener('change', (e) => {
            const enabled = e.target.checked;
            shaderQualityManager.setAutoQuality(enabled);
            qualitySelector.disabled = enabled;
            
            if (this.callbacks.onAutoToggle) {
                this.callbacks.onAutoToggle(enabled);
            }
        });

                // Toggle panneau
        if (panelToggle && panelContent) {
            panelToggle.addEventListener('click', (e) => {
                e.stopPropagation(); // Empêcher la propagation
                panelContent.classList.toggle('collapsed');
                panelToggle.textContent = panelContent.classList.contains('collapsed') ? '+' : '-';
                
                // Garder le contour vert dans tous les cas
                this.panel.style.border = '2px solid #00ff00';
            });
            
            if (!window.SILENT_MODE) {
                console.log('✅ Toggle event listener attached successfully');
            }
        } else {
            console.error('❌ Cannot attach toggle listener - missing elements:', {
                panelToggle: !!panelToggle,
                panelContent: !!panelContent
            });
        }

        // Détection automatique
    detectButton.addEventListener('click', async () => {
            if (this.grassField && this.grassField.detectOptimalQuality) {
                detectButton.disabled = true;
                detectButton.textContent = '🔍 Détection...';
                
                try {
        const renderer = (typeof window !== 'undefined' && window.renderer) ? window.renderer : undefined;
        const optimal = await this.grassField.detectOptimalQuality(renderer);
            // Mettre à jour le sélecteur et l'affichage sur la qualité optimale
            const qualitySelector = this.panel.querySelector('.quality-selector');
            if (qualitySelector) qualitySelector.value = optimal;
            await this.updateDisplay();
                } catch (error) {
                    error('Erreur lors de la détection:', error);
                } finally {
                    detectButton.disabled = false;
                    detectButton.textContent = '🔍 Détecter Qualité Optimale';
                }
            }
        });

    // La boucle de métriques est gérée par startMetricsUpdating() pour éviter les doublons
    }

    async updateDisplay() {
        try {
            log('🔄 Mise à jour du panneau de qualité...');
            
            // Obtenir la qualité actuelle
            let currentQuality = QUALITY_LEVELS.MEDIUM; // défaut (lowercase)
            
            if (shaderQualityManager && shaderQualityManager.getCurrentLevel) {
                try {
                    currentQuality = shaderQualityManager.getCurrentLevel();
                    log(`📊 Qualité actuelle: ${currentQuality}`);
                } catch (error) {
                    warn('Erreur lors de la récupération de la qualité actuelle:', error);
                }
            }
            
            // Obtenir les infos détaillées
            let qualityInfo = { features: {}, description: this.getQualityDescription(currentQuality) };
            
            if (shaderQualityManager && shaderQualityManager.getShaders) {
                try {
                    qualityInfo = await shaderQualityManager.getShaders();
                    qualityInfo.features = qualityInfo.features || {};
                } catch (error) {
                    warn('Erreur lors de la récupération des shaders:', error);
                }
            }

            // Mise à jour du sélecteur - correction importante
            const qualitySelector = this.panel.querySelector('.quality-selector');
            if (qualitySelector) {
                qualitySelector.value = currentQuality;
                log(`🔄 Sélecteur mis à jour: ${currentQuality}`);
            }

            // Mise à jour de la description
            const description = this.panel.querySelector('.quality-description');
            if (description) {
                description.textContent = qualityInfo.description;
            }

            // Mise à jour des features
            const features = qualityInfo.features;
            const windStatus = this.panel.querySelector('.wind-status');
            if (windStatus) windStatus.textContent = features.wind ? 'Oui' : 'Non';
            
            const shadowsStatus = this.panel.querySelector('.shadows-status');
            if (shadowsStatus) shadowsStatus.textContent = features.shadows ? 'Oui' : 'Non';
            
            const debugStatus = this.panel.querySelector('.debug-status');
            if (debugStatus) debugStatus.textContent = features.debug ? 'Oui' : 'Non';
            
            const densityStatus = this.panel.querySelector('.density-status');
            if (densityStatus) densityStatus.textContent = features.density?.toFixed(1) || '1.0';
            
            // Nom du shader selon la qualité
            const shaderStatus = this.panel.querySelector('.shader-status');
            if (shaderStatus) {
                shaderStatus.textContent = String(currentQuality);
            }

            // Mise à jour de la qualité active
            const currentQualityDisplay = this.panel.querySelector('.current-quality');
            if (currentQualityDisplay) {
                const name = String(currentQuality);
                currentQualityDisplay.textContent = name.charAt(0).toUpperCase() + name.slice(1);
            }
            
            log(`✅ Panneau mis à jour pour qualité: ${currentQuality}`);
        } catch (error) {
            error('❌ Erreur lors de la mise à jour de l\'affichage:', error);
        }
    }

    getQualityDescription(quality) {
        const key = String(quality).toLowerCase();
        const descriptions = {
            low: 'Performance maximale - Rendu simplifié',
            medium: 'Équilibre performance/qualité - recommandé',
            high: 'Qualité élevée - Effets avancés',
            ultra: 'Qualité maximale - Tous les effets',
            dev: 'Mode développement - Debug activé',
            experimental: 'Fonctions expérimentales'
        };
        return descriptions[key] || 'Qualité personnalisée';
    }

    updatePerformanceDisplay() {
        // Debug réduit (log toutes les 5 secondes seulement)
        if (!this._lastDebugLog) this._lastDebugLog = 0;
        const shouldLog = performance.now() - this._lastDebugLog > 5000;
        if (shouldLog) {
            this._lastDebugLog = performance.now();
            log('🔄 updatePerformanceDisplay (log réduit)');
        }
        
        // Calcul FPS temps réel
        this.frameCount++;
        const now = performance.now();
        if (now - this.lastFPSUpdate > 1000) { // Mise à jour toutes les secondes
            this.currentFPS = Math.round((this.frameCount * 1000) / (now - this.lastFPSUpdate));
            this.frameCount = 0;
            this.lastFPSUpdate = now;
            // Log FPS seulement si debug activé
            if (shouldLog) log(`📊 FPS calculé: ${this.currentFPS}`);
        }
        
        // Mise à jour FPS
        const fpsCounter = this.panel?.querySelector('.fps-counter');
        if (fpsCounter) {
            fpsCounter.textContent = this.currentFPS || '0';
            // Log réduit
            if (shouldLog && !window.SILENT_MODE) log(`✅ FPS affiché: ${this.currentFPS || '0'}`);
            
            // Couleur selon FPS
            fpsCounter.className = 'fps-counter';
            if (this.currentFPS < 30) {
                fpsCounter.classList.add('low');
            } else if (this.currentFPS < 50) {
                fpsCounter.classList.add('medium');
            }
        } else {
            warn('❌ Element .fps-counter introuvable');
        }

        // Affichage des brins d'herbe: réel si dispo, estimation sinon
        const grassCount = this.panel?.querySelector('.grass-count');
        if (grassCount) {
            const currentQuality = this.getCurrentQualityLevel();
            let value = null;
            try {
                if (this.grassField && this.grassField.getQualityInfo) {
                    const info = this.grassField.getQualityInfo();
                    if (info && typeof info.totalGrassInstances === 'number' && info.totalGrassInstances > 0) {
                        value = info.totalGrassInstances;
                    }
                }
            } catch (_) {}
            if (value == null) value = this.getEstimatedGrassCount(currentQuality);
            grassCount.textContent = Number(value).toLocaleString();
            // Log réduit
            if (shouldLog && !window.SILENT_MODE) log(`✅ Brins d'herbe affichés: ${value} (source: ${value === this.getEstimatedGrassCount(currentQuality) ? 'estimation' : 'réel'})`);
        } else {
            warn('❌ Element .grass-count introuvable');
        }
        
        // Affichage des chunks: réel si dispo, estimation sinon
        const chunksCount = this.panel?.querySelector('.chunks-count');
        if (chunksCount) {
            const currentQuality = this.getCurrentQualityLevel();
            let value = null;
            try {
                if (this.grassField && this.grassField.getQualityInfo) {
                    const info = this.grassField.getQualityInfo();
                    if (info && typeof info.activeChunks === 'number' && info.activeChunks >= 0) {
                        value = info.activeChunks;
                    }
                }
            } catch (_) {}
            if (value == null) value = this.getEstimatedChunksCount(currentQuality);
            chunksCount.textContent = String(value);
            // Log réduit
            if (shouldLog && !window.SILENT_MODE) log(`✅ Chunks affichés: ${value} (source: ${value === this.getEstimatedChunksCount(currentQuality) ? 'estimation' : 'réel'})`);
        } else {
            warn('❌ Element .chunks-count introuvable');
        }
    }

    getCurrentQualityLevel() {
        try {
            if (shaderQualityManager && shaderQualityManager.getCurrentLevel) {
                return shaderQualityManager.getCurrentLevel();
            }
        } catch (error) {
            warn('Erreur getCurrentLevel:', error);
        }
        
        // Fallback: lire depuis le sélecteur
        const qualitySelector = this.panel?.querySelector('.quality-selector');
        if (qualitySelector && qualitySelector.value) {
            return qualitySelector.value;
        }
        
        return 'MEDIUM'; // Défaut
    }

    getEstimatedGrassCount(quality) {
        const key = String(quality).toLowerCase();
        const grassCounts = {
            low: 50000,
            medium: 125000,
            high: 250000,
            ultra: 500000,
            dev: 100000,
            experimental: 200000
        };
        return grassCounts[key] || 125000;
    }

    getEstimatedChunksCount(quality) {
        const key = String(quality).toLowerCase();
        const chunksCounts = {
            low: 4,
            medium: 9,
            high: 16,
            ultra: 25,
            dev: 12,
            experimental: 20
        };
        return chunksCounts[key] || 9;
    }

    /**
     * Démarre la mise à jour périodique des métriques
     */
    startMetricsUpdating() {
        // Boucle FPS via requestAnimationFrame
        const step = () => {
            this.updatePerformanceDisplay();
            this._rafId = requestAnimationFrame(step);
        };
        if (!this._rafId) this._rafId = requestAnimationFrame(step);
    }

    /**
     * Arrête la mise à jour des métriques
     */
    stopMetricsUpdating() {
    if (this._rafId) cancelAnimationFrame(this._rafId);
    this._rafId = null;
    }

    /**
     * Lie le panneau à un champ d'herbe
     */
    bindToGrassField(grassField) {
        this.grassField = grassField;
        this.updateDisplay();
    }

    /**
     * Définit les callbacks
     */
    setCallbacks(callbacks) {
        this.callbacks = { ...this.callbacks, ...callbacks };
    }

    /**
     * Montre/cache le panneau
     */
    toggle(visible) {
        this.panel.style.display = visible ? 'block' : 'none';
    }

    /**
     * Nettoie le panneau
     */
    dispose() {
        this.stopMetricsUpdating();
        if (this.panel && this.panel.parentNode) {
            this.panel.parentNode.removeChild(this.panel);
        }
    }

    /**
     * Force la mise à jour immédiate des métriques
     */
    forceUpdate() {
        this.updatePerformanceDisplay();
    }
}

export default QualityControlPanel;
