/*
 * DIAMANTS - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
 * 
 * Copyright (c) 2025 DIAMANTS Project Contributors
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * DIAMANTS Dashboard Controller
 * ================================
 * Contrôleur principal du dashboard web
 */

class DiamantDashboard {
    constructor(wsClient) {
        this.wsClient = wsClient;
        this.missionStartTime = null;
        this.missionTimer = null;
        
        // Éléments DOM
        this.elements = {
            activeDrones: document.getElementById('active-drones'),
            intelligenceScore: document.getElementById('intelligence-score'),
            coverageArea: document.getElementById('coverage-area'),
            missionTime: document.getElementById('mission-time'),
            
            // Contrôles
            startMissionBtn: document.getElementById('start-mission'),
            pauseMissionBtn: document.getElementById('pause-mission'),
            stopMissionBtn: document.getElementById('stop-mission'),
            explorationMode: document.getElementById('exploration-mode'),
            speedSlider: document.getElementById('speed-slider'),
            speedValue: document.getElementById('speed-value'),
            
            // Métriques intelligence
            globalScoreBar: document.getElementById('global-score-bar'),
            globalScoreText: document.getElementById('global-score-text'),
            coordinationBar: document.getElementById('coordination-bar'),
            coordinationText: document.getElementById('coordination-text'),
            efficiencyBar: document.getElementById('efficiency-bar'),
            efficiencyText: document.getElementById('efficiency-text'),
            adaptationBar: document.getElementById('adaptation-bar'),
            adaptationText: document.getElementById('adaptation-text'),
            
            // Tables
            dronesTable: document.getElementById('drones-table')
        };
        
        this.init();
    }
    
    /**
     * Initialisation dashboard
     */
    init() {
        console.log('🎮 Initialisation Dashboard Controller');
        
        // Initialisation de la visualisation 3D DIAMANTS
        this.initEnhanced3DVisualization();
        
        this.setupEventListeners();
        this.setupWebSocketHandlers();
        this.updateUI();
        
        // Démarrer timer interface
        this.startUITimer();
    }
    
    /**
     * Initialisation visualisation 3D DIAMANTS
     */
    initEnhanced3DVisualization() {
        try {
            if (typeof DiamantEnhancedVisualization !== 'undefined') {
                this.diamantViz = new DiamantEnhancedVisualization('threejs-container');
                console.log('✅ Visualisation 3D DIAMANTS initialisée');
                
                // Configuration des contrôles
                this.setupVisualizationControls();
            } else {
                console.warn('⚠️ DiamantEnhancedVisualization non disponible');
                this.createVisualizationFallback();
            }
        } catch (error) {
            console.error('❌ Erreur init visualisation 3D:', error);
            this.createVisualizationFallback();
        }
    }
    
    /**
     * Configuration des contrôles de visualisation
     */
    setupVisualizationControls() {
        // Boutons de contrôle
        const resetCameraBtn = document.getElementById('reset-camera');
        const toggleTrailsBtn = document.getElementById('toggle-trails');
        const toggleGridBtn = document.getElementById('toggle-grid');
        const toggleDiamantFieldsBtn = document.getElementById('toggle-diamant-fields');
        const fullscreenBtn = document.getElementById('fullscreen-viz');
        
        if (resetCameraBtn) {
            resetCameraBtn.addEventListener('click', () => {
                if (this.diamantViz && this.diamantViz.camera) {
                    this.diamantViz.camera.position.set(12, 8, 12);
                    this.diamantViz.camera.lookAt(0, 0, 0);
                }
            });
        }
        
        if (toggleTrailsBtn) {
            toggleTrailsBtn.addEventListener('click', () => {
                if (this.diamantViz) {
                    this.diamantViz.toggleTrails();
                    toggleTrailsBtn.classList.toggle('active');
                }
            });
        }
        
        if (toggleGridBtn) {
            toggleGridBtn.addEventListener('click', () => {
                if (this.diamantViz) {
                    this.diamantViz.toggleGrid();
                    toggleGridBtn.classList.toggle('active');
                }
            });
        }
        
        if (toggleDiamantFieldsBtn) {
            toggleDiamantFieldsBtn.addEventListener('click', () => {
                if (this.diamantViz) {
                    this.diamantViz.toggleDiamantFields();
                    toggleDiamantFieldsBtn.classList.toggle('active');
                }
            });
        }
        
        if (fullscreenBtn) {
            fullscreenBtn.addEventListener('click', () => {
                this.toggleFullscreenVisualization();
            });
        }
    }
    
    /**
     * Fallback si visualisation 3D échoue
     */
    createVisualizationFallback() {
        const container = document.getElementById('threejs-container');
        if (container) {
            container.innerHTML = `
                <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100%; background: linear-gradient(45deg, #0d1b2a, #1b263b); color: #90e0ef; text-align: center;">
                    <div style="font-size: 3rem; margin-bottom: 1rem;">💎</div>
                    <h3>Visualisation DIAMANTS</h3>
                    <p style="margin: 1rem 0; opacity: 0.8;">Chargement des formules authentiques...</p>
                    <div class="loading-spinner" style="margin-top: 1rem;"></div>
                </div>
            `;
        }
    }
    
    /**
     * Plein écran pour la visualisation
     */
    toggleFullscreenVisualization() {
        const container = document.getElementById('threejs-container');
        if (container) {
            if (!document.fullscreenElement) {
                container.requestFullscreen().then(() => {
                    container.style.zIndex = '9999';
                    if (this.diamantViz) {
                        this.diamantViz.onWindowResize();
                    }
                });
            } else {
                document.exitFullscreen().then(() => {
                    container.style.zIndex = '';
                    if (this.diamantViz) {
                        this.diamantViz.onWindowResize();
                    }
                });
            }
        }
    }
    
    /**
     * Configuration événements DOM
     */
    setupEventListeners() {
        // Boutons mission
        if (this.elements.startMissionBtn) {
            this.elements.startMissionBtn.addEventListener('click', () => {
                this.startMission();
            });
        }
        
        if (this.elements.pauseMissionBtn) {
            this.elements.pauseMissionBtn.addEventListener('click', () => {
                this.pauseMission();
            });
        }
        
        if (this.elements.stopMissionBtn) {
            this.elements.stopMissionBtn.addEventListener('click', () => {
                this.stopMission();
            });
        }
        
        // Mode exploration
        if (this.elements.explorationMode) {
            this.elements.explorationMode.addEventListener('change', (e) => {
                this.setExplorationMode(e.target.value);
            });
        }
        
        // Slider vitesse
        if (this.elements.speedSlider) {
            this.elements.speedSlider.addEventListener('input', (e) => {
                const speed = parseFloat(e.target.value);
                this.setSwarmSpeed(speed);
                
                if (this.elements.speedValue) {
                    this.elements.speedValue.textContent = speed.toFixed(1);
                }
            });
        }
        
        // Boutons formation
        document.querySelectorAll('.formation-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const formation = e.target.dataset.formation;
                this.setFormation(formation);
                
                // Mettre à jour état visuel
                document.querySelectorAll('.formation-btn').forEach(b => 
                    b.classList.remove('active'));
                e.target.classList.add('active');
            });
        });
        
        // Contrôles visualisation
        const resetCameraBtn = document.getElementById('reset-camera');
        if (resetCameraBtn) {
            resetCameraBtn.addEventListener('click', () => {
                if (window.visualization) {
                    window.visualization.resetCamera();
                }
            });
        }
        
        const toggleTrailsBtn = document.getElementById('toggle-trails');
        if (toggleTrailsBtn) {
            toggleTrailsBtn.addEventListener('click', () => {
                if (window.visualization) {
                    window.visualization.toggleTrails();
                }
            });
        }
        
        const toggleGridBtn = document.getElementById('toggle-grid');
        if (toggleGridBtn) {
            toggleGridBtn.addEventListener('click', () => {
                if (window.visualization) {
                    window.visualization.toggleGrid();
                }
            });
        }
        
        const fullscreenBtn = document.getElementById('fullscreen-viz');
        if (fullscreenBtn) {
            fullscreenBtn.addEventListener('click', () => {
                this.toggleFullscreenVisualization();
            });
        }
    }
    
    /**
     * Configuration handlers WebSocket
     */
    setupWebSocketHandlers() {
        // Handler positions drones
        this.wsClient.onMessage('drone_position', (data) => {
            this.updateDronePosition(data);
        });
        
        // Handler intelligence collective
        this.wsClient.onMessage('intelligence_score', (data) => {
            this.updateIntelligenceScore(data.score);
        });
        
        // Handler couverture zone
        this.wsClient.onMessage('coverage_area', (data) => {
            this.updateCoverageArea(data.area);
        });
        
        // Handler rapport statut
        this.wsClient.onMessage('status_report', (data) => {
            this.updateStatusReport(data);
        });
        
        // Handler état initial
        this.wsClient.onMessage('initial_state', (data) => {
            this.updateFromInitialState(data);
        });
    }
    
    /**
     * Actions mission
     */
    startMission() {
        const missionType = this.elements.explorationMode?.value || 'autonomous';
        
        if (this.wsClient.startMission(missionType)) {
            this.missionStartTime = Date.now();
            this.startMissionTimer();
            
            // Mettre à jour interface
            if (this.elements.startMissionBtn) {
                this.elements.startMissionBtn.disabled = true;
            }
            if (this.elements.pauseMissionBtn) {
                this.elements.pauseMissionBtn.disabled = false;
            }
            if (this.elements.stopMissionBtn) {
                this.elements.stopMissionBtn.disabled = false;
            }
            
            this.showNotification('🚀 Mission démarrée', 'success');
        }
    }
    
    pauseMission() {
        if (this.wsClient.pauseMission()) {
            this.stopMissionTimer();
            this.showNotification('⏸️ Mission en pause', 'warning');
        }
    }
    
    stopMission() {
        if (this.wsClient.stopMission()) {
            this.missionStartTime = null;
            this.stopMissionTimer();
            
            // Réinitialiser interface
            if (this.elements.startMissionBtn) {
                this.elements.startMissionBtn.disabled = false;
            }
            if (this.elements.pauseMissionBtn) {
                this.elements.pauseMissionBtn.disabled = true;
            }
            if (this.elements.stopMissionBtn) {
                this.elements.stopMissionBtn.disabled = true;
            }
            
            this.showNotification('⏹️ Mission arrêtée', 'info');
        }
    }
    
    /**
     * Configuration paramètres
     */
    setExplorationMode(mode) {
        if (this.wsClient.setExplorationMode(mode)) {
            this.showNotification(`Mode: ${mode}`, 'info');
        }
    }
    
    setSwarmSpeed(speed) {
        if (this.wsClient.setSwarmSpeed(speed)) {
            console.log(`Vitesse essaim: ${speed} m/s`);
        }
    }
    
    setFormation(formation) {
        if (this.wsClient.setFormation(formation)) {
            this.showNotification(`Formation: ${formation}`, 'info');
        }
    }
    
    /**
     * Mise à jour données temps réel
     */
    updateDronePosition(data) {
        // Mettre à jour compteur drones actifs
        const activeDrones = this.wsClient.getActiveDroneCount();
        if (this.elements.activeDrones) {
            this.elements.activeDrones.textContent = activeDrones;
        }
        
        // Mettre à jour visualisation 3D DIAMANTS
        if (this.diamantViz && data.drones) {
            this.diamantViz.updateDroneData(data.drones);
            
            // Mise à jour des harmoniques DIAMANTS
            this.updateDiamantHarmonics(data.drones);
        }
        
        // Mettre à jour table drones
        this.updateDronesTable();
    }
    
    /**
     * Mise à jour des harmoniques DIAMANTS
     */
    updateDiamantHarmonics(droneData) {
        if (!this.diamantViz || !this.diamantViz.diamantFormulas) return;
        
        try {
            const harmonics = this.diamantViz.diamantFormulas.calculateHarmonics(droneData);
            const harmonicIds = [1, 2, 3, 7, 11, 12]; // Harmoniques affichées
            
            harmonicIds.forEach(id => {
                const value = harmonics[id - 1] || 0;
                
                // Mise à jour barre de progression
                const progressBar = document.getElementById(`harmonic-${id}`);
                if (progressBar) {
                    progressBar.style.width = `${value * 100}%`;
                }
                
                // Mise à jour valeur affichée
                const valueElement = document.getElementById(`value-${id}`);
                if (valueElement) {
                    valueElement.textContent = value.toFixed(2);
                }
            });
            
            // Animation des barres selon l'activité
            this.animateHarmonicBars(harmonics);
            
        } catch (error) {
            console.warn('Erreur mise à jour harmoniques:', error);
        }
    }
    
    /**
     * Animation des barres harmoniques
     */
    animateHarmonicBars(harmonics) {
        const harmonicIds = [1, 2, 3, 7, 11, 12];
        
        harmonicIds.forEach(id => {
            const progressBar = document.getElementById(`harmonic-${id}`);
            if (progressBar) {
                const intensity = harmonics[id - 1] || 0;
                
                // Animation selon l'intensité
                if (intensity > 0.8) {
                    progressBar.style.boxShadow = '0 0 10px currentColor';
                    progressBar.style.animation = 'pulse 0.5s ease-in-out';
                } else {
                    progressBar.style.boxShadow = 'none';
                    progressBar.style.animation = 'none';
                }
            }
        });
    }
    
    updateIntelligenceScore(score) {
        if (this.elements.intelligenceScore) {
            this.elements.intelligenceScore.textContent = score.toFixed(1);
        }
        
        // Mettre à jour métriques détaillées
        this.updateIntelligenceMetrics(score);
    }
    
    updateCoverageArea(area) {
        if (this.elements.coverageArea) {
            this.elements.coverageArea.textContent = `${(area * 100).toFixed(1)}%`;
        }
    }
    
    updateStatusReport(report) {
        // Traiter rapport statut global
        console.log('📊 Rapport statut reçu:', report);
        
        // Mettre à jour métriques intelligence si disponibles
        if (report.intelligence_metrics) {
            this.updateIntelligenceMetrics(
                report.intelligence_score || 0,
                report.intelligence_metrics
            );
        }
    }
    
    updateFromInitialState(data) {
        // Mettre à jour avec état initial
        if (data.drone_positions) {
            this.updateDronesTable();
        }
        
        if (data.intelligence_score !== undefined) {
            this.updateIntelligenceScore(data.intelligence_score);
        }
        
        if (data.coverage_area !== undefined) {
            this.updateCoverageArea(data.coverage_area);
        }
    }
    
    /**
     * Mise à jour métriques intelligence
     */
    updateIntelligenceMetrics(globalScore, metrics = {}) {
        // Score global
        this.updateProgressBar(
            this.elements.globalScoreBar,
            this.elements.globalScoreText,
            globalScore * 100,
            globalScore.toFixed(2)
        );
        
        // Métriques détaillées
        const coordination = metrics.coordination || globalScore * 0.9;
        const efficiency = metrics.efficiency || globalScore * 1.1;
        const adaptation = metrics.adaptation || globalScore * 0.8;
        
        this.updateProgressBar(
            this.elements.coordinationBar,
            this.elements.coordinationText,
            coordination * 100,
            coordination.toFixed(2)
        );
        
        this.updateProgressBar(
            this.elements.efficiencyBar,
            this.elements.efficiencyText,
            efficiency * 100,
            efficiency.toFixed(2)
        );
        
        this.updateProgressBar(
            this.elements.adaptationBar,
            this.elements.adaptationText,
            adaptation * 100,
            adaptation.toFixed(2)
        );
    }
    
    /**
     * Mise à jour barre de progression
     */
    updateProgressBar(barElement, textElement, percentage, text) {
        if (barElement) {
            barElement.style.setProperty('--progress', `${Math.min(percentage, 100)}%`);
        }
        
        if (textElement) {
            textElement.textContent = text;
        }
    }
    
    /**
     * Mise à jour table drones
     */
    updateDronesTable() {
        if (!this.elements.dronesTable) return;
        
        const tbody = this.elements.dronesTable.querySelector('tbody');
        if (!tbody) return;
        
        // Vider table
        tbody.innerHTML = '';
        
        // Ajouter lignes pour chaque drone
        const dronePositions = this.wsClient.getAllDronePositions();
        
        dronePositions.forEach(droneData => {
            const row = document.createElement('tr');
            
            const position = droneData.position;
            const positionStr = `(${position.x.toFixed(1)}, ${position.y.toFixed(1)}, ${position.z.toFixed(1)})`;
            
            row.innerHTML = `
                <td>${droneData.drone_id}</td>
                <td>${positionStr}</td>
                <td>85%</td>
                <td>🟢 Actif</td>
                <td>Exploration</td>
            `;
            
            tbody.appendChild(row);
        });
    }
    
    /**
     * Timer mission
     */
    startMissionTimer() {
        this.missionTimer = setInterval(() => {
            this.updateMissionTime();
        }, 1000);
    }
    
    stopMissionTimer() {
        if (this.missionTimer) {
            clearInterval(this.missionTimer);
            this.missionTimer = null;
        }
    }
    
    updateMissionTime() {
        if (!this.missionStartTime || !this.elements.missionTime) return;
        
        const elapsed = Date.now() - this.missionStartTime;
        const minutes = Math.floor(elapsed / 60000);
        const seconds = Math.floor((elapsed % 60000) / 1000);
        
        this.elements.missionTime.textContent = 
            `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
    }
    
    /**
     * Timer interface utilisateur
     */
    startUITimer() {
        setInterval(() => {
            this.updateUI();
        }, 5000); // Mise à jour toutes les 5 secondes
    }
    
    updateUI() {
        // Mise à jour générale interface
        const activeDrones = this.wsClient.getActiveDroneCount();
        if (this.elements.activeDrones) {
            this.elements.activeDrones.textContent = activeDrones;
        }
        
        const intelligenceScore = this.wsClient.getIntelligenceScore();
        if (this.elements.intelligenceScore && intelligenceScore) {
            this.elements.intelligenceScore.textContent = intelligenceScore.toFixed(1);
        }
        
        const coverageArea = this.wsClient.getCoverageArea();
        if (this.elements.coverageArea && coverageArea) {
            this.elements.coverageArea.textContent = `${(coverageArea * 100).toFixed(1)}%`;
        }
    }
    
    /**
     * Notifications utilisateur
     */
    showNotification(message, type = 'info') {
        // Créer notification temporaire
        const notification = document.createElement('div');
        notification.className = `notification ${type}`;
        notification.textContent = message;
        
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 12px 20px;
            border-radius: 8px;
            color: white;
            font-weight: 600;
            z-index: 1000;
            transition: all 0.3s ease;
        `;
        
        // Couleur selon type
        switch (type) {
            case 'success':
                notification.style.background = 'linear-gradient(135deg, #06ffa5, #00d4aa)';
                break;
            case 'warning':
                notification.style.background = 'linear-gradient(135deg, #ffb700, #ff8c00)';
                break;
            case 'danger':
                notification.style.background = 'linear-gradient(135deg, #ff006e, #c7004e)';
                break;
            default:
                notification.style.background = 'linear-gradient(135deg, #00b4d8, #0077b6)';
        }
        
        document.body.appendChild(notification);
        
        // Animation apparition
        setTimeout(() => {
            notification.style.transform = 'translateX(0)';
        }, 100);
        
        // Suppression automatique
        setTimeout(() => {
            notification.style.transform = 'translateX(100%)';
            setTimeout(() => {
                document.body.removeChild(notification);
            }, 300);
        }, 3000);
    }
    
    /**
     * Mode plein écran visualisation
     */
    toggleFullscreenVisualization() {
        const container = document.getElementById('threejs-container');
        if (!container) return;
        
        if (!document.fullscreenElement) {
            container.requestFullscreen().catch(err => {
                console.error('Erreur plein écran:', err);
            });
        } else {
            document.exitFullscreen();
        }
    }
}

// Export pour utilisation globale
window.DiamantDashboard = DiamantDashboard;