/**
 * DIAMANTS - Exploration Minimap avec Timer
 * Affiche la carte d'exploration avec indicateur de temps
 */

export class ExplorationMinimap {
    constructor(canvasId = 'minimap_canvas') {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas?.getContext('2d');
        
        // Configuration
        this.config = {
            gridSize: 40,           // Résolution de la grille (plus fin)
            zoneSize: 80,           // Taille de la zone en mètres
            updateInterval: 100,    // ms entre updates
            showHeatmap: true,
            showTrails: true
        };
        
        // État exploration
        this.explorationGrid = this.createGrid();
        this.explorationStartTime = null;
        this.totalCoverage = 0;
        this.dronePositions = new Map();
        this.droneTrails = new Map();
        
        // Timer
        this.timerElement = null;
        this.isRunning = false;
        
        this.init();
    }
    
    init() {
        if (!this.canvas) {
            console.warn('⚠️ Canvas minimap non trouvé');
            return;
        }
        
        // Créer l'élément timer
        this.createTimerElement();
        
        // Démarrer la boucle de rendu
        this.startRenderLoop();
        
        // Exposer globalement
        window.DIAMANTS_MINIMAP = this;
        
        console.log('🗺️ Exploration Minimap initialisée');
    }
    
    createGrid() {
        const grid = [];
        for (let i = 0; i < this.config.gridSize; i++) {
            grid[i] = [];
            for (let j = 0; j < this.config.gridSize; j++) {
                grid[i][j] = 0; // 0 = inexploré, 1 = complètement exploré
            }
        }
        return grid;
    }
    
    createTimerElement() {
        const minimap = document.getElementById('minimap');
        if (!minimap) return;
        
        // Créer container timer
        let timerContainer = document.getElementById('exploration-timer');
        if (!timerContainer) {
            timerContainer = document.createElement('div');
            timerContainer.id = 'exploration-timer';
            timerContainer.style.cssText = `
                display: flex;
                justify-content: space-between;
                align-items: center;
                padding: 4px 8px;
                background: rgba(0,0,0,0.6);
                border-radius: 4px;
                margin-top: 6px;
                font-size: 11px;
            `;
            timerContainer.innerHTML = `
                <span style="color: #00FFFF;">⏱️ <span id="exploration-time">00:00</span></span>
                <span style="color: #00FF88;">📊 <span id="exploration-percent">0%</span></span>
            `;
            minimap.appendChild(timerContainer);
        }
        
        this.timerElement = document.getElementById('exploration-time');
        this.percentElement = document.getElementById('exploration-percent');
    }
    
    /**
     * Démarrer le timer d'exploration
     */
    startExploration() {
        this.explorationStartTime = Date.now();
        this.isRunning = true;
        this.explorationGrid = this.createGrid(); // Reset
        console.log('🚀 Exploration démarrée');
    }
    
    /**
     * Arrêter le timer
     */
    stopExploration() {
        this.isRunning = false;
        const elapsed = this.getElapsedTime();
        console.log(`⏹️ Exploration arrêtée: ${elapsed} - Couverture: ${this.totalCoverage.toFixed(1)}%`);
    }
    
    /**
     * Obtenir le temps écoulé formaté
     */
    getElapsedTime() {
        if (!this.explorationStartTime) return '00:00';
        
        const elapsed = Date.now() - this.explorationStartTime;
        const minutes = Math.floor(elapsed / 60000);
        const seconds = Math.floor((elapsed % 60000) / 1000);
        return `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
    }
    
    /**
     * Mettre à jour la position d'un drone
     */
    updateDronePosition(droneId, position) {
        if (!position) return;
        
        // Stocker position
        this.dronePositions.set(droneId, { ...position, timestamp: Date.now() });
        
        // Ajouter au trail
        if (!this.droneTrails.has(droneId)) {
            this.droneTrails.set(droneId, []);
        }
        const trail = this.droneTrails.get(droneId);
        trail.push({ x: position.x, z: position.z });
        if (trail.length > 50) trail.shift(); // Limite trail
        
        // Marquer zone comme explorée
        this.markExplored(position.x, position.z);
    }
    
    /**
     * Marquer une zone comme explorée
     */
    markExplored(x, z) {
        // Sync zone size from doctrine if available
        if (window.DIAMANTS_DOCTRINE) {
            const dz = window.DIAMANTS_DOCTRINE.zoneParams;
            this.config.zoneSize = Math.max(dz.sizeX, dz.sizeZ);
        }

        const gridX = Math.floor((x + this.config.zoneSize / 2) / this.config.zoneSize * this.config.gridSize);
        const gridZ = Math.floor((z + this.config.zoneSize / 2) / this.config.zoneSize * this.config.gridSize);
        
        if (gridX >= 0 && gridX < this.config.gridSize && gridZ >= 0 && gridZ < this.config.gridSize) {
            // Augmenter progressivement l'exploration
            const current = this.explorationGrid[gridX][gridZ];
            if (current < 1) {
                this.explorationGrid[gridX][gridZ] = Math.min(1, current + 0.1);
            }
            
            // Explorer aussi les cellules voisines (vision du drone)
            for (let dx = -1; dx <= 1; dx++) {
                for (let dz = -1; dz <= 1; dz++) {
                    const nx = gridX + dx;
                    const nz = gridZ + dz;
                    if (nx >= 0 && nx < this.config.gridSize && nz >= 0 && nz < this.config.gridSize) {
                        const neighborCurrent = this.explorationGrid[nx][nz];
                        if (neighborCurrent < 1) {
                            this.explorationGrid[nx][nz] = Math.min(1, neighborCurrent + 0.03);
                        }
                    }
                }
            }
        }
    }
    
    /**
     * Calculer la couverture totale
     */
    calculateCoverage() {
        let total = 0;
        let explored = 0;
        
        for (let i = 0; i < this.config.gridSize; i++) {
            for (let j = 0; j < this.config.gridSize; j++) {
                total++;
                explored += this.explorationGrid[i][j];
            }
        }
        
        this.totalCoverage = total > 0 ? (explored / total) * 100 : 0;
        return this.totalCoverage;
    }
    
    /**
     * Boucle de rendu
     */
    startRenderLoop() {
        const render = () => {
            this.render();
            requestAnimationFrame(render);
        };
        render();
    }
    
    /**
     * Rendu de la minimap
     */
    render() {
        if (!this.ctx) return;

        // Sync zone size from doctrine dynamically
        if (window.DIAMANTS_DOCTRINE) {
            const dz = window.DIAMANTS_DOCTRINE.zoneParams;
            this.config.zoneSize = Math.max(dz.sizeX, dz.sizeZ);
        }

        // Add margin so drones near edges stay visible (10% extra)
        const displayZone = this.config.zoneSize * 1.1;
        
        const width = this.canvas.width;
        const height = this.canvas.height;
        
        // Grid cell sizes in world and canvas space
        const worldCellSize = this.config.zoneSize / this.config.gridSize;
        const cellW = (worldCellSize / displayZone) * width;
        const cellH = (worldCellSize / displayZone) * height;
        // Offset: grid covers [-zoneSize/2, zoneSize/2] inside displayZone
        const gridOffsetX = ((displayZone - this.config.zoneSize) / 2 / displayZone) * width;
        const gridOffsetY = ((displayZone - this.config.zoneSize) / 2 / displayZone) * height;
        
        // Fond
        this.ctx.fillStyle = '#0a1628';
        this.ctx.fillRect(0, 0, width, height);
        
        // Grille d'exploration avec couleurs
        for (let i = 0; i < this.config.gridSize; i++) {
            for (let j = 0; j < this.config.gridSize; j++) {
                const coverage = this.explorationGrid[i][j];
                
                if (coverage > 0) {
                    const x = gridOffsetX + i * cellW;
                    const y = gridOffsetY + j * cellH;
                    
                    // Couleur selon niveau d'exploration
                    let color;
                    if (coverage >= 0.9) {
                        // Vert - complètement exploré
                        color = `rgba(46, 204, 113, ${0.4 + coverage * 0.4})`;
                    } else if (coverage >= 0.6) {
                        // Jaune-vert - bien exploré
                        color = `rgba(241, 196, 15, ${0.3 + coverage * 0.4})`;
                    } else if (coverage >= 0.3) {
                        // Orange - partiellement exploré
                        color = `rgba(230, 126, 34, ${0.2 + coverage * 0.3})`;
                    } else {
                        // Rouge clair - légèrement exploré
                        color = `rgba(231, 76, 60, ${0.1 + coverage * 0.2})`;
                    }
                    
                    this.ctx.fillStyle = color;
                    this.ctx.fillRect(x, y, cellW - 1, cellH - 1);
                }
            }
        }
        
        // Grille de référence
        this.ctx.strokeStyle = 'rgba(0, 255, 136, 0.15)';
        this.ctx.lineWidth = 0.5;
        for (let i = 0; i <= this.config.gridSize; i += 4) {
            const px = gridOffsetX + i * cellW;
            const py = gridOffsetY + i * cellH;
            this.ctx.beginPath();
            this.ctx.moveTo(px, 0);
            this.ctx.lineTo(px, height);
            this.ctx.moveTo(0, py);
            this.ctx.lineTo(width, py);
            this.ctx.stroke();
        }
        
        // Trails des drones
        if (this.config.showTrails) {
            this.droneTrails.forEach((trail, droneId) => {
                if (trail.length < 2) return;
                
                this.ctx.strokeStyle = this.getDroneColor(droneId, 0.4);
                this.ctx.lineWidth = 1;
                this.ctx.beginPath();
                
                for (let i = 0; i < trail.length; i++) {
                    const x = ((trail[i].x + displayZone / 2) / displayZone) * width;
                    const y = ((trail[i].z + displayZone / 2) / displayZone) * height;
                    
                    if (i === 0) {
                        this.ctx.moveTo(x, y);
                    } else {
                        this.ctx.lineTo(x, y);
                    }
                }
                this.ctx.stroke();
            });
        }
        
        // Drones
        this.dronePositions.forEach((pos, droneId) => {
            const x = ((pos.x + displayZone / 2) / displayZone) * width;
            const y = ((pos.z + displayZone / 2) / displayZone) * height;
            
            // Halo
            this.ctx.beginPath();
            this.ctx.arc(x, y, 6, 0, Math.PI * 2);
            this.ctx.fillStyle = this.getDroneColor(droneId, 0.3);
            this.ctx.fill();
            
            // Corps
            this.ctx.beginPath();
            this.ctx.arc(x, y, 3, 0, Math.PI * 2);
            this.ctx.fillStyle = this.getDroneColor(droneId, 1);
            this.ctx.fill();
            
            // ID
            this.ctx.fillStyle = '#fff';
            this.ctx.font = 'bold 7px Arial';
            this.ctx.textAlign = 'center';
            this.ctx.fillText(droneId.toString(), x, y + 2);
        });
        
        // Zone boundary (inner rectangle showing actual exploration limits)
        this.ctx.strokeStyle = 'rgba(0, 255, 136, 0.35)';
        this.ctx.lineWidth = 1;
        this.ctx.setLineDash([4, 4]);
        this.ctx.strokeRect(gridOffsetX, gridOffsetY,
            (this.config.zoneSize / displayZone) * width,
            (this.config.zoneSize / displayZone) * height);
        this.ctx.setLineDash([]);

        // Bordure
        this.ctx.strokeStyle = '#00ff88';
        this.ctx.lineWidth = 2;
        this.ctx.strokeRect(1, 1, width - 2, height - 2);
        
        // Update timer et pourcentage
        this.updateUI();
    }
    
    /**
     * Obtenir couleur selon drone ID
     */
    getDroneColor(droneId, alpha = 1) {
        const colors = [
            `rgba(0, 255, 136, ${alpha})`,   // Vert
            `rgba(0, 200, 255, ${alpha})`,   // Cyan
            `rgba(255, 170, 0, ${alpha})`,   // Orange
            `rgba(255, 100, 150, ${alpha})`, // Rose
            `rgba(150, 100, 255, ${alpha})`, // Violet
            `rgba(255, 255, 0, ${alpha})`,   // Jaune
            `rgba(100, 255, 200, ${alpha})`, // Turquoise
            `rgba(255, 150, 100, ${alpha})`  // Pêche
        ];
        return colors[droneId % colors.length];
    }
    
    /**
     * Mise à jour UI (timer + pourcentage)
     */
    updateUI() {
        if (this.timerElement && this.isRunning) {
            this.timerElement.textContent = this.getElapsedTime();
        }
        
        if (this.percentElement) {
            const coverage = this.calculateCoverage();
            this.percentElement.textContent = `${coverage.toFixed(1)}%`;
            
            // Couleur selon progression
            if (coverage >= 85) {
                this.percentElement.style.color = '#00FF88';
            } else if (coverage >= 50) {
                this.percentElement.style.color = '#FFD700';
            } else {
                this.percentElement.style.color = '#FF6666';
            }
        }
    }
    
    /**
     * Reset complet
     */
    reset() {
        this.explorationGrid = this.createGrid();
        this.explorationStartTime = null;
        this.isRunning = false;
        this.dronePositions.clear();
        this.droneTrails.clear();
        this.totalCoverage = 0;
        console.log('🔄 Minimap réinitialisée');
    }
    
    /**
     * Obtenir les statistiques d'exploration
     */
    getStats() {
        return {
            coverage: this.totalCoverage,
            elapsedTime: this.getElapsedTime(),
            elapsedMs: this.explorationStartTime ? Date.now() - this.explorationStartTime : 0,
            activeDrones: this.dronePositions.size,
            isRunning: this.isRunning
        };
    }
}

// Auto-init si le canvas existe
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
        new ExplorationMinimap();
    });
} else {
    new ExplorationMinimap();
}

export default ExplorationMinimap;
