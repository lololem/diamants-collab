/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Discovery Minimap (Pixel Coverage)
 * ═══════════════════════════════════════════════
 * Minimap de découverte pixel par pixel : chaque cellule passe
 * progressivement de inexploré (sombre) à exploré (cyan vif)
 * quand un drone passe à proximité. Les voisins reçoivent un
 * splash partiel, donnant un effet de "fog of war" qui se lève.
 *
 * Schéma BLEU (distinct de la carte stigmergie vert/jaune/rouge) :
 *   Bleu foncé   → légèrement exploré
 *   Bleu moyen   → partiellement exploré
 *   Cyan         → bien exploré
 *   Blanc-cyan   → complètement exploré
 */

const DRONE_COLORS_RGBA = [
    [0, 255, 136],   // Vert
    [0, 200, 255],   // Cyan
    [255, 170, 0],   // Orange
    [255, 100, 150], // Rose
    [150, 100, 255], // Violet
    [255, 255, 0],   // Jaune
    [100, 255, 200], // Turquoise
    [255, 150, 100], // Pêche
];

/** Extract numeric index from drone ID (string or number). */
function _droneIdx(id) {
    if (typeof id === 'number') return id;
    const m = String(id).match(/(\d+)/);
    return m ? parseInt(m[1], 10) - 1 : 0;  // "crazyflie_01" → 0
}

export class DiscoveryMinimap {
    constructor(canvasId = 'discovery_canvas') {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas?.getContext('2d');

        // Configuration
        this.config = {
            gridSize: 60,           // Résolution de la grille (60×60 = 2m per cell for 120m zone)
            zoneSize: 120,          // Taille de la zone en mètres (max from doctrine)
            showTrails: true,
        };

        // ── Adaptive zoom ──────────────────────────────────────────
        this._adaptiveZone = 40;       // current display zone (smoothed)

        // État exploration
        this.explorationGrid = this._createGrid();
        this.explorationStartTime = null;
        this.totalCoverage = 0;
        this.isRunning = false;

        // Drones
        this.dronePositions = new Map();
        this.droneTrails = new Map();

        // Timer elements (appended inside #discovery-map)
        this.timerElement = null;
        this.percentElement = null;

        this._init();
    }

    // ════════════════════════════════════════════════════════════════
    // INIT
    // ════════════════════════════════════════════════════════════════

    _init() {
        if (!this.canvas) {
            console.warn('⚠️ Discovery canvas non trouvé');
            return;
        }

        this._createTimerElement();
        this._startRenderLoop();
        this.canvas._minimapInstance = this;

        window.DIAMANTS_DISCOVERY = this;
        console.log('🟩 Discovery Minimap initialisée (pixel coverage)');
    }

    _createGrid() {
        const grid = [];
        for (let i = 0; i < this.config.gridSize; i++) {
            grid[i] = new Float32Array(this.config.gridSize); // all zeros
        }
        return grid;
    }

    _createTimerElement() {
        const panel = document.getElementById('discovery-map');
        if (!panel) return;

        let container = document.getElementById('discovery-timer');
        if (!container) {
            container = document.createElement('div');
            container.id = 'discovery-timer';
            container.style.cssText = `
                display: flex;
                justify-content: space-between;
                align-items: center;
                padding: 5px 10px;
                background: rgba(0,0,0,0.6);
                border-radius: 4px;
                margin: 0 6px 4px;
                font-size: calc(var(--minimap-fs, 12px) * 1.083);
            `;
            container.innerHTML = `
                <span style="color: #00FFFF;">⏱️ <span id="discovery-time">00:00</span></span>
                <span style="color: #00FF88;">📊 <span id="discovery-percent">0%</span></span>
            `;
            panel.appendChild(container);
        }

        this.timerElement = document.getElementById('discovery-time');
        this.percentElement = document.getElementById('discovery-percent');
    }

    // ════════════════════════════════════════════════════════════════
    // PUBLIC API
    // ════════════════════════════════════════════════════════════════

    /**
     * Update drone position and mark cells explored.
     * @param {number} droneId
     * @param {{x:number, y:number, z:number}} position
     */
    updateDronePosition(droneId, position) {
        if (!position) return;

        this.dronePositions.set(droneId, { ...position, timestamp: Date.now() });

        // Trail
        if (!this.droneTrails.has(droneId)) {
            this.droneTrails.set(droneId, []);
        }
        const trail = this.droneTrails.get(droneId);
        trail.push({ x: position.x, z: position.z });
        if (trail.length > 50) trail.shift();

        // Mark explored
        this._markExplored(position.x, position.z);
    }

    startExploration() {
        this.explorationStartTime = Date.now();
        this.isRunning = true;
        this.explorationGrid = this._createGrid();
        console.log('🟩 Discovery: exploration démarrée');
    }

    stopExploration() {
        this.isRunning = false;
        console.log(`🟩 Discovery: arrêtée — ${this.totalCoverage.toFixed(1)}%`);
    }

    reset() {
        this.explorationGrid = this._createGrid();
        this.explorationStartTime = null;
        this.isRunning = false;
        this.dronePositions.clear();
        this.droneTrails.clear();
        this.totalCoverage = 0;
    }

    getStats() {
        return {
            coverage: this.totalCoverage,
            elapsedTime: this._getElapsedTime(),
            elapsedMs: this.explorationStartTime ? Date.now() - this.explorationStartTime : 0,
            activeDrones: this.dronePositions.size,
            isRunning: this.isRunning,
        };
    }

    // ════════════════════════════════════════════════════════════════
    // JOURNAL SYNC (collaborative shared exploration data)
    // ════════════════════════════════════════════════════════════════

    /**
     * Sync exploration grid from the StigmergyEngine's shared journal.
     * This ensures ALL drones' exploration data is reflected, not just
     * what was received via updateDronePosition().
     */
    _syncFromJournal() {
        // ── Primary: sync from engine's visitedCells (permanent, no flicker) ──
        const visitedCells = window.DIAMANTS_VISITED_CELLS;
        const cellSize = window.DIAMANTS_CELL_SIZE || 2;
        if (visitedCells && visitedCells.size > 0) {
            // Clear and re-fill from visitedCells each frame (grid maps adaptive zone)
            const gs = this.config.gridSize;
            const zs = this._adaptiveZone;
            // Reset grid before re-fill
            for (let i = 0; i < gs; i++) for (let j = 0; j < gs; j++) this.explorationGrid[i][j] = 0;
            const cellsPerEngine = cellSize / (zs / gs); // how many discovery cells per engine cell
            for (const key of visitedCells) {
                const [cx, cz] = key.split(',').map(Number);
                const worldX = (cx + 0.5) * cellSize;
                const worldZ = (cz + 0.5) * cellSize;
                const gx = Math.floor((worldX + zs / 2) / zs * gs);
                const gz = Math.floor((worldZ + zs / 2) / zs * gs);
                if (gx >= 0 && gx < gs && gz >= 0 && gz < gs) {
                    this.explorationGrid[gx][gz] = 1.0;
                    // Mark 5×5 neighbors for seamless gap-free coverage
                    for (let dx = -2; dx <= 2; dx++) {
                        for (let dz = -2; dz <= 2; dz++) {
                            if (dx === 0 && dz === 0) continue;
                            const nx = gx + dx, nz = gz + dz;
                            if (nx >= 0 && nx < gs && nz >= 0 && nz < gs) {
                                const dist = Math.max(Math.abs(dx), Math.abs(dz));
                                const boost = dist <= 1 ? 0.95 : 0.85;
                                this.explorationGrid[nx][nz] = Math.max(this.explorationGrid[nx][nz], boost);
                            }
                        }
                    }
                }
            }
            return; // visitedCells is authoritative — skip journal
        }

        // ── Fallback: sync from stigmergy journal (legacy) ──
        const engine = window.DIAMANTS_STIGMERGY_INSTANCE;
        if (!engine || typeof engine.getExplorationJournal !== 'function') return;

        const journal = engine.getExplorationJournal();
        if (!journal || journal.size === 0) return;

        const gs = this.config.gridSize;
        const zs = this._adaptiveZone;

        for (const [key, entry] of journal) {
            const [jx, jz] = key.split(',').map(Number);
            // Convert journal cell coords to discovery grid coords
            const worldX = jx * 2 + 1; // journal cellSize = 2m, center
            const worldZ = jz * 2 + 1;
            const gx = Math.floor((worldX + zs / 2) / zs * gs);
            const gz = Math.floor((worldZ + zs / 2) / zs * gs);

            if (gx >= 0 && gx < gs && gz >= 0 && gz < gs) {
                // Journal provides a permanent boost (stronger than per-frame deposit)
                const journalBoost = Math.min(1, entry.visits * 0.15);
                if (this.explorationGrid[gx][gz] < journalBoost) {
                    this.explorationGrid[gx][gz] = journalBoost;
                }
            }
        }
    }

    // ════════════════════════════════════════════════════════════════
    // EXPLORATION MARKING
    // ════════════════════════════════════════════════════════════════

    _markExplored(x, z) {
        // Sync max zone from doctrine
        if (window.DIAMANTS_DOCTRINE) {
            const dz = window.DIAMANTS_DOCTRINE.zoneParams;
            this.config.zoneSize = Math.max(dz.sizeX, dz.sizeZ);
        }

        // Use adaptive zone for grid mapping (same as _render)
        const gs = this.config.gridSize;
        const zs = this._adaptiveZone;
        const gx = Math.floor((x + zs / 2) / zs * gs);
        const gz = Math.floor((z + zs / 2) / zs * gs);

        if (gx >= 0 && gx < gs && gz >= 0 && gz < gs) {
            // Direct cell — strong fill
            if (this.explorationGrid[gx][gz] < 1) {
                this.explorationGrid[gx][gz] = Math.min(1, this.explorationGrid[gx][gz] + 0.15);
            }

            // Neighbor cells — splash (drone's effective vision range ~4m = 2 grid cells)
            for (let dx = -2; dx <= 2; dx++) {
                for (let dz = -2; dz <= 2; dz++) {
                    if (dx === 0 && dz === 0) continue;
                    const nx = gx + dx;
                    const nz = gz + dz;
                    if (nx >= 0 && nx < gs && nz >= 0 && nz < gs) {
                        const ring = Math.max(Math.abs(dx), Math.abs(dz));
                        const splash = ring === 1 ? 0.06 : 0.02;
                        if (this.explorationGrid[nx][nz] < 1) {
                            this.explorationGrid[nx][nz] = Math.min(1, this.explorationGrid[nx][nz] + splash);
                        }
                    }
                }
            }
        }
    }

    // ════════════════════════════════════════════════════════════════
    // STATS
    // ════════════════════════════════════════════════════════════════

    _calculateCoverage() {
        let total = 0, explored = 0;
        const gs = this.config.gridSize;
        for (let i = 0; i < gs; i++) {
            for (let j = 0; j < gs; j++) {
                total++;
                explored += this.explorationGrid[i][j];
            }
        }
        this.totalCoverage = total > 0 ? (explored / total) * 100 : 0;
        return this.totalCoverage;
    }

    _getElapsedTime() {
        if (!this.explorationStartTime) return '00:00';
        const elapsed = Date.now() - this.explorationStartTime;
        const m = Math.floor(elapsed / 60000);
        const s = Math.floor((elapsed % 60000) / 1000);
        return `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    }

    // ════════════════════════════════════════════════════════════════
    // RENDERING
    // ════════════════════════════════════════════════════════════════

    _startRenderLoop() {
        this._lastRender = 0;
        const loop = () => {
            requestAnimationFrame(loop);
            const now = performance.now();
            if (now - this._lastRender < 80) return; // ~12 FPS (same as exploration minimap)
            this._lastRender = now;
            this._render();
        };
        loop();
    }

    _render() {
        if (!this.ctx) return;

        // Sync max zone from doctrine
        if (window.DIAMANTS_DOCTRINE) {
            const dz = window.DIAMANTS_DOCTRINE.zoneParams;
            this.config.zoneSize = Math.max(dz.sizeX, dz.sizeZ);
        }

        // ═══ SYNC FROM SHARED JOURNAL (collaborative data) ═══
        this._syncFromJournal();

        // ── Adaptive zoom: compute display zone from drone extent ──
        const maxConfigZone = this.config.zoneSize;
        let rawExtent = 20; // minimum half-extent
        this.dronePositions.forEach((pos) => {
            const ex = Math.abs(pos.x);
            const ez = Math.abs(pos.z);
            const m = Math.max(ex, ez);
            if (m > rawExtent) rawExtent = m;
        });
        const targetZone = Math.min(maxConfigZone, Math.max(40, rawExtent * 2 * 1.3));
        // Smooth transition (only grow)
        if (targetZone > this._adaptiveZone) {
            this._adaptiveZone += (targetZone - this._adaptiveZone) * 0.08;
            if (this._adaptiveZone > targetZone - 1) this._adaptiveZone = targetZone;
        }
        const adaptiveZone = this._adaptiveZone;
        const displayZone = adaptiveZone * 1.1;
        const w = this.canvas.width;
        const h = this.canvas.height;
        const gs = this.config.gridSize;
        const worldCellSize = adaptiveZone / gs;
        const cellW = (worldCellSize / displayZone) * w;
        const cellH = (worldCellSize / displayZone) * h;
        const gridOffsetX = ((displayZone - adaptiveZone) / 2 / displayZone) * w;
        const gridOffsetY = ((displayZone - adaptiveZone) / 2 / displayZone) * h;

        // ── Background ─────────────────────────────────────────────
        this.ctx.fillStyle = '#0a1628';
        this.ctx.fillRect(0, 0, w, h);

        // ── Exploration grid (pixel fog-of-war) ────────────────────
        for (let i = 0; i < gs; i++) {
            for (let j = 0; j < gs; j++) {
                const coverage = this.explorationGrid[i][j];
                if (coverage <= 0) continue;

                const x = gridOffsetX + i * cellW;
                const y = gridOffsetY + j * cellH;

                let color;
                if (coverage >= 0.9) {
                    color = `rgba(140, 230, 255, ${0.45 + coverage * 0.4})`; // bright cyan
                } else if (coverage >= 0.6) {
                    color = `rgba(60, 170, 220, ${0.35 + coverage * 0.35})`; // medium blue
                } else if (coverage >= 0.3) {
                    color = `rgba(30, 100, 180, ${0.25 + coverage * 0.3})`; // darker blue
                } else {
                    color = `rgba(20, 60, 140, ${0.15 + coverage * 0.25})`; // dark blue
                }

                this.ctx.fillStyle = color;
                this.ctx.fillRect(x, y, cellW - 1, cellH - 1);
            }
        }

        // ── Reference grid ─────────────────────────────────────────
        this.ctx.strokeStyle = 'rgba(80, 180, 255, 0.12)';
        this.ctx.lineWidth = 0.5;
        for (let i = 0; i <= gs; i += 4) {
            const px = gridOffsetX + i * cellW;
            const py = gridOffsetY + i * cellH;
            this.ctx.beginPath();
            this.ctx.moveTo(px, 0); this.ctx.lineTo(px, h);
            this.ctx.moveTo(0, py); this.ctx.lineTo(w, py);
            this.ctx.stroke();
        }

        // ── Drone trails ───────────────────────────────────────────
        if (this.config.showTrails) {
            this.droneTrails.forEach((trail, droneId) => {
                if (trail.length < 2) return;
                const [r, g, b] = DRONE_COLORS_RGBA[_droneIdx(droneId) % DRONE_COLORS_RGBA.length];
                this.ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.4)`;
                this.ctx.lineWidth = 1;
                this.ctx.beginPath();
                for (let i = 0; i < trail.length; i++) {
                    const x = ((trail[i].x + displayZone / 2) / displayZone) * w;
                    const y = ((trail[i].z + displayZone / 2) / displayZone) * h;
                    if (i === 0) this.ctx.moveTo(x, y);
                    else this.ctx.lineTo(x, y);
                }
                this.ctx.stroke();
            });
        }

        // ── Drone dots ─────────────────────────────────────────────
        this.dronePositions.forEach((pos, droneId) => {
            const x = ((pos.x + displayZone / 2) / displayZone) * w;
            const y = ((pos.z + displayZone / 2) / displayZone) * h;
            const [r, g, b] = DRONE_COLORS_RGBA[_droneIdx(droneId) % DRONE_COLORS_RGBA.length];

            // Halo
            this.ctx.beginPath();
            this.ctx.arc(x, y, 6, 0, Math.PI * 2);
            this.ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.3)`;
            this.ctx.fill();

            // Body
            this.ctx.beginPath();
            this.ctx.arc(x, y, 3, 0, Math.PI * 2);
            this.ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 1)`;
            this.ctx.fill();

            // ID
            this.ctx.fillStyle = '#fff';
            this.ctx.font = 'bold 7px Arial';
            this.ctx.textAlign = 'center';
            this.ctx.fillText(droneId.toString(), x, y + 2);
        });

        // ── Zone boundary ──────────────────────────────────────────
        this.ctx.strokeStyle = 'rgba(80, 180, 255, 0.35)';
        this.ctx.lineWidth = 1;
        this.ctx.setLineDash([4, 4]);
        this.ctx.strokeRect(gridOffsetX, gridOffsetY,
            (adaptiveZone / displayZone) * w,
            (adaptiveZone / displayZone) * h);
        this.ctx.setLineDash([]);

        // ── Adaptive zone indicator ────────────────────────────────
        this.ctx.fillStyle = 'rgba(80, 180, 255, 0.6)';
        this.ctx.font = 'bold 8px monospace';
        this.ctx.textAlign = 'right';
        this.ctx.fillText(`${Math.round(adaptiveZone)}m / ${Math.round(maxConfigZone)}m`, w - 4, h - 4);

        // ── Coverage legend (bottom-left, blue scheme) ──────────
        const legendY = h - 14;
        this.ctx.font = '7px monospace';
        this.ctx.textAlign = 'left';
        this.ctx.fillStyle = 'rgba(20, 60, 140, 0.9)';
        this.ctx.fillRect(4, legendY - 4, 8, 8);
        this.ctx.fillStyle = '#4488cc';
        this.ctx.fillText('Low', 15, legendY + 2);
        this.ctx.fillStyle = 'rgba(60, 170, 220, 0.9)';
        this.ctx.fillRect(55, legendY - 4, 8, 8);
        this.ctx.fillStyle = '#55aadd';
        this.ctx.fillText('Med', 66, legendY + 2);
        this.ctx.fillStyle = 'rgba(140, 230, 255, 0.9)';
        this.ctx.fillRect(106, legendY - 4, 8, 8);
        this.ctx.fillStyle = '#8ce6ff';
        this.ctx.fillText('Full', 117, legendY + 2);

        // ── Border (blue theme) ───────────────────────────────────
        this.ctx.strokeStyle = '#4488cc';
        this.ctx.lineWidth = 2;
        this.ctx.strokeRect(1, 1, w - 2, h - 2);

        // ── UI ─────────────────────────────────────────────────────
        this._updateUI();
    }

    _updateUI() {
        if (this.timerElement && this.isRunning) {
            this.timerElement.textContent = this._getElapsedTime();
        }

        if (this.percentElement) {
            const cov = this._calculateCoverage();
            this.percentElement.textContent = `${cov.toFixed(1)}%`;
            this.percentElement.style.color = cov >= 85 ? '#8ce6ff'
                                             : cov >= 50 ? '#55aadd'
                                             : '#4488cc';
        }
    }

    // ════════════════════════════════════════════════════════════════
    // CLEANUP
    // ════════════════════════════════════════════════════════════════

    dispose() {
        // nothing critical to clean
    }
}

export default DiscoveryMinimap;
