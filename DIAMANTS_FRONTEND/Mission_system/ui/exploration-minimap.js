/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Exploration Minimap (Stigmergy Pheromone Field)
 * 
 * Vraie visualisation stigmergique avec :
 *   - Grid de pheromones persistante (dépôt + diffusion + évaporation)
 *   - Effet glow radial comme des vrais pheromones de fourmis
 *   - Dégradé vert→jaune→blanc selon l'intensité
 *   - Positions des drones avec heading + ID
 *   - Waypoints cibles
 *   - Timer et % couverture
 *
 * AUCUNE DETECTION ICI - la détection est sur la SITAC.
 * Modèle stigmergique basé sur stigmergy-engine.js
 */

const DRONE_COLORS = [
    '#00FF88', '#00C8FF', '#FFAA00', '#FF6496',
    '#9664FF', '#FFFF00', '#64FFC8', '#FF9664',
];

function _droneIdx(id) {
    if (typeof id === 'number') return id;
    const m = String(id).match(/(\d+)/);
    return m ? parseInt(m[1], 10) - 1 : 0;
}

// Pheromone grid constants (matches stigmergy-engine.js)
const PHERO_GRID = 80;              // cells per side (80x80)
const PHERO_RES = 1.5;              // meters per cell
const PHERO_HALF = (PHERO_GRID * PHERO_RES) / 2;  // 60m half-extent
const PHERO_DEPOSIT = 8.0;          // deposit per frame when drone is on cell
const PHERO_DEPOSIT_RADIUS = 3;     // deposit radius in cells (~4.5m)
const PHERO_EVAP_RATE = 0.0004;     // evaporation per frame (~0.005/sec at 12FPS, matches real engine)
const PHERO_DIFFUSION_RATE = 0.08;  // diffusion to neighbors
const PHERO_MAX = 100;              // max intensity

export class ExplorationMinimap {
    constructor(canvasId = 'minimap_canvas') {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas?.getContext('2d');

        this.config = {
            gridSize: 100,
            zoneSize: 120,
            updateInterval: 80,
            showWaypoints: true,
        };

        this.explorationStartTime = null;
        this.totalCoverage = 0;
        this.isRunning = false;

        this._adaptiveZone = 40;
        this._maxExtent = 0;

        /** @type {Map<number, {x:number, y:number, z:number, heading:number, waypoint:{x:number,z:number}|null}>} */
        this.dronePositions = new Map();

        // === PHEROMONE GRID (persistent, with diffusion + evaporation) ===
        this._pheroGrid = new Float32Array(PHERO_GRID * PHERO_GRID);
        this._pheroScratch = new Float32Array(PHERO_GRID * PHERO_GRID);
        this._diffuseCounter = 0;

        // Pheromone image buffer (RGBA) for fast rendering
        this._pheroImageData = null;

        this._snapInterval = 200;
        this._lastSnapTime = 0;
        this._journal = null;
        this._journalCoverage = 0;

        this.timerElement = null;
        this.percentElement = null;
        this._lastRender = 0;

        this._init();
    }

    _init() {
        if (!this.canvas) {
            console.warn('Canvas minimap non trouve');
            return;
        }
        this._offscreen = document.createElement('canvas');
        this._offscreen.width = this.canvas.width;
        this._offscreen.height = this.canvas.height;
        this._offCtx = this._offscreen.getContext('2d');
        this.canvas._minimapInstance = this;

        this._createTimerElement();
        this._startRenderLoop();
        window.DIAMANTS_MINIMAP = this;
        console.log('Exploration Minimap initialisee (pheromone field)');
    }

    _createTimerElement() {
        const minimap = document.getElementById('minimap');
        if (!minimap) return;

        let timerContainer = document.getElementById('exploration-timer');
        if (!timerContainer) {
            timerContainer = document.createElement('div');
            timerContainer.id = 'exploration-timer';
            timerContainer.style.cssText = 'display:flex;justify-content:space-between;align-items:center;padding:5px 10px;background:rgba(0,0,0,0.6);border-radius:4px;margin-top:6px;font-size:calc(var(--minimap-fs, 12px) * 1.083);';
            timerContainer.innerHTML = '<span style="color:#00FFFF;"><span id="exploration-time">00:00</span></span><span style="color:#00FF88;"><span id="exploration-percent">0%</span></span>';
            minimap.appendChild(timerContainer);
        }
        this.timerElement = document.getElementById('exploration-time');
        this.percentElement = document.getElementById('exploration-percent');
    }

    // -- PUBLIC API --
    updateDronePosition(droneId, position, heading = 0, waypoint = null) {
        if (!position) return;
        this.dronePositions.set(droneId, {
            x: position.x, y: position.y, z: position.z,
            heading, waypoint, timestamp: Date.now(),
        });
    }

    startExploration() {
        this.explorationStartTime = Date.now();
        this.isRunning = true;
    }

    stopExploration() {
        this.isRunning = false;
    }

    reset() {
        this.explorationStartTime = null;
        this.isRunning = false;
        this.dronePositions.clear();
        this._pheroGrid.fill(0);
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

    // =========================================================================
    // PHEROMONE SIMULATION (deposit, diffuse, evaporate)
    // =========================================================================

    /** Deposit pheromone at world position (x, z) with Gaussian falloff radius */
    _deposit(wx, wz, intensity) {
        const gx0 = Math.floor((wx + PHERO_HALF) / PHERO_RES);
        const gz0 = Math.floor((wz + PHERO_HALF) / PHERO_RES);
        const R = PHERO_DEPOSIT_RADIUS;

        for (let dz = -R; dz <= R; dz++) {
            for (let dx = -R; dx <= R; dx++) {
                const gx = gx0 + dx;
                const gz = gz0 + dz;
                if (gx < 0 || gx >= PHERO_GRID || gz < 0 || gz >= PHERO_GRID) continue;
                const dist = Math.sqrt(dx * dx + dz * dz);
                if (dist > R + 0.5) continue;
                const falloff = 1 - dist / (R + 1);
                const idx = gz * PHERO_GRID + gx;
                this._pheroGrid[idx] = Math.min(PHERO_MAX, this._pheroGrid[idx] + intensity * falloff * falloff);
            }
        }
    }

    /** Evaporate all cells: value *= (1 - rate) */
    _evaporate() {
        for (let i = 0; i < this._pheroGrid.length; i++) {
            if (this._pheroGrid[i] > 0) {
                this._pheroGrid[i] *= (1 - PHERO_EVAP_RATE);
                if (this._pheroGrid[i] < 0.05) this._pheroGrid[i] = 0;
            }
        }
    }

    /** Diffuse pheromones to 8-neighbors (Gaussian-like, run every 3 frames) */
    _diffuse() {
        const g = this._pheroGrid;
        const s = this._pheroScratch;
        const N = PHERO_GRID;
        s.set(g);

        const rate = PHERO_DIFFUSION_RATE;
        // 8-neighbor: orthogonal=1.0, diagonal=0.707
        const totalW = 4 * 1.0 + 4 * 0.707; // 6.828
        const shareOrtho = rate / totalW;
        const shareDiag = rate * 0.707 / totalW;

        for (let z = 1; z < N - 1; z++) {
            for (let x = 1; x < N - 1; x++) {
                const idx = z * N + x;
                const val = g[idx];
                if (val < 0.1) continue;
                const give = val * rate;
                s[idx] -= give;
                // Orthogonal
                s[idx - 1] += val * shareOrtho;
                s[idx + 1] += val * shareOrtho;
                s[idx - N] += val * shareOrtho;
                s[idx + N] += val * shareOrtho;
                // Diagonal
                s[idx - N - 1] += val * shareDiag;
                s[idx - N + 1] += val * shareDiag;
                s[idx + N - 1] += val * shareDiag;
                s[idx + N + 1] += val * shareDiag;
            }
        }

        // Clamp
        for (let i = 0; i < s.length; i++) {
            this._pheroGrid[i] = Math.min(PHERO_MAX, Math.max(0, s[i]));
        }
    }

    /** Full pheromone tick: deposit for each drone, evaporate, diffuse every 3 frames */
    _tickPheromones() {
        // Deposit at each drone position
        this.dronePositions.forEach((pos) => {
            this._deposit(pos.x, pos.z, PHERO_DEPOSIT);
        });

        // Evaporate
        this._evaporate();

        // Diffuse every 3 frames (performance)
        this._diffuseCounter++;
        if (this._diffuseCounter >= 3) {
            this._diffuse();
            this._diffuseCounter = 0;
        }
    }

    // -- COVERAGE SYNC --
    _syncCoverage() {
        const now = performance.now();
        if (now - this._lastSnapTime < this._snapInterval) return;
        this._lastSnapTime = now;

        const engine = window.DIAMANTS_STIGMERGY_INSTANCE;
        if (window.DIAMANTS_VISITED_CELLS) {
            // Coverage computed in _render()
        } else if (engine && typeof engine.getMetrics === 'function') {
            const m = engine.getMetrics();
            if (m.explorationCoverage) this.totalCoverage = parseFloat(m.explorationCoverage) || 0;
            if (m.journalCoverage) this._journalCoverage = parseFloat(m.journalCoverage) || 0;
        }
    }

    // -- HELPERS --
    _getElapsedTime() {
        if (!this.explorationStartTime) return '00:00';
        const elapsed = Date.now() - this.explorationStartTime;
        const m = Math.floor(elapsed / 60000);
        const s = Math.floor((elapsed % 60000) / 1000);
        return m.toString().padStart(2, '0') + ':' + s.toString().padStart(2, '0');
    }

    _getDroneColor(id, alpha = 1) {
        const c = DRONE_COLORS[_droneIdx(id) % DRONE_COLORS.length];
        if (alpha >= 1) return c;
        const r = parseInt(c.slice(1, 3), 16);
        const g = parseInt(c.slice(3, 5), 16);
        const b = parseInt(c.slice(5, 7), 16);
        return 'rgba(' + r + ',' + g + ',' + b + ',' + alpha + ')';
    }

    // -- RENDER --
    _startRenderLoop() {
        const loop = () => {
            requestAnimationFrame(loop);
            const now = performance.now();
            if (now - this._lastRender < this.config.updateInterval) return;
            this._lastRender = now;
            this._tickPheromones();
            this._syncCoverage();
            this._render();
        };
        loop();
    }

    _render() {
        if (!this.ctx) return;
        const ctx = this._offCtx || this.ctx;
        const w = this.canvas.width;
        const h = this.canvas.height;

        if (window.DIAMANTS_DOCTRINE) {
            const dz = window.DIAMANTS_DOCTRINE.zoneParams;
            this.config.zoneSize = Math.max(dz.sizeX, dz.sizeZ);
        }

        // Adaptive zoom
        const maxConfigZone = this.config.zoneSize;
        let rawExtent = 20;
        this.dronePositions.forEach((pos) => {
            const m = Math.max(Math.abs(pos.x), Math.abs(pos.z));
            if (m > rawExtent) rawExtent = m;
        });
        const visitedCells = window.DIAMANTS_VISITED_CELLS;
        const engineCellSize = window.DIAMANTS_CELL_SIZE || 2;
        if (visitedCells && visitedCells.size > 0) {
            for (const cellKey of visitedCells) {
                const parts = cellKey.split(',');
                const cx = Math.abs((parseInt(parts[0], 10) + 0.5) * engineCellSize);
                const cz = Math.abs((parseInt(parts[1], 10) + 0.5) * engineCellSize);
                const m = Math.max(cx, cz);
                if (m > rawExtent) rawExtent = m;
            }
        }
        const targetZone = Math.min(maxConfigZone, Math.max(40, rawExtent * 2 * 1.3));
        if (targetZone > this._adaptiveZone) {
            this._adaptiveZone += (targetZone - this._adaptiveZone) * 0.08;
            if (this._adaptiveZone > targetZone - 1) this._adaptiveZone = targetZone;
        }
        const zoneSize = this._adaptiveZone;
        const halfZone = zoneSize / 2;

        const _wToPixX = (wx) => ((wx + halfZone) / zoneSize) * w;
        const _wToPixY = (wz) => ((wz + halfZone) / zoneSize) * h;

        // Coverage %
        if (visitedCells && visitedCells.size > 0) {
            const arenaArea = maxConfigZone * maxConfigZone;
            const coveredArea = visitedCells.size * engineCellSize * engineCellSize;
            this.totalCoverage = Math.min(100, (coveredArea / arenaArea) * 100);
            this._journalCoverage = this.totalCoverage;
        }

        // === BACKGROUND ===
        ctx.fillStyle = '#0a1628';
        ctx.fillRect(0, 0, w, h);

        // === SUBTLE GRID ===
        ctx.strokeStyle = 'rgba(0, 255, 136, 0.04)';
        ctx.lineWidth = 0.5;
        for (let wpos = -halfZone; wpos <= halfZone; wpos += 10) {
            const px = _wToPixX(wpos);
            const py = _wToPixY(wpos);
            ctx.beginPath();
            ctx.moveTo(px, 0); ctx.lineTo(px, h);
            ctx.moveTo(0, py); ctx.lineTo(w, py);
            ctx.stroke();
        }

        // === PHEROMONE FIELD (the real stigmergy visualization) ===
        // Render the pheromone grid as colored cells with glow
        // Color ramp: dark green (low) -> bright green -> yellow -> white (high)
        const cellPxW = (PHERO_RES / zoneSize) * w;
        const cellPxH = (PHERO_RES / zoneSize) * h;

        // Compute visible grid range
        const gxMin = Math.max(0, Math.floor((-halfZone + PHERO_HALF) / PHERO_RES) - 1);
        const gxMax = Math.min(PHERO_GRID - 1, Math.ceil((halfZone + PHERO_HALF) / PHERO_RES) + 1);
        const gzMin = gxMin;
        const gzMax = gxMax;

        for (let gz = gzMin; gz <= gzMax; gz++) {
            for (let gx = gxMin; gx <= gxMax; gx++) {
                const val = this._pheroGrid[gz * PHERO_GRID + gx];
                if (val < 0.3) continue;

                const intensity = val / PHERO_MAX; // 0..1
                const worldX = (gx + 0.5) * PHERO_RES - PHERO_HALF;
                const worldZ = (gz + 0.5) * PHERO_RES - PHERO_HALF;
                const px = _wToPixX(worldX) - cellPxW / 2;
                const py = _wToPixY(worldZ) - cellPxH / 2;

                // Color ramp: green (low) -> yellow (med) -> white (high)
                let r, g, b, a;
                if (intensity < 0.3) {
                    // Dark green
                    r = 0; g = Math.floor(80 + intensity * 500); b = Math.floor(20 + intensity * 100);
                    a = intensity * 1.5;
                } else if (intensity < 0.6) {
                    // Bright green -> yellow
                    const t = (intensity - 0.3) / 0.3;
                    r = Math.floor(t * 255); g = Math.floor(230 + t * 25); b = Math.floor(50 * (1 - t));
                    a = 0.35 + t * 0.25;
                } else {
                    // Yellow -> white hot
                    const t = (intensity - 0.6) / 0.4;
                    r = 255; g = 255; b = Math.floor(t * 200);
                    a = 0.55 + t * 0.35;
                }

                ctx.fillStyle = 'rgba(' + r + ',' + g + ',' + b + ',' + a + ')';
                ctx.fillRect(px, py, cellPxW + 0.5, cellPxH + 0.5);
            }
        }

        // === GLOW PASS (larger, softer overlay for high-intensity areas) ===
        ctx.save();
        ctx.globalCompositeOperation = 'screen';
        for (let gz = gzMin; gz <= gzMax; gz++) {
            for (let gx = gxMin; gx <= gxMax; gx++) {
                const val = this._pheroGrid[gz * PHERO_GRID + gx];
                if (val < 15) continue; // only glow for stronger deposits

                const intensity = val / PHERO_MAX;
                const worldX = (gx + 0.5) * PHERO_RES - PHERO_HALF;
                const worldZ = (gz + 0.5) * PHERO_RES - PHERO_HALF;
                const px = _wToPixX(worldX);
                const py = _wToPixY(worldZ);

                const glowR = cellPxW * (1.5 + intensity * 2);
                const grad = ctx.createRadialGradient(px, py, 0, px, py, glowR);

                if (intensity > 0.5) {
                    grad.addColorStop(0, 'rgba(255, 255, 200, ' + (intensity * 0.2) + ')');
                    grad.addColorStop(0.4, 'rgba(180, 255, 50, ' + (intensity * 0.12) + ')');
                    grad.addColorStop(1, 'rgba(0, 100, 30, 0)');
                } else {
                    grad.addColorStop(0, 'rgba(0, 255, 100, ' + (intensity * 0.15) + ')');
                    grad.addColorStop(0.5, 'rgba(0, 150, 60, ' + (intensity * 0.08) + ')');
                    grad.addColorStop(1, 'rgba(0, 60, 20, 0)');
                }

                ctx.fillStyle = grad;
                ctx.fillRect(px - glowR, py - glowR, glowR * 2, glowR * 2);
            }
        }
        ctx.restore();

        // === WAYPOINTS (hidden at high autonomy — no orchestrator) ===
        const _autoLevel = (typeof window !== 'undefined' ? window.DIAMANTS_AUTONOMY_LEVEL : 0) ?? 0;
        if (this.config.showWaypoints && _autoLevel < 75) {
            this.dronePositions.forEach((pos, id) => {
                const wp = pos.waypoint;
                if (!wp) return;
                const wx = _wToPixX(wp.x);
                const wy = _wToPixY(wp.z);
                const color = this._getDroneColor(id, 0.7);

                ctx.strokeStyle = color;
                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.moveTo(wx - 4, wy - 4); ctx.lineTo(wx + 4, wy + 4);
                ctx.moveTo(wx + 4, wy - 4); ctx.lineTo(wx - 4, wy + 4);
                ctx.stroke();

                const dx = _wToPixX(pos.x);
                const dy = _wToPixY(pos.z);
                ctx.setLineDash([2, 4]);
                ctx.strokeStyle = this._getDroneColor(id, 0.2);
                ctx.lineWidth = 0.8;
                ctx.beginPath();
                ctx.moveTo(dx, dy);
                ctx.lineTo(wx, wy);
                ctx.stroke();
                ctx.setLineDash([]);
            });
        }

        // === DRONE POSITIONS ===
        this.dronePositions.forEach((pos, id) => {
            const cx = _wToPixX(pos.x);
            const cy = _wToPixY(pos.z);
            const color = DRONE_COLORS[_droneIdx(id) % DRONE_COLORS.length];

            // Soft halo
            ctx.beginPath();
            ctx.arc(cx, cy, 6, 0, Math.PI * 2);
            ctx.fillStyle = this._getDroneColor(id, 0.25);
            ctx.fill();

            // Body
            ctx.beginPath();
            ctx.arc(cx, cy, 3, 0, Math.PI * 2);
            ctx.fillStyle = color;
            ctx.fill();

            // Heading arrow
            if (pos.heading !== undefined) {
                const ax = cx + Math.sin(pos.heading) * 9;
                const ay = cy + Math.cos(pos.heading) * 9;
                ctx.beginPath();
                ctx.moveTo(cx, cy);
                ctx.lineTo(ax, ay);
                ctx.strokeStyle = color;
                ctx.lineWidth = 1.5;
                ctx.stroke();
            }

            // ID label
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 7px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(id.toString(), cx, cy - 8);
        });

        // === ZONE SIZE ===
        ctx.fillStyle = 'rgba(0, 255, 136, 0.5)';
        ctx.font = 'bold 8px monospace';
        ctx.textAlign = 'right';
        ctx.fillText(Math.round(zoneSize) + 'm', w - 4, h - 4);

        // === BORDER ===
        ctx.strokeStyle = '#00ff88';
        ctx.lineWidth = 2;
        ctx.strokeRect(1, 1, w - 2, h - 2);

        // Blit
        if (this._offCtx) {
            this.ctx.clearRect(0, 0, w, h);
            this.ctx.drawImage(this._offscreen, 0, 0);
        }

        this._updateUI();
    }

    _updateUI() {
        if (this.timerElement && this.isRunning) {
            this.timerElement.textContent = this._getElapsedTime();
        }
        if (this.percentElement) {
            const cov = this._journalCoverage || this.totalCoverage;
            this.percentElement.textContent = cov.toFixed(1) + '%';
            this.percentElement.style.color = cov >= 85 ? '#00FF88' : cov >= 50 ? '#FFD700' : '#FF6666';
        }
        const header = document.querySelector('#minimap .minimap-header');
        if (header) {
            const dm = window.doctrineManager;
            if (dm) {
                const doc = dm.currentDoctrine;
                const coa = dm.currentCOA;
                const label = (doc?.icon || '') + ' ' + (doc?.name || 'Exploration') + ' - ' + (coa?.icon || '') + ' ' + (coa?.name || '');
                // Update only the first text node to preserve child elements (detach button, expand indicator)
                let textNode = Array.from(header.childNodes).find(n => n.nodeType === Node.TEXT_NODE);
                if (textNode) { textNode.textContent = label + ' '; }
                else { header.insertBefore(document.createTextNode(label + ' '), header.firstChild); }
            }
        }
    }
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => { new ExplorationMinimap(); });
} else {
    setTimeout(() => new ExplorationMinimap(), 100);
}

export default ExplorationMinimap;
