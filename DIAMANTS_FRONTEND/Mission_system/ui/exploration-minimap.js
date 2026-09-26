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
 * Visualisation stigmergique authentique avec :
 *   - Grille de phéromones persistante (dépôt + diffusion + évaporation)
 *   - Échelle étendue adaptée au théâtre opérationnel 240x240m (256m / ±128m)
 *   - Conditions aux limites dissipatives (élimination des barres blanches saturées)
 *   - Dégradé continu vert émeraude → jaune doré → rehaut doux
 *   - Rendu haute performance à double passe (cellules nettes + halo diffusé)
 *   - Positions des drones avec heading + identifiant court NATO
 *   - Synchronisation de résolution dynamique (DPR / _syncSize)
 *   - Waypoints cibles et indicateur de couverture
 *
 * Modèle stigmergique aligné sur la doctrine et le vol autonome.
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

function _shortId(id) {
    const s = String(id);
    const m = s.match(/(\d+)\s*$/);
    const n = m ? m[1] : '';
    if (/^crazyflie/i.test(s)) return 'CF' + n;
    if (/^x500/i.test(s)) return 'X' + n;
    if (/^s500/i.test(s)) return 'S' + n;
    if (/^colossus/i.test(s)) return 'COL' + (n.replace(/^0+/, '') || n);
    return s.length > 7 ? s.slice(0, 7) : s;
}

// ─── Pheromone grid constants ────────────────────────────────────────
// Grille 128×128 à 2.0m de résolution = 256m d'étendue totale (±128m).
// Englobe l'intégralité du théâtre opérationnel 240×240m avec marge de confort.
const PHERO_GRID = 128;             // cellules par côté (128×128 = 16 384 cellules)
const PHERO_RES = 2.0;              // mètres par cellule
const PHERO_HALF = (PHERO_GRID * PHERO_RES) / 2; // 128.0m demi-étendue
const PHERO_DEPOSIT = 4.5;          // dépôt par frame sous chaque drone
const PHERO_DEPOSIT_RADIUS = 2;     // rayon de dépôt en cellules (~4m autour de l'appareil)
const PHERO_EVAP_RATE = 0.0012;     // évaporation douce (~1.2% / sec à 10FPS, sillage visible ~45-60s)
const PHERO_DIFFUSION_RATE = 0.08;  // fraction redistribuée aux 8 voisins
const PHERO_MAX = 100;              // intensité maximale

export class ExplorationMinimap {
    constructor(canvasId = 'minimap_canvas') {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas?.getContext('2d');

        this.config = {
            gridSize: 120,
            zoneSize: 240,
            updateInterval: 100,
            showWaypoints: true,
        };

        this.explorationStartTime = null;
        this.totalCoverage = 0;
        this.isRunning = false;

        this._adaptiveZone = 60;
        this._maxExtent = 0;

        /** @type {Map<number, {x:number, y:number, z:number, heading:number, waypoint:{x:number,z:number}|null}>} */
        this.dronePositions = new Map();

        // === PHEROMONE GRID (persistent, with diffusion + evaporation) ===
        this._pheroGrid = new Float32Array(PHERO_GRID * PHERO_GRID);
        this._pheroScratch = new Float32Array(PHERO_GRID * PHERO_GRID);
        this._diffuseCounter = 0;
        this._visitedSyncedSize = 0;

        this._champCv = null;
        this._champCtx = null;
        this._champImg = null;

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
        this._offscreen.width = this.canvas.width || 340;
        this._offscreen.height = this.canvas.height || 250;
        this._offCtx = this._offscreen.getContext('2d');

        const ancienne = this.canvas._minimapInstance;
        if (ancienne && ancienne !== this) ancienne._retiree = true;
        this.canvas._minimapInstance = this;

        this._createTimerElement();
        this._startRenderLoop();
        window.DIAMANTS_MINIMAP = this;
        console.log('Exploration Minimap initialisee (pheromone field 256m, dissipative boundaries)');
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

    _syncSize() {
        const c = this.canvas;
        if (!c) return false;
        const r = c.getBoundingClientRect();
        const cw = Math.max(80, Math.round(r.width));
        const ch = Math.max(80, Math.round(r.height));
        const dpr = Math.min(2.5, window.devicePixelRatio || 1);
        const bw = Math.round(cw * dpr), bh = Math.round(ch * dpr);
        if (c.width !== bw || c.height !== bh) { c.width = bw; c.height = bh; }
        if (this._offscreen && (this._offscreen.width !== bw || this._offscreen.height !== bh)) {
            this._offscreen.width = bw; this._offscreen.height = bh;
        }
        this._cssW = cw; this._cssH = ch; this._dpr = dpr;
        return true;
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
        this._visitedSyncedSize = 0;
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

        // Skip if entirely outside grid
        if (gx0 < -R || gx0 >= PHERO_GRID + R || gz0 < -R || gz0 >= PHERO_GRID + R) return;

        for (let dz = -R; dz <= R; dz++) {
            const gz = gz0 + dz;
            if (gz < 0 || gz >= PHERO_GRID) continue;
            const rowOffset = gz * PHERO_GRID;
            for (let dx = -R; dx <= R; dx++) {
                const gx = gx0 + dx;
                if (gx < 0 || gx >= PHERO_GRID) continue;
                const dist = Math.sqrt(dx * dx + dz * dz);
                if (dist > R + 0.5) continue;
                const falloff = 1 - dist / (R + 1);
                const idx = rowOffset + gx;
                this._pheroGrid[idx] = Math.min(PHERO_MAX, this._pheroGrid[idx] + intensity * falloff * falloff);
            }
        }
    }

    /** Evaporate all cells: value *= (1 - rate) */
    _evaporate() {
        const decay = 1 - PHERO_EVAP_RATE;
        for (let i = 0; i < this._pheroGrid.length; i++) {
            if (this._pheroGrid[i] > 0) {
                this._pheroGrid[i] *= decay;
                if (this._pheroGrid[i] < 0.05) this._pheroGrid[i] = 0;
            }
        }
    }

    /**
     * Diffuse pheromones to 8-neighbors with dissipative open-world boundaries.
     * Open boundary condition: any pheromone diffusing to the perimeter
     * naturally dissipates into space instead of forming saturated edge walls.
     */
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
            const row = z * N;
            for (let x = 1; x < N - 1; x++) {
                const idx = row + x;
                const val = g[idx];
                if (val < 0.08) continue;
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

        // Dissipative boundaries: zero the outer perimeter so no accumulation can occur
        const lastRow = (N - 1) * N;
        for (let i = 0; i < N; i++) {
            s[i] = 0;                 // top row (z = 0)
            s[lastRow + i] = 0;       // bottom row (z = N - 1)
            s[i * N] = 0;             // left column (x = 0)
            s[i * N + (N - 1)] = 0;   // right column (x = N - 1)
        }

        // Write back clamped
        for (let i = 0; i < s.length; i++) {
            this._pheroGrid[i] = Math.min(PHERO_MAX, Math.max(0, s[i]));
        }
    }

    /** Sync visited cells from flight engine so explored terrain is immediately visible */
    _syncVisitedCells() {
        const visited = window.DIAMANTS_VISITED_CELLS;
        const cellSize = window.DIAMANTS_CELL_SIZE || 2;
        if (!visited || visited.size === 0) return;
        if (this._visitedSyncedSize === visited.size) return;
        this._visitedSyncedSize = visited.size;

        for (const cellKey of visited) {
            const parts = cellKey.split(',');
            const wx = (parseInt(parts[0], 10) + 0.5) * cellSize;
            const wz = (parseInt(parts[1], 10) + 0.5) * cellSize;
            const gx = Math.floor((wx + PHERO_HALF) / PHERO_RES);
            const gz = Math.floor((wz + PHERO_HALF) / PHERO_RES);
            if (gx >= 0 && gx < PHERO_GRID && gz >= 0 && gz < PHERO_GRID) {
                const idx = gz * PHERO_GRID + gx;
                if (this._pheroGrid[idx] < 20.0) {
                    this._pheroGrid[idx] = 20.0;
                }
            }
        }
    }

    /** Full pheromone tick: deposit for each drone, evaporate, diffuse every 3 frames */
    _tickPheromones() {
        this.dronePositions.forEach((pos) => {
            this._deposit(pos.x, pos.z, PHERO_DEPOSIT);
        });

        this._syncVisitedCells();
        this._evaporate();

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
            // Computed in _render
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
            if (this._retiree) return;
            requestAnimationFrame(loop);
            const now = performance.now();
            if (now - this._lastRender < this.config.updateInterval) return;
            this._lastRender = now;
            const _el = this.canvas || this.ctx?.canvas;
            if (_el && _el.getClientRects().length === 0 && !_el._viewerShowing) return;
            this._tickPheromones();
            this._syncCoverage();
            this._render();
        };
        loop();
    }

    /**
     * Peint le champ de phéromones sur le canevas intermédiaire 128×128.
     * Dégradé organique continu :
     *   - Faible (< 0.35) : vert forêt → émeraude vif
     *   - Moyen (0.35..0.75) : émeraude → jaune doré ambré
     *   - Fort (> 0.75) : ambre doré → blanc doré chaud (sans rupture)
     */
    _peindreChamp(gxMin, gxMax, gzMin, gzMax) {
        if (!this._champCv || this._champCv.width !== PHERO_GRID) {
            this._champCv = document.createElement('canvas');
            this._champCv.width = this._champCv.height = PHERO_GRID;
            this._champCtx = this._champCv.getContext('2d');
            this._champImg = this._champCtx.createImageData(PHERO_GRID, PHERO_GRID);
        }
        const d = this._champImg.data;
        d.fill(0);

        const z0 = Math.max(0, gzMin), z1 = Math.min(PHERO_GRID - 1, gzMax);
        const x0 = Math.max(0, gxMin), x1 = Math.min(PHERO_GRID - 1, gxMax);

        for (let gz = z0; gz <= z1; gz++) {
            const rowOffset = gz * PHERO_GRID;
            for (let gx = x0; gx <= x1; gx++) {
                const val = this._pheroGrid[rowOffset + gx];
                if (val < 1.0) continue;

                const intensity = Math.min(1.0, val / PHERO_MAX);
                const o = (rowOffset + gx) * 4;

                let r, g, b, a;
                if (intensity < 0.35) {
                    const t = intensity / 0.35;
                    r = Math.floor(t * 35);
                    g = Math.floor(130 + t * 110);
                    b = Math.floor(55 * (1 - t) + 15);
                    a = Math.floor(50 + t * 90);
                } else if (intensity < 0.75) {
                    const t = (intensity - 0.35) / 0.40;
                    r = Math.floor(35 + t * 205);
                    g = Math.floor(240 + t * 10);
                    b = Math.floor(15 * (1 - t));
                    a = Math.floor(140 + t * 65);
                } else {
                    const t = (intensity - 0.75) / 0.25;
                    r = 255;
                    g = Math.floor(250 + t * 5);
                    b = Math.floor(t * 160);
                    a = Math.floor(205 + t * 45);
                }

                d[o] = r;
                d[o + 1] = g;
                d[o + 2] = b;
                d[o + 3] = a;
            }
        }
        this._champCtx.putImageData(this._champImg, 0, 0);
    }

    _render() {
        if (!this.ctx) return;
        if (!this._syncSize()) return;

        const ctx = this._offCtx || this.ctx;
        const dpr = this._dpr || 1;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.textBaseline = 'alphabetic';

        const w = this._cssW;
        const h = this._cssH;

        // Synchroniser la taille de zone avec la doctrine
        let maxConfigZone = this.config.zoneSize;
        if (window.DIAMANTS_DOCTRINE?.zoneParams) {
            const dz = window.DIAMANTS_DOCTRINE.zoneParams;
            maxConfigZone = Math.max(dz.sizeX, dz.sizeZ);
            this.config.zoneSize = maxConfigZone;
        }

        // Zoom adaptatif fluide couvrant toute la flotte
        let rawExtent = 30;
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
        const targetZone = Math.min(maxConfigZone, Math.max(60, rawExtent * 2 * 1.15));
        if (targetZone > this._adaptiveZone) {
            this._adaptiveZone += (targetZone - this._adaptiveZone) * 0.08;
            if (this._adaptiveZone > targetZone - 1) this._adaptiveZone = targetZone;
        } else if (targetZone < this._adaptiveZone && this._adaptiveZone > maxConfigZone) {
            this._adaptiveZone = maxConfigZone;
        }

        const zoneSize = this._adaptiveZone;
        const halfZone = zoneSize / 2;

        const _wToPixX = (wx) => ((wx + halfZone) / zoneSize) * w;
        const _wToPixY = (wz) => ((wz + halfZone) / zoneSize) * h;

        // Calcul du % de couverture
        if (visitedCells && visitedCells.size > 0) {
            const arenaArea = maxConfigZone * maxConfigZone;
            const coveredArea = visitedCells.size * engineCellSize * engineCellSize;
            this.totalCoverage = Math.min(100, (coveredArea / arenaArea) * 100);
            this._journalCoverage = this.totalCoverage;
        }

        // === FOND DE CARTE TACTIQUE ===
        ctx.fillStyle = '#0a1628';
        ctx.fillRect(0, 0, w, h);

        // === GRILLE DE RÉFÉRENCE TACTIQUE ===
        ctx.strokeStyle = 'rgba(0, 255, 136, 0.05)';
        ctx.lineWidth = 0.5;
        const pasGrille = zoneSize > 150 ? 20 : 10;
        for (let wpos = -halfZone; wpos <= halfZone; wpos += pasGrille) {
            const px = _wToPixX(wpos);
            const py = _wToPixY(wpos);
            ctx.beginPath();
            ctx.moveTo(px, 0); ctx.lineTo(px, h);
            ctx.moveTo(0, py); ctx.lineTo(w, py);
            ctx.stroke();
        }

        // === CHAMP DE PHÉROMONES (Double passe haute performance) ===
        const cellPxW = (PHERO_RES / zoneSize) * w;
        const cellPxH = (PHERO_RES / zoneSize) * h;

        const gxMin = Math.max(0, Math.floor((-halfZone + PHERO_HALF) / PHERO_RES) - 1);
        const gxMax = Math.min(PHERO_GRID - 1, Math.ceil((halfZone + PHERO_HALF) / PHERO_RES) + 1);
        const gzMin = Math.max(0, Math.floor((-halfZone + PHERO_HALF) / PHERO_RES) - 1);
        const gzMax = Math.min(PHERO_GRID - 1, Math.ceil((halfZone + PHERO_HALF) / PHERO_RES) + 1);

        this._peindreChamp(gxMin, gxMax, gzMin, gzMax);

        const cx0 = _wToPixX(-PHERO_HALF), cx1 = _wToPixX(PHERO_HALF);
        const cy0 = _wToPixY(-PHERO_HALF), cy1 = _wToPixY(PHERO_HALF);
        const dw = Math.abs(cx1 - cx0);
        const dh = Math.abs(cy1 - cy0);
        const dx = Math.min(cx0, cx1);
        const dy = Math.min(cy0, cy1);

        // Passe 1 : cellules discrètes (trame tactile nette)
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(this._champCv, dx, dy, dw, dh);

        // Passe 2 : halo diffusé continu (glow organique de phéromones)
        ctx.save();
        ctx.globalCompositeOperation = 'screen';
        ctx.imageSmoothingEnabled = true;
        const flou = Math.max(2.0, cellPxW * 0.9);
        ctx.filter = `blur(${flou.toFixed(1)}px)`;
        ctx.drawImage(this._champCv, dx, dy, dw, dh);
        ctx.filter = 'none';
        ctx.restore();

        // Échelle d'interface proportionnelle
        const k = Math.max(0.85, Math.min(2.0, Math.min(w, h) / 200));

        // === WAYPOINTS CIBLES ===
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
                ctx.moveTo(wx - 4 * k, wy - 4 * k); ctx.lineTo(wx + 4 * k, wy + 4 * k);
                ctx.moveTo(wx + 4 * k, wy - 4 * k); ctx.lineTo(wx - 4 * k, wy + 4 * k);
                ctx.stroke();

                const dpx = _wToPixX(pos.x);
                const dpy = _wToPixY(pos.z);
                ctx.setLineDash([2, 4]);
                ctx.strokeStyle = this._getDroneColor(id, 0.25);
                ctx.lineWidth = 0.8;
                ctx.beginPath();
                ctx.moveTo(dpx, dpy);
                ctx.lineTo(wx, wy);
                ctx.stroke();
                ctx.setLineDash([]);
            });
        }

        // === POSITIONS DES DRONES ===
        this.dronePositions.forEach((pos, id) => {
            const cx = _wToPixX(pos.x);
            const cy = _wToPixY(pos.z);
            const color = DRONE_COLORS[_droneIdx(id) % DRONE_COLORS.length];

            // Halo doux
            ctx.beginPath();
            ctx.arc(cx, cy, 6 * k, 0, Math.PI * 2);
            ctx.fillStyle = this._getDroneColor(id, 0.25);
            ctx.fill();

            // Corps du drone
            ctx.beginPath();
            ctx.arc(cx, cy, 3 * k, 0, Math.PI * 2);
            ctx.fillStyle = color;
            ctx.fill();

            // Flèche de cap
            if (pos.heading !== undefined) {
                const ax = cx + Math.sin(pos.heading) * (9 * k);
                const ay = cy + Math.cos(pos.heading) * (9 * k);
                ctx.beginPath();
                ctx.moveTo(cx, cy);
                ctx.lineTo(ax, ay);
                ctx.strokeStyle = color;
                ctx.lineWidth = 1.5;
                ctx.stroke();
            }

            // Identifiant court NATO
            ctx.fillStyle = '#ffffff';
            ctx.font = `bold ${Math.round(8 * k)}px monospace`;
            ctx.textAlign = 'center';
            ctx.fillText(_shortId(id), cx, cy - 8 * k);
        });

        // === INDICATEUR DE TAILLE DE ZONE ===
        ctx.fillStyle = 'rgba(0, 255, 136, 0.6)';
        ctx.font = `bold ${Math.round(9 * k)}px monospace`;
        ctx.textAlign = 'right';
        ctx.fillText(Math.round(zoneSize) + 'm', w - 6, h - 6);

        // === CADRE DU RADAR ===
        ctx.strokeStyle = '#00ff88';
        ctx.lineWidth = 1.5;
        ctx.strokeRect(1, 1, w - 2, h - 2);

        // Recopie du tampon vers le canevas d'affichage
        if (this._offCtx) {
            this.ctx.setTransform(1, 0, 0, 1, 0, 0);
            this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
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
                let textNode = Array.from(header.childNodes).find(n => n.nodeType === Node.TEXT_NODE);
                if (textNode) { textNode.textContent = label + ' '; }
                else { header.insertBefore(document.createTextNode(label + ' '), header.firstChild); }
            }
        }
    }
}

function _autoInstancier() {
    const cv = document.getElementById('minimap_canvas');
    if (cv && cv._minimapInstance) return;
    new ExplorationMinimap();
}
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => { setTimeout(_autoInstancier, 400); });
} else {
    setTimeout(_autoInstancier, 400);
}

export default ExplorationMinimap;
