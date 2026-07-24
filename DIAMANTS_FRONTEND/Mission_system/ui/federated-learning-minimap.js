/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Federated Learning Minimap
 * ========================================
 * Canvas minimap affichant l'entraînement fédéré en temps réel:
 *   - Grille de couverture heatmap (cellules explorées)
 *   - Zones d'attribution des coordinateurs (Voronoï simplifié)
 *   - Courbe de convergence reward/coverage par round
 *   - Indicateurs agent: poids sync, reward, coverage
 *
 * À côté des autres minimaps dans #minimaps-container.
 * Canvas ID: federated_canvas
 *
 * Écoute:
 *   - diamants:federated-update   → métriques par round
 *   - diamants:swarm-coordination → positions + zones coordinateurs
 *   - diamants:drone-positions    → positions temps réel
 *   - diamants:mission-status     → couverture globale
 */

const log = (...a) => console.log('[FedMap]', ...a);

export class FederatedLearningMinimap {
    constructor(canvasId = 'federated_canvas') {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) {
            log('⚠ Canvas not found:', canvasId);
            return;
        }
        this.ctx = this.canvas.getContext('2d');

        // ── Offscreen double-buffer ──
        this._off = document.createElement('canvas');
        this._off.width = this.canvas.width;
        this._off.height = this.canvas.height;
        this._offCtx = this._off.getContext('2d');
        this.canvas._minimapInstance = this;

        // ── Data ──
        this.zoneSize = 80;            // meters
        this.gridDim = 40;             // cells (2m resolution)
        this.coverageGrid = new Float32Array(this.gridDim * this.gridDim);

        // Coordinator zones
        this.coordinators = new Map();  // id → {x, z, role, coverage, color}

        // Training history
        this.roundHistory = [];         // [{round, coverage, avgReward}]
        this.maxHistoryLen = 60;

        // Federation state
        this.currentRound = 0;
        this.strategy = 'fedavg';
        this.globalCoverage = 0;

        // Drone positions (all types)
        this.dronePositions = new Map(); // id → {x, z, type}

        // Color palette for coordinators
        this._coordColors = ['#a78bfa', '#67e8f9', '#f59e0b', '#f472b6', '#34d399', '#fb923c'];

        // ── Mode toggle ──
        this._mode = 'heatmap'; // 'heatmap' | 'convergence'

        this._init();
        log('✅ FederatedLearningMinimap initialized');
    }

    _init() {
        this._hookEvents();
        this._setupClickToggle();
        this._startRenderLoop();

        window.DIAMANTS_FED_MINIMAP = this;
    }

    // =========================================================================
    // Event Hooks
    // =========================================================================

    _hookEvents() {
        // Federated training updates (from backend or simulated)
        window.addEventListener('diamants:federated-update', (evt) => {
            const d = evt.detail;
            if (d.round !== undefined) this.currentRound = d.round;
            if (d.strategy) this.strategy = d.strategy;
            if (d.globalCoverage !== undefined) this.globalCoverage = d.globalCoverage;

            // Append to history
            if (d.round !== undefined) {
                this.roundHistory.push({
                    round: d.round,
                    coverage: d.globalCoverage || 0,
                    avgReward: d.avgReward || 0,
                });
                if (this.roundHistory.length > this.maxHistoryLen) {
                    this.roundHistory.shift();
                }
            }

            // Coverage grid update
            if (d.coverageGrid && d.coverageGrid.length === this.gridDim * this.gridDim) {
                this.coverageGrid.set(d.coverageGrid);
            }

            // Agent info
            if (d.agents) {
                for (const a of d.agents) {
                    this._updateCoordinator(a);
                }
            }
        });

        // Swarm coordination (from SwarmCoordinationPanel events)
        window.addEventListener('diamants:swarm-coordination', (evt) => {
            const d = evt.detail;
            if (d.coordinators) {
                for (const c of d.coordinators) {
                    this._updateCoordinator(c);
                }
            }
            if (d.globalCoverage !== undefined) this.globalCoverage = d.globalCoverage;
            if (d.federationRound !== undefined) this.currentRound = d.federationRound;
        });

        // Drone positions for live heatmap
        window.addEventListener('diamants:drone-positions', (evt) => {
            const positions = evt.detail;
            if (!positions || typeof positions !== 'object') return;

            // Update drone positions map
            for (const [id, data] of Object.entries(positions)) {
                const pos = data.position || data;
                this.dronePositions.set(id, {
                    x: pos.x ?? pos.n ?? 0,
                    z: pos.z ?? pos.e ?? 0,
                    type: data.type || this._inferType(id),
                });
            }

            // Mark coverage cells from drone positions
            for (const [, pos] of this.dronePositions) {
                this._markCoverage(pos.x, pos.z, 3.0);
            }
        });

        // Mission status
        window.addEventListener('diamants:mission-status', (evt) => {
            if (evt.detail?.coverage !== undefined) {
                this.globalCoverage = evt.detail.coverage;
            }
        });
    }

    _updateCoordinator(c) {
        const id = c.id || c.coord_id;
        if (!id) return;
        const existing = this.coordinators.get(id);
        const idx = [...this.coordinators.keys()].indexOf(id);
        const colorIdx = idx >= 0 ? idx : this.coordinators.size;
        const color = this._coordColors[colorIdx % this._coordColors.length];

        this.coordinators.set(id, {
            x: c.zone?.n ?? c.position_n ?? existing?.x ?? 0,
            z: c.zone?.e ?? c.position_e ?? existing?.z ?? 0,
            role: c.role || existing?.role || 'coordinator',
            coverage: c.localCoverage ?? c.local_coverage ?? existing?.coverage ?? 0,
            color: existing?.color || color,
        });
    }

    _inferType(id) {
        const fleet = window.FLEET_CONFIG;
        if (fleet?.drones) {
            const d = fleet.drones.find(d => d.id === id);
            if (d) return d.type;
        }
        return 'crazyflie';
    }

    _markCoverage(worldX, worldZ, radius) {
        const half = this.zoneSize / 2;
        const cellSize = this.zoneSize / this.gridDim;
        const ci = Math.floor((worldX + half) / cellSize);
        const cj = Math.floor((worldZ + half) / cellSize);
        const r = Math.max(1, Math.floor(radius / cellSize));

        for (let di = -r; di <= r; di++) {
            for (let dj = -r; dj <= r; dj++) {
                const ni = ci + di;
                const nj = cj + dj;
                if (ni >= 0 && ni < this.gridDim && nj >= 0 && nj < this.gridDim) {
                    const idx = ni * this.gridDim + nj;
                    this.coverageGrid[idx] = Math.min(1.0, this.coverageGrid[idx] + 0.1);
                }
            }
        }
    }

    _setupClickToggle() {
        if (!this.canvas) return;
        this.canvas.addEventListener('click', () => {
            this._mode = this._mode === 'heatmap' ? 'convergence' : 'heatmap';
        });
    }

    // =========================================================================
    // Render
    // =========================================================================

    _startRenderLoop() {
        setInterval(() => this._draw(), 500); // 2 FPS is sufficient
    }

    _draw() {
        if (!this._offCtx) return;
        const ctx = this._offCtx;
        const w = this._off.width;
        const h = this._off.height;

        ctx.clearRect(0, 0, w, h);

        if (this._mode === 'heatmap') {
            this._drawHeatmap(ctx, w, h);
        } else {
            this._drawConvergence(ctx, w, h);
        }

        // Blit to visible canvas
        this.ctx.clearRect(0, 0, w, h);
        this.ctx.drawImage(this._off, 0, 0);
    }

    // ── Heatmap Mode ──

    _drawHeatmap(ctx, w, h) {
        const cellW = w / this.gridDim;
        const cellH = h / this.gridDim;

        // Background
        ctx.fillStyle = '#080c18';
        ctx.fillRect(0, 0, w, h);

        // Coverage heatmap
        for (let i = 0; i < this.gridDim; i++) {
            for (let j = 0; j < this.gridDim; j++) {
                const val = this.coverageGrid[i * this.gridDim + j];
                if (val > 0.01) {
                    const intensity = Math.min(1.0, val);
                    // Green → Yellow → White intensity gradient
                    const r = Math.floor(intensity * 255 * 0.3);
                    const g = Math.floor(40 + intensity * 200);
                    const b = Math.floor(intensity * 100 * 0.2);
                    ctx.fillStyle = `rgba(${r}, ${g}, ${b}, ${0.3 + intensity * 0.6})`;
                    ctx.fillRect(j * cellW, i * cellH, cellW + 0.5, cellH + 0.5);
                }
            }
        }

        // Coordinator zone overlays
        const half = this.zoneSize / 2;
        const scaleX = w / this.zoneSize;
        const scaleY = h / this.zoneSize;

        for (const [id, coord] of this.coordinators) {
            const px = (coord.x + half) * scaleX;
            const py = (coord.z + half) * scaleY;

            // Zone circle (influence radius ~20m)
            ctx.beginPath();
            ctx.arc(px, py, 20 * scaleX, 0, Math.PI * 2);
            ctx.strokeStyle = coord.color + '55';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 4]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Agent marker
            const isPatrol = coord.role === 'patrol';
            ctx.beginPath();
            if (isPatrol) {
                // Diamond for patrol
                ctx.moveTo(px, py - 6);
                ctx.lineTo(px + 5, py);
                ctx.lineTo(px, py + 6);
                ctx.lineTo(px - 5, py);
                ctx.closePath();
            } else {
                // Triangle for coordinator
                ctx.moveTo(px, py - 7);
                ctx.lineTo(px + 6, py + 4);
                ctx.lineTo(px - 6, py + 4);
                ctx.closePath();
            }
            ctx.fillStyle = coord.color;
            ctx.fill();
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 1;
            ctx.stroke();

            // Label
            ctx.fillStyle = '#fff';
            ctx.font = '8px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(id.replace('drone_', 'D'), px, py + 16);

            // Coverage arc
            const covAngle = (coord.coverage || 0) * Math.PI * 2;
            if (covAngle > 0.01) {
                ctx.beginPath();
                ctx.arc(px, py, 10, -Math.PI / 2, -Math.PI / 2 + covAngle);
                ctx.strokeStyle = coord.color;
                ctx.lineWidth = 2;
                ctx.stroke();
            }
        }

        // Draw scout drones (small dots)
        for (const [id, pos] of this.dronePositions) {
            if (this.coordinators.has(id)) continue; // skip coordinators
            const px = (pos.x + half) * scaleX;
            const py = (pos.z + half) * scaleY;
            if (px < -5 || px > w + 5 || py < -5 || py > h + 5) continue;

            ctx.beginPath();
            ctx.arc(px, py, 2.5, 0, Math.PI * 2);
            ctx.fillStyle = '#34d399';
            ctx.fill();
        }

        // HUD overlay
        ctx.fillStyle = 'rgba(0,0,0,0.6)';
        ctx.fillRect(0, 0, w, 18);
        ctx.fillStyle = '#d4d4d8';
        ctx.font = '9px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`Fed R:${this.currentRound} | ${this.strategy.toUpperCase()}`, 4, 12);
        ctx.textAlign = 'right';
        // Self-compute coverage from grid (more accurate than coordinator's value)
        const covPct = Math.max(Math.round(this.globalCoverage * 100), this._gridCoveragePct());
        ctx.fillStyle = covPct > 70 ? '#10b981' : covPct > 40 ? '#f59e0b' : '#ef4444';
        ctx.fillText(`Cov: ${covPct}%`, w - 4, 12);

        // Mode indicator
        ctx.fillStyle = '#555';
        ctx.font = '8px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('▸ click: convergence', w / 2, h - 4);
    }

    // ── Convergence Mode ──

    _drawConvergence(ctx, w, h) {
        ctx.fillStyle = '#080c18';
        ctx.fillRect(0, 0, w, h);

        const margin = { top: 22, bottom: 25, left: 35, right: 10 };
        const plotW = w - margin.left - margin.right;
        const plotH = h - margin.top - margin.bottom;

        // Title
        ctx.fillStyle = '#d4d4d8';
        ctx.font = 'bold 10px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('Federated Convergence', w / 2, 14);

        // Axes
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(margin.left, margin.top);
        ctx.lineTo(margin.left, margin.top + plotH);
        ctx.lineTo(margin.left + plotW, margin.top + plotH);
        ctx.stroke();

        // Y-axis labels
        ctx.fillStyle = '#888';
        ctx.font = '8px monospace';
        ctx.textAlign = 'right';
        for (let p = 0; p <= 100; p += 25) {
            const y = margin.top + plotH - (p / 100) * plotH;
            ctx.fillText(`${p}%`, margin.left - 4, y + 3);
            // Grid line
            ctx.strokeStyle = '#1a1a2e';
            ctx.beginPath();
            ctx.moveTo(margin.left, y);
            ctx.lineTo(margin.left + plotW, y);
            ctx.stroke();
        }

        if (this.roundHistory.length < 2) {
            ctx.fillStyle = '#555';
            ctx.font = '10px monospace';
            ctx.textAlign = 'center';
            ctx.fillText('Waiting for data...', w / 2, h / 2);
            ctx.fillText('▸ click: heatmap', w / 2, h - 8);
            return;
        }

        // X-axis
        const nPoints = this.roundHistory.length;
        const stepX = plotW / Math.max(1, nPoints - 1);
        ctx.fillStyle = '#888';
        ctx.font = '8px monospace';
        ctx.textAlign = 'center';

        // Coverage curve (green)
        ctx.beginPath();
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = 2;
        for (let i = 0; i < nPoints; i++) {
            const x = margin.left + i * stepX;
            const y = margin.top + plotH - this.roundHistory[i].coverage * plotH;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // Reward curve (purple, normalized 0-1)
        const maxReward = Math.max(1, ...this.roundHistory.map(r => Math.abs(r.avgReward)));
        ctx.beginPath();
        ctx.strokeStyle = '#a78bfa';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([3, 3]);
        for (let i = 0; i < nPoints; i++) {
            const x = margin.left + i * stepX;
            const normReward = (this.roundHistory[i].avgReward + maxReward) / (2 * maxReward);
            const y = margin.top + plotH - normReward * plotH;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);

        // Legend
        ctx.font = '8px monospace';
        const legY = margin.top + plotH + 14;
        ctx.fillStyle = '#10b981';
        ctx.fillRect(margin.left, legY - 4, 10, 3);
        ctx.fillStyle = '#aaa';
        ctx.textAlign = 'left';
        ctx.fillText('Coverage', margin.left + 14, legY);

        ctx.fillStyle = '#a78bfa';
        ctx.fillRect(margin.left + 70, legY - 4, 10, 3);
        ctx.fillStyle = '#aaa';
        ctx.fillText('Reward', margin.left + 84, legY);

        // Round counter
        ctx.fillStyle = '#555';
        ctx.textAlign = 'right';
        ctx.fillText(`R:${this.currentRound}`, w - margin.right, legY);

        // Mode indicator
        ctx.fillStyle = '#555';
        ctx.font = '8px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('▸ click: heatmap', w / 2, h - 4);
    }

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * Compute coverage % from own grid (count non-zero cells).
     */
    _gridCoveragePct() {
        let filled = 0;
        for (let i = 0; i < this.coverageGrid.length; i++) {
            if (this.coverageGrid[i] > 0.01) filled++;
        }
        return Math.round((filled / this.coverageGrid.length) * 100);
    }

    /**
     * Manually push a federation round result (for testing or backend bridge).
     */
    pushRound(round, coverage, avgReward) {
        this.currentRound = round;
        this.globalCoverage = coverage;
        this.roundHistory.push({ round, coverage, avgReward });
        if (this.roundHistory.length > this.maxHistoryLen) {
            this.roundHistory.shift();
        }
    }

    /**
     * Reset all data.
     */
    reset() {
        this.coverageGrid.fill(0);
        this.coordinators.clear();
        this.dronePositions.clear();
        this.roundHistory = [];
        this.currentRound = 0;
        this.globalCoverage = 0;
    }
}
