/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Federated Learning Minimap (RL & Swarm MARL)
 * ========================================================
 * Visualisation temps réel de l'apprentissage par renforcement fédéré (FedAvg / MARL) :
 *   - Grille de couverture heatmap dynamique synchronisée (théâtre 240x240m)
 *   - Conditionnement aux limites et projection fidèle de l'arène opérationnelle
 *   - Zones de coordination Voronoï & têtes de grappes (X500 leaders / coordinateurs)
 *   - Faisceaux d'agrégation fédérée (liaison scouts/patrouilleurs ↔ agrégateurs)
 *   - Courbes de convergence multi-agents (taux de couverture & retour moyen de l'essaim)
 *   - Synchronisation de résolution dynamique (DPR / _syncSize) avec le dock
 *   - Bascule tactile/clic instantanée : Heatmap spatiale ⇄ Courbe de convergence
 *
 * Canvas ID: federated_canvas
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
        this._off.width = this.canvas.width || 340;
        this._off.height = this.canvas.height || 220;
        this._offCtx = this._off.getContext('2d');
        this.canvas._minimapInstance = this;

        // ── Data & Theater Dimensions ──
        this.zoneSize = 240;            // meters (-120m to +120m)
        this.gridDim = 60;             // 60x60 cells (4m per cell, covers 240m)
        this.coverageGrid = new Float32Array(this.gridDim * this.gridDim);
        this._visitedSyncedSize = 0;

        // Coordinator zones & Clusters
        this.coordinators = new Map();  // id → {x, z, role, coverage, color, radius}

        // Training history (convergence over FedAvg rounds)
        this.roundHistory = [];         // [{round, coverage, avgReward, timestamp}]
        this.maxHistoryLen = 80;

        // Federation state
        this.currentRound = 0;
        this.strategy = 'fedavg';
        this.learningRate = 0.01;
        this.globalCoverage = 0;
        this._lastRoundTime = 0;
        this._lastAvgReward = 0.15;

        // Drone positions (all types)
        this.dronePositions = new Map(); // id → {x, z, heading, type, role}

        // Color palette for coordinators / clusters
        this._coordColors = ['#a78bfa', '#38bdf8', '#fbbf24', '#f472b6', '#34d399', '#fb923c'];

        // ── Mode toggle ──
        this._mode = 'heatmap'; // 'heatmap' | 'convergence'

        this._init();
        log('✅ FederatedLearningMinimap initialized (240m theater, FedAvg active)');
    }

    _init() {
        this._hookEvents();
        this._setupClickToggle();
        this._startRenderLoop();

        window.DIAMANTS_FED_MINIMAP = this;
    }

    // =========================================================================
    // Dynamic Canvas Sizing & High-DPI Support
    // =========================================================================

    _syncSize() {
        const c = this.canvas;
        if (!c) return false;
        const r = c.getBoundingClientRect();
        const cw = Math.max(80, Math.round(r.width));
        const ch = Math.max(80, Math.round(r.height));
        const dpr = Math.min(2.5, window.devicePixelRatio || 1);
        const bw = Math.round(cw * dpr);
        const bh = Math.round(ch * dpr);

        if (c.width !== bw || c.height !== bh) {
            c.width = bw;
            c.height = bh;
        }
        if (this._off.width !== bw || this._off.height !== bh) {
            this._off.width = bw;
            this._off.height = bh;
        }
        this._dpr = dpr;
        this._cssW = cw;
        this._cssH = ch;
        return true;
    }

    // =========================================================================
    // Event Hooks
    // =========================================================================

    _hookEvents() {
        // Federated training updates (from backend or simulated)
        window.addEventListener('diamants:federated-update', (evt) => {
            const d = evt.detail;
            if (!d) return;
            if (d.round !== undefined) this.currentRound = d.round;
            if (d.strategy) this.strategy = d.strategy;
            if (d.globalCoverage !== undefined) this.globalCoverage = d.globalCoverage;

            if (d.round !== undefined) {
                this._recordRound(d.round, d.globalCoverage || 0, d.avgReward || 0);
            }

            if (d.coverageGrid && d.coverageGrid.length === this.gridDim * this.gridDim) {
                this.coverageGrid.set(d.coverageGrid);
            }

            if (d.agents) {
                for (const a of d.agents) {
                    this._updateCoordinator(a);
                }
            }
        });

        // Swarm coordination (from SwarmCoordinationPanel events)
        window.addEventListener('diamants:swarm-coordination', (evt) => {
            const d = evt.detail;
            if (!d) return;
            if (d.coordinators) {
                for (const c of d.coordinators) {
                    this._updateCoordinator(c);
                }
            }
            if (d.globalCoverage !== undefined) this.globalCoverage = d.globalCoverage;
            if (d.federationRound !== undefined) this.currentRound = d.federationRound;
        });

        // Drone positions broadcast fallback
        window.addEventListener('diamants:drone-positions', (evt) => {
            const positions = evt.detail;
            if (!positions || typeof positions !== 'object') return;

            for (const [id, data] of Object.entries(positions)) {
                const pos = data.position || data;
                this.updateDronePosition(id, pos, data.heading || 0, data.role || data.type);
            }
        });

        // Mission status
        window.addEventListener('diamants:mission-status', (evt) => {
            if (evt.detail?.coverage !== undefined) {
                this.globalCoverage = evt.detail.coverage;
            }
        });
    }

    _recordRound(round, coverage, avgReward) {
        this.roundHistory.push({
            round,
            coverage: coverage || 0,
            avgReward: avgReward || 0,
            timestamp: performance.now(),
        });
        if (this.roundHistory.length > this.maxHistoryLen) {
            this.roundHistory.shift();
        }
    }

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * Direct drone position feed called per-frame from flight loop.
     * @param {string} id Drone identifier
     * @param {{x:number, y:number, z:number}} position Position in world meters
     * @param {number} heading Heading in radians
     * @param {string|null} role Role override (coordinator, patrol, scout, etc.)
     */
    updateDronePosition(id, position, heading = 0, role = null) {
        if (!position) return;
        const x = position.x ?? position.n ?? 0;
        const z = position.z ?? position.e ?? 0;
        const type = this._inferType(id);
        const resolvedRole = role || this._inferRole(id, type);

        this.dronePositions.set(id, {
            x,
            z,
            heading: heading || 0,
            type,
            role: resolvedRole
        });

        // Check if this drone should be registered as a coordinator / cluster head
        if (resolvedRole === 'coordinator' || resolvedRole === 'leader' || id.includes('leader') || id.includes('x500_0')) {
            this._updateCoordinator({
                id,
                position_n: x,
                position_e: z,
                role: resolvedRole,
                localCoverage: this.globalCoverage
            });
        }

        // Mark coverage cell footprint (radius 4.5m for drones, 3m for Crazyflie)
        const footprint = type === 'crazyflie' ? 3.5 : 5.5;
        this._markCoverage(x, z, footprint);
    }

    _inferType(id) {
        if (!id) return 'crazyflie';
        const s = String(id).toLowerCase();
        if (s.includes('x500') || s.includes('leader') || s.includes('heavy') || s.includes('cognitive')) return 'x500';
        if (s.includes('s500') || s.includes('patrol')) return 's500';
        if (s.includes('ugv') || s.includes('rover')) return 'ugv';
        const fleet = window.FLEET_CONFIG;
        if (fleet?.drones) {
            const d = fleet.drones.find(d => d.id === id);
            if (d) return d.type;
        }
        return 'crazyflie';
    }

    _inferRole(id, type) {
        if (!id) return 'scout';
        const s = String(id).toLowerCase();
        if (s.includes('leader') || s.includes('coord') || s.includes('x500_0') || s.includes('cognitive')) return 'coordinator';
        if (s.includes('patrol') || s.includes('s500')) return 'patrol';
        if (s.includes('ugv') || s.includes('rover')) return 'ugv';
        if (type === 'x500') return 'coordinator';
        if (type === 's500') return 'patrol';
        return 'scout';
    }

    _updateCoordinator(c) {
        const id = c.id || c.coord_id;
        if (!id) return;
        const role = c.role;
        const isCoord = role === 'coordinator' || role === 'leader' || id.includes('leader') || id.includes('x500_0');
        // Do not add scouts or patrols to coordinators map
        if (!isCoord && !this.coordinators.has(id)) {
            return;
        }

        const existing = this.coordinators.get(id);
        const idx = [...this.coordinators.keys()].indexOf(id);
        const colorIdx = idx >= 0 ? idx : this.coordinators.size;
        const color = this._coordColors[colorIdx % this._coordColors.length];

        this.coordinators.set(id, {
            x: c.zone?.n ?? c.position_n ?? existing?.x ?? 0,
            z: c.zone?.e ?? c.position_e ?? existing?.z ?? 0,
            role: isCoord ? (role || 'coordinator') : 'scout',
            coverage: c.localCoverage ?? c.local_coverage ?? existing?.coverage ?? 0,
            color: existing?.color || color,
        });
    }

    _markCoverage(worldX, worldZ, radius = 5.0) {
        // Synchronize zoneSize with doctrine if available
        if (window.DIAMANTS_DOCTRINE?.zoneParams?.sizeX) {
            this.zoneSize = window.DIAMANTS_DOCTRINE.zoneParams.sizeX;
        }
        const half = this.zoneSize / 2;
        const cellSize = this.zoneSize / this.gridDim;
        const ci = Math.floor((worldZ + half) / cellSize); // row (Z)
        const cj = Math.floor((worldX + half) / cellSize); // col (X)
        const r = Math.max(1, Math.round(radius / cellSize));

        for (let di = -r; di <= r; di++) {
            for (let dj = -r; dj <= r; dj++) {
                if (di * di + dj * dj <= r * r) {
                    const ni = ci + di;
                    const nj = cj + dj;
                    if (ni >= 0 && ni < this.gridDim && nj >= 0 && nj < this.gridDim) {
                        const idx = ni * this.gridDim + nj;
                        this.coverageGrid[idx] = Math.min(1.0, this.coverageGrid[idx] + 0.15);
                    }
                }
            }
        }
    }

    _setupClickToggle() {
        if (!this.canvas) return;
        this.canvas.addEventListener('click', () => {
            this._mode = this._mode === 'heatmap' ? 'convergence' : 'heatmap';
            this._draw();
        });
    }

    // =========================================================================
    // Simulation & MARL Synchronization
    // =========================================================================

    _syncWithSimulation() {
        // 1. Sync permanent visited cells from flight engine
        if (window.DIAMANTS_VISITED_CELLS && window.DIAMANTS_VISITED_CELLS.size > this._visitedSyncedSize) {
            const cellSize = window.DIAMANTS_CELL_SIZE || 2;
            const half = this.zoneSize / 2;
            const gridCellW = this.zoneSize / this.gridDim;
            for (const key of window.DIAMANTS_VISITED_CELLS) {
                const sep = key.indexOf(',');
                if (sep === -1) continue;
                const cx = +key.slice(0, sep);
                const cz = +key.slice(sep + 1);
                const wx = cx * cellSize;
                const wz = cz * cellSize;
                const gi = Math.floor((wz + half) / gridCellW);
                const gj = Math.floor((wx + half) / gridCellW);
                if (gi >= 0 && gi < this.gridDim && gj >= 0 && gj < this.gridDim) {
                    const idx = gi * this.gridDim + gj;
                    this.coverageGrid[idx] = Math.max(this.coverageGrid[idx], 0.65);
                }
            }
            this._visitedSyncedSize = window.DIAMANTS_VISITED_CELLS.size;
        }

        // 2. Fallback: if dronePositions is empty, check autonomousFlightEngine
        if (this.dronePositions.size === 0) {
            const fe = window.diamantsSystem?.integratedController?.autonomousFlightEngine;
            if (fe?.drones) {
                for (const [id, st] of fe.drones) {
                    if (st.position) {
                        this.updateDronePosition(id, st.position, st.heading || 0, st.role || null);
                    }
                }
            }
        }

        // 3. Ensure at least one coordinator exists if drones are present
        if (this.coordinators.size === 0 && this.dronePositions.size > 0) {
            let elected = null;
            for (const [id, dp] of this.dronePositions) {
                if (dp.type === 'x500' || dp.role === 'coordinator' || dp.role === 'leader' || id.includes('x500')) {
                    elected = { id, dp };
                    break;
                }
            }
            if (!elected) {
                const firstId = this.dronePositions.keys().next().value;
                elected = { id: firstId, dp: this.dronePositions.get(firstId) };
            }
            if (elected) {
                this._updateCoordinator({
                    id: elected.id,
                    position_n: elected.dp.x,
                    position_e: elected.dp.z,
                    role: 'coordinator',
                    localCoverage: this.globalCoverage
                });
            }
        }

        // 4. Update coordinator positions to follow their lead drones
        for (const [id, coord] of this.coordinators) {
            const dp = this.dronePositions.get(id);
            if (dp) {
                coord.x = dp.x;
                coord.z = dp.z;
            }
        }

        // 5. MARL / FedAvg training progress
        const ic = window.diamantsSystem?.integratedController;
        const mac = ic?.multiAgentCoordinator;
        if (mac && mac.globalSteps > 0) {
            this.currentRound = mac.globalSteps;
            let sumR = 0, countR = 0;
            if (mac.agents) {
                for (const [, ag] of mac.agents) {
                    const r = ag.brain?.avgReward ?? ag.episodeReward;
                    if (r !== undefined && !isNaN(r)) {
                        sumR += r;
                        countR++;
                    }
                }
            }
            if (countR > 0) this._lastAvgReward = sumR / countR;
        }

        // 6. Advance FedAvg rounds & record convergence history
        const now = performance.now();
        if (now - this._lastRoundTime > 2000) {
            const covRatio = this._gridCoveragePct() / 100;
            this.globalCoverage = covRatio;

            // Advance round counter if swarm is exploring
            if (this.dronePositions.size > 0) {
                this.currentRound++;
                const prevCov = this.roundHistory[this.roundHistory.length - 1]?.coverage || 0;
                const dCov = Math.max(0, covRatio - prevCov);
                // Reward formulation: base progress + delta coverage reward
                const reward = Math.min(1.0, 0.20 + covRatio * 0.70 + dCov * 2.0);
                this._lastAvgReward = (this._lastAvgReward * 0.7) + (reward * 0.3);

                this._recordRound(this.currentRound, covRatio, this._lastAvgReward);
                this._lastRoundTime = now;
            }
        }
    }

    // =========================================================================
    // Render Loop
    // =========================================================================

    _startRenderLoop() {
        setInterval(() => {
            const el = this.canvas || this.ctx?.canvas || null;
            if (el && el.getClientRects().length === 0 && !el._viewerShowing) return;
            this._draw();
        }, 200); // 5 FPS is smooth and lightweight
    }

    _draw() {
        if (!this.ctx || !this._offCtx) return;
        if (!this._syncSize()) return;

        const ctx = this._offCtx;
        const dpr = this._dpr || 1;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

        const w = this._cssW;
        const h = this._cssH;

        ctx.clearRect(0, 0, w, h);

        // Sync state from simulation
        this._syncWithSimulation();

        if (this._mode === 'heatmap') {
            this._drawHeatmap(ctx, w, h);
        } else {
            this._drawConvergence(ctx, w, h);
        }

        // Blit to visible canvas with 1:1 transform
        this.ctx.setTransform(1, 0, 0, 1, 0, 0);
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        this.ctx.drawImage(this._off, 0, 0);
    }

    // =========================================================================
    // Heatmap Mode
    // =========================================================================

    _drawHeatmap(ctx, w, h) {
        const half = this.zoneSize / 2;
        const scaleX = w / this.zoneSize;
        const scaleY = h / this.zoneSize;
        const cellW = w / this.gridDim;
        const cellH = h / this.gridDim;

        // 1. Tactical Deep Space Background
        ctx.fillStyle = '#080c18';
        ctx.fillRect(0, 0, w, h);

        // 2. Tactical Theater Grid Lines (every 40 meters)
        ctx.strokeStyle = 'rgba(30, 58, 138, 0.25)';
        ctx.lineWidth = 1;
        const gridStep = 40 * scaleX;
        for (let x = (half % 40) * scaleX; x < w; x += gridStep) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, h);
            ctx.stroke();
        }
        for (let y = (half % 40) * scaleY; y < h; y += gridStep) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(w, y);
            ctx.stroke();
        }

        // Center crosshair (0,0)
        const cx0 = half * scaleX;
        const cy0 = half * scaleY;
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
        ctx.beginPath();
        ctx.moveTo(cx0 - 8, cy0);
        ctx.lineTo(cx0 + 8, cy0);
        ctx.moveTo(cx0, cy0 - 8);
        ctx.lineTo(cx0, cy0 + 8);
        ctx.stroke();

        // Cardinal markers (N, S, E, W)
        ctx.fillStyle = 'rgba(148, 163, 184, 0.4)';
        ctx.font = '8px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('N', cx0, 26);
        ctx.fillText('S', cx0, h - 16);
        ctx.fillText('W', 12, cy0 + 3);
        ctx.fillText('E', w - 12, cy0 + 3);

        // 3. Coverage Heatmap Grid
        for (let i = 0; i < this.gridDim; i++) {
            for (let j = 0; j < this.gridDim; j++) {
                const val = this.coverageGrid[i * this.gridDim + j];
                if (val > 0.02) {
                    const intensity = Math.min(1.0, val);
                    let r, g, b, a;
                    if (intensity < 0.4) {
                        // Emerald green
                        r = Math.floor(16 + intensity * 40);
                        g = Math.floor(140 + intensity * 90);
                        b = Math.floor(80 + intensity * 60);
                        a = 0.25 + intensity * 0.45;
                    } else if (intensity < 0.8) {
                        // Cyan / Teal
                        r = Math.floor(6 + intensity * 50);
                        g = Math.floor(180 + intensity * 60);
                        b = Math.floor(212 + intensity * 40);
                        a = 0.45 + intensity * 0.35;
                    } else {
                        // Warm Gold / White core
                        r = Math.floor(245 + intensity * 10);
                        g = Math.floor(158 + intensity * 90);
                        b = Math.floor(11 + intensity * 150);
                        a = 0.70 + intensity * 0.25;
                    }
                    ctx.fillStyle = `rgba(${r}, ${g}, ${b}, ${a})`;
                    ctx.fillRect(j * cellW, i * cellH, cellW + 0.6, cellH + 0.6);
                }
            }
        }

        // 4. Federated Aggregation Links (Scouts ↔ Nearest Coordinator)
        if (this.coordinators.size > 0) {
            ctx.setLineDash([3, 4]);
            ctx.lineWidth = 1;
            for (const [id, pos] of this.dronePositions) {
                if (this.coordinators.has(id)) continue;
                const px = (pos.x + half) * scaleX;
                const py = (pos.z + half) * scaleY;

                // Find closest coordinator
                let closest = null, minDist = Infinity;
                for (const [, coord] of this.coordinators) {
                    const cpx = (coord.x + half) * scaleX;
                    const cpy = (coord.z + half) * scaleY;
                    const d = Math.hypot(cpx - px, cpy - py);
                    if (d < minDist) {
                        minDist = d;
                        closest = { cpx, cpy, color: coord.color };
                    }
                }

                if (closest && minDist < 180) {
                    ctx.strokeStyle = closest.color + '44';
                    ctx.beginPath();
                    ctx.moveTo(px, py);
                    ctx.lineTo(closest.cpx, closest.cpy);
                    ctx.stroke();
                }
            }
            ctx.setLineDash([]);
        }

        // 5. Coordinator Zones (Influence Circles & Voronoi Partitions)
        for (const [id, coord] of this.coordinators) {
            const px = (coord.x + half) * scaleX;
            const py = (coord.z + half) * scaleY;

            // Zone radius ~35m in world coordinates
            const zoneR = 35 * scaleX;
            ctx.beginPath();
            ctx.arc(px, py, zoneR, 0, Math.PI * 2);
            ctx.strokeStyle = coord.color + '55';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 4]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Soft radial glow
            const grad = ctx.createRadialGradient(px, py, 4, px, py, zoneR);
            grad.addColorStop(0, coord.color + '22');
            grad.addColorStop(1, coord.color + '00');
            ctx.fillStyle = grad;
            ctx.beginPath();
            ctx.arc(px, py, zoneR, 0, Math.PI * 2);
            ctx.fill();

            // Coordinator Marker: Upward Triangle with halo
            ctx.beginPath();
            ctx.moveTo(px, py - 8);
            ctx.lineTo(px + 7, py + 5);
            ctx.lineTo(px - 7, py + 5);
            ctx.closePath();
            ctx.fillStyle = coord.color;
            ctx.fill();
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1.5;
            ctx.stroke();

            // Coordinator Short NATO Label
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 8px monospace';
            ctx.textAlign = 'center';
            const shortId = id.replace(/drone_|x500_|crazyflie_/g, 'X').toUpperCase();
            ctx.fillText(shortId.slice(0, 5), px, py + 16);

            // Circular Local Coverage Ring
            const covRatio = coord.coverage || this.globalCoverage || 0;
            if (covRatio > 0.01) {
                ctx.beginPath();
                ctx.arc(px, py, 11, -Math.PI / 2, -Math.PI / 2 + Math.PI * 2 * Math.min(1.0, covRatio));
                ctx.strokeStyle = coord.color;
                ctx.lineWidth = 2;
                ctx.stroke();
            }
        }

        // 6. Scout & Patrol Drone Markers
        for (const [id, pos] of this.dronePositions) {
            if (this.coordinators.has(id)) continue;
            const px = (pos.x + half) * scaleX;
            const py = (pos.z + half) * scaleY;
            if (px < -10 || px > w + 10 || py < -10 || py > h + 10) continue;

            const isPatrol = pos.role === 'patrol' || pos.type === 's500';
            const isUgv = pos.role === 'ugv' || pos.type === 'ugv';

            ctx.save();
            ctx.translate(px, py);

            if (isPatrol) {
                // Amber Diamond for Patrol
                ctx.beginPath();
                ctx.moveTo(0, -5);
                ctx.lineTo(4, 0);
                ctx.lineTo(0, 5);
                ctx.lineTo(-4, 0);
                ctx.closePath();
                ctx.fillStyle = '#f59e0b';
                ctx.fill();
                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 1;
                ctx.stroke();
            } else if (isUgv) {
                // Cyan Square for Ground Rover
                ctx.fillStyle = '#38bdf8';
                ctx.fillRect(-3.5, -3.5, 7, 7);
                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 1;
                ctx.strokeRect(-3.5, -3.5, 7, 7);
            } else {
                // Emerald Chevron / Dot for Scout Drone with Heading
                ctx.rotate(pos.heading || 0);
                ctx.beginPath();
                ctx.moveTo(0, -4.5);
                ctx.lineTo(3.5, 3.5);
                ctx.lineTo(0, 1.5);
                ctx.lineTo(-3.5, 3.5);
                ctx.closePath();
                ctx.fillStyle = '#34d399';
                ctx.fill();
                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 0.8;
                ctx.stroke();
            }
            ctx.restore();

            // Short Label
            ctx.fillStyle = '#94a3b8';
            ctx.font = '7px monospace';
            ctx.textAlign = 'center';
            const shortName = id.replace(/drone_|crazyflie_/g, 'C').replace('ugv_', 'U').toUpperCase();
            ctx.fillText(shortName.slice(0, 4), px, py + 12);
        }

        // 7. HUD Status Header (Top)
        ctx.fillStyle = 'rgba(8, 12, 24, 0.85)';
        ctx.fillRect(0, 0, w, 20);
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.2)';
        ctx.beginPath();
        ctx.moveTo(0, 20);
        ctx.lineTo(w, 20);
        ctx.stroke();

        ctx.fillStyle = '#e2e8f0';
        ctx.font = 'bold 9px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`Fed R:${this.currentRound} | ${this.strategy.toUpperCase()}`, 6, 13);

        // Center agent count
        ctx.fillStyle = '#94a3b8';
        ctx.textAlign = 'center';
        ctx.fillText(`${this.dronePositions.size} clients`, w / 2, 13);

        // Right coverage percent
        const covPct = Math.max(Math.round(this.globalCoverage * 100), this._gridCoveragePct());
        ctx.fillStyle = covPct > 60 ? '#10b981' : covPct > 25 ? '#fbbf24' : '#f87171';
        ctx.textAlign = 'right';
        ctx.fillText(`Cov: ${covPct}%`, w - 6, 13);

        // 8. HUD Footer Indicator (Bottom)
        ctx.fillStyle = 'rgba(8, 12, 24, 0.7)';
        ctx.fillRect(0, h - 16, w, 16);
        ctx.fillStyle = '#64748b';
        ctx.font = '8px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('▸ click: convergence curve (FedAvg)', w / 2, h - 5);
    }

    // =========================================================================
    // Convergence Mode
    // =========================================================================

    _drawConvergence(ctx, w, h) {
        ctx.fillStyle = '#080c18';
        ctx.fillRect(0, 0, w, h);

        const margin = { top: 26, bottom: 26, left: 32, right: 12 };
        const plotW = Math.max(20, w - margin.left - margin.right);
        const plotH = Math.max(20, h - margin.top - margin.bottom);

        // Title
        ctx.fillStyle = '#e2e8f0';
        ctx.font = 'bold 10px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`FedAvg Convergence [R:${this.currentRound}]`, margin.left, 16);

        // Axes
        ctx.strokeStyle = '#334155';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(margin.left, margin.top);
        ctx.lineTo(margin.left, margin.top + plotH);
        ctx.lineTo(margin.left + plotW, margin.top + plotH);
        ctx.stroke();

        // Horizontal Grid lines & Y-axis labels (0%, 25%, 50%, 75%, 100%)
        ctx.fillStyle = '#64748b';
        ctx.font = '8px monospace';
        ctx.textAlign = 'right';
        for (let p = 0; p <= 100; p += 25) {
            const y = margin.top + plotH - (p / 100) * plotH;
            ctx.fillText(`${p}%`, margin.left - 4, y + 3);
            ctx.strokeStyle = 'rgba(51, 65, 85, 0.35)';
            ctx.beginPath();
            ctx.moveTo(margin.left, y);
            ctx.lineTo(margin.left + plotW, y);
            ctx.stroke();
        }

        // Empty state fallback: ensure synthetic baseline history if newly started
        if (this.roundHistory.length < 2) {
            const currentCov = this.globalCoverage || 0.05;
            this._recordRound(Math.max(0, this.currentRound - 1), currentCov * 0.7, 0.15);
            this._recordRound(this.currentRound, currentCov, this._lastAvgReward || 0.25);
        }

        // Plot curves
        const nPoints = this.roundHistory.length;
        const stepX = plotW / Math.max(1, nPoints - 1);

        // 1. Swarm Coverage Curve (Emerald #10b981)
        ctx.beginPath();
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = 2;
        for (let i = 0; i < nPoints; i++) {
            const x = margin.left + i * stepX;
            const y = margin.top + plotH - Math.min(1.0, this.roundHistory[i].coverage) * plotH;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // 2. Mean Swarm Reward Curve (Violet #a78bfa)
        const maxReward = Math.max(1, ...this.roundHistory.map(r => Math.abs(r.avgReward || 0)));
        ctx.beginPath();
        ctx.strokeStyle = '#a78bfa';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([3, 3]);
        for (let i = 0; i < nPoints; i++) {
            const x = margin.left + i * stepX;
            const rNorm = Math.min(1.0, Math.max(0, (this.roundHistory[i].avgReward || 0) / maxReward));
            const y = margin.top + plotH - rNorm * plotH;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);

        // Legend at bottom
        const legY = margin.top + plotH + 16;
        ctx.fillStyle = '#10b981';
        ctx.fillRect(margin.left, legY - 5, 8, 4);
        ctx.fillStyle = '#94a3b8';
        ctx.font = '8px monospace';
        ctx.textAlign = 'left';
        ctx.fillText('Coverage', margin.left + 12, legY);

        ctx.fillStyle = '#a78bfa';
        ctx.fillRect(margin.left + 65, legY - 5, 8, 4);
        ctx.fillStyle = '#94a3b8';
        ctx.fillText('Mean Reward', margin.left + 77, legY);

        // Right status
        ctx.fillStyle = '#64748b';
        ctx.textAlign = 'right';
        ctx.fillText('▸ click: heatmap', w - margin.right, legY);
    }

    _gridCoveragePct() {
        let filled = 0;
        for (let i = 0; i < this.coverageGrid.length; i++) {
            if (this.coverageGrid[i] > 0.02) filled++;
        }
        return Math.round((filled / this.coverageGrid.length) * 100);
    }

    pushRound(round, coverage, avgReward) {
        this.currentRound = round;
        this.globalCoverage = coverage;
        this._recordRound(round, coverage, avgReward);
    }

    reset() {
        this.coverageGrid.fill(0);
        this.coordinators.clear();
        this.dronePositions.clear();
        this.roundHistory = [];
        this.currentRound = 0;
        this.globalCoverage = 0;
        this._visitedSyncedSize = 0;
    }
}
