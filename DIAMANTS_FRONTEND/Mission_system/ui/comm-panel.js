/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Communication Panel
 * ================================
 * Canvas minimap affichant les communications inter-drones en temps réel:
 *   - Liens de communication actifs (beams entre drones)
 *   - Type de message (MAP_SYNC, DIRECTIVE, HEARTBEAT)
 *   - Connaissance locale de chaque drone (cells connues)
 *   - Flux P2P global (info flow ratio)
 *   - Statistiques de communication (syncs, directives, cells partagées)
 *
 * Canvas ID: comm_canvas
 *
 * Écoute:
 *   - diamants:drone-positions → positions temps réel
 *   - diamants:comm-event      → événements de communication
 *   - diamants:federated-update → round info
 */

const log = (...a) => console.log('[CommPanel]', ...a);

// ── Message type colors ──
const MSG_COLORS = {
    MAP_SYNC:  '#00ffd0',
    DIRECTIVE: '#ff8800',
    HEARTBEAT: '#4488aa',
    ALERT:     '#ff4444',
};

const MSG_ICONS = {
    MAP_SYNC:  '⇄',
    DIRECTIVE: '⚡',
    HEARTBEAT: '💓',
    ALERT:     '⚠',
};

export class CommPanel {
    constructor(canvasId = 'comm_canvas') {
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
        this.zoneSize = 80;               // meters
        this.dronePositions = new Map();   // id → {x, z, type}
        this.commLinks = [];              // [{from, to, type, age, cells}]
        this.maxLinks = 50;
        this.droneKnowledge = new Map();  // id → {cells, infoFlow}

        // ── Global stats ──
        this.stats = {
            mapSyncs: 0,
            directivesSent: 0,
            cellsShared: 0,
            globalInfoFlow: 0,
            activePairs: 0,
        };

        // ── Comm event log (scrollable) ──
        this._eventLog = [];             // [{timestamp, from, to, type, detail}]
        this._maxLogEntries = 30;

        // ── Mode toggle ──
        this._mode = 'network';           // 'network' | 'log'

        this._init();
        log('✅ CommPanel initialized');
    }

    _init() {
        this._hookEvents();
        this._setupClickToggle();
        this._startRenderLoop();

        window.DIAMANTS_COMM_PANEL = this;
    }

    // =========================================================================
    // Event Hooks
    // =========================================================================

    _hookEvents() {
        // Drone positions (from engine or coordinator)
        window.addEventListener('diamants:drone-positions', (evt) => {
            const detail = evt.detail;
            if (!detail) return;
            for (const [id, data] of Object.entries(detail)) {
                if (data.source !== 'engine') continue; // skip backend positions
                const pos = data.position || data;
                this.dronePositions.set(id, {
                    x: pos.x ?? 0,
                    z: pos.z ?? 0,
                    type: data.type || (id.startsWith('x500') || id.startsWith('s500') ? 'x500' : 'crazyflie'),
                });
            }
        });

        // Comm events (emitted by swarm-comm-manager)
        window.addEventListener('diamants:comm-event', (evt) => {
            const d = evt.detail;
            if (!d) return;
            this._addCommLink(d);
            this._addLogEntry(d);
        });

        // Also poll comm manager directly for stats
        this._pollInterval = setInterval(() => this._pollCommManager(), 1000);
    }

    _pollCommManager() {
        const cm = window._diamantsCommManager;
        if (!cm) return;

        // Update stats
        const stats = cm.getStats();
        if (stats) {
            this.stats.mapSyncs = stats.mapSyncs || 0;
            this.stats.directivesSent = stats.directivesSent || 0;
            this.stats.cellsShared = stats.cellsShared || 0;
        }
        this.stats.globalInfoFlow = cm.getGlobalInfoFlowRatio() || 0;

        // Update per-drone knowledge
        for (const [id] of this.dronePositions) {
            const knowledge = cm.getLocalKnowledge(id);
            const flow = cm.getInfoFlow(id);
            const details = cm.getCommDetails(id);
            this.droneKnowledge.set(id, {
                cells: knowledge,
                infoFlow: flow?.ratio || 0,
                active: details?.active || false,
                peers: details?.peers || [],
            });

            // Build active links from comm details
            if (details?.peers) {
                for (const peer of details.peers) {
                    if (peer.age < 2000) {
                        this._addCommLink({
                            from: id,
                            to: peer.id,
                            type: peer.dataType || 'MAP_SYNC',
                            cells: peer.cells || 0,
                        });
                    }
                }
            }
        }

        // Count active pairs
        const now = performance.now();
        this.stats.activePairs = this.commLinks.filter(l => now - l.timestamp < 2000).length;
    }

    _addCommLink(data) {
        const now = performance.now();
        // Deduplicate: update existing link if same pair within 500ms
        const existing = this.commLinks.find(
            l => l.from === data.from && l.to === data.to && (now - l.timestamp) < 500
        );
        if (existing) {
            existing.timestamp = now;
            existing.type = data.type || existing.type;
            existing.cells = data.cells || existing.cells;
            return;
        }

        this.commLinks.push({
            from: data.from || data.fromId,
            to: data.to || data.toId,
            type: data.type || 'MAP_SYNC',
            cells: data.cells || data.cellCount || 0,
            timestamp: now,
        });

        // Trim old links
        if (this.commLinks.length > this.maxLinks) {
            this.commLinks = this.commLinks.slice(-this.maxLinks);
        }
    }

    _addLogEntry(data) {
        this._eventLog.push({
            timestamp: performance.now(),
            from: data.from || data.fromId || '?',
            to: data.to || data.toId || '?',
            type: data.type || 'SYNC',
            detail: data.detail || `${data.cells || 0} cells`,
        });
        if (this._eventLog.length > this._maxLogEntries) {
            this._eventLog.shift();
        }
    }

    // =========================================================================
    // Click toggle between modes
    // =========================================================================

    _setupClickToggle() {
        if (!this.canvas) return;
        // Right-click to toggle mode
        this.canvas.addEventListener('contextmenu', (e) => {
            e.preventDefault();
            this._mode = this._mode === 'network' ? 'log' : 'network';
        });
    }

    // =========================================================================
    // Render Loop
    // =========================================================================

    _startRenderLoop() {
        if (!this.canvas) return;
        const fps = 4; // 4 Hz — low since comm updates at 2Hz
        setInterval(() => this._render(), 1000 / fps);
    }

    _render() {
        const ctx = this._offCtx;
        const W = this._off.width;
        const H = this._off.height;

        // Clear
        ctx.fillStyle = '#080C18';
        ctx.fillRect(0, 0, W, H);

        if (this._mode === 'network') {
            this._drawNetworkView(ctx, W, H);
        } else {
            this._drawLogView(ctx, W, H);
        }

        // Blit to visible canvas
        this.ctx.drawImage(this._off, 0, 0);
    }

    // =========================================================================
    // Network View — Top-down comm link visualization
    // =========================================================================

    _drawNetworkView(ctx, W, H) {
        const now = performance.now();

        // ── Autoscale: compute fleet center and max extent ──
        let cx = 0, cz = 0, count = 0;
        for (const pos of this.dronePositions.values()) {
            cx += pos.x; cz += pos.z; count++;
        }
        if (count > 0) { cx /= count; cz /= count; }

        // Find max distance from center to any drone
        let maxDist = 20; // minimum 20m radius
        for (const pos of this.dronePositions.values()) {
            const dx = pos.x - cx;
            const dz = pos.z - cz;
            const d = Math.sqrt(dx * dx + dz * dz);
            if (d > maxDist) maxDist = d;
        }

        // Add 25% margin so drones at the edge aren't on the border
        const range = maxDist * 1.25;
        // Smooth transition toward new zone size (avoid jitter)
        if (!this._adaptiveRange) this._adaptiveRange = range;
        this._adaptiveRange += (range - this._adaptiveRange) * 0.08;
        const half = this._adaptiveRange;

        const midX = W / 2;
        const midY = H / 2;
        const scale = Math.min(W, H) / 2 / half;

        // Helper: world → canvas coords (centered on fleet)
        const toX = (wx) => midX + (wx - cx) * scale;
        const toY = (wz) => midY + (wz - cz) * scale;

        // ── Grid (adaptive) ──
        ctx.strokeStyle = 'rgba(40, 60, 80, 0.3)';
        ctx.lineWidth = 0.5;
        const gridStep = half < 30 ? 5 : half < 60 ? 10 : 20;
        const gridMin = Math.floor((cx - half) / gridStep) * gridStep;
        const gridMax = Math.ceil((cx + half) / gridStep) * gridStep;
        for (let g = gridMin; g <= gridMax; g += gridStep) {
            const gx = toX(g);
            ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, H); ctx.stroke();
        }
        const gridMinZ = Math.floor((cz - half) / gridStep) * gridStep;
        const gridMaxZ = Math.ceil((cz + half) / gridStep) * gridStep;
        for (let g = gridMinZ; g <= gridMaxZ; g += gridStep) {
            const gy = toY(g);
            ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(W, gy); ctx.stroke();
        }

        // ── Center cross ──
        ctx.strokeStyle = 'rgba(60, 80, 100, 0.4)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(W / 2, 0); ctx.lineTo(W / 2, H);
        ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2);
        ctx.stroke();

        // ── Comm links ──
        for (const link of this.commLinks) {
            const age = now - link.timestamp;
            if (age > 3000) continue; // expire after 3s

            const fromPos = this.dronePositions.get(link.from);
            const toPos = this.dronePositions.get(link.to);
            if (!fromPos || !toPos) continue;

            const x1 = toX(fromPos.x);
            const y1 = toY(fromPos.z);
            const x2 = toX(toPos.x);
            const y2 = toY(toPos.z);

            const alpha = Math.max(0.1, 1.0 - age / 3000);
            const color = MSG_COLORS[link.type] || '#00ffd0';

            // Draw comm beam
            ctx.save();
            ctx.globalAlpha = alpha * 0.7;
            ctx.strokeStyle = color;
            ctx.lineWidth = link.type === 'DIRECTIVE' ? 2.5 : 1.5;
            ctx.setLineDash(link.type === 'HEARTBEAT' ? [3, 4] : []);
            ctx.beginPath();
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
            ctx.setLineDash([]);

            // Draw pulse at midpoint for MAP_SYNC
            if (link.type === 'MAP_SYNC' && age < 1500) {
                const mx = (x1 + x2) / 2;
                const my = (y1 + y2) / 2;
                const pulseR = 3 + (age / 1500) * 6;
                ctx.globalAlpha = alpha * 0.4;
                ctx.beginPath();
                ctx.arc(mx, my, pulseR, 0, Math.PI * 2);
                ctx.fillStyle = color;
                ctx.fill();
            }

            // Cell count label on active syncs
            if (link.cells > 0 && age < 1500) {
                const mx = (x1 + x2) / 2;
                const my = (y1 + y2) / 2 - 6;
                ctx.globalAlpha = alpha;
                ctx.font = 'bold 7px monospace';
                ctx.fillStyle = color;
                ctx.textAlign = 'center';
                ctx.fillText(`${link.cells}c`, mx, my);
            }

            ctx.restore();
        }

        // ── Drone dots ──
        for (const [id, pos] of this.dronePositions) {
            const dx = toX(pos.x);
            const dy = toY(pos.z);
            const isCF = pos.type === 'crazyflie';
            const knowledge = this.droneKnowledge.get(id);
            const isActive = knowledge?.active || false;

            // Comm range circle (faint)
            const range = isCF ? 10 : 30;
            const rangeR = (range / this.zoneSize) * W;
            ctx.save();
            ctx.globalAlpha = 0.08;
            ctx.beginPath();
            ctx.arc(dx, dy, rangeR, 0, Math.PI * 2);
            ctx.fillStyle = isActive ? '#00ffd0' : '#334466';
            ctx.fill();
            ctx.restore();

            // Drone dot
            const r = isCF ? 3 : 5;
            const dotColor = isCF ? '#00ff88' : '#4488ff';

            // Glow for active communicators
            if (isActive) {
                ctx.save();
                ctx.globalAlpha = 0.4;
                ctx.beginPath();
                ctx.arc(dx, dy, r + 4, 0, Math.PI * 2);
                ctx.fillStyle = '#00ffd0';
                ctx.fill();
                ctx.restore();
            }

            ctx.beginPath();
            ctx.arc(dx, dy, r, 0, Math.PI * 2);
            ctx.fillStyle = dotColor;
            ctx.fill();

            // Short label
            ctx.font = '6px monospace';
            ctx.fillStyle = '#8899aa';
            ctx.textAlign = 'center';
            const shortId = id.replace('crazyflie_', 'CF').replace('x500_', 'X5_').replace('s500_', 'S5_');
            ctx.fillText(shortId, dx, dy - r - 3);

            // Knowledge count
            if (knowledge && knowledge.cells > 0) {
                ctx.font = '5px monospace';
                ctx.fillStyle = '#557766';
                ctx.fillText(`${knowledge.cells}`, dx, dy + r + 6);
            }
        }

        // ── Stats overlay ──
        this._drawStatsOverlay(ctx, W, H);
    }

    _drawStatsOverlay(ctx, W, H) {
        const pad = 4;
        const lineH = 10;
        const x = pad;
        let y = H - pad;

        ctx.font = 'bold 8px monospace';
        ctx.textAlign = 'left';

        // Bottom-left stats
        const lines = [
            { label: '📡 Syncs:', value: `${this.stats.mapSyncs}`, color: '#00ffd0' },
            { label: '⚡ Dir:', value: `${this.stats.directivesSent}`, color: '#ff8800' },
            { label: '📦 Cells:', value: `${this.stats.cellsShared}`, color: '#aabbcc' },
            { label: '🔄 P2P:', value: `${(this.stats.globalInfoFlow * 100).toFixed(0)}%`, color: '#67e8f9' },
            { label: '🔗 Active:', value: `${this.stats.activePairs}`, color: '#a78bfa' },
        ];

        // Draw from bottom up
        for (let i = lines.length - 1; i >= 0; i--) {
            const line = lines[i];
            ctx.fillStyle = '#556677';
            ctx.fillText(line.label, x, y);
            ctx.fillStyle = line.color;
            ctx.fillText(line.value, x + ctx.measureText(line.label).width + 2, y);
            y -= lineH;
        }

        // Top-right: mode indicator
        ctx.font = '7px monospace';
        ctx.fillStyle = '#445566';
        ctx.textAlign = 'right';
        ctx.fillText('RÉSEAU', W - pad, 10);
        ctx.font = '5px monospace';
        ctx.fillStyle = '#334455';
        ctx.fillText('clic-droit: LOG', W - pad, 18);
    }

    // =========================================================================
    // Log View — Scrollable event feed
    // =========================================================================

    _drawLogView(ctx, W, H) {
        const now = performance.now();
        const pad = 6;
        const lineH = 12;
        let y = pad + 14;

        // Header
        ctx.font = 'bold 9px monospace';
        ctx.fillStyle = '#00ffd0';
        ctx.textAlign = 'left';
        ctx.fillText('📋 COMMUNICATION LOG', pad, pad + 8);

        ctx.font = '7px monospace';
        ctx.fillStyle = '#334455';
        ctx.textAlign = 'right';
        ctx.fillText('clic-droit: RÉSEAU', W - pad, pad + 8);

        // Divider
        ctx.strokeStyle = 'rgba(0, 255, 208, 0.2)';
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(pad, y); ctx.lineTo(W - pad, y);
        ctx.stroke();
        y += 4;

        // Column headers
        ctx.font = 'bold 6px monospace';
        ctx.fillStyle = '#556677';
        ctx.textAlign = 'left';
        ctx.fillText('TYPE', pad, y + 6);
        ctx.fillText('FROM → TO', pad + 50, y + 6);
        ctx.fillText('DÉTAIL', pad + 150, y + 6);
        ctx.fillText('AGE', W - pad - 20, y + 6);
        y += lineH;

        // Divider
        ctx.strokeStyle = 'rgba(50, 70, 90, 0.5)';
        ctx.beginPath();
        ctx.moveTo(pad, y); ctx.lineTo(W - pad, y);
        ctx.stroke();
        y += 2;

        // Log entries (most recent first)
        const maxVisible = Math.floor((H - y - pad) / lineH);
        const entries = this._eventLog.slice(-maxVisible).reverse();

        for (const entry of entries) {
            if (y > H - pad) break;
            const age = now - entry.timestamp;
            const alpha = Math.max(0.3, 1.0 - age / 10000);

            ctx.save();
            ctx.globalAlpha = alpha;

            // Type badge
            const typeColor = MSG_COLORS[entry.type] || '#aabbcc';
            const typeIcon = MSG_ICONS[entry.type] || '•';
            ctx.font = 'bold 7px monospace';
            ctx.fillStyle = typeColor;
            ctx.textAlign = 'left';
            ctx.fillText(`${typeIcon} ${(entry.type || '').slice(0, 6)}`, pad, y + 7);

            // From → To
            const fromShort = (entry.from || '').replace('crazyflie_', 'CF').replace('x500_', 'X5').replace('s500_', 'S5');
            const toShort = (entry.to || '').replace('crazyflie_', 'CF').replace('x500_', 'X5').replace('s500_', 'S5');
            ctx.font = '7px monospace';
            ctx.fillStyle = '#8899aa';
            ctx.fillText(`${fromShort}→${toShort}`, pad + 50, y + 7);

            // Detail
            ctx.fillStyle = '#667788';
            ctx.font = '6px monospace';
            const detailText = (entry.detail || '').slice(0, 20);
            ctx.fillText(detailText, pad + 150, y + 7);

            // Age
            const ageSec = (age / 1000).toFixed(0);
            ctx.fillStyle = '#445566';
            ctx.textAlign = 'right';
            ctx.fillText(`${ageSec}s`, W - pad, y + 7);

            ctx.restore();
            y += lineH;
        }

        // ── Bottom stats bar ──
        this._drawStatsOverlay(ctx, W, H);
    }

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * Force-refresh from comm manager.
     */
    refresh() {
        this._pollCommManager();
    }

    /**
     * Get current stats for external display.
     */
    getStats() {
        return { ...this.stats };
    }

    /**
     * Cleanup.
     */
    destroy() {
        if (this._pollInterval) clearInterval(this._pollInterval);
        window.DIAMANTS_COMM_PANEL = null;
    }
}
