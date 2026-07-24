/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Swarm Coordination RL Panel
 * =========================================
 * Panneau latéral gauche affichant l'état temps réel de la coordination
 * RL fédérée des coordinateurs X500/S500.
 *
 * Affiche:
 *   - Role de chaque coordinateur (coordinator / patrol)
 *   - Zone assignée + couverture locale
 *   - Action RL courante (dx, dz, spread, signal)
 *   - Scouts rattachés
 *   - Latence inférence
 *   - État de la fédération (round, strategy, global coverage)
 *
 * Toggle: touche [B] (Brain)
 * Écoute: diamants:swarm-coordination, diamants:drone-positions, diamants:mission-status
 */

import { makeDraggable } from './panel-utils.js';

const log = (...a) => console.log('[SwarmRL]', ...a);

/**
 * @typedef {Object} CoordinatorState
 * @property {string} id
 * @property {string} role
 * @property {{n:number, e:number}} zone
 * @property {number} localCoverage
 * @property {number} inferenceMs
 * @property {{dx:number, dz:number, spread:number, signal:number}} action
 * @property {string[]} scouts
 * @property {boolean} rlAvailable
 */

export class SwarmCoordinationPanel {
    constructor() {
        /** @type {Map<string, CoordinatorState>} */
        this.coordinators = new Map();
        this.globalCoverage = 0;
        this.federationRound = 0;
        this.strategy = 'fedavg';
        this.totalInfluences = 0;
        this.rlDecisions = 0;
        this.heuristicFallback = 0;

        this._visible = false;
        this._root = null;
        this._body = null;
        this._badge = null;
        this._lastUpdate = 0;

        this._buildDOM();
        this._hookEvents();
        this._hookKeyboard();

        log('✅ SwarmCoordinationPanel initialized');
    }

    // =========================================================================
    // DOM Construction
    // =========================================================================

    _buildDOM() {
        this._root = document.createElement('div');
        this._root.id = 'swarm-rl-panel';
        Object.assign(this._root.style, {
            position: 'fixed',
            top: '60px',
            left: '10px',
            width: '300px',
            maxHeight: 'calc(100vh - 80px)',
            background: 'rgba(5, 12, 24, 0.96)',
            border: '1.5px solid #7c3aed',
            borderRadius: '10px',
            fontFamily: "'JetBrains Mono', 'Fira Code', 'Consolas', monospace",
            fontSize: '11px',
            color: '#d4d4d8',
            zIndex: '5000',
            display: 'none',
            flexDirection: 'column',
            backdropFilter: 'blur(8px)',
            boxShadow: '0 4px 24px rgba(124, 58, 237, 0.3)',
            overflow: 'hidden',
        });

        // ── Header ──
        const header = document.createElement('div');
        Object.assign(header.style, {
            padding: '8px 12px',
            background: 'linear-gradient(90deg, #5b21b6, #7c3aed)',
            color: '#fff',
            fontWeight: '700',
            fontSize: '11.5px',
            letterSpacing: '0.5px',
            textTransform: 'uppercase',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            cursor: 'pointer',
            userSelect: 'none',
        });
        header.textContent = '🧠 Swarm RL Coordination';
        header.addEventListener('dblclick', () => this.hide());

        // Status badge
        this._badge = document.createElement('span');
        Object.assign(this._badge.style, {
            fontSize: '9px',
            padding: '2px 6px',
            borderRadius: '4px',
            background: 'rgba(255,255,255,0.15)',
        });
        this._badge.textContent = 'IDLE';
        header.appendChild(this._badge);
        this._root.appendChild(header);

        // ── Federation summary bar ──
        this._fedBar = document.createElement('div');
        Object.assign(this._fedBar.style, {
            padding: '6px 12px',
            background: 'rgba(124, 58, 237, 0.12)',
            borderBottom: '1px solid rgba(124, 58, 237, 0.3)',
            fontSize: '10px',
            display: 'flex',
            justifyContent: 'space-between',
            gap: '8px',
        });
        this._fedBar.innerHTML = `
            <span>Fed: <b id="srl-fed-round">0</b></span>
            <span>Cov: <b id="srl-global-cov">0%</b></span>
            <span>RL: <b id="srl-rl-count">0</b></span>
            <span>Inf: <b id="srl-total-inf">0</b></span>
        `;
        this._root.appendChild(this._fedBar);

        // ── Body: coordinator cards ──
        this._body = document.createElement('div');
        Object.assign(this._body.style, {
            padding: '6px',
            overflowY: 'auto',
            flex: '1',
            maxHeight: 'calc(100vh - 180px)',
        });
        this._root.appendChild(this._body);

        // ── Footer: legend ──
        const footer = document.createElement('div');
        Object.assign(footer.style, {
            padding: '4px 12px',
            borderTop: '1px solid rgba(124, 58, 237, 0.2)',
            fontSize: '9px',
            color: '#888',
            textAlign: 'center',
        });
        footer.textContent = '[B] Toggle | Coordinateurs X500/S500 | Scouts Crazyflie';
        this._root.appendChild(footer);

        document.body.appendChild(this._root);

        // ── Drag support ──
        makeDraggable(this._root, header);
    }

    // =========================================================================
    // Coordinator Card
    // =========================================================================

    _buildCoordCard(coord) {
        const card = document.createElement('div');
        Object.assign(card.style, {
            background: 'rgba(30, 30, 50, 0.7)',
            border: `1px solid ${coord.role === 'patrol' ? '#f59e0b' : '#7c3aed'}`,
            borderRadius: '6px',
            padding: '8px 10px',
            marginBottom: '6px',
        });

        const roleColor = coord.role === 'patrol' ? '#f59e0b' : '#a78bfa';
        const roleIcon = coord.role === 'patrol' ? '🛡️' : '🎯';
        const rlStatus = coord.rlAvailable ? '🟢 RL' : '🟡 Heuristic';

        // Coverage bar
        const covPct = Math.round((coord.localCoverage || 0) * 100);
        const covColor = covPct > 70 ? '#10b981' : covPct > 40 ? '#f59e0b' : '#ef4444';

        // Action values
        const a = coord.action || { dx: 0, dz: 0, spread: 5, signal: 0.5 };

        card.innerHTML = `
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:4px;">
                <span style="color:${roleColor}; font-weight:700;">${roleIcon} ${coord.id}</span>
                <span style="font-size:9px; color:#888;">${rlStatus} · ${(coord.inferenceMs || 0).toFixed(1)}ms</span>
            </div>
            <div style="display:flex; gap:8px; font-size:10px; margin-bottom:4px;">
                <span>Zone: <b>(${(coord.zone?.n || 0).toFixed(0)}, ${(coord.zone?.e || 0).toFixed(0)})</b></span>
                <span>Role: <b style="color:${roleColor}">${coord.role}</b></span>
            </div>
            <div style="margin-bottom:4px;">
                <div style="display:flex; justify-content:space-between; font-size:10px;">
                    <span>Couverture locale</span>
                    <span style="color:${covColor}">${covPct}%</span>
                </div>
                <div style="height:4px; background:#1a1a2e; border-radius:2px; overflow:hidden;">
                    <div style="height:100%; width:${covPct}%; background:${covColor}; border-radius:2px; transition:width 0.5s;"></div>
                </div>
            </div>
            <div style="display:grid; grid-template-columns:1fr 1fr; gap:3px 10px; font-size:10px; color:#aaa;">
                <span>dx: <b style="color:#c4b5fd">${a.dx.toFixed(1)}m</b></span>
                <span>dz: <b style="color:#c4b5fd">${a.dz.toFixed(1)}m</b></span>
                <span>spread: <b style="color:#67e8f9">${a.spread.toFixed(1)}m</b></span>
                <span>signal: <b style="color:#fbbf24">${(a.signal * 100).toFixed(0)}%</b></span>
            </div>
            ${coord.scouts && coord.scouts.length > 0 ? `
                <div style="margin-top:4px; font-size:9px; color:#6b7280;">
                    Scouts: ${coord.scouts.map(s =>
                        `<span style="color:#34d399; margin-right:4px;">●${s}</span>`
                    ).join('')}
                </div>
            ` : ''}
        `;

        return card;
    }

    // =========================================================================
    // Rendering
    // =========================================================================

    _render() {
        if (!this._visible) return;

        // Update summary bar
        const fedRound = this._root.querySelector('#srl-fed-round');
        const globalCov = this._root.querySelector('#srl-global-cov');
        const rlCount = this._root.querySelector('#srl-rl-count');
        const totalInf = this._root.querySelector('#srl-total-inf');

        if (fedRound) fedRound.textContent = this.federationRound;
        if (globalCov) globalCov.textContent = `${Math.round(this.globalCoverage * 100)}%`;
        if (rlCount) rlCount.textContent = this.rlDecisions;
        if (totalInf) totalInf.textContent = this.totalInfluences;

        // Update badge
        const nCoord = this.coordinators.size;
        const nRL = [...this.coordinators.values()].filter(c => c.rlAvailable).length;
        this._badge.textContent = nRL > 0 ? `${nRL}/${nCoord} RL` : `${nCoord} HEUR`;
        this._badge.style.background = nRL > 0 ? 'rgba(16, 185, 129, 0.3)' : 'rgba(245, 158, 11, 0.3)';

        // Rebuild coordinator cards
        this._body.innerHTML = '';

        if (this.coordinators.size === 0) {
            const empty = document.createElement('div');
            Object.assign(empty.style, {
                textAlign: 'center',
                padding: '20px',
                color: '#555',
                fontSize: '10px',
            });
            empty.innerHTML = 'No coordinator detected<br><span style="font-size:9px;color:#444">Les X500/S500 apparaîtront ici</span>';
            this._body.appendChild(empty);
            return;
        }

        // Sort: coordinators first, then patrol
        const sorted = [...this.coordinators.values()].sort((a, b) => {
            if (a.role === b.role) return a.id.localeCompare(b.id);
            return a.role === 'coordinator' ? -1 : 1;
        });

        for (const coord of sorted) {
            this._body.appendChild(this._buildCoordCard(coord));
        }
    }

    // =========================================================================
    // Events
    // =========================================================================

    _hookEvents() {
        // Main data source: swarm coordination updates
        window.addEventListener('diamants:swarm-coordination', (evt) => {
            const d = evt.detail;
            if (d.coordinators) {
                for (const c of d.coordinators) {
                    this.coordinators.set(c.id, c);
                }
            }
            if (d.globalCoverage !== undefined) this.globalCoverage = d.globalCoverage;
            if (d.federationRound !== undefined) this.federationRound = d.federationRound;
            if (d.strategy) this.strategy = d.strategy;
            if (d.totalInfluences !== undefined) this.totalInfluences = d.totalInfluences;
            if (d.rlDecisions !== undefined) this.rlDecisions = d.rlDecisions;
            if (d.heuristicFallback !== undefined) this.heuristicFallback = d.heuristicFallback;
            this._render();
        });

        // Derive coordinator info from drone positions when no backend
        window.addEventListener('diamants:drone-positions', (evt) => {
            const now = Date.now();
            if (now - this._lastUpdate < 500) return; // throttle
            this._lastUpdate = now;
            this._deriveFromPositions(evt.detail);
        });

        // Mission status for coverage
        window.addEventListener('diamants:mission-status', (evt) => {
            const d = evt.detail;
            if (d.coverage !== undefined) {
                this.globalCoverage = d.coverage;
            }
        });
    }

    /**
     * When no backend is connected, derive coordinator states from
     * drone positions + fleet config. This makes the panel useful
     * even in pure frontend simulation.
     */
    _deriveFromPositions(positions) {
        if (!positions || typeof positions !== 'object') return;

        const fleet = window.FLEET_CONFIG;
        if (!fleet?.drones) return;

        // Build coordinator map from fleet config
        for (const drone of fleet.drones) {
            const dtype = (drone.type || '').toLowerCase();
            if (dtype !== 'x500' && dtype !== 's500') continue;

            const did = drone.id;
            const pos = positions[did];

            // Get or create coordinator state
            let coord = this.coordinators.get(did);
            if (!coord) {
                coord = {
                    id: did,
                    role: dtype === 's500' ? 'patrol' : 'coordinator',
                    zone: { n: 0, e: 0 },
                    localCoverage: 0,
                    inferenceMs: 0,
                    action: { dx: 0, dz: 0, spread: 5, signal: 0.5 },
                    scouts: [],
                    rlAvailable: false,
                };
                this.coordinators.set(did, coord);
            }

            // Update position from simulation
            if (pos?.position) {
                const p = pos.position;
                coord.zone = { n: p.x || p.n || 0, e: p.z || p.e || 0 };
            }
        }

        // Assign scouts
        const coordIds = [...this.coordinators.keys()];
        if (coordIds.length === 0) return;

        let scoutIdx = 0;
        for (const drone of fleet.drones) {
            const dtype = (drone.type || '').toLowerCase();
            if (dtype === 'x500' || dtype === 's500') continue;
            const assignTo = coordIds[scoutIdx % coordIds.length];
            const coord = this.coordinators.get(assignTo);
            if (coord && !coord.scouts.includes(drone.id)) {
                coord.scouts.push(drone.id);
            }
            scoutIdx++;
        }

        this._render();
    }

    _hookKeyboard() {
        document.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
            if (e.code === 'KeyB') {
                e.preventDefault();
                this.toggle();
            }
        });
    }

    // =========================================================================
    // Visibility
    // =========================================================================

    show() {
        this._visible = true;
        this._root.style.display = 'flex';
        this._render();
    }

    hide() {
        this._visible = false;
        this._root.style.display = 'none';
    }

    toggle() {
        this._visible ? this.hide() : this.show();
    }
}
