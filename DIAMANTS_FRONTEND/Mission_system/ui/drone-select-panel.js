/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Drone Selection Panel (Tap-to-select)
 * ==================================================
 * Touch/click on a drone in the 3D scene to select it and show its info panel.
 * Works on mobile (touch) and desktop (click).
 * Shows: id, type, phase, comms, position, battery...
 */
import * as THREE from 'three';

const CSS = `
/* ═══ Drone Select Info Panel ═══ */
#drone-select-panel {
    position: fixed;
    bottom: 80px;
    left: 12px;
    width: 260px;
    max-height: 360px;
    background: rgba(10, 14, 23, 0.94);
    border: 1px solid rgba(0, 180, 255, 0.4);
    border-radius: 14px;
    padding: 14px;
    color: #f0f0f0;
    font-family: 'Inter', -apple-system, sans-serif;
    font-size: 13px;
    z-index: 7000;
    display: none;
    flex-direction: column;
    gap: 8px;
    box-shadow: 0 6px 30px rgba(0,0,0,0.6);
    backdrop-filter: blur(12px);
    -webkit-backdrop-filter: blur(12px);
    overflow-y: auto;
    touch-action: pan-y;
    -webkit-overflow-scrolling: touch;
}
#drone-select-panel.visible {
    display: flex;
}

#drone-select-panel .dsp-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    border-bottom: 1px solid rgba(255,255,255,0.1);
    padding-bottom: 8px;
}
#drone-select-panel .dsp-title {
    font-weight: 700;
    font-size: 15px;
    color: #00d4ff;
    display: flex;
    align-items: center;
    gap: 6px;
}
#drone-select-panel .dsp-close {
    width: 32px;
    height: 32px;
    border-radius: 50%;
    border: 1px solid rgba(255,255,255,0.15);
    background: rgba(255,255,255,0.06);
    color: #aaa;
    font-size: 16px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    touch-action: manipulation;
    -webkit-tap-highlight-color: transparent;
}
#drone-select-panel .dsp-close:active {
    background: rgba(255,255,255,0.2);
}

#drone-select-panel .dsp-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 12px;
    padding: 3px 0;
}
#drone-select-panel .dsp-label {
    color: #888;
    font-weight: 500;
}
#drone-select-panel .dsp-value {
    color: #e0e0e0;
    font-family: 'Roboto Mono', monospace;
    font-weight: 600;
    text-align: right;
    max-width: 140px;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
}

#drone-select-panel .dsp-comm-section {
    border-top: 1px solid rgba(255,255,255,0.08);
    padding-top: 6px;
    margin-top: 2px;
}
#drone-select-panel .dsp-comm-title {
    font-weight: 700;
    font-size: 12px;
    color: #10b981;
    margin-bottom: 4px;
}
#drone-select-panel .dsp-comm-msg {
    font-size: 11px;
    color: #ccc;
    background: rgba(255,255,255,0.04);
    border-radius: 6px;
    padding: 6px 8px;
    font-family: 'Roboto Mono', monospace;
    word-break: break-word;
    max-height: 80px;
    overflow-y: auto;
    line-height: 1.4;
}

#drone-select-panel .dsp-follow-btn {
    margin-top: 4px;
    padding: 10px;
    border: 1px solid rgba(16, 185, 129, 0.4);
    border-radius: 10px;
    background: rgba(16, 185, 129, 0.12);
    color: #10b981;
    font-size: 13px;
    font-weight: 700;
    cursor: pointer;
    text-align: center;
    touch-action: manipulation;
    -webkit-tap-highlight-color: rgba(16, 185, 129, 0.3);
    min-height: 44px;
    display: flex;
    align-items: center;
    justify-content: center;
}
#drone-select-panel .dsp-follow-btn:active {
    background: rgba(16, 185, 129, 0.3);
}

/* Ring highlight for selected drone */
.drone-select-ring {
    pointer-events: none;
}

@media (max-width: 768px) {
    #drone-select-panel {
        bottom: calc(var(--mobile-bar, 62px) + 8px);
        left: 8px;
        right: 8px;
        width: auto;
        max-height: 50vh;
    }
}
`;

export class DroneSelectPanel {
    constructor() {
        this._selectedIndex = -1;
        this._ring = null;
        this._raycaster = new THREE.Raycaster();
        this._pointer = new THREE.Vector2();
        this._els = {};
        this._updateInterval = null;
        this._injectCSS();
        this._buildDOM();
        this._bindEvents();
    }

    _injectCSS() {
        if (document.getElementById('drone-select-panel-css')) return;
        const s = document.createElement('style');
        s.id = 'drone-select-panel-css';
        s.textContent = CSS;
        document.head.appendChild(s);
    }

    _buildDOM() {
        const panel = document.createElement('div');
        panel.id = 'drone-select-panel';
        panel.innerHTML = `
            <div class="dsp-header">
                <div class="dsp-title">🚁 <span id="dsp-drone-name">—</span></div>
                <button class="dsp-close" id="dsp-close">✕</button>
            </div>
            <div id="dsp-rows"></div>
            <div class="dsp-comm-section" id="dsp-comm-section" style="display:none">
                <div class="dsp-comm-title">📡 Messagerie interne</div>
                <div class="dsp-comm-msg" id="dsp-comm-msg">—</div>
            </div>
            <button class="dsp-follow-btn" id="dsp-follow-btn">🎯 Suivre ce drone</button>
        `;
        document.body.appendChild(panel);
        this._els.panel = panel;
        this._els.name = panel.querySelector('#dsp-drone-name');
        this._els.rows = panel.querySelector('#dsp-rows');
        this._els.commSection = panel.querySelector('#dsp-comm-section');
        this._els.commMsg = panel.querySelector('#dsp-comm-msg');
        this._els.closeBtn = panel.querySelector('#dsp-close');
        this._els.followBtn = panel.querySelector('#dsp-follow-btn');
    }

    _bindEvents() {
        // Canvas tap/click to pick drones
        const addTap = (el, fn) => {
            let handled = 0;
            const guard = () => { const n = Date.now(); if (n - handled < 300) return false; handled = n; return true; };
            el.addEventListener('pointerup', (e) => { if (guard()) fn(e); });
            el.addEventListener('click', (e) => { if (guard()) fn(e); });
        };

        // Close button
        addTap(this._els.closeBtn, () => this.deselect());
        addTap(this._els.followBtn, () => this._followSelected());

        // Tap on canvas — use pointerup on the renderer's domElement
        // We defer binding until first render when canvas exists
        this._canvasBound = false;
        this._tryBindCanvas();
        if (!this._canvasBound) {
            // Retry after 3s
            setTimeout(() => this._tryBindCanvas(), 3000);
        }

        // Touch exclusion -stop panel itself from propagating to 3D controls
        this._els.panel.addEventListener('touchstart', (e) => e.stopPropagation(), { passive: true });
    }

    _tryBindCanvas() {
        const canvas = document.querySelector('#canvas_container canvas') || document.querySelector('canvas');
        if (!canvas) return;
        this._canvasBound = true;

        // We need a short-tap detector that doesn't fire on drag/orbit
        let downPos = null;
        let downTime = 0;
        canvas.addEventListener('pointerdown', (e) => {
            downPos = { x: e.clientX, y: e.clientY };
            downTime = Date.now();
        });
        canvas.addEventListener('pointerup', (e) => {
            if (!downPos) return;
            const dx = e.clientX - downPos.x;
            const dy = e.clientY - downPos.y;
            const dt = Date.now() - downTime;
            downPos = null;
            // Only count as a tap if short duration and minimal movement
            if (dt < 350 && Math.abs(dx) < 15 && Math.abs(dy) < 15) {
                this._onCanvasTap(e);
            }
        });
    }

    _onCanvasTap(e) {
        const sys = window.diamantsSystem;
        if (!sys || !sys.camera || !sys.renderer || !sys.drones?.length) return;

        const rect = sys.renderer.domElement.getBoundingClientRect();
        this._pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        this._pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

        this._raycaster.setFromCamera(this._pointer, sys.camera);

        // Collect all drone meshes
        const meshes = [];
        for (let i = 0; i < sys.drones.length; i++) {
            const d = sys.drones[i];
            if (d.mesh) meshes.push({ mesh: d.mesh, index: i });
        }
        if (meshes.length === 0) return;

        // Raycast with recursive children check (drone meshes are groups)
        const objects = meshes.map(m => m.mesh);
        const hits = this._raycaster.intersectObjects(objects, true);

        if (hits.length > 0) {
            // Find which drone index this hit belongs to
            const hitObj = hits[0].object;
            let foundIdx = -1;
            for (const { mesh, index } of meshes) {
                if (mesh === hitObj || this._isChild(mesh, hitObj)) {
                    foundIdx = index;
                    break;
                }
            }
            if (foundIdx >= 0) {
                this.select(foundIdx);
                return;
            }
        }
        // Tap on empty space — deselect
        // (only if panel is visible, otherwise do nothing)
        if (this._selectedIndex >= 0) {
            this.deselect();
        }
    }

    _isChild(parent, obj) {
        let cur = obj;
        while (cur) {
            if (cur === parent) return true;
            cur = cur.parent;
        }
        return false;
    }

    select(index) {
        const sys = window.diamantsSystem;
        if (!sys || index < 0 || index >= sys.drones.length) return;

        this._selectedIndex = index;
        this._updatePanel();
        this._els.panel.classList.add('visible');
        this._showRing(sys.drones[index]);

        // Auto-update every 500ms while selected
        clearInterval(this._updateInterval);
        this._updateInterval = setInterval(() => {
            if (this._selectedIndex >= 0) this._updatePanel();
        }, 500);
    }

    deselect() {
        this._selectedIndex = -1;
        this._els.panel.classList.remove('visible');
        this._hideRing();
        clearInterval(this._updateInterval);
    }

    _updatePanel() {
        const sys = window.diamantsSystem;
        if (!sys || this._selectedIndex < 0) return;
        const drone = sys.drones[this._selectedIndex];
        if (!drone) return;

        // Name
        this._els.name.textContent = drone.id || `Drone ${this._selectedIndex + 1}`;

        // Status info
        const engine = sys.integratedController?.autonomousFlightEngine;
        const engineState = engine?.drones?.get(drone.id);
        const phase = engineState?.phase || drone.state || 'IDLE';
        const pos = drone.mesh?.position || drone.position;

        const rows = [
            ['Type', drone.profileLabel || drone.droneProfile || drone.type || '—'],
            ['Rôle', drone.agentRole || '—'],
            ['Phase', phase],
            ['Position', pos ? `${pos.x.toFixed(1)}, ${pos.y.toFixed(1)}, ${pos.z.toFixed(1)}` : '—'],
            ['Altitude', pos ? `${pos.y.toFixed(1)} m` : '—'],
        ];

        // Add battery if available
        if (engineState?.battery != null) {
            const b = Math.round(engineState.battery);
            rows.push(['Batterie', `${b}%`]);
        }

        // Add autonomy if available
        const autonomy = engineState?.autonomy ?? drone.autonomy;
        if (autonomy != null) {
            rows.push(['Autonomie', `${Math.round(autonomy)}%`]);
        }

        this._els.rows.innerHTML = rows.map(([label, val]) =>
            `<div class="dsp-row"><span class="dsp-label">${label}</span><span class="dsp-value">${val}</span></div>`
        ).join('');

        // Communication / messaging
        const labelUpdater = drone._labelSprite?.userData?.updateFn || drone.mesh?.userData?.labelUpdate;
        const commDetails = engineState?.commDetails;
        const comm = engineState?.comm || drone.comm;

        if (commDetails) {
            this._els.commSection.style.display = '';
            const lines = [];
            if (commDetails.peers?.length) {
                lines.push(`📡 Pairs: ${commDetails.peers.join(', ')}`);
            }
            if (commDetails.directive) {
                lines.push(`⚡ ${commDetails.directive.from || '?'} → ${commDetails.directive.target || '?'}: ${commDetails.directive.msg || ''}`);
            }
            if (commDetails.lastMsg) {
                lines.push(commDetails.lastMsg);
            }
            this._els.commMsg.textContent = lines.join('\n') || '—';
        } else if (comm) {
            this._els.commSection.style.display = '';
            this._els.commMsg.textContent = comm;
        } else {
            this._els.commSection.style.display = 'none';
        }

        // Update ring position
        this._updateRingPosition(drone);
    }

    _followSelected() {
        const sys = window.diamantsSystem;
        if (!sys || this._selectedIndex < 0) return;
        sys.autoFollow = true;
        sys.followDroneIndex = this._selectedIndex;
    }

    _showRing(drone) {
        this._hideRing();
        const pos = drone.mesh?.position || drone.position;
        if (!pos) return;
        const sys = window.diamantsSystem;
        if (!sys?.scene) return;

        const geo = new THREE.RingGeometry(1.8, 2.2, 32);
        geo.rotateX(-Math.PI / 2);
        const mat = new THREE.MeshBasicMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.6, side: THREE.DoubleSide, depthWrite: false });
        this._ring = new THREE.Mesh(geo, mat);
        this._ring.position.copy(pos);
        this._ring.position.y = pos.y - 0.5;
        this._ring.className = 'drone-select-ring';
        sys.scene.add(this._ring);
    }

    _updateRingPosition(drone) {
        if (!this._ring) return;
        const pos = drone.mesh?.position || drone.position;
        if (pos) {
            this._ring.position.set(pos.x, pos.y - 0.5, pos.z);
        }
    }

    _hideRing() {
        if (this._ring) {
            this._ring.parent?.remove(this._ring);
            this._ring.geometry?.dispose();
            this._ring.material?.dispose();
            this._ring = null;
        }
    }
}
