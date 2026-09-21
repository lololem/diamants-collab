/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Panneau Hypervision Fractale — stack L0→L5, forage vers couche ROS basse.
 */

const STATUS_COLORS = {
    ok: '#22c55e',
    sim: '#38bdf8',
    idle: '#64748b',
    degraded: '#f59e0b',
    offline: '#ef4444',
    ros: '#4ade80',
};

export class FractalHypervisionPanel {
    constructor(hypervision) {
        this.hypervision = hypervision;
        this.visible = false;
        this._root = null;
        this._stackEl = null;
        this._detailEl = null;
        this._onUpdate = () => this.render();
    }

    mount() {
        if (this._root) return;

        const root = document.createElement('div');
        root.id = 'fractal-hypervision-panel';
        root.style.cssText = `
            position: fixed; top: 72px; left: 12px; z-index: 4400;
            width: 280px; max-height: calc(100vh - 100px);
            background: rgba(8, 15, 30, 0.92); border: 1px solid rgba(100,116,139,0.35);
            border-radius: 10px; backdrop-filter: blur(8px);
            font-family: 'Segoe UI', system-ui, sans-serif; color: #e2e8f0;
            display: none; flex-direction: column; overflow: hidden;
            box-shadow: 0 8px 32px rgba(0,0,0,0.45);
        `;

        const header = document.createElement('div');
        header.style.cssText = `
            padding: 10px 12px; border-bottom: 1px solid rgba(255,255,255,0.08);
            display: flex; justify-content: space-between; align-items: center;
        `;
        header.innerHTML = `
            <span style="font-weight:700;font-size:12px;letter-spacing:0.05em;">HYPERVISION FRACTALE</span>
            <span style="font-size:10px;color:#94a3b8;">V · ↑↓ forer</span>
        `;

        const stack = document.createElement('div');
        stack.id = 'fractal-stack';
        stack.style.cssText = 'padding: 8px; display: flex; flex-direction: column; gap: 4px;';

        const detail = document.createElement('div');
        detail.id = 'fractal-detail';
        detail.style.cssText = `
            padding: 10px 12px; border-top: 1px solid rgba(255,255,255,0.08);
            font-size: 10px; line-height: 1.45; color: #94a3b8; min-height: 72px;
        `;

        const legend = document.createElement('div');
        legend.style.cssText = 'padding: 6px 12px 10px; font-size: 9px; color: #64748b;';
        legend.innerHTML = `
            <div style="display:flex;gap:8px;flex-wrap:wrap;">
                <span>● ROS</span><span style="color:#38bdf8">● Sim</span><span style="color:#64748b">● Idle</span>
            </div>
            <div style="margin-top:4px;">L5 = /{cfN}/odom · Gazebo · cmd_vel</div>
        `;

        root.appendChild(header);
        root.appendChild(stack);
        root.appendChild(detail);
        root.appendChild(legend);
        document.body.appendChild(root);

        this._root = root;
        this._stackEl = stack;
        this._detailEl = detail;

        window.addEventListener('diamants:fractal-updated', this._onUpdate);
        this.render();
    }

    toggle() {
        this.visible = !this.visible;
        if (this._root) {
            this._root.style.display = this.visible ? 'flex' : 'none';
        }
        if (this.visible) this.render();
    }

    render() {
        if (!this._stackEl || !this.hypervision?.snapshot) return;
        const snap = this.hypervision.snapshot;
        const focus = this.hypervision.focusLayer;

        this._stackEl.innerHTML = '';
        for (const layer of snap.layers) {
            const isFocus = layer.id === focus;
            const statusColor = STATUS_COLORS[layer.status] || STATUS_COLORS.idle;
            const row = document.createElement('button');
            row.type = 'button';
            row.style.cssText = `
                display: flex; align-items: center; gap: 8px; width: 100%;
                padding: 6px 8px; border: none; border-radius: 6px; cursor: pointer;
                background: ${isFocus ? 'rgba(255,255,255,0.1)' : 'transparent'};
                border-left: 3px solid ${layer.color};
                text-align: left; color: inherit;
            `;
            row.innerHTML = `
                <span style="
                    width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0;
                    background: ${statusColor}; box-shadow: 0 0 6px ${statusColor};
                "></span>
                <span style="flex:1;min-width:0;">
                    <span style="font-weight:700;font-size:11px;">${layer.id}</span>
                    <span style="font-size:10px;color:#cbd5e1;margin-left:4px;">${layer.name}</span>
                    <div style="font-size:9px;color:#64748b;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;">
                        ${layer.summary}
                    </div>
                </span>
                ${layer.ros ? '<span style="font-size:8px;color:#4ade80;font-weight:700;">ROS</span>' : ''}
            `;
            row.addEventListener('click', () => {
                this.hypervision.setFocusLayer(layer.id);
                this.render();
            });
            this._stackEl.appendChild(row);
        }

        const focused = snap.layers.find((l) => l.id === focus);
        if (focused && this._detailEl) {
            const metrics = focused.metrics
                ? Object.entries(focused.metrics).map(([k, v]) => `${k}: ${v}`).join(' · ')
                : '';
            this._detailEl.innerHTML = `
                <div style="color:${focused.color};font-weight:700;margin-bottom:4px;">
                    ${focused.id} — ${focused.name}
                </div>
                <div style="color:#cbd5e1;margin-bottom:4px;">${focused.authority}</div>
                <div>${focused.summary}</div>
                ${metrics ? `<div style="margin-top:6px;font-family:monospace;font-size:9px;">${metrics}</div>` : ''}
                <div style="margin-top:6px;color:#64748b;">
                    Source: ${focused.ros ? 'ROS2/Gazebo (couche basse)' : 'Moteur local CAS-2'}
                </div>
            `;
        }
    }

    destroy() {
        window.removeEventListener('diamants:fractal-updated', this._onUpdate);
        this._root?.remove();
        this._root = null;
    }
}
