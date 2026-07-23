/**
 * CAS Controller — explicit runtime mode + HUD (FPS, CAS badge)
 *
 * CAS 1: backend positions via WebSocket (ROS2 / Gazebo)
 * CAS 2: AutonomousFlightEngine (standalone demo path)
 * CAS 3: minimal fallback (simple drone.update)
 */

const CAS_LABELS = {
    1: { short: 'CAS-1', title: 'Backend ROS2', color: '#22c55e', hint: 'Positions Gazebo via WebSocket' },
    2: { short: 'CAS-2', title: 'Autonome', color: '#38bdf8', hint: 'Moteur PID local (démo standalone)' },
    3: { short: 'CAS-3', title: 'Minimal', color: '#f97316', hint: 'Fallback sans moteur PID' },
};

export class CasController {
    constructor(system) {
        this.system = system;
        this.level = 2;
        this.wsConnected = false;
        this.backendActive = false;
        this.engineReady = false;
        this._fps = 0;
        this._frameCount = 0;
        this._fpsAccum = 0;
        this._root = null;
        this._badgeEl = null;
        this._fpsEl = null;
        this._hintEl = null;
        this._boundOnWsConnected = () => this._onWsChange(true);
        this._boundOnWsDisconnected = () => this._onWsChange(false);
        this._boundOnWsGaveUp = () => this._onWsGaveUp();
    }

    mount() {
        if (this._root) return;

        const root = document.createElement('div');
        root.id = 'diamants-cas-hud';
        root.style.cssText = `
            position: fixed; top: 12px; right: 12px; z-index: 4500;
            display: flex; flex-direction: column; align-items: flex-end; gap: 6px;
            font-family: 'Segoe UI', system-ui, sans-serif; pointer-events: none;
            user-select: none;
        `;

        const row = document.createElement('div');
        row.style.cssText = 'display: flex; gap: 8px; align-items: center;';

        const badge = document.createElement('div');
        badge.id = 'diamants-cas-badge';
        badge.style.cssText = `
            padding: 4px 10px; border-radius: 6px; font-size: 11px; font-weight: 700;
            letter-spacing: 0.04em; color: #fff; background: rgba(15,23,42,0.85);
            border: 1px solid rgba(255,255,255,0.15); backdrop-filter: blur(6px);
        `;

        const fps = document.createElement('div');
        fps.id = 'diamants-fps-counter';
        fps.style.cssText = `
            padding: 4px 8px; border-radius: 6px; font-size: 11px; font-weight: 600;
            color: #cbd5e1; background: rgba(15,23,42,0.7);
            border: 1px solid rgba(255,255,255,0.08); font-variant-numeric: tabular-nums;
        `;
        fps.textContent = '— FPS';

        row.appendChild(badge);
        row.appendChild(fps);

        const hint = document.createElement('div');
        hint.id = 'diamants-cas-hint';
        hint.style.cssText = `
            max-width: 220px; text-align: right; font-size: 10px; color: #94a3b8;
            line-height: 1.3;
        `;

        root.appendChild(row);
        root.appendChild(hint);
        document.body.appendChild(root);

        this._root = root;
        this._badgeEl = badge;
        this._fpsEl = fps;
        this._hintEl = hint;

        window.addEventListener('diamants:ws-connected', this._boundOnWsConnected);
        window.addEventListener('diamants:ws-disconnected', this._boundOnWsDisconnected);
        window.addEventListener('diamants:ws-gave-up', this._boundOnWsGaveUp);

        this._evaluateLevel();
        this._publish();
        this._renderBadge();
        this._emitCasChanged();
        this._syncSidebarStatus();
    }

    destroy() {
        window.removeEventListener('diamants:ws-connected', this._boundOnWsConnected);
        window.removeEventListener('diamants:ws-disconnected', this._boundOnWsDisconnected);
        window.removeEventListener('diamants:ws-gave-up', this._boundOnWsGaveUp);
        this._root?.remove();
        this._root = null;
    }

    tick(deltaSeconds) {
        this._frameCount += 1;
        this._fpsAccum += deltaSeconds;
        if (this._fpsAccum >= 0.5) {
            this._fps = Math.round(this._frameCount / this._fpsAccum);
            this._frameCount = 0;
            this._fpsAccum = 0;
            if (this._fpsEl) {
                const color = this._fps >= 55 ? '#86efac' : this._fps >= 30 ? '#fde047' : '#fca5a5';
                this._fpsEl.textContent = `${this._fps} FPS`;
                this._fpsEl.style.color = color;
            }
        }

        const prev = this.level;
        this._evaluateLevel();
        if (prev !== this.level) {
            this._publish();
            this._renderBadge();
            this._emitCasChanged();
        }
        this._syncSidebarStatus();
    }

    _onWsChange(connected) {
        this.wsConnected = connected;
        this._evaluateLevel();
        this._publish();
        this._renderBadge();
        this._emitCasChanged();
        this._syncSidebarStatus();
    }

    _onWsGaveUp() {
        this.wsConnected = false;
        if (this._hintEl) {
            this._hintEl.dataset.gaveUp = '1';
            this._hintEl.textContent = 'Bridge :8765 indisponible — démo autonome CAS-2';
        }
    }

    _evaluateLevel() {
        const sys = this.system;
        this.wsConnected = !!(sys?.ros?.connected && sys.ros.ws?.readyState === WebSocket.OPEN);
        this.engineReady = !!sys?.integratedController?.autonomousFlightEngine;

        const drones = sys?.drones || sys?.integratedController?.drones || [];
        const now = Date.now();
        this.backendActive = drones.some((d) => {
            const rd = d?.rosData;
            if (!rd?.position) return false;
            if (rd.source === 'engine') return false;
            return (now - (rd.lastUpdate || 0)) < 5000;
        });

        if (this.wsConnected && this.backendActive) {
            this.level = 1;
        } else if (this.engineReady) {
            this.level = 2;
        } else {
            this.level = 3;
        }
    }

    _publish() {
        window.DIAMANTS = window.DIAMANTS || {};
        window.DIAMANTS.casLevel = this.level;
        window.DIAMANTS.cas = {
            level: this.level,
            wsConnected: this.wsConnected,
            backendActive: this.backendActive,
            engineReady: this.engineReady,
            fps: this._fps,
        };
    }

    _renderBadge() {
        const meta = CAS_LABELS[this.level] || CAS_LABELS[2];
        if (this._badgeEl) {
            this._badgeEl.textContent = meta.short;
            this._badgeEl.title = meta.title;
            this._badgeEl.style.borderColor = meta.color;
            this._badgeEl.style.boxShadow = `0 0 12px ${meta.color}44`;
        }
        if (this._hintEl && !this._hintEl.dataset.gaveUp) {
            this._hintEl.textContent = meta.hint;
        }
    }

    _emitCasChanged() {
        try {
            window.dispatchEvent(new CustomEvent('diamants:cas-changed', {
                detail: { level: this.level, ...(window.DIAMANTS?.cas || {}) },
            }));
        } catch (_) { /* safe */ }
    }

    /** Sync sidebar status panel (#system_status, #mode_badge via event, footer). */
    _syncSidebarStatus() {
        const meta = CAS_LABELS[this.level] || CAS_LABELS[2];
        const ic = this.system?.integratedController;
        const drones = this.system?.drones || ic?.drones || [];
        const total = drones.length;

        const statusEl = document.getElementById('system_status');
        if (statusEl) {
            if (ic?.missionStarted) statusEl.textContent = 'Mission active';
            else if (this.backendActive) statusEl.textContent = 'Backend ROS';
            else if (this.engineReady) statusEl.textContent = 'Autonome';
            else statusEl.textContent = 'Initialisation';
        }

        const footer = document.getElementById('system_status_footer');
        if (footer && total > 0) {
            const flying = drones.filter((d) => d.state && d.state !== 'IDLE' && d.state !== 'LANDED').length;
            footer.textContent = `${flying}/${total} actifs · ${meta.short} · ${this._fps || '—'} FPS`;
        }

        if (!this.wsConnected) {
            const badge = document.getElementById('mode_badge');
            if (badge) {
                badge.textContent = meta.short;
                badge.title = meta.title;
                badge.style.background = meta.color + '22';
                badge.style.borderColor = meta.color;
                badge.style.color = meta.color;
            }
        }
    }
}
