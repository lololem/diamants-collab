/**
 * DIAMANTS — Follow Drone FAB
 * =============================
 * Floating action button for drone follow/tracking mode.
 * Placed above the AI Command FAB (bottom-right).
 * Tap to follow → cycles through drones → tap again to exit.
 * Shows current tracked drone ID.
 * Mobile-safe: explicit exit button, no accidental orbit-exit.
 */

const CSS = `
/* ═══ Follow Drone FAB ═══ */
#follow-drone-fab {
    position: fixed;
    bottom: 140px;
    right: 20px;
    width: 52px;
    height: 52px;
    border-radius: 50%;
    background: linear-gradient(135deg, #10b981, #059669);
    border: 2px solid rgba(255,255,255,0.15);
    color: white;
    font-size: 22px;
    cursor: pointer;
    z-index: 7500;
    display: flex;
    align-items: center;
    justify-content: center;
    box-shadow: 0 4px 20px rgba(16, 185, 129, 0.4);
    transition: transform 0.2s, box-shadow 0.2s, background 0.2s;
    touch-action: manipulation;
    -webkit-tap-highlight-color: rgba(16, 185, 129, 0.3);
    user-select: none;
    font-family: 'Inter', -apple-system, sans-serif;
}
#follow-drone-fab:hover {
    transform: scale(1.1);
    box-shadow: 0 6px 30px rgba(16, 185, 129, 0.6);
}
#follow-drone-fab.active {
    background: linear-gradient(135deg, #f59e0b, #d97706);
    box-shadow: 0 4px 20px rgba(245, 158, 11, 0.5);
    animation: follow-pulse 1.5s infinite;
}
@keyframes follow-pulse {
    0%, 100% { box-shadow: 0 4px 20px rgba(245, 158, 11, 0.5); }
    50% { box-shadow: 0 4px 30px rgba(245, 158, 11, 0.8); }
}

/* Follow HUD overlay — shows tracked drone name */
#follow-drone-hud {
    position: fixed;
    bottom: 200px;
    right: 12px;
    background: rgba(10, 14, 23, 0.92);
    border: 1px solid rgba(16, 185, 129, 0.4);
    border-radius: 12px;
    padding: 10px 16px;
    color: #f9fafb;
    font-family: 'Inter', -apple-system, sans-serif;
    font-size: 13px;
    z-index: 7500;
    display: none;
    flex-direction: column;
    gap: 6px;
    min-width: 160px;
    box-shadow: 0 4px 20px rgba(0,0,0,0.5);
    touch-action: manipulation;
}
#follow-drone-hud.visible {
    display: flex;
}
#follow-drone-hud .follow-title {
    font-weight: 700;
    color: #f59e0b;
    font-size: 14px;
    display: flex;
    align-items: center;
    gap: 6px;
}
#follow-drone-hud .follow-drone-id {
    color: #06b6d4;
    font-family: monospace;
    font-size: 14px;
    font-weight: 600;
}
#follow-drone-hud .follow-actions {
    display: flex;
    gap: 6px;
    margin-top: 4px;
}
#follow-drone-hud .follow-btn {
    flex: 1;
    padding: 8px 10px;
    border: 1px solid rgba(255,255,255,0.15);
    border-radius: 8px;
    background: rgba(255,255,255,0.08);
    color: #f9fafb;
    font-size: 12px;
    font-weight: 600;
    cursor: pointer;
    text-align: center;
    touch-action: manipulation;
    -webkit-tap-highlight-color: rgba(6,182,212,0.3);
    min-height: 40px;
    display: flex;
    align-items: center;
    justify-content: center;
}
#follow-drone-hud .follow-btn:active {
    background: rgba(255,255,255,0.2);
    transform: scale(0.95);
}
#follow-drone-hud .follow-btn.exit {
    border-color: rgba(239, 68, 68, 0.4);
    color: #ef4444;
}

@media (max-width: 768px) {
    #follow-drone-fab {
        bottom: calc(var(--mobile-bar, 62px) + 66px);
        right: 14px;
        width: 50px;
        height: 50px;
        font-size: 20px;
        z-index: 10000;
    }
    #follow-drone-hud {
        bottom: calc(var(--mobile-bar, 62px) + 128px);
        right: 4px;
        min-width: 180px;
    }
    #follow-drone-hud .follow-btn {
        min-height: 44px;
        font-size: 13px;
    }
}
@media (max-height: 500px) and (max-width: 960px) {
    #follow-drone-fab {
        bottom: 66px;
        top: auto;
        right: 62px;
        width: 44px;
        height: 44px;
        font-size: 18px;
    }
    #follow-drone-hud {
        bottom: 120px;
        top: auto;
        right: 62px;
        min-width: 160px;
        font-size: 11px;
    }
}
`;

export class FollowDroneFab {
    constructor() {
        this._active = false;
        this._droneIndex = -1;
        this._els = {};
        this._injectCSS();
        this._buildDOM();
        this._bindEvents();
    }

    _injectCSS() {
        if (document.getElementById('follow-drone-fab-css')) return;
        const style = document.createElement('style');
        style.id = 'follow-drone-fab-css';
        style.textContent = CSS;
        document.head.appendChild(style);
    }

    _buildDOM() {
        // FAB button
        const fab = document.createElement('button');
        fab.id = 'follow-drone-fab';
        fab.innerHTML = '🎯';
        fab.title = 'Suivre un drone';
        document.body.appendChild(fab);
        this._els.fab = fab;

        // HUD overlay
        const hud = document.createElement('div');
        hud.id = 'follow-drone-hud';
        hud.innerHTML = `
            <div class="follow-title">🎯 <span>Suivi drone</span></div>
            <div class="follow-drone-id" id="follow-hud-drone-id">—</div>
            <div class="follow-actions">
                <button class="follow-btn" id="follow-btn-next">Suivant ▶</button>
                <button class="follow-btn exit" id="follow-btn-exit">✕ Quitter</button>
            </div>
        `;
        document.body.appendChild(hud);
        this._els.hud = hud;
        this._els.droneId = hud.querySelector('#follow-hud-drone-id');
        this._els.btnNext = hud.querySelector('#follow-btn-next');
        this._els.btnExit = hud.querySelector('#follow-btn-exit');
    }

    _bindEvents() {
        // Robust tap for FAB (mobile + desktop)
        const addTap = (el, fn) => {
            let handled = 0;
            const guard = () => { const n = Date.now(); if (n - handled < 400) return false; handled = n; return true; };
            el.addEventListener('pointerup', (e) => { e.preventDefault(); e.stopPropagation(); if (guard()) fn(); });
            el.addEventListener('click', (e) => { e.preventDefault(); e.stopPropagation(); if (guard()) fn(); });
            el.addEventListener('touchstart', (e) => { e.stopPropagation(); }, { capture: true, passive: false });
            el.addEventListener('touchend', (e) => { e.preventDefault(); e.stopPropagation(); if (guard()) fn(); });
        };

        addTap(this._els.fab, () => this._toggle());
        addTap(this._els.btnNext, () => this._nextDrone());
        addTap(this._els.btnExit, () => this._exitFollow());

        // Listen for external follow-mode changes (e.g. orbit interaction exits follow)
        // Poll every 500ms to sync state
        setInterval(() => this._syncState(), 500);
    }

    _getSystem() {
        return window.diamantsSystem || null;
    }

    _toggle() {
        const sys = this._getSystem();
        if (!sys) return;
        const droneCount = sys.drones?.length || 0;
        if (droneCount === 0) return;

        if (sys.autoFollow) {
            // Already following → next drone or exit
            this._nextDrone();
        } else {
            // Start following first drone
            sys.autoFollow = true;
            sys.followDroneIndex = 0;
            this._active = true;
            this._droneIndex = 0;
            this._updateUI();
        }
    }

    _nextDrone() {
        const sys = this._getSystem();
        if (!sys || !sys.autoFollow) return;
        const droneCount = sys.drones?.length || 0;
        if (droneCount === 0) return;

        const next = (sys.followDroneIndex + 1) % droneCount;
        if (next === 0 && sys.followDroneIndex === droneCount - 1) {
            // Full cycle → back to first drone (don't exit like the old handler)
            sys.followDroneIndex = 0;
        } else {
            sys.followDroneIndex = next;
        }
        this._droneIndex = sys.followDroneIndex;
        this._updateUI();
    }

    _exitFollow() {
        const sys = this._getSystem();
        if (!sys) return;

        sys.autoFollow = false;
        sys.followDroneIndex = -1;
        this._active = false;
        this._droneIndex = -1;

        // Keep current camera position — just stop tracking
        if (sys.controls) sys.controls.update();

        this._updateUI();
    }

    _syncState() {
        const sys = this._getSystem();
        if (!sys) return;

        const wasActive = this._active;
        this._active = !!sys.autoFollow;
        this._droneIndex = sys.followDroneIndex ?? -1;

        if (wasActive !== this._active) {
            this._updateUI();
        }
    }

    _updateUI() {
        const sys = this._getSystem();
        const fab = this._els.fab;
        const hud = this._els.hud;

        if (this._active && this._droneIndex >= 0) {
            fab.classList.add('active');
            fab.innerHTML = '📡';
            hud.classList.add('visible');

            const drone = sys?.drones?.[this._droneIndex];
            const droneId = drone?.id || `Drone ${this._droneIndex + 1}`;
            const total = sys?.drones?.length || 0;
            this._els.droneId.textContent = `${droneId} (${this._droneIndex + 1}/${total})`;
        } else {
            fab.classList.remove('active');
            fab.innerHTML = '🎯';
            hud.classList.remove('visible');
        }
    }
}
