/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * view-persistence.js — Persists all UI view states to localStorage.
 *
 * Saves: minimap modes/positions/sizes, LLM chat position/size/visibility,
 *        sidebar open/closed, sidebar section collapse states,
 *        orchestration console collapsed/expanded + height.
 *
 * Usage:
 *   import { ViewPersistence } from './services/view-persistence.js';
 *   const vp = new ViewPersistence();
 *   vp.init();          // restores saved state on load
 *   vp.save();          // manual save (also auto-saves on state changes)
 */

const STORAGE_KEY = 'diamants-view-state';
const SAVE_DEBOUNCE_MS = 400;
const log   = (...a) => console.log('[ViewPersistence]', ...a);
const error = (...a) => console.error('[ViewPersistence]', ...a);

export class ViewPersistence {
    constructor() {
        this._debounceTimer = null;
        this._initialized = false;
    }

    // ═══════════════════════════════════════════════════════════════
    //  PUBLIC API
    // ═══════════════════════════════════════════════════════════════

    /** Restore all saved view states and start watching for changes. */
    init() {
        if (this._initialized) return;
        this._initialized = true;

        const saved = this._load();
        if (saved) {
            this._restoreAll(saved);
        }

        // Auto-save before unload
        window.addEventListener('beforeunload', () => this._saveNow());

        log('💾 ViewPersistence initialisé');
    }

    /** Force-save current state immediately. */
    save() {
        this._saveNow();
    }

    /** Debounced save — call after ANY state change. */
    scheduleSave() {
        clearTimeout(this._debounceTimer);
        this._debounceTimer = setTimeout(() => this._saveNow(), SAVE_DEBOUNCE_MS);
    }

    /** Clear all saved view state. */
    reset() {
        try { localStorage.removeItem(STORAGE_KEY); } catch (_) {}
        log('💾 ViewPersistence: état réinitialisé');
    }

    // ═══════════════════════════════════════════════════════════════
    //  SNAPSHOT — capture current DOM state
    // ═══════════════════════════════════════════════════════════════

    _snapshot() {
        return {
            v: 2,                              // schema version
            ts: Date.now(),
            minimaps: this._snapshotMinimaps(),
            llmChat: this._snapshotLlmChat(),
            sidebar: this._snapshotSidebar(),
            sections: this._snapshotSections(),
            orchConsole: this._snapshotOrchConsole(),
        };
    }

    // ── Minimaps ──────────────────────────────────────────────────

    _snapshotMinimaps() {
        const panels = document.querySelectorAll('.minimap-panel');
        const result = {};
        for (const panel of panels) {
            const id = panel.id;
            if (!id) continue;

            const isDetached = panel.classList.contains('detached');
            const isSnappedLeft = panel.classList.contains('snapped-left');
            const isSnappedRight = panel.classList.contains('snapped-right');

            let mode = 'docked';
            if (isSnappedLeft) mode = 'snapped-left';
            else if (isSnappedRight) mode = 'snapped-right';
            else if (isDetached) mode = 'detached';

            const entry = { mode };

            if (isDetached && !isSnappedLeft && !isSnappedRight) {
                // Save floating position & size
                const r = panel.getBoundingClientRect();
                entry.left = Math.round(r.left);
                entry.top = Math.round(r.top);
                entry.width = Math.round(r.width);
                entry.height = Math.round(r.height);
            }

            // Save pre-snap rect if snapped (for unsnap restore)
            if ((isSnappedLeft || isSnappedRight) && panel._preSnapRect) {
                const ps = panel._preSnapRect;
                entry.preSnap = {
                    left: Math.round(ps.left || ps.x || 0),
                    top: Math.round(ps.top || ps.y || 0),
                    width: Math.round(ps.width),
                    height: Math.round(ps.height),
                };
            }

            // Save last detached rect (survives dock→reload→re-detach)
            if (panel._lastDetachedRect) {
                entry.lastDetached = { ...panel._lastDetachedRect };
            } else if (isDetached && !isSnappedLeft && !isSnappedRight) {
                // Currently floating — treat current rect as lastDetached
                const r = panel.getBoundingClientRect();
                entry.lastDetached = { left: Math.round(r.left), top: Math.round(r.top), width: Math.round(r.width), height: Math.round(r.height) };
            }

            result[id] = entry;
        }
        return result;
    }

    // ── LLM Chat panel ────────────────────────────────────────────

    _snapshotLlmChat() {
        const panel = document.getElementById('llm-chat-panel');
        if (!panel) return null;

        const isVisible = panel.classList.contains('visible');
        const r = panel.getBoundingClientRect();

        return {
            visible: isVisible,
            left: Math.round(r.left),
            top: Math.round(r.top),
            width: Math.round(r.width),
            height: Math.round(r.height),
        };
    }

    // ── Sidebar (left panel) ──────────────────────────────────────

    _snapshotSidebar() {
        const panel = document.getElementById('ros_interface');
        if (!panel) return null;

        const isOpen = panel.style.display !== 'none' && panel.style.display !== '';

        return { open: isOpen };
    }

    // ── Collapsible sidebar sections ──────────────────────────────

    _snapshotSections() {
        const ids = [
            'metrics-content',
            'intelligence-content',
            'llm-decisions-content',
            'camera-content',
            'config-content',
            'benchmark-content',
        ];
        const result = {};
        for (const id of ids) {
            const el = document.getElementById(id);
            if (!el) continue;
            result[id] = el.style.display !== 'none';
        }
        return result;
    }

    // ── Orchestration console ─────────────────────────────────────

    _snapshotOrchConsole() {
        const root = document.getElementById('orch-console');
        if (!root) return null;

        const height = parseInt(root.style.height) || 32;
        const collapsed = height <= 36;
        const logBody = root.querySelector('[style*="overflow"]') || root.querySelector('div:nth-child(3)');

        return {
            collapsed,
            height: collapsed ? (parseInt(root.dataset.savedHeight) || 220) : height,
        };
    }

    // ═══════════════════════════════════════════════════════════════
    //  RESTORE — apply saved state to DOM
    // ═══════════════════════════════════════════════════════════════

    _restoreAll(state) {
        if (!state || state.v !== 2) return;

        // Use requestAnimationFrame to ensure DOM is ready
        requestAnimationFrame(() => {
            try {
                if (state.minimaps) this._restoreMinimaps(state.minimaps);
                if (state.llmChat) this._restoreLlmChat(state.llmChat);
                if (state.sidebar) this._restoreSidebar(state.sidebar);
                if (state.sections) this._restoreSections(state.sections);
                if (state.orchConsole) this._restoreOrchConsole(state.orchConsole);
                log('💾 ViewPersistence: état restauré');
            } catch (e) {
                error('💾 ViewPersistence restore error:', e);
            }
        });
    }

    // ── Minimaps ──────────────────────────────────────────────────

    _restoreMinimaps(saved) {
        // Minimaps are now accessed via picker only — skip restore entirely
        return;
        /* eslint-disable no-unreachable */
        for (const [id, state] of Object.entries(saved)) {
            const panel = document.getElementById(id);
            if (!panel) continue;

            if (state.mode === 'docked') {
                // Restore lastDetachedRect even when docked, so re-detach remembers size
                if (state.lastDetached) panel._lastDetachedRect = { ...state.lastDetached };
                continue;
            }

            // Need to detach first
            const btn = panel.querySelector('.minimap-detach-btn');
            if (btn && !panel.classList.contains('detached')) {
                btn.click(); // triggers detachPanel()
            }

            if (state.mode === 'detached') {
                // Set floating position
                if (state.left != null) panel.style.left = state.left + 'px';
                if (state.top != null) panel.style.top = state.top + 'px';
                if (state.width != null) panel.style.width = state.width + 'px';
                if (state.height != null) panel.style.height = state.height + 'px';
                panel.style.right = 'auto';
                panel.style.bottom = 'auto';

                // Update font scale
                const fs = panel.style.getPropertyValue('--minimap-fs');
                if (state.width) {
                    const BASE_W = 280, BASE_FONT = 12;
                    const scale = Math.min(2.0, Math.max(0.85, state.width / BASE_W));
                    panel.style.setProperty('--minimap-fs', (BASE_FONT * scale).toFixed(1) + 'px');
                }

            } else if (state.mode === 'snapped-left' || state.mode === 'snapped-right') {
                // Restore lastDetachedRect for snapped panels too
                if (state.lastDetached) panel._lastDetachedRect = { ...state.lastDetached };
                // Save pre-snap rect for unsnap
                if (state.preSnap) {
                    panel._preSnapRect = new DOMRect(
                        state.preSnap.left, state.preSnap.top,
                        state.preSnap.width, state.preSnap.height
                    );
                }
                // Clear inline positioning, apply snap class
                panel.style.left = ''; panel.style.top = '';
                panel.style.right = ''; panel.style.bottom = '';
                panel.style.width = ''; panel.style.height = '';
                const side = state.mode === 'snapped-left' ? 'left' : 'right';
                panel.classList.add('snapped-' + side);

                // Update font scale for half-viewport
                const BASE_FONT = 12;
                const scale = Math.min(2.0, Math.max(0.85, (window.innerWidth * 0.5) / 280));
                panel.style.setProperty('--minimap-fs', (BASE_FONT * scale).toFixed(1) + 'px');

                // Sync canvas after layout settles
                this._syncMinimapCanvas(panel, 350);
            }
        }
    }

    _syncMinimapCanvas(panel, delay) {
        const canvas = panel.querySelector('canvas');
        if (!canvas) return;
        const inst = canvas._minimapInstance;
        setTimeout(() => {
            const cRect = canvas.getBoundingClientRect();
            const w = Math.round(cRect.width), h = Math.round(cRect.height);
            if (w < 10 || h < 10) return;
            canvas.width = w; canvas.height = h;
            if (inst) {
                if (inst._offscreen) { inst._offscreen.width = w; inst._offscreen.height = h; }
                if (inst._off) { inst._off.width = w; inst._off.height = h; }
                if (inst._imgData && inst.ctx) { inst._imgData = inst.ctx.createImageData(w, h); }
            }
        }, delay || 100);
    }

    // ── LLM Chat panel ────────────────────────────────────────────

    _restoreLlmChat(state) {
        const panel = document.getElementById('llm-chat-panel');
        if (!panel) return;

        // Restore size and position
        if (state.width) panel.style.width = state.width + 'px';
        if (state.height) panel.style.height = state.height + 'px';
        if (state.left != null) {
            panel.style.left = state.left + 'px';
            panel.style.right = 'auto';
        }
        if (state.top != null) {
            panel.style.top = state.top + 'px';
            panel.style.bottom = 'auto';
        }

        // Restore visibility — find chatPanel instance on window
        if (state.visible) {
            // Defer to let the LLM panel class init first
            setTimeout(() => {
                const cp = window.DIAMANTS?.chatPanel || window._llmChatPanel;
                if (cp && !cp._visible) cp.show();
            }, 500);
        }
    }

    // ── Sidebar ───────────────────────────────────────────────────

    _restoreSidebar(state) {
        const panel = document.getElementById('ros_interface');
        const canvas = document.getElementById('canvas_container');
        const button = document.getElementById('toggle_panel');
        const mobile = window.innerWidth <= 768;
        if (!panel) return;

        if (state.open) {
            panel.style.display = 'block';
            if (canvas) canvas.classList.add('panel-visible');
            if (button) { button.classList.add('panel-visible'); button.innerHTML = mobile ? '✕' : '◄'; }
            // Show backdrop on mobile
            const backdrop = document.getElementById('mobile-sidebar-backdrop');
            if (backdrop && mobile) backdrop.classList.add('active');
        } else {
            panel.style.display = 'none';
            if (canvas) canvas.classList.remove('panel-visible');
            if (button) { button.classList.remove('panel-visible'); button.innerHTML = mobile ? '☰' : '►'; }
        }
    }

    // ── Sections ──────────────────────────────────────────────────

    _restoreSections(sections) {
        for (const [id, isOpen] of Object.entries(sections)) {
            const el = document.getElementById(id);
            if (!el) continue;

            el.style.display = isOpen ? 'block' : 'none';

            // Update header arrow
            const header = el.previousElementSibling;
            if (header && header.tagName === 'H3') {
                for (const node of header.childNodes) {
                    if (node.nodeType === Node.TEXT_NODE) {
                        node.textContent = node.textContent.replace(/[▼▶]/g, isOpen ? '▼' : '▶');
                    }
                }
                const arrowSpan = header.querySelector('[id$="-arrow"]');
                if (arrowSpan) arrowSpan.textContent = isOpen ? '▼' : '▶';
            }
        }
    }

    // ── Orchestration console ─────────────────────────────────────

    _restoreOrchConsole(state) {
        const root = document.getElementById('orch-console');
        if (!root) return;

        // Store saved height for later toggle
        if (state.height) root.dataset.savedHeight = state.height;

        // Find the OrchestrationConsole instance
        const oc = window.DIAMANTS?.system?.orchestrationConsole;

        if (!state.collapsed) {
            // Was expanded — expand it
            if (oc) {
                oc._savedHeight = state.height || 220;
                if (oc._collapsed) oc.toggle();
            } else {
                // Fallback: manipulate DOM directly
                root.style.height = (state.height || 220) + 'px';
                const logBody = root.querySelectorAll('div')[2]; // 3rd div = log body
                if (logBody) logBody.style.display = '';
            }
        }
        // collapsed is default — nothing to do
    }

    // ═══════════════════════════════════════════════════════════════
    //  STORAGE
    // ═══════════════════════════════════════════════════════════════

    _saveNow() {
        try {
            const state = this._snapshot();
            localStorage.setItem(STORAGE_KEY, JSON.stringify(state));
        } catch (e) {
            // localStorage full or unavailable — ignore silently
        }
    }

    _load() {
        try {
            const raw = localStorage.getItem(STORAGE_KEY);
            if (!raw) return null;
            return JSON.parse(raw);
        } catch (_) {
            return null;
        }
    }
}
