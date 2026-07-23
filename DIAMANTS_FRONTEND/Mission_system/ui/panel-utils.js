/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Shared Panel Utilities
 * ===================================
 * Provides drag, resize, and responsive scaling for floating panels.
 *
 * Usage:
 *   import { makeDraggable, makeScalable, centerInViewport } from './panel-utils.js';
 *
 *   makeDraggable(panelRoot, headerElement);
 *   makeScalable(panelRoot);                   // adds resize:both + transform scale
 *   centerInViewport(panelRoot, 720, 600);     // center right of sidebar
 */

const SIDEBAR_WIDTH = 420;

/* ──────────────────────────────────────────────────────
 *  centerInViewport — position panel in the 3D viewport
 * ────────────────────────────────────────────────────── */
export function centerInViewport(panel, designW = 720, designH = 600) {
    const SW = SIDEBAR_WIDTH;
    const vpW = window.innerWidth - SW;
    const pW = Math.min(designW, vpW - 40);
    const pH = Math.min(designH, window.innerHeight - 60);
    const pL = SW + Math.round((vpW - pW) / 2);
    const pT = Math.round((window.innerHeight - pH) / 2);
    panel.style.left = pL + 'px';
    panel.style.top = pT + 'px';
    panel.style.width = pW + 'px';
    panel.style.height = pH + 'px';
}

/* ──────────────────────────────────────────────────────
 *  makeDraggable — drag panel via a header element
 * ────────────────────────────────────────────────────── */
export function makeDraggable(panel, header) {
    if (!panel || !header) return;

    let dragging = false, sx, sy, ox, oy;

    header.style.cursor = 'grab';

    header.addEventListener('mousedown', (e) => {
        if (e.target.closest('button, input, select, a')) return;
        dragging = true;
        const r = panel.getBoundingClientRect();
        ox = r.left;
        oy = r.top;
        sx = e.clientX;
        sy = e.clientY;
        // Freeze position to px (remove %, calc, transform centering)
        panel.style.left = ox + 'px';
        panel.style.top = oy + 'px';
        panel.style.right = 'auto';
        panel.style.bottom = 'auto';
        panel.style.transform = 'none';
        document.body.style.userSelect = 'none';
        header.style.cursor = 'grabbing';
        e.preventDefault();
    });

    document.addEventListener('mousemove', (e) => {
        if (!dragging) return;
        panel.style.left = Math.max(0, ox + e.clientX - sx) + 'px';
        panel.style.top = Math.max(0, oy + e.clientY - sy) + 'px';
    });

    document.addEventListener('mouseup', () => {
        if (!dragging) return;
        dragging = false;
        document.body.style.userSelect = '';
        header.style.cursor = 'grab';
    });
}

/* ──────────────────────────────────────────────────────
 *  makeScalable — CSS resize + transform:scale wrapper
 *
 *  Wraps all children in a #<id>-scale-wrapper and uses
 *  ResizeObserver + transform:scale() so that ALL content
 *  (including inline px styles) scales proportionally.
 * ────────────────────────────────────────────────────── */
export function makeScalable(panel, { baseW = 720, baseH = 600 } = {}) {
    if (!panel) return;

    // Ensure the panel supports CSS resize
    panel.style.overflow = 'hidden';
    panel.style.resize = 'both';

    // Create scale wrapper
    const wrapper = document.createElement('div');
    wrapper.id = (panel.id || 'panel') + '-scale-wrapper';
    wrapper.style.cssText =
        'display:flex;flex-direction:column;overflow-y:auto;overflow-x:hidden;transform-origin:0 0;';

    // Set initial design size
    wrapper.style.width = baseW + 'px';
    wrapper.style.height = baseH + 'px';

    // Move all children into wrapper
    while (panel.firstChild) wrapper.appendChild(panel.firstChild);
    panel.appendChild(wrapper);

    // Observe and scale
    if (typeof ResizeObserver === 'undefined') return wrapper;

    let rafId = 0;
    const observer = new ResizeObserver((entries) => {
        cancelAnimationFrame(rafId);
        rafId = requestAnimationFrame(() => {
            for (const entry of entries) {
                const w = entry.contentRect.width;
                const h = entry.contentRect.height;
                if (w < 10) continue;
                const sx = w / baseW;
                const sy = h / baseH;
                const s = Math.min(sx, sy);
                wrapper.style.transform = `scale(${s})`;
                wrapper.style.width = (w / s) + 'px';
                wrapper.style.height = (h / s) + 'px';
            }
        });
    });
    observer.observe(panel);

    return wrapper;
}
