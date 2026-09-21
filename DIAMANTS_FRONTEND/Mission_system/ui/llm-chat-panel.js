/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — LLM Chat Panel
 * ===========================
 * Floating chatbot window for human ↔ LLM mission interaction.
 * Press [C] to toggle. Features a full chat UI with message history,
 * quick-action buttons, mission status, and beacon placement.
 *
 * Architecture:
 *   ┌──────────────────────────────────────────────────┐
 *   │ 🤖 DIAMANTS Command AI         [📍] [—] [✕]    │
 *   ├──────────────────────────────────────────────────┤
 *   │ ┌ System ─────────────────────────────────────┐  │
 *   │ │ Prêt. 11 drones actifs. Mode: Hybride      │  │
 *   │ └────────────────────────────────────────────────┘│
 *   │ ┌ User ───────────────────────────────────────┐  │
 *   │ │ Rechercher zone nord                        │  │
 *   │ └────────────────────────────────────────────────┘│
 *   │ ┌ AI ─────────────────────────────────────────┐  │
 *   │ │ ✅ Mission lancée: Recherche zone nord      │  │
 *   │ │ 📍 Zone: (0, -30) rayon 25m                │  │
 *   │ │ 🎛️ Mode: Hybride                           │  │
 *   │ └────────────────────────────────────────────────┘│
 *   ├──────────────────────────────────────────────────┤
 *   │ [🔍Chercher] [🎯Balise] [🔄Patrouille] [🏠RTL] │
 *   ├──────────────────────────────────────────────────┤
 *   │ [📝 Entrez votre ordre...              ] [➤]    │
 *   └──────────────────────────────────────────────────┘
 *
 * @module llm-chat-panel
 */

import { LLMMissionService, MissionType } from '../services/llm-mission-service.js';
import { BeaconSystem }                    from '../tools/beacon-system.js';

// ─── STYLES ──────────────────────────────────────────────────────────

const CSS = `
/* ═══ LLM Chat Panel ═══ */
#llm-chat-panel {
    position: fixed;
    bottom: 60px;
    right: 20px;
    width: 520px;
    height: 680px;
    max-width: calc(100vw - 24px);
    max-height: calc(100vh - 80px);
    background: rgba(10, 14, 23, 0.97);
    border: 1px solid rgba(6, 182, 212, 0.3);
    border-radius: 12px;
    display: none;
    flex-direction: column;
    z-index: 7500;
    box-shadow: 0 8px 32px rgba(0, 0, 0, 0.6), 0 0 60px rgba(6, 182, 212, 0.1);
    font-family: 'Inter', -apple-system, sans-serif;
    --llm-fs: 13px;
    font-size: var(--llm-fs);
    color: #f9fafb;
    overflow: hidden;
    min-width: 320px;
    min-height: 360px;
}
#llm-chat-panel.visible {
    display: flex;
}

/* Resize handles — all 4 corners + 4 edges */
.llm-resize {
    position: absolute;
    z-index: 10;
}
/* Corners */
.llm-resize.corner {
    width: 14px;
    height: 14px;
}
.llm-resize.corner::after {
    content: '';
    position: absolute;
    width: 8px;
    height: 8px;
    border-color: rgba(6, 182, 212, 0.4);
    border-style: solid;
    border-width: 0;
    opacity: 0;
    transition: opacity 0.15s;
}
.llm-resize.corner:hover::after { opacity: 1; }
.llm-resize.tl { top: 0; left: 0; cursor: nwse-resize; }
.llm-resize.tl::after { top: 2px; left: 2px; border-top-width: 2px; border-left-width: 2px; border-radius: 3px 0 0 0; }
.llm-resize.tr { top: 0; right: 0; cursor: nesw-resize; }
.llm-resize.tr::after { top: 2px; right: 2px; border-top-width: 2px; border-right-width: 2px; border-radius: 0 3px 0 0; }
.llm-resize.bl { bottom: 0; left: 0; cursor: nesw-resize; }
.llm-resize.bl::after { bottom: 2px; left: 2px; border-bottom-width: 2px; border-left-width: 2px; border-radius: 0 0 0 3px; }
.llm-resize.br { bottom: 0; right: 0; cursor: nwse-resize; }
.llm-resize.br::after { bottom: 2px; right: 2px; border-bottom-width: 2px; border-right-width: 2px; border-radius: 0 0 3px 0; }
/* Edges */
.llm-resize.edge-t { top: 0; left: 14px; right: 14px; height: 5px; cursor: ns-resize; }
.llm-resize.edge-b { bottom: 0; left: 14px; right: 14px; height: 5px; cursor: ns-resize; }
.llm-resize.edge-l { left: 0; top: 14px; bottom: 14px; width: 5px; cursor: ew-resize; }
.llm-resize.edge-r { right: 0; top: 14px; bottom: 14px; width: 5px; cursor: ew-resize; }

/* ═══ Responsive: tablets / small screens ═══ */
@media (max-width: 768px), (max-height: 500px) and (max-width: 960px) {
    #llm-chat-panel {
        left: 0 !important;
        right: 0 !important;
        bottom: 0 !important;
        top: auto !important;
        width: 100vw !important;
        height: 80vh !important;
        max-width: 100vw !important;
        max-height: 80vh !important;
        min-width: 0 !important;
        border-radius: 18px 18px 0 0 !important;
        z-index: 9999 !important;
    }
    #llm-chat-panel::before {
        content: '';
        display: block;
        width: 40px; height: 4px;
        background: rgba(255,255,255,0.3);
        border-radius: 2px;
        margin: 8px auto 0;
        pointer-events: none;
    }
    #llm-chat-fab {
        bottom: calc(var(--mobile-bar, 62px) + 8px);
        right: 14px;
        width: 50px;
        height: 50px;
        font-size: 22px;
        z-index: 10000 !important;
    }
    .llm-header {
        padding: 8px 14px !important;
    }
    .llm-header-actions {
        position: relative;
        z-index: 50;
    }
    .llm-header-actions button {
        width: 44px !important;
        height: 44px !important;
        font-size: 22px !important;
        border-radius: 8px !important;
        background: rgba(255,255,255,0.12) !important;
        border: 1px solid rgba(255,255,255,0.2) !important;
        pointer-events: auto !important;
        touch-action: manipulation !important;
        -webkit-tap-highlight-color: rgba(0,180,255,0.3);
        cursor: pointer !important;
    }
    .llm-header-actions button:active {
        background: rgba(255,255,255,0.25) !important;
        transform: scale(0.92);
    }
    .llm-quick-actions {
        padding: 8px 12px;
        gap: 8px;
        flex-wrap: wrap;
    }
    .llm-quick-btn {
        font-size: 12px;
        padding: 8px 14px;
        border-radius: 20px;
        touch-action: manipulation;
        -webkit-tap-highlight-color: rgba(0,180,255,0.2);
    }
    .llm-input-bar {
        padding: 10px 12px calc(env(safe-area-inset-bottom, 0px) + 10px) !important;
    }
    .llm-input {
        font-size: 16px !important;
        padding: 12px 14px !important;
    }
    .llm-send-btn {
        width: 48px !important;
        height: 48px !important;
        font-size: 20px !important;
        touch-action: manipulation !important;
        -webkit-tap-highlight-color: rgba(0,180,255,0.3);
    }
    .llm-resize { display: none !important; }
    .llm-header { cursor: default !important; }
    .llm-msg { font-size: 14px !important; }
}
@media (max-height: 600px) {
    #llm-chat-panel {
        bottom: 8px !important;
        max-height: calc(100vh - 16px);
    }
    .llm-quick-actions { display: none; }
}

/* Header */
.llm-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 14px;
    background: linear-gradient(135deg, rgba(6, 182, 212, 0.15), rgba(59, 130, 246, 0.1));
    border-bottom: 1px solid rgba(6, 182, 212, 0.2);
    cursor: move;
    user-select: none;
    flex-shrink: 0;
}
.llm-header-title {
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 700;
    font-size: calc(var(--llm-fs) * 1.077);
}
.llm-header-title .ai-icon {
    font-size: calc(var(--llm-fs) * 1.385);
    animation: llm-pulse 2s infinite;
}
@keyframes llm-pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.6; }
}
.llm-header-actions {
    display: flex;
    gap: 4px;
}
.llm-header-actions button {
    font: inherit;
    background: none;
    border: 1px solid rgba(255,255,255,0.1);
    color: #9ca3af;
    width: 28px;
    height: 28px;
    border-radius: 6px;
    cursor: pointer;
    font-size: calc(var(--llm-fs) * 1.077);
    display: flex;
    align-items: center;
    justify-content: center;
    transition: all 0.15s;
}
.llm-header-actions button:hover {
    background: rgba(255,255,255,0.1);
    color: #f9fafb;
}
.llm-header-actions button.active {
    background: rgba(6, 182, 212, 0.3);
    color: #06b6d4;
    border-color: rgba(6, 182, 212, 0.5);
}

/* Mission status bar */
.llm-status-bar {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 6px 14px;
    background: rgba(17, 24, 39, 0.8);
    border-bottom: 1px solid rgba(75, 85, 99, 0.3);
    font-size: calc(var(--llm-fs) * 0.846);
    color: #9ca3af;
    font-family: 'JetBrains Mono', monospace;
    flex-shrink: 0;
}
.llm-status-dot {
    width: 7px;
    height: 7px;
    border-radius: 50%;
    background: #6b7280;
    flex-shrink: 0;
}
.llm-status-dot.active { background: #10b981; animation: llm-pulse 1.5s infinite; }
.llm-status-dot.busy   { background: #f59e0b; animation: llm-pulse 0.5s infinite; }

/* Messages area */
.llm-messages {
    flex: 1;
    overflow-y: auto;
    -webkit-overflow-scrolling: touch;
    overscroll-behavior: contain;
    touch-action: pan-y;
    padding: 12px 14px;
    display: flex;
    flex-direction: column;
    gap: 10px;
    min-height: 0;
}
.llm-messages::-webkit-scrollbar { width: 4px; }
.llm-messages::-webkit-scrollbar-thumb { background: rgba(75, 85, 99, 0.5); border-radius: 2px; }

/* Message bubbles */
.llm-msg {
    max-width: 92%;
    padding: 8px 12px;
    border-radius: 10px;
    line-height: 1.5;
    font-size: var(--llm-fs);
    word-wrap: break-word;
    white-space: pre-wrap;
}
.llm-msg.system {
    align-self: center;
    background: rgba(139, 92, 246, 0.15);
    border: 1px solid rgba(139, 92, 246, 0.3);
    color: #c4b5fd;
    font-size: calc(var(--llm-fs) * 0.846);
    text-align: center;
    max-width: 100%;
}
.llm-msg.user {
    align-self: flex-end;
    background: rgba(59, 130, 246, 0.2);
    border: 1px solid rgba(59, 130, 246, 0.3);
    color: #bfdbfe;
}
.llm-msg.assistant {
    align-self: flex-start;
    background: rgba(6, 182, 212, 0.1);
    border: 1px solid rgba(6, 182, 212, 0.2);
    color: #e0f2fe;
}
.llm-msg.assistant .mission-tag {
    display: inline-block;
    font-size: calc(var(--llm-fs) * 0.77);
    font-weight: 700;
    padding: 1px 6px;
    border-radius: 4px;
    background: rgba(16, 185, 129, 0.2);
    color: #10b981;
    margin-top: 4px;
}
.llm-msg.error {
    align-self: center;
    background: rgba(239, 68, 68, 0.15);
    border: 1px solid rgba(239, 68, 68, 0.3);
    color: #fca5a5;
    font-size: calc(var(--llm-fs) * 0.923);
}

/* Clickable beacon locator links */
.llm-beacon-link {
    display: inline-block;
    cursor: pointer;
    padding: 3px 8px;
    margin-top: 4px;
    border-radius: 6px;
    background: rgba(251, 191, 36, 0.15);
    border: 1px solid rgba(251, 191, 36, 0.35);
    color: #fbbf24;
    font-size: calc(var(--llm-fs) * 0.85);
    font-weight: 600;
    transition: background 0.2s, transform 0.15s;
}
.llm-beacon-link:hover {
    background: rgba(251, 191, 36, 0.3);
    transform: scale(1.03);
}
.llm-beacon-link:active {
    transform: scale(0.97);
}

/* Quick actions bar */
.llm-quick-actions {
    display: flex;
    gap: 6px;
    padding: 8px 14px;
    border-top: 1px solid rgba(75, 85, 99, 0.3);
    background: rgba(17, 24, 39, 0.5);
    flex-wrap: wrap;
    flex-shrink: 0;
}
.llm-quick-btn {
    font: inherit;
    padding: 4px 10px;
    border: 1px solid rgba(75, 85, 99, 0.5);
    border-radius: 16px;
    background: rgba(31, 41, 55, 0.8);
    color: #d1d5db;
    font-size: calc(var(--llm-fs) * 0.846);
    cursor: pointer;
    transition: all 0.15s;
    white-space: nowrap;
}
.llm-quick-btn:hover {
    background: rgba(6, 182, 212, 0.15);
    border-color: rgba(6, 182, 212, 0.4);
    color: #06b6d4;
}

/* Input bar */
.llm-input-bar {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 10px 14px;
    border-top: 1px solid rgba(75, 85, 99, 0.3);
    background: rgba(17, 24, 39, 0.8);
    flex-shrink: 0;
    position: relative;
    z-index: 2;
}
.llm-input {
    font: inherit;
    flex: 1;
    background: rgba(31, 41, 55, 0.9);
    border: 1px solid rgba(75, 85, 99, 0.5);
    border-radius: 8px;
    padding: 8px 12px;
    color: #f9fafb;
    font-size: var(--llm-fs);
    outline: none;
    transition: border-color 0.15s;
}
.llm-input:focus {
    border-color: rgba(6, 182, 212, 0.5);
}
.llm-input::placeholder {
    color: #6b7280;
}
.llm-send-btn {
    font: inherit;
    width: 36px;
    height: 36px;
    border: none;
    border-radius: 8px;
    background: linear-gradient(135deg, #06b6d4, #3b82f6);
    color: white;
    font-size: calc(var(--llm-fs) * 1.231);
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    transition: transform 0.15s, box-shadow 0.15s;
    flex-shrink: 0;
}
.llm-send-btn:hover {
    transform: translateY(-1px);
    box-shadow: 0 4px 12px rgba(6, 182, 212, 0.4);
}
.llm-send-btn:disabled {
    opacity: 0.4;
    cursor: not-allowed;
    transform: none;
    box-shadow: none;
}

/* Typing indicator */
.llm-typing {
    display: none;
    align-items: center;
    gap: 4px;
    padding: 4px 8px;
    font-size: calc(var(--llm-fs) * 0.846);
    color: #6b7280;
}
.llm-typing.active { display: flex; }
.llm-typing-dot {
    width: 5px;
    height: 5px;
    border-radius: 50%;
    background: #6b7280;
    animation: llm-typing-bounce 0.8s ease-in-out infinite;
}
.llm-typing-dot:nth-child(2) { animation-delay: 0.1s; }
.llm-typing-dot:nth-child(3) { animation-delay: 0.2s; }
@keyframes llm-typing-bounce {
    0%, 80%, 100% { transform: translateY(0); }
    40% { transform: translateY(-4px); }
}

/* FAB button (always visible) */
#llm-chat-fab {
    position: fixed;
    bottom: 16px;
    right: 20px;
    width: 52px;
    height: 52px;
    border-radius: 50%;
    background: linear-gradient(135deg, #06b6d4, #8b5cf6);
    border: 2px solid rgba(255,255,255,0.15);
    color: white;
    font-size: 24px;
    cursor: pointer;
    z-index: 7500;
    display: flex;
    align-items: center;
    justify-content: center;
    box-shadow: 0 4px 20px rgba(6, 182, 212, 0.4);
    transition: transform 0.2s, box-shadow 0.2s;
}
#llm-chat-fab:hover {
    transform: scale(1.1);
    box-shadow: 0 6px 30px rgba(6, 182, 212, 0.6);
}
#llm-chat-fab.open {
    background: linear-gradient(135deg, #4b5563, #374151);
    box-shadow: 0 2px 10px rgba(0,0,0,0.3);
}

/* Beacon count badge */
.llm-beacon-badge {
    position: absolute;
    top: -4px;
    right: -4px;
    background: #ef4444;
    color: white;
    font-size: 10px;
    font-weight: 700;
    width: 18px;
    height: 18px;
    border-radius: 50%;
    display: none;
    align-items: center;
    justify-content: center;
}
.llm-beacon-badge.show { display: flex; }
`;

// ─── QUICK ACTIONS ───────────────────────────────────────────────────

const QUICK_ACTIONS = [
    { label: '🔍 Chercher zone nord', cmd: 'Rechercher zone nord' },
    { label: '🎯 Trouver balise', cmd: 'Trouver la balise' },
    { label: '🔄 Patrouille', cmd: 'Patrouiller la zone' },
    { label: '📐 Formation', cmd: 'Formation regroupement' },
    { label: '🧭 Explorer', cmd: 'Explorer librement' },
    { label: '🏠 Retour base', cmd: 'Retour base' },
];

// ─── PANEL CLASS ─────────────────────────────────────────────────────

export class LLMChatPanel {
    constructor() {
        /** @type {LLMMissionService} */
        this.missionService = new LLMMissionService();
        /** @type {BeaconSystem|null} */
        this.beaconSystem = null;
        this._visible = false;
        this._processing = false;
        this._els = {};

        this._injectCSS();
        this._buildDOM();
        this._hookKeyboard();
        this._hookEvents();

        // Welcome message — show LLM status after probe completees
        const showWelcome = () => {
            const isLLM = this.missionService._ollamaOnline;
            const model = this.missionService._model;
            if (isLLM) {
                this._addMessage('system',
                    `🧠 NextGEN Command AI — connecté à **${model}** via Ollama.\n` +
                    `Parlez naturellement : demandez une mission, posez une question, donnez un ordre.\n` +
                    `[C] pour ouvrir/fermer • [📍] pour placer une balise dans la scène 3D`
                );
            } else {
                this._addMessage('system',
                    `⚠️ NextGEN Command AI — mode dégradé (pas de LLM).\n` +
                    `Ollama non détecté sur localhost:11434. Commandes regex uniquement.\n` +
                    `[C] pour ouvrir/fermer • [📍] pour placer une balise dans la scène 3D`
                );
            }
        };
        // Delay to let probe finish
        setTimeout(showWelcome, 2000);
    }

    // ════════════════════════════════════════════════════════════════════
    //  INITIALIZATION
    // ════════════════════════════════════════════════════════════════════

    /**
     * Initialize beacon system (called after scene is ready).
     * @param {THREE.Scene} scene
     */
    initBeaconSystem(scene) {
        this.beaconSystem = new BeaconSystem(scene, {
            onFound: (beaconId, droneId) => {
                this.missionService.markBeaconFound(beaconId, droneId);
                this._addBeaconMessage('assistant',
                    `🎯 **Balise trouvée !**\n` +
                    `Balise \`${beaconId}\` localisée par \`${droneId}\` ✅`,
                    beaconId
                );
                this._updateBeaconBadge();

                // Report to swarm comm for beacon field reinforcement (P2P sharing)
                const beacon = this.beaconSystem.beacons.get(beaconId);
                if (beacon) {
                    const engine = window.DIAMANTS?.integratedController?.autonomousFlightEngine;
                    if (engine?.commManager) {
                        engine.commManager.reportBeaconFound(droneId, beacon.position, beaconId);
                    }
                }
            },
        });

        // Listen for beacon placement events
        window.addEventListener('diamants:beacon-placed', (e) => {
            const { id, x, z } = e.detail;
            this.missionService.registerBeacon(id, { x, y: 0.5, z });
            this._addBeaconMessage('system',
                `📍 Balise placée à (${x.toFixed(1)}, ${z.toFixed(1)}) — ID: ${id}`,
                id
            );
            this._updateBeaconBadge();
        });
    }

    // ════════════════════════════════════════════════════════════════════
    //  CSS
    // ════════════════════════════════════════════════════════════════════

    _injectCSS() {
        if (document.getElementById('llm-chat-css')) return;
        const style = document.createElement('style');
        style.id = 'llm-chat-css';
        style.textContent = CSS;
        document.head.appendChild(style);
    }

    // ════════════════════════════════════════════════════════════════════
    //  DOM
    // ════════════════════════════════════════════════════════════════════

    _buildDOM() {
        // ── FAB button ──
        const fab = document.createElement('button');
        fab.id = 'llm-chat-fab';
        fab.innerHTML = '🤖';
        fab.title = 'NextGEN Command AI [C]';
        const badge = document.createElement('span');
        badge.className = 'llm-beacon-badge';
        badge.textContent = '0';
        fab.appendChild(badge);
        fab.addEventListener('click', () => this.toggle());
        document.body.appendChild(fab);
        this._els.fab = fab;
        this._els.badge = badge;

        // ── Panel ──
        const panel = document.createElement('div');
        panel.id = 'llm-chat-panel';

        panel.innerHTML = `
            <div class="llm-header">
                <div class="llm-header-title">
                    <span class="ai-icon">🤖</span>
                    <span>NextGEN Command AI</span>
                </div>
                <div class="llm-header-actions">
                    <button id="llm-btn-beacon" title="Placer une balise [📍]">📍</button>
                    <button id="llm-btn-minimize" title="Minimize">—</button>
                    <button id="llm-btn-close" title="Fermer [C]">✕</button>
                </div>
            </div>
            <div class="llm-status-bar">
                <span class="llm-status-dot" id="llm-status-dot"></span>
                <span id="llm-status-text">En attente…</span>
            </div>
            <div class="llm-messages" id="llm-messages"></div>
            <div class="llm-typing" id="llm-typing">
                <span class="llm-typing-dot"></span>
                <span class="llm-typing-dot"></span>
                <span class="llm-typing-dot"></span>
                <span style="margin-left: 4px;">Analyse en cours…</span>
            </div>
            <div class="llm-quick-actions" id="llm-quick-actions"></div>
            <div class="llm-input-bar">
                <input type="text" class="llm-input" id="llm-input"
                    placeholder="Describe your mission order…"
                    autocompletee="off" />
                <button class="llm-send-btn" id="llm-send-btn" title="Envoyer">➤</button>
            </div>
        `;

        document.body.appendChild(panel);
        this._els.panel = panel;

        // Cache refs
        this._els.messages = panel.querySelector('#llm-messages');
        this._els.input = panel.querySelector('#llm-input');
        this._els.sendBtn = panel.querySelector('#llm-send-btn');
        this._els.typing = panel.querySelector('#llm-typing');
        this._els.quickActions = panel.querySelector('#llm-quick-actions');
        this._els.statusDot = panel.querySelector('#llm-status-dot');
        this._els.statusText = panel.querySelector('#llm-status-text');

        // Header buttons — ultra-robust mobile support via multiple event types
        const btnClose = panel.querySelector('#llm-btn-close');
        const btnMin = panel.querySelector('#llm-btn-minimize');
        const btnBeacon = panel.querySelector('#llm-btn-beacon');

        // Robust tap handler: pointerup + click + touchstart (capture) for maximum reliability
        const addRobustTap = (el, fn) => {
            let handled = 0;
            const guard = () => {
                const now = Date.now();
                if (now - handled < 400) return false; // debounce
                handled = now;
                return true;
            };
            // Primary: pointerup (works on both mouse + touch)
            el.addEventListener('pointerup', (e) => {
                e.preventDefault(); e.stopPropagation();
                if (guard()) fn();
            });
            // Fallback 1: click (for older browsers)
            el.addEventListener('click', (e) => {
                e.preventDefault(); e.stopPropagation();
                if (guard()) fn();
            });
            // Fallback 2: touchstart in capture phase (fires FIRST, before any parent handler)
            el.addEventListener('touchstart', (e) => {
                e.stopPropagation(); // don't let parent touch handlers see this
            }, { capture: true, passive: false });
            // Fallback 3: touchend
            el.addEventListener('touchend', (e) => {
                e.preventDefault(); e.stopPropagation();
                if (guard()) fn();
            });
        };
        addRobustTap(btnClose, () => this.hide());
        addRobustTap(btnMin, () => this.hide());
        addRobustTap(btnBeacon, () => this._toggleBeaconMode());

        // Send — also multi-bind for mobile
        const sendFn = () => this._send();
        addRobustTap(this._els.sendBtn, sendFn);
        this._els.input.addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                this._send();
            }
            // Don't propagate keyboard events to prevent [C] toggle while typing
            e.stopPropagation();
        });

        // Quick actions
        this._buildQuickActions();

        // Draggable
        this._makeDraggable(panel, panel.querySelector('.llm-header'));

        // Resizable — all 4 corners + 4 edges
        const edges = ['tl','tr','bl','br','edge-t','edge-b','edge-l','edge-r'];
        for (const dir of edges) {
            const h = document.createElement('div');
            h.className = `llm-resize ${dir.startsWith('edge') ? dir : 'corner ' + dir}`;
            h.dataset.dir = dir;
            panel.appendChild(h);
        }
        this._initResize(panel);

        // Mobile swipe-down-to-close on header drag handle
        this._initSwipeClose(panel);
    }

    _buildQuickActions() {
        const container = this._els.quickActions;
        for (const action of QUICK_ACTIONS) {
            const btn = document.createElement('button');
            btn.className = 'llm-quick-btn';
            btn.textContent = action.label;
            btn.addEventListener('click', () => {
                this._els.input.value = action.cmd;
                this._send();
            });
            container.appendChild(btn);
        }
    }

    // ════════════════════════════════════════════════════════════════════
    //  SWIPE-DOWN-TO-CLOSE (mobile)
    // ════════════════════════════════════════════════════════════════════

    _initSwipeClose(panel) {
        const header = panel.querySelector('.llm-header');
        if (!header) return;
        let startY = 0;
        let swiping = false;
        header.addEventListener('touchstart', (e) => {
            if (e.target.closest('.llm-header-actions')) return;
            startY = e.touches[0].clientY;
            swiping = true;
        }, { passive: true });
        header.addEventListener('touchmove', (e) => {
            if (!swiping) return;
            const dy = e.touches[0].clientY - startY;
            // Visual feedback: translate panel down as user swipes
            if (dy > 0) {
                panel.style.transform = `translateY(${Math.min(dy, 200)}px)`;
                panel.style.transition = 'none';
            }
        }, { passive: true });
        header.addEventListener('touchend', (e) => {
            if (!swiping) return;
            swiping = false;
            const dy = e.changedTouches[0].clientY - startY;
            panel.style.transition = 'transform 0.25s ease';
            panel.style.transform = '';
            if (dy > 80) {
                this.hide();
            }
        });
    }

    // ════════════════════════════════════════════════════════════════════
    //  DRAG
    // ════════════════════════════════════════════════════════════════════

    _makeDraggable(el, handle) {
        let isDragging = false;
        let startX, startY, origX, origY;

        handle.addEventListener('mousedown', (e) => {
            if (e.target.closest('button')) return;
            isDragging = true;
            startX = e.clientX;
            startY = e.clientY;
            const rect = el.getBoundingClientRect();
            origX = rect.left;
            origY = rect.top;
            e.preventDefault();
        });

        document.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            const dx = e.clientX - startX;
            const dy = e.clientY - startY;
            el.style.left = (origX + dx) + 'px';
            el.style.top = (origY + dy) + 'px';
            el.style.right = 'auto';
            el.style.bottom = 'auto';
        });

        document.addEventListener('mouseup', () => {
            if (isDragging) window.DIAMANTS?.viewPersistence?.scheduleSave();
            isDragging = false;
        });
    }

    // ════════════════════════════════════════════════════════════════════
    //  RESIZE
    // ════════════════════════════════════════════════════════════════════

    _initResize(el) {
        const MIN_W = 320, MIN_H = 360;
        let active = null;          // current direction string
        let sx, sy, sr;             // start mouse x/y and start rect

        el.addEventListener('mousedown', (e) => {
            const handle = e.target.closest('.llm-resize');
            if (!handle) return;
            active = handle.dataset.dir;
            sx = e.clientX;
            sy = e.clientY;
            sr = el.getBoundingClientRect();
            e.preventDefault();
            e.stopPropagation();
        });

        document.addEventListener('mousemove', (e) => {
            if (!active) return;
            const dx = e.clientX - sx;
            const dy = e.clientY - sy;
            const maxW = window.innerWidth - 24;
            const maxH = window.innerHeight - 16;
            let l = sr.left, t = sr.top, w = sr.width, h = sr.height;

            // Horizontal component
            if (active.includes('l') || active === 'edge-l') {
                w = Math.min(maxW, Math.max(MIN_W, sr.width - dx));
                l = sr.right - w;
            } else if (active.includes('r') || active === 'edge-r') {
                w = Math.min(maxW, Math.max(MIN_W, sr.width + dx));
            }
            // Vertical component
            if (active.includes('t') || active === 'edge-t') {
                h = Math.min(maxH, Math.max(MIN_H, sr.height - dy));
                t = sr.bottom - h;
            } else if (active.includes('b') || active === 'edge-b') {
                h = Math.min(maxH, Math.max(MIN_H, sr.height + dy));
            }

            el.style.left   = l + 'px';
            el.style.top    = t + 'px';
            el.style.width  = w + 'px';
            el.style.height = h + 'px';
            el.style.right  = 'auto';
            el.style.bottom = 'auto';

            // Scale fonts proportionally
            this._updateFontScale(el, w);
        });

        document.addEventListener('mouseup', () => {
            if (active) window.DIAMANTS?.viewPersistence?.scheduleSave();
            active = null;
        });
    }

    // ════════════════════════════════════════════════════════════════════
    //  FONT SCALE — text grows / shrinks with panel size
    // ════════════════════════════════════════════════════════════════════

    _updateFontScale(el, w) {
        const BASE_W = 520;          // design width
        const BASE_FONT = 13;       // base font-size in px
        const MIN_SCALE = 0.85;
        const MAX_SCALE = 1.8;
        const width = w || el.offsetWidth || BASE_W;
        const scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, width / BASE_W));
        el.style.setProperty('--llm-fs', (BASE_FONT * scale).toFixed(1) + 'px');
    }

    // ════════════════════════════════════════════════════════════════════
    //  MESSAGES
    // ════════════════════════════════════════════════════════════════════

    _addMessage(role, text, mission = null) {
        const container = this._els.messages;
        const div = document.createElement('div');
        div.className = `llm-msg ${role}`;

        // Simple markdown-like formatting
        let html = text
            .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
            .replace(/`(.*?)`/g, '<code style="background:rgba(255,255,255,0.1);padding:1px 4px;border-radius:3px;font-size:calc(var(--llm-fs)*0.923);">$1</code>')
            .replace(/\n/g, '<br>');

        if (mission) {
            html += `<br><span class="mission-tag">📋 ${mission.type} — ${mission.id}</span>`;
        }

        div.innerHTML = html;
        container.appendChild(div);

        // Auto-scroll
        requestAnimationFrame(() => {
            container.scrollTop = container.scrollHeight;
        });
    }

    /**
     * Add a message with a clickable "locate beacon" link.
     */
    _addBeaconMessage(role, text, beaconId) {
        const container = this._els.messages;
        const div = document.createElement('div');
        div.className = `llm-msg ${role}`;

        let html = text
            .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
            .replace(/`(.*?)`/g, '<code style="background:rgba(255,255,255,0.1);padding:1px 4px;border-radius:3px;font-size:calc(var(--llm-fs)*0.923);">$1</code>')
            .replace(/\n/g, '<br>');

        div.innerHTML = html;

        // Add clickable locate link
        const link = document.createElement('span');
        link.className = 'llm-beacon-link';
        link.textContent = '📍 Locate in the scene';
        link.addEventListener('click', () => this._focusOnBeacon(beaconId));
        div.appendChild(document.createElement('br'));
        div.appendChild(link);

        container.appendChild(div);
        requestAnimationFrame(() => {
            container.scrollTop = container.scrollHeight;
        });
    }

    /**
     * Smoothly fly the camera to a beacon's position in the 3D scene.
     */
    _focusOnBeacon(beaconId) {
        const system = window.diamantsSystem;
        if (!system?.camera || !system?.controls) {
            console.warn('[LLM-Chat] Camera not available');
            return;
        }

        // Look up beacon position from mission service registry
        const beacon = this.missionService?.beacons?.get(beaconId);
        if (!beacon?.position) {
            console.warn(`[LLM-Chat] Beacon ${beaconId} position unknown`);
            return;
        }

        const { x, y, z } = beacon.position;

        // Exit follow mode if active
        if (system.autoFollow && system.panelController) {
            system.panelController._exitFollowMode(system);
        }

        // Smooth camera fly-to using animation
        const cam = system.camera;
        const ctrl = system.controls;
        const targetPos = { x, y: (y || 0.5) + 12, z: z + 15 };
        const targetLook = { x, y: y || 0.5, z };

        const startPos = { x: cam.position.x, y: cam.position.y, z: cam.position.z };
        const startLook = { x: ctrl.target.x, y: ctrl.target.y, z: ctrl.target.z };
        const duration = 800;
        const startTime = performance.now();

        const animate = (now) => {
            const t = Math.min((now - startTime) / duration, 1);
            // Ease-out cubic
            const e = 1 - Math.pow(1 - t, 3);

            cam.position.set(
                startPos.x + (targetPos.x - startPos.x) * e,
                startPos.y + (targetPos.y - startPos.y) * e,
                startPos.z + (targetPos.z - startPos.z) * e
            );
            ctrl.target.set(
                startLook.x + (targetLook.x - startLook.x) * e,
                startLook.y + (targetLook.y - startLook.y) * e,
                startLook.z + (targetLook.z - startLook.z) * e
            );
            ctrl.update();

            if (t < 1) requestAnimationFrame(animate);
        };
        requestAnimationFrame(animate);

        console.log(`[LLM-Chat] 📍 Flying camera to beacon ${beaconId} at (${x.toFixed(1)}, ${(y||0.5).toFixed(1)}, ${z.toFixed(1)})`);
    }

    // ════════════════════════════════════════════════════════════════════
    //  SEND / PROCESS
    // ════════════════════════════════════════════════════════════════════

    async _send() {
        const text = this._els.input.value.trim();
        if (!text || this._processing) return;

        this._els.input.value = '';
        this._addMessage('user', text);

        this._processing = true;
        this._els.sendBtn.disabled = true;
        this._els.typing.classList.add('active');

        const isLLM = this.missionService._ollamaOnline;
        this._setStatus('busy', isLLM ? `🧠 ${this.missionService._model} réfléchit…` : 'Traitement…');

        try {
            const result = await this.missionService.processMessage(text);
            this._addMessage('assistant', result.text, result.mission);

            if (result.mission) {
                this._setStatus('active', `Mission ${result.mission.type} — ${result.mission.autonomy}`);

                // Log to orchestration console
                window.DIAMANTS?.system?.orchestrationConsole?.log?.(
                    'mission',
                    `LLM Mission: ${result.mission.type} [${result.mission.id}]`
                );
            } else {
                this._setStatus('idle', 'En attente…');
            }
        } catch (err) {
            this._addMessage('error', `❌ Erreur: ${err.message}`);
            this._setStatus('idle', 'Erreur — réessayez');
        } finally {
            this._processing = false;
            this._els.sendBtn.disabled = false;
            this._els.typing.classList.remove('active');
        }
    }

    // ════════════════════════════════════════════════════════════════════
    //  STATUS
    // ════════════════════════════════════════════════════════════════════

    _setStatus(state, text) {
        this._els.statusDot.className = `llm-status-dot ${state === 'busy' ? 'busy' : state === 'active' ? 'active' : ''}`;
        this._els.statusText.textContent = text;
    }

    // ════════════════════════════════════════════════════════════════════
    //  BEACON MODE
    // ════════════════════════════════════════════════════════════════════

    _toggleBeaconMode() {
        if (!this.beaconSystem) {
            this._addMessage('error', '⚠️ Système de balises non initialisé (scène 3D requise)');
            return;
        }

        const btn = this._els.panel.querySelector('#llm-btn-beacon');
        if (this.beaconSystem.placementMode) {
            this.beaconSystem.exitPlacementMode();
            btn.classList.remove('active');
            this._addMessage('system', '📍 Mode placement balise désactivé');
        } else {
            this.beaconSystem.enterPlacementMode();
            btn.classList.add('active');
            this._addMessage('system', '📍 Mode placement balise activé — cliquez dans la scène 3D pour placer une balise');
            // Fermer le panneau pour permettre l'interaction avec la scène 3D
            this.hide();
        }
    }

    _updateBeaconBadge() {
        if (!this.beaconSystem) return;
        const count = this.beaconSystem.beacons.size;
        this._els.badge.textContent = count;
        this._els.badge.classList.toggle('show', count > 0);
    }

    // ════════════════════════════════════════════════════════════════════
    //  KEYBOARD & EVENTS
    // ════════════════════════════════════════════════════════════════════

    _hookKeyboard() {
        document.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA' || e.target.tagName === 'SELECT') return;
            if (e.code === 'KeyC' && !e.ctrlKey && !e.altKey && !e.metaKey) {
                e.preventDefault();
                this.toggle();
            }
        });
    }

    _hookEvents() {
        // Listen for autonomy changes to update status
        window.addEventListener('diamants:autonomy-change', (e) => {
            if (this._visible) {
                this._addMessage('system',
                    `🎛️ Mode autonomie changé → ${e.detail.mode} (${e.detail.level}%)`
                );
            }
        });

        // Listen for beacon mode exit (e.g. via Escape)
        window.addEventListener('diamants:beacon-mode', (e) => {
            const btn = this._els.panel.querySelector('#llm-btn-beacon');
            if (!e.detail.active) {
                btn.classList.remove('active');
            }
        });

        // Listen for mission events from LLMMissionService
        window.addEventListener('diamants:llm-mission', (e) => {
            const { event, data } = e.detail || {};
            switch (event) {
                case 'auto-chain':
                    this._addMessage('system',
                        `🚀 Lancement automatique` +
                        (data.takeoff ? ` + décollage` : '') +
                        ` avant mission **${data.action}**…`
                    );
                    break;
                case 'search-started':
                    if (data.type === 'SEARCH_BEACON') {
                        this._addMessage('system',
                            `🔍 **Recherche balise lancée**\n` +
                            `${data.drones} drones en exploration auto-organisée\n` +
                            `📋 Doctrine: **${data.doctrine}** • COA: **${data.coa}** • Mode: **${data.mode}**\n` +
                            `🎯 ${data.beacons} balise(s) à trouver — les drones ne connaissent pas leur position`
                        );
                    } else if (data.type === 'SEARCH_ZONE') {
                        this._addMessage('system',
                            `🔍 **Fouille de zone lancée**\n` +
                            `${data.drones} drones déployés en spirale autour de (${data.zone.cx}, ${data.zone.cz}) r=${data.zone.radius}m`
                        );
                    }
                    break;
                case 'mission-completee':
                    this._addMessage('system',
                        `✅ **Mission terminée** — toutes les balises ont été trouvées !`
                    );
                    break;
                case 'beacons-placed': {
                    const ct = data.count || 0;
                    this._addMessage('system',
                        `📍 **${ct} balise(s) placée(s)** ${data.random ? 'aléatoirement' : ''} dans la scène`
                    );
                    break;
                }
                case 'config-changed': {
                    const labels = { doctrine: '🎯 Doctrine', coa: '🧭 COA', autonomy: '⚡ Autonomie' };
                    const label = labels[data.key] || data.key;
                    const val = data.mode ? `${data.value}% (${data.mode})` : data.value;
                    this._addMessage('system', `${label} → **${val}**`);
                    break;
                }
                case 'beacons-cleared': {
                    this._addMessage('system', `🗑️ **Toutes les balises supprimées**`);
                    break;
                }
            }
            this._updateStatusBar();
        });

        // Periodically update status bar (faster when mission active)
        setInterval(() => this._updateStatusBar(), 2000);
    }

    _updateStatusBar() {
        if (!this._visible || this._processing) return;

        const D = window.DIAMANTS;
        const coord = D?.controller?.multiAgentCoordinator;
        const isLLM = this.missionService._ollamaOnline;

        const llmTag = isLLM ? `🧠 ${this.missionService._model}` : '⚠️ Regex local';

        if (!coord) {
            this._setStatus('idle', `${llmTag} — Coordinateur non disponible`);
            return;
        }

        const droneCount = coord.agents?.size || 0;
        const mode = window.DIAMANTS_AUTONOMY_MODE || 'HYBRIDE';
        const mission = this.missionService.activeMission;
        const beaconCount = this.beaconSystem?.beacons?.size || 0;

        let statusParts = [llmTag, `${droneCount} drones`, `Mode: ${mode}`];
        if (beaconCount > 0) {
            const found = [...(this.beaconSystem?.beacons?.values() || [])].filter(b => b.found).length;
            statusParts.push(`Balises: ${found}/${beaconCount}`);
        }
        if (mission) {
            statusParts.push(`Mission: ${mission.type}`);
            this._setStatus('active', statusParts.join(' • '));
        } else {
            this._setStatus('idle', statusParts.join(' • '));
        }
    }

    // ════════════════════════════════════════════════════════════════════
    //  SHOW / HIDE / TOGGLE
    // ════════════════════════════════════════════════════════════════════

    show() {
        this._visible = true;
        this._els.panel.classList.add('visible');
        this._els.fab.classList.add('open');
        this._els.fab.innerHTML = '✕';
        this._els.fab.appendChild(this._els.badge);
        this._updateStatusBar();
        // Apply correct font scale for current size
        requestAnimationFrame(() => {
            this._updateFontScale(this._els.panel);
            this._els.input.focus();
        });
        window.DIAMANTS?.viewPersistence?.scheduleSave();
    }

    hide() {
        this._visible = false;
        this._els.panel.classList.remove('visible');
        this._els.fab.classList.remove('open');
        this._els.fab.innerHTML = '🤖';
        this._els.fab.appendChild(this._els.badge);

        // Exit beacon mode if active
        if (this.beaconSystem?.placementMode) {
            this.beaconSystem.exitPlacementMode();
        }
        window.DIAMANTS?.viewPersistence?.scheduleSave();
    }

    toggle() {
        if (this._visible) this.hide();
        else this.show();
    }

    // ════════════════════════════════════════════════════════════════════
    //  PUBLIC API
    // ════════════════════════════════════════════════════════════════════

    /**
     * Called each frame to check beacon proximity.
     * @param {Map|Array} drones
     */
    tick(drones) {
        if (this.beaconSystem) {
            this.beaconSystem.checkProximity(drones);
        }
    }
}
