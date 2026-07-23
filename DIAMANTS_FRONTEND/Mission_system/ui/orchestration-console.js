/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Orchestration Console
 * =================================
 * Real-time console panel that traces ALL system events:
 *  • Drone positions & state changes
 *  • SLAM map updates
 *  • Mission orchestration
 *  • WebSocket connection status
 *  • Swarm intelligence metrics
 *  • Backend errors
 *
 * Usage:
 *   import { OrchestrationConsole } from './ui/orchestration-console.js';
 *   const console = new OrchestrationConsole();
 */

const MAX_LINES = 500;
const AUTO_SCROLL_THRESHOLD = 40; // px from bottom

export class OrchestrationConsole {
    constructor(options = {}) {
        /** @type {HTMLElement} */
        this.root = null;
        /** @type {HTMLElement} */
        this.logBody = null;
        /** @type {HTMLElement} */
        this.badge = null;
        /** @type {HTMLElement} */
        this.filterBar = null;

        this._lines = [];
        this._lineId = 0;
        this._collapsed = true;
        this._paused = false;
        this._unreadCount = 0;
        this._filter = 'all';         // 'all' | category key
        this._searchQuery = '';

        // Throttle for high-frequency events
        this._lastPositionLog = 0;
        this._positionLogInterval = 2000;  // ms — log positions summary every 2s
        this._lastSlamLog = 0;
        this._slamLogInterval = 3000;

        // Drone state tracking (detect transitions)
        this._droneStates = {};        // {droneId: lastStatus}
        this._dronePositions = {};     // {droneId: {x,y,z}}

        // Drag / resize state
        this._isDocked = true;         // docked = attached to top-center
        this._savedHeight = 220;       // px, persisted across collapse

        // Stats
        this._stats = {
            wsMessages: 0,
            posUpdates: 0,
            slamUpdates: 0,
            missionEvents: 0,
            errors: 0,
        };

        this._categories = {
            system:  { label: '⚙️ System',  color: '#00AAFF' },
            drone:   { label: '🚁 Drones',  color: '#00FFCC' },
            slam:    { label: '🗺️ SLAM',    color: '#FFAA00' },
            mission: { label: '🎯 Mission', color: '#FF55FF' },
            swarm:   { label: '🧠 Swarm',   color: '#AAFFAA' },
            ws:      { label: '🔌 WS',      color: '#8888FF' },
            error:   { label: '❌ Error',   color: '#FF4444' },
        };

        this._buildDOM();
        this._hookEvents();
        this._hookConsole();
        this.log('system', 'Orchestration Console initialised');
    }

    /* ─────────────── DOM ─────────────── */

    _buildDOM() {
        // Root container — docked top-center, draggable via title bar, resizable
        this.root = document.createElement('div');
        this.root.id = 'orch-console';
        Object.assign(this.root.style, {
            position: 'fixed',
            top: '0',
            left: '22.5%',
            width: '55%',
            height: '32px',
            minHeight: '32px',
            maxHeight: '80vh',
            background: 'rgba(0, 8, 16, 0.94)',
            borderBottom: '2px solid #00AAFF',
            fontFamily: "'JetBrains Mono', 'Fira Code', 'Consolas', monospace",
            fontSize: '11.5px',
            color: '#ccddee',
            zIndex: '6000',
            display: 'flex',
            flexDirection: 'column',
            backdropFilter: 'blur(6px)',
            transition: 'none',
            overflow: 'hidden',
            borderRadius: '0 0 8px 8px',
        });

        // ── Resize handle (top edge) ──
        const resizeHandle = document.createElement('div');
        Object.assign(resizeHandle.style, {
            position: 'absolute',
            bottom: '0', left: '0', right: '0',
            height: '5px',
            cursor: 'ns-resize',
            zIndex: '6001',
        });
        this.root.appendChild(resizeHandle);
        this._setupResize(resizeHandle);

        // ── Title bar (draggable to undock/move) ──
        const titleBar = document.createElement('div');
        Object.assign(titleBar.style, {
            display: 'flex',
            alignItems: 'center',
            gap: '8px',
            padding: '4px 12px',
            background: 'rgba(0, 30, 60, 0.7)',
            borderBottom: '1px solid #003355',
            cursor: 'grab',
            userSelect: 'none',
            flexShrink: '0',
        });
        this._setupDrag(titleBar);

        const title = document.createElement('span');
        title.textContent = '📡 NextGEN Orchestration Console';
        title.style.fontWeight = '600';
        title.style.color = '#00CCFF';
        title.style.flex = '1';

        this.badge = document.createElement('span');
        Object.assign(this.badge.style, {
            background: '#0066AA',
            color: 'white',
            borderRadius: '10px',
            padding: '1px 7px',
            fontSize: '10px',
            display: 'none',
        });

        // Stats bar
        this._statsEl = document.createElement('span');
        Object.assign(this._statsEl.style, {
            fontSize: '10px',
            color: '#6699AA',
            marginRight: '8px',
        });

        const pauseBtn = this._makeBtn('⏸', 'Pause/Resume', () => {
            this._paused = !this._paused;
            pauseBtn.textContent = this._paused ? '▶' : '⏸';
            pauseBtn.title = this._paused ? 'Resume' : 'Pause';
            this.log('system', this._paused ? 'Console paused' : 'Console resumed');
        });
        const clearBtn = this._makeBtn('🗑', 'Clear', () => this.clear());
        const collapseBtn = this._makeBtn('▼', 'Toggle', () => this.toggle());

        titleBar.append(title, this._statsEl, this.badge, pauseBtn, clearBtn, collapseBtn);
        titleBar.addEventListener('dblclick', () => this.toggle());

        // ── Filter bar ──
        this.filterBar = document.createElement('div');
        Object.assign(this.filterBar.style, {
            display: 'none',
            alignItems: 'center',
            gap: '4px',
            padding: '3px 12px',
            background: 'rgba(0, 20, 40, 0.5)',
            borderBottom: '1px solid #002244',
            flexShrink: '0',
            flexWrap: 'wrap',
        });

        // "All" filter chip
        const allChip = this._makeFilterChip('all', '🔍 All');
        allChip.classList.add('active');
        this.filterBar.appendChild(allChip);
        for (const [key, cat] of Object.entries(this._categories)) {
            this.filterBar.appendChild(this._makeFilterChip(key, cat.label));
        }

        // Search input
        const searchInput = document.createElement('input');
        searchInput.type = 'text';
        searchInput.placeholder = 'filter…';
        Object.assign(searchInput.style, {
            background: 'rgba(0, 30, 60, 0.6)',
            border: '1px solid #003366',
            borderRadius: '3px',
            color: '#aaccee',
            padding: '2px 6px',
            fontSize: '10px',
            width: '100px',
            marginLeft: 'auto',
            outline: 'none',
        });
        searchInput.addEventListener('input', () => {
            this._searchQuery = searchInput.value.toLowerCase();
            this._applyFilters();
        });
        this.filterBar.appendChild(searchInput);

        // ── Log body ──
        this.logBody = document.createElement('div');
        Object.assign(this.logBody.style, {
            flex: '1',
            overflowY: 'auto',
            padding: '4px 0',
            scrollBehavior: 'smooth',
            display: 'none',
        });
        // Custom scrollbar
        const scrollStyle = document.createElement('style');
        scrollStyle.textContent = `
            #orch-console ::-webkit-scrollbar { width: 6px; }
            #orch-console ::-webkit-scrollbar-track { background: rgba(0,10,20,0.5); }
            #orch-console ::-webkit-scrollbar-thumb { background: #003366; border-radius: 3px; }
            #orch-console ::-webkit-scrollbar-thumb:hover { background: #005599; }
            .orch-filter-chip {
                background: rgba(0, 40, 80, 0.5);
                border: 1px solid #003355;
                border-radius: 12px;
                color: #6699AA;
                padding: 1px 8px;
                font-size: 10px;
                cursor: pointer;
                transition: all 0.15s;
                user-select: none;
            }
            .orch-filter-chip:hover { background: rgba(0, 60, 120, 0.7); color: #88bbdd; }
            .orch-filter-chip.active { background: #005588; border-color: #00AAFF; color: #00EEFF; }
            .orch-line { padding: 1px 12px; border-bottom: 1px solid rgba(0,30,50,0.3); white-space: pre-wrap; word-break: break-all; line-height: 1.5; }
            .orch-line:hover { background: rgba(0, 40, 80, 0.3); }
            .orch-line .ts { color: #445566; margin-right: 6px; }
            .orch-line .cat { font-weight: 600; margin-right: 6px; }
        `;

        this.root.append(scrollStyle, titleBar, this.filterBar, this.logBody);
        document.body.appendChild(this.root);
    }

    _makeBtn(text, title, onClick) {
        const btn = document.createElement('button');
        btn.textContent = text;
        btn.title = title;
        Object.assign(btn.style, {
            background: 'transparent',
            border: '1px solid #334455',
            borderRadius: '3px',
            color: '#88AACC',
            cursor: 'pointer',
            padding: '1px 6px',
            fontSize: '12px',
        });
        btn.addEventListener('click', (e) => { e.stopPropagation(); onClick(); });
        return btn;
    }

    _makeFilterChip(key, label) {
        const chip = document.createElement('span');
        chip.className = 'orch-filter-chip';
        chip.textContent = label;
        chip.dataset.cat = key;
        chip.addEventListener('click', () => {
            this.filterBar.querySelectorAll('.orch-filter-chip').forEach(c => c.classList.remove('active'));
            chip.classList.add('active');
            this._filter = key;
            this._applyFilters();
        });
        return chip;
    }

    /* ─────────────── Logging API ─────────────── */

    /**
     * @param {'system'|'drone'|'slam'|'mission'|'swarm'|'ws'|'error'} category
     * @param {string} message
     */
    log(category, message) {
        if (this._paused) return;
        const cat = this._categories[category] || this._categories.system;
        const now = new Date();
        const ts = `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}:${String(now.getSeconds()).padStart(2,'0')}.${String(now.getMilliseconds()).padStart(3,'0')}`;

        const id = ++this._lineId;
        const lineData = { id, category, message, ts };
        this._lines.push(lineData);

        // Trim old lines
        while (this._lines.length > MAX_LINES) {
            const old = this._lines.shift();
            const el = this.logBody.querySelector(`[data-lid="${old.id}"]`);
            if (el) el.remove();
        }

        // Build DOM element
        const visible = this._matchesFilter(lineData);
        const el = document.createElement('div');
        el.className = 'orch-line';
        el.dataset.lid = id;
        el.dataset.cat = category;
        if (!visible) el.style.display = 'none';
        el.innerHTML = `<span class="ts">${ts}</span><span class="cat" style="color:${cat.color}">[${cat.label}]</span> ${this._escapeHtml(message)}`;

        // Auto-scroll
        const atBottom = this.logBody.scrollHeight - this.logBody.scrollTop - this.logBody.clientHeight < AUTO_SCROLL_THRESHOLD;
        this.logBody.appendChild(el);
        if (atBottom) this.logBody.scrollTop = this.logBody.scrollHeight;

        // Unread badge if collapsed
        if (this._collapsed) {
            this._unreadCount++;
            this.badge.textContent = this._unreadCount;
            this.badge.style.display = '';
        }
    }

    clear() {
        this._lines = [];
        this.logBody.innerHTML = '';
        this._unreadCount = 0;
        this.badge.style.display = 'none';
        this.log('system', 'Console cleared');
    }

    toggle() {
        this._collapsed = !this._collapsed;
        if (this._collapsed) {
            this._savedHeight = parseInt(this.root.style.height) || this._savedHeight;
            this.root.style.height = '32px';
            this.root.style.minHeight = '32px';
            this.logBody.style.display = 'none';
            this.filterBar.style.display = 'none';
        } else {
            this.root.style.height = this._savedHeight + 'px';
            this.root.style.minHeight = '32px';
            this.logBody.style.display = '';
            this.filterBar.style.display = 'flex';
            this._unreadCount = 0;
            this.badge.style.display = 'none';
            this.logBody.scrollTop = this.logBody.scrollHeight;
        }
        window.DIAMANTS?.viewPersistence?.scheduleSave();
    }

    _matchesFilter(lineData) {
        if (this._filter !== 'all' && lineData.category !== this._filter) return false;
        if (this._searchQuery && !lineData.message.toLowerCase().includes(this._searchQuery)) return false;
        return true;
    }

    _applyFilters() {
        for (const el of this.logBody.children) {
            const cat = el.dataset.cat;
            const lid = parseInt(el.dataset.lid);
            const lineData = this._lines.find(l => l.id === lid);
            if (!lineData) { el.style.display = 'none'; continue; }
            el.style.display = this._matchesFilter(lineData) ? '' : 'none';
        }
    }

    _escapeHtml(str) {
        return str.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
    }

    _updateStats() {
        const s = this._stats;
        this._statsEl.textContent = `WS:${s.wsMessages} | Pos:${s.posUpdates} | SLAM:${s.slamUpdates} | Mission:${s.missionEvents} | Err:${s.errors}`;
    }

    /* ─────────────── Drag & Resize ─────────────── */

    _setupDrag(titleBar) {
        let startX, startY, origLeft, origTop;
        let dragging = false;

        const onMouseDown = (e) => {
            if (e.target.closest('button')) return; // don't drag when clicking buttons
            dragging = true;
            startX = e.clientX;
            startY = e.clientY;
            const rect = this.root.getBoundingClientRect();
            origLeft = rect.left;
            origTop = rect.top;
            titleBar.style.cursor = 'grabbing';
            e.preventDefault();
        };

        const onMouseMove = (e) => {
            if (!dragging) return;
            const dx = e.clientX - startX;
            const dy = e.clientY - startY;

            // Once moved >5px, undock from top-center
            if (this._isDocked && (Math.abs(dx) > 5 || Math.abs(dy) > 5)) {
                this._isDocked = false;
                this.root.style.width = Math.min(window.innerWidth, 900) + 'px';
                this.root.style.borderRadius = '0 0 6px 6px';
                this.root.style.boxShadow = '0 0 20px rgba(0,170,255,0.3)';
            }

            this.root.style.left = (origLeft + dx) + 'px';
            this.root.style.top = (origTop + dy) + 'px';
        };

        const onMouseUp = () => {
            if (!dragging) return;
            dragging = false;
            titleBar.style.cursor = 'grab';

            // Snap back to dock if near top edge
            const rect = this.root.getBoundingClientRect();
            if (rect.top <= 10) {
                this._isDocked = true;
                this.root.style.top = '0';
                this.root.style.left = '22.5%';
                this.root.style.width = '55%';
                this.root.style.borderRadius = '0 0 8px 8px';
                this.root.style.boxShadow = '';
            }
        };

        titleBar.addEventListener('mousedown', onMouseDown);
        document.addEventListener('mousemove', onMouseMove);
        document.addEventListener('mouseup', onMouseUp);
    }

    _setupResize(handle) {
        let startY, startH;
        let resizing = false;

        const onMouseDown = (e) => {
            resizing = true;
            startY = e.clientY;
            startH = parseInt(this.root.style.height) || this.root.offsetHeight;
            document.body.style.cursor = 'ns-resize';
            e.preventDefault();
        };

        const onMouseMove = (e) => {
            if (!resizing) return;
            // Dragging down = larger (top-docked), dragging up = smaller
            const dy = e.clientY - startY;
            const newH = Math.max(32, Math.min(window.innerHeight * 0.8, startH + dy));
            this.root.style.height = newH + 'px';
        };

        const onMouseUp = () => {
            if (!resizing) return;
            resizing = false;
            document.body.style.cursor = '';
            this._savedHeight = parseInt(this.root.style.height) || this._savedHeight;
        };

        handle.addEventListener('mousedown', onMouseDown);
        document.addEventListener('mousemove', onMouseMove);
        document.addEventListener('mouseup', onMouseUp);
    }

    /* ─────────────── Event Hooks ─────────────── */

    _hookEvents() {
        // ── Drone positions (throttled summary) ──
        window.addEventListener('diamants:drone-positions', (evt) => {
            this._stats.posUpdates++;
            const now = Date.now();
            const drones = evt.detail;
            if (!drones || typeof drones !== 'object') return;

            // Detect state transitions
            for (const [id, info] of Object.entries(drones)) {
                const pos = info?.position || (info?.x !== undefined ? { x: info.x, y: info.y, z: info.z } : null);
                const status = info?.status || info?.state;
                if (pos) this._dronePositions[id] = pos;

                if (status && this._droneStates[id] !== status) {
                    const prev = this._droneStates[id] || '—';
                    this._droneStates[id] = status;
                    this.log('drone', `${id}: state ${prev} → ${status}`);
                }
            }

            // Throttled position summary
            if (now - this._lastPositionLog > this._positionLogInterval) {
                this._lastPositionLog = now;
                const count = Object.keys(drones).length;
                const sample = Object.entries(drones).slice(0, 2).map(([id, info]) => {
                    const p = info?.position || info;
                    return `${id}(${p.x?.toFixed?.(1) ?? '?'},${p.y?.toFixed?.(1) ?? '?'},${p.z?.toFixed?.(1) ?? '?'})`;
                }).join(' ');
                this.log('drone', `${count} drones positions → ${sample}…`);
                this._updateStats();
            }
        });

        // ── SLAM map updates ──
        window.addEventListener('diamants:slam-map', (evt) => {
            this._stats.slamUpdates++;
            const now = Date.now();
            if (now - this._lastSlamLog > this._slamLogInterval) {
                this._lastSlamLog = now;
                const data = evt.detail;
                const cells = data?.cells?.length || '?';
                const cov = data?.coverage !== undefined ? ` coverage=${Number(data.coverage).toFixed(1)}%` : '';
                this.log('slam', `Map: ${cells} visited cells${cov} (update #${this._stats.slamUpdates})`);
                this._updateStats();
            }
        });

        // ── Mission status ──
        window.addEventListener('diamants:mission-status', (evt) => {
            this._stats.missionEvents++;
            const d = evt.detail || {};
            const phase = d.phase || 'UNKNOWN';
            const flying = d.flyingDrones !== undefined ? ` ${d.flyingDrones}/${d.total || '?'} flying` : '';
            const cov = d.coverage ? ` cov=${d.coverage}%` : '';
            const suc = d.success ? ` success=${d.success}%` : '';
            this.log('mission', `[${phase}]${flying}${cov}${suc}`);
            this._updateStats();
        });

        // ── Swarm intelligence ──
        window.addEventListener('diamants:swarm-update', (evt) => {
            const d = evt.detail || {};
            const formation = d.formation || d.type || '';
            const mode = d.mode ? `[${d.mode}]` : '';
            const auto = d.autonomy !== undefined ? ` auto=${d.autonomy}%` : '';
            const coh = d.coherence ? ` coherence=${d.coherence.toFixed(2)}` : '';
            const emer = d.emergence ? ` emergence=${d.emergence.toFixed(2)}` : '';
            this.log('swarm', `${mode} ${formation}${auto}${coh}${emer}`);
        });

        // ── System status ──
        window.addEventListener('diamants:system-status', (evt) => {
            const d = evt.detail || {};
            this.log('system', `System: ${d.status || JSON.stringify(d)}`);
        });

        // ── WebSocket connection events ──
        window.addEventListener('diamants:ws-connected', () => {
            this._stats.wsMessages++;
            this.log('ws', 'WebSocket connected to DiamantsBridge');
            this._updateStats();
        });
        window.addEventListener('diamants:ws-disconnected', () => {
            this.log('ws', 'WebSocket disconnected — will retry');
        });
        window.addEventListener('diamants:ws-message', (evt) => {
            this._stats.wsMessages++;
        });
        window.addEventListener('diamants:ws-error', (evt) => {
            this._stats.errors++;
            this.log('ws', 'Backend offline — running in CAS 2 (autonomous engine). Start DiamantsBridge on :8765 to enable CAS 1.');
            this._updateStats();
        });

        // ── Drone decision logging (waypoint picks, phase transitions) ──
        window.addEventListener('diamants:drone-decision', (evt) => {
            const d = evt.detail || {};
            const drone = d.droneId || '??';
            const action = d.action || '';
            const phase = d.phase || '';
            const wp = d.waypointsVisited ?? '';
            const autonomy = d.autonomy ?? '';
            const doctrine = d.doctrine ? ` ${d.doctrine}` : '';
            const coa = d.coa ? `/${d.coa}` : '';
            this.log('drone', `${drone} → ${action}  [${phase}]  auto=${autonomy}%${doctrine}${coa}  WP#${wp}`);
        });

        // ── Propeller speeds (very low freq log) ──
        let propLogTime = 0;
        window.addEventListener('diamants:propeller-speeds', () => {
            const now = Date.now();
            if (now - propLogTime > 10000) {
                propLogTime = now;
                this.log('drone', 'Propeller speed data received');
            }
        });

        // Periodic stats refresh
        setInterval(() => this._updateStats(), 5000);
    }

    /**
     * Intercept console.warn and console.error so backend relay logs
     * also appear in the orchestration console.
     */
    _hookConsole() {
        const origWarn = console.warn;
        const origError = console.error;

        console.warn = (...args) => {
            origWarn.apply(console, args);
            const msg = args.map(a => typeof a === 'string' ? a : JSON.stringify(a)).join(' ');
            // Only capture DIAMANTS-relevant warnings (emoji-tagged)
            if (/^[📡🚁🗺️🎯🧠⚡🔌⚙️🔄📊]/.test(msg)) {
                this.log('system', msg);
            }
        };
        console.error = (...args) => {
            origError.apply(console, args);
            this._stats.errors++;
            const msg = args.map(a => {
                if (typeof a === 'string') return a;
                if (a instanceof Error) return `${a.message} (${a.stack?.split('\n')[1]?.trim() || ''})`;
                try { return JSON.stringify(a); } catch { return String(a); }
            }).join(' ');
            this.log('error', msg);
            this._updateStats();
        };
    }
}
