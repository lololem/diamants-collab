/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — LLM Mission Service
 * ================================
 * Parses natural language intentions into structured mission orders,
 * dispatches them to agents via the coordinator, and adapts behavior
 * based on the current autonomy mode (centralized ↔ distributed).
 *
 * Mission JSON Protocol:
 *   {
 *     id:        string,       // unique mission identifier
 *     type:      'SEARCH_ZONE' | 'SEARCH_BEACON' | 'PATROL' | 'FORMATION' | 'RTL',
 *     priority:  number,       // 0-10
 *     zone:      { cx, cz, radius } | null,
 *     target:    { x, y, z }   | null,   // for beacon search
 *     agents:    string[]      | 'all',   // targeted drone IDs
 *     params:    Object,                  // mission-specific parameters
 *     autonomy:  'centralized' | 'guided' | 'hybrid' | 'semi-auto' | 'distributed',
 *     timestamp: number
 *   }
 *
 * @module llm-mission-service
 */

import { RaftCommand } from '../agent/raft-consensus.js';

// ─── MISSION TYPES ───────────────────────────────────────────────────

export const MissionType = Object.freeze({
    SEARCH_ZONE:   'SEARCH_ZONE',
    SEARCH_BEACON: 'SEARCH_BEACON',
    PATROL:        'PATROL',
    FORMATION:     'FORMATION',
    RTL:           'RTL',             // Return To Launch
    EXPLORE:       'EXPLORE',
    SURVEILLANCE:  'SURVEILLANCE',
});

// ─── INTENT PARSER (NLU) ────────────────────────────────────────────

/**
 * Parse a natural language string into a structured mission intent.
 * Uses pattern matching — can be replaced by a real LLM backend.
 */
function parseIntent(text) {
    const lower = text.toLowerCase().trim();

    // ─── Search zone: "chercher zone x=10 z=20 r=15" / "recherche zone 10,20"
    let m;
    m = lower.match(/(?:cherch|recherch|scan|explor|surveil)\w*\s+(?:la\s+)?zone\s+(?:x\s*[=:]\s*)?([-\d.]+)[,\s]+(?:z\s*[=:]\s*)?([-\d.]+)(?:[,\s]+(?:r(?:ayon)?\s*[=:]\s*)?([-\d.]+))?/);
    if (m) {
        return {
            type: MissionType.SEARCH_ZONE,
            zone: { cx: parseFloat(m[1]), cz: parseFloat(m[2]), radius: parseFloat(m[3]) || 20 },
            confidence: 0.95,
        };
    }

    // ─── Zone shorthand: "zone nord" / "zone sud-est"
    const ZONE_PRESETS = {
        'nord':       { cx:  0, cz: -30, radius: 25 },
        'sud':        { cx:  0, cz:  30, radius: 25 },
        'est':        { cx:  30, cz:  0, radius: 25 },
        'ouest':      { cx: -30, cz:  0, radius: 25 },
        'nord-est':   { cx:  25, cz: -25, radius: 20 },
        'nord-ouest': { cx: -25, cz: -25, radius: 20 },
        'sud-est':    { cx:  25, cz:  25, radius: 20 },
        'sud-ouest':  { cx: -25, cz:  25, radius: 20 },
        'centre':     { cx:  0, cz:   0, radius: 15 },
        'north':      { cx:  0, cz: -30, radius: 25 },
        'south':      { cx:  0, cz:  30, radius: 25 },
        'east':       { cx:  30, cz:  0, radius: 25 },
        'west':       { cx: -30, cz:  0, radius: 25 },
    };
    for (const [key, zone] of Object.entries(ZONE_PRESETS)) {
        if (lower.includes(key) && /(?:cherch|recherch|scan|explor|zone|surveil|patrol)/i.test(lower)) {
            return { type: MissionType.SEARCH_ZONE, zone, confidence: 0.85 };
        }
    }

    // ─── Search beacon: "trouver la balise" / "find the beacon"
    if (/(?:trouv|cherch|localise?|find|beacon|balise|target|cible)/i.test(lower)) {
        return { type: MissionType.SEARCH_BEACON, confidence: 0.9 };
    }

    // ─── Patrol: "patrouiller" / "patrol the area"
    if (/(?:patrouill|patrol|ronde|surveill)/i.test(lower)) {
        return { type: MissionType.PATROL, confidence: 0.85 };
    }

    // ─── Formation: "formation" / "regrouper"
    if (/(?:formation|regroup|rassembl|convergez)/i.test(lower)) {
        return { type: MissionType.FORMATION, confidence: 0.85 };
    }

    // ─── Return to launch: "retour base" / "RTL"
    if (/(?:retour|rtl|rentrer|base|atterri|land)/i.test(lower)) {
        return { type: MissionType.RTL, confidence: 0.9 };
    }

    // ─── Free exploration: "explorer" / "explore"
    if (/(?:explor|découvr|discover|explore)/i.test(lower)) {
        return { type: MissionType.EXPLORE, confidence: 0.8 };
    }

    // ─── Fallback: low-confidence exploration
    return { type: MissionType.EXPLORE, confidence: 0.3 };
}

// ─── AUTONOMY MAPPING ────────────────────────────────────────────────

function getAutonomyMode() {
    const level = window.DIAMANTS_AUTONOMY_LEVEL ?? 50;
    if (level <= 10) return 'centralized';
    if (level <= 30) return 'guided';
    if (level <= 60) return 'hybrid';
    if (level <= 85) return 'semi-auto';
    return 'distributed';
}

// ─── SERVICE ─────────────────────────────────────────────────────────

let _missionCounter = 0;

export class LLMMissionService {
    constructor() {
        /** @type {Object[]} conversation history */
        this.history = [];
        /** @type {Object|null} current active mission */
        this.activeMission = null;
        /** @type {string|null} Ollama base URL */
        this._ollamaUrl = 'http://localhost:11434';
        /** @type {string} model to use */
        this._model = 'mistral:7b-instruct';
        /** @type {boolean} whether Ollama is reachable */
        this._ollamaOnline = false;
        /** @type {Map<string, Object>} beacon registry */
        this.beacons = new Map();
        /** @type {Function[]} listeners for mission events */
        this._listeners = [];
        /** @type {Function|null} streaming callback (token) => void */
        this.onStream = null;

        // Auto-probe Ollama on construction
        this._probeOllama();
    }

    // ── Event API ──

    onMissionEvent(fn) {
        this._listeners.push(fn);
        return () => { this._listeners = this._listeners.filter(f => f !== fn); };
    }

    _emit(event, data) {
        for (const fn of this._listeners) {
            try { fn(event, data); } catch (_) {}
        }
        window.dispatchEvent(new CustomEvent('diamants:llm-mission', { detail: { event, data } }));
    }

    // ── LLM Backend ──

    /**
     * Probe Ollama availability.
     */
    async _probeOllama() {
        try {
            const r = await fetch(`${this._ollamaUrl}/api/tags`, { signal: AbortSignal.timeout(3000) });
            if (r.ok) {
                const data = await r.json();
                const names = data.models?.map(m => m.name) || [];
                this._ollamaOnline = true;
                // Pick best available model
                const preferred = ['mistral:7b-instruct', 'mistral-nemo', 'mistral-nemo:latest', 'mistral:latest'];
                for (const p of preferred) {
                    if (names.some(n => n === p || n.startsWith(p.split(':')[0]))) {
                        this._model = names.find(n => n === p || n.startsWith(p.split(':')[0]));
                        break;
                    }
                }
                console.log(`[LLM] ✅ Ollama connecté — modèle: ${this._model} (${names.length} modèles)`);
            }
        } catch (_) {
            this._ollamaOnline = false;
            console.warn('[LLM] ⚠️ Ollama non disponible — mode regex local');
        }
    }

    /**
     * Configure LLM settings.
     * @param {{ url?: string, model?: string }} opts
     */
    setLLMEndpoint(url, model) {
        if (url) this._ollamaUrl = url.replace(/\/+$/, '');
        if (model) this._model = model;
        this._probeOllama();
    }

    /**
     * Process a human message and generate a mission.
     * 1. Parse intent (local NLU or LLM)
     * 2. Build mission JSON
     * 3. Dispatch to agents
     * 4. Return assistant response
     *
     * @param {string} userMessage
     * @returns {Promise<{text: string, mission: Object|null}>}
     */
    async processMessage(userMessage) {
        this.history.push({ role: 'user', content: userMessage, ts: Date.now() });

        // ── Get system context ──
        const ctx = this._buildContext();

        // ── Route: real LLM or local NLU ──
        if (this._ollamaOnline) {
            return this._processWithLLM(userMessage, ctx);
        } else {
            return this._processWithRegex(userMessage, ctx);
        }
    }

    /**
     * Process via real Ollama LLM — conversational + action dispatch.
     */
    async _processWithLLM(userMessage, ctx) {
        try {
            const response = await this._callOllama(userMessage, ctx);
            this._lastRawResponse = response; // debug cache
            console.log('[LLM] Raw response:', response);

            // Parse actions from LLM response
            const { text, actions } = this._parseLLMResponse(response);

            // Execute any actions returned by the LLM
            let mission = null;
            const feedbacks = [];
            if (actions && actions.length > 0) {
                for (const action of actions) {
                    const result = this._executeAction(action, ctx);
                    if (result?.mission) mission = result.mission;
                    // Build user-visible feedback
                    if (result?.deferred) {
                        feedbacks.push(`🚀 Lancement automatique + décollage avant ${action.type}…`);
                    }
                    if (result?.feedback) {
                        feedbacks.push(result.feedback);
                    }
                    if (!result?.success && result?.reason) {
                        feedbacks.push(`⚠️ ${action.type}: ${result.reason}`);
                    }
                }
            }

            // Append operational feedback to the response
            let finalText = text;
            if (feedbacks.length > 0) {
                finalText += '\n\n' + feedbacks.join('\n');
            }

            this.history.push({ role: 'assistant', content: finalText, ts: Date.now(), mission });
            if (mission) this._emit('mission-dispatched', { mission });
            return { text: finalText, mission };

        } catch (err) {
            console.warn('[LLM] Ollama call failed, falling back to regex:', err.message);
            return this._processWithRegex(userMessage, ctx);
        }
    }

    /**
     * Fallback: process via regex NLU (original behavior).
     */
    async _processWithRegex(userMessage, ctx) {
        const intent = parseIntent(userMessage);
        const mission = this._buildMission(intent, ctx);
        let responseText;

        if (intent.confidence < 0.4) {
            responseText = `Je n'ai pas bien compris. Pouvez-vous préciser votre intention ?\n` +
                `Exemples :\n` +
                `• "Rechercher zone nord"\n` +
                `• "Trouver la balise"\n` +
                `• "Patrouiller zone x=10, z=-20, rayon=15"\n` +
                `• "Formation"\n` +
                `• "Retour base"`;
            this.history.push({ role: 'assistant', content: responseText, ts: Date.now() });
            return { text: responseText, mission: null };
        }

        const result = this._dispatchMission(mission);
        responseText = this._generateResponse(mission, result, ctx);
        this.history.push({ role: 'assistant', content: responseText, ts: Date.now(), mission });
        this._emit('mission-dispatched', { mission, result });
        return { text: responseText, mission };
    }

    // ── Internal: build context ──

    _buildContext() {
        const D = window.DIAMANTS;
        const coord = D?.controller?.multiAgentCoordinator;
        const engine = D?.controller?.autonomousFlightEngine;

        const agentStates = [];
        if (coord?.agents) {
            for (const [id, agent] of coord.agents) {
                agentStates.push({
                    id,
                    phase: agent.phase,
                    position: agent.position ? { x: agent.position.x, y: agent.position.y, z: agent.position.z } : null,
                    brainType: agent.brain?.constructor?.name,
                    role: agent._agentRole || 'unknown',
                });
            }
        }

        return {
            autonomyLevel: window.DIAMANTS_AUTONOMY_LEVEL ?? 50,
            autonomyMode: getAutonomyMode(),
            droneCount: agentStates.length,
            agents: agentStates,
            cognitiveAgents: agentStates.filter(a => a.brainType === 'CognitiveBrain'),
            reactiveAgents: agentStates.filter(a => a.brainType !== 'CognitiveBrain'),
            beacons: [...this.beacons.values()],
            activeMission: this.activeMission,
            engineReady: !!engine,
        };
    }

    // ── Internal: build mission JSON ──

    _buildMission(intent, ctx) {
        _missionCounter++;
        const id = `LLM-${Date.now().toString(36)}-${_missionCounter}`;
        const autonomy = ctx.autonomyMode;

        const base = {
            id,
            type: intent.type,
            priority: 5,
            zone: intent.zone || null,
            target: intent.target || null,
            agents: 'all',
            params: {},
            autonomy,
            timestamp: Date.now(),
        };

        switch (intent.type) {
            case MissionType.SEARCH_ZONE:
                base.priority = 7;
                base.params = {
                    pattern: autonomy === 'centralized' ? 'BOUSTROPHEDON' : 'SWARM_DISPERSION',
                    altitude: 4,
                    speed: 2,
                    overlap: 0.2,
                };
                break;

            case MissionType.SEARCH_BEACON:
                base.priority = 9;
                base.params = {
                    beaconIds: [...this.beacons.keys()],
                    searchRadius: 40,
                    collaborative: true,
                };
                break;

            case MissionType.PATROL:
                base.priority = 5;
                base.params = {
                    patternType: 'perimeter',
                    loopCount: -1, // infinite
                    altitude: 4,
                };
                break;

            case MissionType.FORMATION:
                base.priority = 4;
                base.params = {
                    formationType: 'circle',
                    center: { x: 0, y: 4, z: 0 },
                    radius: 8,
                };
                break;

            case MissionType.RTL:
                base.priority = 8;
                base.agents = 'all';
                break;

            case MissionType.EXPLORE:
                base.priority = 3;
                base.params = {
                    algorithm: autonomy === 'distributed' ? 'voronoi' : 'stigmergy',
                };
                break;
        }

        return base;
    }

    // ── Internal: dispatch mission to agents ──

    _dispatchMission(mission) {
        const D = window.DIAMANTS;
        const coord = D?.controller?.multiAgentCoordinator;
        const engine = D?.controller?.autonomousFlightEngine;

        if (!coord || !engine) {
            return { success: false, reason: 'Système non initialisé — lancez la mission d\'abord' };
        }

        this.activeMission = mission;
        const results = { success: true, dispatched: 0, agents: [] };

        // ── Adapt dispatch strategy based on autonomy mode ──
        const mode = mission.autonomy;

        switch (mission.type) {

            case MissionType.SEARCH_ZONE: {
                const zone = mission.zone;
                if (!zone) return { success: false, reason: 'Zone non définie' };

                // Use the same persistent search pattern as SEARCH_BEACON
                // by converting to a beacon-like search with zone center
                this._dispatchZoneSearch(coord, engine, zone, mission);
                results.dispatched = coord.agents.size;
                break;
            }

            case MissionType.SEARCH_BEACON: {
                // All drones collaborate to find beacon(s)
                this._dispatchBeaconSearch(coord, engine, mission);
                results.dispatched = coord.agents.size;
                break;
            }

            case MissionType.PATROL: {
                this._dispatchPatrol(coord, engine, mission);
                results.dispatched = coord.agents.size;
                break;
            }

            case MissionType.FORMATION: {
                // Force centralized mode temporarily
                engine.missionWaypointProvider = null;
                engine.setAutonomyLevel(0);
                results.dispatched = coord.agents.size;
                break;
            }

            case MissionType.RTL: {
                // Emergency return
                engine.missionWaypointProvider = null;
                for (const [id, agent] of coord.agents) {
                    agent.setPhase?.('returning');
                }
                D.emergencyStop?.();
                results.dispatched = coord.agents.size;
                break;
            }

            case MissionType.EXPLORE:
            default: {
                // Clear any active mission provider to resume normal exploration
                engine.missionWaypointProvider = null;
                engine.startExploration?.();
                results.dispatched = coord.agents.size;
                break;
            }
        }

        this._emit('mission-active', mission);
        return results;
    }

    // ── Dispatch strategies ──

    _dispatchCentralizedSearch(coord, engine, zone, mission) {
        // Split the zone into N sectors (one per agent)
        const agents = [...coord.agents.entries()];
        const N = agents.length;
        const angleStep = (2 * Math.PI) / N;

        for (let i = 0; i < N; i++) {
            const [droneId, agent] = agents[i];
            const angle = angleStep * i;
            const sectorCx = zone.cx + (zone.radius * 0.5) * Math.cos(angle);
            const sectorCz = zone.cz + (zone.radius * 0.5) * Math.sin(angle);

            // Set waypoint target for each drone
            this._setAgentTarget(engine, droneId, sectorCx, 4, sectorCz);

            // Propagate via Raft as mission_order
            agent.raft?.proposeCommand?.({
                type: RaftCommand.MISSION_ORDER,
                payload: {
                    missionId: mission.id,
                    type: 'search_sector',
                    zone: { cx: sectorCx, cz: sectorCz, radius: zone.radius / N * 2 },
                },
            });
        }
    }

    _dispatchHybridSearch(coord, engine, zone, mission) {
        // Cognitive agents get primary search sectors
        // Reactive agents cluster around nearest cognitive
        const cognitive = [];
        const reactive = [];
        for (const [id, agent] of coord.agents) {
            if (agent.brain?.constructor?.name === 'CognitiveBrain') {
                cognitive.push([id, agent]);
            } else {
                reactive.push([id, agent]);
            }
        }

        const NC = cognitive.length || 1;
        const angleStep = (2 * Math.PI) / NC;

        for (let i = 0; i < NC; i++) {
            const [droneId, agent] = cognitive[i];
            const angle = angleStep * i;
            const sx = zone.cx + (zone.radius * 0.6) * Math.cos(angle);
            const sz = zone.cz + (zone.radius * 0.6) * Math.sin(angle);
            this._setAgentTarget(engine, droneId, sx, 4, sz);

            agent.raft?.proposeCommand?.({
                type: RaftCommand.MISSION_ORDER,
                payload: {
                    missionId: mission.id, type: 'search_sector',
                    zone: { cx: sx, cz: sz, radius: zone.radius / NC * 1.5 },
                },
            });
        }

        // Reactive agents follow nearest cognitive
        for (const [id] of reactive) {
            const droneState = engine.drones?.get(id);
            if (droneState) {
                droneState.waypoint = null;  // let engine re-pick via coordinator
                droneState.waypointTimer = 999;
            }
        }
    }

    _dispatchDistributedSearch(coord, zone, mission) {
        // Broadcast mission to all agents via Raft — each decides independently
        for (const [, agent] of coord.agents) {
            agent.raft?.proposeCommand?.({
                type: RaftCommand.MISSION_ORDER,
                payload: {
                    missionId: mission.id,
                    type: 'distributed_search',
                    zone,
                },
            });
        }
    }

    _dispatchBeaconSearch(coord, engine, mission) {
        // ── SEARCH_BEACON: drones do NOT know beacon positions ──
        // They explore using the current doctrine/COA (adaptive, stigmergy,
        // boustrophedon, etc.) and discover beacons naturally via proximity
        // detection (BeaconSystem.checkProximity in the tick loop).

        const N = coord.agents.size;
        const unfound = [...this.beacons.values()].filter(b => !b.found).length;

        // Clear any existing mission waypoint provider — let the engine
        // drive exploration with its autonomy-level + doctrine + COA.
        engine.missionWaypointProvider = null;

        // Ensure exploration is running (flush stale waypoint caches)
        engine.startExploration?.();

        // Read current doctrine/COA for feedback
        const dm = engine.doctrineManager;
        const doctrine = dm?.currentDoctrine?.id || 'exploration';
        const coa = dm?.currentCOA?.id || 'adaptatif';
        const autonomyPct = engine.autonomyLevel ?? 100;
        const modeLabel = autonomyPct >= 90 ? 'distribué'
                        : autonomyPct >= 50 ? 'hybride'
                        : 'centralisé';

        console.log(`[LLM] 🔍 Beacon search active: ${N} drones, ${unfound} beacon(s) to find, doctrine=${doctrine}/${coa}, mode=${modeLabel}`);
        this._emit('search-started', {
            type: 'SEARCH_BEACON',
            drones: N,
            beacons: unfound,
            doctrine,
            coa,
            mode: modeLabel,
        });
    }

    _dispatchZoneSearch(coord, engine, zone, mission) {
        // Install persistent search pattern over a zone (same spiral approach)
        const agents = [...coord.agents.entries()];
        const N = agents.length;
        const searchState = new Map();

        for (let i = 0; i < N; i++) {
            const [droneId] = agents[i];
            // Distribute drones in sectors around zone center
            const angle = (2 * Math.PI / N) * i;
            const startX = zone.cx + zone.radius * 0.3 * Math.cos(angle);
            const startZ = zone.cz + zone.radius * 0.3 * Math.sin(angle);
            searchState.set(droneId, {
                cx: startX, cz: startZ,
                angle: angle,
                ring: 0, step: 0,
                spiralSpacing: 4,
                maxRing: Math.ceil(zone.radius / 4),
            });
            this._setAgentTarget(engine, droneId, startX, 4, startZ);
        }

        const self = this;
        engine.missionWaypointProvider = (droneId, state) => {
            if (!self.activeMission || self.activeMission.id !== mission.id) {
                engine.missionWaypointProvider = null;
                return null;
            }
            const ss = searchState.get(droneId);
            if (!ss) return null;

            ss.step++;
            ss.ring = Math.min(ss.ring + 0.3, ss.maxRing);
            ss.angle += 1.1;

            const r = ss.ring * ss.spiralSpacing;
            const x = ss.cx + r * Math.cos(ss.angle);
            const z = ss.cz + r * Math.sin(ss.angle);

            const bounds = engine.explorationBounds || 60;
            const wp = {
                x: Math.max(-bounds + 2, Math.min(bounds - 2, x)),
                y: 0,
                z: Math.max(-bounds + 2, Math.min(bounds - 2, z)),
            };
            wp._label = '🔍 SEARCH_ZONE';
            return wp;
        };
        console.log(`[LLM] 🔍 Zone search installed: ${N} drones, zone=(${zone.cx},${zone.cz}) r=${zone.radius}m`);
        this._emit('search-started', {
            type: 'SEARCH_ZONE',
            drones: N,
            zone,
        });
    }

    _dispatchPatrol(coord, engine, mission) {
        const agents = [...coord.agents.entries()];
        const N = agents.length;
        const R = 25;

        for (let i = 0; i < N; i++) {
            const [droneId] = agents[i];
            const angle = (2 * Math.PI / N) * i;
            const x = R * Math.cos(angle);
            const z = R * Math.sin(angle);
            this._setAgentTarget(engine, droneId, x, 4, z);
        }
    }

    // ── Helpers ──

    _setAgentTarget(engine, droneId, x, y, z) {
        const droneState = engine.drones?.get(droneId);
        if (!droneState) return;

        // Create a waypoint-like target
        if (window.THREE) {
            droneState.waypoint = new window.THREE.Vector3(x, y, z);
        } else {
            droneState.waypoint = { x, y, z };
        }
        droneState.waypointTimer = 0;

        // Ensure drone is in EXPLORE phase
        if (droneState.phase === 'HOVER' || droneState.phase === 'TAKEOFF') {
            droneState.phase = 'EXPLORE';
        }
    }

    _generateResponse(mission, result, ctx) {
        if (!result.success) {
            return `⚠️ ${result.reason}`;
        }

        const modeLabel = {
            centralized: '🔗 Centralisé',
            guided: '📐 Guidé',
            hybrid: '🔀 Hybride',
            'semi-auto': '🧠 Semi-autonome',
            distributed: '⚡ Distribué',
        }[mission.autonomy] || mission.autonomy;

        let text = '';

        switch (mission.type) {
            case MissionType.SEARCH_ZONE:
                text = `✅ **Mission lancée**: Recherche de zone\n` +
                    `📍 Zone: (${mission.zone.cx}, ${mission.zone.cz}) rayon ${mission.zone.radius}m\n` +
                    `🎛️ Mode: ${modeLabel}\n` +
                    `🚁 ${ctx.droneCount} drones déployés\n` +
                    `📋 Pattern: ${mission.params.pattern}`;
                break;

            case MissionType.SEARCH_BEACON: {
                const unfound = [...this.beacons.values()].filter(b => !b.found).length;
                const dm = window.DIAMANTS?.controller?.autonomousFlightEngine?.doctrineManager;
                const doctrine = dm?.currentDoctrine?.id || 'exploration';
                const coa = dm?.currentCOA?.id || 'adaptatif';
                text = `✅ **Recherche balise lancée**\n` +
                    `🎯 ${unfound} balise(s) à découvrir (position inconnue)\n` +
                    `📋 Doctrine: ${doctrine} • COA: ${coa}\n` +
                    `🎛️ Mode: ${modeLabel}\n` +
                    `🚁 ${ctx.droneCount} drones en exploration auto-organisée`;
                break;
            }

            case MissionType.PATROL:
                text = `✅ **Mission lancée**: Patrouille\n` +
                    `🎛️ Mode: ${modeLabel}\n` +
                    `🚁 ${ctx.droneCount} drones en patrouille périmétrique`;
                break;

            case MissionType.FORMATION:
                text = `✅ **Formation activée**\n` +
                    `📐 Type: cercle, rayon 8m\n` +
                    `🚁 ${ctx.droneCount} drones en formation`;
                break;

            case MissionType.RTL:
                text = `🏠 **Retour base ordonné**\nTous les drones reviennent à la base.`;
                break;

            case MissionType.EXPLORE:
                text = `✅ **Exploration libre lancée**\n` +
                    `🎛️ Mode: ${modeLabel}\n` +
                    `🧭 Algorithme: ${mission.params?.algorithm || 'auto'}`;
                break;

            default:
                text = `✅ Mission ${mission.type} lancée — ${ctx.droneCount} drones.`;
        }

        return text;
    }

    // ── External LLM call — Ollama Chat API ──

    _buildSystemPrompt(ctx) {
        // Beacon summary — include found beacon positions for information queries
        const totalBeacons = this.beacons.size;
        const foundBeacons = [...this.beacons.values()].filter(b => b.found);
        const unfoundBeacons = [...this.beacons.values()].filter(b => !b.found);
        let beaconSummary;
        if (totalBeacons === 0) {
            beaconSummary = 'aucune balise placée';
        } else {
            const parts = [];
            if (foundBeacons.length > 0) {
                const foundList = foundBeacons.map(b => {
                    const pos = b.position ? `(${b.position.x?.toFixed?.(1) ?? b.position.x}, ${b.position.z?.toFixed?.(1) ?? b.position.z})` : '?';
                    const finder = b.foundBy || '?';
                    return `${b.id} à ${pos} (trouvée par ${finder})`;
                }).join(', ');
                parts.push(`Trouvées (${foundBeacons.length}): ${foundList}`);
            }
            if (unfoundBeacons.length > 0) {
                parts.push(`Non trouvées: ${unfoundBeacons.length} (positions inconnues des drones)`);
            }
            beaconSummary = parts.join('\n  ');
        }

        const droneList = ctx.agents.slice(0, 20).map(a =>
            `  - ${a.id}: phase=${a.phase}, pos=${a.position ? `(${a.position.x.toFixed(1)},${a.position.y.toFixed(1)},${a.position.z.toFixed(1)})` : 'inconnue'}, rôle=${a.role}`
        ).join('\n') || '  Aucun drone actif.';

        // Current doctrine/COA
        const dm = window.DIAMANTS_DOCTRINE;
        const currentDoctrine = dm?.currentDoctrine?.id || 'exploration';
        const currentCOA = dm?.currentCOA?.id || 'adaptive';

        // Coverage stats
        const engine = window.DIAMANTS?.controller?.autonomousFlightEngine;
        const visited = engine?.visitedCells?.size || 0;
        const zoneSize = engine?.zoneSize || 100;
        const halfZone = zoneSize / 2;
        const cs = engine?.cellSize || 2;
        const maxCells = Math.floor(zoneSize / cs) ** 2;
        const coveragePct = maxCells > 0 ? (visited / maxCells * 100).toFixed(1) : '?';

        // Scenario knowledge — gather from ScenarioEngine if available, else static
        const scenarioEngine = window.DIAMANTS?.scenarioEngine;
        const scenarioList = scenarioEngine?._scenarios;
        let scenarioBlock;
        if (scenarioList && scenarioList.length > 0) {
            scenarioBlock = scenarioList.map((s, i) => {
                const statusTag = scenarioEngine._activeId === s.id ? ' [ACTIF]' : '';
                return `  ${i + 1}. ${s.name}${statusTag} (${s.category}) — ${s.description}\n     Objectif: ${s.objective}`;
            }).join('\n');
        } else {
            scenarioBlock = `  1. Rupture de Lien (informationnel) — Coupe toutes les communications inter-agents. Chaque drone devient isolé.
     Objectif: Démontrer que la couverture collaborative repose sur l'échange pair-à-pair.
  2. Propagation P2P (informationnel) — Place une balise aléatoire. L'information se propage de proche en proche via BEACON_ALERT.
     Objectif: Visualiser la latence et la topologie de diffusion d'une information critique.
  3. Asymétrie de Connaissance (cognitif) — Efface la mémoire d'exploration de la moitié de la flotte.
     Objectif: Prouver que la convergence des connaissances se fait uniquement par contact radio local.
  4. Perte de Noeud Cognitif (cognitif) — Désactive un agent cognitif haute-capacité (X500).
     Objectif: Tester la résilience face à la perte d'un noeud clé.
  5. Fragmentation Réseau (physique) — Fractionne le réseau en deux sous-réseaux isolés (pairs/impairs).
     Objectif: Observer le comportement autonome de sous-groupes isolés.
  6. Stress Multi-Stimuli (transverse) — Place 5 points d'intérêt simultanément.
     Objectif: Évaluer la coordination multi-objectifs et l'allocation distribuée de ressources.`;
        }

        return `Tu es l'IA de commandement du système DIAMANTS — essaim de drones autonomes pour recherche et sauvetage en forêt.
Tu as le CONTRÔLE TOTAL de la simulation. Tu peux agir directement: lancer, décoller, atterrir, placer des balises, changer la doctrine, le mode de recherche, le niveau d'autonomie.

ÉTAT SYSTÈME:
- ${ctx.droneCount} drones actifs, mode ${ctx.autonomyMode} (${ctx.autonomyLevel}%), engine: ${ctx.engineReady ? 'PRÊT' : 'NON INIT'}
- Doctrine: ${currentDoctrine} • COA: ${currentCOA}
- Mission: ${ctx.activeMission ? ctx.activeMission.type + ' (' + ctx.activeMission.id + ')' : 'aucune'}
- Couverture: ${coveragePct}% (${visited} cellules explorées)
- Balises: ${beaconSummary}

DRONES:
${droneList}

SCÉNARIOS DE DÉMONSTRATION (panneau Scénarios):
Le système possède 6 scénarios de démonstration conçus pour tester la résilience et l'autonomie de l'essaim. Ils sont accessibles via le panneau "Scénarios" de l'interface.
${scenarioBlock}

ACTIONS DISPONIBLES (copie EXACTEMENT ces tags, ne modifie RIEN):

── Contrôle de mission ──
- <<<ACTION:{"type":"LAUNCH"}>>> = démarrer le système (spawn drones)
- <<<ACTION:{"type":"TAKEOFF"}>>> = faire décoller tous les drones
- <<<ACTION:{"type":"LAND"}>>> = faire atterrir tous les drones
- <<<ACTION:{"type":"STOP"}>>> = arrêt d'urgence
- <<<ACTION:{"type":"RTL"}>>> = retour base

── Missions ──
- <<<ACTION:{"type":"SEARCH_BEACON"}>>> = lancer la recherche de balises (auto-organisée selon doctrine/COA)
- <<<ACTION:{"type":"SEARCH_ZONE","zone":{"cx":0,"cz":-30,"radius":25}}>>> = fouiller une zone spécifique
- <<<ACTION:{"type":"EXPLORE"}>>> = exploration libre
- <<<ACTION:{"type":"PATROL"}>>> = patrouille périmétrique
- <<<ACTION:{"type":"FORMATION","params":{"formationType":"circle","radius":8}}>>> = formation

── Balises ──
- <<<ACTION:{"type":"PLACE_BEACON","x":20,"z":-15}>>> = placer une balise à une position précise
- <<<ACTION:{"type":"PLACE_BEACON","random":true,"count":3}>>> = placer N balises à des positions aléatoires
- <<<ACTION:{"type":"CLEAR_BEACONS"}>>> = supprimer toutes les balises

── Configuration ──
- <<<ACTION:{"type":"SET_DOCTRINE","doctrine":"exploration"}>>> = changer la doctrine (exploration, swarm, formation, coverage, search)
- <<<ACTION:{"type":"SET_COA","coa":"stigmergy"}>>> = changer le course of action (adaptive, stigmergy, grid, boustrophedon, spiral, radial, perimeter)
- <<<ACTION:{"type":"SET_AUTONOMY","level":100}>>> = changer le niveau d'autonomie (0=centralisé, 100=distribué)

RÈGLES STRICTES:
1. Réponds TOUJOURS en français, de façon concise et opérationnelle.
2. QUESTIONS D'INFORMATION (positions, état, "où sont...?", "combien...", "c'est quoi..."):
   → Réponds avec les données de l'ÉTAT SYSTÈME ci-dessus. PAS d'action, juste du texte informatif.
   → Pour "où sont les balises?": donne la liste des balises trouvées avec leurs positions, et le nombre de non trouvées.
   → Pour "où sont les drones?": donne les positions depuis la section DRONES.
   → Pour "état du système?" / "ça en est où?": résume missions, couverture, balises, drones.
   → Pour "quels sont les scénarios?": décris les 6 scénarios de la section SCÉNARIOS DE DÉMONSTRATION avec leur nom, catégorie et objectif.
3. DEMANDES D'ACTION (lancer, décoller, placer, changer, chercher...):
   → Écris UNE courte phrase puis les tags <<<ACTION:...>>> sur des lignes séparées à la FIN.
4. NE LISTE JAMAIS toutes les actions possibles. Exécute UNIQUEMENT ce qui est demandé.
5. Si le système n'est pas prêt (engine NON INIT), chaîne: LAUNCH + TAKEOFF + action demandée.
6. Copie les tags ACTION exactement comme ci-dessus. N'ajoute PAS de champs supplémentaires. JSON VALIDE uniquement.
7. Quand on te demande de placer des balises, FAIS-LE avec <<<ACTION:{"type":"PLACE_BEACON",...}>>>. Tu as le pouvoir d'agir.
8. UN SEUL tag ACTION par étape, SAUF si plusieurs étapes sont nécessaires (ex: LAUNCH + TAKEOFF + SEARCH_BEACON).
9. Pour des balises aléatoires, utilise "random":true. Pour une position précise, utilise "x" et "z".
10. IMPORTANT: Distingue TOUJOURS une question d'information d'une demande d'action. "Où sont les balises?" = information, "Cherche les balises" = action.
11. N'utilise JAMAIS d'emojis, d'icônes ou de caractères spéciaux décoratifs (pas de ▶️, ✅, ❓, 🔴, etc.). Texte brut uniquement.
12. Quand on te demande les "scénarios", réponds avec les SCÉNARIOS DE DÉMONSTRATION ci-dessus — PAS les actions/commandes de mission.`;
    }

    async _callOllama(userMessage, ctx) {
        // Build conversation messages (keep last 10 for context window)
        const sysPrompt = this._buildSystemPrompt(ctx);
        const recent = this.history.slice(-10).map(h => ({
            role: h.role === 'assistant' ? 'assistant' : 'user',
            content: h.content,
        }));

        const messages = [
            { role: 'system', content: sysPrompt },
            ...recent,
            { role: 'user', content: userMessage },
        ];

        const res = await fetch(`${this._ollamaUrl}/api/chat`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                model: this._model,
                messages,
                stream: false,
                options: {
                    temperature: 0.4,
                    num_predict: 800,
                },
            }),
        });

        if (!res.ok) throw new Error(`Ollama HTTP ${res.status}`);
        const data = await res.json();
        return data.message?.content || '';
    }

    /**
     * Parse LLM response: extract natural text + action blocks.
     * Actions are embedded as <<<ACTION:{...}>>> in the response.
     */
    _parseLLMResponse(raw) {
        const actions = [];

        // ── Sanitize a captured JSON string so it survives JSON.parse ──
        const _sanitizeJSON = (s) => {
            // 1. Collapse internal newlines / carriage returns
            let j = s.replace(/[\r\n]+/g, ' ').trim();
            // 2. Replace Python-style tuples  (x, y)  →  [x, y]
            j = j.replace(/\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)/g, '[$1, $2]');
            j = j.replace(/\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)/g, '[$1, $2, $3]');
            // 3. Strip trailing commas before } or ]
            j = j.replace(/,\s*([}\]])/g, '$1');
            // 4. Ensure property names are double-quoted (handles unquoted keys)
            j = j.replace(/([{,])\s*([a-zA-Z_]\w*)\s*:/g, '$1"$2":');
            // 5. Replace single-quoted string values with double-quoted
            j = j.replace(/:\s*'([^']*)'/g, ':"$1"');
            return j;
        };

        // ── Primary regex: <<<ACTION:{...}>>> with multiline support ──
        const cleaned = raw.replace(/<<<\s*ACTION\s*:\s*(\{[\s\S]*?\})\s*>>>/g, (match, json) => {
            const sanitized = _sanitizeJSON(json);
            try {
                const action = JSON.parse(sanitized);
                actions.push(action);
                return `\n▶️ **${action.type}**`;
            } catch (e) {
                console.warn('[LLM] Failed to parse action JSON:', json, '→ sanitized:', sanitized, e.message);
                // Last-resort: extract type with regex
                const typeMatch = sanitized.match(/"type"\s*:\s*"([^"]+)"/);
                if (typeMatch) {
                    const fallback = { type: typeMatch[1].trim() };
                    // Try to extract zone/params too
                    const zoneMatch = sanitized.match(/"zone"\s*:\s*(\{[^}]*\})/);
                    if (zoneMatch) try { fallback.zone = JSON.parse(zoneMatch[1]); } catch (_) {}
                    actions.push(fallback);
                    console.log('[LLM] Recovered action from malformed JSON:', fallback);
                    return `\n▶️ **${fallback.type}**`;
                }
            }
            return '';
        });

        // ── Fallback: detect ACTION:{...} without <<< >>> delimiters ──
        if (actions.length === 0) {
            const fallbackRe = /ACTION\s*:\s*(\{[\s\S]*?\})/g;
            let fm;
            while ((fm = fallbackRe.exec(raw)) !== null) {
                const sanitized = _sanitizeJSON(fm[1]);
                try {
                    const action = JSON.parse(sanitized);
                    actions.push(action);
                } catch (e) {
                    const typeMatch = sanitized.match(/"type"\s*:\s*"([^"]+)"/);
                    if (typeMatch) {
                        actions.push({ type: typeMatch[1].trim() });
                    }
                }
            }
        }

        // Clean up the text
        const text = cleaned.replace(/\n{3,}/g, '\n\n').trim();
        return { text, actions };
    }

    /**
     * Check if drones are airborne (at least one in EXPLORE/HOVER).
     */
    _dronesAirborne() {
        const engine = window.DIAMANTS?.controller?.autonomousFlightEngine;
        if (!engine?.drones?.size) return false;
        for (const [, s] of engine.drones) {
            if (s.phase === 'EXPLORE' || s.phase === 'HOVER') return true;
        }
        return false;
    }

    /**
     * Execute a single action from the LLM.
     */
    _executeAction(action, ctx) {
        const D = window.DIAMANTS;
        const coord = D?.controller?.multiAgentCoordinator;
        const engine = D?.controller?.autonomousFlightEngine;

        console.log('[LLM] Executing action:', action.type, action);

        // Special meta-actions that call panel controller
        switch (action.type) {
            case 'LAUNCH': {
                if (typeof window.launchMission === 'function') {
                    window.launchMission();
                    return { success: true, feedback: '🚀 Système lancé' };
                }
                return { success: false, reason: 'launchMission not available' };
            }
            case 'TAKEOFF': {
                if (typeof window.takeoffAllDrones === 'function') {
                    window.takeoffAllDrones();
                    return { success: true, feedback: '🛫 Décollage en cours' };
                }
                return { success: false, reason: 'takeoffAllDrones not available' };
            }
            case 'LAND': {
                if (typeof window.landAllDrones === 'function') {
                    window.landAllDrones();
                    return { success: true, feedback: '🛬 Atterrissage en cours' };
                }
                return { success: false, reason: 'landAllDrones not available' };
            }
            case 'STOP': {
                if (typeof window.emergencyLand === 'function') {
                    window.emergencyLand();
                    return { success: true, feedback: '🛑 Arrêt d\'urgence déclenché' };
                }
                return { success: false, reason: 'emergencyLand not available' };
            }
            case 'PLACE_BEACON': {
                return this._executePlaceBeacon(action);
            }
            case 'SET_DOCTRINE': {
                return this._executeSetDoctrine(action);
            }
            case 'SET_COA': {
                return this._executeSetCOA(action);
            }
            case 'SET_AUTONOMY': {
                return this._executeSetAutonomy(action);
            }
            case 'CLEAR_BEACONS': {
                return this._executeClearBeacons();
            }
        }

        // Mission-type actions: build + dispatch
        const intent = {
            type: action.type,
            zone: action.zone || null,
            target: action.target || null,
            confidence: 0.95,
        };

        const mission = this._buildMission(intent, ctx);
        if (action.params) Object.assign(mission.params, action.params);

        // ── Auto-chain: LAUNCH + TAKEOFF if drones are not airborne ──
        const needsLaunch = !coord || !engine;
        const needsTakeoff = !this._dronesAirborne();

        if (needsLaunch || needsTakeoff) {
            console.log(`[LLM] Auto-chain: launch=${needsLaunch} takeoff=${needsTakeoff} for ${action.type}`);
            this._emit('auto-chain', { action: action.type, launch: needsLaunch, takeoff: needsTakeoff });

            if (needsLaunch && typeof window.launchMission === 'function') {
                window.launchMission();
            }

            // Schedule takeoff + mission dispatch with appropriate delays
            const takeoffDelay = needsLaunch ? 1500 : 0;
            const missionDelay = takeoffDelay + 2500; // takeoff needs ~2.5s to reach hover

            setTimeout(() => {
                if (typeof window.takeoffAllDrones === 'function') {
                    window.takeoffAllDrones();
                    console.log('[LLM] Auto-takeoff triggered');
                }
                // Force EXPLORE after takeoff hover stabilizes
                setTimeout(() => {
                    try {
                        const eng = window.DIAMANTS?.controller?.autonomousFlightEngine;
                        if (eng) eng.startExploration();
                    } catch (_) {}
                }, 2000);
            }, takeoffDelay);

            setTimeout(() => {
                try {
                    this._dispatchMission(mission);
                    console.log(`[LLM] Deferred mission ${mission.type} dispatched`);
                } catch (e) {
                    console.warn('[LLM] Deferred dispatch failed:', e.message);
                }
            }, missionDelay);

            return { success: true, mission, deferred: true };
        }

        const result = this._dispatchMission(mission);
        return { success: result.success, mission, result };
    }

    // ── New action handlers: PLACE_BEACON, SET_DOCTRINE, SET_COA, SET_AUTONOMY ──

    _executePlaceBeacon(action) {
        const placeBeacon = window.DIAMANTS?.placeBeacon;
        if (!placeBeacon) return { success: false, reason: 'placeBeacon not available' };

        const placed = [];

        if (action.random) {
            // Place N beacons at random positions within exploration bounds
            const count = Math.min(action.count || 1, 10); // cap at 10
            const dm = window.DIAMANTS_DOCTRINE;
            const halfX = (dm?.zoneParams?.sizeX || 80) / 2 - 5;
            const halfZ = (dm?.zoneParams?.sizeZ || 80) / 2 - 5;

            for (let i = 0; i < count; i++) {
                const rx = (Math.random() * 2 - 1) * halfX;
                const rz = (Math.random() * 2 - 1) * halfZ;
                const b = placeBeacon(rx, rz, `balise-${i + 1}`);
                if (b) placed.push({ x: rx.toFixed(1), z: rz.toFixed(1), id: b.userData?.beaconId || `beacon-${i}` });
            }

            const feedback = `📍 ${placed.length} balise(s) placée(s) aléatoirement`;
            this._emit('beacons-placed', { count: placed.length, random: true, positions: placed });
            window.dispatchEvent(new CustomEvent('diamants:beacons-updated'));
            return { success: true, feedback };
        } else {
            // Place a single beacon at specified coordinates
            const x = parseFloat(action.x) || 0;
            const z = parseFloat(action.z) || 0;
            const b = placeBeacon(x, z, action.label || 'balise');
            if (b) {
                const feedback = `📍 Balise placée à (${x.toFixed(1)}, ${z.toFixed(1)})`;
                return { success: true, feedback };
            }
            return { success: false, reason: 'placeBeacon returned null' };
        }
    }

    _executeSetDoctrine(action) {
        const dm = window.DIAMANTS_DOCTRINE;
        if (!dm?.setDoctrine) return { success: false, reason: 'DoctrineManager not available' };

        const doctrine = (action.doctrine || '').toLowerCase();
        const valid = ['exploration', 'swarm', 'formation', 'coverage', 'search'];
        if (!valid.includes(doctrine)) {
            return { success: false, reason: `Doctrine inconnue: ${doctrine}. Valides: ${valid.join(', ')}` };
        }

        dm.setDoctrine(doctrine);
        // Sync the UI select
        const select = document.getElementById('mission_type');
        if (select) {
            select.value = doctrine;
            // Trigger change so panel-controller.wireSelects picks it up
            select.dispatchEvent(new Event('change', { bubbles: true }));
        }

        const feedback = `🎯 Doctrine → ${doctrine}`;
        this._emit('config-changed', { key: 'doctrine', value: doctrine });
        console.log(`[LLM] ${feedback}`);
        return { success: true, feedback };
    }

    _executeSetCOA(action) {
        const dm = window.DIAMANTS_DOCTRINE;
        if (!dm?.setCOA) return { success: false, reason: 'DoctrineManager not available' };

        const coa = (action.coa || '').toLowerCase();
        const valid = ['adaptive', 'stigmergy', 'grid', 'boustrophedon', 'spiral', 'radial', 'perimeter'];
        if (!valid.includes(coa)) {
            return { success: false, reason: `COA inconnu: ${coa}. Valides: ${valid.join(', ')}` };
        }

        dm.setCOA(coa);
        // Sync the UI select
        const select = document.getElementById('mode_select');
        if (select) {
            select.value = coa;
            select.dispatchEvent(new Event('change', { bubbles: true }));
        }

        const feedback = `🧭 COA → ${coa}`;
        this._emit('config-changed', { key: 'coa', value: coa });
        console.log(`[LLM] ${feedback}`);
        return { success: true, feedback };
    }

    _executeSetAutonomy(action) {
        // Use the UI controller's stepped slider (0-5 steps mapped to 0-100 levels)
        const uiCtrl = window.diamantsSystem?.uiController;
        const engine = window.DIAMANTS?.controller?.autonomousFlightEngine;

        const level = Math.max(0, Math.min(100, parseInt(action.level) || 50));

        // UI controller handles slider sync, engine sync, and global variable update
        if (uiCtrl?.setAutonomyLevel) {
            uiCtrl.setAutonomyLevel(level);
        } else if (engine?.setAutonomyLevel) {
            // Fallback: direct engine update
            engine.setAutonomyLevel(level);
            // Try to sync the stepped slider manually
            const steps = [0, 25, 50, 75, 90, 100];
            let bestStep = 5;
            for (let i = 0; i < steps.length; i++) {
                if (Math.abs(steps[i] - level) < Math.abs(steps[bestStep] - level)) bestStep = i;
            }
            const slider = document.getElementById('autonomy-slider');
            if (slider) { slider.value = bestStep; slider.dispatchEvent(new Event('input', { bubbles: true })); }
        } else {
            return { success: false, reason: 'Ni UI controller ni engine disponible' };
        }

        const modeName = level <= 10 ? 'Centralisé' : level <= 30 ? 'Guidé' : level <= 50 ? 'Hybride' : level <= 70 ? 'Semi-auto' : level <= 90 ? 'Autonome' : 'Distribué';
        const feedback = `⚡ Autonomie → ${level}% (${modeName})`;
        this._emit('config-changed', { key: 'autonomy', value: level, mode: modeName });
        console.log(`[LLM] ${feedback}`);
        return { success: true, feedback };
    }

    _executeClearBeacons() {
        // Clear from beacon system (3D scene)
        const bs = window.DIAMANTS?.llmChat?.beaconSystem;
        if (bs) bs.clearAll();
        // Clear from mission service registry
        this.beacons.clear();
        // Clear active mission if searching for beacons
        if (this.activeMission?.type === 'SEARCH_BEACON') {
            this.activeMission = null;
            const engine = window.DIAMANTS?.controller?.autonomousFlightEngine;
            if (engine) engine.missionWaypointProvider = null;
        }
        this._emit('beacons-cleared', {});
        window.dispatchEvent(new CustomEvent('diamants:beacons-updated'));
        console.log('[LLM] 🗑️ All beacons cleared');
        return { success: true, feedback: '🗑️ Toutes les balises supprimées' };
    }

    // ── Beacon management (delegated from beacon-system.js) ──

    registerBeacon(id, position) {
        this.beacons.set(id, { id, position, found: false, ts: Date.now() });
    }

    removeBeacon(id) {
        this.beacons.delete(id);
    }

    markBeaconFound(id, foundBy) {
        const b = this.beacons.get(id);
        if (b) {
            b.found = true;
            b.foundTs = Date.now();
            if (foundBy) b.foundBy = foundBy;
            this._emit('beacon-found', b);

            // ── Auto-complete SEARCH_BEACON mission when all beacons found ──
            const unfound = [...this.beacons.values()].filter(bb => !bb.found);
            if (this.activeMission?.type === 'SEARCH_BEACON' && unfound.length === 0) {
                console.log('[LLM] 🎯 All beacons found! Mission complete.');
                const mission = this.activeMission;
                this.activeMission = null;
                // Clear any leftover mission provider
                const engine = window.DIAMANTS?.controller?.autonomousFlightEngine;
                if (engine) engine.missionWaypointProvider = null;
                this._emit('mission-complete', mission);
            }
        }
    }

    // ── Get mission status summary ──

    getStatus() {
        return {
            activeMission: this.activeMission,
            beacons: [...this.beacons.values()],
            historyCount: this.history.length,
            autonomyMode: getAutonomyMode(),
        };
    }
}
