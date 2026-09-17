/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Plugs YOUR models into the running simulator.
 *
 * Same surface as DroneIntelligenceManager, so the flight engine and the
 * "LLM Intelligence" panel use it unchanged. It is switched on by the
 * controller when `intelligence/model-providers/agent-models.json` exists.
 *
 * For every drone whose profile id has an entry in that file:
 *   engine state ──► Observation ──► NeuroSymbolicAgent.step ──► engine result
 *
 * The engine applies the result exactly as it would any brain's: a bounded
 * waypoint bias (8 m max, clamped to the arena), a label, a safety action.
 * The PID controller still flies the drone.
 */
import { NeuroSymbolicAgent, DIRECTIONS } from './agent-model-interface.js';
import { createModel } from './model-providers/index.js';
import { ExampleRules } from './model-providers/example-rules.js';

const UNIT = {
    N: { x: 0, z: -1 }, NE: { x: 0.707, z: -0.707 }, E: { x: 1, z: 0 }, SE: { x: 0.707, z: 0.707 },
    S: { x: 0, z: 1 }, SW: { x: -0.707, z: 0.707 }, W: { x: -1, z: 0 }, NW: { x: -0.707, z: -0.707 },
};

export function compass(dx, dz) {
    const deg = (Math.atan2(dx, -dz) * 180 / Math.PI + 360) % 360;
    return DIRECTIONS[Math.round(deg / 45) % 8];
}

/** Engine state → Observation. Only what this drone could know. */
export function observe(state, engine, agentType, radioRange = 30) {
    const p = state.position;
    const neighbours = [];
    for (const [id, other] of engine.drones || []) {
        if (id === state.id) continue;
        const dx = other.position.x - p.x, dz = other.position.z - p.z;
        const dist = Math.hypot(dx, dz);
        if (dist <= radioRange) neighbours.push({ id, dist, dir: compass(dx, dz) });
    }
    let coverage = 0;
    if (engine.visitedCells && engine.cellSize && engine._getHalfBoundsXZ) {
        const hb = engine._getHalfBoundsXZ();
        coverage = Math.min(100, 100 * engine.visitedCells.size * engine.cellSize ** 2 / (4 * hb.x * hb.z));
    }
    return {
        agentId: state.id,
        agentType,
        phase: state.phase || 'IDLE',
        position: { x: p.x, z: p.z, alt: p.y },
        speed: Math.hypot(state.velocity?.x || 0, state.velocity?.z || 0),
        battery: Number.isFinite(state.battery) ? state.battery : 100,
        neighbours: neighbours.sort((a, b) => a.dist - b.dist).slice(0, 5),
        sharedFindings: [],
        coverage,
    };
}

/** step() outcome → what the engine consumes in _llmResults. */
export function toEngineResult(outcome) {
    if (outcome.kind === 'veto') {
        return { reasoning: outcome.reason, safetyAction: outcome.action, influence: null };
    }
    if (outcome.kind !== 'decision') return null;
    const d = outcome.decision;
    const dir = UNIT[d.direction];
    return {
        reasoning: d.reasoning || d.action,
        safetyAction: null,
        influence: {
            direction: dir || null,
            weight: dir ? d.confidence : 0,
            adjustWeights: d.params?.weights || null,
        },
    };
}

class Brain {
    constructor(id, type, entry, rules, manager) {
        this.manager = manager;
        this.id = id;
        this.type = type;
        this.profile = { enableLLM: true };
        this.enabled = false;
        this.busy = false;
        this.agent = new NeuroSymbolicAgent(createModel(entry), rules, {
            allowedActions: entry.allowedActions,
            minConfidence: entry.minConfidence,
            timeoutMs: entry.timeoutMs,
            systemPrompt: entry.systemPrompt,
        });
    }

    async evaluate(state, engine) {
        if (this.busy) return null;            // one request per drone at a time
        const obs = observe(state, engine, this.type);
        // A local server answers one request at a time: past `maxConcurrent`
        // the MODEL call is skipped (it comes back at the next waypoint) rather
        // than queued into a timeout. The rules are not: a veto never waits
        // for a free slot.
        const m = this.manager;
        const saturated = m && m.inFlight >= m.maxConcurrent;
        if (saturated && !this.agent.rules.before(obs)?.veto) { m.skipped++; return null; }
        this.busy = true;
        if (m && !saturated) m.inFlight++;
        try {
            const outcome = await this.agent.step(obs);
            if (typeof window !== 'undefined') {
                const d = outcome.decision || {};
                window.dispatchEvent(new CustomEvent('diamants:llm-decision', {
                    detail: {
                        droneId: this.id, droneType: this.type,
                        kind: outcome.kind,
                        action: outcome.kind === 'veto' ? outcome.action : d.action,
                        direction: d.direction, confidence: d.confidence ?? (outcome.kind === 'veto' ? 1 : 0),
                        reasoning: outcome.kind === 'decision' ? d.reasoning : outcome.reason,
                        model: this.agent.model.name, latencyMs: d.latencyMs, observation: obs,
                    },
                }));
            }
            return toEngineResult(outcome);
        } finally {
            this.busy = false;
            if (m && !saturated) m.inFlight--;
        }
    }

    setEnabled(v) { this.enabled = !!v; }
    reset() { /* stateless */ }
}

export class NeuroSymbolicIntelligenceManager {
    /**
     * @param {object} registry  parsed agent-models.json: { PROFILE_ID: entry }
     * @param {RuleLayer} [rules] your rule layer; ExampleRules by default
     * @param {{maxConcurrent?: number}} [options] requests in flight at once, all drones together
     *        (default 2; raise it for a server that batches, e.g. vLLM)
     */
    constructor(registry, rules = new ExampleRules(), options = {}) {
        this.maxConcurrent = options.maxConcurrent ?? registry?._settings?.maxConcurrent ?? 2;
        this.inFlight = 0;
        this.skipped = 0;
        this.registry = registry;
        this.rules = rules;
        this.brains = new Map();
        this.globalEnabled = false;
        this.connector = { isAvailable: true, _connected: true, _requestCount: 0 };
    }

    async init() { return true; }

    registerDrone(id, type) {
        const entry = this.registry[type];
        if (!entry || !entry.provider) return;
        const brain = new Brain(id, type, entry, this.rules, this);
        brain.setEnabled(this.globalEnabled);
        this.brains.set(id, brain);
    }

    getBrain(id) {
        const b = this.brains.get(id);
        if (b) this.connector._requestCount = this.getStats().requests;
        return b || null;
    }

    async evaluate(id, state, engine) {
        return this.brains.get(id)?.evaluate(state, engine) ?? null;
    }

    setEnabled(id, v) { this.brains.get(id)?.setEnabled(v); }
    setGlobalEnabled(v) {
        this.globalEnabled = !!v;
        for (const b of this.brains.values()) b.setEnabled(v);
    }
    setInfluence() { /* bounded by the engine */ }
    setGlobalInfluence() { /* bounded by the engine */ }

    /** Hot-swap: give every drone of a profile a different model, rules unchanged. */
    setModelForType(type, model) {
        const entry = { ...this.registry[type], model };
        this.registry[type] = entry;
        for (const b of this.brains.values()) if (b.type === type) b.agent.model = createModel(entry);
    }
    setModel(id, model) {
        const b = this.brains.get(id);
        if (b) b.agent.model = createModel({ ...this.registry[b.type], model });
    }
    setOllamaUrl() { /* per entry, in agent-models.json */ }

    reset() {}
    resetAll() {}

    getStats() {
        const s = { requests: 0, vetoes: 0, rejected: 0, accepted: 0, failures: 0 };
        for (const b of this.brains.values()) for (const k of Object.keys(s)) s[k] += b.agent.stats[k === 'requests' ? 'calls' : k];
        return {
            ...s,
            skipped: this.skipped,
            activeBrains: [...this.brains.values()].filter(b => b.enabled).length,
            connector: { requests: s.requests, cacheHits: 0, failures: s.failures + s.rejected },
        };
    }
}
