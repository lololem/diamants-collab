/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Bring your own model: the neurosymbolic contract
 * ============================================================
 *
 * Any agent — a drone, a ground vehicle — can be given a model that proposes
 * decisions: a language model, a reinforcement-learning policy, a classical
 * planner, a neural network behind an HTTP endpoint. The model never flies the
 * aircraft. It PROPOSES; a symbolic rule layer DISPOSES:
 *
 *      observation ──► rules.before ──► model.decide ──► rules.after ──► decision
 *                          │ veto                            │ reject
 *                          ▼                                 ▼
 *                    imposed action                   reactive fallback
 *
 * - rules.before  runs first, on the observation. A critical rule (battery,
 *                 bounds, collision) returns a VETO: its action is imposed and
 *                 the model is not consulted.
 * - model.decide  returns a Decision, or null when it has nothing useful.
 * - rules.after   checks the Decision: allowed action, confidence threshold,
 *                 bounded effect. A rejected decision is dropped — the flight
 *                 controller carries on with its reactive behaviour.
 *
 * Swap the model, keep the rules: that is how a better model is introduced
 * without re-certifying the safety behaviour.
 */

/**
 * What an agent knows at the moment it decides — its own view only.
 * @typedef {Object} Observation
 * @property {string}  agentId
 * @property {string}  agentType        profile id, e.g. 'X500_V2', 'CRAZYFLIE_2_1'
 * @property {string}  phase            flight phase, e.g. 'EXPLORE', 'HOVER'
 * @property {{x:number,z:number,alt:number}} position   in the agent's own frame
 * @property {number}  speed            m/s
 * @property {number}  battery          0-100
 * @property {Array<{id:string,dist:number,dir:string}>} neighbours   within radio range
 * @property {Array<{dist:number,dir:string,via:string}>} sharedFindings
 *           things other agents reported (beacons, fires…), with who reported them
 * @property {number}  coverage         0-100, share of the zone this agent believes covered
 * @property {Object}  [extra]          anything a specific agent type adds
 */

/**
 * What a model proposes.
 * @typedef {Object} Decision
 * @property {string}  action           must be one of the agent's allowed actions
 * @property {string}  [direction]      'N' | 'NE' | … | 'NW'
 * @property {Object}  [params]         action-specific, e.g. { weights: { frontier: 0.7 } }
 * @property {number}  confidence       0-1
 * @property {string}  [reasoning]      short, for the operator
 * @property {string}  [model]          filled in by the composer
 * @property {number}  [latencyMs]      filled in by the composer
 */

/** A model: anything that turns an Observation into a Decision. */
export class AgentModel {
    /** @param {object} config  provider-specific settings */
    constructor(config = {}) {
        this.config = config;
    }

    /** Human-readable name shown on screen, e.g. 'ollama:my-model'. */
    get name() {
        return this.constructor.name;
    }

    /**
     * @param {Observation} observation
     * @param {{allowedActions:string[], systemPrompt?:string, timeoutMs?:number, signal?:AbortSignal}} context
     *        `signal` is aborted when the composer gives up (timeout): pass it to
     *        fetch() so the server really cancels the work instead of queueing it.
     * @returns {Promise<Decision|null>}
     */
    async decide(observation, context) {
        throw new Error(`${this.name}.decide() is not implemented`);
    }
}

/** A rule layer: the symbolic half. */
export class RuleLayer {
    /**
     * @param {Observation} observation
     * @returns {{veto: {action:string, rule:string, reason:string}}|null}
     */
    before(observation) {
        return null;
    }

    /**
     * @param {Decision} decision
     * @param {Observation} observation
     * @param {{allowedActions:string[], minConfidence?:number}} context
     * @returns {{ok:true, decision:Decision}|{ok:false, reason:string}}
     */
    after(decision, observation, context) {
        return { ok: true, decision };
    }
}

/**
 * Wires a model and a rule layer together. The flight engine only ever talks
 * to this object.
 */
export class NeuroSymbolicAgent {
    /**
     * @param {AgentModel} model
     * @param {RuleLayer}  rules
     * @param {{allowedActions:string[], minConfidence?:number, timeoutMs?:number, systemPrompt?:string}} context
     */
    constructor(model, rules, context) {
        this.model = model;
        this.rules = rules || new RuleLayer();
        // unset fields keep their defaults (an entry without timeoutMs must not mean "0 ms")
        const given = Object.fromEntries(Object.entries(context || {}).filter(([, v]) => v !== undefined));
        this.context = { minConfidence: 0.6, timeoutMs: 15000, ...given };
        this.stats = { calls: 0, vetoes: 0, rejected: 0, accepted: 0, failures: 0 };
    }

    /**
     * @param {Observation} observation
     * @returns {Promise<{kind:'veto'|'decision'|'fallback', action?:string, decision?:Decision, reason?:string}>}
     */
    async step(observation) {
        const veto = this.rules.before(observation);
        if (veto && veto.veto) {
            this.stats.vetoes++;
            return { kind: 'veto', action: veto.veto.action, reason: `${veto.veto.rule}: ${veto.veto.reason}` };
        }

        let decision = null;
        const t0 = Date.now();
        const abort = new AbortController();
        try {
            this.stats.calls++;
            decision = await withTimeout(this.model.decide(observation, { ...this.context, signal: abort.signal }), this.context.timeoutMs, abort);
        } catch (e) {
            this.stats.failures++;
            return { kind: 'fallback', reason: `model failed: ${e.message}` };
        }
        if (!decision) return { kind: 'fallback', reason: 'model returned nothing' };
        decision = { ...decision, model: this.model.name, latencyMs: Date.now() - t0 };

        const checked = this.rules.after(decision, observation, this.context);
        if (!checked.ok) {
            this.stats.rejected++;
            return { kind: 'fallback', reason: `rejected by rules: ${checked.reason}`, decision };
        }
        this.stats.accepted++;
        return { kind: 'decision', decision: checked.decision };
    }
}

function withTimeout(promise, ms, abort) {
    let timer;
    return Promise.race([
        promise,
        new Promise((_, reject) => {
            timer = setTimeout(() => {
                // cancel the request on the server too: an abandoned request left
                // running keeps its model loaded and blocks the next one
                abort?.abort();
                reject(new Error(`timeout after ${ms} ms`));
            }, ms);
        }),
    ]).finally(() => clearTimeout(timer));
}

export const DIRECTIONS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
