import { describe, it, expect } from 'vitest';
import { AgentModel, NeuroSymbolicAgent } from '../intelligence/agent-model-interface.js';
import { ExampleRules } from '../intelligence/model-providers/example-rules.js';
import { parseDecision, observationToPrompt, createModel, registerProvider } from '../intelligence/model-providers/index.js';

const obs = (over = {}) => ({
    agentId: 'x500_09', agentType: 'X500_V2', phase: 'EXPLORE',
    position: { x: 10, z: -4, alt: 8 }, speed: 3, battery: 80, coverage: 40,
    neighbours: [{ id: 'crazyflie_02', dist: 12, dir: 'NE' }],
    sharedFindings: [{ dist: 28, dir: 'NW', via: 'crazyflie_04' }],
    ...over,
});
const ctx = { allowedActions: ['EXPLORE', 'AVOID', 'HOVER', 'RTL'], minConfidence: 0.6, timeoutMs: 500 };

class FixedModel extends AgentModel {
    get name() { return 'fixed'; }
    async decide() { return this.config.decision; }
}

describe('neurosymbolic contract', () => {
    it('a critical rule vetoes before the model is asked', async () => {
        const model = new FixedModel({ decision: { action: 'EXPLORE', direction: 'N', confidence: 0.9 } });
        const agent = new NeuroSymbolicAgent(model, new ExampleRules(), ctx);
        const r = await agent.step(obs({ battery: 9 }));
        expect(r.kind).toBe('veto');
        expect(r.action).toBe('EMERGENCY_LAND');
        expect(agent.stats.calls).toBe(0);
    });

    it('an allowed, confident decision goes through, tagged with the model name', async () => {
        const agent = new NeuroSymbolicAgent(new FixedModel({ decision: { action: 'EXPLORE', direction: 'NW', confidence: 0.8 } }), new ExampleRules(), ctx);
        const r = await agent.step(obs());
        expect(r.kind).toBe('decision');
        expect(r.decision.model).toBe('fixed');
    });

    it('rules reject a forbidden action or a low confidence — the flight falls back', async () => {
        for (const d of [{ action: 'SELF_DESTRUCT', confidence: 0.99 }, { action: 'EXPLORE', direction: 'N', confidence: 0.3 }]) {
            const agent = new NeuroSymbolicAgent(new FixedModel({ decision: d }), new ExampleRules(), ctx);
            const r = await agent.step(obs());
            expect(r.kind).toBe('fallback');
            expect(r.reason).toMatch(/rejected by rules/);
        }
    });

    it('fields left unset in a registry entry keep their defaults', async () => {
        class Late extends AgentModel { async decide() { await new Promise(r => setTimeout(r, 30)); return { action: 'EXPLORE', direction: 'N', confidence: 0.9 }; } }
        const agent = new NeuroSymbolicAgent(new Late(), new ExampleRules(), { allowedActions: ['EXPLORE'], timeoutMs: undefined, minConfidence: undefined });
        expect(agent.context).toMatchObject({ timeoutMs: 15000, minConfidence: 0.6 });
        expect((await agent.step(obs())).kind).toBe('decision');
    });

    it('a slow model times out into the fallback', async () => {
        class Slow extends AgentModel { async decide() { return new Promise(res => setTimeout(() => res({ action: 'EXPLORE', confidence: 1 }), 2000)); } }
        const r = await new NeuroSymbolicAgent(new Slow(), new ExampleRules(), ctx).step(obs());
        expect(r.kind).toBe('fallback');
        expect(r.reason).toMatch(/timeout/);
    });

    it('a timeout aborts the request the model started', async () => {
        let aborted = false;
        class Hanging extends AgentModel { decide(o, ctx) { return new Promise((_, rej) => ctx.signal.addEventListener('abort', () => { aborted = true; rej(new Error('aborted')); })); } }
        const r = await new NeuroSymbolicAgent(new Hanging(), new ExampleRules(), { ...ctx, timeoutMs: 50 }).step(obs());
        expect(r.kind).toBe('fallback');
        expect(aborted).toBe(true);
    });
});

describe('providers', () => {
    it('parses a JSON decision out of model text, thinking blocks included', () => {
        const d = parseDecision('<think>hmm</think> Sure: {"action":"avoid","direction":"sw","confidence":0.88,"reasoning":"peer close"}');
        expect(d).toMatchObject({ action: 'AVOID', direction: 'SW', confidence: 0.88 });
        expect(parseDecision('no json here')).toBeNull();
    });

    it('renders an observation into a prompt that names who shared what', () => {
        expect(observationToPrompt(obs())).toMatch(/reported by crazyflie_04/);
    });

    it('builds registered providers and accepts new ones', async () => {
        expect(createModel({ provider: 'ollama', model: 'm' }).name).toBe('ollama:m');
        expect(() => createModel({ provider: 'nope' })).toThrow(/unknown model provider/);
        class Onnx extends AgentModel { get name() { return 'onnx'; } async decide() { return { action: 'HOVER', confidence: 1 }; } }
        registerProvider('onnx-local', Onnx);
        expect(createModel({ provider: 'onnx-local' }).name).toBe('onnx');
    });
});
