import { describe, it, expect } from 'vitest';
import { AgentModel } from '../intelligence/agent-model-interface.js';
import { registerProvider } from '../intelligence/model-providers/index.js';
import { NeuroSymbolicIntelligenceManager, observe, compass, toEngineResult } from '../intelligence/neurosymbolic-bridge.js';

class Scripted extends AgentModel {
    get name() { return `scripted:${this.config.model}`; }
    async decide() { return { action: 'EXPLORE', direction: this.config.model === 'b' ? 'S' : 'E', confidence: 0.9, reasoning: 'test' }; }
}
registerProvider('scripted', Scripted);

const drone = (id, x, z, extra = {}) => [id, { id, phase: 'EXPLORE', position: { x, y: 8, z }, velocity: { x: 1, z: 0 }, profile: { id: 'X500' }, ...extra }];
const engine = (...ds) => ({ drones: new Map(ds) });
const registry = { X500: { provider: 'scripted', model: 'a', allowedActions: ['EXPLORE', 'HOVER'] } };

describe('neurosymbolic bridge', () => {
    it('compass follows the scene convention (-z is north)', () => {
        expect(compass(0, -1)).toBe('N');
        expect(compass(1, 0)).toBe('E');
        expect(compass(-1, 1)).toBe('SW');
    });

    it('observes only neighbours within radio range', () => {
        const e = engine(drone('x1', 0, 0), drone('x2', 10, 0), drone('x3', 200, 0));
        const o = observe(e.drones.get('x1'), e, 'X500');
        expect(o.neighbours.map(n => n.id)).toEqual(['x2']);
        expect(o.neighbours[0].dir).toBe('E');
    });

    it('turns a decision into a bounded waypoint bias, and a veto into a safety action', () => {
        expect(toEngineResult({ kind: 'decision', decision: { action: 'EXPLORE', direction: 'N', confidence: 0.8 } }).influence)
            .toMatchObject({ direction: { x: 0, z: -1 }, weight: 0.8 });
        expect(toEngineResult({ kind: 'veto', action: 'HOVER', reason: 'COLLISION' }).safetyAction).toBe('HOVER');
        expect(toEngineResult({ kind: 'fallback' })).toBeNull();
    });

    it('drives only the profiles listed, and hot-swaps a model per type', async () => {
        const mgr = new NeuroSymbolicIntelligenceManager(registry);
        mgr.registerDrone('x1', 'X500');
        mgr.registerDrone('cf1', 'CRAZYFLIE');
        mgr.setGlobalEnabled(true);
        expect(mgr.getBrain('cf1')).toBeNull();
        const e = engine(drone('x1', 0, 0));
        const r1 = await mgr.getBrain('x1').evaluate(e.drones.get('x1'), e);
        expect(r1.influence.direction).toEqual({ x: 1, z: 0 });
        mgr.setModelForType('X500', 'b');
        const r2 = await mgr.getBrain('x1').evaluate(e.drones.get('x1'), e);
        expect(r2.influence.direction).toEqual({ x: 0, z: 1 });
        expect(mgr.getStats()).toMatchObject({ requests: 2, accepted: 2 });
    });

    it('skips a thought instead of queueing past maxConcurrent', async () => {
        class Slow extends AgentModel { async decide() { await new Promise(r => setTimeout(r, 50)); return { action: 'EXPLORE', direction: 'N', confidence: 0.9 }; } }
        registerProvider('slow', Slow);
        const mgr = new NeuroSymbolicIntelligenceManager({ X500: { provider: 'slow', allowedActions: ['EXPLORE'] } }, undefined, { maxConcurrent: 2 });
        const e = engine(drone('a', 0, 0), drone('b', 50, 0), drone('c', -50, 0));
        for (const id of ['a', 'b', 'c']) mgr.registerDrone(id, 'X500');
        const out = await Promise.all(['a', 'b', 'c'].map(id => mgr.getBrain(id).evaluate(e.drones.get(id), e)));
        expect(out.filter(Boolean).length).toBe(2);
        expect(mgr.getStats()).toMatchObject({ requests: 2, skipped: 1 });
        expect(mgr.inFlight).toBe(0);
    });

    it('a veto is never skipped, even when every slot is taken', async () => {
        const mgr = new NeuroSymbolicIntelligenceManager(registry, undefined, { maxConcurrent: 1 });
        mgr.registerDrone('x1', 'X500');
        mgr.setGlobalEnabled(true);
        mgr.inFlight = 1;                         // server saturated
        const e = engine(drone('x1', 0, 0, { battery: 5 }));
        const r = await mgr.getBrain('x1').evaluate(e.drones.get('x1'), e);
        expect(r.safetyAction).toBe('EMERGENCY_LAND');
        expect(mgr.inFlight).toBe(1);
        const e2 = engine(drone('x1', 0, 0));
        expect(await mgr.getBrain('x1').evaluate(e2.drones.get('x1'), e2)).toBeNull();
    });

    it('the rules still veto whatever model is plugged in', async () => {
        const mgr = new NeuroSymbolicIntelligenceManager(registry);
        mgr.registerDrone('x1', 'X500');
        const e = engine(drone('x1', 0, 0), drone('x2', 1, 0));
        const r = await mgr.getBrain('x1').evaluate(e.drones.get('x1'), e);
        expect(r.safetyAction).toBe('HOVER');
        expect(mgr.getStats().requests).toBe(0);
    });
});
