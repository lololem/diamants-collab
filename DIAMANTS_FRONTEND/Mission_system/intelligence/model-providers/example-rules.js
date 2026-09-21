/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * An EXAMPLE rule layer — deliberately small, to show the shape. Real
 * deployments write their own: the rules are the part that must be reviewed,
 * tested and certified, whatever model sits behind them.
 */
import { RuleLayer, DIRECTIONS } from '../agent-model-interface.js';

export class ExampleRules extends RuleLayer {
    constructor({ criticalBattery = 15, minSeparation = 1.5 } = {}) {
        super();
        this.criticalBattery = criticalBattery;
        this.minSeparation = minSeparation;
    }

    before(o) {
        if (o.battery < this.criticalBattery) {
            return { veto: { action: 'EMERGENCY_LAND', rule: 'BATTERY_CRITICAL', reason: `battery ${Math.round(o.battery)} %` } };
        }
        const close = (o.neighbours || []).find(n => n.dist < this.minSeparation);
        if (close) {
            return { veto: { action: 'HOVER', rule: 'COLLISION_IMMINENT', reason: `${close.id} at ${close.dist.toFixed(1)} m` } };
        }
        return null;
    }

    after(d, o, context) {
        if (!context.allowedActions.includes(d.action)) return { ok: false, reason: `action ${d.action} not allowed for ${o.agentType}` };
        if (d.direction && !DIRECTIONS.includes(d.direction)) return { ok: false, reason: `direction ${d.direction} unknown` };
        if (d.confidence < (context.minConfidence ?? 0.6)) return { ok: false, reason: `confidence ${d.confidence} below threshold` };
        return { ok: true, decision: d };
    }
}
