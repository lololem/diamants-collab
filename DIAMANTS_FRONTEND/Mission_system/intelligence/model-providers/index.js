/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Example model providers for the neurosymbolic contract
 * (see ../agent-model-interface.js).
 *
 *   ollama             a local Ollama server (any model you pulled or created)
 *   openai-compatible  any /v1/chat/completions server: llama.cpp, vLLM, LM Studio…
 *   http-policy        any HTTP endpoint that takes the observation and returns a
 *                      decision — an RL policy, an ONNX runtime, a planner. No LLM.
 *
 * All three return the same Decision shape; the rule layer does not care which
 * one produced it.
 */
import { AgentModel } from '../agent-model-interface.js';

/** Plain-text rendering of an observation, for language models. */
export function observationToPrompt(o) {
    const n = (o.neighbours || []).map(x => `${x.id}@${Math.round(x.dist)}m ${x.dir}`).join(', ') || 'none';
    const f = (o.sharedFindings || []).map(x => `${Math.round(x.dist)}m ${x.dir} (reported by ${x.via})`).join(', ') || 'none';
    return [
        `AGENT ${o.agentId} (${o.agentType}), phase ${o.phase}`,
        `- Position (own frame): x=${o.position.x.toFixed(1)}, z=${o.position.z.toFixed(1)}, alt=${o.position.alt.toFixed(1)} m`,
        `- Speed ${o.speed.toFixed(1)} m/s · battery ${Math.round(o.battery)} % · zone covered ${Math.round(o.coverage)} %`,
        `- Neighbours in range: ${n}`,
        `- Findings shared by others: ${f}`,
        'Choose the best action and direction.',
    ].join('\n');
}

/** Pull the first JSON object out of a model's text answer. */
export function parseDecision(text) {
    const clean = String(text || '').replace(/<think>[\s\S]*?<\/think>/g, '');
    const m = clean.match(/\{[\s\S]*\}/);
    if (!m) return null;
    try {
        const d = JSON.parse(m[0]);
        if (!d || typeof d.action !== 'string') return null;
        return {
            action: d.action.toUpperCase(),
            direction: typeof d.direction === 'string' ? d.direction.toUpperCase() : undefined,
            params: d.params || d.adjustWeights || undefined,
            confidence: Number.isFinite(+d.confidence) ? Math.max(0, Math.min(1, +d.confidence)) : 0.5,
            reasoning: typeof d.reasoning === 'string' ? d.reasoning.slice(0, 80) : '',
        };
    } catch (_) {
        return null;
    }
}

export class OllamaModel extends AgentModel {
    get name() { return `ollama:${this.config.model}`; }
    async decide(observation, context) {
        const res = await fetch(`${this.config.baseUrl || 'http://localhost:11434'}/api/generate`, {
            method: 'POST',
            signal: context.signal,
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                model: this.config.model,
                system: context.systemPrompt || '',
                prompt: observationToPrompt(observation),
                format: 'json', stream: false,
                keep_alive: this.config.keepAlive || '30m',
                options: { temperature: this.config.temperature ?? 0.2, num_predict: this.config.maxTokens ?? 150 },
            }),
        });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return parseDecision((await res.json()).response);
    }
}

export class OpenAICompatibleModel extends AgentModel {
    get name() { return `openai:${this.config.model}`; }
    async decide(observation, context) {
        const res = await fetch(`${this.config.baseUrl}/v1/chat/completions`, {
            method: 'POST',
            signal: context.signal,
            headers: { 'Content-Type': 'application/json', ...(this.config.apiKey ? { Authorization: `Bearer ${this.config.apiKey}` } : {}) },
            body: JSON.stringify({
                model: this.config.model,
                temperature: this.config.temperature ?? 0.2,
                max_tokens: this.config.maxTokens ?? 150,
                response_format: { type: 'json_object' },
                messages: [
                    { role: 'system', content: context.systemPrompt || '' },
                    { role: 'user', content: observationToPrompt(observation) },
                ],
            }),
        });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const data = await res.json();
        return parseDecision(data.choices?.[0]?.message?.content);
    }
}

/**
 * A non-LLM policy served over HTTP. It receives the structured observation
 * (not a prompt) and must answer with a Decision JSON.
 */
export class HttpPolicyModel extends AgentModel {
    get name() { return `policy:${this.config.name || this.config.url}`; }
    async decide(observation, context) {
        const res = await fetch(this.config.url, {
            method: 'POST',
            signal: context.signal,
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ observation, allowedActions: context.allowedActions }),
        });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const d = await res.json();
        return d && typeof d.action === 'string' ? { ...d, action: d.action.toUpperCase() } : null;
    }
}

const PROVIDERS = { 'ollama': OllamaModel, 'openai-compatible': OpenAICompatibleModel, 'http-policy': HttpPolicyModel };

/** Build a model from a registry entry: { provider, ...settings }. */
export function createModel(entry) {
    const Cls = PROVIDERS[entry?.provider];
    if (!Cls) throw new Error(`unknown model provider "${entry?.provider}" — expected one of ${Object.keys(PROVIDERS).join(', ')}`);
    return new Cls(entry);
}

/** Register a provider of your own: registerProvider('onnx-local', MyOnnxModel). */
export function registerProvider(id, cls) {
    PROVIDERS[id] = cls;
}
