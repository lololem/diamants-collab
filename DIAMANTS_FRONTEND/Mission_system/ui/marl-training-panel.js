/**
 * DIAMANTS — MARL Training Panel
 * =================================
 * Panneau d'entraînement Multi-Agent Reinforcement Learning.
 * Permet de lancer des cycles d'apprentissage collaboratif fédéré
 * directement depuis le frontend, avec visualisation temps réel
 * des courbes de convergence par agent.
 *
 * Fonctionnalités:
 *   - Configuration: rounds, steps/round, stratégie fédérée, lr, gamma
 *   - Lancer/Pause/Stop training
 *   - Courbes de reward + coverage par agent (canvas)
 *   - Barre de progression par round
 *   - État individuel de chaque agent cognitif
 *   - Curriculum learning (5 stages: Novice → Expert)
 *   - PPO-based policy optimization (replaces REINFORCE)
 *   - Persistent weight storage across sessions (localStorage)
 *   - Auto warm-start: each session improves on the previous
 *
 * Toggle: touche [T] (Training)
 * Écoute: diamants:marl-training-tick, diamants:drone-positions
 */

import {
    GymnasiumSwarmEnv,
    PPOPolicy,
    MARLSessionStore,
    GYM_OBS_DIM,
    GymObsIndex,
    CURRICULUM,
    BoxSpace,
} from '../agent/gymnasium-env.js';

const log = (...a) => console.log('[MARL-UI]', ...a);

// ═══════════════════════════════════════════════════════════════════
// Federated Trainer (in-browser) — uses GymnasiumSwarmEnv + PPOPolicy
// Now follows Gymnasium API: step → {obs, rewards, terminated, truncated, info}
// Weights persist across sessions via MARLSessionStore
// ═══════════════════════════════════════════════════════════════════

class BrowserFederatedTrainer {
    constructor(config = {}) {
        this.numAgents = config.numAgents || 4;
        this.numRounds = config.numRounds || 30;
        this.stepsPerRound = config.stepsPerRound || 128;
        this.strategy = config.strategy || 'fedavg';
        this.lr = config.lr || 0.0003;
        this.gamma = config.gamma || 0.99;
        this.curriculumStage = config.curriculumStage ?? 1;

        // Session store for persistence
        this.store = config.store || null;

        // Gymnasium-aligned environment
        this.env = new GymnasiumSwarmEnv({
            numAgents: this.numAgents,
            curriculumStage: this.curriculumStage,
            maxSteps: this.stepsPerRound,
        });

        // PPO policies (one per agent)
        this.policies = [];
        for (let i = 0; i < this.numAgents; i++) {
            this.policies.push(new PPOPolicy(GYM_OBS_DIM, 64, 4));
        }

        // Warm-start: restore weights from previous session
        if (this.store) {
            const restored = this.store.restore(this.policies);
            if (restored) {
                log(`♻️ Warm-start: poids restaurés de la session précédente`);
            }
        }

        this.currentRound = 0;
        this.running = false;
        this.paused = false;

        // History
        this.history = {
            rounds: [],
            coverage: [],
            rewards: [],           // avg reward per round
            agentRewards: [],      // per-agent per round: [[a0,a1,a2,a3], ...]
            agentCoverage: [],
        };

        this._onTick = null;
        this._onComplete = null;
    }

    async run(onTick, onComplete) {
        this.running = true;
        this.paused = false;
        this._onTick = onTick;
        this._onComplete = onComplete;

        for (this.currentRound = 0; this.currentRound < this.numRounds; this.currentRound++) {
            if (!this.running) break;

            while (this.paused) {
                await new Promise(r => setTimeout(r, 100));
                if (!this.running) break;
            }
            if (!this.running) break;

            const roundResult = await this._runOneRound();

            // Federated aggregation
            this._federatedAggregate();

            // Record history
            this.history.rounds.push(this.currentRound);
            this.history.coverage.push(roundResult.coverage);
            this.history.rewards.push(roundResult.avgReward);
            this.history.agentRewards.push(roundResult.perAgentReward);
            this.history.agentCoverage.push(roundResult.perAgentCoverage);

            // Emit tick
            if (this._onTick) this._onTick(this._getTickData());

            // Dispatch global event for other UI components
            window.dispatchEvent(new CustomEvent('diamants:marl-training-tick', {
                detail: this._getTickData()
            }));

            // Yield to UI thread every round
            await new Promise(r => setTimeout(r, 0));
        }

        this.running = false;

        // Persist trained weights
        if (this.store) {
            this.store.save(this.policies, this.history, this.curriculumStage);
            log(`💾 Poids sauvegardés après ${this.history.rounds.length} rounds`);
        }

        if (this._onComplete) this._onComplete(this.history);
    }

    async _runOneRound() {
        // Gymnasium API: reset → {obs, info}
        const resetResult = this.env.reset();
        let currentObs = resetResult.obs;

        const trajectories = [];
        for (let i = 0; i < this.numAgents; i++) trajectories.push([]);

        for (let s = 0; s < this.stepsPerRound; s++) {
            const actions = [];
            const envActions = [];

            for (let i = 0; i < this.numAgents; i++) {
                const obs = new Float32Array(currentObs[i]);
                // PPO: sample action with log probability and value estimate
                const { action, logProb, value } = this.policies[i].sampleAction(obs);
                const envAct = this.policies[i].toEnvAction(action);
                actions.push(action);
                envActions.push(envAct);
                trajectories[i].push({ obs: currentObs[i], action: Array.from(action), logProb, value, reward: 0, done: false });
            }

            // Gymnasium API: step → {obs, rewards, terminated, truncated, info}
            const result = this.env.step(envActions);
            currentObs = result.obs;
            const done = result.terminated || result.truncated;

            // Assign rewards and done flags
            for (let i = 0; i < this.numAgents; i++) {
                trajectories[i][trajectories[i].length - 1].reward = result.rewards[i];
                trajectories[i][trajectories[i].length - 1].done = done;
            }

            if (done) break;
        }

        // PPO update for each agent (replaces REINFORCE)
        // ppoUpdate is async — yields between mini-batches for UI responsiveness
        for (let i = 0; i < this.numAgents; i++) {
            await this.policies[i].ppoUpdate(trajectories[i], this.lr);
        }

        const perAgentReward = this.env.agents.map(a => a.cumulativeReward);
        const avgReward = perAgentReward.reduce((s, v) => s + v, 0) / this.numAgents;
        const perAgentCoverage = this.env.agents.map(() => this.env.getCoverage());

        return {
            coverage: this.env.getCoverage(),
            avgReward,
            perAgentReward: [...perAgentReward],
            perAgentCoverage: [...perAgentCoverage],
        };
    }

    _federatedAggregate() {
        if (this.strategy === 'none') return;

        // FedAvg: average all policy weights
        const allWeights = this.policies.map(p => p.getWeights());
        const dim = allWeights[0].length;
        const global = new Float32Array(dim);

        if (this.strategy === 'fedavg') {
            for (let w = 0; w < dim; w++) {
                let sum = 0;
                for (const weights of allWeights) sum += weights[w];
                global[w] = sum / this.numAgents;
            }
        } else if (this.strategy === 'weighted') {
            // Reward-weighted
            const rewards = this.env.agents.map(a => Math.max(0.01, a.cumulativeReward + 100));
            const totalR = rewards.reduce((s, v) => s + v, 0);
            for (let w = 0; w < dim; w++) {
                let sum = 0;
                for (let i = 0; i < this.numAgents; i++) sum += allWeights[i][w] * rewards[i];
                global[w] = sum / totalR;
            }
        } else if (this.strategy === 'fedprox') {
            // FedProx: average + small pull
            const mu = 0.01;
            for (let w = 0; w < dim; w++) {
                let sum = 0;
                for (const weights of allWeights) sum += weights[w];
                global[w] = sum / this.numAgents;
            }
            // Proximal pull toward previous (softly)
            if (this._prevGlobal) {
                for (let w = 0; w < dim; w++) {
                    global[w] = global[w] * (1 - mu) + this._prevGlobal[w] * mu;
                }
            }
        }

        this._prevGlobal = new Float32Array(global);

        // Distribute to all agents
        for (const p of this.policies) p.setWeights(new Float32Array(global));
    }

    _getTickData() {
        const lastIdx = this.history.rounds.length - 1;
        return {
            round: this.currentRound,
            totalRounds: this.numRounds,
            coverage: lastIdx >= 0 ? this.history.coverage[lastIdx] : 0,
            avgReward: lastIdx >= 0 ? this.history.rewards[lastIdx] : 0,
            perAgentReward: lastIdx >= 0 ? this.history.agentRewards[lastIdx] : [],
            perAgentCoverage: lastIdx >= 0 ? this.history.agentCoverage[lastIdx] : [],
            perAgentLoss: lastIdx >= 0 ? (this.history.agentLoss?.[lastIdx] ?? []) : [],
            strategy: this.strategy,
            lr: this.lr,
            stepsPerRound: this.stepsPerRound,
            curriculum: this.curriculumStage,
            curriculumName: CURRICULUM[this.curriculumStage]?.name || '?',
            history: this.history,
        };
    }

    stop() { this.running = false; }
    pause() { this.paused = true; }
    resume() { this.paused = false; }
}

// ═══════════════════════════════════════════════════════════════════
// Live Federated Trainer — operates on REAL mission data
// Listens to diamants:federated-update for live metrics.
// Periodically performs federated averaging on AgentBrain Q-tables.
// Zero heavy computation — averaging Float32Arrays takes <1ms.
// ═══════════════════════════════════════════════════════════════════

export class LiveFederatedTrainer {
    constructor(config = {}) {
        this.numAgents = config.numAgents || 4;
        this.numRounds = config.numRounds || 30;
        this.roundIntervalMs = (config.roundIntervalSec || 10) * 1000;
        this.strategy = config.strategy || 'fedavg';
        this.mixingRate = config.mixingRate ?? 0.5;
        this.coordinator = config.coordinator || null;

        this.currentRound = 0;
        this.running = false;
        this.paused = false;

        this.history = {
            rounds: [],
            coverage: [],
            rewards: [],
            agentRewards: [],
            agentCoverage: [],
        };

        this._latestUpdate = null;
        this._eventHandler = null;
        this._roundTimerId = null;
        this._onTick = null;
        this._onComplete = null;
        this._prevGlobal = null;
        this.fedHistory = [];
    }

    async run(onTick, onComplete) {
        this.running = true;
        this.paused = false;
        this._onTick = onTick;
        this._onComplete = onComplete;

        // Listen to real federated-update events from MultiAgentCoordinator
        this._eventHandler = (evt) => {
            if (this.running) this._latestUpdate = evt.detail;
        };
        window.addEventListener('diamants:federated-update', this._eventHandler);

        // Run rounds on a timer (non-blocking, no heavy compute)
        this._roundTimerId = setInterval(() => {
            if (!this.running || this.paused) return;
            this._processRound();
            this.currentRound++;
            if (this.currentRound >= this.numRounds) {
                this.stop();
                if (this._onComplete) this._onComplete(this.history);
            }
        }, this.roundIntervalMs);
    }

    _processRound() {
        // Try event data first, fallback to reading coordinator directly
        let update = this._latestUpdate;
        if (!update && this.coordinator?.agents) {
            // Build update from coordinator state
            const agentsArr = [];
            let totalReward = 0;
            for (const [id, agent] of this.coordinator.agents) {
                const brain = agent.brain;
                const reward = brain?.avgReward ?? 0;
                let localCov = 0;
                try {
                    const m = agent.getMetrics?.();
                    if (m) localCov = parseFloat(m.coverage) || 0;
                } catch (_) { /* agent not fully initialized */ }
                agentsArr.push({ id, localCoverage: localCov / 100, reward, position: agent.position });
                totalReward += reward;
            }
            // Use actual coordinator method for global coverage (property doesn't exist)
            const gc = typeof this.coordinator._computeGlobalCoveragePercent === 'function'
                ? this.coordinator._computeGlobalCoveragePercent() / 100
                : 0;
            update = {
                globalCoverage: gc,
                avgReward: agentsArr.length > 0 ? totalReward / agentsArr.length : 0,
                agents: agentsArr,
            };
        }
        const coverage = update?.globalCoverage || 0;
        const avgReward = update?.avgReward || 0;
        const agents = update?.agents || [];

        // Extract per-agent metrics ordered by DRONE_ID_MAP
        const droneMap = MARLTrainingPanel.DRONE_ID_MAP;
        const perAgentReward = droneMap.map(id => {
            const a = agents.find(ag => ag.id === id);
            return a?.reward ?? a?.localCoverage ?? 0;
        });
        const perAgentCoverage = droneMap.map(id => {
            const a = agents.find(ag => ag.id === id);
            return a?.localCoverage ?? 0;
        });

        // Collect real TD-error loss from each agent brain
        const perAgentLoss = droneMap.map(id => {
            const agent = this.coordinator?.agents?.get(id);
            return agent?.brain?.lastLoss ?? 0;
        });

        // Federated averaging on real Q-tables
        const fedResult = this._federatedAverageQTables();
        this.fedHistory.push(fedResult);

        this.history.rounds.push(this.currentRound);
        this.history.coverage.push(coverage);
        this.history.rewards.push(avgReward);
        this.history.agentRewards.push(perAgentReward);
        this.history.agentCoverage.push(perAgentCoverage);
        if (!this.history.agentLoss) this.history.agentLoss = [];
        this.history.agentLoss.push(perAgentLoss);

        if (this._onTick) this._onTick(this._getTickData());

        window.dispatchEvent(new CustomEvent('diamants:marl-training-tick', {
            detail: this._getTickData()
        }));
    }

    /**
     * Federated averaging on REAL AgentBrain Q-table weights.
     * Ultra lightweight: just Float32Array averaging, takes <1ms.
     */
    _federatedAverageQTables() {
        if (this.strategy === 'none') return { applied: false, reason: 'strategy=none' };
        if (!this.coordinator?.agents) return { applied: false, reason: 'no coordinator' };

        const brains = [];
        for (const [id, agent] of this.coordinator.agents) {
            if (agent.brain?.weights && agent.brain.brainType !== 'reactive') {
                brains.push(agent.brain);
            }
        }
        if (brains.length < 2) return { applied: false, reason: 'not enough trainable agents' };

        const dim = brains[0].weights.length;
        if (!brains.every(b => b.weights.length === dim)) {
            return { applied: false, reason: 'weight dimension mismatch' };
        }

        const global = new Float32Array(dim);

        if (this.strategy === 'fedavg') {
            for (let w = 0; w < dim; w++) {
                let sum = 0;
                for (const brain of brains) sum += brain.weights[w];
                global[w] = sum / brains.length;
            }
        } else if (this.strategy === 'weighted') {
            const rewards = brains.map(b => Math.max(0.01, (b.avgReward || 0) + 100));
            const totalR = rewards.reduce((s, v) => s + v, 0);
            for (let w = 0; w < dim; w++) {
                let sum = 0;
                for (let i = 0; i < brains.length; i++) sum += brains[i].weights[w] * rewards[i];
                global[w] = sum / totalR;
            }
        } else if (this.strategy === 'fedprox') {
            const mu = 0.01;
            for (let w = 0; w < dim; w++) {
                let sum = 0;
                for (const brain of brains) sum += brain.weights[w];
                global[w] = sum / brains.length;
            }
            if (this._prevGlobal) {
                for (let w = 0; w < dim; w++) {
                    global[w] = global[w] * (1 - mu) + this._prevGlobal[w] * mu;
                }
            }
        }

        this._prevGlobal = new Float32Array(global);

        // Apply global weights with mixing rate
        const mix = this.mixingRate;
        for (const brain of brains) {
            for (let w = 0; w < dim; w++) {
                brain.weights[w] = brain.weights[w] * (1 - mix) + global[w] * mix;
            }
            if (brain.targetWeights) brain.targetWeights.set(brain.weights);
        }

        return { applied: true, agentsUpdated: brains.length, dimensions: dim };
    }

    _getTickData() {
        const lastIdx = this.history.rounds.length - 1;
        return {
            round: this.currentRound,
            totalRounds: this.numRounds,
            coverage: lastIdx >= 0 ? this.history.coverage[lastIdx] : 0,
            avgReward: lastIdx >= 0 ? this.history.rewards[lastIdx] : 0,
            perAgentReward: lastIdx >= 0 ? this.history.agentRewards[lastIdx] : [],
            perAgentCoverage: lastIdx >= 0 ? this.history.agentCoverage[lastIdx] : [],
            perAgentLoss: lastIdx >= 0 ? (this.history.agentLoss?.[lastIdx] ?? []) : [],
            strategy: this.strategy,
            mixingRate: this.mixingRate,
            history: this.history,
        };
    }

    stop() {
        this.running = false;
        if (this._roundTimerId) { clearInterval(this._roundTimerId); this._roundTimerId = null; }
        if (this._eventHandler) {
            window.removeEventListener('diamants:federated-update', this._eventHandler);
            this._eventHandler = null;
        }
    }

    pause() { this.paused = true; }
    resume() { this.paused = false; }
}

// ═══════════════════════════════════════════════════════════════════
// UI Panel
// ═══════════════════════════════════════════════════════════════════

const AGENT_COLORS = ['#a78bfa', '#67e8f9', '#f59e0b', '#f472b6'];
// Only the 3 REAL cognitive drones (X500/S500 with CognitiveBrain)
// crazyflie_08 is reactive — excluded from MARL panel
const AGENT_NAMES = ['X500_09', 'S500_10', 'X500_11'];

export class MARLTrainingPanel {
    constructor() {
        this._visible = false;
        this._root = null;
        this._trainer = null;
        this._chartCanvas = null;
        this._chartCtx = null;

        // Session store for persistent learning across sessions
        this._store = new MARLSessionStore();

        // Config defaults
        this._config = {
            rounds: 30,
            roundIntervalSec: 10,
            strategy: 'fedavg',
            mixingRate: 0.5,
            curriculumStage: this._store.getRecommendedCurriculum(),
        };

        // Before-metrics snapshot (set when training starts)
        this._beforeMetrics = null;

        this._buildDOM();
        this._hookKeyboard();

        const stats = this._store.getStats();
        if (stats.sessionCount > 0) {
            log(`✅ MARLTrainingPanel initialized — session #${stats.sessionCount + 1}, ` +
                `best=${(stats.bestCoverage * 100).toFixed(1)}%, ` +
                `curriculum=${stats.curriculumName}`);
        } else {
            log('✅ MARLTrainingPanel initialized — première session');
        }
    }

    // =========================================================================
    // DOM
    // =========================================================================

    _buildDOM() {
        this._root = document.createElement('div');
        this._root.id = 'marl-training-panel';

        // ── Position: center in the 3D viewport (right of 420px sidebar) ──
        const SW = 420;
        const vpW = window.innerWidth - SW;
        const pW = Math.min(720, vpW - 40);
        const pH = Math.min(600, window.innerHeight - 60);
        const pL = SW + Math.round((vpW - pW) / 2);
        const pT = Math.round((window.innerHeight - pH) / 2);

        this._root.style.cssText = `
            position: fixed;
            top: ${pT}px;
            left: ${pL}px;
            width: ${pW}px;
            height: ${pH}px;
            background: rgba(5, 12, 24, 0.97);
            border: 2px solid #7c3aed;
            border-radius: 14px;
            font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace;
            font-size: 13px;
            color: #d4d4d8;
            z-index: 7000;
            display: none;
            flex-direction: column;
            backdrop-filter: blur(10px);
            box-shadow: 0 8px 48px rgba(124, 58, 237, 0.4);
            overflow: hidden;
            resize: both;
            min-width: 380px;
            min-height: 280px;
        `;

        this._root.innerHTML = `
        <div id="marl-header" style="padding: 10px 14px; background: linear-gradient(90deg, #5b21b6, #7c3aed); border-radius: 12px 12px 0 0; display: flex; justify-content: space-between; align-items: center; cursor: grab; user-select: none; flex-shrink: 0;">
            <span style="font-size: 15px; font-weight: 700; letter-spacing: 0.5px;">🧠 MARL — Apprentissage Collaboratif</span>
            <span style="font-size: 11px; opacity: 0.7;">☰ glisser | [T] fermer</span>
        </div>

        <!-- Config Section -->
        <div class="marl-body" style="padding: 8px 14px; border-bottom: 1px solid #1e293b; flex-shrink: 0; overflow-y: auto;">
            <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(90px, 1fr)); gap: 6px; margin-bottom: 6px;">
                <label style="display: flex; flex-direction: column; gap: 2px;">
                    <span style="font-size: 10px; color: #94a3b8; text-transform: uppercase; font-weight: 600;">Rounds</span>
                    <input type="number" id="marl-rounds" value="30" min="1" max="10000" style="background: #1e293b; border: 1px solid #334155; color: #e2e8f0; padding: 5px; border-radius: 4px; font-size: 12px; font-family: inherit; width: 100%; box-sizing: border-box;">
                </label>
                <label style="display: flex; flex-direction: column; gap: 2px;">
                    <span style="font-size: 10px; color: #94a3b8; text-transform: uppercase; font-weight: 600;">Intervalle (s)</span>
                    <input type="number" id="marl-steps" value="10" min="1" max="300" style="background: #1e293b; border: 1px solid #334155; color: #e2e8f0; padding: 5px; border-radius: 4px; font-size: 12px; font-family: inherit; width: 100%; box-sizing: border-box;">
                </label>
                <label style="display: flex; flex-direction: column; gap: 2px;">
                    <span style="font-size: 10px; color: #94a3b8; text-transform: uppercase; font-weight: 600;">Stratégie</span>
                    <select id="marl-strategy" style="background: #1e293b; border: 1px solid #334155; color: #e2e8f0; padding: 5px; border-radius: 4px; font-size: 12px; font-family: inherit; width: 100%; box-sizing: border-box;">
                        <option value="fedavg" selected>FedAvg</option>
                        <option value="fedprox">FedProx</option>
                        <option value="weighted">Weighted</option>
                        <option value="none">Indépendant</option>
                    </select>
                </label>
                <label style="display: flex; flex-direction: column; gap: 2px;">
                    <span style="font-size: 10px; color: #94a3b8; text-transform: uppercase; font-weight: 600;">Taux Mixage</span>
                    <input type="number" id="marl-lr" value="0.5" min="0" max="1" step="0.05" style="background: #1e293b; border: 1px solid #334155; color: #e2e8f0; padding: 5px; border-radius: 4px; font-size: 12px; font-family: inherit; width: 100%; box-sizing: border-box;">
                </label>
                <label style="display: flex; flex-direction: column; gap: 2px;">
                    <span style="font-size: 10px; color: #94a3b8; text-transform: uppercase; font-weight: 600;">Curriculum</span>
                    <select id="marl-curriculum" style="background: #1e293b; border: 1px solid #334155; color: #e2e8f0; padding: 5px; border-radius: 4px; font-size: 12px; font-family: inherit; width: 100%; box-sizing: border-box;">
                        <option value="0">0 — Novice</option>
                        <option value="1" selected>1 — Apprenti</option>
                        <option value="2">2 — Intermédiaire</option>
                        <option value="3">3 — Avancé</option>
                        <option value="4">4 — Expert</option>
                    </select>
                </label>
            </div>
            <!-- Session persistence info -->
            <div id="marl-session-info" style="display: flex; gap: 8px; margin-bottom: 6px; font-size: 11px; color: #64748b; align-items: center; flex-wrap: wrap;">
                <span id="marl-session-count">Session: #1</span>
                <span id="marl-best-coverage">Best: —</span>
                <span id="marl-warmstart-badge" style="background: #1e293b; padding: 2px 6px; border-radius: 4px; color: #94a3b8; font-size: 10px;">Cold start</span>
                <button id="marl-btn-clear" style="margin-left: auto; padding: 3px 8px; background: transparent; color: #ef4444; border: 1px solid #ef4444; border-radius: 4px; cursor: pointer; font-size: 10px; font-family: inherit;">🗑 Reset</button>
            </div>
            <div style="display: flex; gap: 6px;">
                <button id="marl-btn-start" style="flex: 1; padding: 7px; background: #16a34a; color: white; border: none; border-radius: 6px; cursor: pointer; font-weight: 700; font-size: 12px; font-family: inherit;">▶ Lancer</button>
                <button id="marl-btn-pause" style="flex: 1; padding: 7px; background: #d97706; color: white; border: none; border-radius: 6px; cursor: pointer; font-weight: 700; font-size: 12px; font-family: inherit;" disabled>⏸ Pause</button>
                <button id="marl-btn-stop" style="flex: 1; padding: 7px; background: #dc2626; color: white; border: none; border-radius: 6px; cursor: pointer; font-weight: 700; font-size: 12px; font-family: inherit;" disabled>⏹ Stop</button>
            </div>
        </div>

        <!-- Progress -->
        <div id="marl-progress-section" style="padding: 6px 14px; border-bottom: 1px solid #1e293b; display: none; flex-shrink: 0;">
            <div style="display: flex; justify-content: space-between; margin-bottom: 4px; flex-wrap: wrap; gap: 4px;">
                <span id="marl-round-label" style="font-size: 11px; color: #94a3b8; font-weight: 600;">Round 0 / —</span>
                <span id="marl-coverage-label" style="font-size: 11px; color: #34d399; font-weight: 600;">Coverage: 0%</span>
                <span id="marl-reward-label" style="font-size: 11px; color: #f59e0b; font-weight: 600;">Reward: 0.0</span>
            </div>
            <div style="background: #1e293b; height: 8px; border-radius: 4px; overflow: hidden;">
                <div id="marl-progress-bar" style="height: 100%; width: 0%; background: linear-gradient(90deg, #7c3aed, #a78bfa); transition: width 0.15s;"></div>
            </div>
        </div>

        <!-- Agent Cards -->
        <div id="marl-agents" style="padding: 6px 14px; border-bottom: 1px solid #1e293b; flex-shrink: 0;">
            <div style="font-size: 10px; color: #64748b; text-transform: uppercase; margin-bottom: 4px; letter-spacing: 1px; font-weight: 600;">Agents Cognitifs</div>
            <div id="marl-agent-cards" style="display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 6px;">
            </div>
        </div>

        <!-- Charts -->
        <div style="padding: 8px 14px; flex: 1; display: flex; flex-direction: column; min-height: 0; overflow: hidden;">
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; flex-wrap: wrap; gap: 4px;">
                <span style="font-size: 10px; color: #64748b; text-transform: uppercase; letter-spacing: 1px; font-weight: 600;">Courbes d'Apprentissage</span>
                <div style="display: flex; gap: 4px;">
                    <button id="marl-chart-reward" class="marl-chart-btn" style="padding: 4px 10px; background: #334155; color: #e2e8f0; border: 1px solid #475569; border-radius: 4px; cursor: pointer; font-size: 11px; font-family: inherit;">Reward</button>
                    <button id="marl-chart-coverage" class="marl-chart-btn" style="padding: 4px 10px; background: #7c3aed; color: white; border: 1px solid #7c3aed; border-radius: 4px; cursor: pointer; font-size: 11px; font-family: inherit;">Coverage</button>
                </div>
            </div>
            <canvas id="marl-chart-canvas" width="1400" height="450" style="width: 100%; flex: 1; min-height: 120px; border-radius: 4px; background: #0f172a;"></canvas>
            <div id="marl-chart-legend" style="display: flex; gap: 10px; justify-content: center; margin-top: 4px; flex-wrap: wrap; font-size: 10px;"></div>
        </div>

        <!-- Transfer Dashboard -->
        <div id="marl-transfer-dashboard" style="display: none; padding: 0 20px 14px 20px; border-top: 2px solid #7c3aed;">
            <!-- Header -->
            <div style="display: flex; justify-content: space-between; align-items: center; padding: 12px 0 8px 0; flex-wrap: wrap; gap: 4px;">
                <span style="font-size: 15px; font-weight: 700; color: #a78bfa; letter-spacing: 0.5px;">🔄 TRANSFERT → AGENTS RÉELS</span>
                <span id="marl-transfer-timestamp" style="font-size: 11px; color: #64748b;"></span>
            </div>

            <!-- Pipeline steps -->
            <div style="font-size: 11px; color: #64748b; text-transform: uppercase; margin-bottom: 6px; font-weight: 600; letter-spacing: 1px;">Pipeline</div>
            <div id="marl-transfer-pipeline" style="display: grid; grid-template-columns: 1fr; gap: 4px; margin-bottom: 12px;"></div>

            <!-- Per-agent transfer cards -->
            <div style="font-size: 11px; color: #64748b; text-transform: uppercase; margin-bottom: 8px; font-weight: 600; letter-spacing: 1px;">Détails par Agent</div>
            <div id="marl-transfer-agents" style="display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 8px; margin-bottom: 12px;"></div>

            <!-- Before/After Comparison -->
            <div id="marl-transfer-comparison" style="display: none; margin-bottom: 12px;">
                <div style="font-size: 11px; color: #64748b; text-transform: uppercase; margin-bottom: 8px; font-weight: 600; letter-spacing: 1px;">Avant / Après Transfert</div>
                <div id="marl-transfer-ba-grid" style="display: grid; grid-template-columns: 1fr 40px 1fr 1fr; gap: 0; align-items: stretch;"></div>
            </div>

            <!-- Emergence monitoring progress -->
            <div id="marl-transfer-emergence" style="display: none; margin-bottom: 12px;">
                <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; flex-wrap: wrap; gap: 4px;">
                    <span style="font-size: 12px; color: #94a3b8;">📊 Monitoring Émergence Post-Transfert</span>
                    <span id="marl-emergence-timer" style="font-size: 12px; color: #64748b;">60s</span>
                </div>
                <div style="background: #1e293b; height: 6px; border-radius: 3px; overflow: hidden;">
                    <div id="marl-emergence-bar" style="height: 100%; width: 0%; background: linear-gradient(90deg, #a78bfa, #34d399); transition: width 0.3s;"></div>
                </div>
                <div id="marl-emergence-live" style="font-size: 12px; color: #94a3b8; margin-top: 6px; text-align: center;"></div>
            </div>

            <!-- Verdict -->
            <div id="marl-transfer-verdict" style="display: none; padding: 10px 14px; border-radius: 8px; text-align: center; font-weight: 700; font-size: 14px; margin-bottom: 12px;"></div>

            <!-- Session History -->
            <div id="marl-transfer-history-section" style="display: none;">
                <div style="font-size: 11px; color: #64748b; text-transform: uppercase; margin-bottom: 8px; font-weight: 600; letter-spacing: 1px;">📈 Historique des Transferts</div>
                <div id="marl-transfer-history-items" style="display: flex; gap: 8px; flex-wrap: wrap;"></div>
            </div>
        </div>

        <!-- Status bar -->
        <div id="marl-status" style="padding: 6px 14px; font-size: 11px; color: #64748b; border-top: 1px solid #1e293b; text-align: center; flex-shrink: 0;">
            Prêt — configurer les paramètres et lancer l'entraînement
        </div>
        `;

        document.body.appendChild(this._root);

        // Wrap all children in a scale-wrapper for zoom-based responsive scaling
        const _sw = document.createElement('div');
        _sw.id = 'marl-scale-wrapper';
        _sw.style.cssText = 'display:flex;flex-direction:column;overflow-y:auto;overflow-x:hidden;transform-origin:0 0;';
        while (this._root.firstChild) _sw.appendChild(this._root.firstChild);
        this._root.appendChild(_sw);

        // Cache DOM refs
        this._els = {
            rounds: this._root.querySelector('#marl-rounds'),
            steps: this._root.querySelector('#marl-steps'),
            strategy: this._root.querySelector('#marl-strategy'),
            lr: this._root.querySelector('#marl-lr'),
            curriculum: this._root.querySelector('#marl-curriculum'),
            sessionCount: this._root.querySelector('#marl-session-count'),
            bestCoverage: this._root.querySelector('#marl-best-coverage'),
            warmStartBadge: this._root.querySelector('#marl-warmstart-badge'),
            btnClear: this._root.querySelector('#marl-btn-clear'),
            btnStart: this._root.querySelector('#marl-btn-start'),
            btnPause: this._root.querySelector('#marl-btn-pause'),
            btnStop: this._root.querySelector('#marl-btn-stop'),
            progressSection: this._root.querySelector('#marl-progress-section'),
            roundLabel: this._root.querySelector('#marl-round-label'),
            coverageLabel: this._root.querySelector('#marl-coverage-label'),
            rewardLabel: this._root.querySelector('#marl-reward-label'),
            progressBar: this._root.querySelector('#marl-progress-bar'),
            agentCards: this._root.querySelector('#marl-agent-cards'),
            chartCanvas: this._root.querySelector('#marl-chart-canvas'),
            legend: this._root.querySelector('#marl-chart-legend'),
            status: this._root.querySelector('#marl-status'),
            // Transfer dashboard refs
            transferDashboard: this._root.querySelector('#marl-transfer-dashboard'),
            transferTimestamp: this._root.querySelector('#marl-transfer-timestamp'),
            transferPipeline: this._root.querySelector('#marl-transfer-pipeline'),
            transferAgents: this._root.querySelector('#marl-transfer-agents'),
            transferComparison: this._root.querySelector('#marl-transfer-comparison'),
            transferBAGrid: this._root.querySelector('#marl-transfer-ba-grid'),
            transferEmergence: this._root.querySelector('#marl-transfer-emergence'),
            emergenceTimer: this._root.querySelector('#marl-emergence-timer'),
            emergenceBar: this._root.querySelector('#marl-emergence-bar'),
            emergenceLive: this._root.querySelector('#marl-emergence-live'),
            transferVerdict: this._root.querySelector('#marl-transfer-verdict'),
            transferHistorySection: this._root.querySelector('#marl-transfer-history-section'),
            transferHistoryItems: this._root.querySelector('#marl-transfer-history-items'),
        };

        this._chartCtx = this._els.chartCanvas.getContext('2d');
        this._chartMode = 'coverage'; // 'coverage' or 'reward'

        // Inject CSS animations for transfer dashboard
        this._injectTransferCSS();

        // Transfer data (populated during transfer)
        this._lastTransferData = null;

        // Build agent cards
        this._buildAgentCards();

        // Build legend
        this._buildLegend();

        // Update session info display
        this._updateSessionInfo();

        // ── Drag-to-move via header ──────────────────────────────────
        this._initDrag();
        

        // ── ResizeObserver for responsive scaling ────────────────────
        this._initResponsiveScale();

        // Wire buttons
        this._els.btnStart.addEventListener('click', () => this._onStart());
        this._els.btnPause.addEventListener('click', () => this._onPause());
        this._els.btnStop.addEventListener('click', () => this._onStop());
        this._els.btnClear.addEventListener('click', () => {
            this._store.clear();
            this._updateSessionInfo();
        });

        // Chart mode buttons
        this._root.querySelector('#marl-chart-reward').addEventListener('click', () => {
            this._chartMode = 'reward';
            this._updateChartButtons();
            this._drawChart();
        });
        this._root.querySelector('#marl-chart-coverage').addEventListener('click', () => {
            this._chartMode = 'coverage';
            this._updateChartButtons();
            this._drawChart();
        });
    }

    _buildAgentCards() {
        this._els.agentCards.innerHTML = '';
        const droneMap = MARLTrainingPanel.DRONE_ID_MAP;
        const numAgents = droneMap.length;
        for (let i = 0; i < numAgents; i++) {
            const card = document.createElement('div');
            card.id = `marl-agent-${i}`;
            Object.assign(card.style, {
                background: '#0f172a',
                border: `1px solid ${AGENT_COLORS[i % AGENT_COLORS.length]}40`,
                borderRadius: '8px',
                padding: '10px 14px',
            });
            // Determine role from real agent data if available
            const droneId = droneMap[i];
            const agent = this.coordinator?.agents?.get(droneId);
            const role = agent?._agentRole || (droneId.startsWith('x500') ? 'Coordinator' : droneId.startsWith('s500') ? 'Patrol' : 'Scout');
            const displayName = AGENT_NAMES[i] || droneId.toUpperCase();
            const color = AGENT_COLORS[i % AGENT_COLORS.length];
            card.innerHTML = `
                <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px;">
                    <span style="color: ${color}; font-weight: 700; font-size: 15px;">${displayName}</span>
                    <span style="font-size: 11px; color: #64748b; background: #1e293b; padding: 2px 8px; border-radius: 4px;">${role}</span>
                </div>
                <div style="display: flex; justify-content: space-between; font-size: 13px;">
                    <span>Reward: <span class="agent-reward" style="color: #f59e0b;">—</span></span>
                    <span>Loss: <span class="agent-loss" style="color: #ef4444;">—</span></span>
                </div>
                <div style="font-size: 11px; margin-top: 4px; color: #475569;">
                    <span class="agent-health">🧠 Replay: — | Learn: — | ε: —</span>
                </div>
                <div style="margin-top: 6px; background: #1e293b; height: 6px; border-radius: 3px; overflow: hidden;">
                    <div class="agent-bar" style="height: 100%; width: 0%; background: ${color}; transition: width 0.15s;"></div>
                </div>
            `;
            this._els.agentCards.appendChild(card);
        }
    }

    _buildLegend() {
        this._els.legend.innerHTML = '';
        const droneMap = MARLTrainingPanel.DRONE_ID_MAP;
        const numAgents = droneMap.length;
        for (let i = 0; i < numAgents; i++) {
            const color = AGENT_COLORS[i % AGENT_COLORS.length];
            const name = AGENT_NAMES[i] || droneMap[i].toUpperCase();
            const item = document.createElement('span');
            item.style.cssText = `font-size: 13px; color: ${color}; display: flex; align-items: center; gap: 6px;`;
            item.innerHTML = `<span style="display: inline-block; width: 16px; height: 4px; background: ${color}; border-radius: 2px;"></span>${name}`;
            this._els.legend.appendChild(item);
        }
        // Global average
        const avg = document.createElement('span');
        avg.style.cssText = 'font-size: 13px; color: #ffffff; display: flex; align-items: center; gap: 6px;';
        avg.innerHTML = '<span style="display: inline-block; width: 16px; height: 4px; background: #ffffff; border-radius: 2px;"></span>Moyenne';
        this._els.legend.appendChild(avg);
    }

    _updateChartButtons() {
        this._root.querySelectorAll('.marl-chart-btn').forEach(btn => {
            btn.style.background = '#334155';
            btn.style.border = '1px solid #475569';
            btn.style.color = '#e2e8f0';
        });
        const sel = this._chartMode === 'reward' ? '#marl-chart-reward' : '#marl-chart-coverage';
        const btn = this._root.querySelector(sel);
        btn.style.background = '#7c3aed';
        btn.style.border = '1px solid #7c3aed';
        btn.style.color = 'white';
    }

    // =========================================================================
    // Training Controls
    // =========================================================================

    _onStart() {
        // Check DIAMANTS system is available
        const D = window.DIAMANTS;
        const coord = D?.controller?.multiAgentCoordinator;
        if (!coord || !coord.agents || coord.agents.size === 0) {
            this._els.status.textContent = '⚠️ Lancez la mission — coordinateur non disponible';
            log('⚠️ No coordinator available — start the mission first');
            return;
        }

        // Store coordinator reference for health indicators
        this.coordinator = coord;

        // Read config
        this._config.rounds = parseInt(this._els.rounds.value) || 30;
        this._config.roundIntervalSec = parseInt(this._els.steps.value) || 10;
        this._config.strategy = this._els.strategy.value;
        this._config.mixingRate = parseFloat(this._els.lr.value) || 0.5;

        // Disable config inputs
        this._setConfigDisabled(true);
        this._els.btnStart.disabled = true;
        this._els.btnPause.disabled = false;
        this._els.btnStop.disabled = false;
        this._els.progressSection.style.display = 'block';

        // Snapshot BEFORE metrics for comparison
        this._beforeMetrics = this._snapshotSwarmMetrics(D);

        // Create LIVE trainer — no simulation, real Q-table federation
        const droneMap = MARLTrainingPanel.DRONE_ID_MAP;
        this._trainer = new LiveFederatedTrainer({
            numAgents: droneMap.length,
            numRounds: this._config.rounds,
            roundIntervalSec: this._config.roundIntervalSec,
            strategy: this._config.strategy,
            mixingRate: this._config.mixingRate,
            coordinator: coord,
        });

        this._els.status.textContent = `🔴 LIVE — ${this._config.strategy.toUpperCase()} — ` +
            `${this._config.rounds} rounds × ${this._config.roundIntervalSec}s — ` +
            `${droneMap.length} agents cognitifs — Fédération Q-tables en temps réel`;

        // Run (non-blocking: wires event listeners + setInterval timer)
        this._trainer.run(
            (tick) => this._onTick(tick),
            (history) => this._onComplete(history),
        );
    }

    _onPause() {
        if (!this._trainer) return;
        if (this._trainer.paused) {
            this._trainer.resume();
            this._els.btnPause.textContent = '⏸ Pause';
            this._els.status.textContent = `Reprise — Round ${this._trainer.currentRound} / ${this._config.rounds}`;
        } else {
            this._trainer.pause();
            this._els.btnPause.textContent = '▶ Reprendre';
            this._els.status.textContent = `En pause — Round ${this._trainer.currentRound} / ${this._config.rounds}`;
        }
    }

    _onStop() {
        if (this._trainer) this._trainer.stop();
        this._trainer = null;
        this._setConfigDisabled(false);
        this._els.btnStart.disabled = false;
        this._els.btnPause.disabled = true;
        this._els.btnStop.disabled = true;
        this._els.btnPause.textContent = '⏸ Pause';
        this._els.status.textContent = 'Arrêté';
    }

    _onComplete(history) {
        this._setConfigDisabled(false);
        this._els.btnStart.disabled = false;
        this._els.btnPause.disabled = true;
        this._els.btnStop.disabled = true;
        this._els.btnPause.textContent = '⏸ Pause';

        const finalCov = history.coverage[history.coverage.length - 1] || 0;
        const fedCount = this._trainer?.fedHistory?.length || 0;
        const applied = this._trainer?.fedHistory?.filter(f => f.applied).length || 0;
        this._els.status.textContent = `✅ Terminé — ${history.rounds.length} rounds — ` +
            `Coverage réelle: ${(finalCov * 100).toFixed(1)}% — ` +
            `${applied}/${fedCount} fédérations Q-tables appliquées`;

        // Show federation results in transfer dashboard
        this._showFederationResults(history);
    }

    _onTick(tick) {
        // Progress
        const pct = ((tick.round + 1) / tick.totalRounds * 100).toFixed(1);
        this._els.progressBar.style.width = pct + '%';
        this._els.roundLabel.textContent = `Round ${tick.round + 1} / ${tick.totalRounds}`;
        this._els.coverageLabel.textContent = `Coverage: ${(tick.coverage * 100).toFixed(1)}%`;
        this._els.rewardLabel.textContent = `Reward: ${tick.avgReward.toFixed(1)}`;

        // Agent cards
        const droneMap = MARLTrainingPanel.DRONE_ID_MAP;
        const numAgents = droneMap.length;
        for (let i = 0; i < numAgents; i++) {
            const card = this._root.querySelector(`#marl-agent-${i}`);
            if (!card) continue;
            const reward = tick.perAgentReward[i] ?? 0;
            card.querySelector('.agent-reward').textContent = reward.toFixed(1);

            // Real TD-error loss from agent brain
            const loss = tick.perAgentLoss?.[i] ?? 0;
            card.querySelector('.agent-loss').textContent = loss.toFixed(2);

            // Health indicators: replay size, learn count, epsilon
            const agent = this.coordinator?.agents?.get(droneMap[i]);
            const stats = agent?.brain?.getStats?.();
            const healthEl = card.querySelector('.agent-health');
            if (healthEl && stats) {
                const replay = stats.replaySize ?? 0;
                const learn = stats.learnCount ?? 0;
                const eps = stats.epsilon ?? agent?.brain?.epsilon ?? 0;
                const feeding = replay > 0 ? '🟢' : '🔴';
                healthEl.textContent = `${feeding} Replay: ${replay} | Learn: ${learn} | ε: ${eps.toFixed(3)}`;
            }

            // Bar = relative reward (normalized to best agent this round)
            const maxR = Math.max(...tick.perAgentReward.map(Math.abs), 1);
            const barPct = Math.max(0, Math.min(100, (Math.abs(reward) / maxR) * 100));
            card.querySelector('.agent-bar').style.width = barPct + '%';
        }

        // Chart
        this._drawChart();
    }

    // =========================================================================
    // Chart Rendering (canvas)
    // =========================================================================

    _drawChart() {
        const canvas = this._els.chartCanvas;
        const ctx = this._chartCtx;
        if (!this._trainer || !this._trainer.history) return;

        const hist = this._trainer.history;
        if (hist.rounds.length < 2) return;

        const W = canvas.width;
        const H = canvas.height;
        const pad = { t: 30, r: 20, b: 36, l: 65 };
        const plotW = W - pad.l - pad.r;
        const plotH = H - pad.t - pad.b;

        ctx.clearRect(0, 0, W, H);

        // Background
        ctx.fillStyle = '#0f172a';
        ctx.fillRect(0, 0, W, H);

        const n = hist.rounds.length;
        const isCoverage = this._chartMode === 'coverage';
        const data = isCoverage ? hist.coverage : hist.rewards;
        const perAgent = isCoverage ? hist.agentCoverage : hist.agentRewards;

        // Y range
        let yMin = Infinity, yMax = -Infinity;
        for (const v of data) { if (v < yMin) yMin = v; if (v > yMax) yMax = v; }
        for (const arr of perAgent) {
            for (const v of arr) { if (v < yMin) yMin = v; if (v > yMax) yMax = v; }
        }
        if (yMin === yMax) { yMin -= 1; yMax += 1; }
        const yPad = (yMax - yMin) * 0.1;
        yMin -= yPad; yMax += yPad;

        const xScale = (i) => pad.l + (i / (n - 1)) * plotW;
        const yScale = (v) => pad.t + plotH - ((v - yMin) / (yMax - yMin)) * plotH;

        // Grid lines
        ctx.strokeStyle = '#1e293b';
        ctx.lineWidth = 1;
        const numGridY = 5;
        for (let i = 0; i <= numGridY; i++) {
            const y = pad.t + (i / numGridY) * plotH;
            ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke();
            // Label
            const val = yMax - (i / numGridY) * (yMax - yMin);
            ctx.fillStyle = '#64748b';
            ctx.font = '16px monospace';
            ctx.textAlign = 'right';
            ctx.fillText(isCoverage ? (val * 100).toFixed(0) + '%' : val.toFixed(0), pad.l - 8, y + 5);
        }

        // X axis labels
        ctx.fillStyle = '#64748b';
        ctx.textAlign = 'center';
        ctx.font = '16px monospace';
        const xLabels = 6;
        for (let i = 0; i < xLabels; i++) {
            const idx = Math.round((i / (xLabels - 1)) * (n - 1));
            ctx.fillText(`R${hist.rounds[idx]}`, xScale(idx), H - 6);
        }

        // Per-agent curves
        for (let a = 0; a < 4; a++) {
            ctx.strokeStyle = AGENT_COLORS[a];
            ctx.lineWidth = 2;
            ctx.globalAlpha = 0.6;
            ctx.beginPath();
            for (let i = 0; i < n; i++) {
                const v = perAgent[i][a];
                const x = xScale(i), y = yScale(v);
                i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
            }
            ctx.stroke();
            ctx.globalAlpha = 1;
        }

        // Global average curve (thick white)
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3;
        ctx.beginPath();
        for (let i = 0; i < n; i++) {
            const v = data[i];
            const x = xScale(i), y = yScale(v);
            i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        }
        ctx.stroke();

        // Title
        ctx.fillStyle = '#94a3b8';
        ctx.font = 'bold 18px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(isCoverage ? 'COVERAGE PAR ROUND' : 'REWARD CUMULÉ PAR ROUND', pad.l, 22);
    }

    // =========================================================================
    // MARL → Real Agents Transfer (Policy Distillation)
    // =========================================================================

    /**
     * MARL agent index → real drone ID mapping
     * Dynamically reads actual IDs from the coordinator by matching agent number suffix.
     * Matches AGENT_NAMES: [X500_09, X500_10, X500_11, S500_08]
     */
    static get DRONE_ID_MAP() {
        const coord = window.DIAMANTS?.controller?.multiAgentCoordinator;
        if (coord?.agents?.size > 0) {
            // Only return cognitive agents (those with a trainable brain)
            const cognitive = [];
            for (const [id, agent] of coord.agents) {
                if (agent.brain && agent.brain.brainType !== 'reactive') {
                    cognitive.push(id);
                }
            }
            if (cognitive.length > 0) return cognitive;
        }
        // Fallback: the 3 known cognitive drones
        return ['x500_09', 's500_10', 'x500_11'];
    }

    /**
     * Map MARL 16-dim observation (Gymnasium) → real 24-dim observation.
     * The 16-dim obs provides much richer data than the old 7-dim,
     * enabling a more accurate mapping to the real agent's observation space.
     *
     * GymObsIndex (16-dim):
     *   0: POS_X, 1: POS_Z, 2: VEL_X, 3: VEL_Z, 4: HEADING,
     *   5: OBSTACLE_FRONT, 6: OBSTACLE_LEFT, 7: OBSTACLE_RIGHT,
     *   8: FRONTIER_DIST, 9: LOCAL_COVERAGE, 10: GLOBAL_COVERAGE,
     *   11: NEIGHBOR_COUNT, 12: NEAREST_DIST, 13: NEAREST_DIR_X,
     *   14: NEAREST_DIR_Z, 15: TIME_REMAINING
     *
     * Real ObsIndex (24-dim):
     *   0: POS_X_NORM, 1: POS_Z_NORM, 2: HEADING_SIN, 3: HEADING_COS,
     *   4: SPEED_NORM, 5: BATTERY_NORM, 6: ALTITUDE_NORM, 7: TIME_IN_PHASE,
     *   8-11: OBSTACLE_FRONT/LEFT/RIGHT/BACK, 12-13: FRONTIER_DIR,
     *   14: FRONTIER_DIST, 15: LOCAL_COVERAGE, 16: NEIGHBOR_COUNT,
     *   17: NEAREST_DIST, 18-19: NEAREST_DIR, 20: SWARM_COVERAGE,
     *   21: OVERLAP_RATIO, 22: CONSENSUS_ROLE, 23: COMM_QUALITY
     */
    _mapObsToReal(marlObs) {
        const real = new Float32Array(24);
        // POS_X_NORM (0): from [-1,1] → [0,1]
        real[0] = (marlObs[GymObsIndex.POS_X] + 1) / 2;
        // POS_Z_NORM (1): from [-1,1] → [0,1]
        real[1] = (marlObs[GymObsIndex.POS_Z] + 1) / 2;
        // HEADING_SIN (2): from heading/π → sin
        const heading = marlObs[GymObsIndex.HEADING] * Math.PI;
        real[2] = Math.sin(heading);
        // HEADING_COS (3)
        real[3] = Math.cos(heading);
        // SPEED_NORM (4): from velocity components
        const speed = Math.sqrt(marlObs[GymObsIndex.VEL_X] ** 2 + marlObs[GymObsIndex.VEL_Z] ** 2);
        real[4] = Math.min(1, speed / 1.5);
        // BATTERY_NORM (5): assume nominal
        real[5] = 0.9;
        // ALTITUDE_NORM (6): assume cruise altitude
        real[6] = 0.5;
        // TIME_IN_PHASE (7): from TIME_REMAINING
        real[7] = 1 - marlObs[GymObsIndex.TIME_REMAINING];
        // OBSTACLE_FRONT (8): directly from ray-cast sensing
        real[8] = marlObs[GymObsIndex.OBSTACLE_FRONT];
        // OBSTACLE_LEFT (9)
        real[9] = marlObs[GymObsIndex.OBSTACLE_LEFT];
        // OBSTACLE_RIGHT (10)
        real[10] = marlObs[GymObsIndex.OBSTACLE_RIGHT];
        // OBSTACLE_BACK (11): not sensed in MARL, assume clear
        real[11] = 0.8;
        // FRONTIER_DIR (12-13): from heading direction (frontier is ahead)
        real[12] = Math.sin(heading);
        real[13] = Math.cos(heading);
        // FRONTIER_DIST (14): directly from Gymnasium env
        real[14] = marlObs[GymObsIndex.FRONTIER_DIST];
        // LOCAL_COVERAGE (15): directly mapped
        real[15] = marlObs[GymObsIndex.LOCAL_COVERAGE];
        // NEIGHBOR_COUNT (16): directly mapped
        real[16] = marlObs[GymObsIndex.NEIGHBOR_COUNT];
        // NEAREST_DIST (17): directly mapped
        real[17] = marlObs[GymObsIndex.NEAREST_DIST];
        // NEAREST_DIR (18-19): directly mapped
        real[18] = marlObs[GymObsIndex.NEAREST_DIR_X];
        real[19] = marlObs[GymObsIndex.NEAREST_DIR_Z];
        // SWARM_COVERAGE (20): from GLOBAL_COVERAGE
        real[20] = marlObs[GymObsIndex.GLOBAL_COVERAGE];
        // OVERLAP_RATIO (21): estimate from local vs global coverage
        real[21] = Math.max(0, marlObs[GymObsIndex.LOCAL_COVERAGE] - marlObs[GymObsIndex.GLOBAL_COVERAGE]) * 2;
        // CONSENSUS_ROLE (22): coordinator role
        real[22] = 0.75;
        // COMM_QUALITY (23): assume good
        real[23] = 0.8;
        return real;
    }

    /**
     * Map MARL continuous action {dx, dz, spread, signal} → discrete AgentAction (0-9).
     */
    _mapActionToDiscrete(envAction) {
        const { dx, dz, spread, signal } = envAction;
        const mag = Math.sqrt(dx * dx + dz * dz);

        // High communication → SHARE_MAP (8)
        if (signal > 0.7 && mag < 3) return 8;
        // Barely moving → HOVER (3)
        if (mag < 2) return 3;
        // High spread → GOTO_FRONTIER (5)
        if (spread > 10) return 5;
        // Low spread → GOTO_NEIGHBOR (6)
        if (spread < 4) return 6;

        // Directional movement
        const angle = Math.atan2(dx, -dz); // forward = -z
        if (angle > -Math.PI / 4 && angle < Math.PI / 4) return 0;  // EXPLORE_FORWARD
        if (angle >= Math.PI / 4 && angle < 3 * Math.PI / 4) return 2; // EXPLORE_RIGHT
        if (angle <= -Math.PI / 4 && angle > -3 * Math.PI / 4) return 1; // EXPLORE_LEFT
        return 4; // RETREAT (backward)
    }

    /**
     * After MARL training, distill learned policy into real agents:
     *   1. Run evaluation episodes with trained policies
     *   2. Map observations + actions to real format
     *   3. Inject synthetic experiences into real agents' replay buffers
     *   4. Force learning steps
     *   5. Assign exploration zones
     *   6. Adjust cooperative weights + epsilon
     *   7. Snapshot before/after metrics for emergence comparison
     */
    /**
     * Synchronous transfer entry point (used by tests and legacy code).
     * Delegates to the async version but catches errors.
     */
    _transferToRealAgents(history) {
        this._transferToRealAgentsAsync(history).catch(e => {
            log(`⚠️ Transfert échoué: ${e.message}`);
            this._els.status.textContent = `⚠️ Transfert échoué: ${e.message}`;
        });
    }

    /**
     * Async transfer — yields to the UI thread between agents so the
     * browser stays responsive. Creates only ONE eval env (reused via reset).
     */
    async _transferToRealAgentsAsync(history) {
        const D = window.DIAMANTS;
        if (!D || !D.controller) {
            log('⚠️ Transfert impossible — système DIAMANTS non disponible');
            this._els.status.textContent += ' ⚠️ Transfert échoué (système non initialisé)';
            return;
        }

        const coord = D.controller.multiAgentCoordinator;
        if (!coord || !coord.agents) {
            log('⚠️ Transfert impossible — coordinateur multi-agent non disponible');
            this._els.status.textContent += ' ⚠️ Transfert échoué (coordinateur absent)';
            return;
        }

        // ── Transfer visualization data ──
        const transferData = {
            steps: [],
            agents: [],
            beforeMetrics: null,
            totalExperiences: 0,
            agentsUpdated: 0,
            strategy: '',
            timestamp: Date.now(),
        };

        // ── Step 0: Snapshot BEFORE metrics for A/B comparison ──
        const beforeMetrics = this._snapshotSwarmMetrics(D);
        transferData.beforeMetrics = beforeMetrics;
        transferData.steps.push({ label: 'Snapshot métriques avant', status: 'done', detail: `émergence=${((beforeMetrics.emergence || 0) * 100).toFixed(1)}%` });

        log('🔄 Début du transfert MARL → agents réels…');
        this._els.status.textContent = '🔄 Transfert de l\'apprentissage collectif vers les agents réels…';

        const trainer = this._trainer;
        if (!trainer) return;

        transferData.strategy = trainer.strategy;

        let totalExperiences = 0;
        let agentsUpdated = 0;

        // ── Step 1: Generate synthetic experiences by running evaluation ──
        // Reuse ONE env (reset between episodes) instead of creating 80
        const numEvalEpisodes = 5;
        const maxEvalSteps = 200;
        transferData.steps.push({ label: `Évaluation entraînée (${numEvalEpisodes} épisodes × ${maxEvalSteps} steps)`, status: 'running', detail: '' });

        // Show dashboard early with pipeline progress
        this._renderTransferDashboard(transferData);

        const evalEnv = new GymnasiumSwarmEnv({
            numAgents: 4,
            curriculumStage: trainer.curriculumStage ?? 1,
            maxSteps: maxEvalSteps,
        });

        for (let agentIdx = 0; agentIdx < 4; agentIdx++) {
            const droneId = MARLTrainingPanel.DRONE_ID_MAP[agentIdx];
            const realAgent = coord.agents.get(droneId);

            const agentData = {
                droneId,
                name: AGENT_NAMES[agentIdx],
                color: AGENT_COLORS[agentIdx],
                experiences: 0,
                learnSteps: 0,
                epsilonBefore: 0,
                epsilonAfter: 0,
                zoneAssigned: false,
                cooperativeWeight: 0,
                skipped: false,
                skipReason: '',
            };

            if (!realAgent || !realAgent.brain) {
                log(`⚠️ Agent ${droneId} non trouvé, skip`);
                agentData.skipped = true;
                agentData.skipReason = 'non trouvé';
                transferData.agents.push(agentData);
                continue;
            }

            // Only inject into trainable brains (cognitive / q-learning)
            if (realAgent.brain.brainType === 'reactive' || !realAgent.brain.replay) {
                log(`ℹ️ Agent ${droneId} a un cerveau réactif, skip`);
                agentData.skipped = true;
                agentData.skipReason = 'cerveau réactif';
                transferData.agents.push(agentData);
                continue;
            }

            agentData.epsilonBefore = realAgent.brain.epsilon;
            agentData.cooperativeWeight = realAgent.brain._cooperativeWeight ?? 0;

            const policy = trainer.policies[agentIdx];
            if (!policy) {
                agentData.skipped = true;
                agentData.skipReason = 'pas de politique';
                transferData.agents.push(agentData);
                continue;
            }

            let injectedCount = 0;

            // Run evaluation episodes — reuse the single evalEnv via reset()
            for (let ep = 0; ep < numEvalEpisodes; ep++) {
                const resetResult = evalEnv.reset();
                let currentObs = resetResult.obs;

                for (let step = 0; step < maxEvalSteps; step++) {
                    const obsI = new Float32Array(currentObs[agentIdx]);

                    // Get trained policy action (deterministic forward pass)
                    const rawAction = policy.forward(obsI);
                    const envAction = policy.toEnvAction(rawAction);

                    // Build actions for all agents (trained policies)
                    const allActions = [];
                    for (let j = 0; j < 4; j++) {
                        const obsJ = new Float32Array(currentObs[j]);
                        const rawJ = trainer.policies[j].forward(obsJ);
                        allActions.push(trainer.policies[j].toEnvAction(rawJ));
                    }

                    // Gymnasium step → {obs, rewards, terminated, truncated, info}
                    const result = evalEnv.step(allActions);
                    const nextObs = result.obs;
                    const done = result.terminated || result.truncated;

                    // Map 16-dim Gymnasium obs → 24-dim real obs
                    const realObs = this._mapObsToReal(currentObs[agentIdx]);
                    const realNextObs = this._mapObsToReal(nextObs[agentIdx]);
                    const discreteAction = this._mapActionToDiscrete(envAction);
                    const reward = result.rewards[agentIdx];

                    // Inject into real agent's replay buffer
                    realAgent.brain.replay.add(
                        realObs,
                        discreteAction,
                        reward,
                        realNextObs,
                        done
                    );
                    injectedCount++;

                    currentObs = nextObs;
                    if (done) break;
                }
            }

            agentData.experiences = injectedCount;

            // ── Step 2: Force learning on injected experiences ──
            const learnSteps = Math.min(50, Math.floor(injectedCount / realAgent.brain.batchSize));
            for (let ls = 0; ls < learnSteps; ls++) {
                realAgent.brain._learnMinibatch();
            }
            agentData.learnSteps = learnSteps;

            // ── Step 3: Adjust hyperparameters based on training results ──
            const finalCov = history.coverage[history.coverage.length - 1] || 0;

            // Lower epsilon proportionally to training convergence (more confident)
            const epsilonReduction = Math.min(0.15, finalCov * 0.2);
            const newEpsilon = Math.max(
                realAgent.brain.epsilonMin,
                realAgent.brain.epsilon - epsilonReduction
            );
            realAgent.brain.epsilon = newEpsilon;
            agentData.epsilonAfter = newEpsilon;

            // ── Step 4: Zone assignment from MARL learned positions ──
            if (realAgent.brain.claimZone) {
                const marlAgent = trainer.env.agents[agentIdx];
                if (marlAgent) {
                    const zoneHalf = 20;
                    realAgent.brain.claimZone({
                        xMin: marlAgent.x - zoneHalf,
                        zMin: marlAgent.z - zoneHalf,
                        xMax: marlAgent.x + zoneHalf,
                        zMax: marlAgent.z + zoneHalf,
                    });
                    agentData.zoneAssigned = true;
                }
            }

            // ── Step 5: Boost cooperative weight if federated strategy worked ──
            if (realAgent.brain._cooperativeWeight !== undefined) {
                const strategyBonus = trainer.strategy !== 'none' ? 0.1 : -0.05;
                realAgent.brain._cooperativeWeight = Math.min(1,
                    Math.max(0, realAgent.brain._cooperativeWeight + strategyBonus)
                );
                agentData.cooperativeWeight = realAgent.brain._cooperativeWeight;
            }

            totalExperiences += injectedCount;
            agentsUpdated++;

            transferData.agents.push(agentData);
            log(`✅ ${droneId}: ${injectedCount} exp injectées, ${learnSteps} steps apprentissage, ε=${newEpsilon.toFixed(3)}`);

            // ── Yield to the UI thread between agents ──
            await new Promise(r => setTimeout(r, 0));
        }

        // Finalize pipeline steps
        transferData.steps[1].status = 'done';
        transferData.steps[1].detail = `${totalExperiences} observations mappées`;
        transferData.steps.push({ label: `Mapping 16-dim → 24-dim`, status: 'done', detail: `${totalExperiences} observations converties` });
        transferData.steps.push({ label: `Injection replay buffers`, status: 'done', detail: `${totalExperiences} expériences injectées` });
        transferData.steps.push({ label: `Apprentissage forcé`, status: 'done', detail: `${transferData.agents.filter(a => !a.skipped).reduce((s, a) => s + a.learnSteps, 0)} mini-batches` });
        transferData.steps.push({ label: `Ajustement ε + zones`, status: 'done', detail: `${agentsUpdated} agents configurés` });

        transferData.totalExperiences = totalExperiences;
        transferData.agentsUpdated = agentsUpdated;
        this._lastTransferData = transferData;

        // ── Step 6: Emit transfer event ──
        const finalCov = history.coverage[history.coverage.length - 1] || 0;
        const transferSummary = {
            agentsUpdated,
            totalExperiences,
            strategy: trainer.strategy,
            finalCoverage: finalCov,
            rounds: history.rounds.length,
            timestamp: Date.now(),
            beforeMetrics,
        };

        window.dispatchEvent(new CustomEvent('diamants:marl-transfer-complete', {
            detail: transferSummary
        }));

        // Update status
        const statusMsg = agentsUpdated > 0
            ? `✅ Transfert réussi — ${agentsUpdated} agents mis à jour, ${totalExperiences} expériences injectées, stratégie ${trainer.strategy.toUpperCase()}`
            : `⚠️ Aucun agent cognitif trouvé pour le transfert`;
        this._els.status.textContent = statusMsg;

        log(`🎯 Transfert terminé: ${JSON.stringify(transferSummary)}`);

        // ── Re-render the Transfer Dashboard (final state) ──
        this._renderTransferDashboard(transferData);

        // ── Step 7: Schedule emergence monitoring (measure AFTER over 60s) ──
        if (agentsUpdated > 0) {
            this._scheduleEmergenceMonitor(D, beforeMetrics, transferSummary);
        }
    }

    /**
     * Show federation results dashboard after live MARL session completes.
     * Displays real metrics from actual mission data + Q-table federation.
     */
    _showFederationResults(history) {
        const D = window.DIAMANTS;
        const coord = D?.controller?.multiAgentCoordinator;
        const trainer = this._trainer;
        if (!trainer) return;

        const transferData = {
            steps: [],
            agents: [],
            beforeMetrics: this._beforeMetrics || null,
            totalExperiences: 0,
            agentsUpdated: 0,
            strategy: trainer.strategy,
            timestamp: Date.now(),
        };

        // Pipeline steps
        const fedCount = trainer.fedHistory?.length || 0;
        const applied = trainer.fedHistory?.filter(f => f.applied).length || 0;
        transferData.steps.push({ label: 'Écoute données mission (temps réel)', status: 'done', detail: `${history.rounds.length} rounds collectés` });
        transferData.steps.push({ label: `Fédération Q-tables (${trainer.strategy.toUpperCase()})`, status: 'done', detail: `${applied}/${fedCount} synchronisations` });
        transferData.steps.push({ label: `Mixage poids (taux: ${trainer.mixingRate})`, status: 'done', detail: `Q-tables moyennées` });
        transferData.steps.push({ label: 'Agents mis à jour en temps réel', status: 'done', detail: 'ε + poids synchronisés' });

        // Per-agent data from real coordinator
        if (coord?.agents) {
            for (let i = 0; i < MARLTrainingPanel.DRONE_ID_MAP.length; i++) {
                const droneId = MARLTrainingPanel.DRONE_ID_MAP[i];
                const agent = coord.agents.get(droneId);
                const agentData = {
                    droneId,
                    name: AGENT_NAMES[i],
                    color: AGENT_COLORS[i],
                    experiences: agent?.brain?.replay?.size || agent?.brain?.replay?.buffer?.length || 0,
                    learnSteps: agent?.brain?.learnCount || 0,
                    epsilonBefore: this._beforeMetrics?.agentEpsilons?.[droneId] || 0.3,
                    epsilonAfter: agent?.brain?.epsilon || 0,
                    zoneAssigned: false,
                    cooperativeWeight: agent?.brain?._cooperativeWeight || 0,
                    skipped: !agent?.brain?.weights,
                    skipReason: !agent ? 'non trouvé' : (!agent.brain?.weights ? 'cerveau réactif' : ''),
                };
                transferData.agents.push(agentData);
                if (!agentData.skipped) transferData.agentsUpdated++;
            }
        }

        transferData.totalExperiences = transferData.agents.reduce((s, a) => s + a.experiences, 0);
        this._lastTransferData = transferData;

        // Emit transfer complete event
        const transferSummary = {
            agentsUpdated: transferData.agentsUpdated,
            totalExperiences: transferData.totalExperiences,
            strategy: trainer.strategy,
            finalCoverage: history.coverage[history.coverage.length - 1] || 0,
            rounds: history.rounds.length,
            timestamp: Date.now(),
            beforeMetrics: this._beforeMetrics,
        };

        window.dispatchEvent(new CustomEvent('diamants:marl-transfer-complete', {
            detail: transferSummary
        }));

        // Render dashboard
        this._renderTransferDashboard(transferData);

        // Schedule emergence monitoring if agents were updated
        if (transferData.agentsUpdated > 0 && D) {
            this._scheduleEmergenceMonitor(D, this._beforeMetrics || {}, transferSummary);
        }

        log(`🎯 Fédération terminée: ${JSON.stringify(transferSummary)}`);
    }

    /**
     * Snapshot current swarm metrics (alignment, cohesion, coverage, etc.)
     * Used for before/after comparison.
     */
    _snapshotSwarmMetrics(D) {
        try {
            const formulas = D.controller?.diamantFormulas || D.system?.formulas || D.controller?.formulas;
            const sm = formulas?.swarmMetrics;
            const coordMetrics = D.controller?.multiAgentCoordinator?.getMetrics?.();

            return {
                timestamp: Date.now(),
                alignment: sm?.alignment || 0,
                separation: sm?.separation || 0,
                cohesion: sm?.cohesion || 0,
                emergence: sm?.emergence || 0,
                stigmergyCoverage: sm?.stigmergyCoverage || 0,
                globalCoverage: parseFloat(coordMetrics?.globalCoverage) || 0,
                agentCount: coordMetrics?.agentCount || 0,
                emergenceDetail: formulas?._emergenceDetail || null,
                agentEpsilons: this._getAgentEpsilons(D),
            };
        } catch (e) {
            log(`⚠️ Snapshot métriques échoué: ${e.message}`);
            return { timestamp: Date.now(), error: e.message };
        }
    }

    /**
     * Get epsilon values for all agents (exploration rate).
     */
    _getAgentEpsilons(D) {
        const epsilons = {};
        const coord = D.controller?.multiAgentCoordinator;
        if (!coord?.agents) return epsilons;
        for (const [id, agent] of coord.agents) {
            epsilons[id] = agent.brain?.epsilon ?? null;
        }
        return epsilons;
    }

    /**
     * Monitor emergence for 60 seconds after transfer.
     * Samples swarm metrics every 5s, then computes Delta (after vs before).
     */
    _scheduleEmergenceMonitor(D, beforeMetrics, transferSummary) {
        // Clear any previous emergence timer to prevent accumulation
        if (this._emergenceTimerId) {
            clearInterval(this._emergenceTimerId);
            this._emergenceTimerId = null;
        }

        log('📊 Monitoring émergence post-transfert (60 secondes)…');
        this._els.status.textContent += ' — 📊 Analyse émergence en cours…';

        // Show emergence monitor in dashboard
        this._els.transferEmergence.style.display = 'block';

        const samples = [];
        const sampleInterval = 5000; // 5s
        const totalDuration = 60000; // 60s
        let elapsed = 0;

        this._emergenceTimerId = setInterval(() => {
            elapsed += sampleInterval;
            const snapshot = this._snapshotSwarmMetrics(D);
            samples.push(snapshot);

            // Update status with live emergence score
            const currentEmergence = snapshot.emergence ?? 0;
            const delta = currentEmergence - (beforeMetrics.emergence ?? 0);
            const sign = delta >= 0 ? '+' : '';
            this._els.status.textContent =
                `📊 Émergence: ${(currentEmergence * 100).toFixed(1)}% (${sign}${(delta * 100).toFixed(1)}%) — ${Math.round((totalDuration - elapsed) / 1000)}s restantes`;

            // Update dashboard emergence monitor
            const pct = Math.min(100, (elapsed / totalDuration) * 100);
            this._els.emergenceBar.style.width = pct + '%';
            this._els.emergenceTimer.textContent = `${Math.round((totalDuration - elapsed) / 1000)}s`;
            const liveColor = delta >= 0 ? '#34d399' : '#f87171';
            this._els.emergenceLive.innerHTML = `Émergence: <span style="color:${liveColor}; font-weight:700;">${(currentEmergence * 100).toFixed(1)}%</span> (${sign}${(delta * 100).toFixed(1)}%)`;

            if (elapsed >= totalDuration) {
                clearInterval(this._emergenceTimerId);
                this._emergenceTimerId = null;
                this._finalizeEmergenceReport(beforeMetrics, samples, transferSummary);
            }
        }, sampleInterval);
    }

    /**
     * Compute final emergence report: before vs after comparison.
     * Emits 'diamants:emergence-report' event with full analysis.
     */
    _finalizeEmergenceReport(before, afterSamples, transferSummary) {
        // Average the after-samples for stability
        const avg = (arr, key) => {
            const vals = arr.map(s => s[key] || 0).filter(v => !isNaN(v));
            return vals.length > 0 ? vals.reduce((a, b) => a + b, 0) / vals.length : 0;
        };

        const after = {
            alignment: avg(afterSamples, 'alignment'),
            separation: avg(afterSamples, 'separation'),
            cohesion: avg(afterSamples, 'cohesion'),
            emergence: avg(afterSamples, 'emergence'),
            stigmergyCoverage: avg(afterSamples, 'stigmergyCoverage'),
            globalCoverage: avg(afterSamples, 'globalCoverage'),
        };

        // Delta for each metric
        const delta = {};
        for (const key of Object.keys(after)) {
            delta[key] = after[key] - (before[key] || 0);
        }

        // Composite emergence improvement score
        // Weighted: emergence(40%), coverage(30%), cohesion(15%), separation(15%)
        const compositeImprovement =
            delta.emergence * 0.40 +
            delta.globalCoverage / 100 * 0.30 + // globalCoverage is in %
            delta.cohesion * 0.15 +
            delta.separation * 0.15;

        const report = {
            before,
            after,
            delta,
            compositeImprovement,
            samples: afterSamples.length,
            verdict: compositeImprovement > 0.05 ? '✅ ÉMERGENCE DÉTECTÉE'
                : compositeImprovement > 0 ? '🔶 Amélioration marginale'
                : '⚠️ Pas d\'amélioration mesurable',
            transferSummary,
            timestamp: Date.now(),
        };

        // Log full report
        log('═══════════════════════════════════════════');
        log('📊 RAPPORT D\'ÉMERGENCE POST-TRANSFERT MARL');
        log('═══════════════════════════════════════════');
        log(`  Avant  → Émergence: ${((before.emergence ?? 0) * 100).toFixed(1)}%, Coverage: ${(before.globalCoverage ?? 0).toFixed(1)}%, Cohésion: ${((before.cohesion ?? 0) * 100).toFixed(1)}%`);
        log(`  Après  → Émergence: ${((after.emergence ?? 0) * 100).toFixed(1)}%, Coverage: ${(after.globalCoverage ?? 0).toFixed(1)}%, Cohésion: ${((after.cohesion ?? 0) * 100).toFixed(1)}%`);
        log(`  Delta  → Émergence: ${(delta.emergence ?? 0) > 0 ? '+' : ''}${((delta.emergence ?? 0) * 100).toFixed(1)}%, Coverage: ${(delta.globalCoverage ?? 0) > 0 ? '+' : ''}${(delta.globalCoverage ?? 0).toFixed(1)}%`);
        log(`  Score composite: ${(compositeImprovement * 100).toFixed(1)}%`);
        log(`  Verdict: ${report.verdict}`);
        log('═══════════════════════════════════════════');

        // Update status bar with final verdict
        this._els.status.textContent = `${report.verdict} — Δ émergence: ${(delta.emergence ?? 0) > 0 ? '+' : ''}${((delta.emergence ?? 0) * 100).toFixed(1)}% | Δ coverage: ${(delta.globalCoverage ?? 0) > 0 ? '+' : ''}${(delta.globalCoverage ?? 0).toFixed(1)}% | Score: ${(compositeImprovement * 100).toFixed(1)}%`;

        // ── Update Transfer Dashboard with before/after + verdict ──
        this._renderBeforeAfter(before, after, delta);
        this._renderVerdict(report.verdict, compositeImprovement);
        this._saveTransferHistory(report);
        this._renderTransferHistory();

        // Emit event for other UI components
        window.dispatchEvent(new CustomEvent('diamants:emergence-report', {
            detail: report
        }));

        // Store on window for console access
        window.__MARL_EMERGENCE_REPORT = report;
        log('💡 Rapport accessible via: window.__MARL_EMERGENCE_REPORT');
    }

    // =========================================================================
    // Transfer Dashboard Rendering
    // =========================================================================

    /**
     * Inject CSS keyframe animations for the transfer dashboard.
     * Guarded: only injects once per page lifetime.
     */
    _injectTransferCSS() {
        try {
            if (MARLTrainingPanel._cssInjected) return;
            MARLTrainingPanel._cssInjected = true;
            const style = document.createElement('style');
            style.id = 'marl-transfer-css';
            style.textContent = `
                @keyframes marlFadeSlideIn {
                    from { opacity: 0; transform: translateX(-12px); }
                    to   { opacity: 1; transform: translateX(0); }
                }
                @keyframes marlPulseGlow {
                    0%, 100% { box-shadow: 0 0 4px rgba(124,58,237,0.3); }
                    50%      { box-shadow: 0 0 16px rgba(124,58,237,0.7); }
                }
                @keyframes marlCountUp {
                    from { opacity: 0; transform: scale(0.8); }
                    to   { opacity: 1; transform: scale(1); }
                }
                .marl-step-row {
                    display: flex; align-items: center; gap: 10px;
                    padding: 6px 12px; border-radius: 6px;
                    background: #0f172a; font-size: 13px;
                    animation: marlFadeSlideIn 0.4s ease-out both;
                }
                .marl-step-icon {
                    width: 22px; height: 22px; border-radius: 50%;
                    display: flex; align-items: center; justify-content: center;
                    font-size: 12px; flex-shrink: 0;
                }
                .marl-step-done .marl-step-icon {
                    background: #16a34a; color: white;
                }
                .marl-step-running .marl-step-icon {
                    background: #d97706; color: white;
                    animation: marlPulseGlow 1.5s ease-in-out infinite;
                }
                .marl-step-pending .marl-step-icon {
                    background: #334155; color: #64748b;
                }
                .marl-step-label { flex: 1; color: #d4d4d8; }
                .marl-step-detail { color: #64748b; font-size: 12px; text-align: right; }
                .marl-agent-transfer-card {
                    background: #0f172a; border-radius: 8px; padding: 10px 12px;
                    animation: marlCountUp 0.5s ease-out both;
                }
                .marl-ba-card {
                    background: #0f172a; border-radius: 8px; padding: 12px 16px;
                    animation: marlFadeSlideIn 0.5s ease-out both;
                }
                .marl-ba-row {
                    display: flex; justify-content: space-between;
                    padding: 4px 0; font-size: 13px;
                }
                .marl-delta-pill {
                    display: inline-block; padding: 2px 8px; border-radius: 10px;
                    font-size: 12px; font-weight: 700;
                    animation: marlCountUp 0.6s ease-out both;
                }
                .marl-delta-positive { background: rgba(52,211,153,0.15); color: #34d399; }
                .marl-delta-negative { background: rgba(248,113,113,0.15); color: #f87171; }
                .marl-delta-neutral  { background: rgba(148,163,184,0.15); color: #94a3b8; }
            `;
            document.body.appendChild(style);
        } catch (e) {
            // Silently ignore in test (mock DOM)
        }
    }

    /**
     * Render the full Transfer Dashboard after _transferToRealAgents completes.
     */
    _renderTransferDashboard(data) {
        // Show the dashboard
        this._els.transferDashboard.style.display = 'block';

        // Timestamp
        const now = new Date(data.timestamp);
        this._els.transferTimestamp.textContent = now.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });

        // Pipeline steps
        this._renderPipelineSteps(data.steps);

        // Agent cards
        this._renderTransferAgentCards(data.agents);

        // Load and render history
        this._renderTransferHistory();

        // Reset emergence section
        this._els.transferEmergence.style.display = 'none';
        this._els.emergenceBar.style.width = '0%';
        this._els.emergenceLive.innerHTML = '';
        this._els.transferComparison.style.display = 'none';
        this._els.transferVerdict.style.display = 'none';

        // Auto-scroll to the dashboard
        try {
            this._els.transferDashboard.scrollIntoView?.({ behavior: 'smooth', block: 'start' });
        } catch (_) { /* ignore in test */ }

        log('📊 Transfer Dashboard rendu');
    }

    /**
     * Render pipeline steps with staggered animation.
     */
    _renderPipelineSteps(steps) {
        this._els.transferPipeline.innerHTML = '';
        steps.forEach((step, i) => {
            const statusClass = step.status === 'done' ? 'marl-step-done'
                : step.status === 'running' ? 'marl-step-running'
                : 'marl-step-pending';
            const icon = step.status === 'done' ? '✓'
                : step.status === 'running' ? '⏳'
                : '○';
            const row = document.createElement('div');
            row.className = `marl-step-row ${statusClass}`;
            row.style.animationDelay = `${i * 0.15}s`;
            row.innerHTML = `
                <div class="marl-step-icon">${icon}</div>
                <span class="marl-step-label">${step.label}</span>
                <span class="marl-step-detail">${step.detail || ''}</span>
            `;
            this._els.transferPipeline.appendChild(row);
        });
    }

    /**
     * Render per-agent transfer cards with details.
     */
    _renderTransferAgentCards(agents) {
        this._els.transferAgents.innerHTML = '';
        agents.forEach((agent, i) => {
            const card = document.createElement('div');
            card.className = 'marl-agent-transfer-card';
            card.style.borderLeft = `3px solid ${agent.color}`;
            card.style.animationDelay = `${0.3 + i * 0.12}s`;

            if (agent.skipped) {
                card.innerHTML = `
                    <div style="font-weight: 700; color: ${agent.color}; font-size: 14px; margin-bottom: 6px;">${agent.name}</div>
                    <div style="color: #94a3b8; font-size: 12px;">⏭ ${agent.skipReason}</div>
                `;
            } else {
                const epsDelta = agent.epsilonAfter - agent.epsilonBefore;
                const epsColor = epsDelta < 0 ? '#34d399' : '#f87171';
                const epsSign = epsDelta < 0 ? '' : '+';
                card.innerHTML = `
                    <div style="font-weight: 700; color: ${agent.color}; font-size: 14px; margin-bottom: 8px;">${agent.name}</div>
                    <div style="display: grid; grid-template-columns: auto 1fr; gap: 2px 8px; font-size: 12px;">
                        <span style="color: #64748b;">Expériences:</span>
                        <span style="color: #e2e8f0; font-weight: 600;">${agent.experiences.toLocaleString()}</span>
                        <span style="color: #64748b;">Apprentissage:</span>
                        <span style="color: #e2e8f0;">${agent.learnSteps} batches</span>
                        <span style="color: #64748b;">Epsilon:</span>
                        <span>${agent.epsilonBefore.toFixed(3)} → <span style="color:${epsColor}; font-weight:600;">${agent.epsilonAfter.toFixed(3)}</span> <span style="color:${epsColor}; font-size:11px;">(${epsSign}${epsDelta.toFixed(3)})</span></span>
                        <span style="color: #64748b;">Coopération:</span>
                        <span style="color: #a78bfa; font-weight: 600;">${(agent.cooperativeWeight * 100).toFixed(0)}%</span>
                        <span style="color: #64748b;">Zone:</span>
                        <span>${agent.zoneAssigned ? '<span style="color:#34d399;">✓ Assignée</span>' : '<span style="color:#94a3b8;">—</span>'}</span>
                    </div>
                `;
            }
            this._els.transferAgents.appendChild(card);
        });
    }

    /**
     * Render before/after comparison cards (called after emergence monitoring).
     */
    _renderBeforeAfter(before, after, delta) {
        this._els.transferComparison.style.display = 'block';
        const grid = this._els.transferBAGrid;
        grid.innerHTML = '';

        const metrics = [
            { key: 'emergence', label: 'Émergence', fmt: (v) => `${(v * 100).toFixed(1)}%`, fmtD: (v) => `${(v * 100).toFixed(1)}%` },
            { key: 'globalCoverage', label: 'Coverage', fmt: (v) => `${v.toFixed(1)}%`, fmtD: (v) => `${v.toFixed(1)}%` },
            { key: 'cohesion', label: 'Cohésion', fmt: (v) => `${(v * 100).toFixed(1)}%`, fmtD: (v) => `${(v * 100).toFixed(1)}%` },
            { key: 'separation', label: 'Séparation', fmt: (v) => `${(v * 100).toFixed(1)}%`, fmtD: (v) => `${(v * 100).toFixed(1)}%` },
            { key: 'alignment', label: 'Alignement', fmt: (v) => `${(v * 100).toFixed(1)}%`, fmtD: (v) => `${(v * 100).toFixed(1)}%` },
        ];

        // Before card
        const beforeCard = document.createElement('div');
        beforeCard.className = 'marl-ba-card';
        beforeCard.style.animationDelay = '0.1s';
        let beforeHTML = '<div style="font-size: 12px; color: #64748b; text-transform: uppercase; font-weight: 600; margin-bottom: 8px; text-align: center;">AVANT</div>';
        for (const m of metrics) {
            const val = before[m.key] || 0;
            beforeHTML += `<div class="marl-ba-row"><span style="color:#94a3b8;">${m.label}</span><span style="color:#e2e8f0; font-weight:600;">${m.fmt(val)}</span></div>`;
        }
        beforeCard.innerHTML = beforeHTML;

        // Arrow column
        const arrowCol = document.createElement('div');
        arrowCol.style.cssText = 'display: flex; flex-direction: column; justify-content: center; align-items: center; gap: 6px; padding-top: 28px;';
        for (let i = 0; i < metrics.length; i++) {
            const arrow = document.createElement('div');
            arrow.style.cssText = 'color: #7c3aed; font-size: 16px; line-height: 22px;';
            arrow.textContent = '→';
            arrowCol.appendChild(arrow);
        }

        // After card
        const afterCard = document.createElement('div');
        afterCard.className = 'marl-ba-card';
        afterCard.style.animationDelay = '0.3s';
        let afterHTML = '<div style="font-size: 12px; color: #64748b; text-transform: uppercase; font-weight: 600; margin-bottom: 8px; text-align: center;">APRÈS</div>';
        for (const m of metrics) {
            const val = after[m.key] || 0;
            afterHTML += `<div class="marl-ba-row"><span style="color:#94a3b8;">${m.label}</span><span style="color:#e2e8f0; font-weight:600;">${m.fmt(val)}</span></div>`;
        }
        afterCard.innerHTML = afterHTML;

        // Delta column
        const deltaCol = document.createElement('div');
        deltaCol.style.cssText = 'display: flex; flex-direction: column; gap: 4px; padding-top: 28px; padding-left: 8px;';
        metrics.forEach((m, i) => {
            const d = delta[m.key] || 0;
            const cls = d > 0.001 ? 'marl-delta-positive' : d < -0.001 ? 'marl-delta-negative' : 'marl-delta-neutral';
            const sign = d > 0 ? '+' : '';
            const pill = document.createElement('div');
            pill.className = `marl-delta-pill ${cls}`;
            pill.style.animationDelay = `${0.5 + i * 0.1}s`;
            pill.style.lineHeight = '22px';
            pill.textContent = `${sign}${m.fmtD(d)}`;
            deltaCol.appendChild(pill);
        });

        grid.appendChild(beforeCard);
        grid.appendChild(arrowCol);
        grid.appendChild(afterCard);
        grid.appendChild(deltaCol);
    }

    /**
     * Render the final verdict badge.
     */
    _renderVerdict(verdictText, compositeScore) {
        const el = this._els.transferVerdict;
        el.style.display = 'block';

        if (compositeScore > 0.05) {
            el.style.background = 'linear-gradient(90deg, rgba(52,211,153,0.15), rgba(52,211,153,0.05))';
            el.style.border = '1px solid #34d399';
            el.style.color = '#34d399';
        } else if (compositeScore > 0) {
            el.style.background = 'linear-gradient(90deg, rgba(251,191,36,0.15), rgba(251,191,36,0.05))';
            el.style.border = '1px solid #fbbf24';
            el.style.color = '#fbbf24';
        } else {
            el.style.background = 'linear-gradient(90deg, rgba(248,113,113,0.15), rgba(248,113,113,0.05))';
            el.style.border = '1px solid #f87171';
            el.style.color = '#f87171';
        }

        el.textContent = `${verdictText} — Score composite: ${(compositeScore * 100).toFixed(1)}%`;
    }

    /**
     * Save transfer result to localStorage for cross-session history.
     */
    _saveTransferHistory(report) {
        try {
            const key = 'diamants-marl-transfer-history';
            const raw = localStorage.getItem(key);
            const history = raw ? JSON.parse(raw) : [];
            history.push({
                timestamp: report.timestamp,
                verdict: report.verdict,
                compositeImprovement: report.compositeImprovement,
                coverage: {
                    before: report.before.globalCoverage || 0,
                    after: report.after.globalCoverage || 0,
                    delta: report.delta.globalCoverage || 0,
                },
                emergence: {
                    before: report.before.emergence || 0,
                    after: report.after.emergence || 0,
                    delta: report.delta.emergence || 0,
                },
            });
            // Keep last 20 entries
            if (history.length > 20) history.splice(0, history.length - 20);
            localStorage.setItem(key, JSON.stringify(history));
        } catch (e) {
            log(`⚠️ Sauvegarde historique échouée: ${e.message}`);
        }
    }

    /**
     * Render the session-over-session transfer history.
     */
    _renderTransferHistory() {
        try {
            const key = 'diamants-marl-transfer-history';
            const raw = localStorage.getItem(key);
            const history = raw ? JSON.parse(raw) : [];
            if (history.length === 0) {
                this._els.transferHistorySection.style.display = 'none';
                return;
            }

            this._els.transferHistorySection.style.display = 'block';
            this._els.transferHistoryItems.innerHTML = '';

            history.forEach((entry, i) => {
                const chip = document.createElement('div');
                const improvement = entry.compositeImprovement || 0;
                const isPositive = improvement > 0.001;
                const isNegative = improvement < -0.001;
                const bgColor = isPositive ? 'rgba(52,211,153,0.1)' : isNegative ? 'rgba(248,113,113,0.1)' : 'rgba(148,163,184,0.1)';
                const textColor = isPositive ? '#34d399' : isNegative ? '#f87171' : '#94a3b8';
                const sign = improvement > 0 ? '+' : '';
                const date = new Date(entry.timestamp);
                const timeStr = date.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' });

                chip.style.cssText = `
                    display: inline-flex; flex-direction: column; align-items: center;
                    padding: 6px 12px; border-radius: 8px; font-size: 12px;
                    background: ${bgColor}; border: 1px solid ${textColor}30;
                    min-width: 80px;
                `;
                chip.innerHTML = `
                    <span style="color: #64748b; font-size: 11px;">S${i + 1} • ${timeStr}</span>
                    <span style="color: ${textColor}; font-weight: 700; font-size: 14px;">${sign}${(improvement * 100).toFixed(1)}%</span>
                    <span style="color: #64748b; font-size: 10px;">Δcov: ${sign}${(entry.coverage?.delta || 0).toFixed(1)}%</span>
                `;
                this._els.transferHistoryItems.appendChild(chip);
            });
        } catch (e) {
            // Silently ignore in test
        }
    }

    // =========================================================================
    // Utils
    // =========================================================================

    _setConfigDisabled(disabled) {
        this._els.rounds.disabled = disabled;
        this._els.steps.disabled = disabled;
        this._els.strategy.disabled = disabled;
        this._els.lr.disabled = disabled;
        if (this._els.curriculum) this._els.curriculum.disabled = disabled;
    }

    _updateSessionInfo() {
        const stats = this._store.getStats();
        if (this._els.sessionCount) {
            this._els.sessionCount.textContent = `Session: #${stats.sessionCount + 1}`;
        }
        if (this._els.bestCoverage) {
            this._els.bestCoverage.textContent = stats.bestCoverage > 0
                ? `Best: ${(stats.bestCoverage * 100).toFixed(1)}%`
                : 'Best: —';
        }
        if (this._els.warmStartBadge) {
            if (stats.hasWeights) {
                this._els.warmStartBadge.textContent = '♻️ Warm-start';
                this._els.warmStartBadge.style.color = '#34d399';
                this._els.warmStartBadge.style.borderColor = '#34d399';
            } else {
                this._els.warmStartBadge.textContent = 'Cold start';
                this._els.warmStartBadge.style.color = '#94a3b8';
            }
        }
        if (this._els.curriculum) {
            this._els.curriculum.value = stats.curriculumStage;
        }
    }

    _initDrag() {
        const panel = this._root;
        const header = panel.querySelector('#marl-header');
        if (!header) return;

        let dragging = false, sx, sy, ox, oy;

        header.addEventListener('mousedown', (e) => {
            if (e.target.closest('button, input, select')) return;
            dragging = true;
            const r = panel.getBoundingClientRect();
            ox = r.left; oy = r.top;
            sx = e.clientX; sy = e.clientY;
            // freeze to px so calc() doesn't fight
            panel.style.left = ox + 'px';
            panel.style.top = oy + 'px';
            document.body.style.userSelect = 'none';
            header.style.cursor = 'grabbing';
            e.preventDefault();
        });

        document.addEventListener('mousemove', (e) => {
            if (!dragging) return;
            panel.style.left = Math.max(0, ox + e.clientX - sx) + 'px';
            panel.style.top  = Math.max(0, oy + e.clientY - sy) + 'px';
        });

        document.addEventListener('mouseup', () => {
            if (!dragging) return;
            dragging = false;
            document.body.style.userSelect = '';
            header.style.cursor = 'grab';
        });
    }

    /**
     * ResizeObserver-based responsive scaling via CSS transform.
     * Uses transform: scale() which does NOT affect layout, preventing feedback loops.
     * The wrapper keeps its natural 720px design width and is scaled to fill the panel.
     */
    _initResponsiveScale() {
        if (typeof ResizeObserver === 'undefined') return;
        const wrapper = this._root.querySelector('#marl-scale-wrapper');
        if (!wrapper) return;
        const BASE_W = 720; // design width at scale 1.0
        const BASE_H = 600; // design height
        // Set wrapper to fixed design size
        wrapper.style.width  = BASE_W + 'px';
        wrapper.style.height = BASE_H + 'px';
        let rafId = 0;
        this._resizeObserver = new ResizeObserver((entries) => {
            cancelAnimationFrame(rafId);
            rafId = requestAnimationFrame(() => {
                for (const entry of entries) {
                    const w = entry.contentRect.width;
                    const h = entry.contentRect.height;
                    if (w < 10) continue;
                    const sx = w / BASE_W;
                    const sy = h / BASE_H;
                    const s = Math.min(sx, sy);
                    wrapper.style.transform = `scale(${s})`;
                    // Update wrapper design size so scrollable content fills panel
                    wrapper.style.width  = (w / s) + 'px';
                    wrapper.style.height = (h / s) + 'px';
                }
            });
        });
        this._resizeObserver.observe(this._root);
    }
    _hookKeyboard() {
        document.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA' || e.target.tagName === 'SELECT') return;
            if (e.code === 'KeyT' && !e.ctrlKey && !e.altKey && !e.metaKey) {
                e.preventDefault();
                this.toggle();
            }
        });
    }

    toggle() {
        this._visible = !this._visible;
        this._root.style.display = this._visible ? 'flex' : 'none';
        log(`Panel ${this._visible ? 'visible' : 'hidden'}`);
    }

    show() { this._visible = true; this._root.style.display = 'flex'; }
    hide() { this._visible = false; this._root.style.display = 'none'; }
}
