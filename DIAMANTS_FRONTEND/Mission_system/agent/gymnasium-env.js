/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * gymnasium-env.js — Gymnasium-aligned Multi-Agent Swarm Environment
 * ====================================================================
 * Follows the Gymnasium (Farama) API:
 *   - step(actions)  → { obs, rewards, terminated, truncated, info }
 *   - reset(options)  → { obs, info }
 *   - observation_space / action_space definitions
 *
 * Key improvements over the original MiniSwarmEnv:
 *   1. 16-dim observation per agent (closer to real 24-dim)
 *   2. Proper terminated vs truncated distinction
 *   3. Info dict with per-agent metrics
 *   4. Curriculum support (difficulty scaling)
 *   5. Richer reward with exploration/cooperation/emergence components
 *   6. Deterministic seed support for reproducibility
 *
 * @module gymnasium-env
 */

const log = (...a) => console.log('[Gym-Env]', ...a);

// ─── SPACES (Gymnasium-style descriptors, JS-compatible) ─────────────

/**
 * Box space: continuous vector with bounds.
 * Equivalent to gymnasium.spaces.Box(low, high, shape)
 */
export class BoxSpace {
    constructor(low, high, shape) {
        this.low = low instanceof Float32Array ? low : new Float32Array(shape).fill(low);
        this.high = high instanceof Float32Array ? high : new Float32Array(shape).fill(high);
        this.shape = shape;
        this.dtype = 'float32';
    }

    sample() {
        const s = new Float32Array(this.shape);
        for (let i = 0; i < this.shape; i++) {
            s[i] = this.low[i] + Math.random() * (this.high[i] - this.low[i]);
        }
        return s;
    }

    contains(x) {
        if (!x || x.length !== this.shape) return false;
        for (let i = 0; i < this.shape; i++) {
            if (x[i] < this.low[i] - 1e-6 || x[i] > this.high[i] + 1e-6) return false;
        }
        return true;
    }
}

// ─── OBSERVATION INDICES (16-dim per agent) ──────────────────────────

export const GymObsIndex = Object.freeze({
    // Position & movement (0-4)
    POS_X:           0,   // x / zone_size → [-1, 1]
    POS_Z:           1,   // z / zone_size → [-1, 1]
    VEL_X:           2,   // velocity x (normalized)
    VEL_Z:           3,   // velocity z (normalized)
    HEADING:         4,   // heading / π → [-1, 1]

    // Perception (5-8)
    OBSTACLE_FRONT:  5,   // dist to nearest obstacle ahead / range → [0, 1]
    OBSTACLE_LEFT:   6,   // dist to nearest obstacle left / range → [0, 1]
    OBSTACLE_RIGHT:  7,   // dist to nearest obstacle right / range → [0, 1]
    FRONTIER_DIST:   8,   // dist to nearest frontier / zone_half → [0, 1]

    // Coverage (9-10)
    LOCAL_COVERAGE:  9,   // local area coverage → [0, 1]
    GLOBAL_COVERAGE: 10,  // team coverage → [0, 1]

    // Multi-agent (11-15)
    NEIGHBOR_COUNT:  11,  // normalized neighbor count → [0, 1]
    NEAREST_DIST:    12,  // dist to nearest agent / range → [0, 1]
    NEAREST_DIR_X:   13,  // direction to nearest agent x → [-1, 1]
    NEAREST_DIR_Z:   14,  // direction to nearest agent z → [-1, 1]
    TIME_REMAINING:  15,  // 1 - step/maxSteps → [0, 1]
});

export const GYM_OBS_DIM = 16;

// ─── CURRICULUM STAGES ───────────────────────────────────────────────

export const CURRICULUM = Object.freeze([
    { name: 'Novice',       gridDim: 30, zone: 60,  obstacles: 5,  maxSteps: 300,  agents: 4 },
    { name: 'Apprenti',     gridDim: 40, zone: 80,  obstacles: 15, maxSteps: 500,  agents: 4 },
    { name: 'Intermédiaire', gridDim: 50, zone: 100, obstacles: 25, maxSteps: 700,  agents: 4 },
    { name: 'Avancé',       gridDim: 60, zone: 120, obstacles: 40, maxSteps: 1000, agents: 4 },
    { name: 'Expert',       gridDim: 80, zone: 160, obstacles: 60, maxSteps: 1500, agents: 4 },
]);

// ─── GYMNASIUM SWARM ENVIRONMENT ─────────────────────────────────────

export class GymnasiumSwarmEnv {
    /**
     * @param {Object} config
     * @param {number} config.numAgents - Number of agents (default: 4)
     * @param {number} config.gridDim - Grid dimension (default: 40)
     * @param {number} config.zoneSize - Zone size in meters (default: 80)
     * @param {number} config.maxSteps - Max steps per episode (default: 500)
     * @param {number} config.numObstacles - Number of obstacles (default: 20)
     * @param {number} config.curriculumStage - 0-4 curriculum index (default: 1)
     * @param {number} config.seed - Random seed (default: null)
     */
    constructor(config = {}) {
        // Apply curriculum if specified
        const stage = config.curriculumStage != null
            ? CURRICULUM[Math.min(config.curriculumStage, CURRICULUM.length - 1)]
            : null;

        this.numAgents    = config.numAgents    ?? stage?.agents    ?? 4;
        this.gridDim      = config.gridDim      ?? stage?.gridDim   ?? 40;
        this.zoneSize     = config.zoneSize     ?? stage?.zone      ?? 80;
        this.maxSteps     = config.maxSteps     ?? stage?.maxSteps  ?? 500;
        this.numObstacles = config.numObstacles ?? stage?.obstacles ?? 20;
        this.curriculumStage = config.curriculumStage ?? 1;

        this.cellSize = this.zoneSize / this.gridDim;
        this.grid = new Float32Array(this.gridDim * this.gridDim);
        this.agents = [];
        this._step = 0;
        this.obstacles = [];
        this._seed = config.seed ?? null;
        this._rng = this._createRNG(this._seed);

        // Gymnasium-style spaces
        const obsLow  = new Float32Array(GYM_OBS_DIM).fill(-1);
        const obsHigh = new Float32Array(GYM_OBS_DIM).fill(1);
        // Coverage and distance dims are [0,1]
        for (const idx of [5, 6, 7, 8, 9, 10, 11, 12, 15]) {
            obsLow[idx] = 0;
        }

        this.observation_space = new BoxSpace(obsLow, obsHigh, GYM_OBS_DIM);
        this.action_space = new BoxSpace(-1, 1, 4); // [dx, dz, spread, signal] normalized

        // Track per-agent stats
        this._prevCoverage = 0;
        this._prevAgentPositions = [];
        this._episodeRewards = [];
    }

    // ── Seeded RNG (Mulberry32) ──────────────────────────────────────

    _createRNG(seed) {
        if (seed == null) {
            seed = Math.floor(Math.random() * 2147483647);
        }
        let s = seed | 0;
        return () => {
            s = (s + 0x6D2B79F5) | 0;
            let t = Math.imul(s ^ (s >>> 15), 1 | s);
            t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
            return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
        };
    }

    _rand() { return this._rng(); }
    _randRange(min, max) { return min + this._rand() * (max - min); }

    // ── Gymnasium API: reset() ───────────────────────────────────────

    /**
     * Reset environment to initial state.
     * @param {Object} options - { seed?: number }
     * @returns {{ obs: Float32Array[], info: Object }}
     */
    reset(options = {}) {
        if (options.seed != null) {
            this._seed = options.seed;
            this._rng = this._createRNG(options.seed);
        }

        this.grid.fill(0);
        this._step = 0;
        this.agents = [];
        this.obstacles = [];
        this._prevCoverage = 0;
        this._episodeRewards = [];

        // Place obstacles
        for (let i = 0; i < this.numObstacles; i++) {
            this.obstacles.push({
                x: this._randRange(-0.4, 0.4) * this.zoneSize,
                z: this._randRange(-0.4, 0.4) * this.zoneSize,
                r: 2 + this._rand() * 3,
            });
        }

        // Place agents spread out in quadrants
        const angleOffset = this._rand() * Math.PI * 2;
        for (let i = 0; i < this.numAgents; i++) {
            const angle = angleOffset + (i / this.numAgents) * Math.PI * 2;
            const r = 8 + this._rand() * 6;
            this.agents.push({
                x: r * Math.cos(angle),
                z: r * Math.sin(angle),
                vx: 0, vz: 0,
                heading: angle,
                coverage: 0,
                reward: 0,
                cumulativeReward: 0,
                newCells: 0,
                overlapCells: 0,
            });
        }

        this._markCoverage();
        this._prevAgentPositions = this.agents.map(a => ({ x: a.x, z: a.z }));

        const obs = this._getObs();
        const info = {
            coverage: this.getCoverage(),
            step: 0,
            curriculum: this.curriculumStage,
        };

        return { obs, info };
    }

    // ── Gymnasium API: step(actions) ─────────────────────────────────

    /**
     * Step all agents with given actions.
     * @param {Array<{dx: number, dz: number, spread: number, signal: number}>} actions
     * @returns {{ obs, rewards, terminated, truncated, info }}
     */
    step(actions) {
        this._step++;
        const rewards = [];
        const covBefore = this.getCoverage();

        // === Apply actions ===
        for (let i = 0; i < this.numAgents; i++) {
            const a = this.agents[i];
            const act = actions[i] || { dx: 0, dz: 0, spread: 5, signal: 0.5 };

            // Smooth velocity update (inertia)
            const targetX = a.x + act.dx;
            const targetZ = a.z + act.dz;
            const speed = 1.5;
            const dx = targetX - a.x;
            const dz = targetZ - a.z;
            const dist = Math.sqrt(dx * dx + dz * dz) || 1;
            const newVx = (dx / dist) * Math.min(speed, dist);
            const newVz = (dz / dist) * Math.min(speed, dist);

            // Apply with inertia (0.3 old + 0.7 new)
            a.vx = a.vx * 0.3 + newVx * 0.7;
            a.vz = a.vz * 0.3 + newVz * 0.7;
            a.x += a.vx;
            a.z += a.vz;

            // Clamp to zone
            const half = this.zoneSize / 2 - 2;
            a.x = Math.max(-half, Math.min(half, a.x));
            a.z = Math.max(-half, Math.min(half, a.z));
            a.heading = Math.atan2(a.vz, a.vx);

            // Spread controls virtual scout radius
            a.scouts = Math.max(1, Math.min(4, Math.round(act.spread / 4)));
        }

        // === Coverage update ===
        this._markCoverage();
        const covAfter = this.getCoverage();
        const globalCoverageGain = covAfter - covBefore;

        // === Compute rich rewards per agent ===
        for (let i = 0; i < this.numAgents; i++) {
            const a = this.agents[i];
            const act = actions[i] || { dx: 0, dz: 0, spread: 5, signal: 0.5 };
            let r = 0;

            // ── Individual: coverage reward ──
            r += globalCoverageGain * 60;

            // ── Individual: new cell exploration bonus ──
            r += a.newCells * 0.3;

            // ── Individual: movement efficiency (penalize being stuck) ──
            const speed = Math.sqrt(a.vx * a.vx + a.vz * a.vz);
            if (speed < 0.1) r -= 0.1;

            // ── Individual: obstacle penalty ──
            for (const obs of this.obstacles) {
                const d = Math.sqrt((a.x - obs.x) ** 2 + (a.z - obs.z) ** 2);
                if (d < obs.r + 1) r -= 5;
                else if (d < obs.r + 3) r -= 0.5; // proximity warning
            }

            // ── Cooperative: separation reward/penalty ──
            let tooCloseCount = 0;
            for (let j = 0; j < this.numAgents; j++) {
                if (j === i) continue;
                const b = this.agents[j];
                const d = Math.sqrt((a.x - b.x) ** 2 + (a.z - b.z) ** 2);
                if (d < 3) { r -= 2; tooCloseCount++; }
                else if (d > 5 && d < 25) r += 0.1; // good separation bonus
            }

            // ── Cooperative: overlap penalty ──
            if (a.overlapCells > 0) {
                r -= a.overlapCells * 0.2;
            }

            // ── Communication bonus (scaled by signal strength) ──
            const signal = act.signal ?? 0.5;
            if (signal > 0.5) r += (signal - 0.5) * 1.0;

            // ── Emergence: action diversity across team ──
            const actionsThisStep = actions.map(ac => {
                const mag = Math.sqrt((ac.dx || 0) ** 2 + (ac.dz || 0) ** 2);
                if (mag < 1) return 'hover';
                const angle = Math.atan2(ac.dz || 0, ac.dx || 0);
                if (angle > -0.785 && angle < 0.785) return 'east';
                if (angle >= 0.785 && angle < 2.356) return 'north';
                if (angle < -0.785 && angle > -2.356) return 'south';
                return 'west';
            });
            const uniqueDirections = new Set(actionsThisStep).size;
            if (uniqueDirections >= 3) r += 0.3 * (uniqueDirections / this.numAgents);

            // ── Emergence: spatial spread bonus ──
            const centroid = this._computeCentroid();
            const distFromCentroid = Math.sqrt(
                (a.x - centroid.x) ** 2 + (a.z - centroid.z) ** 2
            );
            if (distFromCentroid > 10 && distFromCentroid < 40) r += 0.15;

            // ── Step penalty ──
            r -= 0.01;

            // ── Mission complete mega-bonus ──
            if (covAfter >= 0.95) r += 50;

            a.reward = r;
            a.cumulativeReward += r;
            a.coverage = covAfter;
            rewards.push(r);
        }

        this._prevCoverage = covAfter;
        this._prevAgentPositions = this.agents.map(a => ({ x: a.x, z: a.z }));

        // === Terminated vs Truncated (Gymnasium v0.26+) ===
        const terminated = covAfter >= 0.95;  // MDP terminal: mission complete
        const truncated = this._step >= this.maxSteps; // Time limit

        // === Info dict ===
        const info = {
            coverage: covAfter,
            coverageGain: globalCoverageGain,
            step: this._step,
            perAgentReward: rewards.slice(),
            perAgentCoverage: this.agents.map(() => covAfter),
            perAgentCumulative: this.agents.map(a => a.cumulativeReward),
            avgReward: rewards.reduce((s, v) => s + v, 0) / this.numAgents,
            curriculum: this.curriculumStage,
        };

        const obs = this._getObs();
        return { obs, rewards, terminated, truncated, info };
    }

    // ── Observation builder (16-dim per agent) ───────────────────────

    _getObs() {
        const halfZone = this.zoneSize / 2;
        const globalCov = this.getCoverage();
        const obs = [];

        for (let i = 0; i < this.numAgents; i++) {
            const a = this.agents[i];
            const o = new Float32Array(GYM_OBS_DIM);

            // Position (normalized to [-1, 1])
            o[GymObsIndex.POS_X] = a.x / halfZone;
            o[GymObsIndex.POS_Z] = a.z / halfZone;

            // Velocity
            o[GymObsIndex.VEL_X] = Math.max(-1, Math.min(1, a.vx));
            o[GymObsIndex.VEL_Z] = Math.max(-1, Math.min(1, a.vz));

            // Heading
            o[GymObsIndex.HEADING] = a.heading / Math.PI;

            // Obstacle sensing (ray-cast approximation in 3 directions)
            const senseRange = 15;
            o[GymObsIndex.OBSTACLE_FRONT] = this._senseObstacle(a, a.heading, senseRange) / senseRange;
            o[GymObsIndex.OBSTACLE_LEFT] = this._senseObstacle(a, a.heading + Math.PI / 3, senseRange) / senseRange;
            o[GymObsIndex.OBSTACLE_RIGHT] = this._senseObstacle(a, a.heading - Math.PI / 3, senseRange) / senseRange;

            // Frontier distance
            o[GymObsIndex.FRONTIER_DIST] = this._nearestFrontierDist(a) / halfZone;

            // Coverage
            o[GymObsIndex.LOCAL_COVERAGE] = this._localCoverage(a);
            o[GymObsIndex.GLOBAL_COVERAGE] = globalCov;

            // Neighbor info
            const { count, nearest, nearDx, nearDz } = this._neighborInfo(i);
            o[GymObsIndex.NEIGHBOR_COUNT] = Math.min(1, count / Math.max(1, this.numAgents - 1));
            o[GymObsIndex.NEAREST_DIST] = Math.min(1, nearest / 40);
            o[GymObsIndex.NEAREST_DIR_X] = nearDx;
            o[GymObsIndex.NEAREST_DIR_Z] = nearDz;

            // Time remaining
            o[GymObsIndex.TIME_REMAINING] = 1 - this._step / this.maxSteps;

            obs.push(o);
        }
        return obs;
    }

    // ── Sensing helpers ──────────────────────────────────────────────

    _senseObstacle(agent, angle, range) {
        // Simple ray-cast: check obstacles along direction
        const cos = Math.cos(angle);
        const sin = Math.sin(angle);
        let minDist = range;
        for (const obs of this.obstacles) {
            const dx = obs.x - agent.x;
            const dz = obs.z - agent.z;
            const proj = dx * cos + dz * sin; // projection on ray
            if (proj < 0 || proj > range) continue;
            const perp = Math.abs(-dx * sin + dz * cos); // perpendicular distance
            if (perp < obs.r + 1) {
                minDist = Math.min(minDist, Math.max(0, proj - obs.r));
            }
        }
        // Also sense zone boundary
        const bx = (cos > 0 ? this.zoneSize / 2 - agent.x : agent.x + this.zoneSize / 2) / Math.abs(cos || 1);
        const bz = (sin > 0 ? this.zoneSize / 2 - agent.z : agent.z + this.zoneSize / 2) / Math.abs(sin || 1);
        minDist = Math.min(minDist, Math.max(0, Math.min(bx, bz)));
        return minDist;
    }

    _nearestFrontierDist(agent) {
        let minDist = this.zoneSize;
        const half = this.zoneSize / 2;
        // Sample frontier cells (unexplored areas)
        for (let attempts = 0; attempts < 50; attempts++) {
            const gx = Math.floor(this._rand() * this.gridDim);
            const gz = Math.floor(this._rand() * this.gridDim);
            const idx = gz * this.gridDim + gx;
            if (this.grid[idx] < 0.3) {
                const wx = (gx / this.gridDim - 0.5) * this.zoneSize;
                const wz = (gz / this.gridDim - 0.5) * this.zoneSize;
                const d = Math.sqrt((wx - agent.x) ** 2 + (wz - agent.z) ** 2);
                if (d < minDist) minDist = d;
            }
        }
        return minDist;
    }

    _localCoverage(agent) {
        // Coverage in a 12m radius around agent
        const radius = 12;
        let total = 0, covered = 0;
        for (let dx = -radius; dx <= radius; dx += this.cellSize) {
            for (let dz = -radius; dz <= radius; dz += this.cellSize) {
                if (dx * dx + dz * dz > radius * radius) continue;
                const idx = this._gridIdx(agent.x + dx, agent.z + dz);
                if (idx >= 0) {
                    total++;
                    if (this.grid[idx] > 0.5) covered++;
                }
            }
        }
        return total > 0 ? covered / total : 0;
    }

    _neighborInfo(agentIdx) {
        const a = this.agents[agentIdx];
        let count = 0;
        let nearest = 999;
        let nearDx = 0, nearDz = 0;

        for (let j = 0; j < this.numAgents; j++) {
            if (j === agentIdx) continue;
            const b = this.agents[j];
            const d = Math.sqrt((a.x - b.x) ** 2 + (a.z - b.z) ** 2);
            if (d < 30) count++;
            if (d < nearest) {
                nearest = d;
                const norm = d || 1;
                nearDx = (b.x - a.x) / norm;
                nearDz = (b.z - a.z) / norm;
            }
        }

        return {
            count,
            nearest: Math.min(nearest, 40),
            nearDx: Math.max(-1, Math.min(1, nearDx)),
            nearDz: Math.max(-1, Math.min(1, nearDz)),
        };
    }

    _computeCentroid() {
        let cx = 0, cz = 0;
        for (const a of this.agents) { cx += a.x; cz += a.z; }
        return { x: cx / this.numAgents, z: cz / this.numAgents };
    }

    // ── Grid helpers ─────────────────────────────────────────────────

    _gridIdx(wx, wz) {
        const half = this.zoneSize / 2;
        const gx = Math.floor((wx + half) / this.cellSize);
        const gz = Math.floor((wz + half) / this.cellSize);
        if (gx < 0 || gx >= this.gridDim || gz < 0 || gz >= this.gridDim) return -1;
        return gz * this.gridDim + gx;
    }

    _markCoverage() {
        for (const a of this.agents) {
            a.newCells = 0;
            a.overlapCells = 0;
            const radius = 6 + (a.scouts || 2) * 2;
            for (let dx = -radius; dx <= radius; dx += this.cellSize) {
                for (let dz = -radius; dz <= radius; dz += this.cellSize) {
                    if (dx * dx + dz * dz > radius * radius) continue;
                    const idx = this._gridIdx(a.x + dx, a.z + dz);
                    if (idx >= 0) {
                        if (this.grid[idx] < 0.3) a.newCells++;
                        else if (this.grid[idx] > 0.7) a.overlapCells++;
                        this.grid[idx] = Math.min(1, this.grid[idx] + 0.3);
                    }
                }
            }
        }
    }

    getCoverage() {
        let covered = 0;
        for (let i = 0; i < this.grid.length; i++) {
            if (this.grid[i] > 0.5) covered++;
        }
        return covered / this.grid.length;
    }
}


// ═══════════════════════════════════════════════════════════════════════
// PPO Policy Network (Pure JS, no dependencies)
// ═══════════════════════════════════════════════════════════════════════

/**
 * PPO-style policy with clipped surrogate objective.
 * Architecture: 16 → 64 → 64 → 4 (tanh) with separate value head.
 *
 * Improvements over TinyPolicy:
 *   - Proper PPO clipped objective (no finite differences)
 *   - Value function baseline for variance reduction
 *   - GAE (Generalized Advantage Estimation)
 *   - Entropy bonus for exploration
 *   - Gradient clipping
 */
export class PPOPolicy {
    constructor(obsDim = GYM_OBS_DIM, hiddenDim = 64, actDim = 4) {
        this.obsDim = obsDim;
        this.hiddenDim = hiddenDim;
        this.actDim = actDim;

        // Policy network: obs → hidden → hidden → action_mean
        this.w1 = this._xavier(obsDim, hiddenDim);
        this.b1 = new Float32Array(hiddenDim);
        this.w2 = this._xavier(hiddenDim, hiddenDim);
        this.b2 = new Float32Array(hiddenDim);
        this.w3 = this._xavier(hiddenDim, actDim);
        this.b3 = new Float32Array(actDim);

        // Value network: obs → hidden → hidden → 1
        this.vw1 = this._xavier(obsDim, hiddenDim);
        this.vb1 = new Float32Array(hiddenDim);
        this.vw2 = this._xavier(hiddenDim, hiddenDim);
        this.vb2 = new Float32Array(hiddenDim);
        this.vw3 = this._xavier(hiddenDim, 1);
        this.vb3 = new Float32Array(1);

        // Log standard deviation (learned, per action dim)
        this.logStd = new Float32Array(actDim).fill(-0.5);

        // Adam state
        this._adamState = {};
        this._t = 0;

        // PPO hyperparameters
        this.clipEps = 0.2;
        this.entropyCoef = 0.01;
        this.valueLossCoef = 0.5;
        this.maxGradNorm = 0.5;
        this.gaeLambda = 0.95;
        this.gamma = 0.99;

        // Training stats
        this.updateCount = 0;
        this.avgPolicyLoss = 0;
        this.avgValueLoss = 0;
        this.avgEntropy = 0;
    }

    _xavier(rows, cols) {
        const scale = Math.sqrt(2 / (rows + cols));
        const a = new Float32Array(rows * cols);
        for (let i = 0; i < a.length; i++) {
            const u1 = Math.random() || 1e-8;
            const u2 = Math.random();
            a[i] = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2) * scale;
        }
        a._rows = rows;
        a._cols = cols;
        return a;
    }

    // ── Forward passes ───────────────────────────────────────────────

    forward(obs) {
        let h = this._linear(obs, this.w1, this.b1);
        h = this._tanh(h);
        h = this._linear(h, this.w2, this.b2);
        h = this._tanh(h);
        return this._linear(h, this.w3, this.b3); // action means
    }

    value(obs) {
        let h = this._linear(obs, this.vw1, this.vb1);
        h = this._tanh(h);
        h = this._linear(h, this.vw2, this.vb2);
        h = this._tanh(h);
        return this._linear(h, this.vw3, this.vb3)[0]; // scalar value
    }

    /**
     * Sample action from Gaussian policy.
     * @returns {{ action: Float32Array, logProb: number, value: number }}
     */
    sampleAction(obs) {
        const mean = this.forward(obs);
        const v = this.value(obs);
        const action = new Float32Array(this.actDim);
        let logProb = 0;

        for (let i = 0; i < this.actDim; i++) {
            const std = Math.exp(this.logStd[i]);
            const u1 = Math.random() || 1e-8;
            const u2 = Math.random();
            const noise = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
            action[i] = mean[i] + noise * std;

            // Log probability of this action under the Gaussian
            logProb += -0.5 * ((action[i] - mean[i]) / std) ** 2
                       - Math.log(std) - 0.5 * Math.log(2 * Math.PI);
        }

        return { action, logProb, value: v };
    }

    /**
     * Compute log probability of action under current policy.
     */
    logProbOf(obs, action) {
        const mean = this.forward(obs);
        let lp = 0;
        for (let i = 0; i < this.actDim; i++) {
            const std = Math.exp(this.logStd[i]);
            lp += -0.5 * ((action[i] - mean[i]) / std) ** 2
                  - Math.log(std) - 0.5 * Math.log(2 * Math.PI);
        }
        return lp;
    }

    /**
     * Compute Gaussian entropy.
     */
    entropy() {
        let h = 0;
        for (let i = 0; i < this.actDim; i++) {
            const std = Math.exp(this.logStd[i]);
            h += 0.5 * Math.log(2 * Math.PI * Math.E * std * std);
        }
        return h;
    }

    // Scale raw output → env actions
    toEnvAction(raw) {
        return {
            dx: Math.tanh(raw[0]) * 15,
            dz: Math.tanh(raw[1]) * 15,
            spread: 3 + (1 + Math.tanh(raw[2])) * 6,
            signal: 1 / (1 + Math.exp(-raw[3])),
        };
    }

    // ── PPO Update ───────────────────────────────────────────────────

    /**
     * PPO clipped surrogate update from a trajectory buffer.
     * @param {Array<{obs, action, logProb, reward, value, done}>} trajectory
     * @param {number} lr - Learning rate
     * @param {number} epochs - Number of optimization epochs (default: 4)
     * @param {number} miniBatchSize - Mini-batch size (default: 64)
     */
    async ppoUpdate(trajectory, lr = 0.0003, epochs = 1, miniBatchSize = 32) {
        if (trajectory.length < 2) return;

        // ── Step 1: Compute GAE advantages and returns ──
        const advantages = new Float32Array(trajectory.length);
        const returns = new Float32Array(trajectory.length);

        let lastGae = 0;
        for (let t = trajectory.length - 1; t >= 0; t--) {
            const nextValue = t < trajectory.length - 1 ? trajectory[t + 1].value : 0;
            const nextDone = t < trajectory.length - 1 ? (trajectory[t + 1].done ? 1 : 0) : 1;
            const delta = trajectory[t].reward + this.gamma * nextValue * (1 - nextDone) - trajectory[t].value;
            lastGae = delta + this.gamma * this.gaeLambda * (1 - nextDone) * lastGae;
            advantages[t] = lastGae;
            returns[t] = advantages[t] + trajectory[t].value;
        }

        // Normalize advantages
        let mean = 0, std = 0;
        for (const a of advantages) mean += a;
        mean /= advantages.length;
        for (const a of advantages) std += (a - mean) ** 2;
        std = Math.sqrt(std / advantages.length) || 1;
        for (let i = 0; i < advantages.length; i++) {
            advantages[i] = (advantages[i] - mean) / std;
        }

        // ── Step 2: Multiple epochs of mini-batch PPO ──
        let totalPolicyLoss = 0, totalValueLoss = 0, totalEntropy = 0;
        let numUpdates = 0;

        for (let epoch = 0; epoch < epochs; epoch++) {
            // Shuffle indices
            const indices = Array.from({ length: trajectory.length }, (_, i) => i);
            for (let i = indices.length - 1; i > 0; i--) {
                const j = Math.floor(Math.random() * (i + 1));
                [indices[i], indices[j]] = [indices[j], indices[i]];
            }

            // Mini-batches
            for (let start = 0; start < indices.length; start += miniBatchSize) {
                const batch = indices.slice(start, start + miniBatchSize);
                if (batch.length < 4) continue;

                // Compute losses with numerical gradients (efficient partial sampling)
                const weights = this.getWeights();
                const grad = new Float32Array(weights.length);
                const eps = 0.005;

                // Sample parameters to update (stochastic coordinate descent)
                const numParams = Math.min(16, weights.length);
                const paramIndices = [];
                for (let p = 0; p < numParams; p++) {
                    paramIndices.push(Math.floor(Math.random() * weights.length));
                }

                let batchPolicyLoss = 0;
                let batchValueLoss = 0;

                for (const pidx of paramIndices) {
                    const orig = weights[pidx];
                    let lossPlus = 0, lossMinus = 0;

                    weights[pidx] = orig + eps;
                    this.setWeights(weights);
                    for (const bi of batch) {
                        const t = trajectory[bi];
                        const obsF = new Float32Array(t.obs);
                        const actF = new Float32Array(t.action);

                        // New log prob
                        const newLP = this.logProbOf(obsF, actF);
                        const ratio = Math.exp(newLP - t.logProb);
                        const adv = advantages[bi];

                        // Clipped surrogate
                        const surr1 = ratio * adv;
                        const surr2 = Math.max(1 - this.clipEps, Math.min(1 + this.clipEps, ratio)) * adv;
                        const policyLoss = -Math.min(surr1, surr2);

                        // Value loss
                        const newV = this.value(obsF);
                        const vLoss = 0.5 * (newV - returns[bi]) ** 2;

                        lossPlus += policyLoss + this.valueLossCoef * vLoss;
                    }

                    weights[pidx] = orig - eps;
                    this.setWeights(weights);
                    for (const bi of batch) {
                        const t = trajectory[bi];
                        const obsF = new Float32Array(t.obs);
                        const actF = new Float32Array(t.action);

                        const newLP = this.logProbOf(obsF, actF);
                        const ratio = Math.exp(newLP - t.logProb);
                        const adv = advantages[bi];

                        const surr1 = ratio * adv;
                        const surr2 = Math.max(1 - this.clipEps, Math.min(1 + this.clipEps, ratio)) * adv;
                        const policyLoss = -Math.min(surr1, surr2);

                        const newV = this.value(obsF);
                        const vLoss = 0.5 * (newV - returns[bi]) ** 2;

                        lossMinus += policyLoss + this.valueLossCoef * vLoss;
                    }

                    weights[pidx] = orig;
                    grad[pidx] = (lossPlus - lossMinus) / (2 * eps * batch.length);

                    batchPolicyLoss = lossPlus / batch.length;
                    batchValueLoss = lossMinus / batch.length;
                }

                this.setWeights(weights);

                // Entropy bonus
                const ent = this.entropy();

                // Gradient clipping
                let gradNorm = 0;
                for (const g of grad) gradNorm += g * g;
                gradNorm = Math.sqrt(gradNorm);
                if (gradNorm > this.maxGradNorm) {
                    const scale = this.maxGradNorm / gradNorm;
                    for (let i = 0; i < grad.length; i++) grad[i] *= scale;
                }

                // Add entropy gradient (encourage exploration)
                for (let i = 0; i < this.logStd.length; i++) {
                    const sIdx = weights.length - this.logStd.length + i;
                    if (sIdx >= 0) grad[sIdx] -= this.entropyCoef * 0.01;
                }

                // Adam update
                this._adamUpdate(weights, grad, lr);

                totalPolicyLoss += batchPolicyLoss;
                totalValueLoss += batchValueLoss;
                totalEntropy += ent;
                numUpdates++;

                // Yield to the event loop between mini-batches so the UI stays responsive
                // Use 4ms (1 frame minimum) to give the browser real breathing room
                if (typeof globalThis.setTimeout === 'function') {
                    await new Promise(r => setTimeout(r, 4));
                }
            }
        }

        this.updateCount++;
        if (numUpdates > 0) {
            this.avgPolicyLoss = totalPolicyLoss / numUpdates;
            this.avgValueLoss = totalValueLoss / numUpdates;
            this.avgEntropy = totalEntropy / numUpdates;
        }
    }

    _adamUpdate(weights, grad, lr) {
        this._t++;
        const beta1 = 0.9, beta2 = 0.999, epsilon = 1e-8;

        if (!this._mFlat || this._mFlat.length !== weights.length) {
            this._mFlat = new Float32Array(weights.length);
            this._vFlat = new Float32Array(weights.length);
        }

        for (let i = 0; i < weights.length; i++) {
            this._mFlat[i] = beta1 * this._mFlat[i] + (1 - beta1) * grad[i];
            this._vFlat[i] = beta2 * this._vFlat[i] + (1 - beta2) * grad[i] * grad[i];
            const mHat = this._mFlat[i] / (1 - beta1 ** this._t);
            const vHat = this._vFlat[i] / (1 - beta2 ** this._t);
            weights[i] -= lr * mHat / (Math.sqrt(vHat) + epsilon);
        }

        this.setWeights(weights);
    }

    // ── Weight serialization ─────────────────────────────────────────

    getWeights() {
        const parts = [
            this.w1, this.b1, this.w2, this.b2, this.w3, this.b3,
            this.vw1, this.vb1, this.vw2, this.vb2, this.vw3, this.vb3,
            this.logStd
        ];
        let total = 0;
        for (const p of parts) total += p.length;
        const flat = new Float32Array(total);
        let offset = 0;
        for (const p of parts) { flat.set(p, offset); offset += p.length; }
        return flat;
    }

    setWeights(flat) {
        const parts = [
            this.w1, this.b1, this.w2, this.b2, this.w3, this.b3,
            this.vw1, this.vb1, this.vw2, this.vb2, this.vw3, this.vb3,
            this.logStd
        ];
        let offset = 0;
        for (const p of parts) {
            for (let i = 0; i < p.length; i++) p[i] = flat[offset + i];
            offset += p.length;
        }
    }

    getWeightCount() {
        return this.getWeights().length;
    }

    // ── Utility ──────────────────────────────────────────────────────

    _linear(x, w, b) {
        const rows = w._rows, cols = w._cols;
        const out = new Float32Array(cols);
        for (let j = 0; j < cols; j++) {
            let s = b[j];
            for (let i = 0; i < rows; i++) s += x[i] * w[i * cols + j];
            out[j] = s;
        }
        return out;
    }

    _tanh(x) {
        const out = new Float32Array(x.length);
        for (let i = 0; i < x.length; i++) out[i] = Math.tanh(x[i]);
        return out;
    }

    /**
     * Clone this policy (deep copy weights).
     */
    clone() {
        const c = new PPOPolicy(this.obsDim, this.hiddenDim, this.actDim);
        c.setWeights(new Float32Array(this.getWeights()));
        c.clipEps = this.clipEps;
        c.entropyCoef = this.entropyCoef;
        c.gamma = this.gamma;
        c.gaeLambda = this.gaeLambda;
        return c;
    }
}


// ═══════════════════════════════════════════════════════════════════════
// MARL Session Store — Persistent weight storage across sessions
// ═══════════════════════════════════════════════════════════════════════

const STORAGE_KEY = 'diamants_marl_session';
const STORAGE_VERSION = 2;

/**
 * Stores and restores MARL training state across browser sessions.
 * Uses localStorage with automatic fallback.
 *
 * Persists:
 *   - Policy weights for all agents
 *   - Training history (rewards, coverage curves)
 *   - Curriculum progress
 *   - Session count
 *   - Best weights (highest coverage model)
 */
export class MARLSessionStore {
    constructor() {
        this._data = null;
        this._load();
    }

    _load() {
        try {
            if (typeof localStorage === 'undefined') {
                this._data = this._default();
                return;
            }
            const raw = localStorage.getItem(STORAGE_KEY);
            if (raw) {
                const parsed = JSON.parse(raw);
                if (parsed.version === STORAGE_VERSION) {
                    this._data = parsed;
                    log(`📂 Session restaurée: ${this._data.sessionCount} sessions, ` +
                        `best coverage=${(this._data.bestCoverage * 100).toFixed(1)}%, ` +
                        `curriculum=${this._data.curriculumStage}`);
                    return;
                }
            }
        } catch (e) {
            console.warn('[MARLStore] Load failed:', e.message);
        }
        this._data = this._default();
    }

    _default() {
        return {
            version: STORAGE_VERSION,
            sessionCount: 0,
            totalRounds: 0,
            totalSteps: 0,
            curriculumStage: 0,
            bestCoverage: 0,
            bestWeights: null,     // { weights: Float32Array-as-array, coverage, timestamp }
            agentWeights: null,    // Array of weight arrays (one per agent)
            history: {
                sessions: [],      // [{ date, coverage, reward, rounds, curriculum }]
            },
            lastTrainDate: null,
        };
    }

    /** Save current training state after a training run. */
    save(policies, history, curriculumStage) {
        const d = this._data;
        d.sessionCount++;
        d.totalRounds += history.rounds?.length || 0;
        d.curriculumStage = curriculumStage;
        d.lastTrainDate = new Date().toISOString();

        // Save policy weights
        d.agentWeights = policies.map(p => Array.from(p.getWeights()));

        // Track best weights
        const finalCov = history.coverage?.[history.coverage.length - 1] || 0;
        if (finalCov > d.bestCoverage) {
            d.bestCoverage = finalCov;
            // Average all agent weights for best model
            const dim = policies[0].getWeights().length;
            const avg = new Float32Array(dim);
            for (const p of policies) {
                const w = p.getWeights();
                for (let i = 0; i < dim; i++) avg[i] += w[i];
            }
            for (let i = 0; i < dim; i++) avg[i] /= policies.length;
            d.bestWeights = {
                weights: Array.from(avg),
                coverage: finalCov,
                timestamp: Date.now(),
            };
            log(`🏆 Nouveau meilleur modèle: coverage=${(finalCov * 100).toFixed(1)}%`);
        }

        // Append session summary
        const avgReward = history.rewards?.[history.rewards.length - 1] || 0;
        d.history.sessions.push({
            date: d.lastTrainDate,
            coverage: finalCov,
            reward: avgReward,
            rounds: history.rounds?.length || 0,
            curriculum: curriculumStage,
        });
        // Keep last 50 sessions
        if (d.history.sessions.length > 50) {
            d.history.sessions = d.history.sessions.slice(-50);
        }

        this._persist();
    }

    /** Load saved weights into policies (warm-start). Returns true if loaded. */
    restore(policies) {
        const d = this._data;
        if (!d.agentWeights || d.agentWeights.length === 0) {
            log('📭 Pas de poids sauvegardés — démarrage à zéro');
            return false;
        }

        const expectedDim = policies[0].getWeights().length;
        let restored = 0;

        for (let i = 0; i < policies.length; i++) {
            const saved = d.agentWeights[i];
            if (saved && saved.length === expectedDim) {
                policies[i].setWeights(new Float32Array(saved));
                restored++;
            } else if (d.bestWeights && d.bestWeights.weights.length === expectedDim) {
                // Fallback to best weights
                policies[i].setWeights(new Float32Array(d.bestWeights.weights));
                restored++;
            }
        }

        if (restored > 0) {
            log(`♻️ Warm-start: ${restored}/${policies.length} agents restaurés ` +
                `(session #${d.sessionCount}, best=${(d.bestCoverage * 100).toFixed(1)}%)`);
            return true;
        }
        return false;
    }

    /** Get recommended curriculum stage based on training history. */
    getRecommendedCurriculum() {
        const d = this._data;
        // Auto-advance curriculum when consistently achieving >70% coverage
        const recent = d.history.sessions.slice(-5);
        if (recent.length >= 3) {
            const avgCov = recent.reduce((s, r) => s + r.coverage, 0) / recent.length;
            if (avgCov > 0.70 && d.curriculumStage < CURRICULUM.length - 1) {
                return d.curriculumStage + 1;
            }
            if (avgCov < 0.30 && d.curriculumStage > 0) {
                return d.curriculumStage - 1;
            }
        }
        return d.curriculumStage;
    }

    /** Get session stats for UI display. */
    getStats() {
        const d = this._data;
        return {
            sessionCount: d.sessionCount,
            totalRounds: d.totalRounds,
            curriculumStage: d.curriculumStage,
            curriculumName: CURRICULUM[d.curriculumStage]?.name || 'Unknown',
            bestCoverage: d.bestCoverage,
            lastTrainDate: d.lastTrainDate,
            hasWeights: !!d.agentWeights,
            sessionsHistory: d.history.sessions.slice(-10),
        };
    }

    /** Clear all saved data. */
    clear() {
        this._data = this._default();
        this._persist();
        log('🗑️ Session store vidé');
    }

    _persist() {
        try {
            if (typeof localStorage === 'undefined') return;
            const json = JSON.stringify(this._data);
            const sizeMB = json.length / (1024 * 1024);
            if (sizeMB < 4) {
                localStorage.setItem(STORAGE_KEY, json);
                log(`💾 Sauvegardé (${(sizeMB * 1024).toFixed(0)}KB)`);
            } else {
                console.warn(`[MARLStore] Trop gros (${sizeMB.toFixed(1)}MB) — trim des anciennes sessions`);
                this._data.history.sessions = this._data.history.sessions.slice(-10);
                const trimmed = JSON.stringify(this._data);
                if (trimmed.length / (1024 * 1024) < 4) {
                    localStorage.setItem(STORAGE_KEY, trimmed);
                }
            }
        } catch (e) {
            console.warn('[MARLStore] Save failed:', e.message);
        }
    }
}
