/**
 * brain-interface.js — Abstract brain interface + Model Registry for MLOps.
 *
 * Provides:
 *   1. BrainInterface — Abstract class defining the contract for any agent brain.
 *      Enables hot-swapping between Q-learning, neural networks, rule-based, etc.
 *
 *   2. ModelRegistry — Central registry for brain weights/snapshots.
 *      Supports versioning, rollback, A/B testing, and experiment tracking.
 *
 * @module brain-interface
 */

// ─── BRAIN INTERFACE ─────────────────────────────────────────────────

/**
 * Abstract base class for all agent brains.
 * Any brain implementation must provide these methods.
 *
 * Built-in implementations:
 *   - AgentBrain (tile-coded Q-learning)
 *   - Future: ONNXBrain (ONNX.js inference), RuleBrain (expert rules)
 */
export class BrainInterface {
    /**
     * @param {string} agentId - Unique identifier
     * @param {string} brainType - Type identifier (e.g. 'q-learning', 'onnx', 'rule')
     */
    constructor(agentId, brainType = 'abstract') {
        if (new.target === BrainInterface) {
            throw new Error('BrainInterface is abstract — use a concrete implementation');
        }
        this.agentId = agentId;
        this.brainType = brainType;
        this.trainingEnabled = true;
        this._createdAt = Date.now();
    }

    /**
     * Select an action given an observation (policy).
     * @param {Float32Array|number[]} observation - Observation vector
     * @param {Float32Array} [actionMask] - Optional mask (0 = disabled)
     * @returns {{ action: number, qValues: Float32Array|null, explore: boolean }}
     */
    chooseAction(observation, actionMask) {
        throw new Error('chooseAction() must be implemented');
    }

    /**
     * Learn from a transition (obs, action, reward, nextObs, done).
     * No-op for inference-only brains.
     * @param {Float32Array} obs
     * @param {number} action
     * @param {number} reward
     * @param {Float32Array} nextObs
     * @param {boolean} done
     */
    learn(obs, action, reward, nextObs, done) {
        // Default: no-op (inference-only brains don't learn)
    }

    /**
     * Export weights/parameters to a JSON-serializable object.
     * Must include { version, brainType, agentId, timestamp, weights/params }.
     * @returns {Object}
     */
    exportWeights() {
        throw new Error('exportWeights() must be implemented');
    }

    /**
     * Import weights/parameters from a previously exported object.
     * @param {Object} data - Previously exported brain data
     * @returns {boolean} success
     */
    importWeights(data) {
        throw new Error('importWeights() must be implemented');
    }

    /**
     * Get current training/inference statistics.
     * @returns {Object} - { totalSteps, avgReward, epsilon, ... }
     */
    getStats() {
        return {
            agentId: this.agentId,
            brainType: this.brainType,
            trainingEnabled: this.trainingEnabled
        };
    }

    /**
     * Get the brain type identifier.
     * @returns {string}
     */
    getType() {
        return this.brainType;
    }

    /**
     * Get total number of parameters (for memory estimation).
     * @returns {number}
     */
    getTotalParams() {
        return 0;
    }

    /**
     * Reset the brain (zero weights, keep architecture).
     */
    reset() {
        throw new Error('reset() must be implemented');
    }

    /**
     * Export full state (weights + replay buffer + stats).
     * @returns {Object}
     */
    exportFull() {
        return this.exportWeights();
    }

    /**
     * Import full state.
     * @param {Object} data
     */
    importFull(data) {
        this.importWeights(data);
    }
}


// ─── MODEL REGISTRY ──────────────────────────────────────────────────

/**
 * Central registry for model weights, snapshots, and experiments.
 *
 * Features:
 *   - Named model storage with versioning
 *   - Auto-snapshots every N episodes
 *   - Rollback to any previous snapshot
 *   - A/B test configuration (assign different models to different agents)
 *   - localStorage persistence (browser-native, no server)
 *   - Import/export for sharing models across sessions
 */
export class ModelRegistry {
    /**
     * @param {Object} config
     * @param {number} config.maxSnapshots - Max snapshots to keep (default 20)
     * @param {number} config.autoSnapshotInterval - Auto-snapshot every N episodes (0 = disabled)
     * @param {string} config.storageKey - localStorage key prefix
     */
    constructor(config = {}) {
        this.maxSnapshots = config.maxSnapshots ?? 20;
        this.autoSnapshotInterval = config.autoSnapshotInterval ?? 10;
        this.storageKey = config.storageKey ?? 'diamants_model_registry';

        /**
         * Named models: name → { weights, metadata }
         * @type {Map<string, Object>}
         */
        this.models = new Map();

        /**
         * Chronological snapshots: [{ id, timestamp, episodeCount, weights, metadata }]
         * @type {Array<Object>}
         */
        this.snapshots = [];

        /**
         * A/B test configurations: testName → { groupA: modelName, groupB: modelName, assignments: Map }
         * @type {Map<string, Object>}
         */
        this.abTests = new Map();

        /**
         * Episode counter for auto-snapshot
         */
        this._episodesSinceSnapshot = 0;

        // Load from localStorage if available
        this._loadFromStorage();
    }

    // ────────────────────────────────────────────────────────────────
    // MODEL CRUD
    // ────────────────────────────────────────────────────────────────

    /**
     * Register a named model.
     * @param {string} name - Model name (e.g. 'baseline-v1', 'cooperative-v2')
     * @param {Object} weights - Exported weights from exportAllWeights()
     * @param {Object} [metadata] - Extra info (description, hyperparams, etc.)
     */
    registerModel(name, weights, metadata = {}) {
        const entry = {
            name,
            weights: JSON.parse(JSON.stringify(weights)), // deep clone
            metadata: {
                ...metadata,
                registeredAt: Date.now(),
                version: (this.models.get(name)?.metadata?.version || 0) + 1
            }
        };
        this.models.set(name, entry);
        this._saveToStorage();
        console.log(`[ModelRegistry] Registered model "${name}" v${entry.metadata.version}`);
        return entry;
    }

    /**
     * Get a named model.
     * @param {string} name
     * @returns {Object|null}
     */
    getModel(name) {
        return this.models.get(name) || null;
    }

    /**
     * List all registered models.
     * @returns {Array<{name, version, registeredAt, description}>}
     */
    listModels() {
        const list = [];
        for (const [name, entry] of this.models) {
            list.push({
                name,
                version: entry.metadata.version,
                registeredAt: entry.metadata.registeredAt,
                description: entry.metadata.description || '',
                agentCount: entry.weights?.agents
                    ? Object.keys(entry.weights.agents).length
                    : 0
            });
        }
        return list;
    }

    /**
     * Delete a named model.
     * @param {string} name
     * @returns {boolean}
     */
    deleteModel(name) {
        const deleted = this.models.delete(name);
        if (deleted) this._saveToStorage();
        return deleted;
    }

    // ────────────────────────────────────────────────────────────────
    // SNAPSHOTS
    // ────────────────────────────────────────────────────────────────

    /**
     * Create a snapshot of all agent weights.
     * @param {Object} weights - From coordinator.exportAllWeights()
     * @param {Object} [metadata] - Episode count, coverage, etc.
     * @returns {Object} The created snapshot
     */
    createSnapshot(weights, metadata = {}) {
        const snapshot = {
            id: this.snapshots.length,
            timestamp: Date.now(),
            episodeCount: metadata.episodeCount || 0,
            globalSteps: metadata.globalSteps || 0,
            coverage: metadata.coverage || 0,
            weights: JSON.parse(JSON.stringify(weights)),
            metadata
        };

        this.snapshots.push(snapshot);

        // Evict oldest if over limit
        while (this.snapshots.length > this.maxSnapshots) {
            this.snapshots.shift();
            // Re-index IDs
            for (let i = 0; i < this.snapshots.length; i++) {
                this.snapshots[i].id = i;
            }
        }

        this._saveToStorage();
        console.log(`[ModelRegistry] Snapshot #${snapshot.id} created (ep=${metadata.episodeCount})`);
        return snapshot;
    }

    /**
     * Notify the registry that an episode ended (for auto-snapshot).
     * @param {Object} weights - Current weights
     * @param {Object} metadata - Episode info
     * @returns {Object|null} Snapshot if one was created
     */
    onEpisodeEnd(weights, metadata) {
        this._episodesSinceSnapshot++;
        if (this.autoSnapshotInterval > 0 &&
            this._episodesSinceSnapshot >= this.autoSnapshotInterval) {
            this._episodesSinceSnapshot = 0;
            return this.createSnapshot(weights, metadata);
        }
        return null;
    }

    /**
     * Get a snapshot by index.
     * @param {number} index
     * @returns {Object|null}
     */
    getSnapshot(index) {
        return this.snapshots[index] || null;
    }

    /**
     * List all snapshots (lightweight — without weights).
     * @returns {Array<Object>}
     */
    listSnapshots() {
        return this.snapshots.map(s => ({
            id: s.id,
            timestamp: s.timestamp,
            episodeCount: s.episodeCount,
            globalSteps: s.globalSteps,
            coverage: s.coverage,
            date: new Date(s.timestamp).toISOString()
        }));
    }

    /**
     * Rollback to a specific snapshot. Returns the weights to apply.
     * @param {number} snapshotIndex
     * @returns {Object|null} Weights to pass to coordinator.importAllWeights()
     */
    rollback(snapshotIndex) {
        const snapshot = this.snapshots[snapshotIndex];
        if (!snapshot) {
            console.warn(`[ModelRegistry] Snapshot #${snapshotIndex} not found`);
            return null;
        }
        console.log(`[ModelRegistry] Rolling back to snapshot #${snapshotIndex} ` +
            `(ep=${snapshot.episodeCount}, ${new Date(snapshot.timestamp).toISOString()})`);
        return JSON.parse(JSON.stringify(snapshot.weights));
    }

    // ────────────────────────────────────────────────────────────────
    // A/B TESTING
    // ────────────────────────────────────────────────────────────────

    /**
     * Create an A/B test between two named models.
     * @param {string} testName - Test identifier
     * @param {string} modelA - Name of model A
     * @param {string} modelB - Name of model B
     * @param {number} splitRatio - Fraction of agents assigned to A (default 0.5)
     * @returns {Object} Test configuration
     */
    createABTest(testName, modelA, modelB, splitRatio = 0.5) {
        if (!this.models.has(modelA) || !this.models.has(modelB)) {
            console.warn(`[ModelRegistry] A/B test: one or both models not found`);
            return null;
        }

        const test = {
            name: testName,
            modelA,
            modelB,
            splitRatio,
            assignments: new Map(),  // droneId → 'A' | 'B'
            results: { A: [], B: [] },
            createdAt: Date.now(),
            active: true
        };

        this.abTests.set(testName, test);
        console.log(`[ModelRegistry] A/B test "${testName}" created: ${modelA} vs ${modelB} (${splitRatio * 100}/${(1 - splitRatio) * 100})`);
        return test;
    }

    /**
     * Assign agents to A/B test groups.
     * @param {string} testName
     * @param {string[]} droneIds - All drone IDs
     * @returns {Map<string, string>} droneId → 'A' | 'B'
     */
    assignABGroups(testName, droneIds) {
        const test = this.abTests.get(testName);
        if (!test) return new Map();

        const shuffled = [...droneIds].sort(() => Math.random() - 0.5);
        const splitIdx = Math.round(shuffled.length * test.splitRatio);

        for (let i = 0; i < shuffled.length; i++) {
            const group = i < splitIdx ? 'A' : 'B';
            test.assignments.set(shuffled[i], group);
        }

        return test.assignments;
    }

    /**
     * Get the model weights for a specific agent in an A/B test.
     * @param {string} testName
     * @param {string} droneId
     * @returns {Object|null} Weights for this agent
     */
    getABWeights(testName, droneId) {
        const test = this.abTests.get(testName);
        if (!test) return null;

        const group = test.assignments.get(droneId);
        if (!group) return null;

        const modelName = group === 'A' ? test.modelA : test.modelB;
        const model = this.models.get(modelName);
        return model?.weights?.agents?.[droneId] || null;
    }

    /**
     * Record episode results for A/B test analysis.
     * @param {string} testName
     * @param {Object} episodeMetrics - { droneId → { reward, coverage, steps } }
     */
    recordABResults(testName, episodeMetrics) {
        const test = this.abTests.get(testName);
        if (!test || !test.active) return;

        const resultA = { timestamp: Date.now(), metrics: {} };
        const resultB = { timestamp: Date.now(), metrics: {} };

        for (const [droneId, metrics] of Object.entries(episodeMetrics)) {
            const group = test.assignments.get(droneId);
            if (group === 'A') resultA.metrics[droneId] = metrics;
            else if (group === 'B') resultB.metrics[droneId] = metrics;
        }

        test.results.A.push(resultA);
        test.results.B.push(resultB);
    }

    /**
     * Get A/B test summary statistics.
     * @param {string} testName
     * @returns {Object|null}
     */
    getABSummary(testName) {
        const test = this.abTests.get(testName);
        if (!test) return null;

        const summarize = (results) => {
            if (results.length === 0) return { episodes: 0, avgReward: 0, avgCoverage: 0 };
            let totalReward = 0, totalCoverage = 0, count = 0;
            for (const ep of results) {
                for (const m of Object.values(ep.metrics)) {
                    totalReward += m.reward || 0;
                    totalCoverage += m.coverage || 0;
                    count++;
                }
            }
            return {
                episodes: results.length,
                avgReward: count > 0 ? totalReward / count : 0,
                avgCoverage: count > 0 ? totalCoverage / count : 0,
                sampleSize: count
            };
        };

        return {
            testName,
            modelA: test.modelA,
            modelB: test.modelB,
            groupA: summarize(test.results.A),
            groupB: summarize(test.results.B),
            active: test.active,
            createdAt: test.createdAt
        };
    }

    /**
     * End an A/B test.
     * @param {string} testName
     * @returns {Object|null} Final summary
     */
    endABTest(testName) {
        const test = this.abTests.get(testName);
        if (!test) return null;
        test.active = false;
        return this.getABSummary(testName);
    }

    // ────────────────────────────────────────────────────────────────
    // PERSISTENCE (localStorage)
    // ────────────────────────────────────────────────────────────────

    _saveToStorage() {
        try {
            if (typeof localStorage === 'undefined') return;

            const data = {
                models: Object.fromEntries(
                    [...this.models].map(([k, v]) => [k, {
                        ...v,
                        // Truncate weights to keep localStorage manageable
                        weights: v.weights
                    }])
                ),
                snapshots: this.snapshots,
                savedAt: Date.now()
            };

            const json = JSON.stringify(data);
            const sizeMB = json.length / (1024 * 1024);

            // Only persist if under 5MB (localStorage typical limit)
            if (sizeMB < 5) {
                localStorage.setItem(this.storageKey, json);
            } else {
                console.warn(`[ModelRegistry] Storage too large (${sizeMB.toFixed(1)}MB), skipping save`);
            }
        } catch (e) {
            // localStorage unavailable (Node.js, quota exceeded, etc.)
        }
    }

    _loadFromStorage() {
        try {
            if (typeof localStorage === 'undefined') return;

            const raw = localStorage.getItem(this.storageKey);
            if (!raw) return;

            const data = JSON.parse(raw);

            // Restore models
            if (data.models) {
                for (const [name, entry] of Object.entries(data.models)) {
                    this.models.set(name, entry);
                }
            }

            // Restore snapshots
            if (data.snapshots) {
                this.snapshots = data.snapshots;
            }

            console.log(`[ModelRegistry] Loaded ${this.models.size} models, ${this.snapshots.length} snapshots from storage`);
        } catch (e) {
            console.warn('[ModelRegistry] Failed to load from storage:', e.message);
        }
    }

    /**
     * Export the entire registry as a JSON object (for file download).
     * @returns {Object}
     */
    exportRegistry() {
        return {
            version: 1,
            exportedAt: Date.now(),
            models: Object.fromEntries(this.models),
            snapshots: this.snapshots,
            abTests: Object.fromEntries(
                [...this.abTests].map(([k, v]) => [k, {
                    ...v,
                    assignments: Object.fromEntries(v.assignments)
                }])
            )
        };
    }

    /**
     * Import a previously exported registry.
     * @param {Object} data
     * @param {boolean} merge - If true, merge with existing. If false, replace.
     */
    importRegistry(data, merge = true) {
        if (!data || data.version !== 1) {
            console.warn('[ModelRegistry] Invalid registry format');
            return;
        }

        if (!merge) {
            this.models.clear();
            this.snapshots = [];
            this.abTests.clear();
        }

        if (data.models) {
            for (const [name, entry] of Object.entries(data.models)) {
                this.models.set(name, entry);
            }
        }

        if (data.snapshots) {
            if (merge) {
                this.snapshots.push(...data.snapshots);
            } else {
                this.snapshots = data.snapshots;
            }
        }

        this._saveToStorage();
        console.log(`[ModelRegistry] Imported registry (${this.models.size} models, ${this.snapshots.length} snapshots)`);
    }

    /**
     * Clear all data.
     */
    clear() {
        this.models.clear();
        this.snapshots = [];
        this.abTests.clear();
        this._episodesSinceSnapshot = 0;
        try {
            if (typeof localStorage !== 'undefined') {
                localStorage.removeItem(this.storageKey);
            }
        } catch (e) { /* ignore */ }
    }
}
