/**
 * DIAMANTS — SLAM Forest Reconstruction Minimap
 * ═══════════════════════════════════════════════════
 * Reconstruction visuelle de la forêt via SLAM collaboratif.
 * Chaque drone scanne l'environnement avec ses capteurs Multi-Ranger (ToF).
 * Les observations de tous les drones sont fusionnées par consensus.
 *
 * Rendu:
 *   - Sol de forêt (cellules FREE) → terrain vert-brun naturel
 *   - Arbres détectés (clusters OCCUPIED) → canopées vues du dessus
 *   - Zones inconnues → brouillard sombre
 *   - Frontière d'exploration → halo de découverte
 *
 * Pipeline SLAM (identique au backend map_merger_node.py):
 *   1. Chaque drone trace des rayons dans 4 directions horizontales
 *   2. Cellules le long du rayon → FREE (sol dégagé)
 *   3. Cellule finale si hit → OCCUPIED (obstacle = arbre)
 *   4. Clustering des cellules OCCUPIED → arbres individuels
 *   5. Rendu canopée avec gradient radial + ombre
 *   6. Résultat final: vue aérienne de la forêt reconstruite
 */

// ─── Cell States (like backend CellValue) ────────────────────────────
const CellState = {
    UNKNOWN:  0,
    FREE:     1,
    OCCUPIED: 2,
};

// ─── Tree validation states (from backend ZoneState / GoldenZoneManager) ──
const TreeState = {
    CANDIDATE: 0,   // Détecté par 1 drone — semi-transparent
    CONSENSUS: 1,   // Confirmé par 2+ drones — rendu normal
    VALIDATED: 2,   // Stable 10+ secondes — permanent, ne décay plus
};

// ─── Defaults ────────────────────────────────────────────────────────
const PERCEPTION_CONFIG = {
    gridSize:     120,    // cells per side (120×120 = one cell per meter for 120m zone)
    resolution:   1.0,    // meters per cell
    maxSensorRange: 4.0,  // ToF max range (meters)

    // Confidence increments (inspired by backend pheromone_inc / consensus)
    freeIncrement:     3,   // confidence gained per FREE observation
    occupiedIncrement: 15,  // confidence gained per OCCUPIED observation
    maxConfidence:     100,

    // Temporal decay (evaporation like backend)
    decayRate:   0.15,     // confidence lost per second for unrefreshed cells
    decayInterval: 2000,   // ms between decay passes

    // ── Backend algo: cluster filtering (anti-noise) ──
    minClusterSize: 2,     // reject OCCUPIED clusters < N cells (backend uses 4-5)

    // ── Backend algo: consensus multi-drone ──
    consensusMinVotes: 2,  // need 2+ drone observations for confirmed tree

    // ── Backend algo: temporal smoothing (anti-flickering) ──
    smoothingFrames: 4,    // ring buffer of N confidence snapshots (backend uses 5)

    // ── Backend algo: golden zones (tree persistence) ──
    goldenValidationMs: 10000,  // 10s stable → tree becomes permanent
    goldenMatchRadius: 3.0,     // cells — max distance to match tree to golden zone

    // Rendering
    renderInterval: 100,   // ms between renders (~10 FPS)
    showFOV: true,         // show drone sensor FOV cones
    showGrid: true,        // show reference grid lines
};

// ─── Drone colors ────────────────────────────────────────────────────
const DRONE_COLORS = [
    '#00FF88', '#00C8FF', '#FFAA00', '#FF6496',
    '#9664FF', '#FFFF00', '#64FFC8', '#FF9664',
];
/** Extract numeric index from drone ID (string or number). */
function _droneIdx(id) {
    if (typeof id === 'number') return id;
    const m = String(id).match(/(\d+)/);
    return m ? parseInt(m[1], 10) - 1 : 0;  // "crazyflie_01" → 0
}
export class PerceptionMinimap {
    /**
     * @param {string} canvasId - ID of the <canvas> element
     * @param {Object} [config] - Override config
     */
    constructor(canvasId = 'perception_canvas', config = {}) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas?.getContext('2d');

        this.cfg = { ...PERCEPTION_CONFIG, ...config };
        const { gridSize, maxConfidence } = this.cfg;

        // ── Occupancy grid ─────────────────────────────────────────
        this.state      = new Uint8Array(gridSize * gridSize);       // CellState
        this.confidence  = new Float32Array(gridSize * gridSize);     // 0..maxConfidence
        this.lastSeen    = new Float32Array(gridSize * gridSize);     // timestamp (ms)
        this.droneVotes  = new Uint8Array(gridSize * gridSize);       // # drones that observed

        this.halfWorld = (gridSize * this.cfg.resolution) / 2;

        // ── Adaptive zoom ──────────────────────────────────────────
        this._adaptiveHalf = 20;        // current display half-extent (smoothed)

        // ── Permanent coverage mask (synced from engine visitedCells, immune to decay) ──
        this._permanentFree = new Uint8Array(gridSize * gridSize); // 1 = permanently explored

        // ── Drone tracking ─────────────────────────────────────────
        /** @type {Map<number, {x:number, z:number, heading:number, ranges:number[]}>} */
        this.droneData = new Map();

        // ── Rendering ──────────────────────────────────────────────
        this._lastRender  = 0;
        this._lastDecay   = 0;
        this._frameCount  = 0;

        // ── Stats ──────────────────────────────────────────────────
        this.stats = {
            freeCells: 0,
            occupiedCells: 0,
            coverage: 0,
            lastUpdate: 0,
        };

        // ── ImageData buffer for fast pixel rendering ──────────────
        if (this.canvas) {
            this._imgData = this.ctx.createImageData(this.canvas.width, this.canvas.height);
        }

        // ── Clustering buffer (pre-allocated for GC-free clustering) ──
        this._clusterVisited = new Uint8Array(gridSize * gridSize);
        this._lastTreeCount = 0;

        // ── Backend algo: temporal smoothing ring buffer ──────────
        this._smoothingBuffer = [];  // Array of Float32Array snapshots
        this._smoothedConfidence = new Float32Array(gridSize * gridSize);

        // ── Backend algo: golden zones (persistent validated trees) ──
        // Key = "gx,gz" (centroid rounded), Value = {state, firstSeen, lastSeen,
        //   maxVotes, centroidX, centroidZ, size, confidence, species}
        /** @type {Map<string, Object>} */
        this._goldenTrees = new Map();

        // ── Per-cell drone tracker (which drones saw each cell) ────
        // Key = cell index, Value = Set of droneIds
        /** @type {Map<number, Set<number>>} */
        this._cellDroneSet = new Map();

        // ── Stigmergy pheromone data (transient, like real pheromones) ──
        this._dangerMarkers = [];    // obstacles <6m (red)
        this._interestMarkers = [];  // obstacles 6–15m (yellow)
        this._pSnapInterval = 200;
        this._lastPSnapTime = 0;

        this._init();
    }

    _init() {
        if (!this.canvas) {
            console.warn('⚠️ Perception minimap canvas not found');
            return;
        }

        // Start render loop
        this._raf = null;
        this._startRenderLoop();
        this.canvas._minimapInstance = this;

        // Expose globally
        window.DIAMANTS_PERCEPTION = this;

        console.log('🌲 SLAM Forest Reconstruction initialisée —',
            `${this.cfg.gridSize}×${this.cfg.gridSize} grid, ${this.cfg.resolution}m/cell`);
    }

    // ════════════════════════════════════════════════════════════════
    // PUBLIC API
    // ════════════════════════════════════════════════════════════════

    /**
     * Feed sensor data from one drone into the occupancy grid.
     * Called every frame from main.js animate() loop.
     *
     * @param {number} droneId
     * @param {{x:number, y:number, z:number}} position
     * @param {number} heading - Yaw in radians
     * @param {Float32Array|number[]} sensorRanges - [front, back, left, right, up]
     */
    updateFromSensor(droneId, position, heading, sensorRanges) {
        if (!sensorRanges || sensorRanges.length < 4) return;

        const now = performance.now();

        // Store drone state for rendering
        this.droneData.set(droneId, {
            x: position.x,
            z: position.z,
            heading,
            ranges: sensorRanges,
        });

        // ── Trace 4 horizontal rays (front, back, left, right) ──
        const maxR = this.cfg.maxSensorRange;
        const directions = [
            heading,                    // front
            heading + Math.PI,          // back
            heading - Math.PI / 2,      // left
            heading + Math.PI / 2,      // right
        ];

        for (let d = 0; d < 4; d++) {
            const range = sensorRanges[d];
            const hit   = range < maxR * 0.98; // small margin for noise
            const rayLen = hit ? range : maxR;
            this._traceRay(position.x, position.z, directions[d], rayLen, hit, droneId, now);
        }

        this.stats.lastUpdate = now;
    }

    /**
     * Reset the entire perception grid.
     */
    reset() {
        this.state.fill(CellState.UNKNOWN);
        this.confidence.fill(0);
        this.lastSeen.fill(0);
        this.droneVotes.fill(0);
        this.droneData.clear();
        this._cellDroneSet.clear();
        this._goldenTrees.clear();
        this._smoothingBuffer.length = 0;
        this._smoothedConfidence.fill(0);
        console.log('🌲 Forest reconstruction grid reset (golden trees cleared)');
    }

    /**
     * Get current perception stats.
     */
    getStats() {
        return { ...this.stats };
    }

    // ════════════════════════════════════════════════════════════════
    // RAY TRACING (2D Bresenham-like)
    // ════════════════════════════════════════════════════════════════

    /**
     * Trace a ray from (startX, startZ) at angle for `length` meters.
     * Mark cells along the ray as FREE, and the endpoint as OCCUPIED if `hit`.
     */
    _traceRay(startX, startZ, angle, length, hit, droneId, now) {
        const res = this.cfg.resolution;
        const steps = Math.ceil(length / (res * 0.5)); // sub-cell resolution for accuracy
        const stepSize = length / steps;
        const dx = Math.sin(angle) * stepSize;  // THREE.js: sin for X from heading
        const dz = Math.cos(angle) * stepSize;  // cos for Z from heading

        let x = startX;
        let z = startZ;
        let prevGx = -1, prevGz = -1;

        for (let i = 0; i <= steps; i++) {
            const gx = this._worldToGridX(x);
            const gz = this._worldToGridZ(z);

            // Skip duplicate grid cells (sub-cell steps)
            if (gx === prevGx && gz === prevGz) {
                x += dx;
                z += dz;
                continue;
            }
            prevGx = gx;
            prevGz = gz;

            if (gx < 0 || gx >= this.cfg.gridSize || gz < 0 || gz >= this.cfg.gridSize) break;

            const idx = gz * this.cfg.gridSize + gx;

            if (i === steps && hit) {
                // ── Endpoint: OCCUPIED ──
                this.state[idx] = CellState.OCCUPIED;
                this.confidence[idx] = Math.min(
                    this.cfg.maxConfidence,
                    this.confidence[idx] + this.cfg.occupiedIncrement
                );
            } else {
                // ── Along ray: FREE ──
                // Don't overwrite a high-confidence OCCUPIED cell
                if (this.state[idx] === CellState.OCCUPIED && this.confidence[idx] > 40) {
                    // But reduce confidence slightly — free observation contests occupancy
                    this.confidence[idx] = Math.max(0, this.confidence[idx] - 2);
                    if (this.confidence[idx] < 10) {
                        this.state[idx] = CellState.FREE;
                    }
                } else {
                    this.state[idx] = CellState.FREE;
                    this.confidence[idx] = Math.min(
                        this.cfg.maxConfidence,
                        this.confidence[idx] + this.cfg.freeIncrement
                    );
                }
            }

            this.lastSeen[idx] = now;
            // ── Backend algo: track unique drones per cell ──
            if (!this._cellDroneSet.has(idx)) {
                this._cellDroneSet.set(idx, new Set());
            }
            this._cellDroneSet.get(idx).add(droneId);
            this.droneVotes[idx] = this._cellDroneSet.get(idx).size;

            x += dx;
            z += dz;
        }
    }

    // ════════════════════════════════════════════════════════════════
    // TEMPORAL DECAY (evaporation like backend)
    // ════════════════════════════════════════════════════════════════

    _applyDecay(now) {
        const elapsed = (now - this._lastDecay) / 1000; // seconds
        if (elapsed < this.cfg.decayInterval / 1000) return;
        this._lastDecay = now;

        const decay = this.cfg.decayRate * elapsed;
        const gs = this.cfg.gridSize;
        const size = gs * gs;

        // ── Backend algo: build set of protected cells near validated golden trees ──
        const protectedCells = this._getProtectedCells();

        for (let i = 0; i < size; i++) {
            if (this.state[i] === CellState.UNKNOWN) continue;

            // ── Golden zone protection: validated trees never decay ──
            if (protectedCells.has(i)) continue;

            // Only decay cells not recently seen AND not permanently marked
            const age = now - this.lastSeen[i];
            if (this._permanentFree[i]) continue; // Visited cells NEVER decay
            if (age > 5000) { // 5 seconds without refresh → start decaying
                this.confidence[i] -= decay;
                if (this.confidence[i] <= 0) {
                    this.confidence[i] = 0;
                    this.state[i] = CellState.UNKNOWN;
                    this._cellDroneSet.delete(i);
                    this.droneVotes[i] = 0;
                }
            }
        }
    }

    /**
     * Build set of cell indices protected by validated golden trees.
     * Cells within goldenMatchRadius of a VALIDATED tree centroid are immune to decay.
     * @returns {Set<number>}
     */
    _getProtectedCells() {
        const protected_ = new Set();
        const gs = this.cfg.gridSize;
        const radius = Math.ceil(this.cfg.goldenMatchRadius);

        for (const tree of this._goldenTrees.values()) {
            if (tree.state !== TreeState.VALIDATED) continue;
            const cx = Math.round(tree.centroidX);
            const cz = Math.round(tree.centroidZ);
            for (let dz = -radius; dz <= radius; dz++) {
                for (let dx = -radius; dx <= radius; dx++) {
                    const gx = cx + dx, gz = cz + dz;
                    if (gx >= 0 && gx < gs && gz >= 0 && gz < gs) {
                        protected_.add(gz * gs + gx);
                    }
                }
            }
        }
        return protected_;
    }

    // ════════════════════════════════════════════════════════════════
    // GRID HELPERS
    // ════════════════════════════════════════════════════════════════

    _worldToGridX(worldX) {
        return Math.floor((worldX + this.halfWorld) / this.cfg.resolution);
    }

    _worldToGridZ(worldZ) {
        return Math.floor((worldZ + this.halfWorld) / this.cfg.resolution);
    }

    _gridToWorldX(gx) {
        return (gx + 0.5) * this.cfg.resolution - this.halfWorld;
    }

    _gridToWorldZ(gz) {
        return (gz + 0.5) * this.cfg.resolution - this.halfWorld;
    }

    // ════════════════════════════════════════════════════════════════
    // STATS
    // ════════════════════════════════════════════════════════════════

    _updateStats() {
        let free = 0, occupied = 0;
        const total = this.cfg.gridSize * this.cfg.gridSize;
        for (let i = 0; i < total; i++) {
            if (this.state[i] === CellState.FREE) free++;
            else if (this.state[i] === CellState.OCCUPIED) occupied++;
        }
        this.stats.freeCells = free;
        this.stats.occupiedCells = occupied;
        this.stats.coverage = ((free + occupied) / total * 100);
    }

    /**
     * Sync coverage from engine's visitedCells — any cell visited by a drone
     * is guaranteed to be at least FREE (no obstacle at drone position).
     * This fills gaps where ToF rays didn't reach.
     * Synced cells get a high confidence + recent timestamp to resist decay.
     */
    _syncFromVisitedCells() {
        const visitedCells = window.DIAMANTS_VISITED_CELLS;
        const cellSize = window.DIAMANTS_CELL_SIZE || 2;
        if (!visitedCells || visitedCells.size === 0) return;

        const gs = this.cfg.gridSize;
        const now = performance.now();
        const res = this.cfg.resolution;
        // Engine cells are cellSize m wide; SLAM cells are res m wide.
        // Each engine cell covers (cellSize/res)² SLAM cells.
        const span = Math.ceil(cellSize / res); // = 2 for cellSize=2, res=1
        const margin = 1; // Extra 1-cell margin to fill gaps between engine cells

        for (const key of visitedCells) {
            const [cx, cz] = key.split(',').map(Number);
            // World origin of this engine cell
            const wx0 = cx * cellSize;
            const wz0 = cz * cellSize;
            // Mark all SLAM cells that overlap this engine cell + margin
            for (let sx = -margin; sx < span + margin; sx++) {
                for (let sz = -margin; sz < span + margin; sz++) {
                    const gx = this._worldToGridX(wx0 + (sx + 0.5) * res);
                    const gz = this._worldToGridZ(wz0 + (sz + 0.5) * res);
                    if (gx < 0 || gx >= gs || gz < 0 || gz >= gs) continue;
                    const idx = gz * gs + gx;
                    // Mark as permanently explored (immune to decay)
                    this._permanentFree[idx] = 1;
                    this.lastSeen[idx] = now;
                    if (this.state[idx] === CellState.UNKNOWN) {
                        this.state[idx] = CellState.FREE;
                    }
                    // High confidence for bright rendering (no visual shadow)
                    this.confidence[idx] = Math.max(this.confidence[idx], 70);
                }
            }
        }
    }

    // ════════════════════════════════════════════════════════════════
    // PHEROMONE DATA SYNC
    // ════════════════════════════════════════════════════════════════

    _syncPheromones(now) {
        if (now - this._lastPSnapTime < this._pSnapInterval) return;
        this._lastPSnapTime = now;

        const engine = window.DIAMANTS_STIGMERGY_INSTANCE;
        const obstacles = engine?._obstacles || [];

        // ── Stigmergy: transient pheromone deposits ──
        const INTEREST_RANGE = 15;
        const DANGER_RANGE = 6;
        const interest = [];
        const danger = [];

        if (obstacles.length > 0 && this.droneData.size > 0) {
            for (const obs of obstacles) {
                let minDist = Infinity;
                for (const data of this.droneData.values()) {
                    const dx = data.x - obs.center.x;
                    const dz = data.z - obs.center.z;
                    const d = Math.sqrt(dx * dx + dz * dz);
                    if (d < minDist) minDist = d;
                }
                if (minDist < DANGER_RANGE) {
                    danger.push({ wx: obs.center.x, wz: obs.center.z, radius: obs.radius || 1.5, dist: minDist });
                } else if (minDist < INTEREST_RANGE) {
                    interest.push({ wx: obs.center.x, wz: obs.center.z, radius: obs.radius || 1.5, dist: minDist });
                }
            }
        }
        this._dangerMarkers = danger;
        this._interestMarkers = interest;
    }

    // ════════════════════════════════════════════════════════════════
    // RENDERING
    // ════════════════════════════════════════════════════════════════

    _startRenderLoop() {
        const loop = () => {
            this._raf = requestAnimationFrame(loop);
            const now = performance.now();
            if (now - this._lastRender < this.cfg.renderInterval) return;
            this._lastRender = now;
            this._frameCount++;

            // Periodic operations
            this._applyDecay(now);
            if (this._frameCount % 3 === 0) {
                this._syncFromVisitedCells(); // Sync coverage from engine (before stats)
            }
            if (this._frameCount % 10 === 0) {
                this._updateStats();
            }

            // ── Backend algo: temporal smoothing ──
            this._updateSmoothing();

            // ── Backend algo: golden zones update ──
            if (this._frameCount % 5 === 0) this._updateGoldenTrees(now);

            // ── Pheromone sync ──
            this._syncPheromones(now);

            this._render();
        };
        loop();
    }

    // ════════════════════════════════════════════════════════════════
    // TEMPORAL SMOOTHING (ring buffer, from backend history_weighted)
    // ════════════════════════════════════════════════════════════════

    /**
     * Push current confidence into the ring buffer, compute smoothed output.
     * Backend: deque(maxlen=smoothing_window) + np.mean()
     */
    _updateSmoothing() {
        const size = this.cfg.gridSize * this.cfg.gridSize;
        const maxFrames = this.cfg.smoothingFrames;

        // Push snapshot
        const snap = new Float32Array(size);
        snap.set(this.confidence);
        this._smoothingBuffer.push(snap);
        if (this._smoothingBuffer.length > maxFrames) {
            this._smoothingBuffer.shift();
        }

        // Average all snapshots → smoothed confidence
        const n = this._smoothingBuffer.length;
        this._smoothedConfidence.fill(0);
        for (let f = 0; f < n; f++) {
            const buf = this._smoothingBuffer[f];
            for (let i = 0; i < size; i++) {
                this._smoothedConfidence[i] += buf[i];
            }
        }
        if (n > 1) {
            for (let i = 0; i < size; i++) {
                this._smoothedConfidence[i] /= n;
            }
        }
    }

    // ════════════════════════════════════════════════════════════════
    // GOLDEN ZONES — Tree Persistence
    // (from backend GoldenZoneManager / UltraRobustGoldenZoneManager)
    //
    //   CANDIDATE  → when cluster detected, votesInCluster < consensusMinVotes
    //   CONSENSUS  → votesInCluster >= consensusMinVotes (2+ drones confirm)
    //   VALIDATED  → stable for goldenValidationMs (10s) → permanent
    // ════════════════════════════════════════════════════════════════

    /**
     * Update the golden tree registry from current OCCUPIED clusters.
     * Backend: update_from_consensus() + state transitions
     */
    _updateGoldenTrees(now) {
        const clusters = this._clusterOccupiedCells();
        const matchR = this.cfg.goldenMatchRadius;
        const matched = new Set(); // golden keys matched this frame

        for (const cluster of clusters) {
            // Find nearest existing golden tree
            let bestKey = null;
            let bestDist = Infinity;
            for (const [key, gt] of this._goldenTrees) {
                const dx = cluster.centroidX - gt.centroidX;
                const dz = cluster.centroidZ - gt.centroidZ;
                const dist = Math.sqrt(dx * dx + dz * dz);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestKey = key;
                }
            }

            if (bestKey && bestDist < matchR) {
                // ── Update existing golden tree ──
                const gt = this._goldenTrees.get(bestKey);
                gt.lastSeen = now;
                gt.centroidX = cluster.centroidX;
                gt.centroidZ = cluster.centroidZ;
                gt.size = cluster.size;
                gt.confidence = cluster.confidence;
                gt.maxVotes = Math.max(gt.maxVotes, cluster.totalVotes);

                // State transitions (like backend _update_zone_state)
                if (gt.state === TreeState.CANDIDATE && cluster.totalVotes >= this.cfg.consensusMinVotes) {
                    gt.state = TreeState.CONSENSUS;
                }
                if (gt.state === TreeState.CONSENSUS && (now - gt.firstSeen) > this.cfg.goldenValidationMs) {
                    gt.state = TreeState.VALIDATED;
                }
                matched.add(bestKey);
            } else {
                // ── Create new golden tree candidate ──
                const key = `${Math.round(cluster.centroidX)},${Math.round(cluster.centroidZ)}`;
                const state = cluster.totalVotes >= this.cfg.consensusMinVotes
                    ? TreeState.CONSENSUS
                    : TreeState.CANDIDATE;
                this._goldenTrees.set(key, {
                    state,
                    firstSeen: now,
                    lastSeen: now,
                    maxVotes: cluster.totalVotes,
                    centroidX: cluster.centroidX,
                    centroidZ: cluster.centroidZ,
                    size: cluster.size,
                    confidence: cluster.confidence,
                    species: Math.floor(cluster.centroidX * 3 + cluster.centroidZ * 7) % 4,
                });
                matched.add(key);
            }
        }

        // ── Expire old CANDIDATE trees not seen recently ──
        // (like backend _cleanup_expired_zones)
        for (const [key, gt] of this._goldenTrees) {
            if (matched.has(key)) continue;
            // VALIDATED trees never expire
            if (gt.state === TreeState.VALIDATED) continue;
            // CONSENSUS trees survive 30s without re-detection
            if (gt.state === TreeState.CONSENSUS && (now - gt.lastSeen) > 30000) {
                this._goldenTrees.delete(key);
            }
            // CANDIDATE trees survive 5s
            if (gt.state === TreeState.CANDIDATE && (now - gt.lastSeen) > 5000) {
                this._goldenTrees.delete(key);
            }
        }
    }

    /**
     * Get golden trees summary for external consumers.
     */
    getGoldenTrees() {
        const result = { candidates: 0, consensus: 0, validated: 0, trees: [] };
        for (const gt of this._goldenTrees.values()) {
            if (gt.state === TreeState.CANDIDATE) result.candidates++;
            else if (gt.state === TreeState.CONSENSUS) result.consensus++;
            else if (gt.state === TreeState.VALIDATED) result.validated++;
            result.trees.push({ ...gt });
        }
        return result;
    }

    // ════════════════════════════════════════════════════════════════
    // TREE CLUSTERING (groups OCCUPIED cells into detected trees)
    // ════════════════════════════════════════════════════════════════

    /**
     * Cluster adjacent OCCUPIED cells into individual tree detections.
     * Uses 8-connected flood-fill.
     * @returns {{centroidX:number, centroidZ:number, size:number, confidence:number}[]}
     */
    _clusterOccupiedCells() {
        const gs = this.cfg.gridSize;
        const visited = this._clusterVisited;
        visited.fill(0);  // Reset pre-allocated buffer
        const clusters = [];

        for (let gz = 0; gz < gs; gz++) {
            for (let gx = 0; gx < gs; gx++) {
                const idx = gz * gs + gx;
                if (this.state[idx] !== CellState.OCCUPIED || visited[idx]) continue;

                // BFS flood-fill (8-connected)
                const cells = [];
                const queue = [[gx, gz]];
                visited[idx] = 1;

                while (queue.length > 0) {
                    const [cx, cz] = queue.shift();
                    cells.push([cx, cz]);

                    for (let dy = -1; dy <= 1; dy++) {
                        for (let dx = -1; dx <= 1; dx++) {
                            if (dx === 0 && dy === 0) continue;
                            const nx = cx + dx, nz = cz + dy;
                            if (nx < 0 || nx >= gs || nz < 0 || nz >= gs) continue;
                            const nIdx = nz * gs + nx;
                            if (this.state[nIdx] === CellState.OCCUPIED && !visited[nIdx]) {
                                visited[nIdx] = 1;
                                queue.push([nx, nz]);
                            }
                        }
                    }
                }

                // ── Backend algo: reject small clusters (anti-noise) ──
                if (cells.length < this.cfg.minClusterSize) continue;

                // Compute centroid + max confidence + total unique drone votes
                let sumX = 0, sumZ = 0, maxConf = 0, totalVotes = 0;
                const clusterDrones = new Set();
                for (const [cx, cz] of cells) {
                    const cellIdx = cz * gs + cx;
                    sumX += cx;
                    sumZ += cz;
                    maxConf = Math.max(maxConf, this.confidence[cellIdx]);
                    // ── Backend algo: aggregate unique drones across cluster ──
                    const drones = this._cellDroneSet.get(cellIdx);
                    if (drones) {
                        for (const d of drones) clusterDrones.add(d);
                    }
                }
                totalVotes = clusterDrones.size;
                clusters.push({
                    centroidX: sumX / cells.length,
                    centroidZ: sumZ / cells.length,
                    size: cells.length,
                    confidence: maxConf / this.cfg.maxConfidence,
                    totalVotes,
                });
            }
        }
        return clusters;
    }

    // ════════════════════════════════════════════════════════════════
    // FOREST RECONSTRUCTION RENDERING
    // ════════════════════════════════════════════════════════════════

    _render() {
        if (!this.ctx || !this.canvas) return;

        const w = this.canvas.width;
        const h = this.canvas.height;
        const gs = this.cfg.gridSize;

        // ── Adaptive zoom: compute display extent from drone positions ──
        let rawExtent = 20; // minimum half-extent 20m
        this.droneData.forEach((data) => {
            const m = Math.max(Math.abs(data.x), Math.abs(data.z));
            if (m > rawExtent) rawExtent = m;
        });
        // Target = extent + 30% padding, capped at grid half-world
        const targetHalf = Math.min(this.halfWorld, Math.max(20, rawExtent * 1.3));
        // Smooth growth (only grow, never shrink)
        if (targetHalf > this._adaptiveHalf) {
            this._adaptiveHalf += (targetHalf - this._adaptiveHalf) * 0.08;
            if (this._adaptiveHalf > targetHalf - 0.5) this._adaptiveHalf = targetHalf;
        }
        const viewHalf = this._adaptiveHalf;
        const viewSize = viewHalf * 2;

        // Map world coords → pixel coords within adaptive viewport
        const worldToPixelX = (wx) => ((wx + viewHalf) / viewSize) * w;
        const worldToPixelY = (wz) => ((wz + viewHalf) / viewSize) * h;

        // Determine visible grid range
        const res = this.cfg.resolution;
        const gxMin = Math.max(0, Math.floor((-viewHalf + this.halfWorld) / res));
        const gxMax = Math.min(gs - 1, Math.floor((viewHalf + this.halfWorld) / res));
        const gzMin = Math.max(0, Math.floor((-viewHalf + this.halfWorld) / res));
        const gzMax = Math.min(gs - 1, Math.floor((viewHalf + this.halfWorld) / res));

        // Cell pixel size based on adaptive viewport
        const cellW = (res / viewSize) * w;
        const cellH = (res / viewSize) * h;

        // ── Background: dark unexplored fog ────────────────────────
        this.ctx.fillStyle = '#060E08';
        this.ctx.fillRect(0, 0, w, h);

        // ── Ground layer: explored terrain ─────────────────────────
        // Draw FREE cells as natural forest-floor terrain (top-down)
        for (let gz = gzMin; gz <= gzMax; gz++) {
            for (let gx = gxMin; gx <= gxMax; gx++) {
                const idx = gz * gs + gx;
                const st = this.state[idx];
                if (st === CellState.UNKNOWN) continue;

                const conf = this.confidence[idx] / this.cfg.maxConfidence;
                // Convert grid → world → pixel
                const worldX = (gx + 0.5) * res - this.halfWorld;
                const worldZ = (gz + 0.5) * res - this.halfWorld;
                const px = worldToPixelX(worldX) - cellW / 2;
                const py = worldToPixelY(worldZ) - cellH / 2;

                if (st === CellState.FREE) {
                    // ── Forest floor: earthy greens/browns ──
                    // Position-based pseudo-random for consistent variation
                    const seed = ((gx * 7 + gz * 13 + gx * gz) % 37) / 37;
                    const seed2 = ((gx * 11 + gz * 3) % 23) / 23;

                    // Mix grass green + earthy brown
                    const r = Math.floor(38 + seed * 25 + seed2 * 10);
                    const g = Math.floor(72 + seed * 40 + seed2 * 15);
                    const b = Math.floor(22 + seed * 15);
                    // Higher floor alpha for permanently synced cells
                    const isPermanent = this._permanentFree[idx];
                    const alpha = isPermanent
                        ? 0.55 + conf * 0.42    // synced cells: always bright (0.55–0.97)
                        : 0.35 + conf * 0.60;   // scanned cells: confidence-driven

                    this.ctx.fillStyle = `rgba(${r}, ${g}, ${b}, ${alpha})`;
                    this.ctx.fillRect(px, py, cellW + 0.5, cellH + 0.5);

                    // Subtle lighter grass patches (occasional)
                    if (seed > 0.7 && conf > 0.4) {
                        this.ctx.fillStyle = `rgba(70, 130, 50, ${conf * 0.15})`;
                        this.ctx.fillRect(px, py, cellW + 0.5, cellH + 0.5);
                    }
                } else if (st === CellState.OCCUPIED) {
                    // ── Trunk contact point: dark earth ──
                    const alpha = 0.3 + conf * 0.5;
                    this.ctx.fillStyle = `rgba(50, 35, 15, ${alpha})`;
                    this.ctx.fillRect(px, py, cellW + 0.5, cellH + 0.5);
                }
            }
        }

        // ── Exploration frontier glow ──────────────────────────────
        // Soft edge where explored meets unexplored
        for (let gz = Math.max(1, gzMin); gz <= Math.min(gs - 2, gzMax); gz++) {
            for (let gx = Math.max(1, gxMin); gx <= Math.min(gs - 2, gxMax); gx++) {
                const idx = gz * gs + gx;
                if (this.state[idx] === CellState.UNKNOWN) continue;

                // Check if any neighbor is UNKNOWN → frontier cell
                let isFrontier = false;
                for (const [dx, dz] of [[-1,0],[1,0],[0,-1],[0,1]]) {
                    const nIdx = (gz + dz) * gs + (gx + dx);
                    if (this.state[nIdx] === CellState.UNKNOWN) {
                        isFrontier = true;
                        break;
                    }
                }
                if (isFrontier) {
                    const worldX = (gx + 0.5) * res - this.halfWorld;
                    const worldZ = (gz + 0.5) * res - this.halfWorld;
                    const px = worldToPixelX(worldX) - cellW / 2;
                    const py = worldToPixelY(worldZ) - cellH / 2;
                    this.ctx.fillStyle = 'rgba(120, 180, 80, 0.15)';
                    this.ctx.fillRect(px - 1, py - 1, cellW + 2, cellH + 2);
                }
            }
        }

        // ── STIGMERGY OVERLAYS (transient pheromones) ───────────
        const percRadiusPx = (15 / viewSize) * w;

        // Green perception circles (subtle)
        this.droneData.forEach((data) => {
            const cx = worldToPixelX(data.x);
            const cy = worldToPixelY(data.z);
            if (cx < -percRadiusPx || cx > w + percRadiusPx) return;
            if (cy < -percRadiusPx || cy > h + percRadiusPx) return;
            this.ctx.beginPath();
            this.ctx.arc(cx, cy, percRadiusPx, 0, Math.PI * 2);
            this.ctx.fillStyle = 'rgba(0, 255, 100, 0.04)';
            this.ctx.fill();
            this.ctx.strokeStyle = 'rgba(0, 255, 100, 0.20)';
            this.ctx.lineWidth = 1;
            this.ctx.stroke();
        });

        // YELLOW: obstacles 6-15m (intérêt — gold ring, doesn't obscure canopy)
        for (const m of this._interestMarkers) {
            const px = worldToPixelX(m.wx);
            const py = worldToPixelY(m.wz);
            if (px < -8 || px > w + 8 || py < -8 || py > h + 8) continue;
            const proximity = 1 - (m.dist - 6) / 9;
            const alpha = 0.25 + proximity * 0.35;
            this.ctx.beginPath();
            this.ctx.arc(px, py, 5, 0, Math.PI * 2);
            this.ctx.strokeStyle = `rgba(255, 215, 0, ${alpha})`;
            this.ctx.lineWidth = 2;
            this.ctx.stroke();
        }

        // RED: obstacles <6m (danger — red ring + fill)
        for (const m of this._dangerMarkers) {
            const px = worldToPixelX(m.wx);
            const py = worldToPixelY(m.wz);
            if (px < -8 || px > w + 8 || py < -8 || py > h + 8) continue;
            const proximity = 1 - m.dist / 6;
            const alpha = 0.5 + proximity * 0.4;
            this.ctx.beginPath();
            this.ctx.arc(px, py, 5, 0, Math.PI * 2);
            this.ctx.fillStyle = `rgba(255, 50, 30, ${alpha * 0.3})`;
            this.ctx.fill();
            this.ctx.strokeStyle = `rgba(255, 40, 20, ${alpha})`;
            this.ctx.lineWidth = 2;
            this.ctx.stroke();
        }

        // ── Tree canopies from GOLDEN TREES (persistent registry) ──
        // Use golden trees instead of raw clusters for stable rendering
        const goldenArray = Array.from(this._goldenTrees.values());
        this._lastTreeCount = goldenArray.length;
        let validatedCount = 0, consensusCount = 0;

        for (const tree of goldenArray) {
            // Convert grid centroid → world → pixel
            const treeWorldX = (tree.centroidX + 0.5) * res - this.halfWorld;
            const treeWorldZ = (tree.centroidZ + 0.5) * res - this.halfWorld;
            const cx = worldToPixelX(treeWorldX);
            const cy = worldToPixelY(treeWorldZ);

            // ── Backend algo: confidence modulated by tree state ──
            let conf = tree.confidence;
            if (tree.state === TreeState.CANDIDATE) conf *= 0.4; // semi-transparent
            if (tree.state === TreeState.VALIDATED) {
                conf = Math.max(conf, 0.85); // always bright
                validatedCount++;
            }
            if (tree.state === TreeState.CONSENSUS) consensusCount++;

            // Canopy radius: inflated from cluster size to realistic canopy
            const baseR = Math.max(2.5, Math.min(6, tree.size * 0.7 + 2.0));
            const canopyR = baseR * Math.min(cellW, cellH);

            // Species-like color variation (deterministic from position)
            const species = tree.species !== undefined ? tree.species
                : Math.floor(tree.centroidX * 3 + tree.centroidZ * 7) % 4;
            let coreR, coreG, coreB, edgeR, edgeG, edgeB;

            switch (species) {
                case 0: // Chêne Vert — dark green
                    coreR = 12; coreG = 52; coreB = 12;
                    edgeR = 30; edgeG = 82; edgeB = 28;
                    break;
                case 1: // Pin Parasol — blue-green
                    coreR = 10; coreG = 48; coreB = 22;
                    edgeR = 25; edgeG = 75; edgeB = 40;
                    break;
                case 2: // Cyprès — very dark green
                    coreR = 8;  coreG = 42; coreB = 10;
                    edgeR = 18; edgeG = 62; edgeB = 20;
                    break;
                case 3: // Olivier — olive green
                    coreR = 28; coreG = 50; coreB = 15;
                    edgeR = 55; edgeG = 85; edgeB = 35;
                    break;
            }

            // Shadow (offset below-right)
            this.ctx.beginPath();
            this.ctx.arc(cx + 1.5, cy + 1.5, canopyR + 1, 0, Math.PI * 2);
            this.ctx.fillStyle = `rgba(0, 0, 0, ${0.25 * conf})`;
            this.ctx.fill();

            // Canopy circle (radial gradient)
            const grad = this.ctx.createRadialGradient(
                cx - canopyR * 0.15,  // Light offset
                cy - canopyR * 0.15,
                0,
                cx, cy, canopyR
            );
            grad.addColorStop(0,   `rgba(${coreR + 20}, ${coreG + 30}, ${coreB + 10}, ${0.85 * conf})`);
            grad.addColorStop(0.4, `rgba(${coreR}, ${coreG}, ${coreB}, ${0.75 * conf})`);
            grad.addColorStop(0.8, `rgba(${edgeR}, ${edgeG}, ${edgeB}, ${0.50 * conf})`);
            grad.addColorStop(1,   `rgba(${edgeR}, ${edgeG}, ${edgeB}, ${0.05 * conf})`);

            this.ctx.beginPath();
            this.ctx.arc(cx, cy, canopyR, 0, Math.PI * 2);
            this.ctx.fillStyle = grad;
            this.ctx.fill();

            // Subtle canopy edge highlight (top-left light)
            if (conf > 0.3) {
                const hlGrad = this.ctx.createRadialGradient(
                    cx - canopyR * 0.3, cy - canopyR * 0.3, 0,
                    cx, cy, canopyR * 0.7
                );
                hlGrad.addColorStop(0, `rgba(120, 200, 100, ${0.12 * conf})`);
                hlGrad.addColorStop(1, 'rgba(120, 200, 100, 0)');
                this.ctx.beginPath();
                this.ctx.arc(cx, cy, canopyR * 0.7, 0, Math.PI * 2);
                this.ctx.fillStyle = hlGrad;
                this.ctx.fill();
            }

            // Trunk dot (small brown center)
            this.ctx.beginPath();
            this.ctx.arc(cx, cy, Math.max(1, cellW * 0.5), 0, Math.PI * 2);
            this.ctx.fillStyle = `rgba(85, 55, 20, ${0.7 * conf})`;
            this.ctx.fill();

            // ── Backend algo: golden state indicator ──
            if (tree.state === TreeState.VALIDATED) {
                // Small golden ring around validated trees
                this.ctx.beginPath();
                this.ctx.arc(cx, cy, canopyR + 1.5, 0, Math.PI * 2);
                this.ctx.strokeStyle = `rgba(255, 215, 0, ${0.35 * conf})`;
                this.ctx.lineWidth = 1;
                this.ctx.stroke();
            }
        }

        // ── Drone positions ────────────────────────────────────────
        this.droneData.forEach((data, droneId) => {
            const color = DRONE_COLORS[_droneIdx(droneId) % DRONE_COLORS.length];
            const cx = worldToPixelX(data.x);
            const cy = worldToPixelY(data.z);

            // Sensor range circle (subtle)
            const rangePixels = (this.cfg.maxSensorRange / viewSize) * w;
            this.ctx.beginPath();
            this.ctx.arc(cx, cy, rangePixels, 0, Math.PI * 2);
            this.ctx.strokeStyle = `${color}1F`; // hex + alpha
            this.ctx.lineWidth = 0.8;
            this.ctx.stroke();

            // Halo
            this.ctx.beginPath();
            this.ctx.arc(cx, cy, 5, 0, Math.PI * 2);
            this.ctx.fillStyle = color + '40';
            this.ctx.fill();

            // Body
            this.ctx.beginPath();
            this.ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
            this.ctx.fillStyle = color;
            this.ctx.fill();

            // Heading arrow
            const arrowLen = 7;
            const ax = cx + Math.sin(data.heading) * arrowLen;
            const ay = cy + Math.cos(data.heading) * arrowLen;
            this.ctx.beginPath();
            this.ctx.moveTo(cx, cy);
            this.ctx.lineTo(ax, ay);
            this.ctx.strokeStyle = color;
            this.ctx.lineWidth = 1.5;
            this.ctx.stroke();

            // ID label
            this.ctx.fillStyle = '#fff';
            this.ctx.font = 'bold 7px monospace';
            this.ctx.textAlign = 'center';
            this.ctx.fillText(droneId.toString(), cx, cy - 7);
        });

        // ── Border ─────────────────────────────────────────────────
        this.ctx.strokeStyle = '#3A7040';
        this.ctx.lineWidth = 2;
        this.ctx.strokeRect(1, 1, w - 2, h - 2);

        // ── Zone indicator (top-right) ─────────────────────────────
        const _dz = window.DIAMANTS_DOCTRINE?.zoneParams;
        const maxCfgZone = Math.max(
            _dz?.sizeX ?? this.halfWorld * 2,
            _dz?.sizeZ ?? this.halfWorld * 2
        );
        const zoneLabel = `${Math.round(viewSize)}m / ${Math.round(maxCfgZone)}m`;
        this.ctx.font = '8px monospace';
        const zlW = this.ctx.measureText(zoneLabel).width + 6;
        this.ctx.fillStyle = 'rgba(0,0,0,0.55)';
        this.ctx.fillRect(w - zlW - 3, 3, zlW, 13);
        this.ctx.fillStyle = '#8f8';
        this.ctx.textAlign = 'right';
        this.ctx.fillText(zoneLabel, w - 5, 13);

        // ── Stats overlay: forest reconstruction info ──────────────
        const treeCount = this._lastTreeCount || 0;
        this.ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
        this.ctx.fillRect(2, h - 18, 270, 16);
        this.ctx.fillStyle = '#88CC66';
        this.ctx.font = '9px monospace';
        this.ctx.textAlign = 'left';
        this.ctx.fillText(
            `🌲${treeCount}  ` +
            `✅${validatedCount} ⏳${consensusCount}  ` +
            `SOL:${this.stats.freeCells}  ` +
            `COV:${this.stats.coverage.toFixed(1)}%`,
            6, h - 7
        );

        // ── Pheromone legend (below zone indicator, top-right area) ─
        const lgY = 20;
        this.ctx.font = '7px monospace';
        this.ctx.textAlign = 'left';
        this.ctx.fillStyle = '#00ff88';
        this.ctx.beginPath(); this.ctx.arc(w - 100, lgY, 3, 0, Math.PI * 2); this.ctx.fill();
        this.ctx.fillText('Explo', w - 95, lgY + 2);
        this.ctx.fillStyle = '#ffd700';
        this.ctx.beginPath(); this.ctx.arc(w - 64, lgY, 3, 0, Math.PI * 2); this.ctx.fill();
        this.ctx.fillText('Int.', w - 59, lgY + 2);
        this.ctx.fillStyle = '#ff3232';
        this.ctx.beginPath(); this.ctx.arc(w - 36, lgY, 3, 0, Math.PI * 2); this.ctx.fill();
        this.ctx.fillText('Dng', w - 31, lgY + 2);
    }

    // ════════════════════════════════════════════════════════════════
    // CLEANUP
    // ════════════════════════════════════════════════════════════════

    dispose() {
        if (this._raf) cancelAnimationFrame(this._raf);
        this._raf = null;
    }
}

export default PerceptionMinimap;
