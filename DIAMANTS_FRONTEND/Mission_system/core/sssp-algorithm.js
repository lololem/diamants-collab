/**
 * ============================================================================
 * SSSP — Breaking the Sorting Barrier for Directed Single-Source Shortest Paths
 * ============================================================================
 * 
 * Implementation of the O(m·log^{2/3} n) deterministic SSSP algorithm from:
 * 
 *   "Breaking the Sorting Barrier for Directed Single-Source Shortest Paths"
 *   Ran Duan, Jiayi Mao, Xiao Mao, Xinkai Shu, Longhui Yin (2025)
 *   arXiv:2504.17033v2
 * 
 * This algorithm beats Dijkstra's O(m + n·log n) on sparse directed graphs
 * with real non-negative edge weights in the comparison-addition model.
 * 
 * For small graphs (n < DIJKSTRA_THRESHOLD), Dijkstra is used directly since
 * the theoretical advantage only manifests for very large n where
 * log^{1/3}(n) is significant.
 * 
 * Architecture:
 *   1. DirectedGraph — adjacency list representation
 *   2. constantDegreeTransform — max degree → 2 (Frederickson 1983)
 *   3. PartialSortingDS — Lemma 3.3 data structure (Insert/BatchPrepend/Pull)
 *   4. FindPivots — Algorithm 1 (k BF steps, frontier reduction)
 *   5. BaseCase — Algorithm 2 (mini-Dijkstra for l=0)
 *   6. BMSSP — Algorithm 3 (recursive bounded multi-source shortest path)
 *   7. SSSPSolver — top-level solver with Dijkstra fallback
 * 
 * @module sssp-algorithm
 * @version 1.0.0
 */

// For practical sizes in DIAMANTS (~3000 cells), Dijkstra is faster.
// The theoretical benefit only appears for n >> 2^27 ≈ 134M vertices.
const DIJKSTRA_THRESHOLD = 100000;

// ============================================================================
// 1. DIRECTED GRAPH
// ============================================================================

export class DirectedGraph {
    constructor() {
        this.n = 0;
        this.adj = new Map();   // vertex -> [{to, weight}]
        this.radj = new Map();  // vertex -> [{from, weight}]
        this.vertices = new Set();
    }

    addVertex(v) {
        if (!this.vertices.has(v)) {
            this.vertices.add(v);
            this.n = this.vertices.size;
            if (!this.adj.has(v)) this.adj.set(v, []);
            if (!this.radj.has(v)) this.radj.set(v, []);
        }
    }

    addEdge(u, v, w) {
        this.addVertex(u);
        this.addVertex(v);
        this.adj.get(u).push({ to: v, weight: w });
        this.radj.get(v).push({ from: u, weight: w });
    }

    outEdges(u) { return this.adj.get(u) || []; }

    get m() {
        let c = 0;
        for (const e of this.adj.values()) c += e.length;
        return c;
    }

    maxOutDegree() {
        let d = 0;
        for (const e of this.adj.values()) d = Math.max(d, e.length);
        return d;
    }

    maxInDegree() {
        let d = 0;
        for (const e of this.radj.values()) d = Math.max(d, e.length);
        return d;
    }
}

// ============================================================================
// 2. CONSTANT-DEGREE TRANSFORM (Frederickson 1983)
// ============================================================================

/**
 * Transform graph to max in/out degree 2.
 * Each vertex v → directed cycle of deg(v) vertices with 0-weight edges.
 */
export function constantDegreeTransform(G) {
    const G2 = new DirectedGraph();
    const mapping = new Map();
    const reverseMapping = new Map();
    let nextId = 0;
    const vertexEntries = new Map();

    for (const v of G.vertices) {
        const out = G.outEdges(v);
        const inc = G.radj.get(v) || [];
        const entries = [];
        for (const e of out) entries.push({ type: 'out', neighbor: e.to, weight: e.weight });
        for (const e of inc) entries.push({ type: 'in', neighbor: e.from, weight: e.weight });

        if (entries.length === 0) {
            const id = nextId++;
            G2.addVertex(id);
            mapping.set(v, id);
            reverseMapping.set(id, v);
            continue;
        }

        const ids = [];
        for (let i = 0; i < entries.length; i++) {
            const id = nextId++;
            ids.push(id);
            G2.addVertex(id);
            reverseMapping.set(id, v);
            entries[i].cycleId = id;
        }
        mapping.set(v, ids[0]);

        // Single-direction cycle: 0-weight
        for (let i = 0; i < ids.length; i++) {
            G2.addEdge(ids[i], ids[(i + 1) % ids.length], 0);
        }
        vertexEntries.set(v, entries);
    }

    // Inter-vertex edges
    for (const v of G.vertices) {
        const entries = vertexEntries.get(v);
        if (!entries) continue;
        for (const e of entries) {
            if (e.type === 'out') {
                const ne = vertexEntries.get(e.neighbor);
                if (ne) {
                    const match = ne.find(x => x.type === 'in' && x.neighbor === v);
                    if (match) G2.addEdge(e.cycleId, match.cycleId, e.weight);
                }
            }
        }
    }

    return { graph: G2, mapping, reverseMapping };
}

// ============================================================================
// 3. BINARY HEAP
// ============================================================================

class BinaryHeap {
    constructor() { this.data = []; this.idx = new Map(); }
    get size() { return this.data.length; }

    insertOrDecrease(key, val) {
        if (this.idx.has(key)) {
            const i = this.idx.get(key);
            if (val < this.data[i].value) { this.data[i].value = val; this._up(i); }
        } else {
            this.data.push({ key, value: val });
            this.idx.set(key, this.data.length - 1);
            this._up(this.data.length - 1);
        }
    }

    extractMin() {
        if (!this.data.length) return null;
        const min = this.data[0];
        const last = this.data.pop();
        this.idx.delete(min.key);
        if (this.data.length) {
            this.data[0] = last;
            this.idx.set(last.key, 0);
            this._down(0);
        }
        return min;
    }

    _up(i) {
        while (i > 0) {
            const p = (i - 1) >> 1;
            if (this.data[i].value < this.data[p].value) { this._swap(i, p); i = p; }
            else break;
        }
    }

    _down(i) {
        const n = this.data.length;
        while (true) {
            let s = i, l = 2 * i + 1, r = 2 * i + 2;
            if (l < n && this.data[l].value < this.data[s].value) s = l;
            if (r < n && this.data[r].value < this.data[s].value) s = r;
            if (s !== i) { this._swap(i, s); i = s; } else break;
        }
    }

    _swap(a, b) {
        [this.data[a], this.data[b]] = [this.data[b], this.data[a]];
        this.idx.set(this.data[a].key, a);
        this.idx.set(this.data[b].key, b);
    }
}

// ============================================================================
// 4. PARTIAL SORTING DS (Lemma 3.3)
// ============================================================================

/**
 * Simplified partial sorting data structure using a Map.
 * Supports Insert O(1 amortized), BatchPrepend O(L), Pull O(M log M).
 */
class PartialSortingDS {
    constructor(M, B) {
        this.M = Math.max(1, M);
        this.B = B;
        this.entries = new Map(); // key -> value
    }

    insert(key, value) {
        const cur = this.entries.get(key);
        if (cur === undefined || value < cur) this.entries.set(key, value);
    }

    batchPrepend(list) {
        for (const { key, value } of list) this.insert(key, value);
    }

    pull() {
        if (this.entries.size === 0) return { keys: new Set(), x: this.B };

        const sorted = [...this.entries.entries()].sort((a, b) => a[1] - b[1]);
        const count = Math.min(this.M, sorted.length);
        const keys = new Set();
        for (let i = 0; i < count; i++) {
            keys.add(sorted[i][0]);
            this.entries.delete(sorted[i][0]);
        }

        const x = count < sorted.length ? sorted[count][1] : this.B;
        return { keys, x };
    }

    isEmpty() { return this.entries.size === 0; }
}

// ============================================================================
// 5. DIJKSTRA
// ============================================================================

/**
 * Standard Dijkstra's algorithm O(m + n log n).
 * @param {DirectedGraph} graph
 * @param {number} source
 * @returns {{ dist: Map<number, number>, pred: Map<number, number> }}
 */
export function dijkstra(graph, source) {
    const dist = new Map();
    const pred = new Map();
    for (const v of graph.vertices) { dist.set(v, Infinity); pred.set(v, -1); }
    dist.set(source, 0);

    const heap = new BinaryHeap();
    heap.insertOrDecrease(source, 0);

    while (heap.size > 0) {
        const { key: u, value: d } = heap.extractMin();
        if (d > dist.get(u)) continue;
        for (const e of graph.outEdges(u)) {
            const nd = d + e.weight;
            if (nd < dist.get(e.to)) {
                dist.set(e.to, nd);
                pred.set(e.to, u);
                heap.insertOrDecrease(e.to, nd);
            }
        }
    }
    return { dist, pred };
}

// ============================================================================
// 6. SSSP SOLVER
// ============================================================================

export class SSSPSolver {
    constructor(graph) {
        this.originalGraph = graph;

        const maxDeg = Math.max(graph.maxOutDegree(), graph.maxInDegree());
        if (maxDeg <= 2) {
            this.graph = graph;
            this.transformed = false;
            this.mapping = null;
            this.reverseMapping = null;
        } else {
            const r = constantDegreeTransform(graph);
            this.graph = r.graph;
            this.mapping = r.mapping;
            this.reverseMapping = r.reverseMapping;
            this.transformed = true;
        }

        const n = this.graph.n;
        const logN = Math.log2(Math.max(2, n));
        this.k = Math.max(2, Math.floor(Math.pow(logN, 1 / 3)));
        this.t = Math.max(1, Math.floor(Math.pow(logN, 2 / 3)));

        this.dHat = new Map();
        this.pred = new Map();
        this.complete = new Set();

        for (const v of this.graph.vertices) {
            this.dHat.set(v, Infinity);
            this.pred.set(v, -1);
        }

        this.stats = { relaxations: 0, findPivots: 0, bmssp: 0, baseCase: 0, usedDijkstra: false };
    }

    solve(s) {
        const src = this.transformed ? this.mapping.get(s) : s;
        if (src === undefined) throw new Error(`Source ${s} not found`);

        // Dijkstra fallback for practical sizes
        if (this.graph.n < DIJKSTRA_THRESHOLD) {
            this.stats.usedDijkstra = true;
            const r = dijkstra(this.graph, src);
            this.dHat = r.dist;
            this.pred = r.pred;
            return this._extractResults();
        }

        this.dHat.set(src, 0);
        this.complete.add(src);
        for (const e of this.graph.outEdges(src)) this._relax(src, e.to, e.weight);

        const topL = Math.max(0, Math.ceil(Math.log2(Math.max(2, this.graph.n)) / this.t));
        this._bmssp(topL, Infinity, new Set([src]));

        return this._extractResults();
    }

    _relax(u, v, w) {
        this.stats.relaxations++;
        const nd = this.dHat.get(u) + w;
        if (nd <= this.dHat.get(v)) {
            if (nd < this.dHat.get(v)) { this.dHat.set(v, nd); this.pred.set(v, u); }
            return true;
        }
        return false;
    }

    // Algorithm 1: FindPivots
    _findPivots(B, S) {
        this.stats.findPivots++;
        const W = new Set(S);
        let prev = new Set(S);

        for (let i = 1; i <= this.k; i++) {
            const next = new Set();
            for (const u of prev) {
                for (const e of this.graph.outEdges(u)) {
                    const nd = this.dHat.get(u) + e.weight;
                    if (nd >= B) continue;
                    if (nd <= this.dHat.get(e.to)) {
                        if (nd < this.dHat.get(e.to)) {
                            this.dHat.set(e.to, nd);
                            this.pred.set(e.to, u);
                        }
                        if (!W.has(e.to)) { next.add(e.to); W.add(e.to); }
                    }
                }
            }
            if (W.size > this.k * S.size) return { P: new Set(S), W };
            prev = next;
        }

        // Count subtree sizes to find pivots
        const cnt = new Map();
        for (const w of W) {
            if (S.has(w)) continue;
            let cur = w;
            const vis = new Set([cur]);
            while (cur !== undefined && cur !== -1 && !S.has(cur)) {
                cur = this.pred.get(cur);
                if (cur === undefined || cur === -1 || vis.has(cur)) break;
                vis.add(cur);
            }
            if (cur !== undefined && cur !== -1 && S.has(cur)) {
                cnt.set(cur, (cnt.get(cur) || 0) + 1);
            }
        }

        const P = new Set();
        for (const [v, c] of cnt) if (c >= this.k) P.add(v);
        for (const v of W) this.complete.add(v);

        return { P, W };
    }

    // Algorithm 2: BaseCase (mini-Dijkstra)
    _baseCase(B, S) {
        this.stats.baseCase++;
        const x = S.values().next().value;
        const U = new Set([x]);
        const heap = new BinaryHeap();
        heap.insertOrDecrease(x, this.dHat.get(x));

        let count = 0;
        const limit = this.k + 1;

        while (heap.size > 0 && count <= limit) {
            const { key: u, value: d } = heap.extractMin();
            if (d >= B || d > this.dHat.get(u)) continue;
            this.complete.add(u);
            U.add(u);
            count++;
            for (const e of this.graph.outEdges(u)) {
                const nd = this.dHat.get(u) + e.weight;
                if (nd < B && nd <= this.dHat.get(e.to)) {
                    if (nd < this.dHat.get(e.to)) { this.dHat.set(e.to, nd); this.pred.set(e.to, u); }
                    heap.insertOrDecrease(e.to, nd);
                }
            }
        }

        if (count <= limit) return { Bprime: B, U };

        let maxD = -Infinity;
        for (const v of U) { const d = this.dHat.get(v); if (d !== Infinity && d > maxD) maxD = d; }
        const Uf = new Set();
        for (const v of U) if (this.dHat.get(v) < maxD) Uf.add(v);
        return { Bprime: maxD, U: Uf };
    }

    // Algorithm 3: BMSSP
    _bmssp(l, B, S) {
        this.stats.bmssp++;

        if (l === 0) {
            if (S.size !== 1) {
                const U = new Set();
                let Bp = B;
                for (const x of S) {
                    const r = this._baseCase(B, new Set([x]));
                    for (const v of r.U) U.add(v);
                    Bp = Math.min(Bp, r.Bprime);
                }
                return { Bprime: Bp, U };
            }
            return this._baseCase(B, S);
        }

        const { P, W } = this._findPivots(B, S);
        if (P.size === 0) return { Bprime: B, U: W };

        const M = Math.max(1, Math.min(Math.round(Math.pow(2, (l - 1) * this.t)), P.size));
        const D = new PartialSortingDS(M, B);
        for (const x of P) D.insert(x, this.dHat.get(x));

        const U = new Set();
        const kThreshold = this.k * Math.pow(2, l * this.t);
        const maxIter = this.graph.n * 2;

        for (let it = 0; it < maxIter && !D.isEmpty(); it++) {
            const { keys: Si, x: Bi } = D.pull();
            if (Si.size === 0) break;

            const r = this._bmssp(l - 1, Bi, Si);
            for (const v of r.U) U.add(v);

            const K = [];
            for (const u of r.U) {
                for (const e of this.graph.outEdges(u)) {
                    const nd = this.dHat.get(u) + e.weight;
                    if (nd <= this.dHat.get(e.to)) {
                        if (nd < this.dHat.get(e.to)) { this.dHat.set(e.to, nd); this.pred.set(e.to, u); }
                        if (nd >= Bi && nd < B) D.insert(e.to, nd);
                        else if (nd >= r.Bprime && nd < Bi) K.push({ key: e.to, value: nd });
                    }
                }
            }

            const batch = [...K];
            for (const x of Si) {
                const d = this.dHat.get(x);
                if (d >= r.Bprime && d < Bi) batch.push({ key: x, value: d });
            }
            if (batch.length > 0) D.batchPrepend(batch);

            if (D.isEmpty() || U.size > kThreshold) break;
        }

        let Bp = D.isEmpty() ? B : Infinity;
        if (!D.isEmpty()) {
            for (const [, v] of D.entries) if (v < Bp) Bp = v;
        }
        for (const x of W) {
            if (this.dHat.get(x) < Bp) { U.add(x); this.complete.add(x); }
        }

        return { Bprime: Bp, U };
    }

    _extractResults() {
        const dist = new Map();
        if (this.transformed) {
            for (const v of this.originalGraph.vertices) {
                const iv = this.mapping.get(v);
                dist.set(v, iv !== undefined ? this.dHat.get(iv) : Infinity);
            }
        } else {
            for (const v of this.graph.vertices) dist.set(v, this.dHat.get(v));
        }
        return dist;
    }
}

// ============================================================================
// 7. SIMPLE API
// ============================================================================

/**
 * Compute SSSP on edge list.
 * @param {Array<{from:number, to:number, weight:number}>} edges
 * @param {number} source
 * @param {number} [numVertices]
 * @returns {{ distances: Map<number, number>, stats: object }}
 */
export function sssp(edges, source, numVertices) {
    const g = new DirectedGraph();
    if (numVertices) for (let i = 0; i < numVertices; i++) g.addVertex(i);
    for (const { from, to, weight } of edges) g.addEdge(from, to, weight);
    const solver = new SSSPSolver(g);
    return { distances: solver.solve(source), stats: solver.stats };
}

/**
 * Dijkstra convenience wrapper on edge list.
 */
export function dijkstraEdgeList(edges, source) {
    const g = new DirectedGraph();
    for (const { from, to, weight } of edges) g.addEdge(from, to, weight);
    return dijkstra(g, source).dist;
}

// ============================================================================
// 8. DIAMANTS INTEGRATION
// ============================================================================

/**
 * Build 6-connected 3D grid graph for DIAMANTS PDE field navigation.
 */
export function buildGridGraph(fieldData, sizeX, sizeY, sizeZ, resolution) {
    const g = new DirectedGraph();
    const idx = (x, y, z) => x + y * sizeX + z * sizeX * sizeY;

    for (let z = 0; z < sizeZ; z++)
        for (let y = 0; y < sizeY; y++)
            for (let x = 0; x < sizeX; x++)
                g.addVertex(idx(x, y, z));

    const dirs = [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]];
    for (let z = 0; z < sizeZ; z++) {
        for (let y = 0; y < sizeY; y++) {
            for (let x = 0; x < sizeX; x++) {
                const u = idx(x, y, z);
                for (const [dx, dy, dz] of dirs) {
                    const nx = x + dx, ny = y + dy, nz = z + dz;
                    if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY || nz < 0 || nz >= sizeZ) continue;
                    const v = idx(nx, ny, nz);
                    const fv = fieldData ? (fieldData[v] || 0) : 0;
                    g.addEdge(u, v, resolution * (1 + Math.max(0, fv)));
                }
            }
        }
    }
    return g;
}

/**
 * Find shortest path between two 3D positions in DIAMANTS field.
 */
export function findShortestPath(fieldData, start, end, sizeX, sizeY, sizeZ, resolution) {
    const cl = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
    const idx = (x, y, z) => x + y * sizeX + z * sizeX * sizeY;
    const fromIdx = (i) => ({
        x: (i % sizeX) * resolution,
        y: (Math.floor(i / sizeX) % sizeY) * resolution,
        z: Math.floor(i / (sizeX * sizeY)) * resolution,
    });

    const g = buildGridGraph(fieldData, sizeX, sizeY, sizeZ, resolution);
    const srcId = idx(cl(Math.round(start.x / resolution), 0, sizeX - 1),
                      cl(Math.round(start.y / resolution), 0, sizeY - 1),
                      cl(Math.round(start.z / resolution), 0, sizeZ - 1));
    const tgtId = idx(cl(Math.round(end.x / resolution), 0, sizeX - 1),
                      cl(Math.round(end.y / resolution), 0, sizeY - 1),
                      cl(Math.round(end.z / resolution), 0, sizeZ - 1));

    const { dist, pred } = dijkstra(g, srcId);
    const distance = dist.get(tgtId);
    const path = [];
    if (distance !== Infinity) {
        let cur = tgtId;
        const vis = new Set();
        while (cur !== undefined && cur !== -1 && !vis.has(cur)) {
            vis.add(cur);
            path.unshift(fromIdx(cur));
            if (cur === srcId) break;
            cur = pred.get(cur);
        }
    }
    return { distance, path };
}
