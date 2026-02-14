/**
 * Tests & Benchmarks — SSSP Algorithm (Duan et al. 2025)
 * Usage: node tests/test-sssp.js
 */

import {
    DirectedGraph, constantDegreeTransform,
    SSSPSolver, dijkstra, sssp, buildGridGraph,
} from '../core/sssp-algorithm.js';

let passed = 0, failed = 0;
function assert(cond, msg) {
    if (cond) { passed++; console.log(`  ✓ ${msg}`); }
    else { failed++; console.error(`  ✗ FAIL: ${msg}`); }
}
function assertClose(a, b, eps, msg) {
    assert(Math.abs(a - b) < eps, `${msg} (got ${a}, expected ${b})`);
}
function section(t) {
    console.log(`\n${'='.repeat(60)}\n  ${t}\n${'='.repeat(60)}`);
}

// ─── Test 1: Graph basics ───────────────────────────────────────────────

section('Test 1: DirectedGraph basics');
{
    const g = new DirectedGraph();
    g.addEdge(0, 1, 4); g.addEdge(0, 2, 2);
    g.addEdge(1, 3, 5); g.addEdge(2, 1, 1); g.addEdge(2, 3, 8);
    assert(g.n === 4, '4 vertices');
    assert(g.m === 5, '5 edges');
    assert(g.outEdges(0).length === 2, 'vertex 0: 2 out-edges');
}

// ─── Test 2: Constant-degree transform ──────────────────────────────────

section('Test 2: Constant-degree transformation');
{
    const g = new DirectedGraph();
    for (let i = 1; i <= 5; i++) g.addEdge(0, i, i);
    const { graph: g2 } = constantDegreeTransform(g);
    assert(g2.maxOutDegree() <= 2, `Max out-degree ≤ 2 (got ${g2.maxOutDegree()})`);
}

// ─── Test 3: Dijkstra correctness ───────────────────────────────────────

section('Test 3: Dijkstra correctness');
{
    const g = new DirectedGraph();
    g.addEdge(0, 1, 4); g.addEdge(0, 2, 2); g.addEdge(1, 3, 5);
    g.addEdge(2, 1, 1); g.addEdge(2, 3, 8); g.addEdge(3, 4, 1);
    const { dist } = dijkstra(g, 0);
    assertClose(dist.get(0), 0, 1e-9, 'dist(0) = 0');
    assertClose(dist.get(1), 3, 1e-9, 'dist(1) = 3');
    assertClose(dist.get(2), 2, 1e-9, 'dist(2) = 2');
    assertClose(dist.get(3), 8, 1e-9, 'dist(3) = 8');
    assertClose(dist.get(4), 9, 1e-9, 'dist(4) = 9');
}

// ─── Test 4: SSSP simple graph ──────────────────────────────────────────

section('Test 4: SSSP — simple graph');
{
    const r = sssp([
        { from: 0, to: 1, weight: 4 }, { from: 0, to: 2, weight: 2 },
        { from: 1, to: 3, weight: 5 }, { from: 2, to: 1, weight: 1 },
        { from: 2, to: 3, weight: 8 }, { from: 3, to: 4, weight: 1 },
    ], 0);
    assertClose(r.distances.get(0), 0, 1e-9, 'dist(0) = 0');
    assertClose(r.distances.get(1), 3, 1e-9, 'dist(1) = 3');
    assertClose(r.distances.get(2), 2, 1e-9, 'dist(2) = 2');
    assertClose(r.distances.get(3), 8, 1e-9, 'dist(3) = 8');
    assertClose(r.distances.get(4), 9, 1e-9, 'dist(4) = 9');
    console.log(`  Stats: ${JSON.stringify(r.stats)}`);
}

// ─── Test 5: SSSP vs Dijkstra — 100 vertices ───────────────────────────

section('Test 5: SSSP vs Dijkstra — random 100 vertices');
{
    let seed = 42;
    const rand = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };

    const n = 100;
    const edges = [];
    const g = new DirectedGraph();
    for (let i = 0; i < n; i++) g.addVertex(i);
    for (let i = 0; i < n; i++) {
        const ne = Math.floor(rand() * 4) + 1;
        for (let j = 0; j < ne; j++) {
            const to = Math.floor(rand() * n);
            if (to !== i) {
                const w = rand() * 10 + 0.1;
                edges.push({ from: i, to, weight: w });
                g.addEdge(i, to, w);
            }
        }
    }

    const dijkDist = dijkstra(g, 0).dist;
    const ssspDist = sssp(edges, 0, n).distances;

    let ok = true, maxErr = 0;
    for (let i = 0; i < n; i++) {
        const dd = dijkDist.get(i), sd = ssspDist.get(i);
        if (dd === Infinity && sd === Infinity) continue;
        const err = Math.abs(dd - sd);
        maxErr = Math.max(maxErr, err);
        if (err > 1e-6) { ok = false; console.error(`    Mismatch at ${i}: Dijk=${dd} SSSP=${sd}`); }
    }
    assert(ok, `All 100 distances match (maxErr=${maxErr.toExponential(2)})`);
}

// ─── Test 6: SSSP vs Dijkstra — 500 vertices ───────────────────────────

section('Test 6: SSSP vs Dijkstra — 500 vertices (sparse)');
{
    let seed = 137;
    const rand = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };

    const n = 500;
    const edges = [];
    const g = new DirectedGraph();
    for (let i = 0; i < n; i++) g.addVertex(i);
    for (let i = 0; i < n; i++) {
        const ne = Math.floor(rand() * 3) + 1;
        for (let j = 0; j < ne; j++) {
            const to = Math.floor(rand() * n);
            if (to !== i) { const w = rand() * 100; edges.push({ from: i, to, weight: w }); g.addEdge(i, to, w); }
        }
    }

    const t0 = performance.now();
    const dijkDist = dijkstra(g, 0).dist;
    const tD = performance.now() - t0;

    const t1 = performance.now();
    const ssspDist = sssp(edges, 0, n).distances;
    const tS = performance.now() - t1;

    let ok = true;
    for (let i = 0; i < n; i++) {
        const dd = dijkDist.get(i), sd = ssspDist.get(i);
        if (dd === Infinity && sd === Infinity) continue;
        if (Math.abs(dd - sd) > 1e-6) { ok = false; break; }
    }
    assert(ok, '500 vertices: all distances match');
    console.log(`  Dijkstra: ${tD.toFixed(2)} ms | SSSP: ${tS.toFixed(2)} ms`);
}

// ─── Test 7: Edge cases ─────────────────────────────────────────────────

section('Test 7: Edge cases');
{
    const r1 = sssp([], 0, 1);
    assertClose(r1.distances.get(0), 0, 1e-9, 'Single vertex: dist=0');

    const r2 = sssp([{ from: 0, to: 1, weight: 5 }], 0, 2);
    assertClose(r2.distances.get(1), 5, 1e-9, 'Two vertices: dist(1)=5');

    const r3 = sssp([{ from: 0, to: 1, weight: 1 }], 0, 3);
    assert(r3.distances.get(2) === Infinity, 'Unreachable: dist=∞');

    const r4 = sssp([
        { from: 0, to: 1, weight: 0 }, { from: 1, to: 2, weight: 0 },
        { from: 2, to: 3, weight: 1 },
    ], 0);
    assertClose(r4.distances.get(2), 0, 1e-9, 'Zero-weight: dist(2)=0');
    assertClose(r4.distances.get(3), 1, 1e-9, 'Zero-weight: dist(3)=1');

    const chain = [];
    for (let i = 0; i < 20; i++) chain.push({ from: i, to: i + 1, weight: 1 });
    assertClose(sssp(chain, 0).distances.get(20), 20, 1e-9, 'Chain: dist(20)=20');
}

// ─── Test 8: 3D Grid (DIAMANTS) ─────────────────────────────────────────

section('Test 8: 3D Grid graph (DIAMANTS)');
{
    const [sx, sy, sz] = [5, 5, 2];
    const data = new Float32Array(sx * sy * sz).fill(0);
    const g = buildGridGraph(data, sx, sy, sz, 1.0);

    assert(g.n === 50, `Grid: ${g.n} vertices (expected 50)`);
    const { dist } = dijkstra(g, 0);
    assertClose(dist.get(0), 0, 1e-9, 'Grid dist(0,0,0)=0');
    assertClose(dist.get(1), 1, 1e-9, 'Grid dist(1,0,0)=1');
}

// ─── Benchmark ──────────────────────────────────────────────────────────

section('Benchmark: 1000 vertices');
{
    let seed = 999;
    const rand = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };

    const n = 1000;
    const edges = [];
    const g = new DirectedGraph();
    for (let i = 0; i < n; i++) g.addVertex(i);
    for (let i = 0; i < n; i++) {
        const ne = Math.floor(rand() * 3) + 1;
        for (let j = 0; j < ne; j++) {
            const to = Math.floor(rand() * n);
            if (to !== i) { const w = rand() * 50; edges.push({ from: i, to, weight: w }); g.addEdge(i, to, w); }
        }
    }

    // Warm up
    dijkstra(g, 0); sssp(edges, 0, n);

    const runs = 5;
    let tD = 0, tS = 0;
    for (let r = 0; r < runs; r++) {
        let t = performance.now(); dijkstra(g, 0); tD += performance.now() - t;
        t = performance.now(); sssp(edges, 0, n); tS += performance.now() - t;
    }
    console.log(`  Dijkstra avg: ${(tD / runs).toFixed(2)} ms`);
    console.log(`  SSSP avg:     ${(tS / runs).toFixed(2)} ms`);
    console.log(`  Ratio:        ${(tS / tD).toFixed(2)}x`);
}

// ─── Summary ────────────────────────────────────────────────────────────

console.log(`\n${'='.repeat(60)}\n  Results: ${passed} passed, ${failed} failed\n${'='.repeat(60)}`);
if (failed) process.exit(1);
