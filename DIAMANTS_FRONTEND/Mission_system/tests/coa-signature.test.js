/**
 * DIAMANTS — does the search pattern leave a MEASURABLE trace?
 * ============================================================
 * The combinatorics suite checked that nothing breaks: finite numbers, points
 * inside bounds, labels present. It passed just as well with a completely inert
 * course of action — and it was inert: its weight was `1 - autonomy/100`, so
 * ZERO when distributed, and below 75 % it only re-ranked eight points drawn at
 * random around a group centre. Measured in flight, four courses of action
 * could not be told apart.
 *
 * These tests ask the opposite: that every pattern be VISIBLE.
 *
 * @vitest-environment jsdom
 */
import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import { AutonomousFlightEngine } from '../physics/autonomous-flight-engine.js';
import { DoctrineManager } from '../missions/mission-doctrine.js';

class StubSwarm {
    constructor() { this.lightweightMode = false; }
    getMetrics() { return {}; }
    setLightweightMode(v) { this.lightweightMode = v; }
    setOrganicMode(v) { this.organicMode = v; }
    initialize() {} reset() {} tick() {} lateInitObstacles() {}
    updateEnvironment() {} setTerrainHeightFunction() {}
    readPheromone(x, z) { return Math.abs(Math.sin(x * 0.5) * Math.cos(z * 0.5)) * 20; }
    depositPheromone() {}
    modulateVelocity(id, v) { return { x: v.x, y: v.y, z: v.z }; }
    computeNextWaypoint(id, state) {
        return new THREE.Vector3(state.position.x, state._cruiseAlt || 3, state.position.z);
    }
}

function build({ coa, autonomy, n = 4, bounds = 60 }) {
    const engine = new AutonomousFlightEngine({ explorationBounds: bounds });
    engine.swarmIntelligence = new StubSwarm();
    const dm = new DoctrineManager();
    dm.setDoctrine('exploration');
    dm.setCOA(coa);
    engine.setDoctrineManager(dm);
    engine.setAutonomyLevel(autonomy);
    for (let i = 1; i <= n; i++) {
        engine.registerDrone(`cf_${i}`, 'CRAZYFLIE', new THREE.Vector3(i * 4 - 8, 0.15, i % 2 ? 5 : -5));
    }
    return engine;
}

const firstDrone = (engine) => [...engine.drones.values()][0];

describe('the pattern weight no longer collapses to zero', () => {
    it('a course of action keeps weight even at full autonomy', () => {
        for (const a of [0, 50, 100]) {
            expect(build({ coa: 'boustrophedon', autonomy: a })._coaWeight()).toBeGreaterThan(0.3);
        }
        // and it weighs more as the decision gets centralised
        expect(build({ coa: 'grid', autonomy: 0 })._coaWeight())
            .toBeGreaterThan(build({ coa: 'grid', autonomy: 100 })._coaWeight());
    });

    it('"adaptive" stays free of any imposed pattern, by design', () => {
        const e = build({ coa: 'adaptive', autonomy: 0 });
        expect(e._coaWeight()).toBe(0);
        expect(e._coaNextWaypoint(firstDrone(e))).toBeNull();
    });
});

describe('each pattern proposes its own geometry', () => {
    it('boustrophedon: stay on the lane, reverse direction on the next one', () => {
        const e = build({ coa: 'boustrophedon', autonomy: 50 });
        const s = firstDrone(e);
        const spacing = e.doctrineManager.currentCOA.params?.lineSpacing || 5;

        s.position.set(0, 3, 0.5 * spacing);
        const even = e._coaNextWaypoint(s);
        s.position.set(0, 3, 1.5 * spacing);
        const odd = e._coaNextWaypoint(s);

        expect(Math.sign(even.x)).toBe(1);
        expect(Math.sign(odd.x)).toBe(-1);
        expect(Math.abs(even.z - 0.5 * spacing)).toBeLessThan(spacing);
    });

    it('spiral: the angle advances in the chosen direction and the radius grows', () => {
        const e = build({ coa: 'spiral', autonomy: 50 });
        const s = firstDrone(e);
        s._territoryCenter = { x: 0, z: 0 };
        s.position.set(10, 3, 0);
        const p = e._coaNextWaypoint(s);
        expect(Math.hypot(p.x, p.z)).toBeGreaterThan(10);
        expect(Math.atan2(p.z, p.x)).toBeLessThan(0);          // clockwise by default
    });

    it('radial: move away from the centre, and start over from it at the edge', () => {
        const e = build({ coa: 'radial', autonomy: 50, bounds: 60 });
        const s = firstDrone(e);
        s._territoryCenter = { x: 0, z: 0 };
        s.position.set(8, 3, 6);
        const out = e._coaNextWaypoint(s);
        expect(Math.hypot(out.x, out.z)).toBeGreaterThan(Math.hypot(8, 6));

        s.position.set(58, 3, 0);
        const back = e._coaNextWaypoint(s);
        expect(Math.hypot(back.x, back.z)).toBeLessThan(5);
    });

    it('perimeter: the next step sits on the boundary ring', () => {
        const e = build({ coa: 'perimeter', autonomy: 50, bounds: 60 });
        const s = firstDrone(e);
        s.position.set(20, 3, 0);
        const p = e._coaNextWaypoint(s);
        expect(Math.abs(Math.hypot(p.x, p.z) - (e._getHalfZone() - 6))).toBeLessThan(2);
    });

    it('no pattern ever leaves the theatre', () => {
        for (const coa of ['grid', 'boustrophedon', 'spiral', 'radial', 'perimeter']) {
            const e = build({ coa, autonomy: 50, bounds: 40 });
            const half = e._getHalfZone();
            const s = firstDrone(e);
            for (const [x, z] of [[half - 1, half - 1], [-half + 1, -half + 1], [0, 0], [half - 1, -half + 1]]) {
                s.position.set(x, 3, z);
                const p = e._coaNextWaypoint(s);
                expect(Math.abs(p.x)).toBeLessThanOrEqual(half);
                expect(Math.abs(p.z)).toBeLessThanOrEqual(half);
                expect(Number.isFinite(p.x) && Number.isFinite(p.z)).toBe(true);
            }
        }
    });
});


// ── In flight: the pattern must show in the track, not only in the code ──

/** Flies the fleet and returns each aircraft's track. */
function fly(coa, autonomy, seconds = 70, seed = 1) {
    let x = seed * 9301 + 49297;                         // reproducible draw
    const realRandom = Math.random;
    Math.random = () => ((x = (x * 9301 + 49297) % 233280) / 233280);
    try {
        const e = build({ coa, autonomy, n: 4, bounds: 60 });
        for (const s of e.drones.values()) { s.phase = 'TAKEOFF'; }
        for (let f = 0; f < 600; f++) e.update(1 / 60);
        for (const s of e.drones.values()) { if (s.phase !== 'EXPLORE') { s.phase = 'EXPLORE'; s.lastWaypointTime = -999; } }
        const tracks = new Map([...e.drones.keys()].map(id => [id, []]));
        for (let f = 0; f < seconds * 60; f++) {
            e.update(1 / 60);
            if (f % 15 === 0) for (const [id, s] of e.drones) tracks.get(id).push([s.position.x, s.position.z]);
        }
        return [...tracks.values()].filter(t => t.length > 10);
    } finally { Math.random = realRandom; }
}

/** Share of the track where lateral motion dominates — a lane's signature. */
function laneLocked(tracks) {
    let n = 0, total = 0;
    for (const t of tracks) for (let i = 1; i < t.length; i++) {
        const dx = Math.abs(t[i][0] - t[i - 1][0]), dz = Math.abs(t[i][1] - t[i - 1][1]);
        if (dx + dz < 0.4) continue;                     // standing still: says nothing
        total++; if (dx > 2 * dz) n++;
    }
    return total ? n / total : 0;
}

/** How consistently it turns the same way around the centre — a spiral's signature. */
function turnConsistency(tracks) {
    let sum = 0, total = 0;
    for (const t of tracks) for (let i = 1; i < t.length; i++) {
        const a0 = Math.atan2(t[i - 1][1], t[i - 1][0]), a1 = Math.atan2(t[i][1], t[i][0]);
        let d = a1 - a0; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI;
        if (Math.abs(d) < 0.02) continue;
        sum += Math.sign(d); total++;
    }
    return total ? Math.abs(sum / total) : 0;
}

const mean = (v) => v.reduce((a, b) => a + b, 0) / v.length;

describe('in flight, the patterns no longer look alike', () => {
    it('boustrophedon flies in lanes where adaptive has none', () => {
        const lanes = [1, 2, 3].map(g => laneLocked(fly('boustrophedon', 30, 70, g)));
        const free = [1, 2, 3].map(g => laneLocked(fly('adaptive', 30, 70, g)));
        expect(mean(lanes)).toBeGreaterThan(mean(free) + 0.05);
    }, 120000);

    it('the spiral turns one way, adaptive does not', () => {
        const spiralRuns = [1, 2, 3].map(g => turnConsistency(fly('spiral', 30, 70, g)));
        const free = [1, 2, 3].map(g => turnConsistency(fly('adaptive', 30, 70, g)));
        expect(mean(spiralRuns)).toBeGreaterThan(mean(free) + 0.08);
    }, 120000);

    it('the pattern acts at full autonomy too — that was the defect', () => {
        const lanes = [1, 2].map(g => laneLocked(fly('boustrophedon', 100, 70, g)));
        const free = [1, 2].map(g => laneLocked(fly('adaptive', 100, 70, g)));
        expect(mean(lanes)).toBeGreaterThan(mean(free) + 0.03);
    }, 120000);
});
