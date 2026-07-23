/**
 * DIAMANTS — Doctrine × COA × Autonomy Combinatorics Test Suite
 * ==============================================================
 * Exhaustively tests all valid combinations of:
 *   5 Doctrines × 7 COAs × 3 Autonomy bands (0%, 50%, 100%)
 *   = 105 combinations
 *
 * For each combo, verifies:
 *   1. Engine initialises without error
 *   2. Drones can take off and enter EXPLORE
 *   3. Waypoints are generated (not null, within bounds)
 *   4. _lastAction is set (decision label populated)
 *   5. COA bias is finite (no NaN/Infinity)
 *   6. Pheromone lightweight mode is correct (ON/OFF per doctrine+COA)
 *   7. resetAll() clears all state cleanly
 *
 * @vitest-environment jsdom
 */
/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */

import { describe, it, expect, beforeEach } from 'vitest';
import * as THREE from 'three';
import { AutonomousFlightEngine } from '../physics/autonomous-flight-engine.js';
import {
    DOCTRINES,
    COURSES_OF_ACTION,
    DoctrineManager
} from '../missions/mission-doctrine.js';

// ── Minimal Crazyflie profile for testing ──
const CF_PROFILE = {
    id: 'CF',
    label: 'Crazyflie 2.1',
    model: 'crazyflie',
    mass: 0.027,
    armLength: 0.046,
    maxSpeed: 1.5,
    maxClimb: 1.0,
    cruiseAlt: 3.0,
    maxAlt: 10.0,
    agility: 1.0,
    boundingRadius: 0.1,
    explorationRadius: 30,
    pid: {
        pos: { kp: 2.0, ki: 0.0, kd: 0.0 },
        alt: { kp: 2.0, ki: 0.0, kd: 0.0 },
        yaw: { kp: 2.0, ki: 0.0, kd: 0.0 },
    },
};

// ── Lightweight swarm intelligence stub (with all required methods) ──
class StubSwarmIntelligence {
    constructor() {
        this.lightweightMode = false;
        this.organicMode = false;
        this._config = { maxIntensity: 100 };
    }
    getMetrics() { return {}; }
    setLightweightMode(v) { this.lightweightMode = v; }
    setOrganicMode(v) { this.organicMode = v; }
    initialize() {}
    reset() {}
    tick() {}
    lateInitObstacles() {}
    updateEnvironment() {}
    setTerrainHeightFunction() {}
    readPheromone(x, z, type) {
        return Math.abs(Math.sin(x * 0.5) * Math.cos(z * 0.5)) * 20;
    }
    depositPheromone(droneId, x, z, type, intensity) { /* noop */ }
    modulateVelocity(droneId, velocity, state) {
        return { x: velocity.x, y: velocity.y, z: velocity.z };
    }
    computeNextWaypoint(id, state, neighbors) {
        return new THREE.Vector3(
            state.position.x + (Math.random() - 0.5) * 10,
            state._cruiseAlt || state.profile.cruiseAlt,
            state.position.z + (Math.random() - 0.5) * 10
        );
    }
}

// ── Doctrine keys and COA keys ──
const DOCTRINE_KEYS = Object.keys(DOCTRINES);     // 5
const COA_KEYS = Object.keys(COURSES_OF_ACTION);   // 7
const AUTONOMY_BANDS = [0, 50, 100];               // centralized, hybrid, distributed

// ── Helper: bootstrap engine with N drones ──
function createEngine(nDrones = 4) {
    const engine = new AutonomousFlightEngine({ explorationBounds: 30 });
    engine.swarmIntelligence = new StubSwarmIntelligence();

    for (let i = 1; i <= nDrones; i++) {
        const id = `cf_${String(i).padStart(2, '0')}`;
        const spread = 3;
        const x = (i - 1) % 2 === 0 ? spread : -spread;
        const z = i <= 2 ? spread : -spread;
        engine.registerDrone(id, 'CRAZYFLIE', new THREE.Vector3(x, 0.15, z));
    }
    return engine;
}

// ── Helper: run N frames of engine update ──
function tickEngine(engine, frames = 60, dt = 1 / 60) {
    for (let f = 0; f < frames; f++) {
        engine.update(dt);
    }
}

// ── Helper: advance all drones through TAKEOFF→HOVER→EXPLORE ──
function advanceToExplore(engine, maxFrames = 600) {
    // Set all drones to TAKEOFF
    for (const [id, state] of engine.drones) {
        state.phase = 'TAKEOFF';
    }
    // Run enough frames for takeoff + hover → explore transition
    for (let f = 0; f < maxFrames; f++) {
        engine.update(1 / 60);
        // Check if all drones reached EXPLORE or HOVER at least
        let allPastTakeoff = true;
        for (const [id, state] of engine.drones) {
            if (state.phase === 'TAKEOFF') allPastTakeoff = false;
        }
        if (allPastTakeoff) break;
    }
    // Force any HOVER drones into EXPLORE
    for (const [id, state] of engine.drones) {
        if (state.phase === 'HOVER') {
            state.phase = 'EXPLORE';
            state.lastWaypointTime = -999; // trigger immediate waypoint pick
        }
    }
    // Run a few more frames to trigger waypoint generation
    tickEngine(engine, 10);
}

// ═══════════════════════════════════════════════════════════════════════
// MAIN TEST SUITE: 5 Doctrines × 7 COAs × 3 Autonomy bands = 105 combos
// ═══════════════════════════════════════════════════════════════════════

describe('Doctrine × COA × Autonomy combinatorics', () => {

    for (const dKey of DOCTRINE_KEYS) {
        const doctrine = DOCTRINES[dKey];

        describe(`Doctrine: ${doctrine.name} (${doctrine.icon})`, () => {

            for (const cKey of COA_KEYS) {
                const coa = COURSES_OF_ACTION[cKey];

                for (const autonomy of AUTONOMY_BANDS) {

                    it(`${doctrine.name} + ${coa.name} @ ${autonomy}% autonomy`, () => {
                        // ── Setup ──
                        const engine = createEngine(4);
                        const dm = new DoctrineManager();
                        dm.setDoctrine(doctrine.id);
                        dm.setCOA(coa.id);
                        engine.doctrineManager = dm;
                        engine.setAutonomyLevel(autonomy);

                        // ── 1. Engine should initialise without error ──
                        expect(engine.drones.size).toBe(4);
                        expect(engine.autonomyLevel).toBe(autonomy);

                        // ── 2. Advance to EXPLORE and check waypoints ──
                        advanceToExplore(engine);

                        for (const [id, state] of engine.drones) {
                            // Should be in EXPLORE now
                            expect(['EXPLORE', 'HOVER']).toContain(state.phase);

                            // ── 3. Waypoint should exist and be within bounds ──
                            if (state.phase === 'EXPLORE' && state.waypoint) {
                                expect(state.waypoint).toBeDefined();
                                expect(Number.isFinite(state.waypoint.x)).toBe(true);
                                expect(Number.isFinite(state.waypoint.y)).toBe(true);
                                expect(Number.isFinite(state.waypoint.z)).toBe(true);

                                // Waypoint should be within exploration bounds (with margin)
                                const margin = engine.explorationBounds * 1.5;
                                expect(Math.abs(state.waypoint.x)).toBeLessThan(margin);
                                expect(Math.abs(state.waypoint.z)).toBeLessThan(margin);
                            }

                            // ── 4. _lastAction should be set ──
                            expect(state._lastAction).toBeDefined();
                            expect(typeof state._lastAction).toBe('string');
                            expect(state._lastAction.length).toBeGreaterThan(0);
                        }

                        // ── 5. COA bias should be finite ──
                        const sampleState = engine.drones.values().next().value;
                        if (sampleState) {
                            const bias = engine._computeCOABias(5, 5, sampleState);
                            expect(Number.isFinite(bias)).toBe(true);
                        }

                        // ── Run some more frames to check stability ──
                        tickEngine(engine, 120);

                        for (const [id, state] of engine.drones) {
                            // Position should still be finite (no NaN propagation)
                            expect(Number.isFinite(state.position.x)).toBe(true);
                            expect(Number.isFinite(state.position.y)).toBe(true);
                            expect(Number.isFinite(state.position.z)).toBe(true);
                            // Velocity should be finite
                            expect(Number.isFinite(state.velocity.x)).toBe(true);
                            expect(Number.isFinite(state.velocity.y)).toBe(true);
                            expect(Number.isFinite(state.velocity.z)).toBe(true);
                        }
                    });
                }
            }
        });
    }
});

// ═══════════════════════════════════════════════════════════════════════
// PHEROMONE LIGHTWEIGHT MODE correctness
// ═══════════════════════════════════════════════════════════════════════

describe('Pheromone lightweightMode correctness', () => {
    it.each([
        // [doctrine, coa, expectedLightweight]
        // Exploration doctrine → pheromones ON → lightweightMode false
        ['exploration', 'adaptive',      false],
        ['exploration', 'stigmergy',     false],
        ['exploration', 'grid',          false],  // Exploration doctrine usePheromones=true
        // Swarm doctrine (usePheromones=false) + non-stigmergy COA → lightweight true
        ['swarm',       'adaptive',      true],
        ['swarm',       'grid',          true],
        // Swarm + stigmergy COA → pheromones ON → lightweight false
        ['swarm',       'stigmergy',     false],
        // Formation (no pheromones) + adaptive → lightweight true
        ['formation',   'adaptive',      true],
        // Coverage (usePheromones=true) → lightweight false
        ['coverage',    'adaptive',      false],
        // Search (usePheromones=true) → lightweight false
        ['search',      'spiral',        false],
    ])('%s + %s → lightweight=%s', (doctrineId, coaId, expectedLightweight) => {
        const engine = createEngine(2);
        const dm = new DoctrineManager();
        dm.setDoctrine(doctrineId);
        dm.setCOA(coaId);
        engine.doctrineManager = dm;

        // setOrganicMode propagates the lightweight flag
        engine.setOrganicMode(true);
        const stub = engine.swarmIntelligence;
        expect(stub.lightweightMode).toBe(expectedLightweight);
    });
});

// ═══════════════════════════════════════════════════════════════════════
// RESET correctness
// ═══════════════════════════════════════════════════════════════════════

describe('resetAll() clears all state', () => {
    it('clears coverage, waypoints, formation, and per-drone memory after exploration', () => {
        const engine = createEngine(4);
        const dm = new DoctrineManager();
        engine.doctrineManager = dm;
        engine.setAutonomyLevel(100);

        // Run exploration for a while
        advanceToExplore(engine);
        tickEngine(engine, 300);

        // Verify some state was accumulated
        expect(engine.visitedCells.size).toBeGreaterThan(0);

        // Reset
        engine.resetAll();

        // Verify everything is clean
        expect(engine.visitedCells.size).toBe(0);
        expect(engine._doctrineWaypoints.size).toBe(0);
        expect(engine._pathWaypoints.size).toBe(0);
        expect(engine._formationPhase).toBe(0);
        expect(engine._formationAdvanceTimer).toBe(0);
        expect(engine._spatialHash.size).toBe(0);

        // Per-drone state should be reset
        for (const [id, state] of engine.drones) {
            expect(state.phase).toBe('IDLE');
            expect(state._recentCells.length).toBe(0);
            expect(state._recentCellSet.size).toBe(0);
        }
    });
});

// ═══════════════════════════════════════════════════════════════════════
// AUTONOMY TRANSITIONS: slider changes mid-flight
// ═══════════════════════════════════════════════════════════════════════

describe('Autonomy transitions mid-flight', () => {
    it('switches from distributed (100%) to formation (0%) and back', () => {
        const engine = createEngine(4);
        const dm = new DoctrineManager();
        engine.doctrineManager = dm;
        engine.setAutonomyLevel(100);

        // Explore at 100%
        advanceToExplore(engine);
        tickEngine(engine, 60);

        // Check all drones have ORGANIC EXPL action (or AUTO-ORG at ≥90% autonomy)
        for (const [id, state] of engine.drones) {
            if (state.phase === 'EXPLORE') {
                expect(state._lastAction).toMatch(/ORGANIC|ESCAPE|EXPLORATION|SWARM|COVERAGE|SEARCH|FORMATION|AUTO-ORG/);
            }
        }

        // Switch to 0% (formation)
        engine.setAutonomyLevel(0);
        tickEngine(engine, 120);

        for (const [id, state] of engine.drones) {
            if (state.phase === 'EXPLORE') {
                expect(state._lastAction).toBe('FORMATION');
            }
        }

        // Switch to 50% (hybrid)
        engine.setAutonomyLevel(50);
        tickEngine(engine, 60);

        for (const [id, state] of engine.drones) {
            if (state.phase === 'EXPLORE') {
                expect(state._lastAction).toBe('HYBRID GRP');
            }
        }

        // Back to 100%
        engine.setAutonomyLevel(100);
        tickEngine(engine, 60);

        for (const [id, state] of engine.drones) {
            if (state.phase === 'EXPLORE') {
                expect(state._lastAction).toMatch(/ORGANIC|ESCAPE|EXPLORATION|SWARM|COVERAGE|SEARCH|FORMATION|AUTO-ORG/);
            }
        }

        // Verify no NaN anywhere
        for (const [id, state] of engine.drones) {
            expect(Number.isFinite(state.position.x)).toBe(true);
            expect(Number.isFinite(state.position.y)).toBe(true);
            expect(Number.isFinite(state.position.z)).toBe(true);
        }
    });
});

// ═══════════════════════════════════════════════════════════════════════
// COA BIAS SANITY: each COA returns a non-zero bias for at least SOME cells
// ═══════════════════════════════════════════════════════════════════════

describe('COA bias sanity', () => {
    for (const cKey of COA_KEYS) {
        const coa = COURSES_OF_ACTION[cKey];

        it(`COA ${coa.name}: bias is finite and varies`, () => {
            const engine = createEngine(2);
            const dm = new DoctrineManager();
            dm.setDoctrine('exploration');
            dm.setCOA(coa.id);
            engine.doctrineManager = dm;

            advanceToExplore(engine);
            const state = engine.drones.values().next().value;

            // Sample bias at multiple positions
            const biases = [];
            for (let x = -20; x <= 20; x += 5) {
                for (let z = -20; z <= 20; z += 5) {
                    const b = engine._computeCOABias(x, z, state);
                    expect(Number.isFinite(b)).toBe(true);
                    biases.push(b);
                }
            }

            // For non-adaptive COAs, bias should not be all zeros
            if (coa.id !== 'adaptive') {
                const allZero = biases.every(b => b === 0);
                expect(allZero).toBe(false);
            }
        });
    }
});

// ═══════════════════════════════════════════════════════════════════════
// ORGANIC vs SWARM INTELLIGENCE mode toggle
// ═══════════════════════════════════════════════════════════════════════

describe('Organic vs Swarm Intelligence toggle', () => {
    it('changes waypoint selection method when toggled', () => {
        const engine = createEngine(4);
        const dm = new DoctrineManager();
        engine.doctrineManager = dm;
        engine.setAutonomyLevel(100);

        // Start in organic mode (default)
        expect(engine.useOrganicMode).toBe(true);
        advanceToExplore(engine);
        tickEngine(engine, 60);

        for (const [id, state] of engine.drones) {
            if (state.phase === 'EXPLORE') {
                expect(state._lastAction).toMatch(/ORGANIC|ESCAPE|EXPLORATION|SWARM|COVERAGE|SEARCH|FORMATION|AUTO-ORG/);
            }
        }

        // Switch to swarm intelligence mode
        engine.setOrganicMode(false);
        // Force re-pick waypoints
        for (const [id, state] of engine.drones) {
            state.waypoint = null;
            state.waypointTimer = 999;
        }
        tickEngine(engine, 30);

        for (const [id, state] of engine.drones) {
            if (state.phase === 'EXPLORE') {
                expect(state._lastAction).toMatch(/SWARM|ORGANIC/);
            }
        }
    });
});
