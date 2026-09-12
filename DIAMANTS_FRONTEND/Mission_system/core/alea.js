/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Random draws, with an optional seed
 * ==============================================
 *
 * WHY THIS EXISTS
 *
 * Every decision path drew on `Math.random()`. Two runs of the same
 * configuration therefore diverged, which makes it impossible to compare two
 * versions of an algorithm: you cannot tell whether a difference came from the
 * change or from the draw.
 *
 * THE CHOICE MADE HERE
 *
 * Determinism is NOT imposed. For reinforcement learning, stochasticity is
 * useful — it is what stops agents overfitting a single scenario. With no seed
 * set, this function delegates to `Math.random()` and nothing changes.
 *
 * Setting a seed makes the sequence reproducible, for an A/B run or a debugging
 * session:
 *
 *     DIAMANTS.setSeed(42);     // two identical runs
 *     DIAMANTS.setSeed(null);   // back to free draws
 *
 * The algorithm is mulberry32: thirty-two bits of state, a handful of
 * operations, distribution good enough for simulation noise. It is not a
 * cryptographic generator and does not need to be.
 */

let _etat = null;   // null = no seed, delegate to Math.random()
let _graine = null;

/** Set a seed, or `null` to go back to free draws. */
export function setGraine(n) {
    if (n === null || n === undefined) { _etat = null; _graine = null; return null; }
    _graine = n >>> 0;
    _etat = _graine;
    return _graine;
}

/** The current seed, or `null` when draws are free. */
export function getGraine() { return _graine; }

/** A float in [0, 1). Reproducible when a seed is set. */
export function alea() {
    if (_etat === null) return Math.random();
    _etat = (_etat + 0x6D2B79F5) >>> 0;
    let t = _etat;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}

if (typeof window !== 'undefined') {
    window.DIAMANTS = window.DIAMANTS || {};
    window.DIAMANTS.setSeed = setGraine;
    window.DIAMANTS.getSeed = getGraine;
}
