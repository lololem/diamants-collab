/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Drone Physics Registry
 * ====================================
 *
 * Charge dynamiquement les profils de drones depuis physics/profiles/*.json.
 * Chaque droniste dépose un fichier JSON pour intégrer son drone.
 *
 * Usage :
 *   import { DronePhysicsRegistry } from '../physics/drone-physics-registry.js';
 *   const registry = DronePhysicsRegistry.getInstance();
 *   const profile = registry.getProfile('CRAZYFLIE');
 *   registry.registerCustomProfile({ id: 'MY_DRONE', ... });
 */

// ─── Profiles discovered in physics/profiles/ ────────────────────────
/*
 * DROP A FILE, GET A DRONE.
 *
 * The five built-ins used to be imported one by one, which quietly broke the
 * promise made in the README: a contributor who dropped their own JSON in
 * physics/profiles/ saw nothing happen, because nothing imported it.
 *
 * Vite resolves this glob at build time and the dev server re-resolves it on
 * restart, so every *.json in that folder is loaded — yours included. The
 * schema file is skipped; a malformed profile is reported and ignored rather
 * than taking the whole fleet down with it.
 */
const PROFILE_FILES = import.meta.glob('./profiles/*.json', { query: '?raw', import: 'default', eager: true });

const BUILT_IN_PROFILES = Object.entries(PROFILE_FILES)
    .filter(([path]) => !path.endsWith('drone-profile.schema.json'))
    .map(([path, raw]) => {
        try {
            return JSON.parse(raw);
        } catch (e) {
            console.warn(`[DronePhysicsRegistry] ${path} is not valid JSON, ignored:`, e.message);
            return null;
        }
    })
    .filter(Boolean);

// ─── Default values for optional fields ─────────────────────────────
const DEFAULTS = {
    physical: { propCount: 4, armLength: 0.1 },
    performance: { agility: 1.0, explorationRadius: 40, endurance_min: 10 },
    pid: {
        pos: { kp: 2.0, ki: 0.03, kd: 1.0 },
        alt: { kp: 3.0, ki: 0.08, kd: 1.2 },
        yaw: { kp: 1.5, ki: 0.0,  kd: 0.2 },
    },
    visual: { scale: 15, color: '0xFFFFFF', model: 'generic' },
};

/**
 * Normalise un profil JSON en flat profile compatible avec AutonomousFlightEngine.
 * Résultat identique à l'ancien DRONE_PROFILES[key].
 */
function normalizeProfile(raw) {
    const perf  = { ...DEFAULTS.performance, ...raw.performance };
    const phys  = { ...DEFAULTS.physical, ...raw.physical };
    const pid   = {
        pos: { ...DEFAULTS.pid.pos, ...(raw.pid?.pos || {}) },
        alt: { ...DEFAULTS.pid.alt, ...(raw.pid?.alt || {}) },
        yaw: { ...DEFAULTS.pid.yaw, ...(raw.pid?.yaw || {}) },
    };
    const vis   = { ...DEFAULTS.visual, ...(raw.visual || {}) };

    // Flat profile — exact same shape as legacy DRONE_PROFILES entries
    return {
        // Meta
        id: raw.id,
        label: raw.label || raw.id,
        manufacturer: raw.manufacturer || 'Unknown',
        category: raw.category || 'custom',

        // Physics (flat, matches v0-origin shape)
        mass: phys.mass,
        maxSpeed: perf.maxSpeed,
        maxClimb: perf.maxClimb,
        cruiseAlt: perf.cruiseAlt,
        maxAlt: perf.maxAlt,
        agility: perf.agility,
        boundingRadius: phys.boundingRadius,
        explorationRadius: perf.explorationRadius,
        endurance_min: perf.endurance_min,
        armLength: phys.armLength,
        propCount: phys.propCount,

        // PID (same nested shape as v0-origin)
        pid,

        // Visual
        scale: vis.scale,
        color: typeof vis.color === 'string' ? parseInt(vis.color, 16) : vis.color,
        model: vis.model,

        // Raw source for advanced use
        _raw: raw,
    };
}

// ─── Singleton Registry ──────────────────────────────────────────────
let _instance = null;

export class DronePhysicsRegistry {
    constructor() {
        /** @type {Map<string, object>} */
        this._profiles = new Map();
        this._loadOrder = [];

        // Load built-in profiles
        for (const raw of BUILT_IN_PROFILES) {
            this._register(raw, 'built-in');
        }

        console.log(`📋 DronePhysicsRegistry: ${this._profiles.size} profiles loaded (${this._loadOrder.join(', ')})`);
    }

    static getInstance() {
        if (!_instance) _instance = new DronePhysicsRegistry();
        return _instance;
    }

    // ─── Core API ────────────────────────────────────────────────────

    /**
     * Get a normalised profile by ID (e.g. 'CRAZYFLIE', 'MAVIC').
     * Falls back to CRAZYFLIE if unknown.
     */
    getProfile(id) {
        return this._profiles.get(id) || this._profiles.get('CRAZYFLIE');
    }

    /**
     * Get all loaded profiles as { ID: profile } object.
     * Drop-in replacement for the old DRONE_PROFILES export.
     */
    getAllProfiles() {
        const out = {};
        for (const [id, profile] of this._profiles) out[id] = profile;
        return out;
    }

    /**
     * List available profile IDs.
     */
    listProfiles() {
        return Array.from(this._profiles.keys());
    }

    /**
     * Register a custom profile at runtime.
     * A droniste can call this with a raw JSON object.
     */
    registerCustomProfile(raw) {
        this._register(raw, 'custom');
        console.log(`📋 DronePhysicsRegistry: added custom profile "${raw.id}"`);
        return this.getProfile(raw.id);
    }

    /**
     * Load a profile from a URL (fetch JSON at runtime).
     * Useful for loading profiles from a server or user upload.
     */
    async loadProfileFromURL(url) {
        try {
            const resp = await fetch(url);
            if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
            const raw = await resp.json();
            return this.registerCustomProfile(raw);
        } catch (err) {
            console.error(`❌ DronePhysicsRegistry: failed to load ${url}:`, err);
            return null;
        }
    }

    // ─── Internal ────────────────────────────────────────────────────

    _register(raw, source) {
        if (!raw.id) {
            console.warn('DronePhysicsRegistry: profile missing "id", skipping');
            return;
        }
        const profile = normalizeProfile(raw);
        this._profiles.set(raw.id, profile);
        this._loadOrder.push(`${raw.id}[${source}]`);
    }
}

// ─── Backward compatibility ─────────────────────────────────────────
// Drop-in replacement: import { DRONE_PROFILES } from '...'
// This produces the exact same object shape as v0-origin.
export const DRONE_PROFILES = DronePhysicsRegistry.getInstance().getAllProfiles();
