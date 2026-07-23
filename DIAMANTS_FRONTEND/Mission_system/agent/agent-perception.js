/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * agent-perception.js — Simulated perception system per drone.
 *
 * Each agent has LOCAL perception only (no God's-eye view):
 *   - 5-direction ToF range sensing (raycasting against scene geometry)
 *   - Camera FOV cone (detect objects/other drones)
 *   - Local occupancy grid (log-odds, Bayesian update)
 *   - Frontier detection (cells between known-free and unknown)
 *   - Sensor noise models (Gaussian, range-dependent)
 *
 * Feeds into the 24-dim observation vector consumed by AgentBrain.
 *
 * @module agent-perception
 */

import * as THREE from 'three';
import { OBS_DIM, ObsIndex } from './agent-brain.js';

// ─── SENSOR NOISE ────────────────────────────────────────────────────

/** Box-Muller Gaussian random */
function gaussNoise(stddev) {
    const u1 = Math.random();
    const u2 = Math.random();
    return stddev * Math.sqrt(-2 * Math.log(Math.max(u1, 1e-10))) * Math.cos(2 * Math.PI * u2);
}

// ─── OCCUPANCY GRID ──────────────────────────────────────────────────

/**
 * 2D occupancy grid (top-down) with log-odds representation.
 * Each agent maintains its OWN grid — no shared state.
 */
export class LocalOccupancyGrid {
    /**
     * @param {Object} config
     * @param {number} config.size - Grid size in meters (square)
     * @param {number} config.resolution - Meters per cell
     */
    constructor(config = {}) {
        this.size       = config.size       ?? 80;
        this.resolution = config.resolution ?? 1.0;
        this.cells      = Math.ceil(this.size / this.resolution);
        this.halfSize   = this.size / 2;

        // Log-odds grid: 0 = unknown, >0 = occupied, <0 = free
        this.grid = new Float32Array(this.cells * this.cells);

        // Params
        this.logFree     = -0.3;
        this.logOccupied = 0.7;
        this.logMax      = 3.0;
        this.logMin      = -2.0;

        // Stats
        this.freeCells    = 0;
        this.occupiedCells = 0;
        this.unknownCells  = this.cells * this.cells;

        this._statsFrame = 0;
    }

    /** World position → grid index (or -1) */
    posToIdx(x, z) {
        const cx = Math.floor((x + this.halfSize) / this.resolution);
        const cz = Math.floor((z + this.halfSize) / this.resolution);
        if (cx < 0 || cx >= this.cells || cz < 0 || cz >= this.cells) return -1;
        return cz * this.cells + cx;
    }

    /** Grid coords → world center */
    idxToPos(idx) {
        const cx = idx % this.cells;
        const cz = Math.floor(idx / this.cells);
        return {
            x: (cx + 0.5) * this.resolution - this.halfSize,
            z: (cz + 0.5) * this.resolution - this.halfSize
        };
    }

    /**
     * Mark a ray from drone position to hit point.
     * Cells along the ray are free, endpoint is occupied.
     * @param {number} fromX - Drone x
     * @param {number} fromZ - Drone z
     * @param {number} toX - Hit x
     * @param {number} toZ - Hit z
     * @param {boolean} hitObstacle - True if ray hit something (vs max range)
     */
    markRay(fromX, fromZ, toX, toZ, hitObstacle) {
        // 2D Bresenham-like ray march
        const dx = toX - fromX;
        const dz = toZ - fromZ;
        const dist = Math.sqrt(dx * dx + dz * dz);
        const steps = Math.ceil(dist / this.resolution);
        if (steps <= 0) return;

        const stepX = dx / steps;
        const stepZ = dz / steps;

        // Mark intermediate cells as free
        for (let i = 0; i < steps - 1; i++) {
            const px = fromX + stepX * i;
            const pz = fromZ + stepZ * i;
            const idx = this.posToIdx(px, pz);
            if (idx >= 0) {
                this.grid[idx] = Math.max(this.logMin, this.grid[idx] + this.logFree);
            }
        }

        // Mark endpoint
        if (hitObstacle) {
            const idx = this.posToIdx(toX, toZ);
            if (idx >= 0) {
                this.grid[idx] = Math.min(this.logMax, this.grid[idx] + this.logOccupied);
            }
        }
    }

    /**
     * Mark the area around the drone as explored (free space below).
     * Simplified: mark a circle of radius around the drone as free.
     * @param {number} x
     * @param {number} z
     * @param {number} radius - In meters
     */
    markExplored(x, z, radius = 3) {
        const cellRadius = Math.ceil(radius / this.resolution);
        const cx0 = Math.floor((x + this.halfSize) / this.resolution);
        const cz0 = Math.floor((z + this.halfSize) / this.resolution);

        for (let dz = -cellRadius; dz <= cellRadius; dz++) {
            for (let dx = -cellRadius; dx <= cellRadius; dx++) {
                if (dx * dx + dz * dz > cellRadius * cellRadius) continue;
                const cx = cx0 + dx;
                const cz = cz0 + dz;
                if (cx < 0 || cx >= this.cells || cz < 0 || cz >= this.cells) continue;
                const idx = cz * this.cells + cx;
                this.grid[idx] = Math.max(this.logMin, this.grid[idx] + this.logFree * 0.5);
            }
        }
    }

    /** Occupancy probability at position */
    probability(x, z) {
        const idx = this.posToIdx(x, z);
        if (idx < 0) return 0.5;
        return 1.0 / (1.0 + Math.exp(-this.grid[idx]));
    }

    /** Is the cell free? (probability < 0.3) */
    isFree(x, z) { return this.probability(x, z) < 0.3; }

    /** Is the cell occupied? (probability > 0.7) */
    isOccupied(x, z) { return this.probability(x, z) > 0.7; }

    /** Is the cell unknown? (probability between 0.3 and 0.7) */
    isUnknown(x, z) {
        const p = this.probability(x, z);
        return p >= 0.3 && p <= 0.7;
    }

    /**
     * Find nearest frontier cell (boundary between free and unknown).
     * @param {number} x - Current position
     * @param {number} z
     * @param {number} maxDist - Max search distance in meters
     * @returns {{ x: number, z: number, dist: number } | null}
     */
    findNearestFrontier(x, z, maxDist = 40) {
        const cx0 = Math.floor((x + this.halfSize) / this.resolution);
        const cz0 = Math.floor((z + this.halfSize) / this.resolution);
        const maxCells = Math.ceil(maxDist / this.resolution);

        let bestDist = Infinity;
        let bestPos = null;

        // Spiral outward search
        for (let r = 2; r <= maxCells; r += 2) {
            for (let dz = -r; dz <= r; dz++) {
                for (let dx = -r; dx <= r; dx++) {
                    if (Math.abs(dx) !== r && Math.abs(dz) !== r) continue; // Only ring
                    const cx = cx0 + dx;
                    const cz = cz0 + dz;
                    if (cx < 1 || cx >= this.cells - 1 || cz < 1 || cz >= this.cells - 1) continue;

                    const idx = cz * this.cells + cx;
                    // Frontier = unknown cell adjacent to a free cell
                    if (this.grid[idx] > -0.1 && this.grid[idx] < 0.1) {
                        // Check if any neighbor is free
                        const hasFreeNeighbor =
                            this.grid[idx - 1] < -0.5 ||
                            this.grid[idx + 1] < -0.5 ||
                            this.grid[idx - this.cells] < -0.5 ||
                            this.grid[idx + this.cells] < -0.5;

                        if (hasFreeNeighbor) {
                            const dist = Math.sqrt(dx * dx + dz * dz) * this.resolution;
                            if (dist < bestDist) {
                                bestDist = dist;
                                bestPos = this.idxToPos(idx);
                            }
                        }
                    }
                }
            }
            if (bestPos && bestDist < r * this.resolution * 0.5) break; // Early exit
        }

        return bestPos ? { ...bestPos, dist: bestDist } : null;
    }

    /**
     * Compute local coverage (fraction of known cells in a radius).
     * @param {number} x
     * @param {number} z
     * @param {number} radius - In meters
     * @returns {number} 0..1
     */
    localCoverage(x, z, radius = 15) {
        const cx0 = Math.floor((x + this.halfSize) / this.resolution);
        const cz0 = Math.floor((z + this.halfSize) / this.resolution);
        const cellR = Math.ceil(radius / this.resolution);

        let known = 0, total = 0;
        for (let dz = -cellR; dz <= cellR; dz += 2) {
            for (let dx = -cellR; dx <= cellR; dx += 2) {
                if (dx * dx + dz * dz > cellR * cellR) continue;
                const cx = cx0 + dx;
                const cz = cz0 + dz;
                if (cx < 0 || cx >= this.cells || cz < 0 || cz >= this.cells) continue;
                total++;
                const idx = cz * this.cells + cx;
                if (Math.abs(this.grid[idx]) > 0.3) known++;
            }
        }
        return total > 0 ? known / total : 0;
    }

    /** Update stats (call every ~30 frames) */
    updateStats() {
        this._statsFrame++;
        if (this._statsFrame % 30 !== 0) return;
        let free = 0, occ = 0;
        for (let i = 0; i < this.grid.length; i++) {
            if (this.grid[i] < -0.5) free++;
            else if (this.grid[i] > 0.5) occ++;
        }
        this.freeCells = free;
        this.occupiedCells = occ;
        this.unknownCells = this.grid.length - free - occ;
    }

    /**
     * Merge another agent's grid (for cooperative map sharing).
     * Uses Bayesian fusion (log-odds addition, capped).
     * @param {LocalOccupancyGrid} otherGrid
     * @param {number} trustFactor - 0..1, how much to trust the other grid
     */
    merge(otherGrid, trustFactor = 0.5) {
        if (otherGrid.cells !== this.cells) return;
        for (let i = 0; i < this.grid.length; i++) {
            if (Math.abs(otherGrid.grid[i]) > 0.2) {
                this.grid[i] = Math.max(this.logMin,
                    Math.min(this.logMax, this.grid[i] + otherGrid.grid[i] * trustFactor));
            }
        }
    }

    /**
     * Export grid as compact sparse representation (for communication).
     * Only exports cells with significant information.
     * @returns {{ cells: Array<[number, number]>, size: number, resolution: number }}
     */
    exportSparse() {
        const cells = [];
        for (let i = 0; i < this.grid.length; i++) {
            if (Math.abs(this.grid[i]) > 0.5) {
                cells.push([i, Math.round(this.grid[i] * 10) / 10]);
            }
        }
        return { cells, size: this.size, resolution: this.resolution };
    }

    /**
     * Import sparse grid from another agent.
     * @param {Object} sparse
     * @param {number} trustFactor
     */
    importSparse(sparse, trustFactor = 0.3) {
        if (!sparse?.cells) return;
        for (const [idx, val] of sparse.cells) {
            if (idx >= 0 && idx < this.grid.length) {
                this.grid[idx] = Math.max(this.logMin,
                    Math.min(this.logMax, this.grid[idx] + val * trustFactor));
            }
        }
    }
}


// ─── SIMULATED PERCEPTION ────────────────────────────────────────────

/**
 * SimulatedPerception — Per-drone sensor simulation.
 *
 * Provides LOCAL-ONLY perception:
 *   - 5 range sensors (front, left, right, back, down) via Three.js raycasting
 *   - Camera FOV to detect nearby drones
 *   - Occupancy grid updated from range data
 *   - Frontier detection for exploration
 *   - Builds the 24-dim observation vector for AgentBrain
 */
export class SimulatedPerception {
    /**
     * @param {string} droneId
     * @param {Object} config
     */
    constructor(droneId, config = {}) {
        this.droneId = droneId;

        // Sensor config
        this.rangeMax     = config.rangeMax     ?? 15;    // meters
        this.rangeFront   = config.rangeFront   ?? 15;
        this.rangeSide    = config.rangeSide    ?? 10;
        this.rangeBack    = config.rangeBack    ?? 8;
        this.cameraFOV    = config.cameraFOV    ?? 90;    // degrees
        this.cameraRange  = config.cameraRange  ?? 30;    // meters
        this.noiseStddev  = config.noiseStddev  ?? 0.05;  // meters (base)
        this.noiseProportion = config.noiseProportion ?? 0.02; // proportional to distance

        // Occupancy grid
        this.grid = new LocalOccupancyGrid({
            size: config.gridSize ?? 80,
            resolution: config.gridResolution ?? 1.0
        });

        // Zone size (for normalization)
        this.zoneSize = config.zoneSize ?? 40;  // half-zone in meters
        this.maxSpeed = config.maxSpeed ?? 3.0;  // m/s
        this.maxAlt   = config.maxAlt   ?? 10;   // meters

        // Three.js raycaster (reused)
        this._raycaster = new THREE.Raycaster();
        this._raycaster.far = this.rangeMax;

        // Ray directions (local frame): front, left, right, back, down
        this._rayDirs = [
            new THREE.Vector3(0, 0, -1),  // front (Three.js -Z = forward)
            new THREE.Vector3(-1, 0, 0),  // left
            new THREE.Vector3(1, 0, 0),   // right
            new THREE.Vector3(0, 0, 1),   // back
        ];
        this._rayRanges = [this.rangeFront, this.rangeSide, this.rangeSide, this.rangeBack];

        // Cached range readings
        this.ranges = new Float32Array(4).fill(this.rangeMax);

        // Perceived neighbors
        /** @type {Array<{id: string, x: number, z: number, dist: number, angle: number, vx: number, vz: number}>} */
        this.neighbors = [];

        // Frontier tracking
        this.nearestFrontier = null;
        this._frontierFrame = 0;

        // Observation buffer
        this._obs = new Float32Array(OBS_DIM);
    }

    /**
     * Update perception from the Three.js scene.
     * Call once per frame (or every N frames for performance).
     *
     * @param {Object} droneState - { position: Vector3, velocity: Vector3, heading: number, battery: number, phase: string, phaseTime: number }
     * @param {THREE.Object3D[]} obstacles - Scene objects to raycast against
     * @param {Array<{id: string, position: THREE.Vector3, velocity: THREE.Vector3}>} otherDrones
     * @param {number} globalCoverage - [0,1] estimate
     * @param {string} consensusRole - 'follower' | 'candidate' | 'leader'
     * @returns {Float32Array} 24-dim observation vector
     */
    update(droneState, obstacles, otherDrones, globalCoverage = 0, consensusRole = 'follower') {
        const pos = droneState.position;
        const vel = droneState.velocity;
        const heading = droneState.heading ?? 0;

        // ── Range sensing (raycasting) ──
        if (obstacles && obstacles.length > 0) {
            const cosH = Math.cos(heading);
            const sinH = Math.sin(heading);

            for (let i = 0; i < 4; i++) {
                const localDir = this._rayDirs[i];
                // Rotate local direction by heading
                const worldDir = new THREE.Vector3(
                    localDir.x * cosH - localDir.z * sinH,
                    0,
                    localDir.x * sinH + localDir.z * cosH
                );

                this._raycaster.far = this._rayRanges[i];
                this._raycaster.set(pos, worldDir);
                // Filter: only raycast against actual Three.js Object3D (not plain descriptors)
                const sceneObs = obstacles.filter(o => o && o.isObject3D);
                const hits = sceneObs.length > 0 ? this._raycaster.intersectObjects(sceneObs, true) : [];

                if (hits.length > 0 && hits[0].distance > 0.1) {
                    const rawDist = hits[0].distance;
                    // Add noise (range-dependent)
                    const noisy = rawDist + gaussNoise(this.noiseStddev + rawDist * this.noiseProportion);
                    this.ranges[i] = Math.max(0.1, Math.min(this._rayRanges[i], noisy));

                    // Update occupancy grid
                    const hitPoint = hits[0].point;
                    this.grid.markRay(pos.x, pos.z, hitPoint.x, hitPoint.z, true);
                } else {
                    this.ranges[i] = this._rayRanges[i]; // max range = no obstacle
                    // Mark ray as free
                    const endX = pos.x + worldDir.x * this._rayRanges[i];
                    const endZ = pos.z + worldDir.z * this._rayRanges[i];
                    this.grid.markRay(pos.x, pos.z, endX, endZ, false);
                }
            }
        }

        // Mark drone's immediate area as explored
        this.grid.markExplored(pos.x, pos.z, 3);
        this.grid.updateStats();

        // ── Neighbor detection (camera FOV) ──
        this.neighbors = [];
        if (otherDrones) {
            const halfFOV = (this.cameraFOV / 2) * Math.PI / 180;
            for (const other of otherDrones) {
                if (other.id === this.droneId) continue;
                const dx = other.position.x - pos.x;
                const dz = other.position.z - pos.z;
                const dist = Math.sqrt(dx * dx + dz * dz);
                if (dist > this.cameraRange || dist < 0.1) continue;

                // Check if in FOV
                const angleToOther = Math.atan2(dx, -dz); // Three.js convention
                let relAngle = angleToOther - heading;
                while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
                while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

                // Within FOV or very close (omnidirectional detection within 5m)
                if (Math.abs(relAngle) < halfFOV || dist < 5) {
                    this.neighbors.push({
                        id: other.id,
                        x: other.position.x,
                        z: other.position.z,
                        dist: dist + gaussNoise(0.5), // noisy distance
                        angle: relAngle,
                        vx: other.velocity?.x ?? 0,
                        vz: other.velocity?.z ?? 0
                    });
                }
            }
        }

        // ── Frontier detection (every 10 frames) ──
        this._frontierFrame++;
        if (this._frontierFrame % 10 === 0) {
            this.nearestFrontier = this.grid.findNearestFrontier(pos.x, pos.z);
        }

        // ── Build observation vector ──
        return this._buildObservation(droneState, globalCoverage, consensusRole);
    }

    /**
     * Build the 24-dim normalized observation vector.
     * @returns {Float32Array}
     */
    _buildObservation(state, globalCoverage, consensusRole) {
        const obs = this._obs;
        const pos = state.position;
        const vel = state.velocity;
        const heading = state.heading ?? 0;

        // ── Local state (0-7) ──
        obs[ObsIndex.POS_X_NORM]    = pos.x / this.zoneSize;           // [-1, 1]
        obs[ObsIndex.POS_Z_NORM]    = pos.z / this.zoneSize;           // [-1, 1]
        obs[ObsIndex.HEADING_SIN]   = Math.sin(heading);
        obs[ObsIndex.HEADING_COS]   = Math.cos(heading);
        obs[ObsIndex.SPEED_NORM]    = Math.min(1.0,
            Math.sqrt(vel.x * vel.x + vel.z * vel.z) / this.maxSpeed);
        obs[ObsIndex.BATTERY_NORM]  = state.battery ?? 1.0;
        obs[ObsIndex.ALTITUDE_NORM] = Math.min(1.0, (pos.y ?? 1.5) / this.maxAlt);
        obs[ObsIndex.TIME_IN_PHASE] = Math.min(1.0, (state.phaseTime ?? 0) / 60);

        // ── Perception (8-15) ──
        obs[ObsIndex.OBSTACLE_FRONT] = this.ranges[0] / this.rangeFront;
        obs[ObsIndex.OBSTACLE_LEFT]  = this.ranges[1] / this.rangeSide;
        obs[ObsIndex.OBSTACLE_RIGHT] = this.ranges[2] / this.rangeSide;
        obs[ObsIndex.OBSTACLE_BACK]  = this.ranges[3] / this.rangeBack;

        if (this.nearestFrontier) {
            const fx = this.nearestFrontier.x - pos.x;
            const fz = this.nearestFrontier.z - pos.z;
            const fAngle = Math.atan2(fx, -fz) - heading;
            obs[ObsIndex.FRONTIER_DIR_SIN] = Math.sin(fAngle);
            obs[ObsIndex.FRONTIER_DIR_COS] = Math.cos(fAngle);
            obs[ObsIndex.FRONTIER_DIST]    = Math.min(1.0, this.nearestFrontier.dist / this.zoneSize);
        } else {
            obs[ObsIndex.FRONTIER_DIR_SIN] = 0;
            obs[ObsIndex.FRONTIER_DIR_COS] = 1;
            obs[ObsIndex.FRONTIER_DIST]    = 1;
        }

        obs[ObsIndex.LOCAL_COVERAGE] = this.grid.localCoverage(pos.x, pos.z);

        // ── Multi-agent (16-23) ──
        const maxNeighbors = 8;
        obs[ObsIndex.NEIGHBOR_COUNT] = Math.min(1.0, this.neighbors.length / maxNeighbors);

        if (this.neighbors.length > 0) {
            // Find nearest neighbor
            let nearest = this.neighbors[0];
            for (const n of this.neighbors) {
                if (n.dist < nearest.dist) nearest = n;
            }
            obs[ObsIndex.NEAREST_DIST]    = Math.min(1.0, nearest.dist / this.cameraRange);
            obs[ObsIndex.NEAREST_DIR_SIN] = Math.sin(nearest.angle);
            obs[ObsIndex.NEAREST_DIR_COS] = Math.cos(nearest.angle);
        } else {
            obs[ObsIndex.NEAREST_DIST]    = 1.0;
            obs[ObsIndex.NEAREST_DIR_SIN] = 0;
            obs[ObsIndex.NEAREST_DIR_COS] = 1;
        }

        obs[ObsIndex.SWARM_COVERAGE] = globalCoverage;
        obs[ObsIndex.OVERLAP_RATIO]  = 0; // Computed externally
        obs[ObsIndex.CONSENSUS_ROLE] = consensusRole === 'leader' ? 1.0 :
                                        consensusRole === 'candidate' ? 0.5 : 0;
        obs[ObsIndex.COMM_QUALITY]   = this.neighbors.length > 0 ?
            Math.min(1.0, this.neighbors.length / 3) : 0;

        return obs;
    }

    /**
     * Get perception data formatted for sharing with neighbors.
     * @returns {Object}
     */
    getShareablePerception() {
        return {
            grid: this.grid.exportSparse(),
            frontier: this.nearestFrontier,
            ranges: Array.from(this.ranges),
            neighborCount: this.neighbors.length,
            localCoverage: this.grid.localCoverage(0, 0) // approximate
        };
    }
}
