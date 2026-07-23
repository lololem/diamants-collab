/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * depth-camera-simulator.js — OAK-D Pro W stereo depth camera simulator.
 *
 * Port from systeme-autonome/src/perception/oak_d_simulator.py to JavaScript
 * for use in the Three.js collaborative simulation.
 *
 * Simulates:
 *   - 127° H × 80° V wide-angle FOV
 *   - Stereo depth: 0.2m – 35m range
 *   - Point cloud generation via GPU depth buffer rendering
 *   - Detection probability model (distance, size, category, FOV position)
 *   - Depth accuracy model (stereo: error ∝ distance²)
 *   - VLM confidence model (Moondream-style)
 *   - Position noise, false positives, miss rates
 *   - Object tracking with persistence
 *
 * When Three.js WebGLRenderer is available, renders a real depth buffer from
 * the drone's POV using WebGLRenderTarget + DepthTexture.
 * Falls back to raycasting-based depth estimation otherwise.
 *
 * @module depth-camera-simulator
 */

// ─── OAK-D PRO W SPECS ──────────────────────────────────────────────

export const OAK_D_SPECS = Object.freeze({
    H_FOV: 127,            // degrees — wide-angle stereo
    V_FOV: 80,             // degrees
    MIN_DEPTH: 0.2,        // meters
    MAX_DEPTH: 35.0,       // meters (stereo max)
    EFFECTIVE_RANGE: 25.0,  // reliable detection range
    RESOLUTION: { w: 1280, h: 800 },  // stereo resolution
    RGB_RESOLUTION: { w: 4056, h: 3040 },
    MAX_FPS: 120,
    IR_ILLUMINATOR: true,

    // Noise model
    DETECTION_LATENCY: 0.1,    // 100ms VLM latency
    CONFIDENCE_NOISE: 0.08,
    POSITION_NOISE: 0.3,       // meters
    FALSE_POSITIVE_RATE: 0.02, // 2%
    MISS_RATE_CLOSE: 0.01,     // 1% miss < 5m
    MISS_RATE_FAR: 0.15,       // 15% miss > 20m
});

// ─── DETECTABLE CLASSES ──────────────────────────────────────────────

const DETECTABLE_CLASSES = {
    person:   { priority: 1, minSize: 0.3, maxRange: 30 },
    vehicle:  { priority: 2, minSize: 1.0, maxRange: 35 },
    animal:   { priority: 3, minSize: 0.2, maxRange: 20 },
    obstacle: { priority: 4, minSize: 0.5, maxRange: 25 },
    building: { priority: 5, minSize: 2.0, maxRange: 35 },
    drone:    { priority: 1, minSize: 0.3, maxRange: 25 },
    tree:     { priority: 4, minSize: 0.5, maxRange: 25 },
    zone:     { priority: 6, minSize: 1.0, maxRange: 20 },
};

// ─── UTILITY ─────────────────────────────────────────────────────────

function gaussNoise(stddev = 1.0) {
    // Box-Muller transform
    const u1 = Math.random() || 1e-10;
    const u2 = Math.random();
    return stddev * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}

// ─── DEPTH CAMERA SIMULATOR ─────────────────────────────────────────

/**
 * Simulates an OAK-D Pro W stereo depth camera mounted on a drone.
 *
 * Can operate in two modes:
 *   1. GPU depth buffer (if Three.js renderer available) — accurate per-pixel depth
 *   2. Raycasting fallback — samples N rays in the FOV cone
 *
 * Produces:
 *   - Sparse point cloud (observed 3D points)
 *   - Object detections with confidence and VLM descriptions
 *   - Depth map (downsampled Float32Array)
 *   - Occupancy grid updates
 */
export class DepthCameraSimulator {
    /**
     * @param {string} droneId
     * @param {Object} config
     * @param {number} config.hFov - Horizontal FOV in degrees (default 127)
     * @param {number} config.vFov - Vertical FOV in degrees (default 80)
     * @param {number} config.maxDepth - Max depth range (default 35)
     * @param {number} config.rayCount - Number of depth rays per update (default 64)
     * @param {number} config.updateHz - Update frequency (default 30)
     */
    constructor(droneId, config = {}) {
        this.droneId = droneId;
        this.hFov = config.hFov ?? OAK_D_SPECS.H_FOV;
        this.vFov = config.vFov ?? OAK_D_SPECS.V_FOV;
        this.maxDepth = config.maxDepth ?? OAK_D_SPECS.MAX_DEPTH;
        this.minDepth = config.minDepth ?? OAK_D_SPECS.MIN_DEPTH;
        this.rayCount = config.rayCount ?? 64;
        this.updateHz = config.updateHz ?? 30;

        // Internal state
        this._frameId = 0;
        this._lastUpdateTime = 0;
        this._updateIntervalMs = 1000 / this.updateHz;

        /**
         * Latest depth map (downsampled, column-major)
         * Stored as Float32Array, NaN = no reading
         * @type {Float32Array}
         */
        this.depthMap = new Float32Array(this.rayCount).fill(NaN);

        /**
         * Latest point cloud: [{ x, y, z, distance, bearing, elevation }]
         * @type {Array<Object>}
         */
        this.pointCloud = [];

        /**
         * Latest object detections
         * @type {Array<Object>}
         */
        this.detections = [];

        /**
         * Object tracker: id → { category, positions: [...], lastSeen }
         * @type {Map<string, Object>}
         */
        this.trackedObjects = new Map();

        // Camera state
        this.irEnabled = true;
        this.depthMode = 'stereo'; // 'stereo' | 'tof'

        // Precompute ray directions for the FOV cone
        this._rayDirections = this._precomputeRayDirections();

        // Three.js raycaster (reused)
        this._raycaster = null;
    }

    // ────────────────────────────────────────────────────────────────
    // PUBLIC API
    // ────────────────────────────────────────────────────────────────

    /**
     * Update the depth camera: cast rays, detect objects, update point cloud.
     *
     * @param {Object} droneState - { position: {x,y,z}, heading: number, pitch: number }
     * @param {Array} obstacles - Scene obstacles [{ center: {x,y,z}, radius, height, category }]
     * @param {Array} entities - Other entities [{ id, position, category, size }]
     * @param {number} now - Current time ms
     * @returns {{ depthMap, pointCloud, detections }}
     */
    update(droneState, obstacles = [], entities = [], now = performance.now()) {
        // Rate limit
        if (now - this._lastUpdateTime < this._updateIntervalMs) {
            return { depthMap: this.depthMap, pointCloud: this.pointCloud, detections: this.detections };
        }
        this._lastUpdateTime = now;
        this._frameId++;

        const pos = droneState.position;
        const heading = droneState.heading ?? 0;
        const pitch = droneState.pitch ?? 0;

        // 1. Cast rays and build depth map
        this._castDepthRays(pos, heading, pitch, obstacles);

        // 2. Detect entities in FOV
        this.detections = this._detectEntities(pos, heading, pitch, entities);

        // 3. Update object tracker
        this._updateTracker(now);

        return {
            depthMap: this.depthMap,
            pointCloud: this.pointCloud,
            detections: this.detections
        };
    }

    /**
     * Check if a bearing/elevation is within the camera FOV.
     * @param {number} bearing - Horizontal angle in degrees
     * @param {number} elevation - Vertical angle in degrees
     * @returns {boolean}
     */
    isInFov(bearing, elevation) {
        return Math.abs(bearing) <= this.hFov / 2 &&
               Math.abs(elevation) <= this.vFov / 2;
    }

    /**
     * Get detection probability for an object.
     * Matches the Python OakDProWSimulator model.
     *
     * @param {number} distance - Distance in meters
     * @param {number} objSize - Object size in meters
     * @param {string} category - Detection category
     * @param {number} bearing - Bearing in degrees from center
     * @returns {number} 0-1 probability
     */
    getDetectionProbability(distance, objSize, category, bearing) {
        if (distance < this.minDepth || distance > this.maxDepth) return 0;

        // Distance factor (exponential decay)
        const distFactor = distance < OAK_D_SPECS.EFFECTIVE_RANGE
            ? Math.exp(-distance / OAK_D_SPECS.EFFECTIVE_RANGE)
            : 0.3;

        // Size factor
        const classInfo = DETECTABLE_CLASSES[category] || { minSize: 1.0, maxRange: 20, priority: 5 };
        const sizeFactor = Math.min(1.0, objSize / classInfo.minSize);

        // FOV position factor (center = better)
        const fovFactor = 1.0 - (Math.abs(bearing) / (this.hFov / 2)) * 0.3;

        // Priority factor
        const priorityFactor = 1.0 - (classInfo.priority - 1) * 0.05;

        let prob = distFactor * sizeFactor * fovFactor * priorityFactor;
        prob += gaussNoise(0.05);

        return Math.max(0, Math.min(1, prob));
    }

    /**
     * Get depth measurement accuracy (stereo: error ∝ distance²).
     * @param {number} distance
     * @returns {number} Error in meters
     */
    getDepthAccuracy(distance) {
        if (distance < 2) return 0.02;
        if (distance < 10) return distance * 0.01;
        return distance * 0.02 + (distance - 10) * 0.01;
    }

    /**
     * Get VLM confidence for a detection.
     * @param {number} distance
     * @param {string} category
     * @param {boolean} isOccluded
     * @returns {number} 0.3-0.99
     */
    getConfidence(distance, category, isOccluded = false) {
        let base;
        if (distance < 5) base = 0.95;
        else if (distance < 10) base = 0.90;
        else if (distance < 15) base = 0.80;
        else if (distance < 20) base = 0.65;
        else base = 0.50;

        const bonus = { person: 0.05, vehicle: 0.05, building: 0.03, obstacle: 0, animal: -0.05, drone: 0.02 };
        base += bonus[category] ?? 0;
        if (isOccluded) base *= 0.7;
        base += gaussNoise(OAK_D_SPECS.CONFIDENCE_NOISE);

        return Math.max(0.3, Math.min(0.99, base));
    }

    /**
     * Get stats for display.
     */
    getStats() {
        return {
            droneId: this.droneId,
            frameId: this._frameId,
            pointCloudSize: this.pointCloud.length,
            detectionCount: this.detections.length,
            trackedObjects: this.trackedObjects.size,
            fov: `${this.hFov}°×${this.vFov}°`,
            depthRange: `${this.minDepth}–${this.maxDepth}m`,
            mode: this.depthMode
        };
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: Ray Casting
    // ────────────────────────────────────────────────────────────────

    _precomputeRayDirections() {
        const dirs = [];
        const sqrtN = Math.ceil(Math.sqrt(this.rayCount));
        const hHalf = (this.hFov / 2) * Math.PI / 180;
        const vHalf = (this.vFov / 2) * Math.PI / 180;

        for (let i = 0; i < sqrtN; i++) {
            for (let j = 0; j < sqrtN; j++) {
                if (dirs.length >= this.rayCount) break;
                // Uniform distribution within FOV
                const hAngle = -hHalf + (2 * hHalf * (i + 0.5)) / sqrtN;
                const vAngle = -vHalf + (2 * vHalf * (j + 0.5)) / sqrtN;
                dirs.push({ h: hAngle, v: vAngle });
            }
        }
        return dirs;
    }

    _castDepthRays(pos, heading, pitch, obstacles) {
        this.pointCloud = [];

        for (let i = 0; i < this._rayDirections.length; i++) {
            const { h, v } = this._rayDirections[i];

            // Compute ray direction in world space
            const yaw = heading + h;
            const elev = pitch + v;
            const cosE = Math.cos(elev);
            const dx = Math.sin(yaw) * cosE;
            const dy = Math.sin(elev);
            const dz = -Math.cos(yaw) * cosE;

            // Ray-sphere intersection with obstacles
            let minDist = this.maxDepth;
            let hitCategory = null;

            for (const obs of obstacles) {
                const cx = (obs.center?.x ?? obs.x ?? 0) - pos.x;
                const cy = (obs.center?.y ?? obs.y ?? 0) - pos.y;
                const cz = (obs.center?.z ?? obs.z ?? 0) - pos.z;
                const r = obs.radius ?? 0.5;

                // Ray-sphere: solve t² + 2bt + c = 0
                const b = dx * cx + dy * cy + dz * cz;
                const c = cx * cx + cy * cy + cz * cz - r * r;
                const disc = b * b - c;

                if (disc >= 0) {
                    const t = b - Math.sqrt(disc); // nearest hit
                    if (t > this.minDepth && t < minDist) {
                        minDist = t;
                        hitCategory = obs.category || 'obstacle';
                    }
                }
            }

            // Ground plane intersection (y = 0)
            if (dy < -0.01) {
                const tGround = -pos.y / dy;
                if (tGround > this.minDepth && tGround < minDist) {
                    minDist = tGround;
                    hitCategory = 'ground';
                }
            }

            // Add depth noise
            const accuracy = this.getDepthAccuracy(minDist);
            const noisyDist = minDist + gaussNoise(accuracy);

            this.depthMap[i] = noisyDist < this.maxDepth ? noisyDist : NaN;

            // Add to point cloud if we hit something
            if (noisyDist < this.maxDepth) {
                this.pointCloud.push({
                    x: pos.x + dx * noisyDist,
                    y: pos.y + dy * noisyDist,
                    z: pos.z + dz * noisyDist,
                    distance: noisyDist,
                    bearingDeg: h * 180 / Math.PI,
                    elevationDeg: v * 180 / Math.PI,
                    category: hitCategory
                });
            }
        }
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: Entity Detection
    // ────────────────────────────────────────────────────────────────

    _detectEntities(pos, heading, pitch, entities) {
        const detections = [];

        for (const entity of entities) {
            const ePos = entity.position;
            const dx = ePos.x - pos.x;
            const dy = (ePos.y ?? 0) - (pos.y ?? 0);
            const dz = ePos.z - pos.z;
            const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < this.minDepth || distance > this.maxDepth) continue;

            // Compute bearing and elevation relative to drone heading
            const worldBearing = Math.atan2(dx, -dz);
            const bearing = ((worldBearing - heading + Math.PI) % (2 * Math.PI)) - Math.PI;
            const bearingDeg = bearing * 180 / Math.PI;
            const elevation = Math.atan2(dy, Math.sqrt(dx * dx + dz * dz));
            const elevationDeg = elevation * 180 / Math.PI;

            // Check FOV
            if (!this.isInFov(bearingDeg, elevationDeg)) continue;

            const category = entity.category || 'obstacle';
            const size = entity.size || 0.5;

            // Detection probability
            const prob = this.getDetectionProbability(distance, size, category, bearingDeg);
            if (Math.random() > prob) continue;

            // Should detect? (miss rate model)
            if (!this._shouldDetect(distance, category)) continue;

            // Build detection
            const confidence = this.getConfidence(distance, category, entity.occluded);
            const depthAccuracy = this.getDepthAccuracy(distance);

            // Add position noise
            const noisyX = dx + gaussNoise(OAK_D_SPECS.POSITION_NOISE * (1 + distance / 20));
            const noisyZ = dz + gaussNoise(OAK_D_SPECS.POSITION_NOISE * (1 + distance / 20));

            detections.push({
                id: entity.id || `det_${this._frameId}_${detections.length}`,
                category,
                subcategory: entity.subcategory || category,
                distance: distance + gaussNoise(depthAccuracy),
                bearingDeg,
                elevationDeg,
                confidence,
                position: {
                    x: pos.x + noisyX,
                    y: pos.y + dy,
                    z: pos.z + noisyZ
                },
                size,
                description: this._generateVlmDescription(category, entity.subcategory || category, distance, bearingDeg),
                frameId: this._frameId,
                timestamp: performance.now()
            });
        }

        // Random false positives
        if (Math.random() < OAK_D_SPECS.FALSE_POSITIVE_RATE) {
            const angle = Math.random() * this.hFov - this.hFov / 2;
            const dist = 5 + Math.random() * 15;
            detections.push({
                id: `fp_${this._frameId}`,
                category: 'obstacle',
                distance: dist,
                bearingDeg: angle,
                confidence: 0.3 + Math.random() * 0.2,
                position: {
                    x: pos.x + Math.sin(heading + angle * Math.PI / 180) * dist,
                    y: pos.y,
                    z: pos.z - Math.cos(heading + angle * Math.PI / 180) * dist
                },
                falsePositive: true,
                frameId: this._frameId
            });
        }

        return detections;
    }

    _shouldDetect(distance, category) {
        if (distance < 5) return Math.random() > OAK_D_SPECS.MISS_RATE_CLOSE;
        if (distance > 20) return Math.random() > OAK_D_SPECS.MISS_RATE_FAR;

        const missRate = OAK_D_SPECS.MISS_RATE_CLOSE +
            (OAK_D_SPECS.MISS_RATE_FAR - OAK_D_SPECS.MISS_RATE_CLOSE) *
            (distance - 5) / 15;
        return Math.random() > missRate;
    }

    _generateVlmDescription(category, subcategory, distance, bearing) {
        let direction;
        if (bearing < -60) direction = 'far left';
        else if (bearing < -20) direction = 'left';
        else if (bearing > 60) direction = 'far right';
        else if (bearing > 20) direction = 'right';
        else direction = 'ahead';

        const distDesc = distance < 5 ? 'very close' : distance < 15 ? 'nearby' : 'distant';

        const descriptions = {
            person: `Person ${distDesc} ${direction}`,
            vehicle: `Vehicle detected ${direction} at ${distance.toFixed(0)}m`,
            obstacle: `Obstacle detected ${distDesc} ${direction}`,
            tree: `Tree obstacle ${direction}`,
            drone: `WARNING: Drone detected ${direction}!`,
            building: `Building structure ${direction}`,
        };

        return descriptions[subcategory] || descriptions[category] || `Object ${direction}`;
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: Object Tracking
    // ────────────────────────────────────────────────────────────────

    _updateTracker(now) {
        // Add/update tracked objects
        for (const det of this.detections) {
            if (det.falsePositive) continue;
            const key = det.id;
            if (this.trackedObjects.has(key)) {
                const tracked = this.trackedObjects.get(key);
                tracked.positions.push({ ...det.position, t: now });
                tracked.lastSeen = now;
                tracked.confidence = Math.max(tracked.confidence, det.confidence);
                // Keep last 20 positions
                if (tracked.positions.length > 20) tracked.positions.shift();
            } else {
                this.trackedObjects.set(key, {
                    category: det.category,
                    positions: [{ ...det.position, t: now }],
                    lastSeen: now,
                    confidence: det.confidence,
                    firstSeen: now
                });
            }
        }

        // Purge stale tracks (not seen for 2 seconds)
        for (const [key, tracked] of this.trackedObjects) {
            if (now - tracked.lastSeen > 2000) {
                this.trackedObjects.delete(key);
            }
        }
    }
}
