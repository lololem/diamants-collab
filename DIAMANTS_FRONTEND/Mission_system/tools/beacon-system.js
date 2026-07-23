/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — 3D Beacon System
 * =============================
 * Manages visual beacons in the Three.js scene that drones can collaboratively
 * search for. Beacons are visible glowing objects with pulsing animations.
 *
 * Features:
 *   • Click-to-place beacons in 3D scene (raycasting)
 *   • Pulsing ring + vertical light column for visibility
 *   • Proximity detection: drones that get close "find" the beacon
 *   • Found beacons change color (red → green)
 *   • Integration with LLMMissionService for search coordination
 *
 * @module beacon-system
 */

const BEACON_DEFAULTS = {
    color: 0xff3366,               // Magenta/red
    foundColor: 0x10b981,          // Green
    height: 0.5,                   // Beacon object height from ground
    columnHeight: 30,              // Light column height (tall — visible above forest)
    radius: 1.0,                   // Base ring radius (large for visibility)
    pulseSpeed: 2,                 // Pulse animation speed
    detectionRadius: 10,            // Drone must be this close to "find" it (XZ plane, generous for altitude)
    ringSegments: 64,
};

let _beaconCounter = 0;

export class BeaconSystem {
    /**
     * @param {THREE.Scene} scene
     * @param {Object} [options]
     * @param {Function} [options.onFound] - Called when a beacon is found: onFound(beaconId, droneId)
     */
    constructor(scene, options = {}) {
        this.scene = scene;
        this.options = { ...BEACON_DEFAULTS, ...options };
        this.onFound = options.onFound || null;

        /** @type {Map<string, BeaconObject>} */
        this.beacons = new Map();

        /** @type {boolean} placement mode active */
        this.placementMode = false;

        // Raycaster for placement
        this._raycaster = null;
        this._mouse = null;
        this._placementHandler = null;
        this._hoverHandler = null;
        this._ghostBeacon = null;

        this._clock = 0;
        this._animating = false;

        this._initThreeHelpers();
    }

    _initThreeHelpers() {
        const THREE = window.THREE;
        if (!THREE) {
            console.warn('[BeaconSystem] THREE.js not loaded yet');
            return;
        }
        this._raycaster = new THREE.Raycaster();
        this._mouse = new THREE.Vector2();
    }

    // ════════════════════════════════════════════════════════════════════
    //  PUBLIC API
    // ════════════════════════════════════════════════════════════════════

    /**
     * Place a beacon at a specific world position.
     * @param {number} x
     * @param {number} z
     * @param {string} [label]
     * @returns {string} beacon ID
     */
    placeBeacon(x, z, label) {
        const THREE = window.THREE;
        if (!THREE) return null;

        _beaconCounter++;
        const id = `beacon-${_beaconCounter}`;

        const group = new THREE.Group();
        group.position.set(x, this.options.height, z);
        group.name = `beacon_${id}`;

        // ── Inner sphere (core) — rendered through trees ──
        const coreGeo = new THREE.SphereGeometry(0.5, 16, 16);
        const coreMat = new THREE.MeshBasicMaterial({
            color: this.options.color,
            transparent: true,
            opacity: 0.9,
            depthTest: false,
        });
        const core = new THREE.Mesh(coreGeo, coreMat);
        core.renderOrder = 999;
        group.add(core);

        // ── Pulsing ring — rendered through trees ──
        const ringGeo = new THREE.TorusGeometry(this.options.radius, 0.08, 8, this.options.ringSegments);
        const ringMat = new THREE.MeshBasicMaterial({
            color: this.options.color,
            transparent: true,
            opacity: 0.8,
            depthTest: false,
        });
        const ring = new THREE.Mesh(ringGeo, ringMat);
        ring.rotation.x = -Math.PI / 2; // Horizontal
        ring.renderOrder = 999;
        group.add(ring);

        // ── Vertical light column — rendered through trees ──
        const colGeo = new THREE.CylinderGeometry(0.15, 0.15, this.options.columnHeight, 8);
        const colMat = new THREE.MeshBasicMaterial({
            color: this.options.color,
            transparent: true,
            opacity: 0.35,
            depthTest: false,
        });
        const column = new THREE.Mesh(colGeo, colMat);
        column.position.y = this.options.columnHeight / 2;
        column.renderOrder = 999;
        group.add(column);

        // ── Point light for glow (strong) ──
        const light = new THREE.PointLight(this.options.color, 4, 20);
        light.position.y = 1;
        group.add(light);

        // ── Label sprite ──
        const labelSprite = this._createLabelSprite(label || `Balise ${_beaconCounter}`);
        labelSprite.position.y = 2.5;
        group.add(labelSprite);

        this.scene.add(group);

        const beacon = {
            id,
            label: label || `Balise ${_beaconCounter}`,
            group,
            core, ring, column, light, labelSprite,
            position: { x, y: this.options.height, z },
            found: false,
            foundBy: null,
            createdAt: Date.now(),
        };

        this.beacons.set(id, beacon);

        // Notify listeners about the new beacon
        window.dispatchEvent(new CustomEvent('diamants:beacon-placed', {
            detail: { id, x, z },
        }));

        // Start animation loop if not running
        if (!this._animating) this._startAnimation();

        return id;
    }

    /**
     * Remove a beacon from the scene.
     * @param {string} id
     */
    removeBeacon(id) {
        const beacon = this.beacons.get(id);
        if (!beacon) return;

        this.scene.remove(beacon.group);
        // Dispose geometries and materials
        beacon.group.traverse(child => {
            if (child.geometry) child.geometry.dispose();
            if (child.material) {
                if (child.material.map) child.material.map.dispose();
                child.material.dispose();
            }
        });
        this.beacons.delete(id);
    }

    /**
     * Remove all beacons.
     */
    clearAll() {
        for (const id of [...this.beacons.keys()]) {
            this.removeBeacon(id);
        }
    }

    /**
     * Mark a beacon as found (changes color to green).
     * @param {string} id
     * @param {string} [droneId]
     */
    markFound(id, droneId) {
        const beacon = this.beacons.get(id);
        if (!beacon || beacon.found) return;

        beacon.found = true;
        beacon.foundBy = droneId || 'unknown';

        const color = this.options.foundColor;
        beacon.core.material.color.setHex(color);
        beacon.ring.material.color.setHex(color);
        beacon.column.material.color.setHex(color);
        beacon.light.color.setHex(color);

        // Update label
        beacon.group.remove(beacon.labelSprite);
        beacon.labelSprite = this._createLabelSprite(`✔ ${beacon.label}`, '#10b981');
        beacon.labelSprite.position.y = 2.5;
        beacon.group.add(beacon.labelSprite);

        if (this.onFound) {
            this.onFound(id, droneId);
        }
    }

    /**
     * Check proximity of all drones to all beacons.
     * Should be called each frame.
     * @param {Array|Map} drones - drone objects with position.x/y/z
     */
    checkProximity(drones) {
        const R2 = this.options.detectionRadius * this.options.detectionRadius;

        for (const [id, beacon] of this.beacons) {
            if (beacon.found) continue;

            const bx = beacon.position.x;
            const bz = beacon.position.z;

            const iter = drones instanceof Map ? drones.entries() : drones.map((d, i) => [d.id || `drone_${i}`, d]);
            for (const [droneId, drone] of iter) {
                const pos = drone.position || drone;
                if (!pos || pos.x === undefined) continue;

                const dx = pos.x - bx;
                const dz = (pos.z || 0) - bz;
                if (dx * dx + dz * dz < R2) {
                    this.markFound(id, droneId);
                    break;
                }
            }
        }
    }

    // ════════════════════════════════════════════════════════════════════
    //  PLACEMENT MODE (click-to-place)
    // ════════════════════════════════════════════════════════════════════

    /**
     * Enter beacon placement mode.
     * Next click on the 3D terrain places a beacon.
     */
    enterPlacementMode() {
        if (this.placementMode) return;
        this.placementMode = true;

        const canvas = window.renderer?.domElement;
        if (!canvas) return;

        canvas.style.cursor = 'crosshair';

        // Ghost beacon for preview
        this._createGhostBeacon();

        this._hoverHandler = (e) => this._onMouseMove(e, canvas);
        this._placementHandler = (e) => this._onPlacementClick(e, canvas);
        this._escHandler = (e) => { if (e.key === 'Escape') this.exitPlacementMode(); };

        canvas.addEventListener('mousemove', this._hoverHandler);
        canvas.addEventListener('click', this._placementHandler);
        document.addEventListener('keydown', this._escHandler);

        window.dispatchEvent(new CustomEvent('diamants:beacon-mode', { detail: { active: true } }));
    }

    /**
     * Exit beacon placement mode.
     */
    exitPlacementMode() {
        if (!this.placementMode) return;
        this.placementMode = false;

        const canvas = window.renderer?.domElement;
        if (canvas) {
            canvas.style.cursor = '';
            canvas.removeEventListener('mousemove', this._hoverHandler);
            canvas.removeEventListener('click', this._placementHandler);
        }
        if (this._escHandler) {
            document.removeEventListener('keydown', this._escHandler);
            this._escHandler = null;
        }

        // Remove ghost
        if (this._ghostBeacon) {
            this.scene.remove(this._ghostBeacon);
            this._ghostBeacon = null;
        }

        window.dispatchEvent(new CustomEvent('diamants:beacon-mode', { detail: { active: false } }));
    }

    _onMouseMove(e, canvas) {
        const THREE = window.THREE;
        if (!THREE || !this._raycaster || !this._ghostBeacon) return;

        const rect = canvas.getBoundingClientRect();
        this._mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        this._mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

        this._raycaster.setFromCamera(this._mouse, window.camera);

        // Intersect with ground plane (y = 0)
        const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
        const intersect = new THREE.Vector3();
        this._raycaster.ray.intersectPlane(plane, intersect);

        if (intersect) {
            this._ghostBeacon.position.set(intersect.x, this.options.height, intersect.z);
            this._ghostBeacon.visible = true;
        }
    }

    _onPlacementClick(e, canvas) {
        const THREE = window.THREE;
        if (!THREE || !this._raycaster) return;

        // Ignore clicks on UI elements
        if (e.target !== canvas) return;

        const rect = canvas.getBoundingClientRect();
        this._mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        this._mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

        this._raycaster.setFromCamera(this._mouse, window.camera);
        const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
        const pt = new THREE.Vector3();
        this._raycaster.ray.intersectPlane(plane, pt);

        if (pt) {
            const id = this.placeBeacon(pt.x, pt.z);
            window.dispatchEvent(new CustomEvent('diamants:beacons-updated'));
            // Stay in placement mode — user exits via button or Escape
        }
    }

    _createGhostBeacon() {
        const THREE = window.THREE;
        if (!THREE) return;

        const geo = new THREE.SphereGeometry(0.4, 12, 12);
        const mat = new THREE.MeshBasicMaterial({
            color: this.options.color,
            transparent: true,
            opacity: 0.4,
            wireframe: true,
        });
        this._ghostBeacon = new THREE.Mesh(geo, mat);
        this._ghostBeacon.visible = false;
        this.scene.add(this._ghostBeacon);
    }

    // ════════════════════════════════════════════════════════════════════
    //  LABEL SPRITES
    // ════════════════════════════════════════════════════════════════════

    _createLabelSprite(text, color = '#ff3366') {
        const THREE = window.THREE;
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 64;
        const ctx = canvas.getContext('2d');

        ctx.fillStyle = 'rgba(0,0,0,0.6)';
        ctx.roundRect(0, 0, 256, 64, 8);
        ctx.fill();

        ctx.fillStyle = color;
        ctx.font = 'bold 24px Inter, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(text, 128, 32);

        const tex = new THREE.CanvasTexture(canvas);
        const mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
        const sprite = new THREE.Sprite(mat);
        sprite.scale.set(3, 0.75, 1);
        return sprite;
    }

    // ════════════════════════════════════════════════════════════════════
    //  ANIMATION
    // ════════════════════════════════════════════════════════════════════

    _startAnimation() {
        this._animating = true;
        const animate = () => {
            if (this.beacons.size === 0) {
                this._animating = false;
                return;
            }
            requestAnimationFrame(animate);
            this._clock += 0.016; // ~60fps

            for (const [, beacon] of this.beacons) {
                if (beacon.found) continue;

                const t = this._clock * this.options.pulseSpeed;

                // Pulse ring scale
                const scale = 1 + 0.3 * Math.sin(t * 3);
                beacon.ring.scale.set(scale, scale, 1);
                beacon.ring.material.opacity = 0.4 + 0.3 * Math.sin(t * 2);

                // Rotate core
                beacon.core.rotation.y += 0.02;

                // Pulse column opacity
                beacon.column.material.opacity = 0.15 + 0.1 * Math.sin(t);

                // Pulse light intensity
                beacon.light.intensity = 1.5 + Math.sin(t * 4) * 0.5;
            }
        };
        animate();
    }

    // ════════════════════════════════════════════════════════════════════
    //  SERIALIZATION
    // ════════════════════════════════════════════════════════════════════

    getState() {
        return [...this.beacons.values()].map(b => ({
            id: b.id,
            label: b.label,
            position: b.position,
            found: b.found,
            foundBy: b.foundBy,
        }));
    }
}
