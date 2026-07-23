/**
 * DIAMANTS — CommWaveRenderer (OPTIMIZED — ultra-lightweight)
 * ═══════════════════════════════════════════════════════════
 * Renders animated visual effects when drones communicate:
 *
 *  • MAP_SYNC  → cyan glowing beam + small travelling pulse
 *  • DIRECTIVE → orange beam (cognitive → ant)
 *  • HEARTBEAT → no 3D visual (too frequent, label-only)
 *
 * Performance budget:
 *   - Pool of 8 reused objects (was 30)
 *   - Single-direction only (no bidirectional spawn)
 *   - Cylinder: 4 radial segments, FrontSide only
 *   - Pulse sphere: 4×3 segments (octahedron-like)
 *   - Zero allocations after warm-up
 *   - No console.log in hot path
 *
 * Toggle: window.DIAMANTS_SHOW_COMM_WAVES (default true)
 */
import * as THREE from 'three';

// ─── Configuration ───────────────────────────────────────────────────
const CFG = {
    POOL_SIZE: 8,            // max concurrent comm visuals (down from 30)
    LIFETIME: 1.6,           // seconds — shorter = less overlap
    TRAVEL_FRACTION: 0.65,   // fraction of lifetime the pulse travels

    BEAM_RADIUS: 0.06,       // metres — thin cylinder
    PULSE_RADIUS: 0.45,      // metres — small sphere

    // Colours & opacity
    SYNC_COLOR:      0x00ffd0,
    DIRECTIVE_COLOR: 0xff8800,
    SYNC_OPACITY:      0.55,
    DIRECTIVE_OPACITY: 0.80,
};

// ─── Shared geometries (created once) ────────────────────────────────
let _cylGeo = null;
let _sphGeo = null;

function getCylGeo() {
    if (!_cylGeo) _cylGeo = new THREE.CylinderGeometry(1, 1, 1, 4, 1); // 4 sides
    return _cylGeo;
}
function getSphGeo() {
    if (!_sphGeo) _sphGeo = new THREE.SphereGeometry(1, 4, 3); // minimal sphere
    return _sphGeo;
}

// ─── Reusable math vectors ──────────────────────────────────────────
const _dir = new THREE.Vector3();
const _mid = new THREE.Vector3();
const _up  = new THREE.Vector3(0, 1, 0);

function orientCylinder(mesh, from, to, radius) {
    _dir.subVectors(to, from);
    const len = _dir.length();
    if (len < 0.01) { mesh.visible = false; return; }
    _mid.addVectors(from, to).multiplyScalar(0.5);
    mesh.position.copy(_mid);
    mesh.scale.set(radius, len, radius);
    _dir.normalize();
    mesh.quaternion.setFromUnitVectors(_up, _dir);
}

// ─── Single comm visual (beam + pulse) ──────────────────────────────
class CommVisual {
    constructor(beam, pulse) {
        this.beam = beam;
        this.pulse = pulse;
        this.active = false;
        this.age = 0;
        this.lifetime = CFG.LIFETIME;
        this.from = new THREE.Vector3();
        this.to = new THREE.Vector3();
        this.fromId = null;
        this.toId = null;
        this.baseOpacity = CFG.SYNC_OPACITY;
    }

    spawn(from, to, type, fromId, toId) {
        this.from.copy(from);
        this.to.copy(to);
        this.fromId = fromId || null;
        this.toId = toId || null;
        this.age = 0;
        this.active = true;
        this.lifetime = CFG.LIFETIME;

        const color = type === 'DIRECTIVE' ? CFG.DIRECTIVE_COLOR : CFG.SYNC_COLOR;
        this.baseOpacity = type === 'DIRECTIVE' ? CFG.DIRECTIVE_OPACITY : CFG.SYNC_OPACITY;

        orientCylinder(this.beam, from, to, CFG.BEAM_RADIUS);
        this.beam.material.color.setHex(color);
        this.beam.material.opacity = this.baseOpacity;
        this.beam.visible = true;

        this.pulse.position.copy(from);
        this.pulse.material.color.setHex(color);
        this.pulse.material.opacity = this.baseOpacity;
        this.pulse.scale.setScalar(CFG.PULSE_RADIUS);
        this.pulse.visible = true;
    }

    /** Update live positions from drone states map */
    syncPositions(droneStates) {
        if (!this.active || !droneStates) return;
        if (this.fromId) {
            const fs = droneStates.get(this.fromId);
            if (fs) this.from.copy(fs.position);
        }
        if (this.toId) {
            const ts = droneStates.get(this.toId);
            if (ts) this.to.copy(ts.position);
        }
    }

    update(dt) {
        if (!this.active) return;
        this.age += dt;
        if (this.age >= this.lifetime) { this.hide(); return; }

        const t = this.age / this.lifetime;

        // Re-orient beam to current live positions
        orientCylinder(this.beam, this.from, this.to, CFG.BEAM_RADIUS);

        // Pulse: travel sender → receiver (ease-out)
        const tt = Math.min(1, t / CFG.TRAVEL_FRACTION);
        const eased = 1 - (1 - tt) * (1 - tt);
        this.pulse.position.lerpVectors(this.from, this.to, eased);

        // Pulse scale: small grow then shrink
        const s = tt < 0.5
            ? CFG.PULSE_RADIUS * (1 + tt)
            : CFG.PULSE_RADIUS * (1.5 - (tt - 0.5));
        this.pulse.scale.setScalar(Math.max(0.15, s));

        // Beam fade-out in second half
        if (t > 0.4) {
            const fade = (t - 0.4) / 0.6;
            const alpha = this.baseOpacity * (1 - fade);
            this.beam.material.opacity = alpha;
            this.pulse.material.opacity = Math.min(alpha * 1.4, this.baseOpacity);
        }
    }

    hide() {
        this.active = false;
        this.beam.visible = false;
        this.pulse.visible = false;
    }
}

// ─── Main renderer ──────────────────────────────────────────────────
export class CommWaveRenderer {
    /** @param {THREE.Scene} scene */
    constructor(scene) {
        this._scene = scene;
        this._pool = [];
        this._enabled = true;
        this._spawnCount = 0;
        this._initPool();
        console.log('[COMM-WAVE] Renderer ready (pool=' + CFG.POOL_SIZE + ')');
    }

    _initPool() {
        const cGeo = getCylGeo();
        const sGeo = getSphGeo();

        for (let i = 0; i < CFG.POOL_SIZE; i++) {
            const bMat = new THREE.MeshBasicMaterial({
                color: CFG.SYNC_COLOR,
                transparent: true,
                opacity: 0,
                depthWrite: false,
                blending: THREE.AdditiveBlending,
                side: THREE.FrontSide,
            });
            const beam = new THREE.Mesh(cGeo, bMat);
            beam.visible = false;
            beam.renderOrder = 900;
            this._scene.add(beam);

            const pMat = new THREE.MeshBasicMaterial({
                color: CFG.SYNC_COLOR,
                transparent: true,
                opacity: 0,
                depthWrite: false,
                blending: THREE.AdditiveBlending,
            });
            const pulse = new THREE.Mesh(sGeo, pMat);
            pulse.visible = false;
            pulse.renderOrder = 901;
            this._scene.add(pulse);

            this._pool.push(new CommVisual(beam, pulse));
        }
    }

    _acquire() {
        for (const v of this._pool) if (!v.active) return v;
        let oldest = this._pool[0];
        for (const v of this._pool) if (v.age > oldest.age) oldest = v;
        oldest.hide();
        return oldest;
    }

    /**
     * @param {THREE.Vector3} fromPos
     * @param {THREE.Vector3} toPos
     * @param {'MAP_SYNC'|'DIRECTIVE'|'HEARTBEAT'} type
     */
    spawnWave(fromPos, toPos, type, fromId, toId) {
        if (!this._enabled) return;
        // HEARTBEAT: skip 3D visual entirely (label-only)
        if (type === 'HEARTBEAT') return;

        this._spawnCount++;
        const v = this._acquire();
        v.spawn(fromPos, toPos, type, fromId, toId);
    }

    /**
     * @param {number} dt
     * @param {Array<{fromId: string, toId: string, type: string}>} pendingWaves
     * @param {Map<string, {position: THREE.Vector3}>} droneStates
     */
    update(dt, pendingWaves, droneStates) {
        if (typeof window !== 'undefined') {
            const show = window.DIAMANTS_SHOW_COMM_WAVES;
            this._enabled = show === undefined ? true : !!show;
        }

        if (this._enabled && pendingWaves && pendingWaves.length > 0) {
            for (const ev of pendingWaves) {
                const fs = droneStates.get(ev.fromId);
                const ts = droneStates.get(ev.toId);
                if (fs && ts) this.spawnWave(fs.position, ts.position, ev.type, ev.fromId, ev.toId);
            }
        }
        if (pendingWaves) pendingWaves.length = 0;

        for (const v of this._pool) {
            if (v.active) {
                if (!this._enabled) { v.hide(); continue; }
                v.syncPositions(droneStates);
                v.update(dt);
            }
        }
    }

    setEnabled(enabled) {
        this._enabled = !!enabled;
        if (typeof window !== 'undefined') window.DIAMANTS_SHOW_COMM_WAVES = this._enabled;
        if (!this._enabled) for (const v of this._pool) v.hide();
    }

    isEnabled() { return this._enabled; }

    dispose() {
        for (const v of this._pool) {
            v.hide();
            if (v.beam.parent) v.beam.parent.remove(v.beam);
            if (v.pulse.parent) v.pulse.parent.remove(v.pulse);
            v.beam.material.dispose();
            v.pulse.material.dispose();
        }
        this._pool.length = 0;
    }
}
