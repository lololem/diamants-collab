/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * drone-visual-factory.js — Factory for creating drone 3D meshes with REAL 3D models.
 *
 * Uses ORIGINAL PX4 SITL DAE/STL meshes directly via ColladaLoader + STLLoader:
 *   - X500:      NXP-HGD-CF.dae frame + 5010Base.dae motors + 5010Bell.dae bells + 1345 propellers (STL)
 *   - OAK-D:     OakDLite.dae camera (x500_depth model)
 *   - Crazyflie: cf2_assembly.glb body + cw/ccw propellers (GLB)
 *   - Mavic/Phantom/Generic: High-detail procedural fallback
 *
 * Assembly positions match the real Gazebo SDF specifications from PX4-Autopilot.
 * BSD-3-Clause (PX4 models by Rudis Laboratories), MIT (Crazyflie models by Bitcraze).
 *
 * @module drone-visual-factory
 */

let THREE = (typeof window !== 'undefined' && window.THREE) ? window.THREE : undefined;

function ensureTHREE() {
    if (!THREE && typeof window !== 'undefined' && window.THREE) THREE = window.THREE;
    if (!THREE) throw new Error('THREE not available');
    return THREE;
}

// ─── MODEL LOADERS (ColladaLoader + STLLoader + GLTFLoader) ─────────

let _gltfLoader = null;
let _colladaLoader = null;
let _stlLoader = null;
let _loaderReady = null;

/**
 * Lazily initialise all 3 loaders:
 *   - ColladaLoader for DAE files (X500 frame, motors, OAK-D)
 *   - STLLoader for STL files (propellers)
 *   - GLTFLoader for GLB files (Crazyflie)
 * @returns {Promise<{ gltf: Object|null, collada: Object|null, stl: Object|null }>}
 */
async function ensureLoaders() {
    if (_loaderReady) return _loaderReady;

    _loaderReady = (async () => {
        const results = { gltf: null, collada: null, stl: null };

        // ── ColladaLoader ──
        try {
            const mod = await import('three/addons/loaders/ColladaLoader.js');
            _colladaLoader = new mod.ColladaLoader();
            results.collada = _colladaLoader;

            // Suppress Three.js Z-UP + "File version" warnings from ColladaLoader
            // These are informational, not errors — and they spam the console.
            if (mod.ColladaLoader?.prototype?.parse && !mod.ColladaLoader.__quietPatched) {
                const _origWarn = console.warn;
                const _origLog = console.log;
                const _origParse = mod.ColladaLoader.prototype.parse;
                mod.ColladaLoader.prototype.parse = function(...args) {
                    const _quietWarn = (...wArgs) => {
                        const joined = wArgs.map(String).join(' ');
                        if (joined.includes('Z-UP') || joined.includes('File version') || joined.includes('ColladaLoader')) return;
                        _origWarn.apply(console, wArgs);
                    };
                    const _quietLog = (...lArgs) => {
                        const joined = lArgs.map(String).join(' ');
                        if (joined.includes('File version') || joined.includes('ColladaLoader')) return;
                        _origLog.apply(console, lArgs);
                    };
                    console.warn = _quietWarn;
                    console.log = _quietLog;
                    try { return _origParse.apply(this, args); }
                    finally { console.warn = _origWarn; console.log = _origLog; }
                };
                mod.ColladaLoader.__quietPatched = true;
            }

            console.log('[DroneVisualFactory] ColladaLoader ready');
        } catch (_e1) {
            try {
                const mod2 = await import('https://unpkg.com/three@0.167.1/examples/jsm/loaders/ColladaLoader.js');
                _colladaLoader = new mod2.ColladaLoader();
                results.collada = _colladaLoader;
            } catch (_e2) {
                console.error('[DroneVisualFactory] ColladaLoader unavailable:', _e1?.message);
            }
        }

        // ── STLLoader ──
        try {
            const mod = await import('three/addons/loaders/STLLoader.js');
            _stlLoader = new mod.STLLoader();
            results.stl = _stlLoader;
            console.log('[DroneVisualFactory] STLLoader ready');
        } catch (_e1) {
            try {
                const mod2 = await import('https://unpkg.com/three@0.167.1/examples/jsm/loaders/STLLoader.js');
                _stlLoader = new mod2.STLLoader();
                results.stl = _stlLoader;
            } catch (_e2) {
                console.error('[DroneVisualFactory] STLLoader unavailable:', _e1?.message);
            }
        }

        // ── GLTFLoader (for Crazyflie GLBs) ──
        if (typeof window !== 'undefined' && window.THREE?.GLTFLoader) {
            _gltfLoader = new window.THREE.GLTFLoader();
            results.gltf = _gltfLoader;
        } else {
            try {
                const mod = await import('three/addons/loaders/GLTFLoader.js');
                _gltfLoader = new mod.GLTFLoader();
                results.gltf = _gltfLoader;
                console.log('[DroneVisualFactory] GLTFLoader ready');
            } catch (_e1) {
                try {
                    const mod2 = await import('https://unpkg.com/three@0.167.1/examples/jsm/loaders/GLTFLoader.js');
                    _gltfLoader = new mod2.GLTFLoader();
                    results.gltf = _gltfLoader;
                } catch (_e2) {
                    console.error('[DroneVisualFactory] GLTFLoader unavailable:', _e1?.message);
                }
            }
        }

        return results;
    })();

    return _loaderReady;
}

// Backward compat
async function ensureGLTFLoader() {
    await ensureLoaders();
    return _gltfLoader;
}

// ── Shared model cache (originals for cloning) ──
const _modelCache = new Map();

/**
 * Load a GLB file and cache it. Returns a deep clone.
 * @param {string} path - Asset path relative to project root
 * @returns {Promise<THREE.Group|null>}
 */
async function loadGLB(path) {
    if (_modelCache.has(path)) {
        return _modelCache.get(path).clone(false); // share geometry+material
    }
    await ensureLoaders();
    if (!_gltfLoader) return null;

    try {
        const gltf = await _gltfLoader.loadAsync(path);
        const scene = gltf.scene;
        _modelCache.set(path, scene);
        return scene.clone(false);
    } catch (err) {
        console.warn(`[DroneVisualFactory] Failed to load GLB ${path}:`, err.message);
        return null;
    }
}

/**
 * Load a DAE (Collada) file and cache it. Returns a deep clone.
 * ColladaLoader handles Z-up → Y-up conversion automatically.
 * @param {string} path - Asset path relative to project root
 * @returns {Promise<THREE.Group|null>}
 */
async function loadDAE(path) {
    if (_modelCache.has(path)) {
        return _modelCache.get(path).clone(true); // deep clone: children carry the actual meshes
    }
    await ensureLoaders();
    if (!_colladaLoader) return null;

    try {
        const collada = await _colladaLoader.loadAsync(path);
        const scene = collada.scene;
        // Strip embedded Blender studio lights from .dae models
        // (5010Bell.dae / 5010Base.dae ship with PointLights that cause flickering)
        const lightsToRemove = [];
        scene.traverse(child => {
            if (child.isLight) lightsToRemove.push(child);
        });
        for (const light of lightsToRemove) {
            light.parent?.remove(light);
        }
        if (lightsToRemove.length > 0) {
            console.log(`[DroneVisualFactory] Stripped ${lightsToRemove.length} embedded lights from ${path}`);
        }
        _modelCache.set(path, scene);
        return scene.clone(true);
    } catch (err) {
        console.warn(`[DroneVisualFactory] Failed to load DAE ${path}:`, err.message);
        return null;
    }
}

/**
 * Load a STL file, wrap in a Group with material, cache it. Returns a deep clone.
 * STL files from PX4 are Z-up — rotation is applied to convert to Y-up.
 * @param {string} path - Asset path relative to project root
 * @param {THREE.Material} material - Material for the STL mesh
 * @returns {Promise<THREE.Group|null>}
 */
async function loadSTLAsGroup(path, material) {
    if (_modelCache.has(path)) {
        return _modelCache.get(path).clone(true);
    }
    await ensureLoaders();
    if (!_stlLoader) return null;

    try {
        const T = ensureTHREE();
        const geometry = await _stlLoader.loadAsync(path);
        geometry.computeVertexNormals();
        const mesh = new T.Mesh(geometry, material);
        // STL is Z-up — rotate to Y-up
        mesh.rotation.x = -Math.PI / 2;
        const group = new T.Group();
        group.name = path.split('/').pop().replace('.stl', '');
        group.add(mesh);
        _modelCache.set(path, group);
        return group.clone(true); // deep clone keeps mesh children
    } catch (err) {
        console.warn(`[DroneVisualFactory] Failed to load STL ${path}:`, err.message);
        return null;
    }
}

/**
 * Synchronous cache-only retrieval. Returns clone from cache or null.
 * Only useful AFTER preloadModels() has completed.
 * @param {string} path
 * @returns {THREE.Group|null}
 */
function loadModelSync(path) {
    if (_modelCache.has(path)) {
        return _modelCache.get(path).clone(true); // deep clone keeps mesh children
    }
    return null;
}

// Backward compat alias
function loadGLBSync(path) { return loadModelSync(path); }


// ─── ASSET PATHS ────────────────────────────────────────────────────

const X500_MESHES = {
    frame:     'assets/x500/meshes/NXP-HGD-CF.dae',      // Original PX4 SITL Collada (CF.png carbon fiber texture)
    motorBase: 'assets/x500/meshes/5010Base.dae',        // Original PX4 SITL Collada
    motorBell: 'assets/x500/meshes/5010Bell.dae',        // Original PX4 SITL Collada
    propCW:    'assets/x500/meshes/1345_prop_cw.stl',    // Original PX4 SITL STL
    propCCW:   'assets/x500/meshes/1345_prop_ccw.stl',   // Original PX4 SITL STL
    oakdLite:  'assets/x500/meshes/oakd/OakDLite.dae',   // OAK-D Lite camera (x500_depth model)
};

const CF_MESHES = {
    body:    'assets/crazyflie/meshes/cf2_assembly.glb',
    propCW:  'assets/crazyflie/meshes/cw_prop.glb',
    propCCW: 'assets/crazyflie/meshes/ccw_prop.glb',
};


// ─── COORDINATE SYSTEM ──────────────────────────────────────────────
// DAE meshes: ColladaLoader handles Z-up → Y-up conversion automatically.
// STL meshes: We apply rotation.x = -π/2 during loading.
// SDF assembly positions are still Z-up and must be converted:
//   SDF (x, y, z) → Three.js (x, z, -y)

function sdfToThree(sx, sy, sz) {
    return [sx, sz, -sy];
}


// ─── ASSEMBLY: X500 (PX4/Holybro X500 v2 — model.sdf) ──────────────
//
// Motor bases (base_link children):
//   motor_0: ( 0.174,  0.174, 0.032)  SDF rot_z = -.45
//   motor_1: (-0.174,  0.174, 0.032)
//   motor_2: ( 0.174, -0.174, 0.032)
//   motor_3: (-0.174, -0.174, 0.032)
//
// Rotors (revolute joints on base_link):
//   rotor_0 @ ( 0.174, -0.174, 0.06)  CCW
//   rotor_1 @ (-0.174,  0.174, 0.06)  CCW
//   rotor_2 @ ( 0.174,  0.174, 0.06)  CW
//   rotor_3 @ (-0.174, -0.174, 0.06)  CW
//
// Per rotor: bell @ (0,0,-0.032) relative, prop @ (-0.022,-0.146,-0.016) scale 0.846

async function assembleX500(scale = 10, color = 0x222222) {
    const T = ensureTHREE();

    // Prop material (STL files have no material)
    const propMat = new T.MeshStandardMaterial({
        color: 0x111111, roughness: 0.5, metalness: 0.0, side: T.DoubleSide,
    });

    const [frame, motorBase, motorBell, propCW, propCCW, oakd] = await Promise.all([
        loadDAE(X500_MESHES.frame),
        loadDAE(X500_MESHES.motorBase),
        loadDAE(X500_MESHES.motorBell),
        loadSTLAsGroup(X500_MESHES.propCW, propMat.clone()),
        loadSTLAsGroup(X500_MESHES.propCCW, propMat.clone()),
        loadDAE(X500_MESHES.oakdLite),
    ]);

    if (!frame) return null;

    const group = new T.Group();
    group.name = 'x500_authentic';

    // Inner sub-group: rotates model so SDF +X (forward) aligns with
    // the heading system's +Z (atan2(vx,vz)=0 → face +Z).
    const inner = new T.Group();
    inner.name = 'x500_orient';
    inner.rotation.y = Math.PI / 2;
    group.add(inner);

    // ── Frame (NXP-HGD-CF) ──
    // SDF pose: 0 0 .025 0 0 π
    // IMPORTANT: SDF yaw must go on a WRAPPER group, not on the ColladaLoader
    // scene (which has rotation.x=-π/2). Euler XYZ would flip the mesh.
    const frameWrap = new T.Group();
    frameWrap.position.set(0, 0.025, 0);
    frameWrap.rotation.y = Math.PI;  // SDF rotZ=π → wrapper rotY=π
    frameWrap.add(frame);            // frame keeps ColladaLoader rotation.x=-π/2
    inner.add(frameWrap);

    // ── Motor bases (4×) — SDF: all rotZ = -0.45 ──
    const motorSDF = [
        { sx:  0.174, sy:  0.174, sz: 0.032 },
        { sx: -0.174, sy:  0.174, sz: 0.032 },
        { sx:  0.174, sy: -0.174, sz: 0.032 },
        { sx: -0.174, sy: -0.174, sz: 0.032 },
    ];

    for (const m of motorSDF) {
        if (!motorBase) break;
        const base = motorBase.clone(true);
        const [tx, ty, tz] = sdfToThree(m.sx, m.sy, m.sz);
        const baseWrap = new T.Group();
        baseWrap.position.set(tx, ty, tz);
        baseWrap.rotation.y = -0.45; // SDF rotZ=-0.45 → wrapper rotY=-0.45
        baseWrap.add(base);          // base keeps ColladaLoader rotation.x=-π/2
        inner.add(baseWrap);
    }

    // ── Rotors: bell + propeller (4×, animated) ──
    const rotorDefs = [
        { sx:  0.174, sy: -0.174, sz: 0.06, type: 'ccw', dir:  1 },
        { sx: -0.174, sy:  0.174, sz: 0.06, type: 'ccw', dir:  1 },
        { sx:  0.174, sy:  0.174, sz: 0.06, type: 'cw',  dir: -1 },
        { sx: -0.174, sy: -0.174, sz: 0.06, type: 'cw',  dir: -1 },
    ];

    const propellers = [];

    for (const r of rotorDefs) {
        const [rx, ry, rz] = sdfToThree(r.sx, r.sy, r.sz);
        const rotorGroup = new T.Group();
        rotorGroup.name = `rotor_${r.type}`;
        rotorGroup.position.set(rx, ry, rz);

        // Motor bell
        if (motorBell) {
            const bell = motorBell.clone(true);
            const [bx, by, bz] = sdfToThree(0, 0, -0.032);
            bell.position.set(bx, by, bz);
            rotorGroup.add(bell);
        }

        // Propeller
        const propTemplate = r.type === 'ccw' ? propCCW : propCW;
        if (propTemplate) {
            const prop = propTemplate.clone(true);
            const [px, py, pz] = sdfToThree(-0.022, -0.146, -0.016);
            prop.position.set(px, py, pz);
            prop.scale.setScalar(0.846);
            rotorGroup.add(prop);
        }

        inner.add(rotorGroup);
        propellers.push({ group: rotorGroup, direction: r.dir });
    }

    // Scene scale
    group.scale.setScalar(scale);

    // Team-color emissive tint on the frame
    frame.traverse(child => {
        if (child.isMesh && child.material) {
            const mat = child.material.clone();
            mat.emissive = new T.Color(color);
            mat.emissiveIntensity = 0.08;
            child.material = mat;
        }
    });

    // ── OAK-D Lite camera — front rail mount on NXP HGD frame ──
    // SDF x500_depth: XY from SDF (0.12, 0.03), Z flush on rail mount
    if (oakd) {
        const cam = oakd.clone(true);
        const [ox, oy, oz] = sdfToThree(0.12, 0.03, 0.022);
        cam.position.set(ox, oy, oz);
        inner.add(cam);
    }

    group.userData.propellers = propellers.map(p => p.group);
    group.userData.propDirections = propellers.map(p => p.direction);
    group.userData.isAuthenticModel = true;

    // ── GPU optimisation: freeze world matrices on static sub-parts ──
    // Only propeller groups need per-frame matrix updates (rotation.y).
    // Everything else (frame, motor bases, OAK-D) is static relative to the drone.
    inner.traverse(child => {
        // Disable per-child frustum culling; the parent group handles it
        child.frustumCulled = false;
        // Freeze matrix on non-animated children
        if (!child.userData?.isAnimated) {
            child.matrixAutoUpdate = false;
            child.updateMatrix();
        }
    });
    // Mark propeller groups as animated so they keep updating
    for (const p of propellers) {
        p.group.matrixAutoUpdate = true;
        p.group.userData.isAnimated = true;
    }

    return group;
}


// ─── SYNC ASSEMBLY: X500 (from preloaded cache only) ─────────────────

function assembleX500Sync(scale = 10, color = 0x222222) {
    if (!_modelsPreloaded) return null;
    const T = ensureTHREE();

    const frame = loadModelSync(X500_MESHES.frame);
    const motorBase = loadModelSync(X500_MESHES.motorBase);
    const motorBell = loadModelSync(X500_MESHES.motorBell);
    const propCW = loadModelSync(X500_MESHES.propCW);
    const propCCW = loadModelSync(X500_MESHES.propCCW);
    const oakd = loadModelSync(X500_MESHES.oakdLite);

    if (!frame) return null;

    const group = new T.Group();
    group.name = 'x500_authentic';

    // Inner sub-group: rotates model so SDF +X (forward) aligns with
    // the heading system's +Z (atan2(vx,vz)=0 → face +Z).
    const inner = new T.Group();
    inner.name = 'x500_orient';
    inner.rotation.y = Math.PI / 2;
    group.add(inner);

    // Frame (NXP-HGD-CF) — SDF pose: 0 0 .025 rotZ=π
    // SDF yaw on WRAPPER group, not on ColladaLoader scene (has rotation.x=-π/2)
    const frameWrap = new T.Group();
    frameWrap.position.set(0, 0.025, 0);
    frameWrap.rotation.y = Math.PI;
    frameWrap.add(frame);
    inner.add(frameWrap);

    // Motor bases (4×) — SDF: all rotZ = -0.45
    const motorSDF = [
        { sx:  0.174, sy:  0.174, sz: 0.032 },
        { sx: -0.174, sy:  0.174, sz: 0.032 },
        { sx:  0.174, sy: -0.174, sz: 0.032 },
        { sx: -0.174, sy: -0.174, sz: 0.032 },
    ];
    for (const m of motorSDF) {
        if (!motorBase) break;
        const base = motorBase.clone(true);
        const [tx, ty, tz] = sdfToThree(m.sx, m.sy, m.sz);
        const baseWrap = new T.Group();
        baseWrap.position.set(tx, ty, tz);
        baseWrap.rotation.y = -0.45;
        baseWrap.add(base);
        inner.add(baseWrap);
    }

    // Rotors: bell + propeller (4×)
    const rotorDefs = [
        { sx:  0.174, sy: -0.174, sz: 0.06, type: 'ccw', dir:  1 },
        { sx: -0.174, sy:  0.174, sz: 0.06, type: 'ccw', dir:  1 },
        { sx:  0.174, sy:  0.174, sz: 0.06, type: 'cw',  dir: -1 },
        { sx: -0.174, sy: -0.174, sz: 0.06, type: 'cw',  dir: -1 },
    ];
    const propellers = [];
    for (const r of rotorDefs) {
        const [rx, ry, rz] = sdfToThree(r.sx, r.sy, r.sz);
        const rotorGroup = new T.Group();
        rotorGroup.name = `rotor_${r.type}`;
        rotorGroup.position.set(rx, ry, rz);
        if (motorBell) {
            const bell = motorBell.clone(true);
            const [bx, by, bz] = sdfToThree(0, 0, -0.032);
            bell.position.set(bx, by, bz);
            rotorGroup.add(bell);
        }
        const propTemplate = r.type === 'ccw' ? propCCW : propCW;
        if (propTemplate) {
            const prop = propTemplate.clone(true);
            const [px, py, pz] = sdfToThree(-0.022, -0.146, -0.016);
            prop.position.set(px, py, pz);
            prop.scale.setScalar(0.846);
            rotorGroup.add(prop);
        }
        inner.add(rotorGroup);
        propellers.push({ group: rotorGroup, direction: r.dir });
    }

    group.scale.setScalar(scale);

    // Team-color emissive tint
    frame.traverse(child => {
        if (child.isMesh && child.material) {
            const mat = child.material.clone();
            mat.emissive = new T.Color(color);
            mat.emissiveIntensity = 0.08;
            child.material = mat;
        }
    });

    // OAK-D Lite camera — front rail mount on NXP HGD frame
    if (oakd) {
        const cam = oakd.clone(true);
        const [ox, oy, oz] = sdfToThree(0.12, 0.03, 0.022);
        cam.position.set(ox, oy, oz);
        inner.add(cam);
    }

    group.userData.propellers = propellers.map(p => p.group);
    group.userData.propDirections = propellers.map(p => p.direction);
    group.userData.isAuthenticModel = true;

    // ── GPU optimisation: same as async version ──
    inner.traverse(child => {
        child.frustumCulled = false;
        if (!child.userData?.isAnimated) {
            child.matrixAutoUpdate = false;
            child.updateMatrix();
        }
    });
    for (const p of propellers) {
        p.group.matrixAutoUpdate = true;
        p.group.userData.isAnimated = true;
    }

    return group;
}


// ─── ASSEMBLY: CRAZYFLIE (Bitcraze Crazyflie 2.1 — SDF) ─────────────
//
// Body pose: (0, 0, 0.017425)
// Motor positions (SDF Z-up):
//   m1: ( 0.031, -0.031, 0.021)  CCW
//   m2: (-0.031, -0.031, 0.021)  CW
//   m3: (-0.031,  0.031, 0.021)  CCW
//   m4: ( 0.031,  0.031, 0.021)  CW

async function assembleCrazyflie(scale = 15, color = 0x00ff88) {
    const T = ensureTHREE();

    const [body, propCW, propCCW] = await Promise.all([
        loadGLB(CF_MESHES.body),
        loadGLB(CF_MESHES.propCW),
        loadGLB(CF_MESHES.propCCW),
    ]);

    if (!body) return null;

    const group = new T.Group();
    group.name = 'crazyflie_authentic';

    // ── Body ──
    body.position.set(0, 0.017425, 0);
    group.add(body);

    // ── Propellers (4×) ──
    const propDefs = [
        { sx:  0.031, sy: -0.031, sz: 0.021, type: 'ccw', dir:  1 },
        { sx: -0.031, sy: -0.031, sz: 0.021, type: 'cw',  dir: -1 },
        { sx: -0.031, sy:  0.031, sz: 0.021, type: 'ccw', dir:  1 },
        { sx:  0.031, sy:  0.031, sz: 0.021, type: 'cw',  dir: -1 },
    ];

    const propellers = [];

    for (const p of propDefs) {
        const template = p.type === 'ccw' ? propCCW : propCW;
        if (!template) continue;

        const [tx, ty, tz] = sdfToThree(p.sx, p.sy, p.sz);
        const propGroup = new T.Group();
        propGroup.name = `prop_${p.type}`;
        propGroup.position.set(tx, ty, tz);
        propGroup.add(template.clone(true));

        group.add(propGroup);
        propellers.push({ group: propGroup, direction: p.dir });
    }

    group.scale.setScalar(scale);

    // Team color tint
    body.traverse(child => {
        if (child.isMesh && child.material) {
            const mat = child.material.clone();
            mat.emissive = new T.Color(color);
            mat.emissiveIntensity = 0.12;
            child.material = mat;
        }
    });

    group.userData.propellers = propellers.map(p => p.group);
    group.userData.propDirections = propellers.map(p => p.direction);
    group.userData.isAuthenticModel = true;

    return group;
}


// ─── SYNC ASSEMBLY: CRAZYFLIE (from preloaded cache only) ────────────

function assembleCrazyflieSync(scale = 15, color = 0x00ff88) {
    if (!_modelsPreloaded) return null;
    const T = ensureTHREE();

    const body = loadGLBSync(CF_MESHES.body);
    const propCW = loadGLBSync(CF_MESHES.propCW);
    const propCCW = loadGLBSync(CF_MESHES.propCCW);

    if (!body) return null;

    const group = new T.Group();
    group.name = 'crazyflie_authentic';

    body.position.set(0, 0.017425, 0);
    group.add(body);

    const propDefs = [
        { sx:  0.031, sy: -0.031, sz: 0.021, type: 'ccw', dir:  1 },
        { sx: -0.031, sy: -0.031, sz: 0.021, type: 'cw',  dir: -1 },
        { sx: -0.031, sy:  0.031, sz: 0.021, type: 'ccw', dir:  1 },
        { sx:  0.031, sy:  0.031, sz: 0.021, type: 'cw',  dir: -1 },
    ];
    const propellers = [];
    for (const p of propDefs) {
        const template = p.type === 'ccw' ? propCCW : propCW;
        if (!template) continue;
        const [tx, ty, tz] = sdfToThree(p.sx, p.sy, p.sz);
        const propGroup = new T.Group();
        propGroup.name = `prop_${p.type}`;
        propGroup.position.set(tx, ty, tz);
        propGroup.add(template.clone(true));
        group.add(propGroup);
        propellers.push({ group: propGroup, direction: p.dir });
    }

    group.scale.setScalar(scale);

    body.traverse(child => {
        if (child.isMesh && child.material) {
            const mat = child.material.clone();
            mat.emissive = new T.Color(color);
            mat.emissiveIntensity = 0.12;
            child.material = mat;
        }
    });

    group.userData.propellers = propellers.map(p => p.group);
    group.userData.propDirections = propellers.map(p => p.direction);
    group.userData.isAuthenticModel = true;
    return group;
}


// ─── HIGH-DETAIL PROCEDURAL FALLBACK ─────────────────────────────────
// Multi-layer: body shell, canopy, battery, arms, motors, ESCs,
// proper blade shapes, landing gear, LEDs, GPS antenna.

function createDetailedProceduralQuad(params) {
    const T = ensureTHREE();
    const {
        armLength = 0.25,
        bodyWidth = 0.15,
        bodyHeight = 0.05,
        bodyLength = 0.18,
        propRadius = 0.12,
        propCount = 4,
        color = 0xff4400,
        scale = 1,
        label = 'drone',
        hasLandingGear = true,
    } = params;

    const group = new T.Group();
    group.name = `drone_${label}`;
    const s = scale;

    // ── Layer 1: Main body (rounded cylinder) ──
    const bodyGeo = new T.CylinderGeometry(
        bodyWidth * 0.5 * s, bodyWidth * 0.55 * s, bodyHeight * s, 12
    );
    const bodyMat = new T.MeshPhongMaterial({ color, emissive: color, emissiveIntensity: 0.1, shininess: 60 });
    const bodyMesh = new T.Mesh(bodyGeo, bodyMat);
    bodyMesh.name = 'body_shell';
    group.add(bodyMesh);

    // ── Layer 2: Top canopy dome ──
    const canopyGeo = new T.SphereGeometry(bodyWidth * 0.35 * s, 12, 6, 0, Math.PI * 2, 0, Math.PI * 0.5);
    const canopyMat = new T.MeshPhongMaterial({ color: 0x333333, shininess: 100, transparent: true, opacity: 0.7 });
    const canopy = new T.Mesh(canopyGeo, canopyMat);
    canopy.position.y = bodyHeight * 0.5 * s;
    group.add(canopy);

    // ── Layer 3: Battery pod ──
    const battGeo = new T.BoxGeometry(bodyWidth * 0.5 * s, bodyHeight * 0.4 * s, bodyLength * 0.4 * s);
    const batt = new T.Mesh(battGeo, new T.MeshPhongMaterial({ color: 0x1a1a2e }));
    batt.position.y = -bodyHeight * 0.5 * s;
    group.add(batt);

    // ── Layer 4: Arms + Motors + ESCs + Propellers ──
    const armMat = new T.MeshPhongMaterial({ color: 0x2d2d2d, shininess: 30 });
    const motorMat = new T.MeshPhongMaterial({ color: 0x111111 });
    const bellMat = new T.MeshPhongMaterial({ color: 0x444444, shininess: 80 });
    const escMat = new T.MeshPhongMaterial({ color: 0x003300 });
    const propMat = new T.MeshPhongMaterial({ color: 0x222222, transparent: true, opacity: 0.75, side: T.DoubleSide });

    const armGeo = new T.BoxGeometry(armLength * s, 0.012 * s, 0.018 * s);
    const motorGeo = new T.CylinderGeometry(0.018 * s, 0.018 * s, 0.025 * s, 12);
    const bellGeo = new T.CylinderGeometry(0.016 * s, 0.012 * s, 0.008 * s, 12);
    const escGeo = new T.BoxGeometry(0.025 * s, 0.004 * s, 0.012 * s);

    // Airfoil-like blade shape
    const bLen = propRadius * s;
    const bWid = propRadius * 0.15 * s;
    const bladeShape = new T.Shape();
    bladeShape.moveTo(0, 0);
    bladeShape.quadraticCurveTo(bLen * 0.3, bWid, bLen * 0.85, bWid * 0.6);
    bladeShape.lineTo(bLen, 0);
    bladeShape.quadraticCurveTo(bLen * 0.85, -bWid * 0.3, bLen * 0.3, -bWid * 0.2);
    bladeShape.lineTo(0, 0);
    const bladeGeo = new T.ExtrudeGeometry(bladeShape, { depth: 0.002 * s, bevelEnabled: false });

    const propellers = [];

    for (let i = 0; i < propCount; i++) {
        const angle = (i / propCount) * Math.PI * 2 + Math.PI / 4;
        const ax = Math.cos(angle) * armLength * s * 0.5;
        const az = Math.sin(angle) * armLength * s * 0.5;

        // Arm
        const arm = new T.Mesh(armGeo, armMat);
        arm.position.set(ax * 0.5, 0, az * 0.5);
        arm.lookAt(new T.Vector3(ax, 0, az));
        group.add(arm);

        // ESC on arm
        const esc = new T.Mesh(escGeo, escMat);
        esc.position.set(ax * 0.35, -0.004 * s, az * 0.35);
        esc.lookAt(new T.Vector3(ax, -0.004 * s, az));
        group.add(esc);

        // Motor
        const motor = new T.Mesh(motorGeo, motorMat);
        motor.position.set(ax, bodyHeight * 0.3 * s, az);
        group.add(motor);

        // Motor bell
        const bell = new T.Mesh(bellGeo, bellMat);
        bell.position.set(ax, bodyHeight * 0.3 * s + 0.017 * s, az);
        group.add(bell);

        // Propeller (hub + 2 blades)
        const propGroup = new T.Group();
        propGroup.position.set(ax, bodyHeight * 0.3 * s + 0.022 * s, az);
        propGroup.name = `prop_${i}`;

        const hubGeo = new T.CylinderGeometry(0.006 * s, 0.006 * s, 0.005 * s, 8);
        propGroup.add(new T.Mesh(hubGeo, motorMat));

        for (let b = 0; b < 2; b++) {
            const blade = new T.Mesh(bladeGeo, propMat);
            blade.rotation.y = b * Math.PI;
            blade.rotation.x = 0.05;
            propGroup.add(blade);
        }

        group.add(propGroup);
        propellers.push(propGroup);
    }

    // ── Layer 5: Landing gear ──
    if (hasLandingGear) {
        const legMat = new T.MeshPhongMaterial({ color: 0x333333 });
        const strutGeo = new T.CylinderGeometry(0.003 * s, 0.003 * s, bodyHeight * 2 * s, 6);
        const skidGeo = new T.CylinderGeometry(0.003 * s, 0.003 * s, bodyWidth * 0.8 * s, 6);

        for (const side of [-1, 1]) {
            for (const fwd of [-0.3, 0.3]) {
                const strut = new T.Mesh(strutGeo, legMat);
                strut.position.set(fwd * bodyLength * s, -bodyHeight * 1.2 * s, side * bodyWidth * 0.35 * s);
                group.add(strut);
            }
            const skid = new T.Mesh(skidGeo, legMat);
            skid.rotation.z = Math.PI / 2;
            skid.position.set(0, -bodyHeight * 2.2 * s, side * bodyWidth * 0.35 * s);
            group.add(skid);
        }
    }

    // ── Layer 6: LEDs (green front, red back) ──
    const ledGeo = new T.SphereGeometry(0.005 * s, 6, 4);
    const frontLed = new T.Mesh(ledGeo, new T.MeshPhongMaterial({ color: 0x00ff00, emissive: 0x00ff00, emissiveIntensity: 0.5 }));
    frontLed.position.set(0, 0, -bodyLength * 0.55 * s);
    group.add(frontLed);

    const backLed = new T.Mesh(ledGeo, new T.MeshPhongMaterial({ color: 0xff0000, emissive: 0xff0000, emissiveIntensity: 0.5 }));
    backLed.position.set(0, 0, bodyLength * 0.55 * s);
    group.add(backLed);

    // ── Layer 7: GPS antenna mast ──
    const mastGeo = new T.CylinderGeometry(0.002 * s, 0.002 * s, 0.03 * s, 4);
    const mast = new T.Mesh(mastGeo, new T.MeshPhongMaterial({ color: 0x666666 }));
    mast.position.set(0, bodyHeight * 0.5 * s + 0.015 * s, -bodyLength * 0.15 * s);
    group.add(mast);

    const antennaGeo = new T.CylinderGeometry(0.008 * s, 0.008 * s, 0.004 * s, 8);
    const antenna = new T.Mesh(antennaGeo, new T.MeshPhongMaterial({ color: 0x222222 }));
    antenna.position.set(0, bodyHeight * 0.5 * s + 0.032 * s, -bodyLength * 0.15 * s);
    group.add(antenna);

    group.userData.propellers = propellers;
    group.userData.propDirections = propellers.map((_, i) => (i % 2 === 0 ? 1 : -1));
    group.userData.isAuthenticModel = false;

    return group;
}


// ─── SENSOR FOV VISUALIZATION ────────────────────────────────────────

function createFovCone(sensorConfig, scale) {
    const T = ensureTHREE();
    const group = new T.Group();
    group.name = 'sensor_fov';

    const hFovRad = (sensorConfig.hFov || 127) * Math.PI / 180;
    const vFovRad = (sensorConfig.vFov || 80) * Math.PI / 180;
    const range = Math.min(sensorConfig.maxDepth || 35, 10);

    const farWidth = Math.tan(hFovRad / 2) * range * scale;
    const farHeight = Math.tan(vFovRad / 2) * range * scale;

    const geo = new T.BufferGeometry();
    const vertices = new Float32Array([
        0, 0, 0,
        -farWidth, -farHeight, -range * scale,
         farWidth, -farHeight, -range * scale,
         farWidth,  farHeight, -range * scale,
        -farWidth,  farHeight, -range * scale,
    ]);
    geo.setAttribute('position', new T.BufferAttribute(vertices, 3));
    geo.setIndex([ 0,1, 0,2, 0,3, 0,4, 1,2, 2,3, 3,4, 4,1 ]);

    const cone = new T.LineSegments(geo,
        new T.LineBasicMaterial({ color: 0x00ffff, transparent: true, opacity: 0.3 })
    );
    cone.frustumCulled = false;
    group.add(cone);
    group.visible = false;
    return group;
}


// ─── DRONE VISUAL FACTORY ────────────────────────────────────────────

const meshConstructors = new Map();
let _modelsPreloaded = false;
let _preloadPromise = null;

/**
 * Factory for creating drone 3D meshes based on physics profile.
 * Supports authentic DAE/STL/GLB models + detailed procedural fallback.
 */
export class DroneVisualFactory {

    /**
     * Register a mesh constructor.
     * @param {string} modelId
     * @param {Function} constructor - (profile, scene) => THREE.Group | Promise<THREE.Group>
     */
    static registerMesh(modelId, constructor) {
        meshConstructors.set(modelId, constructor);
    }

    /**
     * Pre-load all models (DAE + STL + GLB). Call once during app init before create().
     * @returns {Promise<void>}
     */
    static async preloadModels() {
        if (_modelsPreloaded) return;
        if (_preloadPromise) return _preloadPromise;

        _preloadPromise = (async () => {
            console.log('[DroneVisualFactory] ⏳ Preloading authentic 3D models...');
            const t0 = performance.now();

            await ensureLoaders();

            const T = ensureTHREE();
            const propMat = new T.MeshStandardMaterial({
                color: 0x111111, roughness: 0.5, metalness: 0.0, side: T.DoubleSide,
            });

            // X500 DAE/STL models
            const x500Promises = [
                loadDAE(X500_MESHES.frame),
                loadDAE(X500_MESHES.motorBase),
                loadDAE(X500_MESHES.motorBell),
                loadSTLAsGroup(X500_MESHES.propCW, propMat.clone()),
                loadSTLAsGroup(X500_MESHES.propCCW, propMat.clone()),
                loadDAE(X500_MESHES.oakdLite),
            ];

            // Crazyflie GLB models
            const cfPromises = Object.values(CF_MESHES).map(p => loadGLB(p));

            const results = await Promise.allSettled([...x500Promises, ...cfPromises]);
            const totalPaths = Object.keys(X500_MESHES).length + Object.keys(CF_MESHES).length;
            const loaded = results.filter(r => r.status === 'fulfilled' && r.value).length;

            _modelsPreloaded = true;
            console.log(`[DroneVisualFactory] ✅ Preloaded ${loaded}/${totalPaths} models in ${(performance.now() - t0).toFixed(0)}ms`);
        })();

        return _preloadPromise;
    }

    /**
     * Create a drone mesh (sync API, backward-compatible).
     * If models are preloaded → uses authentic mesh directly (sync from cache).
     * Otherwise → detailed procedural with async replacement.
     *
     * @param {Object} profile
     * @param {Object} [scene]
     * @returns {{ mesh: THREE.Group, propellers: Array, fovCone: THREE.Group|null }}
     */
    static create(profile, scene = null) {
        const modelId = profile.model || 'generic';

        // ── FAST PATH: sync assembly from preloaded cache ──
        if (_modelsPreloaded) {
            let syncMesh = null;
            try {
                if (modelId === 'x500') {
                    syncMesh = assembleX500Sync(profile.scale || 10, profile.color || 0x222222);
                } else if (modelId === 's500') {
                    // S500 uses same X500 frame (nearly identical Holybro design) with red accent
                    syncMesh = assembleX500Sync(profile.scale || 10, profile.color || 0x881111);
                    if (syncMesh) syncMesh.name = 's500_authentic';
                } else if (modelId === 'crazyflie') {
                    syncMesh = assembleCrazyflieSync(profile.scale || 15, profile.color || 0x00ff88);
                }
            } catch (e) {
                console.warn(`[DroneVisualFactory] Sync assembly failed for ${modelId}:`, e.message);
            }
            if (syncMesh) {
                console.log(`[DroneVisualFactory] ✅ Sync authentic mesh for ${modelId}`);
                const fovCone = DroneVisualFactory._attachFov(syncMesh, profile);
                return { mesh: syncMesh, propellers: syncMesh.userData.propellers || [], fovCone };
            }
        }

        // ── SLOW PATH: async constructor with procedural placeholder ──
        const constructor = meshConstructors.get(modelId);

        let mesh;
        if (constructor) {
            const result = constructor(profile, scene);
            if (result instanceof Promise) {
                // Create a procedural placeholder first
                mesh = createDetailedProceduralQuad({
                    armLength: profile.armLength || 0.1,
                    bodyWidth: Math.max(0.05, (profile.armLength || 0.1) * 0.6),
                    bodyLength: Math.max(0.06, (profile.armLength || 0.1) * 0.7),
                    bodyHeight: Math.max(0.02, (profile.mass || 0.034) * 0.05),
                    propRadius: Math.max(0.03, (profile.armLength || 0.1) * 0.45),
                    propCount: profile.propCount || 4,
                    color: profile.color || 0xffffff,
                    scale: profile.scale || 15,
                    label: profile.id || 'generic',
                });

                // Async-replace when real model loads
                result.then(real => {
                    if (real && mesh.parent) {
                        real.position.copy(mesh.position);
                        real.rotation.copy(mesh.rotation);
                        real.quaternion.copy(mesh.quaternion);
                        mesh.parent.add(real);
                        mesh.parent.remove(mesh);
                        // Propagate propeller refs to the object that references this mesh
                        Object.assign(mesh.userData, real.userData);
                        // Keep a reference for the caller
                        mesh.userData._replacedBy = real;
                    }
                }).catch(err => {
                    console.warn(`[DroneVisualFactory] Async model load failed for ${modelId}:`, err.message);
                });
            } else {
                mesh = result;
            }
        } else {
            mesh = createDetailedProceduralQuad({
                armLength: profile.armLength || 0.1,
                bodyWidth: Math.max(0.05, (profile.armLength || 0.1) * 0.6),
                bodyLength: Math.max(0.06, (profile.armLength || 0.1) * 0.7),
                bodyHeight: Math.max(0.02, (profile.mass || 0.034) * 0.05),
                propRadius: Math.max(0.03, (profile.armLength || 0.1) * 0.45),
                propCount: profile.propCount || 4,
                color: profile.color || 0xffffff,
                scale: profile.scale || 15,
                label: profile.id || 'generic',
            });
        }

        const fovCone = DroneVisualFactory._attachFov(mesh, profile);
        return { mesh, propellers: mesh.userData.propellers || [], fovCone };
    }

    /**
     * Async version — always awaits real model loading.
     * @param {Object} profile
     * @param {Object} [scene]
     * @returns {Promise<{ mesh: THREE.Group, propellers: Array, fovCone: THREE.Group|null }>}
     */
    static async createAsync(profile, scene = null) {
        const modelId = profile.model || 'generic';
        const constructor = meshConstructors.get(modelId);

        let mesh;
        if (constructor) {
            mesh = await Promise.resolve(constructor(profile, scene));
        }
        if (!mesh) {
            mesh = createDetailedProceduralQuad({
                armLength: profile.armLength || 0.1,
                bodyWidth: Math.max(0.05, (profile.armLength || 0.1) * 0.6),
                bodyLength: Math.max(0.06, (profile.armLength || 0.1) * 0.7),
                bodyHeight: Math.max(0.02, (profile.mass || 0.034) * 0.05),
                propRadius: Math.max(0.03, (profile.armLength || 0.1) * 0.45),
                propCount: profile.propCount || 4,
                color: profile.color || 0xffffff,
                scale: profile.scale || 15,
                label: profile.id || 'generic',
            });
        }

        const fovCone = DroneVisualFactory._attachFov(mesh, profile);
        return { mesh, propellers: mesh.userData.propellers || [], fovCone };
    }

    /** @returns {string[]} */
    static listModels() { return [...meshConstructors.keys()]; }

    /** @returns {boolean} */
    static get modelsReady() { return _modelsPreloaded; }

    /**
     * Attach sensor FOV cone to a mesh if profile has camera config.
     * @param {THREE.Group} mesh
     * @param {Object} profile
     * @returns {THREE.Group|null} fovCone or null
     */
    static _attachFov(mesh, profile) {
        const raw = profile._raw;
        if (raw?.sensors?.camera) {
            const fovCone = createFovCone({
                hFov: raw.sensors.hFov || 127,
                vFov: raw.sensors.vFov || 80,
                maxDepth: raw.sensors.depthRange?.[1] || 35,
            }, profile.scale || 15);
            mesh.add(fovCone);
            return fovCone;
        }
        return null;
    }
}


// ─── REGISTER BUILT-IN MODELS ────────────────────────────────────────

// X500 — Authentic PX4/Holybro from Gazebo meshes (NXP-HGD-CF + 5010 motors + 1345 props)
DroneVisualFactory.registerMesh('x500', async (profile) => {
    const real = await assembleX500(profile.scale || 10, profile.color || 0x222222);
    if (real) return real;
    return createDetailedProceduralQuad({
        armLength: 0.25, bodyWidth: 0.18, bodyLength: 0.22, bodyHeight: 0.08,
        propRadius: 0.15, propCount: 4, color: profile.color || 0x222222,
        scale: profile.scale || 10, label: profile.label || 'X500', hasLandingGear: true,
    });
});

// S500 — Holybro S500 patrol drone — nearly identical frame to X500
// Real difference: slightly larger (500mm), red grip tape on landing gear
// Uses the same X500 DAE mesh with a dark red accent color
DroneVisualFactory.registerMesh('s500', async (profile) => {
    const real = await assembleX500(profile.scale || 10, profile.color || 0x881111);
    if (real) {
        real.name = 's500_authentic';
        return real;
    }
    return createDetailedProceduralQuad({
        armLength: 0.27, bodyWidth: 0.18, bodyLength: 0.22, bodyHeight: 0.08,
        propRadius: 0.15, propCount: 4, color: profile.color || 0x881111,
        scale: profile.scale || 10, label: profile.label || 'S500', hasLandingGear: true,
    });
});

// Crazyflie — Authentic Bitcraze CF2.1 from Gazebo meshes
DroneVisualFactory.registerMesh('crazyflie', async (profile) => {
    const real = await assembleCrazyflie(profile.scale || 15, profile.color || 0x00ff88);
    if (real) return real;
    return createDetailedProceduralQuad({
        armLength: 0.046, bodyWidth: 0.04, bodyLength: 0.045, bodyHeight: 0.015,
        propRadius: 0.023, propCount: 4, color: profile.color || 0x00ff88,
        scale: profile.scale || 15, label: profile.label || 'CF2', hasLandingGear: false,
    });
});

// Mavic — Detailed procedural (foldable frame)
DroneVisualFactory.registerMesh('mavic', (profile) => {
    return createDetailedProceduralQuad({
        armLength: 0.16, bodyWidth: 0.08, bodyLength: 0.12, bodyHeight: 0.05,
        propRadius: 0.10, propCount: 4, color: profile.color || 0x888888,
        scale: profile.scale || 18, label: profile.label || 'Mavic', hasLandingGear: false,
    });
});

// Phantom — Detailed procedural (wide frame)
DroneVisualFactory.registerMesh('phantom', (profile) => {
    return createDetailedProceduralQuad({
        armLength: 0.20, bodyWidth: 0.14, bodyLength: 0.16, bodyHeight: 0.07,
        propRadius: 0.12, propCount: 4, color: profile.color || 0xdddddd,
        scale: profile.scale || 20, label: profile.label || 'Phantom', hasLandingGear: true,
    });
});

// Generic fallback
DroneVisualFactory.registerMesh('generic', (profile) => {
    return createDetailedProceduralQuad({
        armLength: profile.armLength || 0.1,
        bodyWidth: Math.max(0.05, (profile.armLength || 0.1) * 0.6),
        bodyLength: Math.max(0.06, (profile.armLength || 0.1) * 0.7),
        bodyHeight: Math.max(0.02, (profile.mass || 0.034) * 0.05),
        propRadius: Math.max(0.03, (profile.armLength || 0.1) * 0.45),
        propCount: profile.propCount || 4, color: profile.color || 0xffffff,
        scale: profile.scale || 15, label: profile.id || 'Drone',
    });
});
