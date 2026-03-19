/**
 * DIAMANTS - Crazyflie Authentique avec Modèles SDF
 * ====================================================
 * Implémentation authentique basée sur les vraies spécifications SDF de DIAMANTS_FRONTEND
 * Utilise les vraies dimensions, masses et positions des moteurs
 */

// Mode silencieux global - mettre à false pour voir les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Système de logging silencieux pour authentic-crazyflie
const SILENT_DRONE_LOGS = false; // Régler à false pour debug
const droneLog = SILENT_DRONE_LOGS ? () => {} : (...args) => console.log(...args);

// Log spécial pour boundary boxes (toujours silencieux maintenant)
const boundaryLog = SILENT_DRONE_LOGS ? () => {} : (...args) => console.log(...args);

// Import du logger pour diagnostics détaillés
import { logger } from '../core/logger.js';

// Import de ray-aabb pour la détection de collision avancée
import createRay from 'ray-aabb';

// IMPORTANT: dans l'app, THREE est chargé via CDN global (r128).
// L'import ESM de 'three' échoue dans le navigateur sans bundler.
// Utiliser une référence MUTABLE afin d'éviter de capturer 'undefined' au chargement du module.
let THREE = (typeof window !== 'undefined' && window.THREE) ? window.THREE : undefined;
// DiamantFormulas module relocated to private repository

export class AuthenticCrazyflie {
    constructor(id, x = 0, y = 3, z = 0, type = 'SCOUT', scene = null) {
        logger.debug('Drone', `🚁 AuthenticCrazyflie.constructor() - Début création drone ${id}`);

        // Assurer que THREE est bien disponible au moment de l'instanciation
        if (!THREE && typeof window !== 'undefined' && window.THREE) {
            THREE = window.THREE;
        }
        if (!THREE) {
            throw new Error('THREE not available when constructing AuthenticCrazyflie');
        }
        // Permettre un constructeur basé sur un objet de config pour compatibilité avec l'application
        if (id && typeof id === 'object') {
            const cfg = id;
            this.id = cfg.id ?? 'drone';
            this.type = (cfg.type || 'SCOUT');
            this.scene = cfg.scene || null;
            const pos = cfg.position || { x: x, y: y, z: z };
            x = pos.x ?? 0; y = pos.y ?? 0; z = pos.z ?? 0;
        } else {
            this.id = id;
            this.type = type;
            this.scene = scene;
        }

        // Configuration visuelle/type
        this.typeConfig = this.getTypeConfiguration(this.type);

        // États d'exploration et de vol
        this.flightState = {
            isFlying: false,
            isExploring: false,
            altitude: 0,
            targetAltitude: 2.0,
            takeoffSpeed: 0.5,
            mode: 'idle', // idle, takeoff, hover, exploration, landing
            power: 0.0    // Puissance moteur [0-1]
        };

        // Exploration de zone
        this.exploration = {
            mode: 'grid',        // grid, boustrophedon, spiral, coverage, random
            bounds: { x: 20, y: 20 },
            gridSize: 2,
            currentTarget: null,
            visitedZones: new Set(),
            path: [],
            pathIndex: 0,
            centerPoint: new THREE.Vector3(0, 0, 0)
        };

        // Animation des pales
        this.propellerAnimation = {
            rotations: [0, 0, 0, 0], // Rotation actuelle de chaque pale
            speeds: [0, 0, 0, 0],    // Vitesse de rotation de chaque pale
            maxSpeed: 120,           // Vitesse max en rad/s (plus réaliste et visible)
            powerMapping: {          // Mapping puissance -> vitesse pale
                idle: 0.1,           // Légère rotation même à l'arrêt
                takeoff: 0.9,        // Puissance élevée pour décollage
                hover: 0.5,          // Vol stationnaire 
                exploration: 0.8,    // Vol actif d'exploration
                landing: 0.3         // Descente contrôlée
            }
        };

        // États cinématiques
        this.position = new THREE.Vector3(x, y, z);
        this.velocity = new THREE.Vector3(0, 0, 0);
        this.acceleration = new THREE.Vector3(0, 0, 0);
        this.bounds = { x: 0.5, y: 0.5 }; // Bounds adaptées à la taille réelle du Crazyflie (50cm de zone de vol)

        // Vraies commandes de vol (comme un vrai drone)
        this.commands = {
            throttle: 0.0, // 0..1 - Poussée verticale
            roll: 0.0,     // rad - Angle de roulis désiré (+ = droite)
            pitch: 0.0,    // rad - Angle de tangage désiré (+ = avant)
            yaw: 0.0       // rad - Angle de lacet désiré (+ = CCW)
        };
        
        // État d'attitude actuel (Euler angles)
        this.attitude = {
            roll: 0,   // rad - Angle de roulis actuel
            pitch: 0,  // rad - Angle de tangage actuel  
            yaw: 0     // rad - Angle de lacet actuel
        };

        // Contrôleurs PID pour position (génèrent les commandes d'attitude)
        this.positionPID = {
            x: { kp: 2.0, ki: 0.1, kd: 0.5, integral: 0, lastError: 0 },
            y: { kp: 4.0, ki: 0.3, kd: 1.2, integral: 0, lastError: 0 }, // Altitude plus réactive
            z: { kp: 2.0, ki: 0.1, kd: 0.5, integral: 0, lastError: 0 }
        };
        
        // Contrôleurs PID pour attitude (génèrent les commandes moteurs)
        this.attitudePID = {
            roll:  { kp: 6.0, ki: 3.0, kd: 0.3, integral: 0, lastError: 0 },
            pitch: { kp: 6.0, ki: 3.0, kd: 0.3, integral: 0, lastError: 0 },
            yaw:   { kp: 4.0, ki: 1.0, kd: 0.2, integral: 0, lastError: 0 }
        };
        
        // Position et vitesse cibles pour les contrôleurs
        this.setpoint = {
            position: new THREE.Vector3(x, y, z),
            velocity: new THREE.Vector3(0, 0, 0),
            yaw: 0
        };

        // Physique authentique (comme un vrai Crazyflie)
        this.mass = 0.034; // kg (Crazyflie 2.0 real mass from Crazyswarm2)
        this.inertiaDiag = new THREE.Vector3(1.4e-5, 1.8e-5, 1.4e-5); // kg·m² (approx)
        this.orientation = new THREE.Quaternion();
        this.angularVelocity = new THREE.Vector3(0, 0, 0); // rad/s (body frame)

        // Spécifications SDF authentiques du Crazyflie (valeurs par défaut réalistes)
        this.specs = {
            body: {
                size: { x: 0.095, y: 0.095, z: 0.029 },
                pose: { x: 0, y: 0, z: 0 }
            },
            motors: [
                // SDF: m1 (0.031, -0.031, 0.021) ccw
                { id: 'M1', pos: { x: +0.031, y: -0.031, z: 0.021 }, dir: 'ccw', prop: 'ccw' },
                // SDF: m2 (-0.031, -0.031, 0.021) cw
                { id: 'M2', pos: { x: -0.031, y: -0.031, z: 0.021 }, dir: 'cw', prop: 'cw' },
                // SDF: m3 (-0.031, +0.031, 0.021) ccw
                { id: 'M3', pos: { x: -0.031, y: +0.031, z: 0.021 }, dir: 'ccw', prop: 'ccw' },
                // SDF: m4 (+0.031, +0.031, 0.021) cw
                { id: 'M4', pos: { x: +0.031, y: +0.031, z: 0.021 }, dir: 'cw', prop: 'cw' }
            ],
            control: {
                // Légère hausse du gain vertical pour un décollage plus net/visible
                velocityGain: { x: 1.0, y: 2.0, z: 1.0 }
            },
            motorParams: {
                maxRotVelocity: 20000, // RPM max réaliste pour Crazyflie
                motorConstant: 1e-5,
                // PARAMÈTRES RÉALISTES basés sur identification système réelle 
                // Conversion RPM->Force: Force(N) = 2.55077341e-08 * rpm² - 4.92422570e-05 * rpm - 1.51910248e-01
                kf: 2.55077341e-08, // N/rpm² coefficient quadratique (principal)
                kf_linear: -4.92422570e-05, // N/rpm coefficient linéaire  
                kf_offset: -1.51910248e-01, // N offset
                km: 4.0e-8  // N·m/(rad/s)^2 yaw drag coefficient
            },
            sensor: {
                maxRange: 3.5
            }
        };

        // Réglages fins d'alignement visuel (axes gris) — laissons 0 par défaut
        this.alignment = {
            radiusAdjust: 0.0, // m, + vers l'extérieur, - vers l'intérieur
            perMotor: [
                { dx: 0.0, dy: 0.0 }, // M1
                { dx: 0.0, dy: 0.0 }, // M2
                { dx: 0.0, dy: 0.0 }, // M3
                { dx: 0.0, dy: 0.0 }  // M4
            ],
            zOffsetCW: 0.0,
            zOffsetCCW: 0.0,
            seatEpsilon: 0.0001,
            useRaycastSnap: true // activer l'ancrage vertical sur l'axe moteur (raycast)
        };

        // Helper ajuste (x,y) radialement + micro offset
        this._adjustXY = (x, y, idx) => {
            const r = Math.hypot(x, y) || 1e-9;
            const nx = x / r, ny = y / r;
            const dx = (this.alignment.perMotor[idx]?.dx || 0);
            const dy = (this.alignment.perMotor[idx]?.dy || 0);
            const x2 = (r + (this.alignment.radiusAdjust || 0)) * nx + dx;
            const y2 = (r + (this.alignment.radiusAdjust || 0)) * ny + dy;
            return { x: x2, y: y2 };
        };

        // Helper: snap group vertically onto body axis using raycast (world Y)
        // Exclude the group's own meshes to avoid self-intersection drift
        this._snapToAxisY = (group) => {
            try {
                if (!this.alignment.useRaycastSnap) return; // optionnel: désactivé par défaut
                const targetParent = this.modelGroup || this.mesh;
                if (!targetParent) return;
                // Ensure in scene for world transforms
                const origin = group.getWorldPosition(new THREE.Vector3());
                origin.y += 0.2; // cast from above
                const dir = new THREE.Vector3(0, -1, 0);
                const ray = new THREE.Raycaster(origin, dir, 0, 1.0);
                const targets = [];
                const isDescendantOf = (ancestor, obj) => {
                    let p = obj;
                    while (p) {
                        if (p === ancestor) return true;
                        p = p.parent;
                    }
                    return false;
                };
                targetParent.traverse((n) => {
                    if (n.isMesh && !isDescendantOf(group, n)) targets.push(n);
                });
                const hits = ray.intersectObjects(targets, true);
                if (hits && hits.length) {
                    const hitY = hits[0].point.y;
                    const gWorld = group.getWorldPosition(new THREE.Vector3());
                    const desired = hitY + (this.alignment.seatEpsilon || 0);
                    const delta = desired - gWorld.y;
                    group.position.y += delta; // local Y; parent is Y-up
                }
            } catch (_) { /* safe */ }
        };
    // État de mission
    this.state = 'IDLE'; // IDLE, TAKEOFF, FLYING, LANDING, EMERGENCY
    this.missionPhase = 'INIT';
    // Assurer une montée visible: viser au moins +1m au-dessus de l'altitude actuelle
    this.targetAltitude = Math.max(3.0, this.position.y + 1.0);
    this.targetPosition = this.position.clone();
    this.targetPosition.y = this.targetAltitude;

        // Propriétés DIAMANTS
        this.phi = 0; // Potentiel attractif
        this.sigma = 0; // Potentiel répulsif
        this.gradient = new THREE.Vector3(0, 0, 0);
        this.intelligence = 0;
        this.emergence = 0;

        // Système de forces de répulsion pour évitement de collision
        this.repulsionForces = new Map(); // Map<droneId, {vector, force, timestamp}>
        this.repulsionDecayRate = 0.95; // Décroissance des forces par frame
        this.maxRepulsionForce = 3.0; // Force maximale de répulsion en m/s
        this.repulsionTimeout = 2000; // Durée de vie d'une force de répulsion en ms

        // Moteurs avec états réels
        this.motors = this.initializeMotors();
        
        // Commandes moteurs en taux angulaires (sorties du contrôleur d'attitude)
        this.motorCommands = {
            rollRate: 0,
            pitchRate: 0,
            yawRate: 0,
            throttle: 0
        };

        // NOTE: on ne réassigne pas this.specs ici pour ne pas perdre les champs (sensor, control...)
        // Si besoin d'ajuster certaines valeurs, on peut le faire de manière ciblée, par ex.:
        // this.specs.body.size = { x: 0.092, y: 0.092, z: 0.029 };

        // Capteurs
        this.sensors = {
            lidar: { ranges: [3.49, 3.49, 3.49, 3.49], lastUpdate: 0 },
            imu: { pitch: 0, roll: 0, yaw: 0 },
            odometry: { x: x, y: y, z: z, vx: 0, vy: 0, vz: 0 }
        };

        // Mesh 3D
        this.mesh = null;
        this.modelGroup = null; // contiendra soit le modèle simplifié, soit le DAE

        // Debug helpers (rate-limited logs, etc.)
        this._debug = {
            lastTakeoffLog: 0
        };
        this.propellers = [];
        this.initializeMesh();

        // Watchdog de mouvement: détecte un drone "figé" et injecte une micro-intention
        this._motionWatch = {
            lastPos: this.position.clone(),
            accTime: 0,
            lastNudge: 0
        };

        // Intelligence collective
        this.swarmMemory = new Map();
        this.expertise = { type: 'general', level: 0 };
        this.lastCommunication = 0;

        // Métriques de performance
        this.metrics = {
            flightTime: 0,
            distance: 0,
            energyUsed: 0,
            collisions: 0,
            discoveries: 0
        };

    // Options d'affichage
    this.allowFallbackProps = true; // active les hélices fallback visibles si DAE indispo
    }

    getTypeConfiguration(type) {
        const configs = {
            SCOUT: {
                scale: 0.6,
                speed: 0.8,
                agility: 1.2,
                batteryLife: 0.8,
                sensorRange: 1.2,
                color: 0x00FF88,
                role: 'exploration'
            },
            HEAVY: {
                scale: 1.0,
                speed: 0.6,
                agility: 0.8,
                batteryLife: 1.2,
                sensorRange: 1.0,
                color: 0xFF4400,
                role: 'transport'
            },
            STEALTH: {
                scale: 0.4,
                speed: 1.0,
                agility: 1.5,
                batteryLife: 0.6,
                sensorRange: 0.8,
                color: 0x4400FF,
                role: 'reconnaissance'
            },
            LEADER: {
                scale: 0.8,
                speed: 0.7,
                agility: 1.0,
                batteryLife: 1.0,
                sensorRange: 1.5,
                color: 0xFFAA00,
                role: 'coordination'
            }
        };

        return configs[type] || configs.SCOUT;
    }

    initializeMotors() {
        return this.specs.motors.map(motorSpec => ({
            id: motorSpec.id,
            position: motorSpec.pos,
            direction: motorSpec.dir,
            propType: motorSpec.prop,
            rpm: 0,
            targetRpm: 0,
            omega: 0,        // Vitesse angulaire actuelle (rad/s) pour PID
            thrust: 0,       // Force de poussée actuelle (N)
            yawTorque: 0,    // Torque de réaction en lacet (N·m)
            torque: 0,
            temperature: 20, // °C
            efficiency: 1.0
        }));
    }

    initializeMesh() {
        logger.debug('Drone', `🚁 initializeMesh() - Début pour drone ${this.id}`);

        // Groupe principal du drone (toujours créé; l'ajout à la scène est optionnel)
        this.mesh = new THREE.Group();
        this.mesh.position.copy(this.position);
        // Échelle d'affichage pour une meilleure visibilité (réaliste ~10cm → trop petit)
        // On grossit le modèle sans changer la physique/logique
        try {
            this.mesh.scale.setScalar(15);
            logger.trace('Drone', `📏 Drone ${this.id}: échelle définie à 15x`);
        } catch (e) {
            logger.warning('Drone', `⚠️ Erreur échelle drone ${this.id}:`, e);
        }

    logger.info('Drone', `🚀 INITIALISATION CRAZYFLIE AUTHENTIQUE ${this.id} - CHARGEMENT DAE PRIORITAIRE (fallback seulement si échec)`);

    // Désactiver la création d'hélices fallback vert fluo
    this.propellers = [];

        // LED de statut (seul élément non-DAE autorisé)
        this.statusLED = this.createStatusLED();
        this.mesh.add(this.statusLED);

    // CHARGEMENT DAE avec logs détaillés et timeout
        log('🔄 Tentative chargement DAE pour drone', this.id);

        const loadingTimeout = setTimeout(() => {
            warn('⏱️ TIMEOUT chargement DAE pour drone', this.id, '- utilisation fallback visible');
        }, 5000);

        this.tryLoadRealMesh().then((ok) => {
            clearTimeout(loadingTimeout);
            if (ok) {
                log('✅ DAE chargé avec succès pour drone', this.id);
            } else {
                throw new Error('DAE non confirmé');
            }
        }).catch(err => {
            clearTimeout(loadingTimeout);
            console.error('❌ ÉCHEC DAE pour drone', this.id, ':', err);
            warn('🟡 Fallback debug créé (cube) car DAE indisponible', this.id);

            // Créer le fallback uniquement en cas d'échec (évite les cubes par défaut)
            try {
                const debugGeometry = new THREE.BoxGeometry(0.12, 0.03, 0.12);
                const debugMaterial = new THREE.MeshPhongMaterial({
                    color: 0xff6600, // Orange pour signaler l'échec
                    transparent: false,
                    opacity: 1.0,
                    visible: true,
                    emissive: 0x221100
                });
                const debugBody = new THREE.Mesh(debugGeometry, debugMaterial);
                // Décaler le cube pour que son bas soit à y=0 en espace local
                // (half-height = 0.03/2 = 0.015)
                debugBody.position.y = 0.015;
                debugBody.userData = { fallbackType: 'body', debug: true, temporary: true };
                debugBody.visible = true;
                debugBody.castShadow = true;
                debugBody.receiveShadow = true;
                this.mesh.add(debugBody);
                logger.debug('Drone', `✅ Drone ${this.id}: fallback mesh créé (échec DAE)`);
            } catch (_) { /* safe */ }
        });

        // Ajout à la scène si fournie; sinon, l'appelant peut récupérer le mesh via getMesh()
        if (this.scene) {
            this.scene.add(this.mesh);
        }
    }

    async ensureColladaLoader() {
        logger.debug('Drone', `🔧 ensureColladaLoader() - Vérification ColladaLoader`);

        // 1) Préférence: import ESM aligné avec la version de three utilisée par Vite
        if (!this._ColladaLoaderClass) {
            try {
                const mod = await import('three/addons/loaders/ColladaLoader.js');
                if (mod && mod.ColladaLoader) {
                    this._ColladaLoaderClass = mod.ColladaLoader;
                    // Also expose globally so diagnostics can detect it
                    try {
                        if (typeof window !== 'undefined') {
                            window.THREE = window.THREE || THREE;
                            if (window.THREE) window.THREE.ColladaLoader = mod.ColladaLoader;
                        }
                    } catch (_) { /* noop */ }
                    // Globally disable auto Z_UP -> Y_UP conversion; we'll rotate manually.
                    try {
                        if (this._ColladaLoaderClass.prototype && this._ColladaLoaderClass.prototype.options) {
                            this._ColladaLoaderClass.prototype.options.convertUpAxis = false;
                        }
                    } catch (_) { /* noop */ }
                    // Patch parse() to normalize <up_axis>Z_UP</up_axis> to Y_UP and avoid warnings
                    try {
                        if (!this._ColladaLoaderClass.__diamantsParsePatched && this._ColladaLoaderClass.prototype?.parse) {
                            const __origParse = this._ColladaLoaderClass.prototype.parse;
                            this._ColladaLoaderClass.prototype.parse = function(text, path) {
                                try {
                                    if (typeof text === 'string') {
                                        text = text.replace(/<up_axis>\s*Z_UP\s*<\/up_axis>/i, '<up_axis>Y_UP</up_axis>');
                                    } else if (text && typeof text === 'object' && text.nodeType === 9) { // Document
                                        const upNodes = text.getElementsByTagName('up_axis');
                                        if (upNodes && upNodes.length) {
                                            for (let i = 0; i < upNodes.length; i++) {
                                                const n = upNodes[i];
                                                try { if (n && n.textContent && /Z_UP/i.test(n.textContent)) n.textContent = 'Y_UP'; } catch (_) {}
                                            }
                                        }
                                    }
                                } catch (_) { /* noop */ }
                                return __origParse.call(this, text, path);
                            };
                            this._ColladaLoaderClass.__diamantsParsePatched = true;
                        }
                    } catch (_) { /* noop */ }
                    logger.info('Drone', '✅ ColladaLoader ESM chargé via import (addons)');
                    return this._ColladaLoaderClass;
                }
            } catch (e) {
                logger.warning('Drone', '⚠️ Échec import ESM ColladaLoader, tentative fallback global:', e);
            }
        } else {
            return this._ColladaLoaderClass;
        }

        // 2) Fallback global (ancienne méthode basée sur window.THREE)
        if (typeof window !== 'undefined' && window.THREE && window.THREE.ColladaLoader) {
            logger.debug('Drone', '✅ ColladaLoader déjà disponible dans window.THREE');
            try {
                if (window.THREE.ColladaLoader.prototype && window.THREE.ColladaLoader.prototype.options) {
                    window.THREE.ColladaLoader.prototype.options.convertUpAxis = false;
                }
                if (!window.THREE.ColladaLoader.__diamantsParsePatched && window.THREE.ColladaLoader.prototype?.parse) {
                    const __origParse = window.THREE.ColladaLoader.prototype.parse;
                    window.THREE.ColladaLoader.prototype.parse = function(text, path) {
                        try {
                            if (typeof text === 'string') {
                                text = text.replace(/<up_axis>\s*Z_UP\s*<\/up_axis>/i, '<up_axis>Y_UP</up_axis>');
                            } else if (text && typeof text === 'object' && text.nodeType === 9) {
                                const upNodes = text.getElementsByTagName('up_axis');
                                if (upNodes && upNodes.length) {
                                    for (let i = 0; i < upNodes.length; i++) {
                                        const n = upNodes[i];
                                        try { if (n && n.textContent && /Z_UP/i.test(n.textContent)) n.textContent = 'Y_UP'; } catch (_) {}
                                    }
                                }
                            }
                        } catch (_) { /* noop */ }
                        return __origParse.call(this, text, path);
                    };
                    window.THREE.ColladaLoader.__diamantsParsePatched = true;
                }
            } catch (_) { /* noop */ }
            return window.THREE.ColladaLoader;
        }

        // 3) Fallback ESM distant (évite le mélange UMD/ESM et les conflits de version)
        try {
            const modCdn = await import('https://unpkg.com/three@0.167.0/examples/jsm/loaders/ColladaLoader.js');
            if (modCdn && modCdn.ColladaLoader) {
                logger.info('Drone', '✅ ColladaLoader chargé via CDN ESM');
                this._ColladaLoaderClass = modCdn.ColladaLoader;
                try {
                    if (typeof window !== 'undefined') {
                        window.THREE = window.THREE || THREE;
                        if (window.THREE) window.THREE.ColladaLoader = modCdn.ColladaLoader;
                    }
                } catch (_) { /* noop */ }
                try {
                    if (this._ColladaLoaderClass.prototype && this._ColladaLoaderClass.prototype.options) {
                        this._ColladaLoaderClass.prototype.options.convertUpAxis = false;
                    }
                    if (!this._ColladaLoaderClass.__diamantsParsePatched && this._ColladaLoaderClass.prototype?.parse) {
                        const __origParse = this._ColladaLoaderClass.prototype.parse;
                        this._ColladaLoaderClass.prototype.parse = function(text, path) {
                            try {
                                if (typeof text === 'string') {
                                    text = text.replace(/<up_axis>\s*Z_UP\s*<\/up_axis>/i, '<up_axis>Y_UP</up_axis>');
                                } else if (text && typeof text === 'object' && text.nodeType === 9) {
                                    const upNodes = text.getElementsByTagName('up_axis');
                                    if (upNodes && upNodes.length) {
                                        for (let i = 0; i < upNodes.length; i++) {
                                            const n = upNodes[i];
                                            try { if (n && n.textContent && /Z_UP/i.test(n.textContent)) n.textContent = 'Y_UP'; } catch (_) {}
                                        }
                                    }
                                }
                            } catch (_) { /* noop */ }
                            return __origParse.call(this, text, path);
                        };
                        this._ColladaLoaderClass.__diamantsParsePatched = true;
                    }
                } catch (_) { /* noop */ }
                return this._ColladaLoaderClass;
            }
        } catch (e) {
            logger.error('Drone', '❌ Impossible de charger ColladaLoader via CDN ESM', e);
        }

        throw new Error('ColladaLoader indisponible (ESM/local/CDN)');
    }

    // Optional extra sanitizer for environments with aggressive XML parsers
    _ultraSanitizeXml(xml) {
        try {
            // Remove any control chars except CR/LF/TAB
            xml = xml.replace(/[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]/g, '');
            // Ensure there is exactly one XML declaration at top
            xml = xml.replace(/<\?xml[\s\S]*?\?>/g, '').trim();
            xml = `<?xml version="1.0" encoding="utf-8"?>\n` + xml;
            // Ensure root COLLADA tag exists
            if (!/\<COLLADA[\s\S]*\>[\s\S]*<\/COLLADA\>/.test(xml)) {
                // Try to wrap content if missing (very unlikely)
                xml = `<?xml version="1.0" encoding="utf-8"?>\n<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">` + xml + `</COLLADA>`;
            }
        } catch (_) { /* noop */ }
        return xml;
    }

    // Sanitize Collada XML to avoid common XML errors then parse via ColladaLoader.parse
    async _fetchAndParseDAE(url, basePath) {
        try {
            const ColladaLoader = await this.ensureColladaLoader();
            if (!ColladaLoader) throw new Error('ColladaLoader indisponible');
            const res = await fetch(url, { cache: 'no-cache' });
            if (!res.ok) throw new Error(`HTTP ${res.status} ${res.statusText}`);
            let text = await res.text();
            // If server returned HTML (likely a 404 proxy page), abort early
            const sniff = text.slice(0, 200).toLowerCase();
            if (sniff.includes('<!doctype html') || sniff.startsWith('<html') || sniff.includes('<head>') || sniff.includes('<title>')) {
                throw new Error('Reçu HTML au lieu d\'un DAE (chemin invalide)');
            }
            // Remove UTF-8 BOM if present
            if (text.charCodeAt(0) === 0xFEFF) {
                text = text.slice(1);
            }
            // Remove XML comments entirely. XML does not allow "--" inside comments, some exporters include dates like 2022-02-25
            // which break strict parsers. Removing comments is safe for geometry content.
            text = text.replace(/<!--[\s\S]*?-->/g, '');
            // Normalize newlines
            text = text.replace(/\r\n?|\n/g, '\n');
            // Replace stray ampersands that aren't part of an entity to prevent xmlParseEntityRef errors
            const sanitized = text.replace(/&(?!#\d+;|#x[0-9A-Fa-f]+;|[A-Za-z]+;)/g, '&amp;');
            const loader = new ColladaLoader();
            // Disable automatic up-axis conversion; we rotate manually to Y-up
            try { if (loader.options) loader.options.convertUpAxis = false; } catch (_) {}
            const dae = loader.parse(sanitized, basePath || url.substring(0, url.lastIndexOf('/') + 1));
            logger.info('Drone', `🧼 DAE parsé via fallback sanitize/parse: ${url}`);
            return dae;
        } catch (e) {
            logger.error('Drone', `❌ Fallback parse DAE échoué pour ${url}:`, e);
            return null;
        }
    }

    // Base path unifié pour les assets, avec découverte et validation réseau (cache global)
    async _findValidMeshBasePath(filename = 'cf2_assembly.dae') {
        // Cache (par session navigateur)
        if (typeof window !== 'undefined') {
            const cached = window.DIAMANTS_MESH_BASE;
            if (cached && typeof cached === 'string') return cached.endsWith('/') ? cached : (cached + '/');
        }

        const candidates = [];
        try {
            // 0) Absolu clair si on est bien dans Mission_system
            if (typeof window !== 'undefined' && window.location && window.location.pathname.includes('/conception/Mission_system/')) {
                candidates.push('/conception/Mission_system/assets/crazyflie/meshes/');
            }
            // 1) CONFIG.meshPath si fourni par la page (ex: index.html)
            if (typeof window !== 'undefined' && window.CONFIG && window.CONFIG.meshPath) {
                candidates.push(window.CONFIG.meshPath);
            }
            // 2) Déduire depuis la page actuelle
            if (typeof window !== 'undefined' && window.location && window.location.pathname) {
                const m = window.location.pathname.match(/^(.*\/conception\/Mission_system\/).*$/);
                if (m && m[1]) {
                    candidates.push(m[1] + 'assets/crazyflie/meshes/');
                }
                const here = window.location.pathname;
                if (here.includes('/tests/') || here.includes('/sample/')) {
                    candidates.push('../assets/crazyflie/meshes/');
                    candidates.push('../../assets/crazyflie/meshes/');
                }
                candidates.push('./assets/crazyflie/meshes/');
                candidates.push('/assets/crazyflie/meshes/');
            }
            // 3) Vite BASE_URL
            if (typeof import.meta !== 'undefined' && import.meta.env && import.meta.env.BASE_URL !== undefined) {
                const base = import.meta.env.BASE_URL || '/';
                candidates.push(base + 'assets/crazyflie/meshes/');
            }
        } catch (_) { /* noop */ }

        // Dédupliquer en gardant l'ordre
        const seen = new Set();
        const unique = candidates.filter(p => { const k = (p || '').trim(); if (!k || seen.has(k)) return false; seen.add(k); return true; });

        // Tester avec GET + sniff anti-HTML (certains serveurs renvoient index.html pour tout)
        for (const base of unique) {
            try {
                const abs = new URL((base.endsWith('/') ? base : base + '/') + filename, window.location.origin).toString();
                const ctl = typeof AbortController !== 'undefined' ? new AbortController() : null;
                const t = ctl ? setTimeout(() => ctl.abort(), 2000) : null;
                const res = await fetch(abs, { method: 'GET', cache: 'no-cache', signal: ctl?.signal, headers: { 'Accept': 'model/vnd.collada+xml,application/xml,text/xml,*/*' } });
                if (t) clearTimeout(t);
                if (!res.ok) continue;
                const ct = (res.headers.get('content-type') || '').toLowerCase();
                let okType = ct.includes('xml') || ct.includes('collada') || ct.includes('text/plain') || ct === '';
                const head = (await res.clone().text()).slice(0, 256).toLowerCase();
                const looksHtml = head.includes('<!doctype html') || head.startsWith('<html') || head.includes('<head>') || head.includes('<title>');
                const looksCollada = head.includes('<collada') || head.includes('<asset') || head.includes('<library_geometries');
                if (!looksHtml && (okType || looksCollada)) {
                    if (typeof window !== 'undefined') window.DIAMANTS_MESH_BASE = base.endsWith('/') ? base : (base + '/');
                    logger.info('Drone', `📦 Base assets DAE validée: ${window.DIAMANTS_MESH_BASE}`);
                    return window.DIAMANTS_MESH_BASE;
                }
            } catch (_) { /* try next */ }
        }

        // Dernier recours absolu connu
        const fallback = '/conception/Mission_system/assets/crazyflie/meshes/';
        logger.warning('Drone', `⚠️ Base assets DAE: utilisation fallback: ${fallback}`);
        if (typeof window !== 'undefined') window.DIAMANTS_MESH_BASE = fallback;
        return fallback;
    }

    // Compat: conserver l'ancienne API synchrone, mais déléguer à la découverte
    _resolveMeshBasePath() {
        // Retour rapide si déjà en cache
        try {
            if (typeof window !== 'undefined' && window.DIAMANTS_MESH_BASE) {
                const b = window.DIAMANTS_MESH_BASE;
                return b.endsWith('/') ? b : (b + '/');
            }
        } catch (_) { /* noop */ }
        // Valeur provisoire (sera remplacée par _findValidMeshBasePath asynchrone)
        return './assets/crazyflie/meshes/';
    }

    async tryLoadRealMesh() {
        try {
            if (!THREE || !window.THREE) {
                logger.warning('Drone', `⚠️ Drone ${this.id}: THREE not available, skipping DAE load`);
                throw new Error('THREE indisponible');
            }
            logger.info('Drone', `🔄 Drone ${this.id}: DÉBUT CHARGEMENT DAE...`);

            const ColladaLoader = await this.ensureColladaLoader();
            if (!ColladaLoader) {
                logger.warning('Drone', `❌ Drone ${this.id}: ColladaLoader indisponible - utilisation fallback visible`);
                throw new Error('ColladaLoader indisponible');
            }
            logger.info('Drone', `✅ Drone ${this.id}: ColladaLoader disponible`);

            const loader = new ColladaLoader();
            // Keep control of orientation to avoid Z_UP warning
            try { if (loader.options) loader.options.convertUpAxis = false; } catch (_) {}

            // Nouvelle localisation des assets: sous DIAMANTS_FRONTEND_ARCHITECTURE/assets/
            // Comme index.html est dans le même dossier, on peut utiliser un chemin relatif simple.
            // Découverte fiable du chemin des meshes (avec validation réseau)
            const basePath = await this._findValidMeshBasePath('cf2_assembly.dae');
            let url = (basePath.endsWith('/') ? basePath : basePath + '/') + 'cf2_assembly.dae';
            try {
                url = new URL(url, window.location.origin).toString();
            } catch (_) { /* keep as-is */ }

            logger.info('Drone', `📁 Drone ${this.id}: Chemin DAE tenté: ${url}`);
            logger.debug('Drone', `📁 Drone ${this.id}: Base path résolu: ${basePath}`);
            if (window.location) {
                logger.debug('Drone', `🌐 Location href: ${window.location.href}`);
                logger.debug('Drone', `🌐 Location origin: ${window.location.origin}`);
                logger.debug('Drone', `🌐 Location pathname: ${window.location.pathname}`);
            }

            await new Promise((resolve, reject) => {
                logger.info('Drone', `📥 Drone ${this.id}: Démarrage chargement DAE...`);

                // Timeout pour éviter les blocages
                const timeout = setTimeout(() => {
                    logger.error('Drone', `⏱️ Drone ${this.id}: TIMEOUT chargement DAE après 10 secondes`);
                    reject(new Error('Timeout chargement DAE'));
                }, 10000);

                loader.load(url, async (dae) => {
                    clearTimeout(timeout);
                    try {
                        logger.info('Drone', `✅ Drone ${this.id}: DAE CHARGÉ:`, dae);
                        // Certaines versions retournent null/objet incomplet en cas d'erreur silencieuse → tenter fallback parse
                        if (!dae || (typeof dae === 'object' && !dae.scene && !dae.library)) {
                            logger.warning('Drone', `⚠️ Drone ${this.id}: DAE onLoad a renvoyé null — tentative fallback parse()`);
                            dae = await this._fetchAndParseDAE(url, basePath);
                            if (!dae) throw new Error('DAE null après fallback parse');
                        }

                        const model = (dae && dae.scene) ? dae.scene : dae;
                        if (!model) throw new Error('Modèle DAE invalide (null/undefined)');
                        // Normalisation orientation Collada (Z-up) -> Three.js (Y-up)
                        model.rotation.x = -Math.PI / 2;
                        model.scale.set(1, 1, 1);
                        model.position.set(0, 0, 0);

                        logger.debug('Drone', `🔧 Drone ${this.id}: Modèle DAE configuré:`, model);

                        // Log rapide des premiers meshes pour diagnostic
                        try {
                            let firstMesh = null;
                            model.traverse(n => { if (!firstMesh && n.isMesh) firstMesh = n; });
                            if (firstMesh) logger.info('Drone', `🧩 Premier mesh DAE: ${firstMesh.name || '(sans nom)'} | geom=${firstMesh.geometry?.type}`);
                        } catch (_) {}

                        // Recentre le modèle: horizontalement au centre, verticalement le bas à y=0
                        // Ceci garantit que position.y du groupe = altitude du bas du drone
                        try {
                            const bbox = new THREE.Box3().setFromObject(model);
                            const center = new THREE.Vector3();
                            bbox.getCenter(center);
                            model.position.x -= center.x;
                            model.position.z -= center.z;
                            // Décaler verticalement pour que le bas du modèle soit à y=0 en espace local
                            model.position.y -= bbox.min.y;
                            // Stocker l'offset pour aligner les hélices au même niveau
                            this._bodyYOffset = -bbox.min.y;
                            logger.info('Drone', `📐 Drone ${this.id}: bbox min.y=${bbox.min.y.toFixed(4)}, max.y=${bbox.max.y.toFixed(4)}, bodyYOffset=${this._bodyYOffset.toFixed(4)}`);
                        } catch (e) {
                            this._bodyYOffset = 0;
                            logger.warning('Drone', `⚠️ Drone ${this.id}: Calcul centre échoué:`, e);
                        }

                        // Prepare/refresh model group (remove older instance if present)
                        if (this.modelGroup && this.modelGroup.parent) {
                            this.modelGroup.parent.remove(this.modelGroup);
                        }
                        this.modelGroup = new THREE.Group();

                        // Supprimer les fallback debug
                        this.mesh.children.slice().forEach(ch => {
                            if (ch?.userData?.debug || ch?.userData?.fallbackType === 'body') {
                                log('🗑️ Suppression fallback debug:', ch);
                                this.mesh.remove(ch);
                            }
                        });

                        // Insert DAE body
                        this.modelGroup.add(model);
                        this.mesh.add(this.modelGroup);

                        logger.info('Drone', `✅ Drone ${this.id}: Corps DAE ajouté au mesh`);

                        this.modelGroup.traverse((n) => {
                            if (n.isMesh) {
                                n.castShadow = true;
                                n.receiveShadow = true;
                                // Ensure body materials render reliably
                                const mats = Array.isArray(n.material) ? n.material : [n.material];
                                mats.forEach(m => {
                                    if (!m) return;
                                    m.side = THREE.DoubleSide;
                                    if (typeof m.transparent !== 'undefined') m.transparent = false;
                                    if (typeof m.opacity !== 'undefined') m.opacity = 1.0;
                                    if (typeof m.depthWrite !== 'undefined') m.depthWrite = true;
                                });
                            }
                        });

                        logger.debug('Drone', `🎨 Drone ${this.id}: Matériaux DAE configurés`);

                        // Charger les hélices CW/CCW
                        logger.info('Drone', `🚁 Drone ${this.id}: Début chargement hélices DAE...`);
                        this.loadPropellers(basePath).then(() => {
                            logger.info('Drone', `✅ Drone ${this.id}: SUCCÈS COMPLET DAE + HÉLICES`);
                            resolve();
                        }).catch(propError => {
                            logger.error('Drone', `❌ Drone ${this.id}: ÉCHEC hélices DAE:`, propError);
                            // Ne pas invalider le corps DAE si les hélices échouent
                            if (this.allowFallbackProps) {
                                try {
                                    // this.ensureFallbackPropellers(); // SUPPRIMÉ - DAE uniquement
                                    logger.warning('Drone', '🟡 Fallback hélices rétabli (BoxGeometry) après échec DAE');
                                } catch (_) { /* safe */ }
                            }
                            // Considérer la charge du corps comme succès pour supprimer les cubes fallback
                            resolve();
                        });

                    } catch (error) {
                        clearTimeout(timeout);
                        logger.error('Drone', `❌ Drone ${this.id}: ERREUR traitement DAE:`, error);
                        reject(error);
                    }
                }, undefined, async (error) => {
                    clearTimeout(timeout);
                    logger.error('Drone', `❌ Drone ${this.id}: ERREUR chargement DAE:`, error);
                    logger.error('Drone', `📁 Drone ${this.id}: URL tentée: ${url}`);
                    // Tentative de récupération via fetch + parse (sanitized)
                    try {
                        let dae = await this._fetchAndParseDAE(url, basePath);
                        if (!dae) {
                            // One more attempt with ultra sanitizer
                            const res = await fetch(url, { cache: 'no-cache' });
                            let xml = await res.text();
                            xml = this._ultraSanitizeXml(xml);
                            const ColladaLoader2 = await this.ensureColladaLoader();
                            const loader2 = new ColladaLoader2();
                            // Ensure fallback parser also doesn't auto-convert
                            try { if (loader2.options) loader2.options.convertUpAxis = false; } catch (_) {}
                            dae = loader2.parse(xml, basePath || url.substring(0, url.lastIndexOf('/') + 1));
                        }
                        if (!dae) return reject(error);
                        // Simuler le chemin onLoad avec l'objet parsé
                        logger.info('Drone', `↩️ Récupération DAE via parse() — poursuite du pipeline`);
                        const model = (dae && dae.scene) ? dae.scene : dae;
                        if (!model) throw new Error('Modèle DAE invalide après fallback parse');
                        model.rotation.x = -Math.PI / 2;
                        model.scale.set(1, 1, 1);
                        model.position.set(0, 0, 0);

                        // Décaler verticalement pour que le bas du modèle soit à y=0
                        try {
                            const bbox2 = new THREE.Box3().setFromObject(model);
                            const center2 = new THREE.Vector3();
                            bbox2.getCenter(center2);
                            model.position.x -= center2.x;
                            model.position.z -= center2.z;
                            model.position.y -= bbox2.min.y;
                            this._bodyYOffset = -bbox2.min.y;
                        } catch (_) {
                            this._bodyYOffset = 0;
                        }

                        // Prepare/refresh model group (remove older instance if present)
                        if (this.modelGroup && this.modelGroup.parent) {
                            this.modelGroup.parent.remove(this.modelGroup);
                        }
                        this.modelGroup = new THREE.Group();
                        // Supprimer les fallback debug
                        this.mesh.children.slice().forEach(ch => {
                            if (ch?.userData?.debug || ch?.userData?.fallbackType === 'body') {
                                this.mesh.remove(ch);
                            }
                        });
                        this.modelGroup.add(model);
                        this.mesh.add(this.modelGroup);
                        this.modelGroup.traverse((n) => {
                            if (n.isMesh) {
                                n.castShadow = true;
                                n.receiveShadow = true;
                                const mats = Array.isArray(n.material) ? n.material : [n.material];
                                mats.forEach(m => {
                                    if (!m) return;
                                    m.side = THREE.DoubleSide;
                                    if (typeof m.transparent !== 'undefined') m.transparent = false;
                                    if (typeof m.opacity !== 'undefined') m.opacity = 1.0;
                                    if (typeof m.depthWrite !== 'undefined') m.depthWrite = true;
                                });
                            }
                        });
                        // Charger les hélices puis resolve
                        this.loadPropellers(basePath).then(resolve).catch(reject);
                    } catch (e2) {
                        reject(error);
                    }
                });
            });
            // Success reaching here
            return true;
        } catch (e) {
            logger.error('Drone', `❌ Drone ${this.id}: EXCEPTION tryLoadRealMesh:`, e);
            throw e; // propagate to caller
        }
    }

    async loadPropellers(basePath) {
        // ⚠️ GARDE ANTI-BOUCLE INFINIE
        if (this.propellersLoading) {
            logger.warn('Drone', `⚠️ Drone ${this.id}: loadPropellers déjà en cours - IGNORÉ`);
            return;
        }
        if (this.propellersLoaded) {
            logger.debug('Drone', `✅ Drone ${this.id}: Hélices déjà chargées - IGNORÉ`);
            return;
        }

        this.propellersLoading = true;
        logger.debug('Drone', `🚁 Drone ${this.id}: loadPropellers() - Début chargement hélices`);

        try {
            const ColladaLoader = await this.ensureColladaLoader();
            if (!ColladaLoader) {
                logger.error('Drone', `❌ Drone ${this.id}: ColladaLoader indisponible - IMPOSSIBLE de charger les DAE`);
                this.propellersLoading = false;
                return;
            }

            // Ensure we have a validated base path (works even if caller didn't pass one)
            try {
                if (!basePath || typeof basePath !== 'string') {
                    basePath = await this._findValidMeshBasePath('ccw_prop.dae');
                }
            } catch (_) { /* keep provided basePath if discovery fails */ }
            if (basePath && !basePath.endsWith('/')) basePath = basePath + '/';

        const loader = new ColladaLoader();
        // Disable auto axis conversion for propeller DAE loads
        try { if (loader.options) loader.options.convertUpAxis = false; } catch (_) {}
        
        // Nettoyage des hélices existantes AVANT rechargement
        this._cleanupExistingPropellers();
        this.propellerGroups = [];
        // Important: vider les hélices fallback pour éviter d'animer des groupes supprimés
        this.propellers = [];

        log('🔄 Chargement EXCLUSIF des hélices DAE depuis:', basePath);

        // Mapping exact depuis this.specs.motors - POSITIONS AUTHENTIQUES
        const motorSpecs = this.specs.motors.map((motor, index) => ({
            pos: motor.pos, // Position exacte depuis les specs
            file: motor.dir === 'ccw' ? 'ccw_prop.dae' : 'cw_prop.dae',
            dir: motor.dir
        }));

        for (let i = 0; i < this.specs.motors.length; i++) {
            const motor = this.specs.motors[i];
            const motorSpec = motorSpecs[i];
            const propFileName = motorSpec.file;
            // Build robust absolute URL relative to current origin
            let propUrl = basePath + propFileName;
            try {
                // If basePath is relative, new URL with origin will resolve it
                const abs = new URL(propUrl, window.location.origin).toString();
                propUrl = abs;
            } catch (_) { /* keep as-is */ }
            const expectedDir = motorSpec.dir;

            log(`📥 Moteur ${i + 1} - CHARGEMENT DAE OBLIGATOIRE: ${propFileName}`);

            await new Promise((resolve, reject) => {
                const onSuccess = (dae) => {
                    try {
                        log(`✅ DAE RÉUSSI pour moteur ${i + 1}:`, propFileName);

                        // Extraire la scène DAE - STRUCTURE AUTHENTIQUE
                        let daeScene = (dae && dae.scene) ? dae.scene : dae;
                        if (!daeScene) {
                            throw new Error('DAE prop null/invalid');
                        }

                        // Localiser le noeud principal avec la géométrie réelle
                        let propNode = null;
                        daeScene.traverse(node => {
                            // Chercher spécifiquement les noeuds avec géométrie mesh
                            if (node.isMesh && node.geometry) {
                                propNode = node;
                                log(`🎯 GÉOMÉTRIE DAE trouvée:`, node.name, node.geometry.type);
                                return;
                            }
                            // Ou chercher les noeuds nommés correctement
                            if (node.name && (node.name.includes('cw_prop') || node.name.includes('ccw_prop'))) {
                                propNode = node;
                                log(`🎯 NOEUD DAE nommé:`, node.name);
                            }
                        });

                        if (!propNode) {
                            // Essayer le premier enfant avec géométrie
                            for (let child of daeScene.children) {
                                if (child.isMesh || (child.children && child.children.some(c => c.isMesh))) {
                                    propNode = child;
                                    log(`🔄 Utilisation noeud géométrie:`, child.name || 'unnamed');
                                    break;
                                }
                            }
                        }

                        if (!propNode) {
                            throw new Error(`ÉCHEC CRITIQUE: Aucune géométrie DAE trouvée dans ${propFileName}`);
                        }

                        // Cloner la géométrie DAE AUTHENTIQUE
                        const realPropGeometry = propNode.clone(true);

                        // Conversion Z-up (DAE) -> Y-up (Three.js) avec retournement hélices
                        // (flipping to correct leading/trailing edge orientation)
                        realPropGeometry.rotation.x = Math.PI / 2 + Math.PI; // +180° pour retourner (moyeu en bas, pales en haut)

                        // CORRECTION: l'orientation des pales sera ajustée au niveau du groupe (rotation Y)

                        realPropGeometry.scale.set(1, 1, 1);

                        // Direction basée sur le fichier SDF - AUTHENTIQUE
                        const isCW = expectedDir === 'cw';
                        log(`🌀 Moteur ${i + 1}: ${isCW ? 'CW' : 'CCW'} (SDF: ${expectedDir})`);

                        // Correction d'orientation selon CW/CCW
                        // Joint axis est 0 0 1 (Z-up), joints revolute Z
                        // Après conversion Z-up DAE -> Y-up Three.js (rotation.x = PI/2 + PI)
                        // Orientation basée sur le type de fichier DAE et direction moteur
                        realPropGeometry.rotation.y = isCW ? Math.PI : 0;

                        // Matériaux DAE - conservation de l'authenticité + coloration moyeu
                        realPropGeometry.traverse(child => {
                            if (child.isMesh && child.material) {
                                const materials = Array.isArray(child.material) ? child.material : [child.material];
                                materials.forEach(mat => {
                                    if (mat) {
                                        // Conserver les propriétés DAE originales mais assurer la visibilité
                                        mat.side = THREE.DoubleSide;
                                        mat.transparent = false;
                                        mat.opacity = 1.0;
                                        mat.depthWrite = true;

                                        // DÉTECTER LE MOYEU : partie centrale/hub de l'hélice DAE
                                        const isHub = child.name && (
                                            child.name.toLowerCase().includes('hub') ||
                                            child.name.toLowerCase().includes('center') ||
                                            child.name.toLowerCase().includes('moyeu') ||
                                            child.name.toLowerCase().includes('shaft') ||
                                            child.name.toLowerCase().includes('motor') ||
                                            child.name.toLowerCase().includes('rotor') ||
                                            child.name.toLowerCase().includes('mount')
                                        );

                                        // Calculer bounding box pour détecter les petites parties centrales
                                        let isSmallCentralPart = false;
                                        if (child.geometry) {
                                            child.geometry.computeBoundingBox();
                                            const bbox = child.geometry.boundingBox;
                                            if (bbox) {
                                                const size = new THREE.Vector3();
                                                bbox.getSize(size);
                                                const volume = size.x * size.y * size.z;
                                                // Moyeu = partie plus petite ET plus épaisse (cylindrique)
                                                const isThick = size.y > size.x * 0.3; // Plus épais que large
                                                const isSmall = volume < 0.05;
                                                isSmallCentralPart = isSmall && isThick;
                                                log(`📏 Analyse ${child.name || 'unnamed'}:`,
                                                    `vol: ${volume.toFixed(4)}, `,
                                                    `dim: ${size.x.toFixed(3)}x${size.y.toFixed(3)}x${size.z.toFixed(3)}`,
                                                    `thick: ${isThick}, small: ${isSmall}`);
                                            }
                                        }

                                        if (isHub || isSmallCentralPart) {
                                            // MOYEU DÉTECTÉ - ROUGE
                                            mat.color.setHex(0xFF0000);
                                            log(`🔴 MOYEU TROUVÉ ET COLORÉ:`, child.name || 'unnamed');
                                        } else {
                                            // PALES - RENDRE TRANSPARENTES 
                                            mat.color.setHex(0x000000);
                                            mat.transparent = true;
                                            mat.opacity = 0.0;
                                            log(`👻 PALE RENDUE TRANSPARENTE:`, child.name || 'unnamed', {
                                                couleurOriginale: child.material?.color?.getHex?.()?.toString(16) || 'unknown'
                                            });
                                        }
                                        mat.needsUpdate = true;
                                    }
                                });
                            }
                        });

                        // Centrage géométrique pour rotation authentique
                        const bbox = new THREE.Box3().setFromObject(realPropGeometry);
                        const center = new THREE.Vector3();
                        bbox.getCenter(center);
                        const size = new THREE.Vector3();
                        bbox.getSize(size);

                        // Recentrer pour rotation au moyeu
                        realPropGeometry.position.sub(center);

                        // Retourner l'hélice de 180° (les pales sont inversées par défaut)
                        realPropGeometry.rotation.x += Math.PI;

                        // Groupe de rotation SDF-authentique
                        const propGroup = new THREE.Group();
                        propGroup.name = `prop_${i + 1}_${isCW ? 'CW' : 'CCW'}_DAE_REAL`;
                        propGroup.add(realPropGeometry);

                        // Position du groupe selon mapping SDF original (moteurs sur les bras)
                        const sdfMapping = [
                            { pos: { x: 0.031, y: -0.031, z: 0.021 } }, // M1
                            { pos: { x: -0.031, y: -0.031, z: 0.021 } }, // M2  
                            { pos: { x: -0.031, y: 0.031, z: 0.021 } }, // M3
                            { pos: { x: 0.031, y: 0.031, z: 0.021 } }   // M4
                        ];
                        const sdfPos = sdfMapping[i].pos;
                        const worldX = sdfPos.x;
                        const worldZ = -sdfPos.y;
                        // Appliquer le même offset Y que le corps DAE pour aligner les hélices
                        const bodyYOff = this._bodyYOffset || 0;
                        propGroup.position.set(
                            worldX,
                            sdfPos.z - 0.008 + bodyYOff,  // Y: hauteur corps + offset bbox
                            worldZ
                        );

                        // Ajuster la position de l'hélice DANS le groupe pour parfait alignement
                        const motorPos = motor.pos;
                        const offsetX = motorPos.x - sdfPos.x;
                        const offsetZ = -motorPos.y - (-sdfPos.y);
                        realPropGeometry.position.set(offsetX, 0, offsetZ);

                        // CORRECTION: Aligner la rotation de l'hélice avec l'angle du bras moteur + 180°
                        const angle = Math.atan2(worldZ, worldX);
                        propGroup.rotation.y = angle + Math.PI;

                        // Ajouter une jointure/support moteur SOUS l'hélice
                        const motorJoint = this.createMotorJoint();
                        // Retourner le support moteur pour qu'il pointe vers le bas (entre corps et hélice)
                        motorJoint.rotation.x = Math.PI;

                        propGroup.add(motorJoint);

                        // Attachement au corps DAE
                        const parent = this.modelGroup || this.mesh;
                        parent.add(propGroup);

                        // Métadonnées DAE authentiques
                        propGroup.userData = {
                            propVars: {
                                type: isCW ? 'cw' : 'ccw',
                                source: propFileName,
                                authentic: true,
                                daeLoaded: true,
                                sdfFile: 'model.sdf',
                                nodeName: propNode.name || 'dae_node',
                                radius: Math.max(size.x, size.z) * 0.5,
                                thickness: size.y,
                                center: center.toArray(),
                                sdfPosition: [sdfPos.x, sdfPos.y, sdfPos.z],
                                motorPosition: [motorPos.x, motorPos.y, motorPos.z],
                                motorIndex: i,
                                motorId: motor.id
                            }
                        };

                        // Axe de rotation réaliste: horizontal (Z) pour rotation dans le plan XY
                        let spinAxis = 'z';

                        this.propellerGroups[i] = { group: propGroup, blade: realPropGeometry, isCW, spinAxis };

                        log(`✅ HÉLICE DAE AUTHENTIQUE ${i + 1} installée:`, {
                            type: isCW ? 'CW' : 'CCW',
                            position: propGroup.position.toArray(),
                            file: propFileName,
                            size: size.toArray(),
                            spinAxis
                        });

                        resolve();

                    } catch (err) {
                        console.error(`❌ ÉCHEC TRAITEMENT DAE ${propFileName}:`, err);
                        reject(err);
                    }
                };
                const onError = async (loadErr) => {
                    console.error(`❌ ÉCHEC CHARGEMENT DAE ${propFileName}:`, loadErr);
                    // Fallback: fetch + parse sanitized XML
                    try {
                        const dae = await this._fetchAndParseDAE(propUrl, basePath);
                        if (!dae) return reject(loadErr);
                        onSuccess(dae);
                    } catch (e2) {
                        reject(loadErr);
                    }
                };
                loader.load(propUrl, onSuccess, undefined, onError);
            });
        }

        log('✅ CHARGEMENT DAE EXCLUSIF TERMINÉ - Toutes les hélices sont authentiques');
        this.propellersLoaded = true;
        } catch (error) {
            logger.error('Drone', `❌ Drone ${this.id}: Erreur chargement hélices:`, error);
        } finally {
            this.propellersLoading = false;
        }
    }

    // Méthode utilitaire pour nettoyer les hélices existantes
    _cleanupExistingPropellers() {
        if (this.propellerGroups?.length > 0) {
            this.propellerGroups.forEach(group => {
                if (group?.group?.parent) {
                    group.group.parent.remove(group.group);
                }
            });
            log(`🗑️ Nettoyage ${this.propellerGroups.length} hélices existantes`);
        }

        // Supprimer UNIQUEMENT les hélices fallback/procédurales créées par nous
        // Ne pas supprimer les meshes utilisateur/DAE existants
        this.mesh.children.slice().forEach(child => {
            if (child?.userData?.fallbackType === 'prop') {
                this.mesh.remove(child);
                log('🗑️ Suppression hélice fallback (safe):', child.name);
            }
        });
    }

    // AUCUN FALLBACK - CHARGEMENT DAE EXCLUSIF
    // Toutes les géométries doivent provenir des fichiers DAE authentiques

    // Fallback utilitaire: recrée des hélices BoxGeometry visibles si DAE indisponible
    // Fonction fallback supprimée - on utilise uniquement les DAE
    ensureFallbackPropellers() {
        // Fallback désactivé - utilisation exclusive des DAE
        return;
    }

    createMotorJoint() {
        // Créer un support/jointure entre le corps et l'hélice
        const jointGroup = new THREE.Group();

        // Support moteur principal (cylindre vertical)
        const motorBodyGeometry = new THREE.CylinderGeometry(0.004, 0.004, 0.008);
        const motorBodyMaterial = new THREE.MeshPhongMaterial({
            color: 0x333333,
            shininess: 30
        });
        const motorBody = new THREE.Mesh(motorBodyGeometry, motorBodyMaterial);
        motorBody.position.y = 0.004; // Positionner au-dessus du corps
        jointGroup.add(motorBody);

        // Petite base de fixation (disque plat)
        const baseGeometry = new THREE.CylinderGeometry(0.006, 0.006, 0.001);
        const baseMaterial = new THREE.MeshPhongMaterial({
            color: 0x222222,
            shininess: 50
        });
        const base = new THREE.Mesh(baseGeometry, baseMaterial);
        base.position.y = 0.0005; // Juste au-dessus du corps
        jointGroup.add(base);

        // Axe central fin (tige de rotation)
        const axisGeometry = new THREE.CylinderGeometry(0.001, 0.001, 0.012);
        const axisMaterial = new THREE.MeshPhongMaterial({
            color: 0x666666,
            shininess: 80
        });
        const axis = new THREE.Mesh(axisGeometry, axisMaterial);
        axis.position.y = 0.008; // S'étendre jusqu'à l'hélice
        jointGroup.add(axis);

        return jointGroup;
    }

    createStatusLED() {
        const ledGeometry = new THREE.SphereGeometry(0.002);
        // Utiliser MeshStandardMaterial pour supporter 'emissive'
        const ledMaterial = new THREE.MeshStandardMaterial({
            color: 0x00FF00,
            emissive: new THREE.Color(0x004400),
            emissiveIntensity: 1.0
        });
        const led = new THREE.Mesh(ledGeometry, ledMaterial);
        // Y est l'axe vertical (ancien SDF utilisait Z)
        led.position.set(0, 0.025, 0);
        return led;
    }

    /**
     * Mise à jour principale — PURE VIEWER MODE
     * Le drone interpole (lerp) vers targetPosition reçue du backend.
     * Aucune simulation physique, aucune exploration autonome.
     */
    update(deltaTime, allDrones = [], obstacles = [], diamantFormulas = null) {
        // Lerp position → targetPosition (smooth)
        if (this.targetPosition) {
            const lerpFactor = Math.min(1.0, 8.0 * deltaTime); // ~8 Hz smooth
            this.position.lerp(this.targetPosition, lerpFactor);
        }

        // Mise à jour visuelle (mesh sync, propeller spin, LED)
        this.updateVisuals(deltaTime);
        return this.state;
    }

    setRole(role) {
        this.typeConfig.role = role;
        return this;
    }

    setFormationIndex(index) {
        this.formationIndex = index;
        return this;
    }

    isActive() {
        return this.state !== 'EMERGENCY';
    }

    handleCollision(type, other = null) { /* no-op — viewer mode */ }

    applyAvoidanceForce(vec) { /* no-op */ }

    applyDiamantForce(vec) { /* no-op */ }

    getPublicState() {
        return this.getStatus();
    }

    reset(position) {
        if (position && position.isVector3) {
            this.position.copy(position);
        } else if (position && typeof position === 'object') {
            this.position.set(position.x || 0, position.y || 0, position.z || 0);
        } else {
            this.position.set(0, 3, 0);
        }
        this.velocity.set(0, 0, 0);
        this.acceleration.set(0, 0, 0);
        this.state = 'IDLE';
        if (this.mesh) this.mesh.position.copy(this.position);
    }

    emergencyStop() {
        this.state = 'EMERGENCY';
        this.velocity.set(0, 0, 0);
        this.acceleration.set(0, 0, 0);
    }

    /* ═══════════════════════════════════════════════════════════════════
     * SIMULATION METHODS — ALL NO-OPS IN VIEWER MODE
     * The drone renders at positions received from the controller via
     * setTargetPosition(). No physics, no autonomous exploration,
     * no sensor simulation. The controller is the single
     * source of truth for all drone positions and states.
     * ═══════════════════════════════════════════════════════════════════ */

    updateDiamantsProperties() { /* no-op — viewer mode */ }
    updateSensors() { /* no-op */ }
    updateMission() { /* no-op */ }
    updateFlightMission() { /* no-op */ }
    hoverBehavior() { /* no-op */ }
    explorationBehavior() { /* no-op */ }
    formationBehavior() { /* no-op */ }
    getFormationOffset() { return new THREE.Vector3(); }
    avoidObstacles() { /* no-op */ }
    updateFlightControl() { /* no-op */ }
    updateMotorRPM() { /* no-op */ }
    updatePhysics() { /* no-op */ }
    updatePositionController() { /* no-op */ }
    updateAttitudeController() { /* no-op */ }
    updateMotorMixer() { /* no-op */ }
    updateMotorDynamics() { /* no-op */ }
    updateRigidBodyDynamics() { /* no-op */ }
    handleCollisionDetection() { /* no-op */ }
    handleEmergencyAltitudeCorrection() { /* no-op */ }
    applyRepulsionForce() { /* no-op */ }
    updateRepulsionForces() { /* no-op */ }
    clearRepulsionForces() { /* no-op */ }
    checkAdvancedCollisions(pos) { return { hasCollision: false, correctedPosition: pos }; }
    handleAdvancedCollisionDetection() { /* no-op */ }
    searchRescueBehavior() { /* no-op */ }

    // computePID and normalizeAngle kept as utilities
    computePID(pid, error, deltaTime) {
        if (!pid) return 0;
        pid.integral = (pid.integral || 0) + error * deltaTime;
        pid.integral = Math.max(-1, Math.min(1, pid.integral));
        const derivative = (error - (pid.lastError || 0)) / deltaTime;
        pid.lastError = error;
        return (pid.kp || 0) * error + (pid.ki || 0) * pid.integral + (pid.kd || 0) * derivative;
    }
    resetPIDControllers() { /* no-op */ }
    normalizeAngle(angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }


    updateVisuals(deltaTime) {
        if (!this.mesh) return;

        // Position et rotation
        this.mesh.position.copy(this.position);

        // Orientation issue de la dynamique (quaternion)
        try {
            this.mesh.quaternion.copy(this.orientation);
        } catch (_) { /* safe */ }

    // Rotation des hélices (DAE via propellerGroups: rotation sur l'axe horizontal Z; sinon fallback via this.propellers)
        if (this.propellerGroups && this.propellerGroups.length) {
            this.propellerGroups.forEach((pg, index) => {
                if (!pg || !pg.group) return;
                const blade = pg.blade || pg.group;
                const motor = this.motors?.[index];
                let rpm = motor && isFinite(motor.rpm) ? motor.rpm : undefined;
                if (rpm === undefined || rpm <= 0) {
                    // Baseline visuelle même à l'arrêt apparent
                    const base = Math.max(0.15, this.propellerAnimation.speeds?.[index] || 0.15);
                    rpm = base * 60 / (2 * Math.PI); // approx rpm from rad/s
                }
                const angularSpeed = (rpm / 60) * 2 * Math.PI; // rad/s
                const deltaAngle = angularSpeed * deltaTime;
                // Direction: CW = -1, CCW = +1
                const dir = pg.isCW ? -1 : 1;
                if (blade && blade.rotation) {
                    // Tourner sur l'axe horizontal Z par défaut (rotation dans le plan XY)
                    const ax = pg.spinAxis || 'z';
                    if (ax === 'x') blade.rotation.x += dir * deltaAngle;
                    else if (ax === 'y') blade.rotation.y += dir * deltaAngle; 
                    else blade.rotation.z += dir * deltaAngle; // HORIZONTAL
                } else if (pg.group && pg.group.rotation) {
                    // dernier recours
                    pg.group.rotation.z += dir * deltaAngle; // HORIZONTAL
                }

                // Effet de flou léger selon la vitesse
                const blur = Math.min(1, (rpm || 0) / 4000);
                const targetOpacity = 0.35 + 0.4 * (1 - blur);
                try {
                    (blade || pg.group).traverse?.((child) => {
                        if (child.isMesh && child.material) {
                            child.material.transparent = true;
                            if (typeof child.material.opacity === 'number') {
                                child.material.opacity = targetOpacity;
                            }
                        }
                    });
                } catch (_) { /* safe */ }
            });
            // Log léger pour confirmer la branche et une valeur d'angle
            if (!window.SILENT_MODE && Math.random() < 0.003) {
                const sample = this.propellerGroups[0];
                try {
                    const b = sample?.blade;
                    const ax = sample?.spinAxis || 'z';
                    const angle = b?.rotation?.[ax] ?? 0;
                    if (!window.SILENT_MODE) log(`🧭 DAE spin (${ax}) angle≈${(angle || 0).toFixed(2)} rad`);
                } catch (_) { /* ignore */ }
            }
        } else if (this.propellers && this.propellers.length) {
            this.propellers.forEach((propGroup, index) => {
                const motor = this.motors[index];
                const rotationSpeed = (motor.rpm / 60) * 2 * Math.PI * deltaTime;
                // Direction alignée: CW = -1, CCW = +1
                const direction = motor.direction === 'cw' ? -1 : 1;
                // Rotation HORIZONTALE locale (axe Z) pour cohérence avec DAE
                propGroup.rotation.z += rotationSpeed * direction;
                // Flou de rotation léger
                const blurLevel = Math.min(1, motor.rpm / 1000);
                propGroup.traverse(child => {
                    if (child.isMesh && child.material && child.geometry && child.geometry.type === 'BoxGeometry') {
                        child.material.opacity = 0.4 + 0.3 * (1 - blurLevel);
                        child.material.transparent = true;
                    }
                });
            });
            if (!window.SILENT_MODE && Math.random() < 0.003) {
                try {
                    const ang = this.propellers[0]?.rotation?.z ?? 0;
                    const rpm0 = this.motors?.[0]?.rpm ?? 0;
                    log(`🧭 Fallback spin (y) angle≈${(ang || 0).toFixed(2)} rad, rpm0=${rpm0.toFixed(1)}`);
                } catch (_) { /* ignore */ }
            }
        } else {
            // Aucun DAE chargé et fallback désactivé: ne rien créer
            if (this.allowFallbackProps) {
                // try { this.ensureFallbackPropellers(); } catch (_) { /* safe */ } // SUPPRIMÉ - DAE uniquement
            }
        }

        // LED de statut
        this.updateStatusLED();
    }

    updateStatusLED() {
        if (!this.statusLED) return;

        const colors = {
            'IDLE': 0x888888,
            'TAKEOFF': 0x00FF00,
            'FLYING': 0x0088FF,
            'LANDING': 0xFFAA00,
            'EMERGENCY': 0xFF0000
        };

        const base = colors[this.state] || 0x888888;
        this.statusLED.material.color.setHex(base);
        if (this.statusLED.material.emissive) {
            this.statusLED.material.emissive.setHex(base >> 2);
        }
    }

    updateSwarmCommunication(allDrones) {
        const now = Date.now();
        if (now - this.lastCommunication < 100) return; // 10Hz max

        // Partage d'informations avec drones proches
        allDrones.forEach(otherDrone => {
            if (otherDrone.id !== this.id) {
                const distance = this.position.distanceTo(otherDrone.position);
                const commRange = this.typeConfig.sensorRange * 5;

                if (distance < commRange) {
                    // Partage découvertes
                    this.shareInformation(otherDrone);
                }
            }
        });

        this.lastCommunication = now;
    }

    shareInformation(otherDrone) {
        // Échange de données d'exploration
        if (this.intelligence > otherDrone.intelligence) {
            // Partager position d'intérêt
            otherDrone.swarmMemory.set('interest_point', {
                position: this.position.clone(),
                intelligence: this.intelligence,
                timestamp: Date.now(),
                source: this.id
            });
        }
    }

    // Méthodes de contrôle mission
    shouldTakeoff() {
        // CORRIGÉ: Ne pas décoller automatiquement
        // Le décollage doit être initié par l'utilisateur ou le contrôleur
        // Un drone IDLE reste IDLE jusqu'à commande explicite
        return false; // Décollage manuel uniquement
    }

    emergencyLanding() { /* no-op — viewer mode */ }
    returnToBase() { /* no-op */ }
    searchRescueBehavior() { /* no-op */ }
    setMission(missionType, parameters = {}) {
        this.missionPhase = missionType;
        if (parameters.altitude) this.targetAltitude = parameters.altitude;
        if (parameters.position) this.targetPosition.copy(parameters.position);
    }

    getStatus() {
        return {
            id: this.id,
            type: this.type,
            state: this.state,
            position: this.position.toArray(),
            velocity: this.velocity.toArray(),
            intelligence: this.intelligence,
            emergence: this.emergence,
            batteryLevel: Math.max(0, 1 - this.metrics.flightTime / 1200), // 20min autonomie
            sensors: this.sensors,
            metrics: this.metrics
        };
    }

    /**
     * Crée et retourne le mesh 3D du drone
     */
    getMesh() {
        if (this.mesh) {
            return this.mesh;
        }

        try {
            // Créer le mesh 3D basé sur les spécifications SDF
            const group = new THREE.Group();

            // Corps principal (TRÈS gros pour être visible)
            const bodyGeometry = new THREE.BoxGeometry(
                this.specs.body.size.x * 25, // x25 pour MEGA visibilité
                this.specs.body.size.z * 25, // z devient height
                this.specs.body.size.y * 25  // y devient depth
            );

            // Couleur selon le type (plus vive)
            const config = this.getTypeConfiguration(this.type);
            const bodyMaterial = new THREE.MeshLambertMaterial({
                color: config.color,
                emissive: new THREE.Color(config.color).multiplyScalar(0.2), // Légère émission
                transparent: false,
                opacity: 1.0
            });

            const bodyMesh = new THREE.Mesh(bodyGeometry, bodyMaterial);
            bodyMesh.position.set(0, 0, 0); // Centré dans le groupe
            group.add(bodyMesh);

            // Initialiser le tableau des hélices pour updateVisuals()
            this.propellers = [];

            // Ajouter les moteurs/hélices (TRÈS visibles)
            this.specs.motors.forEach((motor, index) => {
                // Support moteur (plus gros)
                const motorGeometry = new THREE.CylinderGeometry(0.15, 0.15, 0.3, 8);
                const motorMaterial = new THREE.MeshLambertMaterial({ color: 0x333333 });
                const motorMesh = new THREE.Mesh(motorGeometry, motorMaterial);
                motorMesh.position.set(motor.pos.x * 25, 0.25, motor.pos.y * 25);
                group.add(motorMesh);

                // Hélice (créer un groupe pour l'animation) - Plus grandes
                const propGroup = new THREE.Group();
                const propGeometry = new THREE.BoxGeometry(0.8, 0.05, 0.08); // Pales TRÈS visibles
                const propMaterial = new THREE.MeshLambertMaterial({
                    color: 0x666666,
                    transparent: true,
                    opacity: 0.8
                });

                // 2 pales par hélice
                for (let i = 0; i < 2; i++) {
                    const propMesh = new THREE.Mesh(propGeometry, propMaterial);
                    propMesh.rotation.y = i * Math.PI;
                    propGroup.add(propMesh);
                }

                propGroup.position.set(motor.pos.x * 25, 0.4, motor.pos.y * 25);
                group.add(propGroup);
                this.propellers.push(propGroup);
            });

            // Ajouter un marqueur d'identification (TRÈS gros)
            const idGeometry = new THREE.SphereGeometry(0.3, 8, 8);
            const idMaterial = new THREE.MeshLambertMaterial({
                color: 0xFFFFFF,
                emissive: 0x444444
            });
            const idMesh = new THREE.Mesh(idGeometry, idMaterial);
            idMesh.position.set(0, 0.6, 0);
            group.add(idMesh);

            // LED de statut (plus grosse et brillante)
            const ledGeometry = new THREE.SphereGeometry(0.1, 6, 6);
            const ledMaterial = new THREE.MeshLambertMaterial({
                color: 0x00FF00,
                emissive: 0x006600
            });
            this.statusLED = new THREE.Mesh(ledGeometry, ledMaterial);
            this.statusLED.position.set(0, 0.5, 0.3);
            group.add(this.statusLED);

            // Positionner le groupe à la position du drone
            group.position.copy(this.position);

            // Échelle selon le type (mais garder une taille minimum visible)
            const config2 = this.getTypeConfiguration(this.type);
            const scale = Math.max(0.5, config2.scale || 1.0);
            group.scale.setScalar(scale);

            // Stocker le mesh pour les futures références
            this.mesh = group;

            log(`✅ Mesh créé pour drone ${this.id} à la position:`, this.position);
            return this.mesh;

        } catch (err) {
            console.error(`❌ Erreur création mesh drone ${this.id}:`, err);

            // Fallback : créer un cube simple rouge
            const fallbackGeometry = new THREE.BoxGeometry(1, 1, 1);
            const fallbackMaterial = new THREE.MeshLambertMaterial({ color: 0xFF0000 });
            this.mesh = new THREE.Mesh(fallbackGeometry, fallbackMaterial);
            this.mesh.position.copy(this.position);

            return this.mesh;
        }
    }

    // =============================================
    // VOL & EXPLORATION — NO-OPS (VIEWER MODE)
    // Le contrôleur gère tout. Le frontend
    // ne fait que rendre la position reçue via setTargetPosition().
    // =============================================

    takeoff() { /* no-op */ }
    startMission() { /* no-op */ }
    _generateCirclePattern() { /* no-op */ }
    _nextWaypoint() { /* no-op */ }
    startExploration() { /* no-op */ }
    generateExplorationPath() { /* no-op */ }
    generateGridPath() { /* no-op */ }
    generateBoustrophedonPath() { /* no-op */ }
    generateSpiralPath() { /* no-op */ }
    generateCoveragePath() { /* no-op */ }
    generateRandomPath() { /* no-op */ }
    updatePropellerSpeeds() { /* no-op */ }
    updateLegacyFlight() { /* no-op */ }
    updateFlightState() { /* no-op */ }
    updateExploration() { /* no-op */ }
    moveTowardsTarget() { /* no-op */ }

    forceCreateVisuals() {
        if (!this.scene) {
            if (window.diamantsSystem && window.diamantsSystem.scene) {
                this.scene = window.diamantsSystem.scene;
            } else if (window.diamantsApp && window.diamantsApp.scene) {
                this.scene = window.diamantsApp.scene;
            }
        }
        if (this.scene && window.THREE) {
            this.createCollisionBox();
        }
    }

    /**
     * Animation des pales selon la puissance
     */
    updatePropellerAnimation(deltaTime) {
        // Priorité: hélices DAE authentiques si disponibles
        if (this.propellerGroups && this.propellerGroups.length > 0) {
            this.animatePropellerGroups(deltaTime);
            return;
        }

    // Fallback: hélices procédurales simples (optionnel)
    if (this.allowFallbackProps && this.propellers && this.propellers.length > 0) {
            for (let i = 0; i < this.propellers.length && i < 4; i++) {
                const propeller = this.propellers[i];
                const speed = this.propellerAnimation.speeds[i] || 0.15; // baseline légère
        // Rotation différentielle : CW vs CCW (CW=-1, CCW=+1)
        const mdir = this.motors?.[i]?.direction;
        const direction = mdir === 'cw' ? -1 : 1;
                this.propellerAnimation.rotations[i] += speed * direction * deltaTime;
                if (propeller && propeller.rotation) {
            // Rotation verticale locale
            propeller.rotation.y = this.propellerAnimation.rotations[i];
                }
            }
        }
    }

    /**
     * Animation des groupes de pales DAE
     */
    animatePropellerGroups(deltaTime) {
        for (let i = 0; i < this.propellerGroups.length && i < 4; i++) {
            const propGroupData = this.propellerGroups[i];
            if (!propGroupData) continue;

            const propGroup = propGroupData.group;
            const blade = propGroupData.blade || propGroup; // rotate blade if available
            // Utiliser moteur.rpm si disponible, sinon animation speed
            const motor = this.motors?.[i];
            let rpm = motor && isFinite(motor.rpm) ? motor.rpm : undefined;
            let speed;
            if (rpm === undefined || rpm <= 0) {
                // rad/s
                speed = Math.max(0.15, this.propellerAnimation.speeds[i] || 0.15);
            } else {
                // convert rpm -> rad/s
                speed = (rpm / 60) * 2 * Math.PI;
            }

            // Direction selon le type de moteur (CW/CCW)
            const motorSpec = this.specs.motors[i];
            // Alignement moteur: CW = -1, CCW = +1
            const direction = motorSpec.dir === 'cw' ? -1 : 1;

            this.propellerAnimation.rotations[i] += speed * direction * deltaTime;

            // Rotation sur l'axe horizontal détecté (défaut 'z') pour DAE
            if (blade && blade.rotation) {
                const ax = propGroupData.spinAxis || 'z';
                if (ax === 'x') blade.rotation.x = this.propellerAnimation.rotations[i];
                else if (ax === 'y') blade.rotation.y = this.propellerAnimation.rotations[i];
                else blade.rotation.z = this.propellerAnimation.rotations[i];
            } else if (propGroup && propGroup.rotation) {
                // Fallback minimal si pas de référence pale
                propGroup.rotation.z = this.propellerAnimation.rotations[i];
            }

            // Effet de flou léger (opacité) en fonction de la vitesse
            const rpmEff = rpm !== undefined ? rpm : (speed * 60) / (2 * Math.PI);
            const blur = Math.min(1, rpmEff / 4000);
            const targetOpacity = 0.35 + 0.4 * (1 - blur);
            try {
                (blade || propGroup).traverse?.((child) => {
                    if (child.isMesh && child.material) {
                        child.material.transparent = true;
                        if (typeof child.material.opacity === 'number') {
                            child.material.opacity = targetOpacity;
                        }
                    }
                });
            } catch (_) { /* safe */ }

            // Debug occasionnel
            if (!window.SILENT_MODE && Math.random() < 0.001) {
                log(`🚁 ${this.id} pale ${i}: vitesse=${speed.toFixed(1)}, rotation=${this.propellerAnimation.rotations[i].toFixed(2)}`);
            }
        }
    }

    /**
     * Gestion collision avec la plateforme
     */
    handlePlatformCollision() { /* no-op — viewer mode */ }

    /**
     * Force la création des boundary boxes si elles n'existent pas
     */
    ensureBoundaryBoxes() {
        if (!this.collisionBox && window.THREE) {
            // Essayer de récupérer la scène si on ne l'a pas
            if (!this.scene) {
                if (window.diamantsSystem && window.diamantsSystem.scene) {
                    this.scene = window.diamantsSystem.scene;
                } else if (window.diamantsApp && window.diamantsApp.scene) {
                    this.scene = window.diamantsApp.scene;
                }
            }
            
            // Maintenant essayer de créer la boundary box
            if (this.scene) {
                boundaryLog(`🔧 ${this.id} : Force création boundary boxes via ensureBoundaryBoxes()`);
                this.createCollisionBox();
            } else {
                boundaryLog(`❌ ${this.id} : Impossible créer boundary boxes - pas de scène disponible`);
            }
        } else if (this.collisionBox) {
            boundaryLog(`✅ ${this.id} : Boundary boxes déjà créées`);
        }
    }

    /**
     * Initialiser/réactiver les bounding boxes de collision
     */
    createCollisionBox() {
        boundaryLog(`🔧 ${this.id} : Tentative création boundary box - scene=${!!this.scene}, THREE=${!!window.THREE}`);
        if (!this.scene || !window.THREE) {
            boundaryLog(`❌ ${this.id} : Impossible créer boundary box - scene=${!!this.scene}, THREE=${!!window.THREE}`);
            return;
        }
        
        // FORCER LE MODE DEBUG À TRUE POUR TEST
        let isDebugMode = true;
        boundaryLog(`🔍 ${this.id} : Mode debug forcé à TRUE pour test`);
        
        if (!isDebugMode) {
            boundaryLog(`🔇 ${this.id} : Mode debug collision désactivé - pas de boundary box visuelle`);
            return;
        }
        
        // Supprimer l'ancien si il existe
        if (this.collisionBox) {
            this.mesh.remove(this.collisionBox);
            boundaryLog(`🗑️ ${this.id} : Ancienne boundary box supprimée`);
        }
        
        if (this.safetyZone) {
            this.scene.remove(this.safetyZone);
            boundaryLog(`🗑️ ${this.id} : Ancienne safety zone supprimée`);
        }
        
        // Créer une nouvelle bounding box visible (wireframe vert)
        const boxSize = 0.2; // Taille réaliste pour un Crazyflie (20cm)
        const geometry = new THREE.BoxGeometry(boxSize, boxSize * 0.5, boxSize);
        const material = new THREE.MeshBasicMaterial({ 
            color: 0x00ff00, 
            wireframe: true, 
            transparent: true, 
            opacity: 0.3 
        });
        
        this.collisionBox = new THREE.Mesh(geometry, material);
        // FIXÉ: Attacher la collision box au drone mesh pour qu'elle le suive automatiquement
        this.collisionBox.position.set(0, 0, 0); // Position relative au drone
        if (this.mesh) {
            this.mesh.add(this.collisionBox);
            boundaryLog(`✅ ${this.id} : Collision box attachée au drone mesh`);
        } else {
            // Fallback si le mesh n'existe pas encore
            this.scene.add(this.collisionBox);
            this.collisionBox.position.copy(this.position);
            boundaryLog(`⚠️ ${this.id} : Collision box en mode fallback (mesh indisponible)`);
        }
        
        // Créer la zone de sécurité rouge au sol (comme dans ton image)
        const circleGeometry = new THREE.CircleGeometry(1.5, 32); // Rayon 1.5m
        const circleMaterial = new THREE.MeshBasicMaterial({ 
            color: 0xff3333, // Rouge
            transparent: true, 
            opacity: 0.4,
            side: THREE.DoubleSide
        });
        
        this.safetyZone = new THREE.Mesh(circleGeometry, circleMaterial);
        this.safetyZone.rotation.x = -Math.PI / 2; // Rotation pour être horizontal
        this.safetyZone.position.set(this.position.x, 0.01, this.position.z); // Au niveau du sol
        this.scene.add(this.safetyZone);
        
        boundaryLog(`📦 ${this.id} : Bounding box et zone de sécurité créées`);
    }

    /**
     * Mettre à jour la position de la bounding box
     */
    updateCollisionBox() {
        if (this.collisionBox) {
            // FIXÉ: Plus besoin de copier la position - elle suit automatiquement le drone !
            // this.collisionBox.position.copy(this.position); // SUPPRIMÉ
            
            // Changer la couleur selon l'état
            if (this.state === 'IDLE') {
                this.collisionBox.material.color.setHex(0x888888); // Gris
            } else if (this.state === 'TAKEOFF') {
                this.collisionBox.material.color.setHex(0xffff00); // Jaune
            } else if (this.state === 'FLYING') {
                this.collisionBox.material.color.setHex(0x00ff00); // Vert
            }
        } else {
            // Log uniquement les premières fois pour éviter le spam
            if (!this.missingBoxLogCount) this.missingBoxLogCount = 0;
            if (this.missingBoxLogCount < 3) {
                boundaryLog(`⚠️ ${this.id} : Pas de collision box à mettre à jour (${this.missingBoxLogCount + 1}/3)`);
                this.missingBoxLogCount++;
            }
        }
        
        // Mettre à jour la zone de sécurité rouge (reste au sol)
        if (this.safetyZone) {
            this.safetyZone.position.x = this.position.x;
            this.safetyZone.position.z = this.position.z;
        } else if (!this.missingSafetyLogCount) {
            boundaryLog(`⚠️ ${this.id} : Pas de safety zone à mettre à jour`);
            this.missingSafetyLogCount = 1;
        }
    }

    /**
     * Atterrissage du drone
     */
    land() {
        log(`🛬 Drone ${this.id} : Début atterrissage`);
        this.flightState.mode = 'landing';
        this.flightState.isExploring = false;
        this.flightState.power = this.propellerAnimation.powerMapping.landing;
        this.updatePropellerSpeeds();
    }

    /**
     * Arrêt complet des moteurs
     */
    stop() {
        log(`⏹️ Drone ${this.id} : Arrêt moteurs`);
        this.flightState.isFlying = false;
        this.flightState.isExploring = false;
        this.flightState.mode = 'idle';
        this.flightState.power = 0;
        this.propellerAnimation.speeds = [0, 0, 0, 0];
    }

    destroy() {
        if (this.mesh && this.scene) {
            this.scene.remove(this.mesh);
        }
    }
}

export default AuthenticCrazyflie;

// Exposition globale pour la compatibilité
window.AuthenticCrazyflie = AuthenticCrazyflie;
