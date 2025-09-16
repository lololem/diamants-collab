/**
 * DIAMANTS V3 - Crazyflie Authentique avec Modèles SDF
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
// Désactiver l'import DiamantFormulas pour éviter les erreurs de dépendance
// import { DiamantFormulas } from '../core/diamants-formulas.js';

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
        }).catch(error => {
            clearTimeout(loadingTimeout);
            error('❌ ÉCHEC DAE pour drone', this.id, ':', error);
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

                        // Recentre horizontalement le modèle pour que (0,0,0) corresponde au centre
                        try {
                            const bbox = new THREE.Box3().setFromObject(model);
                            const center = new THREE.Vector3();
                            bbox.getCenter(center);
                            model.position.x -= center.x;
                            model.position.z -= center.z;
                            logger.trace('Drone', `📐 Drone ${this.id}: Centre calculé:`, center);
                        } catch (e) {
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
                        // Dans Gazebo SDF: joint axis est 0 0 1 (Z-up), joints revolute Z
                        // Après conversion Z-up DAE -> Y-up Three.js (rotation.x = PI/2 + PI)
                        // Orientation basée sur le type de fichier DAE et direction Gazebo
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
                        propGroup.position.set(
                            worldX,
                            sdfPos.z - 0.008,  // Y: hauteur pour poser sur le corps
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

                        // Ajouter une jointure/support moteur avant l'hélice
                        const motorJoint = this.createMotorJoint();

                        // ROTATION 180° DU SUPPORT MOTEUR comme demandé
                        motorJoint.rotation.z += Math.PI;
                        log("🔄 ROTATION 180° APPLIQUÉE AU SUPPORT MOTEUR");

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

                    } catch (error) {
                        error(`❌ ÉCHEC TRAITEMENT DAE ${propFileName}:`, error);
                        reject(error);
                    }
                };
                const onError = async (error) => {
                    error(`❌ ÉCHEC CHARGEMENT DAE ${propFileName}:`, error);
                    // Fallback: fetch + parse sanitized XML
                    try {
                        const dae = await this._fetchAndParseDAE(propUrl, basePath);
                        if (!dae) return reject(error);
                        onSuccess(dae);
                    } catch (e2) {
                        reject(error);
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
     * Mise à jour principale du drone - Améliorée avec animation fluide
     */
    update(deltaTime, allDrones = [], obstacles = [], diamantFormulas = null) {
        // Compteur d'update pour debug
        if (this._updateCount === undefined) this._updateCount = 0;
        this._updateCount++;
        
        // Log réduit pour moins de pollution - Silencé en mode SILENT
        if (this._updateCount % 240 === 1 && !window.SILENT_MODE) { // Log toutes les 4 secondes (240 frames @ 60fps)
            log(`🔄 ${this.id} update: state=${this.state}, pos=[${this.position.x.toFixed(1)}, ${this.position.y.toFixed(1)}, ${this.position.z.toFixed(1)}]`);
        }
        
        // Réinitialiser l'accélération au début de chaque frame pour éviter l'accumulation
        this.acceleration.set(0, 0, 0);
        
        // Mise à jour métriques
        this.metrics.flightTime += deltaTime;

        // Calcul des propriétés DIAMANTS
        if (diamantFormulas) {
            this.updateDiamantsProperties(diamantFormulas, allDrones);
        }

        // Mise à jour capteurs
        this.updateSensors(obstacles);

        // Logique de mission
        this.updateMission(deltaTime, allDrones, obstacles);

        // Contrôle de vol
        this.updateFlightControl(deltaTime);

        // Mise à jour physique
        this.updatePhysics(deltaTime, allDrones);

        // Heartbeat anti-fige: si déplacement quasi nul pendant > 1s, pousser une micro intention
        try {
            const moved = this.position.distanceToSquared(this._motionWatch.lastPos);
            this._motionWatch.accTime += deltaTime;
            // ~4cm^2 en 3D ≈ quasi immobile visuellement à notre échelle
            const veryStill = moved < 0.04 * 0.04;
            if (this._motionWatch.accTime > 1.0) {
                if (veryStill) {
                    // DÉSACTIVÉ: Nudge automatique qui causait des décollages non désirés
                    // Seuls les commandes manuelles ou le contrôleur devraient initier le décollage
                    const now = performance && performance.now ? performance.now() : Date.now();
                    if (!this._motionWatch.lastNudge || (now - this._motionWatch.lastNudge) > 1500) {
                        // COMMENTÉ: this.state = (this.state === 'IDLE') ? 'TAKEOFF' : this.state;
                        // Les drones restent IDLE jusqu'à commande explicite
                        if (!window.SILENT_MODE) log(`🛑 Drone ${this.id} immobile - attente commande utilisateur`);
                        
                        // En état IDLE, maintenir un throttle minimal pour contrer la gravité
                        if (this.state === 'IDLE') {
                            this.commands.throttle = 0.55; // Throttle minimal pour hover au sol
                        }
                        
                        this._motionWatch.lastNudge = now;
                        // Console rare pour debug terrain
                        if (!window.SILENT_MODE && Math.random() < 0.05) {
                            if (!window.SILENT_MODE) log(`🩺 Nudge mouvement appliqué à ${this.id} (stuck watchdog)`);
                        }
                    }
                }
                // Reset fenêtre
                this._motionWatch.accTime = 0;
                this._motionWatch.lastPos.copy(this.position);
            }
        } catch (_) { /* safe */ }

        // Mise à jour visuelle
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

    handleCollision(type, other = null) {
        this.metrics.collisions++;
        // Petite impulsion d'évitement
        if (other && other.getPosition) {
            const away = this.getPosition().sub(other.getPosition()).normalize().multiplyScalar(0.5);
            this.acceleration.add(away);
        }
        if (type === 'environment') {
            // Remonter légèrement
            this.acceleration.z += 1.0;
        }
    }

    applyAvoidanceForce(vec) {
        if (vec && vec.isVector3) this.acceleration.add(vec);
    }

    applyDiamantForce(vec) {
        if (vec && vec.isVector3) this.acceleration.add(vec);
    }

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

    updateDiamantsProperties(diamantFormulas, allDrones) {
        // Obtenir le gradient local
        const gradient = diamantFormulas.getLocalGradient(this.position);
        this.gradient.copy(gradient);

        // Intelligence locale I(t) = |∇(φ+σ)|
        this.intelligence = this.gradient.length();

        // Potentiels locaux
        const metrics = diamantFormulas.getMetrics();
        this.phi = metrics.phi_mean;
        this.sigma = metrics.sigma_mean;

        // Émergence
        this.emergence = metrics.emergence;

        // Application des forces DIAMANTS
        const force = this.gradient.clone().multiplyScalar(0.1 * this.typeConfig.agility);
        this.acceleration.add(force);
    }

    updateSensors(obstacles) {
        // Simulation LiDAR multiranger (4 directions)
        // Détection simple sur le plan X/Z (Y = altitude)
        // Garde défensive si jamais specs.sensor était modifié
        if (!this.specs.sensor) {
            this.specs.sensor = { maxRange: 3.0 };
        }
        const directions = [
            new THREE.Vector3(1, 0, 0),   // Avant
            new THREE.Vector3(-1, 0, 0),  // Arrière
            new THREE.Vector3(0, 0, 1),   // Droite
            new THREE.Vector3(0, 0, -1)   // Gauche
        ];

        const previousRanges = [...this.sensors.lidar.ranges];

        directions.forEach((dir, index) => {
            let minDistance = this.specs.sensor.maxRange;

            obstacles.forEach(obstacle => {
                const distance = this.position.distanceTo(obstacle.position);
                if (distance < minDistance) {
                    minDistance = distance;
                }
            });

            this.sensors.lidar.ranges[index] = minDistance;
        });

        // Mise à jour odométrie
        this.sensors.odometry = {
            x: this.position.x,
            y: this.position.y,
            z: this.position.z,
            vx: this.velocity.x,
            vy: this.velocity.y,
            vz: this.velocity.z
        };

        // 🔍 LOGS DÉTAILLÉS POUR DEBUGGING - Multi-Ranger et SLAM
        if (!this._sensorLogCount) this._sensorLogCount = 0;
        this._sensorLogCount++;
        
        if (this._sensorLogCount % 60 === 0) { // Log toutes les 60 frames (~1s)
            const dirNames = ['Avant', 'Arrière', 'Droite', 'Gauche'];
            const rangeStr = this.sensors.lidar.ranges.map((r, i) => `${dirNames[i]}:${r.toFixed(2)}m`).join(' | ');
            
            if (!window.SILENT_MODE) log(`📡 ${this.id} | MULTI-RANGER: ${rangeStr}`);
            if (!window.SILENT_MODE) log(`🧭 ${this.id} | ODOMETRIE: Pos[${this.sensors.odometry.x.toFixed(2)}, ${this.sensors.odometry.y.toFixed(2)}, ${this.sensors.odometry.z.toFixed(2)}] Vel[${this.sensors.odometry.vx.toFixed(2)}, ${this.sensors.odometry.vy.toFixed(2)}, ${this.sensors.odometry.vz.toFixed(2)}]`);
            
            // Détection de changements significatifs dans les capteurs
            const hasSignificantChange = this.sensors.lidar.ranges.some((range, i) => 
                Math.abs(range - previousRanges[i]) > 0.1
            );
            
            if (hasSignificantChange) {
                log(`⚠️ ${this.id} | OBSTACLE DÉTECTÉ: Changement capteur significatif`);
            }
            
            // Publier les données vers ROS2 pour SLAM collaboratif
            if (typeof window !== 'undefined' && window.rosConnector && window.rosConnector.isConnected) {
                const slamData = {
                    drone_id: this.id,
                    timestamp: Date.now(),
                    lidar_ranges: this.sensors.lidar.ranges,
                    odometry: this.sensors.odometry,
                    obstacles_detected: obstacles.length
                };
                
                try {
                    window.rosConnector.publishData('/slam/sensor_data', slamData);
                    if (this._sensorLogCount % 300 === 0) { // Log moins fréquent pour ROS2
                        log(`📤 ${this.id} | SLAM DATA ENVOYÉ vers ROS2: ${obstacles.length} obstacles`);
                    }
                } catch (e) {
                    warn(`⚠️ ${this.id} | Échec publication SLAM ROS2: ${e.message}`);
                }
            }
        }
        
        this.sensors.lidar.lastUpdate = Date.now();
    }

    updateMission(deltaTime, allDrones, obstacles) {
        switch (this.state) {
            case 'IDLE':
                // DÉSACTIVÉ: Décollage automatique supprimé - contrôle manuel uniquement
                // if (this.shouldTakeoff()) {
                //     this.state = 'TAKEOFF';
                //     // Assurer une montée VISIBILE: au moins +2m par rapport à l'altitude actuelle
                //     this.targetAltitude = Math.max(this.targetAltitude || 2.0, this.position.y + 2.0);
                //     this.targetPosition.y = this.targetAltitude;
                // }
                break;

            case 'TAKEOFF':
                // Si la cible a été abaissée accidentellement, la remonter au moins +0.5m
                if ((this.targetAltitude || 0) < this.position.y + 0.5) {
                    this.targetAltitude = this.position.y + 0.5;
                    this.targetPosition.y = this.targetAltitude;
                }
                if (Math.abs(this.position.y - this.targetAltitude) < 0.5) { // Marge plus large pour 15m
                    this.state = 'FLYING';
                    this.missionPhase = this.exploration.mode === 'hover' ? 'HOVER' : 'EXPLORATION';
                    log(`✅ ${this.id} : Altitude cible atteinte (${this.position.y.toFixed(1)}m) - Passage en mode ${this.missionPhase}`);
                }
                break;

            case 'FLYING':
                this.updateFlightMission(deltaTime, allDrones, obstacles);
                break;

            case 'LANDING':
                if (this.position.y < 0.2) {
                    this.state = 'IDLE';
                    this.velocity.set(0, 0, 0);
                }
                break;

            case 'EMERGENCY':
                droneLog(`🚨 ${this.id} : MODE EMERGENCY ACTIF - Position Y: ${this.position.y.toFixed(2)}m`);
                this.emergencyLanding(deltaTime);
                break;
        }
    }

    updateFlightMission(deltaTime, allDrones, obstacles) {
        switch (this.missionPhase) {
            case 'HOVER':
                this.hoverBehavior(); // Vol stationnaire
                break;
                
            case 'EXPLORATION':
                this.explorationBehavior(allDrones, obstacles);
                break;

            case 'FORMATION':
                this.formationBehavior(allDrones);
                break;

            case 'SEARCH_RESCUE':
                this.searchRescueBehavior(obstacles);
                break;

            case 'RETURN_BASE':
                this.returnToBase();
                break;
        }

        // Évitement d'obstacles SEULEMENT pour les drones actifs (pas IDLE)
        if (this.state !== 'IDLE') {
            this.avoidObstacles(obstacles, allDrones);
        }
    }

    hoverBehavior() {
        // Vol stationnaire : maintenir position fixe à l'altitude cible
        if (!this.hoverPosition) {
            // Définir position de hover à la première fois
            this.hoverPosition = this.position.clone();
            this.hoverPosition.y = this.targetAltitude;
            log(`🚁 ${this.id} : Mode HOVER activé à [${this.hoverPosition.x.toFixed(1)}, ${this.hoverPosition.y.toFixed(1)}, ${this.hoverPosition.z.toFixed(1)}]`);
        }
        
        // Maintenir la position de hover
        this.setpoint.position.copy(this.hoverPosition);
        this.setpoint.velocity.set(0, 0, 0); // Vitesse nulle pour stationnaire
        
        // Correction légère pour contrer les dérives
        const drift = this.position.clone().sub(this.hoverPosition);
        if (drift.length() > 0.5) { // Si dérive > 50cm, corriger
            this.targetPosition.copy(this.hoverPosition);
            if (!window.SILENT_MODE && Math.random() < 0.02) {
                log(`🔧 ${this.id} : Correction dérive hover (${drift.length().toFixed(2)}m)`);
            }
        }
    }

    explorationBehavior(allDrones, obstacles) {
        // Exploration basée sur l'intelligence DIAMANTS
        if (this.intelligence > 0.1) {
            // Zone d'intérêt détectée
            const explorationForce = this.gradient.clone().setY(0).normalize().multiplyScalar(0.5);
            this.targetPosition.add(explorationForce);
        } else {
            // Exploration aléatoire avec biais type-spécifique
            const randomForce = new THREE.Vector3(
                (Math.random() - 0.5) * this.typeConfig.agility,
                0,
                (Math.random() - 0.5) * this.typeConfig.agility
            );
            this.targetPosition.add(randomForce);
        }

        // Contraintes spatiales (X/Z dans l'étendue du terrain, Y = altitude)
        // CORRIGÉ: Zone d'exploration agrandie à 50m x 50m pour permettre une vraie exploration
        const bx = this.bounds?.x ?? 50;  // Zone d'exploration de 50m x 50m
        const bz = this.bounds?.y ?? 50;  // Zone d'exploration de 50m x 50m
        this.targetPosition.x = Math.max(-bx / 2 + 1, Math.min(bx / 2 - 1, this.targetPosition.x));
        this.targetPosition.z = Math.max(-bz / 2 + 1, Math.min(bz / 2 - 1, this.targetPosition.z));
        // CORRIGÉ: Altitude minimale 10m (au-dessus plateforme 8.5m) et max 20m pour vol haute altitude
        this.targetPosition.y = Math.max(10, Math.min(20, this.targetPosition.y || this.targetAltitude));
    }

    formationBehavior(allDrones) {
        // Formation en V pour efficacité énergétique
        const leaders = allDrones.filter(d => d.typeConfig.role === 'coordination');
        if (leaders.length > 0 && this.typeConfig.role !== 'coordination') {
            const leader = leaders[0];
            const offset = this.getFormationOffset(leader, allDrones.indexOf(this));
            this.targetPosition.copy(leader.position).add(offset);
        }
    }

    getFormationOffset(leader, index) {
        // Formation en V basée sur l'index
        const spacing = 2.0;
        const side = index % 2 === 0 ? -1 : 1;
        const rank = Math.floor(index / 2);

        return new THREE.Vector3(
            -rank * spacing,
            0,
            side * spacing * (rank + 1)
        );
    }

    avoidObstacles(obstacles, allDrones) {
        const safeDistance = 2.0 * this.typeConfig.scale;
        let avoidanceForce = new THREE.Vector3(0, 0, 0);

        // Évitement obstacles statiques
        obstacles.forEach(obstacle => {
            const distance = this.position.distanceTo(obstacle.position);
            if (distance < safeDistance) {
                const repulsionForce = this.position.clone()
                    .sub(obstacle.position)
                    .normalize()
                    .multiplyScalar((safeDistance - distance) / safeDistance);
                avoidanceForce.add(repulsionForce);
            }
        });

        // Évitement autres drones
        allDrones.forEach(otherDrone => {
            if (otherDrone.id !== this.id) {
                const distance = this.position.distanceTo(otherDrone.position);
                if (distance < safeDistance) {
                    const repulsionForce = this.position.clone()
                        .sub(otherDrone.position)
                        .normalize()
                        .multiplyScalar((safeDistance - distance) / safeDistance * 2);
                    avoidanceForce.add(repulsionForce);
                }
            }
        });

        // Application force d'évitement
        if (avoidanceForce.length() > 0) {
            this.acceleration.add(avoidanceForce.multiplyScalar(2.0));
        }
    }

    updateFlightControl(deltaTime) {
        // SEULEMENT appliquer le contrôle de vol si le drone n'est PAS en état IDLE
        if (this.state === 'IDLE') {
            // Forcer les commandes à zéro pour les drones IDLE
            this.commands.throttle = 0.0;
            this.commands.roll = 0.0;
            this.commands.pitch = 0.0;
            this.commands.yaw = 0.0;
            return; // Sortir immédiatement
        }

        // Mouvement fluide inspiré du prototype SMA.html qui marche
        const positionError = this.targetPosition.clone().sub(this.position);
        const velocityError = positionError.clone().multiplyScalar(0.5).sub(this.velocity);
        const distance = positionError.length();
        
        // Si en exploration et proche du waypoint, passer au suivant
        if (this.flightState.isExploring && this.exploration.currentTarget && distance < 2.0) {
            this._nextWaypoint();
        }
        
        // Force de mouvement proportionnelle (plus agressive que PID classique)
        const maxSpeed = 8.0 * this.typeConfig.agility; // Vitesse max augmentée
        const dampingFactor = 0.85; // Amortissement
        
        // Force d'attraction vers la cible (comme dans prototype SMA)
        const attractionForce = positionError.clone().multiplyScalar(2.0);
        
        // Amortissement de la vitesse pour éviter oscillations
        const dampingForce = this.velocity.clone().multiplyScalar(-dampingFactor);
        
        // Force totale
        const controlForce = new THREE.Vector3()
            .add(attractionForce)
            .add(dampingForce);

        this.acceleration.add(controlForce);

        // Limitation des commandes
        this.acceleration.clampLength(0, 2.0 * this.typeConfig.agility);

        // Deduire des commandes simples (normalisées) pour le mixer
        // Throttle pour maintenir altitude autour de targetPosition.y
        let th = THREE.MathUtils.clamp(0.5 + positionError.y * 0.12 + velocityError.y * 0.05, 0, 1);
        // Renforcer légèrement le décollage pour une ascension immédiatement perceptible
        if (this.state === 'TAKEOFF') {
            // Hausser légèrement le plancher pour un décollage immédiat visible
            th = Math.max(th, 0.75); // Plus de puissance pour être visible
            
            // LOG DEBUG pour vérifier que le système fonctionne
            if (!window.SILENT_MODE && Math.random() < 0.1) { // 10% chance de log
                log(`🔧 ${this.id} TAKEOFF: pos=${this.position.y.toFixed(2)}, target=${this.targetPosition.y.toFixed(2)}, throttle=${th.toFixed(2)}`);
            }
        }
        // Roll (droite +) basé sur erreur latérale Z
        const rl = THREE.MathUtils.clamp(velocityError.z * 0.15, -1, 1);
        // Pitch (avant +) basé sur erreur avant/arrière X
        const pt = THREE.MathUtils.clamp(velocityError.x * 0.15, -1, 1);
        // Yaw: orienter vers la cible en XZ
        let yw = 0;
        try {
            const toTarget = new THREE.Vector3().subVectors(this.targetPosition, this.position);
            const desiredYaw = Math.atan2(toTarget.z, toTarget.x);
            const currentYaw = this.mesh?.rotation?.y || 0;
            let yawErr = desiredYaw - currentYaw;
            // wrap to [-pi,pi]
            yawErr = Math.atan2(Math.sin(yawErr), Math.cos(yawErr));
            yw = THREE.MathUtils.clamp(yawErr * 0.5, -1, 1);
        } catch (_) { /* safe */ }

    this.commands.throttle = th;
    this.commands.roll = rl;
    this.commands.pitch = pt;
    this.commands.yaw = yw;

    // Debug log for takeoff verification (rate-limited)
    if (this.state === 'TAKEOFF') {
        const now = (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
        // Ensure debug object exists so we persist the timestamp
        if (!this._debug) this._debug = {};
        if (!this._debug.lastTakeoffLog || now - this._debug.lastTakeoffLog > 500) {
            log(`🚁 ${this.id} TAKEOFF: throttle=${th.toFixed(2)}, posErr_y=${positionError.y.toFixed(2)}, vel_y=${this.velocity.y.toFixed(2)}`);
            this._debug.lastTakeoffLog = now;
        }
    }

    // Ne pas injecter de compensation de gravité ici: la gravité est gérée en physique.
    // Laisser this.acceleration pour les forces DIAMANTS/évitement uniquement.

        // Mise à jour RPM moteurs via mixer réaliste
        this.updateMotorRPM();
    }

    updateMotorRPM() {
    // Calcul analytique du RPM d'équilibre (hover) à partir de la masse et de kf
    // T_hover_total = m*g  => par moteur: T = (m*g)/4 = kf * omega^2
    // omega_hover = sqrt(T/kf);  rpm_hover = omega_hover * 60 / (2*pi)
        const maxRPM = this.specs.motorParams.maxRotVelocity;
        const kf = this.specs.motorParams.kf;
        const T_per_motor = (this.mass * 9.81) / 4;
        const omega_hover = Math.sqrt(Math.max(1e-9, T_per_motor / kf)); // rad/s
        const hoverRPM = (omega_hover * 60) / (2 * Math.PI);

    // Mixer réaliste (X-config), indexation SDF convertie:
    // M1 (front-right, CCW):  -roll  -pitch  +yaw
    // M2 (front-left,  CW):   +roll  -pitch  -yaw
    // M3 (back-left,   CCW):  +roll  +pitch  +yaw
    // M4 (back-right,  CW):   -roll  +pitch  -yaw
        const thrustTerm = this.commands?.throttle ?? 0.0; // IDLE par défaut
        const rollTerm = this.commands?.roll ?? 0.0;
        const pitchTerm = this.commands?.pitch ?? 0.0;
        const yawTerm = this.commands?.yaw ?? 0.0; // CCW positif

        const yawFor = (dir) => (dir === 'ccw' ? +yawTerm : -yawTerm);

    const scaleT = 2200; // gain -> RPM (poussée plus visible au décollage)
    const scaleRP = 260; // roll/pitch gain -> RPM
    const scaleY = 180;  // yaw gain -> RPM

    // Le throttle est centré à 0.5 pour le stationnaire
    const tRPM = hoverRPM + (thrustTerm - 0.5) * scaleT;
        const r = rollTerm * scaleRP;
        const p = pitchTerm * scaleRP;
    const y1 = yawFor(this.specs.motors[0].dir) * scaleY; // + yaw (ccw)
    const y2 = yawFor(this.specs.motors[1].dir) * scaleY; // - yaw (cw)
    const y3 = yawFor(this.specs.motors[2].dir) * scaleY; // + yaw (ccw)
    const y4 = yawFor(this.specs.motors[3].dir) * scaleY; // - yaw (cw)

    // Apply matrix described above
    this.motors[0].targetRpm = Math.min(maxRPM, Math.max(0, tRPM - r - p + y1)); // M1 (front-right, CCW)
    this.motors[1].targetRpm = Math.min(maxRPM, Math.max(0, tRPM + r - p + y2)); // M2 (front-left, CW)
    this.motors[2].targetRpm = Math.min(maxRPM, Math.max(0, tRPM + r + p + y3)); // M3 (back-left, CCW)
    this.motors[3].targetRpm = Math.min(maxRPM, Math.max(0, tRPM - r + p + y4)); // M4 (back-right, CW)

        // Filtrage RPM (inertie moteurs)
        this.motors.forEach(motor => {
            motor.rpm += (motor.targetRpm - motor.rpm) * 0.22; // plus nerveux au décollage
            
            // store rad/s for physics
            motor.omega = (motor.rpm / 60) * 2 * Math.PI;
            
            // THRUST RÉALISTE: formule polynomiale basée sur identification système réelle
            const rpm = motor.rpm;
            const forceInGrams = this.specs.motorParams.kf * rpm * rpm + 
                               this.specs.motorParams.kf_linear * rpm + 
                               this.specs.motorParams.kf_offset;
            motor.thrust = Math.max(0, forceInGrams * 9.81 / 1000.0); // Convert grams to Newtons
            
            // reaction yaw torque (sign by dir)
            const yawSign = motor.direction === 'cw' ? -1 : 1;
            motor.yawTorque = yawSign * this.specs.motorParams.km * motor.omega * motor.omega; // N·m
        });
    }

    updatePhysics(deltaTime, allDrones = []) {
        // PHYSIQUE RÉALISTE BASÉE SUR CRAZYFLIE ROS2
        
        droneLog(`🔧 DEBUG: updatePhysics called for ${this.id} with ${allDrones.length} drones`);
        
        // Constantes physiques réalistes (basées sur le simulateur ROS2)
        const CRAZYFLIE_MASS = 0.034; // kg - masse réelle d'un Crazyflie 2.0
        const GRAVITY = 9.81; // m/s²
        const ARM_LENGTH = 0.046; // m - longueur des bras
        const THRUST_TO_TORQUE_RATIO = 0.006; // ratio thrust-to-torque
        
        // Conversion RPM vers Force (polynôme issu de l'identification système réelle)
        // Basé sur https://github.com/IMRCLab/crazyflie-system-id
        const rpmToForce = (rpm) => {
            // Version corrigée pour éviter les forces négatives
            if (rpm < 3000) return 0; // En dessous de 3000 RPM, pas de force
            
            const p = [2.55077341e-08, -4.92422570e-05, -1.51910248e-01];
            const forceInGrams = p[0] * rpm * rpm + p[1] * rpm + p[2];
            // CORRECTION: Multiplication par facteur 2.0 pour compenser les hélices plus petites
            const forceInNewtons = Math.max(0, forceInGrams * GRAVITY / 1000.0 * 2.0);
            
            // Debug pour comprendre pourquoi la force est nulle
            if (rpm > 0 && forceInNewtons > 0) {
                droneLog(`DEBUG RPM=${rpm} -> forceGrams=${forceInGrams.toFixed(3)} -> forceN=${forceInNewtons.toFixed(6)}`);
            }
            
            return forceInNewtons;
        };
        
        // États spéciaux simplifiés
        if (this.state === 'IDLE') {
            // Drone au sol, moteurs arrêtés
            this.velocity.set(0, 0, 0);
            this.acceleration.set(0, 0, 0);
            this.orientation.set(0, 0, 0, 1);
            
            for (let i = 0; i < 4; i++) {
                this.motors[i].rpm = 0;
                this.motors[i].omega = 0;
                this.motors[i].thrust = 0;
            }
            
            // Position au sol si pas déjà définie
            if (this.position.y < 0.1) {
                this.position.y = 0.1;
            }
            return;
        }
        
        if (this.state === 'TAKEOFF') {
            // Pour le takeoff, on utilise un contrôleur simplifié vers l'altitude target
            const targetAltitude = this.targetAltitude || 15.0;
            
            // Contrôleur PID simple pour l'altitude
            const altitudeError = targetAltitude - this.position.y;
            
            // PID corrigé : plus de poussée quand on est en dessous, moins quand on arrive
            const thrustCommand = 0.6 + Math.max(0, altitudeError * 0.15);
            
            // Conversion en RPM - garantir une marge > hover pour éviter toute chute
            // Hover ~14.2k RPM par moteur (observations). On met une base à 15.5k et un plancher à 15k.
            const baseRpm = 15500;
            const controlRpm = Math.max(15000, Math.min(20000, baseRpm + thrustCommand * 2000));
            
            for (let i = 0; i < 4; i++) {
                this.motors[i].rpm = controlRpm;
                this.motors[i].omega = (this.motors[i].rpm / 60) * 2 * Math.PI;
                this.motors[i].thrust = rpmToForce(this.motors[i].rpm);
            }
            
            // Appliquer la physique basique pour le takeoff
            const totalThrust = this.motors.reduce((sum, motor) => sum + motor.thrust, 0);
            const netForce = totalThrust - (CRAZYFLIE_MASS * GRAVITY);
            
            this.acceleration.y = netForce / CRAZYFLIE_MASS;
            this.velocity.y += this.acceleration.y * deltaTime;
            this.position.y += this.velocity.y * deltaTime;
            
            // Contrainte de sol - permettre le décollage si thrust suffisante
            if (this.position.y < 0.1) {
                this.position.y = 0.1;
                // Ne bloquer la vitesse descendante QUE si on touche le sol
                if (this.velocity.y < 0) {
                    this.velocity.y = 0;
                }
            }
            
            // Transition vers FLYING si altitude atteinte
            if (Math.abs(altitudeError) < 0.5 && Math.abs(this.velocity.y) < 0.5) {
                this.state = 'FLYING';
                droneLog(`✅ ${this.id} : TAKEOFF terminé à ${this.position.y.toFixed(1)}m - État=FLYING`);
            }
            
            // Calculer la nouvelle position pour la collision detection pendant TAKEOFF
            this.newPosition = this.position.clone();
            this.newPosition.y += this.velocity.y * deltaTime;
            
            // Collision detection AUSSI pendant TAKEOFF pour éviter la traversée de plateforme
            this.handleCollisionDetection(allDrones);
            
            droneLog(`🚁 ${this.id} : TAKEOFF - altitude=${this.position.y.toFixed(2)}m, target=${targetAltitude}m, thrust=${totalThrust.toFixed(3)}N`);
            return;
        }
        
        // PHYSIQUE COMPLÈTE POUR LE VOL (état FLYING)
        if (this.state === 'FLYING') {
            // Mise à jour des moteurs via le contrôleur d'attitude
            this.updateAttitudeController(deltaTime);
            
            // Calcul des forces réalistes des moteurs
            for (let i = 0; i < 4; i++) {
                this.motors[i].thrust = rpmToForce(this.motors[i].rpm);
                this.motors[i].omega = (this.motors[i].rpm / 60) * 2 * Math.PI;
            }
            
            // Application de la physique réaliste
            const forces = this.motors.map(motor => motor.thrust);
            
            // Matrice de mélange Crazyflie (B0 matrix)
            const arm = 0.707106781 * ARM_LENGTH; // 45° arm position
            const t2t = THRUST_TO_TORQUE_RATIO;
            
            // [thrust, roll_torque, pitch_torque, yaw_torque] = B0 * [f1, f2, f3, f4]
            const totalThrust = forces[0] + forces[1] + forces[2] + forces[3];
            const rollTorque = (-arm * forces[0]) + (-arm * forces[1]) + (arm * forces[2]) + (arm * forces[3]);
            const pitchTorque = (-arm * forces[0]) + (arm * forces[1]) + (arm * forces[2]) + (-arm * forces[3]);
            const yawTorque = (-t2t * forces[0]) + (t2t * forces[1]) + (-t2t * forces[2]) + (t2t * forces[3]);
            
            // Forces dans le repère du drone
            const thrustBodyFrame = new THREE.Vector3(0, 0, totalThrust);
            
            // Rotation des forces vers le repère monde
            const thrustWorldFrame = thrustBodyFrame.clone().applyQuaternion(this.orientation);
            
            // Forces totales dans le repère monde
            const totalForces = new THREE.Vector3(0, -CRAZYFLIE_MASS * GRAVITY, 0).add(thrustWorldFrame);
            
            // Mise à jour de la dynamique translationelle
            this.acceleration = totalForces.divideScalar(CRAZYFLIE_MASS);
            this.velocity.add(this.acceleration.clone().multiplyScalar(deltaTime));
            
            // Calculer la nouvelle position AVANT collision detection
            // Sécurité : vérifier que position existe
            this.newPosition = this.position.clone().add(this.velocity.clone().multiplyScalar(deltaTime));
            
            // Contraintes de sécurité sur la nouvelle position
            if (this.newPosition.y < 0.1) {
                this.newPosition.y = 0.1;
                this.velocity.y = Math.max(0, this.velocity.y);
                this.velocity.x *= 0.8; // Friction au sol
                this.velocity.z *= 0.8;
            }
            
            // Limites de l'espace de vol
            if (this.newPosition.y > 50) {
                this.newPosition.y = 50;
                this.velocity.y = Math.min(0, this.velocity.y);
            }
            
            // Collision detection ACTIVÉ pour éviter la traversée de plateforme
            this.handleCollisionDetection(allDrones);
        }

        // Mise à jour visuelle
        if (this.mesh) {
            this.mesh.position.copy(this.position);
            this.mesh.quaternion.copy(this.orientation);
        }
        
        // Créer la bounding box si nécessaire
        if (!this.collisionBox && window.THREE && this.scene) {
            this.createCollisionBox();
        }
        
        // Mettre à jour la bounding box
        this.updateCollisionBox();
        
        // Animation des hélices
        this.updatePropellerAnimation(deltaTime);
    }

    // Contrôleur PID de position : génère les commandes d'attitude pour atteindre une position
    updatePositionController(deltaTime) {
        const posError = this.setpoint.position.clone().sub(this.position);
        const velError = this.setpoint.velocity.clone().sub(this.velocity);
        
        // PID Position X → Roll command
        const rollCmd = this.computePID(this.positionPID.x, posError.x, deltaTime);
        
        // PID Position Z → Pitch command (attention: Z vers avant = pitch négatif)
        const pitchCmd = -this.computePID(this.positionPID.z, posError.z, deltaTime);
        
        // PID Position Y → Throttle command  
        const throttleCmd = this.computePID(this.positionPID.y, posError.y, deltaTime);
        
        // Limiter les commandes d'attitude (max ~15 degrés comme un vrai drone)
        const MAX_TILT = Math.PI / 12; // 15 degrés
        this.commands.roll = Math.max(-MAX_TILT, Math.min(MAX_TILT, rollCmd));
        this.commands.pitch = Math.max(-MAX_TILT, Math.min(MAX_TILT, pitchCmd));
        
        // Throttle de base + correction altitude (plus de puissance pour atteindre 15m)
        this.commands.throttle = Math.max(0, Math.min(1, 0.65 + throttleCmd * 0.5));
        
        // Yaw vers le setpoint
        this.commands.yaw = this.setpoint.yaw;
        
        // Debug occasionnel
        if (!window.SILENT_MODE && Math.random() < 0.01) {
            log(`🎯 ${this.id} PID: pos_err=[${posError.x.toFixed(2)}, ${posError.y.toFixed(2)}, ${posError.z.toFixed(2)}] → cmd=[roll:${(this.commands.roll*180/Math.PI).toFixed(1)}°, pitch:${(this.commands.pitch*180/Math.PI).toFixed(1)}°, thr:${this.commands.throttle.toFixed(2)}]`);
        }
    }
    
    // Contrôleur PID d'attitude : génère les commandes moteurs pour atteindre l'attitude désirée  
    updateAttitudeController(deltaTime) {
        // Extraire les angles d'Euler de l'orientation actuelle
        const euler = new THREE.Euler().setFromQuaternion(this.orientation, 'YXZ');
        this.attitude.roll = euler.z;   // Roulis
        this.attitude.pitch = euler.x;  // Tangage  
        this.attitude.yaw = euler.y;    // Lacet
        
        // Erreurs d'attitude
        const rollError = this.commands.roll - this.attitude.roll;
        const pitchError = this.commands.pitch - this.attitude.pitch;
        const yawError = this.normalizeAngle(this.commands.yaw - this.attitude.yaw);
        
        // PIDs d'attitude → Commandes en taux angulaires
        this.motorCommands = {
            rollRate: this.computePID(this.attitudePID.roll, rollError, deltaTime),
            pitchRate: this.computePID(this.attitudePID.pitch, pitchError, deltaTime), 
            yawRate: this.computePID(this.attitudePID.yaw, yawError, deltaTime),
            throttle: this.commands.throttle
        };
    }
    
    // Mixeur moteur : convertit les commandes en vitesses individuelles des 4 moteurs
    updateMotorMixer() {
        // Configuration moteurs Crazyflie (+ = CCW, - = CW)
        // M1: Front-Right (CW), M2: Front-Left (CCW), M3: Back-Left (CW), M4: Back-Right (CCW)
        
        const baseThrottle = this.motorCommands.throttle;
        const roll = this.motorCommands.rollRate;
        const pitch = this.motorCommands.pitchRate; 
        const yaw = this.motorCommands.yawRate;
        
        // Mixage authentique Crazyflie
        this.motors[0].targetRpm = baseThrottle + roll + pitch - yaw; // M1 (front-right, CW)
        this.motors[1].targetRpm = baseThrottle - roll + pitch + yaw; // M2 (front-left, CCW)  
        this.motors[2].targetRpm = baseThrottle - roll - pitch - yaw; // M3 (back-left, CW)
        this.motors[3].targetRpm = baseThrottle + roll - pitch + yaw; // M4 (back-right, CCW)
        
        // Normaliser et limiter les commandes moteurs
        for (let i = 0; i < 4; i++) {
            this.motors[i].targetRpm = Math.max(0, Math.min(1, this.motors[i].targetRpm));
        }
    }
    
    // Dynamique des moteurs : convertit les vitesses en forces et torques
    updateMotorDynamics(deltaTime) {
        for (let i = 0; i < 4; i++) {
            const motor = this.motors[i];
            
            // Dynamique du moteur (réponse temporelle)
            const motorTC = 0.02; // Constante de temps moteur ~20ms
            motor.omega += (motor.targetRpm - motor.omega) * deltaTime / motorTC;
            
            // Force de poussée : F = kf * omega^2
            motor.thrust = this.specs.motorParams.kf * motor.omega * motor.omega;
            
            // Torque de réaction : Q = km * omega^2  
            const motorSpec = this.specs.motors[i];
            const torqueSign = (motorSpec.dir === 'cw') ? -1 : 1;
            motor.yawTorque = torqueSign * this.specs.motorParams.km * motor.omega * motor.omega;
        }
    }
    
    // Dynamique du corps rigide : forces et torques → mouvement
    updateRigidBodyDynamics(deltaTime) {
        // Forces/torques from motors in body frame
        const upBody = new THREE.Vector3(0, 1, 0);
        let totalThrust = 0;
        const bodyTorque = new THREE.Vector3(0, 0, 0); // roll (x), pitch (z), yaw (y)

        // Positions des bras moteurs
        const arms = this.specs.motors.map(m => new THREE.Vector3(m.pos.x, m.pos.z, -m.pos.y));

        for (let i = 0; i < 4; i++) {
            const m = this.motors[i];
            totalThrust += m.thrust;
            
            // Torque = r × F (F le long de +Y dans le repère corps)
            const r = arms[i];
            const motorTorque = new THREE.Vector3().copy(r).cross(new THREE.Vector3(0, m.thrust, 0));
            bodyTorque.add(motorTorque);
            
            // Torque de réaction en lacet
            bodyTorque.y += m.yawTorque;
        }

        // Compensation spéciale pour TAKEOFF - augmenter la poussée pour vaincre gravité + montée
        if (this.state === 'TAKEOFF') {
            // Poussée minimale = poids + force d'ascension
            const minThrustForTakeoff = this.mass * 9.81 * 1.8; // 180% du poids pour montée rapide
            totalThrust = Math.max(totalThrust, minThrustForTakeoff);
            droneLog(`⬆️ ${this.id} : TAKEOFF thrust boost - totalThrust=${totalThrust.toFixed(2)}N (min=${minThrustForTakeoff.toFixed(2)}N)`);
        }

        // Transformer la poussée vers le repère monde
        const thrustWorld = upBody.clone().applyQuaternion(this.orientation).multiplyScalar(totalThrust);

        // Gravité
        const gravity = new THREE.Vector3(0, -9.81 * this.mass, 0);

        // Force nette dans le repère monde
        const netForce = thrustWorld.add(gravity);

        // Intégration linéaire (Euler semi-implicite)
        const linAcc = netForce.clone().multiplyScalar(1 / this.mass);
        this.velocity.addScaledVector(linAcc, deltaTime);
        
        // Traînée simple
        this.velocity.multiplyScalar(1 - 0.05 * deltaTime);
        
        // Nouvelle position
        const newPosition = this.position.clone().addScaledVector(this.velocity, deltaTime);
        
        // DEBUG: log physique pour vérifier mouvement
        if (!window.SILENT_MODE && this.state === 'TAKEOFF' && Math.random() < 0.05) {
            log(`🔬 ${this.id} PHYSICS: thrust=${totalThrust.toFixed(2)}N, tilt=[${(this.attitude.roll*180/Math.PI).toFixed(1)}°,${(this.attitude.pitch*180/Math.PI).toFixed(1)}°], acc=${linAcc.length().toFixed(2)}m/s²`);
        }
        
        // Dynamique angulaire dans le repère corps
        const I = this.inertiaDiag;
        const angAcc = new THREE.Vector3(
            bodyTorque.x / I.x,
            bodyTorque.y / I.y,
            bodyTorque.z / I.z
        );
        this.angularVelocity.addScaledVector(angAcc, deltaTime);
        
        // Amortissement angulaire
        this.angularVelocity.multiplyScalar(1 - 0.08 * deltaTime);
        
        // Intégration de l'orientation (quaternions)
        const omegaQuat = new THREE.Quaternion(
            this.angularVelocity.x * 0.5 * deltaTime,
            this.angularVelocity.y * 0.5 * deltaTime,
            this.angularVelocity.z * 0.5 * deltaTime,
            0
        );
        
        const rotationUpdate = new THREE.Quaternion().copy(omegaQuat);
        rotationUpdate.multiply(this.orientation);
        this.orientation.copy(rotationUpdate).normalize();

        // Stocker la nouvelle position pour collision detection
        this.newPosition = newPosition;
        
        // DEBUG: Log détaillé avant la correction d'urgence
        log(`[PHYSICS PRE-CORRECT] ${this.id}: pos.y=${this.position.y.toFixed(3)}, vel.y=${this.velocity.y.toFixed(3)}, newPos.y=${this.newPosition.y.toFixed(3)}`);

        // Appliquer correction d'urgence avant collision detection
        this.handleEmergencyAltitudeCorrection();
    }
    
    // Gestion des collisions
    handleCollisionDetection(allDrones = []) {
        // ANCIEN SYSTÈME DÉSACTIVÉ - Utilise maintenant le système avancé ray-aabb
        droneLog(`🔍 ${this.id} : handleCollisionDetection appelé avec ${allDrones.length} drones`);
        boundaryLog(`🔄 ${this.id} : Redirection vers handleAdvancedCollisionDetection`);
        
        this.handleAdvancedCollisionDetection(allDrones);
    }
    
    // Correction d'urgence altitude
    handleEmergencyAltitudeCorrection() {
        const PLATFORM_HEIGHT = 8.5;
        const MIN_FLIGHT_ALTITUDE = PLATFORM_HEIGHT + 1.5; // 10m minimum, marge sécurité
        
        // Corriger la position actuelle
        if (this.position.y < MIN_FLIGHT_ALTITUDE) {
            const oldY = this.position.y;
            this.position.y = MIN_FLIGHT_ALTITUDE;
            this.velocity.y = Math.max(0, this.velocity.y);
            log(`🚨 CORRECTION D'URGENCE ${this.id}: altitude actuelle ${oldY.toFixed(2)}m → ${this.position.y.toFixed(2)}m (plateforme à ${PLATFORM_HEIGHT}m)`);
        }

        // Corriger aussi newPosition si elle existe
        if (this.newPosition && this.newPosition.y < MIN_FLIGHT_ALTITUDE) {
            const oldY = this.newPosition.y;
            this.newPosition.y = MIN_FLIGHT_ALTITUDE;
            log(`🚨 CORRECTION D'URGENCE ${this.id}: newPosition ${oldY.toFixed(2)}m → ${this.newPosition.y.toFixed(2)}m`);
        }

        // Floor clamp - éviter de tomber sous le sol (mais ne devrait jamais arriver avec plateforme)
        if (this.position.y < 0.05) {
            this.position.y = 0.05;
            if (this.velocity.y < 0) this.velocity.y = 0;
        }
        
        // Floor clamp pour newPosition aussi
        if (this.newPosition && this.newPosition.y < 0.05) {
            this.newPosition.y = 0.05;
            if (this.velocity.y < 0) this.velocity.y = 0;
        }
    }

    // === SYSTÈME DE FORCES DE RÉPULSION ===
    
    /**
     * Applique une force de répulsion contre un autre drone
     */
    applyRepulsionForce(repulsionVector, force, otherDroneId) {
        const timestamp = Date.now();
        this.repulsionForces.set(otherDroneId, {
            vector: { ...repulsionVector },
            force: Math.min(force, this.maxRepulsionForce),
            timestamp: timestamp
        });
        
        droneLog(`🔥 ${this.id} : Force de répulsion ${force.toFixed(3)}m/s appliquée contre ${otherDroneId}`);
    }
    
    /**
     * Met à jour et applique toutes les forces de répulsion actives
     */
    updateRepulsionForces(deltaTime) {
        const now = Date.now();
        const totalRepulsion = new THREE.Vector3(0, 0, 0);
        
        // Nettoyer les forces expirées et appliquer les forces actives
        for (const [droneId, repulsion] of this.repulsionForces.entries()) {
            // Supprimer les forces trop anciennes
            if (now - repulsion.timestamp > this.repulsionTimeout) {
                this.repulsionForces.delete(droneId);
                continue;
            }
            
            // Décroissance de la force
            repulsion.force *= this.repulsionDecayRate;
            
            // Supprimer les forces trop faibles
            if (repulsion.force < 0.01) {
                this.repulsionForces.delete(droneId);
                continue;
            }
            
            // Appliquer la force de répulsion
            totalRepulsion.add(new THREE.Vector3(
                repulsion.vector.x * repulsion.force,
                repulsion.vector.y * repulsion.force * 0.3, // Répulsion verticale réduite
                repulsion.vector.z * repulsion.force
            ));
        }
        
        // Appliquer la force totale au setpoint du drone
        if (totalRepulsion.length() > 0) {
            this.setpoint.position.add(totalRepulsion.multiplyScalar(deltaTime));
            droneLog(`🚀 ${this.id} : Force de répulsion totale appliquée: [${totalRepulsion.x.toFixed(3)}, ${totalRepulsion.y.toFixed(3)}, ${totalRepulsion.z.toFixed(3)}]`);
        }
    }
    
    /**
     * Force l'arrêt de toutes les forces de répulsion (utile en cas d'urgence)
     */
    clearRepulsionForces() {
        this.repulsionForces.clear();
        droneLog(`🧹 ${this.id} : Toutes les forces de répulsion supprimées`);
    }

    // === SYSTÈME DE COLLISION AVANCÉ AVEC RAY-AABB ===
    
    /**
     * Détection de collision avancée utilisant ray-aabb
     * Vérifie les collisions avec d'autres drones et obstacles
     */
    checkAdvancedCollisions(newPosition, otherDrones = []) {
        droneLog(`🔧 ${this.id} : checkAdvancedCollisions - pos=[${newPosition?.x?.toFixed(2)}, ${newPosition?.y?.toFixed(2)}, ${newPosition?.z?.toFixed(2)}], otherDrones=${otherDrones.length}`);
        
        if (!newPosition || !createRay) {
            droneLog(`⚠️ ${this.id} : checkAdvancedCollisions FAILED - newPosition=${!!newPosition}, createRay=${!!createRay}`);
            return { hasCollision: false, correctedPosition: newPosition };
        }

        const currentPos = this.position;
        const targetPos = newPosition;
        
        // Calculer le rayon et la direction du mouvement
        const direction = [
            targetPos.x - currentPos.x,
            targetPos.y - currentPos.y,
            targetPos.z - currentPos.z
        ];
        
        // Normaliser la direction si elle n'est pas nulle
        const length = Math.sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
        if (length === 0) {
            return { hasCollision: false, correctedPosition: newPosition };
        }
        
        const normalizedDirection = [
            direction[0] / length,
            direction[1] / length,
            direction[2] / length
        ];
        
        // Créer le rayon
        const ray = createRay([currentPos.x, currentPos.y, currentPos.z], normalizedDirection);
        
        const DRONE_SIZE = 0.3; // Taille d'un Crazyflie réelle (30cm)
        const SAFETY_MARGIN = 0.7; // Marge de sécurité raisonnable (70cm)
        const COLLISION_SIZE = DRONE_SIZE + SAFETY_MARGIN;
        
        // Vérifier les collisions avec les autres drones
        for (const otherDrone of otherDrones) {
            if (otherDrone.id === this.id) continue; // Ne pas se collisionner avec soi-même
            
            const otherPos = otherDrone.position;
            if (!otherPos) continue;
            
            // Calculer la distance directe entre les drones pour debug
            const directDistance = Math.sqrt(
                Math.pow(targetPos.x - otherPos.x, 2) +
                Math.pow(targetPos.y - otherPos.y, 2) +
                Math.pow(targetPos.z - otherPos.z, 2)
            );
            
            droneLog(`🔍 ${this.id} vs ${otherDrone.id} : distance directe = ${directDistance.toFixed(2)}m, seuil = ${COLLISION_SIZE.toFixed(2)}m`);
            
            // Créer l'AABB de l'autre drone
            const aabb = [
                [otherPos.x - COLLISION_SIZE/2, otherPos.y - COLLISION_SIZE/2, otherPos.z - COLLISION_SIZE/2],
                [otherPos.x + COLLISION_SIZE/2, otherPos.y + COLLISION_SIZE/2, otherPos.z + COLLISION_SIZE/2]
            ];
            
            // Tester l'intersection avec le rayon
            const normal = [0, 0, 0];
            const distance = ray.intersects(aabb, normal);
            
            droneLog(`🔍 ${this.id} vs ${otherDrone.id} : ray.intersects result = ${distance}, length = ${length.toFixed(3)}`);
            
            if (distance !== false && distance <= length) {
                // Collision détectée !
                boundaryLog(`🚨 ${this.id} : COLLISION avec ${otherDrone.id} à distance ${distance.toFixed(3)}m`);
                
                // Calculer le vecteur de répulsion depuis l'autre drone vers ce drone
                const repulsionVector = {
                    x: currentPos.x - otherPos.x,
                    y: currentPos.y - otherPos.y,
                    z: currentPos.z - otherPos.z
                };
                
                // Normaliser le vecteur de répulsion
                const repulsionLength = Math.sqrt(
                    repulsionVector.x * repulsionVector.x +
                    repulsionVector.y * repulsionVector.y +
                    repulsionVector.z * repulsionVector.z
                );
                
                if (repulsionLength > 0) {
                    repulsionVector.x /= repulsionLength;
                    repulsionVector.y /= repulsionLength;
                    repulsionVector.z /= repulsionLength;
                }
                
                // Force de répulsion inversement proportionnelle à la distance
                const REPULSION_STRENGTH = 2.0; // Force de répulsion en mètres
                const repulsionForce = Math.max(0.5, REPULSION_STRENGTH / Math.max(0.1, directDistance));
                
                // Calculer une position corrigée avec force de répulsion
                const correctedPosition = {
                    x: currentPos.x + repulsionVector.x * repulsionForce,
                    y: Math.max(10, currentPos.y + repulsionVector.y * repulsionForce * 0.5), // Répulsion verticale réduite
                    z: currentPos.z + repulsionVector.z * repulsionForce
                };
                
                // Appliquer une force de répulsion continue au drone
                this.applyRepulsionForce(repulsionVector, repulsionForce, otherDrone.id);
                
                boundaryLog(`🔧 ${this.id} : Force de répulsion appliquée contre ${otherDrone.id} : force=${repulsionForce.toFixed(3)}m`);
                
                return {
                    hasCollision: true,
                    correctedPosition: correctedPosition,
                    collisionWith: otherDrone.id,
                    normal: normal,
                    distance: distance,
                    repulsionVector: repulsionVector,
                    repulsionForce: repulsionForce
                };
            }
        }
        
        // Vérifier collision avec le sol/plateforme
        const PLATFORM_HEIGHT = 8.5;
        const MIN_FLIGHT_ALTITUDE = PLATFORM_HEIGHT + 1.5;
        
        droneLog(`🏢 ${this.id} : Test collision plateforme - altitude actuelle: ${targetPos.y.toFixed(2)}m, limite: ${MIN_FLIGHT_ALTITUDE.toFixed(2)}m`);
        
        if (targetPos.y < MIN_FLIGHT_ALTITUDE) {
            droneLog(`🚨 ${this.id} : COLLISION DÉTECTÉE avec plateforme ! altitude=${targetPos.y.toFixed(2)}m < ${MIN_FLIGHT_ALTITUDE.toFixed(2)}m`);
            boundaryLog(`🚨 ${this.id} : COLLISION avec plateforme à altitude ${targetPos.y.toFixed(2)}m`);
            
            return {
                hasCollision: true,
                correctedPosition: {
                    x: targetPos.x,
                    y: MIN_FLIGHT_ALTITUDE,
                    z: targetPos.z
                },
                collisionWith: 'platform'
            };
        }
        
        // Aucune collision détectée
        droneLog(`✅ ${this.id} : Pas de collision plateforme détectée`);
        return { hasCollision: false, correctedPosition: newPosition };
    }

    /**
     * Version améliorée de la détection de collision qui utilise ray-aabb
     */
    handleAdvancedCollisionDetection(allDrones = []) {
        boundaryLog(`🔄 ${this.id} : handleAdvancedCollisionDetection called`);
        
        if (!this.newPosition) {
            droneLog(`❌ ${this.id} : Pas de newPosition dans updatePhysics`);
            return;
        }
        
        droneLog(`🚀 ${this.id} : Appel checkAdvancedCollisions avec ${allDrones.length} drones`);
        // Utiliser le nouveau système de collision avancé
        const collisionResult = this.checkAdvancedCollisions(this.newPosition, allDrones);
        
        if (collisionResult.hasCollision) {
            // Collision détectée, appliquer la correction
            this.position.copy(collisionResult.correctedPosition);
            
            // Arrêter le mouvement dans la direction de la collision
            if (collisionResult.collisionWith === 'platform') {
                this.velocity.y = Math.max(0, this.velocity.y); // Empêcher la descente
            } else {
                // Collision avec un autre drone - réduire la vitesse globale
                this.velocity.multiplyScalar(0.8);
            }
            
            boundaryLog(`🔧 ${this.id} : Collision avec ${collisionResult.collisionWith} corrigée`);
        } else {
            // Pas de collision, utiliser la nouvelle position
            this.position.copy(this.newPosition);
            boundaryLog(`✅ ${this.id} : Aucune collision, position mise à jour`);
        }
        
        // Toujours appliquer la correction d'urgence en dernier
        this.handleEmergencyAltitudeCorrection();
    }

    // Fonction PID générique
    computePID(pid, error, deltaTime) {
        pid.integral += error * deltaTime;
        
        // Anti-windup
        pid.integral = Math.max(-1, Math.min(1, pid.integral));
        
        const derivative = (error - pid.lastError) / deltaTime;
        pid.lastError = error;
        
        return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
    }
    
    // Reset des contrôleurs PID
    resetPIDControllers() {
        Object.values(this.positionPID).forEach(pid => {
            pid.integral = 0;
            pid.lastError = 0;
        });
        Object.values(this.attitudePID).forEach(pid => {
            pid.integral = 0; 
            pid.lastError = 0;
        });
    }
    
    // Normalisation d'angle [-π, π]
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
                // Direction: CW = -1, CCW = +1 (aligné Gazebo)
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

    emergencyLanding(deltaTime) {
        // CORRIGÉ: Emergency landing au-dessus de la plateforme (9m), pas au sol !
        this.targetPosition.y = 9.0; // Au-dessus de la plateforme 8.5m
        // Descente contrôlée mais limitée
        if (this.position.y > 9.0) {
            this.velocity.y = Math.max(-2.0, this.velocity.y - 2.0 * deltaTime);
        } else {
            // Stabiliser au-dessus de la plateforme
            this.velocity.y = 0;
        }
    }

    returnToBase() {
        // Revenir au-dessus de la base à l'altitude cible (Y)
        this.targetPosition.set(0, this.targetAltitude, 0);
        const distanceToBase = this.position.distanceTo(this.targetPosition);

        if (distanceToBase < 1.0) {
            this.state = 'LANDING';
            this.targetPosition.y = 0;
        }
    }

    searchRescueBehavior(obstacles) {
        // Pattern de recherche en spirale
        const time = this.metrics.flightTime;
        const spiralRadius = 5 + time * 0.1;
        const spiralAngle = time * 0.5;

        // Garder l'altitude sur Y
        this.targetPosition.set(
            Math.cos(spiralAngle) * spiralRadius,
            this.targetAltitude,
            Math.sin(spiralAngle) * spiralRadius
        );
    }

    // Interface pour contrôle externe
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

        } catch (error) {
            error(`❌ Erreur création mesh drone ${this.id}:`, error);

            // Fallback : créer un cube simple rouge
            const fallbackGeometry = new THREE.BoxGeometry(1, 1, 1);
            const fallbackMaterial = new THREE.MeshLambertMaterial({ color: 0xFF0000 });
            this.mesh = new THREE.Mesh(fallbackGeometry, fallbackMaterial);
            this.mesh.position.copy(this.position);

            return this.mesh;
        }
    }

    // =============================================
    // SYSTÈME DE VOL ET EXPLORATION
    // =============================================

    /**
     * Démarrer le décollage du drone
     */
    takeoff(targetAltitude = 2.0) {
        // 🔍 LOGS DÉTAILLÉS POUR DEBUGGING DÉCOLLAGE
        const startTime = Date.now();
        const initialPos = { x: this.position.x, y: this.position.y, z: this.position.z };
        const initialState = this.state;
        
        log(`🚁 Drone ${this.id} : ===== DÉMARRAGE DÉCOLLAGE =====`);
        log(`🚁 ${this.id} : Position initiale: [${initialPos.x.toFixed(2)}, ${initialPos.y.toFixed(2)}, ${initialPos.z.toFixed(2)}]`);
        log(`🚁 ${this.id} : État initial: ${initialState} -> TAKEOFF`);
        log(`🚁 ${this.id} : Altitude cible: ${targetAltitude}m`);
        
        // Vérification des pré-requis
        if (!this.motors || this.motors.length !== 4) {
            error(`❌ ${this.id} : Moteurs non initialisés ! (${this.motors?.length || 0}/4)`);
        }
        
        if (!this.propellerAnimation) {
            error(`❌ ${this.id} : Animation des hélices non initialisée !`);
        }
        
        // Vérification WebSocket ROS2
        if (typeof window !== 'undefined' && window.rosConnector) {
            const rosStatus = window.rosConnector.isConnected ? 'CONNECTED' : 'DISCONNECTED';
            log(`🔌 ${this.id} : ROS2 WebSocket: ${rosStatus}`);
            
            if (window.rosConnector.isConnected) {
                // Envoyer commande ROS2
                const takeoffCommand = {
                    drone_id: this.id,
                    command: 'takeoff',
                    target_altitude: targetAltitude,
                    timestamp: startTime
                };
                try {
                    window.rosConnector.sendCommand('/cmd/takeoff', takeoffCommand);
                    log(`📡 ${this.id} : Commande ROS2 envoyée: /cmd/takeoff`);
                } catch (e) {
                    warn(`⚠️ ${this.id} : Échec envoi ROS2: ${e.message}`);
                }
            }
        }
        
        log(`🚁 Drone ${this.id} : Démarrage décollage vers altitude ${targetAltitude}m`);
        this.flightState.isFlying = true;
        this.flightState.mode = 'takeoff';
        this.flightState.targetAltitude = targetAltitude;
        this.flightState.power = this.propellerAnimation.powerMapping.takeoff;
        
        // FORCER état et position cible pour mouvement immédiat
        this.state = 'TAKEOFF';
        this.targetAltitude = Math.max(targetAltitude, this.position.y + 0.5);
        this.targetPosition.y = this.targetAltitude;
        this.commands.throttle = Math.max(0.85, this.commands.throttle || 0.8); // Throttle encore plus élevé pour 15m
        
        log(`🎯 Drone ${this.id} : État forcé -> ${this.state}, cible Y=${this.targetAltitude}m`);
        log(`⚡ Drone ${this.id} : Puissance moteurs -> ${(this.flightState.power * 100).toFixed(1)}%`);
        log(`🎮 Drone ${this.id} : Throttle -> ${this.commands.throttle.toFixed(2)}`);
        
        // Log des données moteurs
        this.motors.forEach((motor, i) => {
            log(`🔧 ${this.id} : Moteur M${i+1} (${motor.id}) - Pos:[${motor.position.x.toFixed(3)}, ${motor.position.y.toFixed(3)}, ${motor.position.z.toFixed(3)}] Dir:${motor.direction}`);
        });
        
        this.updatePropellerSpeeds();
        
        log(`🚁 ${this.id} : ===== FIN PRÉPARATION DÉCOLLAGE =====`);
    }

    /**
     * Démarrer une mission complète (décollage + exploration)
     * Basé sur le prototype SMA.html qui marche
     */
    startMission(targetAltitude = 3.0, explorationMode = 'circle') {
        log(`🚀 Drone ${this.id} : LANCEMENT MISSION - altitude=${targetAltitude}m, mode=${explorationMode}`);
        
        // 1. Forcer l'état de vol actif
        this.state = 'TAKEOFF';
        this.flightState.isFlying = true;
        this.flightState.isExploring = true;
        this.flightState.mode = 'exploration';
        this.flightState.targetAltitude = targetAltitude;
        this.flightState.power = this.propellerAnimation.powerMapping.exploration;
        
        // 2. Définir position cible pour mouvement visible immédiat
        this.targetPosition.copy(this.position);
        this.targetPosition.y = targetAltitude;
        this.targetAltitude = targetAltitude;
        
        // 3. Activer les hélices pour animation visuelle
        this.propellerAnimation.speeds = [
            this.propellerAnimation.maxSpeed * 0.8,
            this.propellerAnimation.maxSpeed * 0.8,
            this.propellerAnimation.maxSpeed * 0.8,
            this.propellerAnimation.maxSpeed * 0.8
        ];
        
        // 4. Démarrer l'exploration en cercle (comme prototype SMA)
        this.exploration.mode = explorationMode;
        this.exploration.bounds = { x: 15, y: 15 };
        this.exploration.centerPoint.copy(this.position);
        this.exploration.centerPoint.y = 0;
        
        // 5. Générer un pattern circulaire simple qui marche
        this._generateCirclePattern();
        
        log(`✅ ${this.id} : Mission activée ! État=${this.state}, exploring=${this.flightState.isExploring}`);
        return true;
    }

    /**
     * Générer un pattern circulaire simple (inspiré du prototype SMA.html)
     */
    _generateCirclePattern() {
        this.exploration.path = [];
        const radius = 8;  // Rayon du cercle d'exploration
        const steps = 16; // Nombre d'étapes
        
        for (let i = 0; i < steps; i++) {
            const angle = (i / steps) * Math.PI * 2;
            const x = this.exploration.centerPoint.x + Math.cos(angle) * radius;
            const z = this.exploration.centerPoint.z + Math.sin(angle) * radius;
            const y = this.flightState.targetAltitude;
            
            this.exploration.path.push(new THREE.Vector3(x, y, z));
        }
        
        this.exploration.pathIndex = 0;
        this.exploration.currentTarget = this.exploration.path[0];
        log(`🎯 ${this.id} : Pattern circulaire généré - ${steps} waypoints, rayon=${radius}m`);
    }

    /**
     * Passer au waypoint suivant dans le pattern d'exploration
     */
    _nextWaypoint() {
        if (!this.exploration.path || this.exploration.path.length === 0) {
            return;
        }
        
        this.exploration.pathIndex = (this.exploration.pathIndex + 1) % this.exploration.path.length;
        this.exploration.currentTarget = this.exploration.path[this.exploration.pathIndex];
        this.targetPosition.copy(this.exploration.currentTarget);
        
        // Log occasionnel pour debug
        if (!window.SILENT_MODE && Math.random() < 0.3) { // 30% chance
            log(`🎯 ${this.id} : Waypoint ${this.exploration.pathIndex}/${this.exploration.path.length} - [${this.targetPosition.x.toFixed(1)}, ${this.targetPosition.y.toFixed(1)}, ${this.targetPosition.z.toFixed(1)}]`);
        }
    }

    /**
     * Démarrer l'exploration de zone
     */
    startExploration(mode = 'grid', bounds = { x: 20, y: 20 }) {
        log(`🎯 Drone ${this.id} : Démarrage exploration mode ${mode}`);
        this.exploration.mode = mode;
        this.exploration.bounds = bounds;
        this.exploration.centerPoint.copy(this.position);
        this.exploration.centerPoint.y = 0; // Centre au sol

        this.generateExplorationPath();
        this.flightState.isExploring = true;
        this.flightState.mode = 'exploration';
        this.flightState.power = this.propellerAnimation.powerMapping.exploration;
        this.updatePropellerSpeeds();
    }

    /**
     * Générer le chemin d'exploration selon le mode sélectionné
     */
    generateExplorationPath() {
        this.exploration.path = [];
        const bounds = this.exploration.bounds;
        const gridSize = this.exploration.gridSize;
        const center = this.exploration.centerPoint;

        switch (this.exploration.mode) {
            case 'grid':
                this.generateGridPath(bounds, gridSize, center);
                break;
            case 'boustrophedon':
                this.generateBoustrophedonPath(bounds, gridSize, center);
                break;
            case 'spiral':
                this.generateSpiralPath(bounds, gridSize, center);
                break;
            case 'coverage':
                this.generateCoveragePath(bounds, gridSize, center);
                break;
            case 'random':
                this.generateRandomPath(bounds, center);
                break;
            default:
                this.generateGridPath(bounds, gridSize, center);
        }

        this.exploration.pathIndex = 0;
        log(`📍 Chemin d'exploration généré : ${this.exploration.path.length} points`);
    }

    /**
     * Génération du chemin en grille
     */
    generateGridPath(bounds, gridSize, center) {
        for (let x = -bounds.x / 2; x <= bounds.x / 2; x += gridSize) {
            for (let z = -bounds.y / 2; z <= bounds.y / 2; z += gridSize) {
                this.exploration.path.push(new THREE.Vector3(
                    center.x + x,
                    this.flightState.targetAltitude,
                    center.z + z
                ));
            }
        }
    }

    /**
     * Génération du chemin boustrophédon (zigzag)
     */
    generateBoustrophedonPath(bounds, gridSize, center) {
        let direction = 1;
        for (let x = -bounds.x / 2; x <= bounds.x / 2; x += gridSize) {
            if (direction > 0) {
                for (let z = -bounds.y / 2; z <= bounds.y / 2; z += gridSize) {
                    this.exploration.path.push(new THREE.Vector3(
                        center.x + x, this.flightState.targetAltitude, center.z + z
                    ));
                }
            } else {
                for (let z = bounds.y / 2; z >= -bounds.y / 2; z -= gridSize) {
                    this.exploration.path.push(new THREE.Vector3(
                        center.x + x, this.flightState.targetAltitude, center.z + z
                    ));
                }
            }
            direction *= -1;
        }
    }

    /**
     * Génération du chemin en spirale
     */
    generateSpiralPath(bounds, gridSize, center) {
        const maxRadius = Math.max(bounds.x, bounds.y) / 2;
        let radius = 0;
        let angle = 0;
        const angleStep = 0.3;
        const radiusStep = 0.2;

        while (radius < maxRadius) {
            const x = center.x + radius * Math.cos(angle);
            const z = center.z + radius * Math.sin(angle);
            this.exploration.path.push(new THREE.Vector3(x, this.flightState.targetAltitude, z));

            angle += angleStep;
            radius += radiusStep;
        }
    }

    /**
     * Génération du chemin de couverture optimisée
     */
    generateCoveragePath(bounds, gridSize, center) {
        // Algorithme de couverture avec optimisation de distance
        const points = [];
        for (let x = -bounds.x / 2; x <= bounds.x / 2; x += gridSize) {
            for (let z = -bounds.y / 2; z <= bounds.y / 2; z += gridSize) {
                points.push(new THREE.Vector3(
                    center.x + x, this.flightState.targetAltitude, center.z + z
                ));
            }
        }

        // Algorithme du plus proche voisin
        const visited = new Set();
        let current = points[0];
        this.exploration.path.push(current);
        visited.add(0);

        while (visited.size < points.length) {
            let nearestIndex = -1;
            let nearestDistance = Infinity;

            for (let i = 0; i < points.length; i++) {
                if (!visited.has(i)) {
                    const distance = current.distanceTo(points[i]);
                    if (distance < nearestDistance) {
                        nearestDistance = distance;
                        nearestIndex = i;
                    }
                }
            }

            if (nearestIndex !== -1) {
                current = points[nearestIndex];
                this.exploration.path.push(current);
                visited.add(nearestIndex);
            }
        }
    }

    /**
     * Génération du chemin aléatoire
     */
    generateRandomPath(bounds, center) {
        const numPoints = 20;
        for (let i = 0; i < numPoints; i++) {
            const x = center.x + (Math.random() - 0.5) * bounds.x;
            const z = center.z + (Math.random() - 0.5) * bounds.y;
            this.exploration.path.push(new THREE.Vector3(x, this.flightState.targetAltitude, z));
        }
    }

    /**
     * Mise à jour des vitesses des pales selon la puissance
     */
    updatePropellerSpeeds() {
        const basePower = this.flightState.power;

        // Calcul différentiel pour chaque moteur (stabilisation)
        this.propellerAnimation.speeds[0] = basePower * this.propellerAnimation.maxSpeed; // M1 (CW)
        this.propellerAnimation.speeds[1] = basePower * this.propellerAnimation.maxSpeed; // M2 (CCW)
        this.propellerAnimation.speeds[2] = basePower * this.propellerAnimation.maxSpeed; // M3 (CW)
        this.propellerAnimation.speeds[3] = basePower * this.propellerAnimation.maxSpeed; // M4 (CCW)

        // 🔍 LOGS DÉTAILLÉS POUR DEBUGGING - Position et Moteurs
        if (this._motorLogCount === undefined) this._motorLogCount = 0;
        this._motorLogCount++;
        
        if (!window.SILENT_MODE && this._motorLogCount % 30 === 0) { // Log toutes les 30 mises à jour (~0.5s)
            const pos = this.position;
            const motorSpeeds = this.propellerAnimation.speeds.map((s, i) => `M${i+1}:${s.toFixed(0)}`).join(' ');
            const pidInfo = this.pidControllers ? `PID[P:${this.pidControllers.pitch.lastOutput?.toFixed(3)}, R:${this.pidControllers.roll.lastOutput?.toFixed(3)}, Y:${this.pidControllers.yaw.lastOutput?.toFixed(3)}]` : 'PID:OFF';
            
            log(`🚁 ${this.id} | State:${this.state} | Pos:[${pos.x.toFixed(2)},${pos.y.toFixed(2)},${pos.z.toFixed(2)}] | Power:${(basePower*100).toFixed(1)}% | ${motorSpeeds} | ${pidInfo}`);
            
            // Vérifie la cohérence ROS2 si disponible
            if (this.rosData && this.rosData.lastUpdate) {
                const deltaTime = Date.now() - this.rosData.lastUpdate;
                const rosPos = this.rosData.position || {x:0, y:0, z:0};
                const posDiff = Math.sqrt(Math.pow(pos.x - rosPos.x, 2) + Math.pow(pos.y - rosPos.y, 2) + Math.pow(pos.z - rosPos.z, 2));
                
                if (deltaTime > 1000) {
                    warn(`⚠️ ${this.id} | ROS2 DATA STALE: ${deltaTime}ms ago`);
                } else if (posDiff > 0.5) {
                    warn(`⚠️ ${this.id} | POS MISMATCH: Local[${pos.x.toFixed(2)},${pos.y.toFixed(2)},${pos.z.toFixed(2)}] vs ROS[${rosPos.x.toFixed(2)},${rosPos.y.toFixed(2)},${rosPos.z.toFixed(2)}] diff:${posDiff.toFixed(2)}m`);
                }
            }
        }

        if (!window.SILENT_MODE) {
            log(`⚡ Drone ${this.id} : Puissance ${(basePower * 100).toFixed(1)}% - Mode ${this.flightState.mode}`);
        }
    }

    /**
     * Ancienne boucle de vol/animation (legacy).
     * Conservée pour référence/tests, mais non utilisée par défaut.
     */
    updateLegacyFlight(deltaTime) {
        if (!this.flightState.isFlying) return;

        // Mise à jour de l'état de vol
        this.updateFlightState(deltaTime);

        // Mise à jour de l'exploration
        if (this.flightState.isExploring) {
            this.updateExploration(deltaTime);
        }

        // Animation des pales
        this.updatePropellerAnimation(deltaTime);

        // Rafraîchissement visuel (propellers, LED, position)
        try { this.updateVisuals(deltaTime); } catch (_) { /* safe */ }
    }

    /**
     * Mise à jour de l'état de vol
     */
    updateFlightState(deltaTime) {
        switch (this.flightState.mode) {
            case 'takeoff':
                if (this.position.y < this.flightState.targetAltitude) {
                    this.position.y += this.flightState.takeoffSpeed * deltaTime;
                    this.flightState.altitude = this.position.y;
                } else {
                    this.position.y = this.flightState.targetAltitude;
                    this.flightState.mode = 'hover';
                    this.flightState.power = this.propellerAnimation.powerMapping.hover;
                    this.updatePropellerSpeeds();
                    log(`✅ Drone ${this.id} : Altitude cible atteinte (${this.flightState.targetAltitude}m)`);
                }
                break;

            case 'hover':
                // Maintien d'altitude avec petites oscillations réalistes
                this.position.y = this.flightState.targetAltitude + Math.sin(Date.now() * 0.001) * 0.05;
                break;

            case 'exploration':
                // Mouvement vers la cible d'exploration
                if (this.exploration.currentTarget) {
                    this.moveTowardsTarget(deltaTime);
                }
                break;
        }
    }

    /**
     * Définir la position cible (pour le système de formation)
     */
    setTargetPosition(x, y, z) {
        if (!this.targetPosition) {
            this.targetPosition = new THREE.Vector3();
        }
        
        if (typeof x === 'object') {
            // Cas où on passe un Vector3 ou objet {x, y, z}
            this.targetPosition.copy(x);
        } else {
            // Cas où on passe des coordonnées séparées
            this.targetPosition.set(x, y, z);
        }
        
        // S'assurer qu'on est en vol pour se déplacer
        if (this.state === 'IDLE') {
            this.state = 'TAKEOFF';
            // Configuration nécessaire pour le décollage
            this.targetAltitude = Math.max(15.0, this.targetPosition.y); // Au moins 15m
            this.targetPosition.y = this.targetAltitude; // Assurer cohérence
            log(`🚁 ${this.id} : DÉCOLLAGE DÉCLENCHÉ vers [${this.targetPosition.x.toFixed(1)}, ${this.targetPosition.y.toFixed(1)}, ${this.targetPosition.z.toFixed(1)}]`);
        } else if (this.state === 'FLYING') {
            log(`🎯 ${this.id} : Nouvelle cible [${this.targetPosition.x.toFixed(1)}, ${this.targetPosition.y.toFixed(1)}, ${this.targetPosition.z.toFixed(1)}]`);
        }
        
        // Debug: afficher l'état actuel
        log(`📊 ${this.id} : État=${this.state}, Position=[${this.position.x.toFixed(1)}, ${this.position.y.toFixed(1)}, ${this.position.z.toFixed(1)}]`);
    }

    /**
     * Décoller immédiatement (pour le bouton Pattern)
     */
    takeoff() {
        this.state = 'TAKEOFF';
        // Configuration nécessaire pour le décollage
        this.targetAltitude = 15.0; // 15m d'altitude
        if (!this.targetPosition) {
            this.targetPosition = new THREE.Vector3(this.position.x, this.targetAltitude, this.position.z);
        } else {
            this.targetPosition.y = this.targetAltitude;
        }
        log(`🚁 ${this.id} : DÉCOLLAGE IMMÉDIAT FORCÉ vers altitude ${this.targetAltitude}m !`);
    }

    /**
     * Forcer la création des éléments visuels (debug)
     */
    forceCreateVisuals() {
        if (!this.scene) {
            // Essayer de récupérer la scène depuis le système global
            if (window.diamantsSystem && window.diamantsSystem.scene) {
                this.scene = window.diamantsSystem.scene;
            } else if (window.diamantsApp && window.diamantsApp.scene) {
                this.scene = window.diamantsApp.scene;
            }
        }
        
        if (this.scene && window.THREE) {
            this.createCollisionBox();
            log(`🔧 ${this.id} : Éléments visuels forcés`);
        } else {
            log(`❌ ${this.id} : Impossible de créer les visuels - scene=${!!this.scene}, THREE=${!!window.THREE}`);
        }
    }

    /**
     * Mise à jour de l'exploration
     */
    updateExploration(deltaTime) {
        if (!this.exploration.currentTarget && this.exploration.pathIndex < this.exploration.path.length) {
            this.exploration.currentTarget = this.exploration.path[this.exploration.pathIndex];
            log(`🎯 Drone ${this.id} : Nouvelle cible [${this.exploration.pathIndex}/${this.exploration.path.length}]`);
        }

        if (this.exploration.currentTarget) {
            const distance = this.position.distanceTo(this.exploration.currentTarget);
            if (distance < 0.5) { // Arrivé à la cible
                log(`📍 Drone ${this.id} : Zone explorée [${this.exploration.pathIndex + 1}/${this.exploration.path.length}]`);
                this.exploration.pathIndex++;
                this.exploration.currentTarget = null;

                if (this.exploration.pathIndex >= this.exploration.path.length) {
                    log(`✅ Drone ${this.id} : Exploration terminée !`);
                    this.flightState.isExploring = false;
                    this.flightState.mode = 'hover';
                    this.flightState.power = this.propellerAnimation.powerMapping.hover;
                    this.updatePropellerSpeeds();
                }
            }
        }
    }

    /**
     * Mouvement vers la cible
     */
    moveTowardsTarget(deltaTime) {
        if (!this.exploration.currentTarget) return;

        const direction = new THREE.Vector3().subVectors(this.exploration.currentTarget, this.position);
        direction.normalize();

        const speed = 2.0; // m/s
        const movement = direction.multiplyScalar(speed * deltaTime);
        this.position.add(movement);

        // Ajustement de puissance selon l'effort requis
        const effort = Math.min(1.0, movement.length() * 5);
        this.flightState.power = this.propellerAnimation.powerMapping.exploration * (0.5 + effort * 0.5);
        this.updatePropellerSpeeds();
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
            // Alignement avec Gazebo: CW = -1, CCW = +1
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
    handlePlatformCollision() {
        // Rayon de la plateforme circulaire : ~15m (estimé d'après l'image)
        const PLATFORM_RADIUS = 15.0;
        const PLATFORM_CENTER = { x: 0, z: 0 }; // Centre de la plateforme
        const PLATFORM_HEIGHT = 2.0; // Hauteur estimée de la plateforme
        
        // Distance horizontale du centre
        const distanceFromCenter = Math.sqrt(
            (this.position.x - PLATFORM_CENTER.x) ** 2 + 
            (this.position.z - PLATFORM_CENTER.z) ** 2
        );
        
        // Si le drone est au-dessus de la plateforme
        if (distanceFromCenter <= PLATFORM_RADIUS) {
            // Contrainte d'altitude minimum au-dessus de la plateforme
            const minAltitude = PLATFORM_HEIGHT + 8.0; // 8m au-dessus de la plateforme
            
            if (this.position.y < minAltitude) {
                this.position.y = minAltitude;
                this.velocity.y = Math.max(0, this.velocity.y); // Empêcher descente
                
                if (!window.SILENT_MODE && Math.random() < 0.01) { // Log occasionnel
                    log(`🚧 ${this.id} : Collision plateforme évitée - altitude forcée à ${minAltitude}m`);
                }
            }
        }
    }

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
