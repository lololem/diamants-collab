/**
 * DIAMANTS V3 - Comportements de Vol Réalistes
 * ============================================
 * Systèmes de vol authentiques pour Crazyflie avec physique réaliste
 */

import { logger } from '../core/logger.js';

// Force silence total pour les behaviors
if (typeof window !== 'undefined' && window.SILENT_MODE) {
    var originalConsoleLog = console.log;
    console.log = (...args) => {}; // Silence total pour flight-behaviors
}
import { AuthenticCrazyflie } from '../drones/authentic-crazyflie.js';

export class FlightBehaviors {
    constructor(config = {}) {
        logger.info('FlightBehaviors', '✈️ FlightBehaviors.constructor() - Initialisation comportements de vol');

        this.config = {
            safetyDistance: config.safetyDistance || 1.5, // mètres
            maxVelocity: config.maxVelocity || 2.0, // m/s
            maxAcceleration: config.maxAcceleration || 1.0, // m/s²
            altitudeRange: { min: 1.0, max: 8.0 }, // mètres
            windResistance: config.windResistance || 0.1,
            batterySim: config.batterySim || true,
            collisionAvoidance: config.collisionAvoidance || true,
            formationStability: config.formationStability || 0.8,
            ...config
        };

        // États de vol réalistes
        this.flightStates = {
            GROUNDED: 'grounded',
            TAKEOFF: 'takeoff',
            HOVER: 'hover',
            FLYING: 'flying',
            LANDING: 'landing',
            EMERGENCY: 'emergency'
        };

        // Métriques physiques par drone
        this.dronePhysics = new Map(); // droneId -> physicsData

        // Système anti-collision temps réel
        this.collisionSystem = {
            checkInterval: 50, // ms
            lastCheck: 0,
            activeAvoidances: new Map(),
            emergencyManeuvers: new Map()
        };

        // Performances de l'essaim
        this.swarmMetrics = {
            totalFlightTime: 0,
            averageAltitude: 0,
            formationAccuracy: 0,
            collisionCount: 0,
            emergencyLandings: 0
        };
    }

    /**
     * Initialise les données physiques pour un drone
     */
    initializeDronePhysics(droneId, initialPosition = { x: 0, y: 0, z: 0 }) {
        this.dronePhysics.set(droneId, {
            position: { ...initialPosition },
            velocity: { x: 0, y: 0, z: 0 },
            acceleration: { x: 0, y: 0, z: 0 },
            orientation: { pitch: 0, roll: 0, yaw: 0 },
            angularVelocity: { x: 0, y: 0, z: 0 },

            // État physique
            mass: 0.027, // kg (Crazyflie 2.1)
            thrust: 0,
            battery: {
                level: 100, // %
                voltage: 4.2, // V
                consumption: 0.8, // A/h moyenne
                timeRemaining: 7 * 60 // 7 minutes autonomie
            },

            // Capteurs simulés
            sensors: {
                multiranger: { front: 0, back: 0, left: 0, right: 0, up: 0 },
                flowDeck: { deltaX: 0, deltaY: 0, confidence: 0 },
                barometer: { altitude: 0, pressure: 1013.25 },
                imu: {
                    accelerometer: { x: 0, y: 0, z: 9.81 },
                    gyroscope: { x: 0, y: 0, z: 0 }
                }
            },

            // État de vol
            flightState: this.flightStates.GROUNDED,
            targetPosition: { ...initialPosition },
            targetVelocity: { x: 0, y: 0, z: 0 },
            isAutonomous: true,

            // Métriques de performance
            flightTime: 0,
            distanceTraveled: 0,
            energyConsumed: 0
        });
    }

    /**
     * Séquence de décollage réaliste
     */
    async performTakeoff(droneId, targetAltitude = 2.0) {
        const physics = this.dronePhysics.get(droneId);
        if (!physics) {
            console.error(`❌ Drone ${droneId} not initialized`);
            return false;
        }

        physics.flightState = this.flightStates.TAKEOFF;
        if (!window.SILENT_MODE) {
            console.log(`🚁 Drone ${droneId}: Démarrage séquence de décollage...`);
        }

        // Vérifications pré-vol
        if (!this.preFlightChecks(droneId)) {
            console.error(`❌ Drone ${droneId}: Échec des vérifications pré-vol`);
            return false;
        }

        // Séquence de décollage progressive
        // Cibler un plafond sécurisé et éviter une cible en dessous du min configuré
        const maxAllowed = this.config.altitudeRange.max;
        const minOperational = Math.max(0.5, this.config.altitudeRange.min * 0.7);
        const safeTarget = Math.min(Math.max(targetAltitude, minOperational), maxAllowed);

        const takeoffSteps = [
            { altitude: 0.3, duration: 500, description: "Mise en route moteurs - Test hélices" },
            { altitude: 1.0, duration: 800, description: "Décollage initial - Sécurisation" },
            { altitude: safeTarget * 0.5, duration: 1000, description: "Montée première phase" },
            { altitude: safeTarget * 0.8, duration: 800, description: "Montée finale approche" },
            { altitude: safeTarget, duration: 600, description: "Position opérationnelle" }
        ];

        for (const step of takeoffSteps) {
            physics.flightState = this.flightStates.TAKEOFF;
            if (!window.SILENT_MODE) {
                console.log(`  ⬆️ ${step.description}: ${step.altitude}m`);
            }
            await this.smoothAltitudeChange(droneId, step.altitude, step.duration);

            // Vérification sécurité entre chaque étape
            if (!this.checkFlightSafety(droneId)) {
                console.error(`⚠️ Drone ${droneId}: Problème de sécurité détecté`);
                await this.emergencyLanding(droneId);
                return false;
            }
        }

        physics.flightState = this.flightStates.HOVER;
        if (!window.SILENT_MODE) {
            console.log(`✅ Drone ${droneId}: Décollage terminé à ${targetAltitude}m`);
        }
        return true;
    }

    /**
     * Vérifications pré-vol Crazyflie
     */
    preFlightChecks(droneId) {
        const physics = this.dronePhysics.get(droneId);

        // Vérification batterie
        if (physics.battery.level < 20) {
            console.error(`🔋 Drone ${droneId}: Batterie insuffisante (${physics.battery.level}%)`);
            return false;
        }

        // Vérification capteurs
        if (!physics.sensors.imu || !physics.sensors.barometer) {
            console.error(`📡 Drone ${droneId}: Capteurs non disponibles`);
            return false;
        }

        // Vérification espace libre
        if (!this.checkTakeoffClearance(droneId)) {
            console.error(`🚧 Drone ${droneId}: Zone de décollage obstruée`);
            return false;
        }

        return true;
    }

    /**
     * Vérification de l'espace libre pour décollage
     */
    checkTakeoffClearance(droneId) {
        const physics = this.dronePhysics.get(droneId);
        const pos = physics.position;

        // Vérifier absence d'obstacles dans un cylindre de sécurité
        const clearanceRadius = 1.0; // mètre
        const clearanceHeight = 3.0; // mètres

        // Ici on devrait vérifier contre les autres drones et obstacles
        // Pour l'instant, vérification basique
        for (const [otherId, otherPhysics] of this.dronePhysics) {
            if (otherId === droneId) continue;

            const distance = Math.sqrt(
                Math.pow(pos.x - otherPhysics.position.x, 2) +
                Math.pow(pos.z - otherPhysics.position.z, 2)
            );

            if (distance < clearanceRadius && otherPhysics.position.y < clearanceHeight) {
                return false;
            }
        }

        return true;
    }

    /**
     * Changement d'altitude en douceur
     */
    async smoothAltitudeChange(droneId, targetAltitude, duration) {
        const physics = this.dronePhysics.get(droneId);
        const startAltitude = physics.position.y;
        const clampedTarget = Math.min(Math.max(targetAltitude, 0), this.config.altitudeRange.max);
        const startTime = Date.now();

        return new Promise((resolve) => {
            const updateAltitude = () => {
                const elapsed = Date.now() - startTime;
                const progress = Math.min(elapsed / duration, 1.0);

                // Courbe d'accélération/décélération
                const smoothProgress = 0.5 - 0.5 * Math.cos(progress * Math.PI);

                const previousY = physics.position.y;
                physics.position.y = startAltitude + (clampedTarget - startAltitude) * smoothProgress;
                // vitesse verticale approximée
                physics.velocity.y = (physics.position.y - previousY) / 0.016;

                // Simulation consommation batterie
                this.updateBatteryConsumption(droneId, 0.016);

                if (progress >= 1.0) {
                    physics.velocity.y = 0;
                    resolve();
                } else {
                    setTimeout(updateAltitude, 16); // ~60 FPS
                }
            };

            updateAltitude();
        });
    }

    /**
     * Vérifications de sécurité en vol
     */
    checkFlightSafety(droneId) {
        const physics = this.dronePhysics.get(droneId);

        // Vérification batterie critique
        if (physics.battery.level < 10) {
            console.warn(`🔋 Drone ${droneId}: Batterie critique (${physics.battery.level}%)`);
            return false;
        }

        // Vérification altitude dans les limites
        // Pendant le décollage/atterrissage, autoriser une altitude minimale plus basse pour éviter des faux positifs
        const isTransitional = physics.flightState === this.flightStates.TAKEOFF || physics.flightState === this.flightStates.LANDING;
        const minAllowed = isTransitional ? 0.0 : this.config.altitudeRange.min;
        if (physics.position.y > this.config.altitudeRange.max ||
            physics.position.y < minAllowed) {
            console.warn(`⬆️ Drone ${droneId}: Altitude hors limites (${physics.position.y}m)`);
            return false;
        }

        // Vérification vitesse excessive
        // Vitesse horizontale pour l'évaluation de sécurité (ignorer la vitesse verticale)
        const speed = Math.sqrt(
            physics.velocity.x ** 2 +
            physics.velocity.z ** 2
        );

        if (speed > this.config.maxVelocity * 1.2) {
            console.warn(`⚡ Drone ${droneId}: Vitesse excessive (${speed.toFixed(1)} m/s)`);
            return false;
        }

        return true;
    }

    /**
     * Atterrissage d'urgence
     */
    async emergencyLanding(droneId) {
        const physics = this.dronePhysics.get(droneId);
        physics.flightState = this.flightStates.EMERGENCY;

        console.log(`🚨 Drone ${droneId}: ATTERRISSAGE D'URGENCE`);

        // Réduction immédiate de vitesse
        physics.velocity = { x: 0, y: 0, z: 0 };

        // Descente rapide mais contrôlée
        physics.flightState = this.flightStates.LANDING;
        await this.smoothAltitudeChange(droneId, 0, 2000);

        physics.flightState = this.flightStates.GROUNDED;
        this.swarmMetrics.emergencyLandings++;

        if (!window.SILENT_MODE) {
            console.log(`🛬 Drone ${droneId}: Atterrissage d'urgence terminé`);
        }
    }

    /**
     * Simulation consommation batterie
     */
    updateBatteryConsumption(droneId, deltaTime) {
        const physics = this.dronePhysics.get(droneId);

        if (physics.flightState === this.flightStates.GROUNDED) return;

        // Consommation basée sur l'effort (thrust, manœuvres)
        let consumptionRate = physics.battery.consumption; // Base: 0.8 A/h

        // Consommation supplémentaire selon l'activité
        if (physics.flightState === this.flightStates.TAKEOFF) {
            consumptionRate *= 1.5; // +50% pendant décollage
        }

        const speed = Math.sqrt(physics.velocity.x ** 2 + physics.velocity.z ** 2);
        consumptionRate *= (1 + speed / this.config.maxVelocity * 0.3); // +30% à vitesse max

        // Mise à jour batterie
        const consumedAh = consumptionRate * deltaTime / 3600; // Ah consommés
        const batteryCapacity = 0.24; // Ah (240 mAh Crazyflie)
        const percentageConsumed = (consumedAh / batteryCapacity) * 100;

        physics.battery.level = Math.max(0, physics.battery.level - percentageConsumed);
        physics.battery.timeRemaining = (physics.battery.level / 100) * 7 * 60; // secondes

        physics.flightTime += deltaTime;
        physics.energyConsumed += consumedAh;
    }

    /**
     * Système anti-collision temps réel
     */
    updateCollisionAvoidance(deltaTime, activeDroneIds = null) {
        if (!this.config.collisionAvoidance) return;

        const now = Date.now();
        if (now - this.collisionSystem.lastCheck < this.collisionSystem.checkInterval) return;

        this.collisionSystem.lastCheck = now;

        // Utiliser seulement les drones actifs si fourni, sinon tous les drones
        const droneIdsToCheck = activeDroneIds || Array.from(this.dronePhysics.keys());
        const drones = droneIdsToCheck.map(id => [id, this.dronePhysics.get(id)]).filter(([id, physics]) => physics);

        for (let i = 0; i < drones.length; i++) {
            for (let j = i + 1; j < drones.length; j++) {
                const [id1, physics1] = drones[i];
                const [id2, physics2] = drones[j];

                const distance = Math.sqrt(
                    Math.pow(physics1.position.x - physics2.position.x, 2) +
                    Math.pow(physics1.position.y - physics2.position.y, 2) +
                    Math.pow(physics1.position.z - physics2.position.z, 2)
                );

                if (distance < this.config.safetyDistance * 2) {
                    this.executeAvoidanceManeuver(id1, id2, distance);
                }
            }
        }
    }

    /**
     * Manœuvre d'évitement automatique
     */
    executeAvoidanceManeuver(droneId1, droneId2, distance) {
        const physics1 = this.dronePhysics.get(droneId1);
        const physics2 = this.dronePhysics.get(droneId2);

        if (!window.SILENT_MODE) console.log(`⚠️ Évitement collision: Drones ${droneId1} ↔ ${droneId2} (${distance.toFixed(2)}m)`);

        // Vecteur séparation
        const separationVector = {
            x: physics1.position.x - physics2.position.x,
            y: physics1.position.y - physics2.position.y,
            z: physics1.position.z - physics2.position.z
        };

        // Normaliser
        const magnitude = Math.sqrt(separationVector.x ** 2 + separationVector.y ** 2 + separationVector.z ** 2);
        separationVector.x /= magnitude;
        separationVector.y /= magnitude;
        separationVector.z /= magnitude;

        // Force d'évitement proportionnelle à la proximité
        const avoidanceForce = (this.config.safetyDistance - distance) / this.config.safetyDistance;
        const forceMultiplier = 0.5; // m/s

        // Appliquer forces opposées
        physics1.velocity.x += separationVector.x * avoidanceForce * forceMultiplier;
        physics1.velocity.z += separationVector.z * avoidanceForce * forceMultiplier;

        physics2.velocity.x -= separationVector.x * avoidanceForce * forceMultiplier;
        physics2.velocity.z -= separationVector.z * avoidanceForce * forceMultiplier;

        // Limiter vitesses
        this.limitVelocity(droneId1);
        this.limitVelocity(droneId2);
    }

    /**
     * Limitation de vitesse
     */
    limitVelocity(droneId) {
        const physics = this.dronePhysics.get(droneId);
        const speed = Math.sqrt(physics.velocity.x ** 2 + physics.velocity.z ** 2);

        if (speed > this.config.maxVelocity) {
            const factor = this.config.maxVelocity / speed;
            physics.velocity.x *= factor;
            physics.velocity.z *= factor;
        }
    }

    /**
     * Mise à jour générale des comportements de vol
     */
    update(deltaTime, activeDroneIds = null) {
        // Si une liste de drones actifs est fournie, ne traiter que ceux-là
        const droneIdsToUpdate = activeDroneIds || Array.from(this.dronePhysics.keys());
        
        // Mise à jour physique pour les drones actifs seulement
        for (const droneId of droneIdsToUpdate) {
            if (this.dronePhysics.has(droneId)) {
                this.updatePhysics(droneId, deltaTime);
                this.updateBatteryConsumption(droneId, deltaTime);
            }
        }

        // Système anti-collision seulement si il y a des drones actifs
        if (droneIdsToUpdate.length > 1) {
            this.updateCollisionAvoidance(deltaTime, droneIdsToUpdate);
        }

        // Mise à jour métriques essaim
        this.updateSwarmMetrics();
    }

    /**
     * Mise à jour physique individuelle
     */
    updatePhysics(droneId, deltaTime) {
        const physics = this.dronePhysics.get(droneId);

        // Intégration position/vitesse
        physics.position.x += physics.velocity.x * deltaTime;
        physics.position.y += physics.velocity.y * deltaTime;
        physics.position.z += physics.velocity.z * deltaTime;

        // Mise à jour distance parcourue
        const distance = Math.sqrt(
            physics.velocity.x ** 2 +
            physics.velocity.y ** 2 +
            physics.velocity.z ** 2
        ) * deltaTime;
        physics.distanceTraveled += distance;

        // Simulation capteurs
        this.updateSensorData(droneId);
    }

    /**
     * Simulation des données capteurs
     */
    updateSensorData(droneId) {
        const physics = this.dronePhysics.get(droneId);

        // Baromètre
        physics.sensors.barometer.altitude = physics.position.y;
        physics.sensors.barometer.pressure = 1013.25 - (physics.position.y * 12); // hPa

        // IMU
        physics.sensors.imu.accelerometer.x = physics.acceleration.x;
        physics.sensors.imu.accelerometer.y = physics.acceleration.y + 9.81;
        physics.sensors.imu.accelerometer.z = physics.acceleration.z;

        // Flow deck (mouvement optique simulé)
        physics.sensors.flowDeck.deltaX = physics.velocity.x * 16; // pixels/frame
        physics.sensors.flowDeck.deltaY = physics.velocity.z * 16;
        physics.sensors.flowDeck.confidence = physics.position.y < 2.0 ? 0.9 : 0.1;
    }

    /**
     * Métriques de performance de l'essaim
     */
    updateSwarmMetrics() {
        const activeDrones = Array.from(this.dronePhysics.values())
            .filter(p => p.flightState !== this.flightStates.GROUNDED);

        if (activeDrones.length === 0) return;

        // Altitude moyenne
        this.swarmMetrics.averageAltitude = activeDrones
            .reduce((sum, p) => sum + p.position.y, 0) / activeDrones.length;

        // Temps de vol total
        this.swarmMetrics.totalFlightTime = activeDrones
            .reduce((sum, p) => sum + p.flightTime, 0);
    }

    /**
     * Obtenir les métriques de performance
     */
    getPerformanceMetrics() {
        return {
            ...this.swarmMetrics,
            activeDrones: Array.from(this.dronePhysics.values())
                .filter(p => p.flightState !== this.flightStates.GROUNDED).length,
            averageBattery: Array.from(this.dronePhysics.values())
                .reduce((sum, p) => sum + p.battery.level, 0) / this.dronePhysics.size
        };
    }

    /**
     * Données de télémétrie pour un drone
     */
    getTelemetry(droneId) {
        const physics = this.dronePhysics.get(droneId);
        if (!physics) return null;

        return {
            position: { ...physics.position },
            velocity: { ...physics.velocity },
            flightState: physics.flightState,
            battery: { ...physics.battery },
            sensors: { ...physics.sensors },
            metrics: {
                flightTime: physics.flightTime,
                distanceTraveled: physics.distanceTraveled,
                energyConsumed: physics.energyConsumed
            }
        };
    }
}
