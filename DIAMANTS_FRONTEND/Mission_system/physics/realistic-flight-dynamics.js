/**
 * DIAMANTS V3 - Système de Dynamique de Vol Réaliste
 * ===================================================
 * Système unifié qui synchronise physique réaliste, contraintes environnementales 
 * et comportements de vol authentiques pour drones Crazyflie.
 */
import * as THREE from 'three';

export class RealisticFlightDynamics {
    constructor(options = {}) {
        // Constantes physiques Crazyflie réelles
        this.PLATFORM_HEIGHT = 8.5; // m - Hauteur plateforme
        this.MASS = 0.027; // kg - Masse Crazyflie 2.1
        this.MAX_THRUST = 0.60; // N - Poussée maximale
        this.DRAG_COEFFICIENT = 0.25; // Coefficient de traînée
        this.WIND_RESISTANCE = 0.15; // Résistance au vent
        
        // Limites sécuritaires
        this.MIN_ALTITUDE = this.PLATFORM_HEIGHT; // Ne jamais descendre sous la plateforme
        this.MAX_ALTITUDE = this.PLATFORM_HEIGHT + 15; // Limite haute 23.5m
        this.MAX_VELOCITY = 8.0; // m/s - Vitesse max réaliste
        this.EMERGENCY_BRAKE_DISTANCE = 2.0; // m - Distance de freinage d'urgence
        
        // Dynamiques de vol
        this.drones = new Map(); // État physique par drone
        this.lastUpdateTime = Date.now();
        
        // Forces environnementales
        this.gravity = new THREE.Vector3(0, -9.81, 0);
        this.windForce = new THREE.Vector3(0, 0, 0);
        
        console.log('🚁 SYSTÈME DYNAMIQUE VOL RÉALISTE initialisé');
        console.log(`📐 Plateforme: ${this.PLATFORM_HEIGHT}m | Limites: ${this.MIN_ALTITUDE}m - ${this.MAX_ALTITUDE}m`);
    }
    
    /**
     * Initialise la physique pour un drone
     */
    initializeDrone(droneId, initialPosition = null) {
        const safePosition = initialPosition || new THREE.Vector3(0, this.PLATFORM_HEIGHT + 1, 0);
        
        // Assurer position sécuritaire
        safePosition.y = Math.max(safePosition.y, this.MIN_ALTITUDE);
        safePosition.y = Math.min(safePosition.y, this.MAX_ALTITUDE);
        
        const droneState = {
            // Physique
            position: safePosition.clone(),
            velocity: new THREE.Vector3(0, 0, 0),
            acceleration: new THREE.Vector3(0, 0, 0),
            
            // État drone
            thrust: this.MASS * 9.81, // Thrust de sustentation
            orientation: new THREE.Quaternion(),
            targetPosition: safePosition.clone(),
            
            // Contrôles
            pidPosition: {
                x: { error: 0, integral: 0, lastError: 0 },
                y: { error: 0, integral: 0, lastError: 0 },
                z: { error: 0, integral: 0, lastError: 0 }
            },
            
            // Limites et sécurité
            emergencyBrake: false,
            lastGroundContact: 0,
            flightTime: 0
        };
        
        this.drones.set(droneId, droneState);
        console.log(`🚁 Drone ${droneId} initialisé à altitude sécuritaire: ${safePosition.y.toFixed(1)}m`);
        
        return droneState;
    }
    
    /**
     * Met à jour la physique de vol pour tous les drones
     */
    update(deltaTime) {
        const currentTime = Date.now();
        const realDeltaTime = Math.min(deltaTime, 1/30); // Cap à 30fps min pour stabilité
        
        for (const [droneId, state] of this.drones) {
            this.updateDronePhysics(droneId, state, realDeltaTime);
        }
        
        this.lastUpdateTime = currentTime;
    }
    
    /**
     * Physique de vol réaliste pour un drone
     */
    updateDronePhysics(droneId, state, deltaTime) {
        // 1. Calculer erreurs PID pour contrôle position
        this.updatePIDControl(state, deltaTime);
        
        // 2. Calculer forces
        const forces = this.calculateForces(state);
        
        // 3. Appliquer contraintes sécuritaires
        this.applySafetyConstraints(state, forces);
        
        // 4. Intégration physique
        this.integratePhysics(state, forces, deltaTime);
        
        // 5. Vérifications post-intégration
        this.postIntegrationSafety(droneId, state);
        
        state.flightTime += deltaTime;
    }
    
    /**
     * Contrôle PID pour stabilité position
     */
    updatePIDControl(state, deltaTime) {
        const KP = 8.0, KI = 0.5, KD = 2.0; // Gains tuned pour Crazyflie
        
        ['x', 'y', 'z'].forEach(axis => {
            const error = state.targetPosition[axis] - state.position[axis];
            const pid = state.pidPosition[axis];
            
            pid.integral += error * deltaTime;
            pid.integral = Math.max(-5, Math.min(5, pid.integral)); // Anti-windup
            
            const derivative = (error - pid.lastError) / deltaTime;
            const output = KP * error + KI * pid.integral + KD * derivative;
            
            // Appliquer force de contrôle
            state.acceleration[axis] = output;
            pid.lastError = error;
        });
    }
    
    /**
     * Calcul des forces physiques
     */
    calculateForces(state) {
        const forces = {
            thrust: new THREE.Vector3(0, state.thrust, 0),
            gravity: this.gravity.clone().multiplyScalar(this.MASS),
            drag: new THREE.Vector3(),
            control: new THREE.Vector3()
        };
        
        // Force de traînée proportionnelle à v²
        const velocityMagnitude = state.velocity.length();
        if (velocityMagnitude > 0) {
            forces.drag = state.velocity.clone()
                .normalize()
                .multiplyScalar(-this.DRAG_COEFFICIENT * velocityMagnitude * velocityMagnitude);
        }
        
        // Forces de contrôle PID
        forces.control.copy(state.acceleration).multiplyScalar(this.MASS);
        
        return forces;
    }
    
    /**
     * Contraintes de sécurité avant intégration
     */
    applySafetyConstraints(state, forces) {
        // Limite altitude minimale - PRIORITÉ ABSOLUE
        if (state.position.y <= this.MIN_ALTITUDE + 0.1) {
            forces.gravity.y = 0; // Annuler gravité
            forces.control.y = Math.max(forces.control.y, this.MASS * 15); // Force remontée
            state.velocity.y = Math.max(0, state.velocity.y); // Empêcher descente
            
            if (state.velocity.y < 0.1) {
                state.velocity.y = 0.5; // Force remontée minimale
            }
        }
        
        // Limite altitude maximale
        if (state.position.y >= this.MAX_ALTITUDE - 0.5) {
            forces.control.y = Math.min(forces.control.y, -this.MASS * 5);
            state.velocity.y = Math.min(0, state.velocity.y);
        }
        
        // Limite vitesse maximale
        const velocity = state.velocity.length();
        if (velocity > this.MAX_VELOCITY) {
            state.velocity.multiplyScalar(this.MAX_VELOCITY / velocity);
        }
        
        // Freinage d'urgence près des limites
        const distanceToGround = state.position.y - this.MIN_ALTITUDE;
        if (distanceToGround < this.EMERGENCY_BRAKE_DISTANCE && state.velocity.y < 0) {
            const brakeFactor = 1 - (distanceToGround / this.EMERGENCY_BRAKE_DISTANCE);
            state.velocity.y *= (1 - brakeFactor * 0.8);
            state.emergencyBrake = true;
        } else {
            state.emergencyBrake = false;
        }
    }
    
    /**
     * Intégration physique Verlet modifiée
     */
    integratePhysics(state, forces, deltaTime) {
        // Force totale
        const totalForce = new THREE.Vector3()
            .add(forces.thrust)
            .add(forces.gravity)
            .add(forces.drag)
            .add(forces.control);
        
        // Accélération - CLONE avant division pour éviter corruption
        const acceleration = totalForce.clone().divideScalar(this.MASS);
        
        // Intégration Verlet pour stabilité
        const newPosition = state.position.clone()
            .add(state.velocity.clone().multiplyScalar(deltaTime))
            .add(acceleration.clone().multiplyScalar(0.5 * deltaTime * deltaTime));
        
        const newVelocity = state.velocity.clone()
            .add(acceleration.multiplyScalar(deltaTime));
        
        // Mise à jour
        state.position.copy(newPosition);
        state.velocity.copy(newVelocity);
    }
    
    /**
     * Vérifications post-intégration
     */
    postIntegrationSafety(droneId, state) {
        // Forcer contrainte altitude absolue
        if (state.position.y < this.MIN_ALTITUDE) {
            console.warn(`🚨 SÉCURITÉ: Drone ${droneId} sous plateforme! Correction forcée.`);
            state.position.y = this.MIN_ALTITUDE;
            state.velocity.y = Math.max(0, state.velocity.y);
        }
        
        if (state.position.y > this.MAX_ALTITUDE) {
            state.position.y = this.MAX_ALTITUDE;
            state.velocity.y = Math.min(0, state.velocity.y);
        }
        
        // Détection comportement anormal
        if (state.velocity.length() > this.MAX_VELOCITY * 1.5) {
            console.warn(`🚨 VITESSE ANORMALE: Drone ${droneId} à ${state.velocity.length().toFixed(1)}m/s`);
            state.velocity.multiplyScalar(0.5); // Réduction urgente
        }
    }
    
    /**
     * Définit une cible de position pour un drone
     */
    setTargetPosition(droneId, targetPosition) {
        const state = this.drones.get(droneId);
        if (!state) {
            console.warn(`Drone ${droneId} non initialisé`);
            return;
        }
        
        // Contraintes sur la cible
        const safeTarget = targetPosition.clone();
        safeTarget.y = Math.max(safeTarget.y, this.MIN_ALTITUDE + 0.5);
        safeTarget.y = Math.min(safeTarget.y, this.MAX_ALTITUDE - 0.5);
        
        state.targetPosition.copy(safeTarget);
    }
    
    /**
     * Obtient l'état physique d'un drone
     */
    getDroneState(droneId) {
        return this.drones.get(droneId);
    }
    
    /**
     * Force un drone à une altitude sécuritaire
     */
    emergencyAltitudeCorrection(droneId) {
        const state = this.drones.get(droneId);
        if (!state) return;
        
        const emergencyAltitude = this.PLATFORM_HEIGHT + 1.0;
        state.position.y = emergencyAltitude;
        state.targetPosition.y = emergencyAltitude;
        state.velocity.y = 0;
        
        console.log(`🚨 CORRECTION URGENCE: Drone ${droneId} repositionné à ${emergencyAltitude}m`);
    }
    
    /**
     * Statistiques système
     */
    getSystemStats() {
        const stats = {
            dronesCount: this.drones.size,
            averageAltitude: 0,
            emergencyBrakes: 0,
            unsafeDrones: 0
        };
        
        if (stats.dronesCount > 0) {
            let totalAltitude = 0;
            for (const state of this.drones.values()) {
                totalAltitude += state.position.y;
                if (state.emergencyBrake) stats.emergencyBrakes++;
                if (state.position.y < this.MIN_ALTITUDE + 0.5) stats.unsafeDrones++;
            }
            stats.averageAltitude = totalAltitude / stats.dronesCount;
        }
        
        return stats;
    }
}
