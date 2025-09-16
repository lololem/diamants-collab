/**
 * DIAMANTS - Drone Physics Simulation
 * ======================================
 * This class encapsulates the physics simulation for a single drone,
 * including aerodynamics, motor thrust, and PID-based stabilization.
 */
import * as THREE from 'three';
import { PIDController } from './pid-controller.js';

export class DronePhysics {
    constructor(options = {}) {
        // Physical properties (based on Crazyflie 2.1)
        this.mass = options.mass || 0.027; // kg
        this.inertia = options.inertia || new THREE.Matrix3().set(
            1.4e-5, 0, 0,
            0, 1.4e-5, 0,
            0, 0, 2.17e-5
        );
        this.inertiaInverse = new THREE.Matrix3().copy(this.inertia).invert();

        // Aerodynamic properties
        this.dragCoefficient = options.dragCoefficient || 0.25;
        this.liftConstant = options.liftConstant || 3e-6; // Relates RPM^2 to thrust

        // State vectors
        this.position = new THREE.Vector3();
        this.velocity = new THREE.Vector3();
        this.attitude = new THREE.Quaternion(); // World frame
        this.angularVelocity = new THREE.Vector3(); // Body frame

        // PID Controllers for attitude stabilization
        this.pidControllers = {
            // Gains tuned for a responsive but stable Crazyflie
            roll: new PIDController(0.25, 0.05, 0.1),
            pitch: new PIDController(0.25, 0.05, 0.1),
            yaw: new PIDController(0.2, 0.01, 0.05),
            altitude: new PIDController(10.0, 2.0, 3.0)
        };

        // Gravity
        this.gravity = new THREE.Vector3(0, -9.81, 0);
    }

    /**
     * Updates the physics state for a given time step.
     * @param {number} deltaTime - The time step in seconds.
     * @param {Array<number>} motorRPMs - An array of 4 motor RPM values.
     * @param {object} controlTargets - The desired targets { position, yaw }.
     */
    update(deltaTime, motorRPMs, controlTargets) {
        // 1. Calculate forces and torques from motors
        const { totalThrust, totalTorque } = this.calculateMotorEffects(motorRPMs);

        // 2. Calculate aerodynamic drag
        const dragForce = this.calculateDrag();

        // 3. Calculate total force and update linear motion
        const totalForce = new THREE.Vector3()
            .add(totalThrust)
            .add(dragForce)
            .add(this.gravity.clone().multiplyScalar(this.mass));

        const linearAcceleration = totalForce.divideScalar(this.mass);
        this.velocity.add(linearAcceleration.multiplyScalar(deltaTime));
        this.position.add(this.velocity.clone().multiplyScalar(deltaTime));

        // 4. Update angular motion
        const angularAcceleration = totalTorque.clone().applyMatrix3(this.inertiaInverse);
        this.angularVelocity.add(angularAcceleration.multiplyScalar(deltaTime));

        // Integrate angular velocity to get new attitude
        const deltaRotation = new THREE.Quaternion().setFromEuler(
            new THREE.Euler(
                this.angularVelocity.x * deltaTime,
                this.angularVelocity.y * deltaTime,
                this.angularVelocity.z * deltaTime,
                'XYZ'
            )
        );
        this.attitude.multiply(deltaRotation).normalize();

        // 5. Ground constraint - CORRIGÉ pour la plateforme surélevée
        const PLATFORM_HEIGHT = 8.5; // Hauteur réelle de la plateforme
        if (this.position.y < PLATFORM_HEIGHT) {
            this.position.y = PLATFORM_HEIGHT;
            this.velocity.y = Math.max(0, this.velocity.y); // Empêcher de descendre mais autoriser les montées
        }
    }

    /**
     * Calculates the total thrust vector and torque from motor RPMs.
     * @param {Array<number>} motorRPMs - [M1, M2, M3, M4] RPMs.
     * @returns {{totalThrust: THREE.Vector3, totalTorque: THREE.Vector3}}
     */
    calculateMotorEffects(motorRPMs) {
        // Thrust is proportional to RPM^2 and acts along the drone's body Z-axis
        const thrustMagnitude = motorRPMs.reduce((sum, rpm) => sum + this.liftConstant * rpm * rpm, 0);
        const thrustVector = new THREE.Vector3(0, 1, 0) // Body Y is up
            .applyQuaternion(this.attitude)
            .multiplyScalar(thrustMagnitude);

        // Simplified torque calculation (more complex in reality)
        // For now, we assume PID controllers will generate required torques.
        // This part will be driven by the attitude controller.
        const totalTorque = new THREE.Vector3(); // This will be set by the controller

        return { totalThrust: thrustVector, totalTorque };
    }

    /**
     * Calculates aerodynamic drag force.
     * @returns {THREE.Vector3} The drag force vector.
     */
    calculateDrag() {
        return this.velocity.clone().multiplyScalar(-this.dragCoefficient);
    }

    /**
     * Runs the attitude and altitude controllers to get required forces/torques.
     * @param {object} controlTargets - { position, yaw }
     * @returns {{thrust: number, torque: THREE.Vector3}}
     */
    runControllers(controlTargets) {
        // Altitude control -> Vertical thrust
        const altitudeError = controlTargets.position.y - this.position.y;
        const verticalThrust = this.pidControllers.altitude.update(altitudeError);

        // Attitude control -> Torques
        const desiredQuaternion = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, controlTargets.yaw, 0));
        const currentAttitude = this.attitude;

        // Simplified error calculation for roll/pitch
        const desiredRoll = 0; // Assume level flight for now
        const desiredPitch = 0;

        const euler = new THREE.Euler().setFromQuaternion(currentAttitude, 'XYZ');
        const rollError = desiredRoll - euler.x;
        const pitchError = desiredPitch - euler.z;
        const yawError = controlTargets.yaw - euler.y;

        const torque = new THREE.Vector3(
            this.pidControllers.roll.update(rollError),
            this.pidControllers.yaw.update(yawError),
            this.pidControllers.pitch.update(pitchError)
        );

        // Combine hover thrust with vertical control thrust
        const hoverThrust = this.mass * 9.81;
        const totalThrust = hoverThrust + verticalThrust;

        return { thrust: totalThrust, torque };
    }
}
