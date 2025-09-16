/**
 * DIAMANTS - PID Controller
 * ============================
 * A standard PID (Proportional-Integral-Derivative) controller.
 * Used for stabilizing drone attitude (roll, pitch, yaw) and altitude.
 */
export class PIDController {
    constructor(kp, ki, kd, options = {}) {
        this.kp = kp; // Proportional gain
        this.ki = ki; // Integral gain
        this.kd = kd; // Derivative gain

        const { integralLimit = 1.0, outputLimit = 1.0, sampleRate = 1 / 60 } = options;
        this.integralLimit = integralLimit;
        this.outputLimit = outputLimit;
        this.sampleRate = sampleRate;

        this._integral = 0;
        this._previousError = 0;
    }

    /**
     * Resets the controller's internal state.
     */
    reset() {
        this._integral = 0;
        this._previousError = 0;
    }

    /**
     * Updates the controller with a new error value.
     * @param {number} error - The current error (e.g., target_altitude - current_altitude).
     * @returns {number} The calculated control output.
     */
    update(error) {
        // Proportional term
        const p = this.kp * error;

        // Integral term with anti-windup
        this._integral += error * this.sampleRate;
        this._integral = Math.max(-this.integralLimit, Math.min(this.integralLimit, this._integral));
        const i = this.ki * this._integral;

        // Derivative term
        const derivative = (error - this._previousError) / this.sampleRate;
        const d = this.kd * derivative;

        // Combine terms
        let output = p + i + d;

        // Clamp output
        output = Math.max(-this.outputLimit, Math.min(this.outputLimit, output));

        // Store error for next iteration
        this._previousError = error;

        return output;
    }

    /**
     * Sets new PID gains.
     * @param {number} kp - New proportional gain.
     * @param {number} ki - New integral gain.
     * @param {number} kd - New derivative gain.
     */
    setGains(kp, ki, kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.reset();
    }
}
