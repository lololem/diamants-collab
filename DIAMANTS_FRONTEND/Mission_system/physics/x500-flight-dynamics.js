/**
 * DIAMANTS — X500 Realistic Flight Dynamics
 * =============================================
 * Physics-based flight model for Holybro X500 V2 cognitive drones.
 *
 * Unlike the kinematic model (PID→velocity→Euler), this system simulates:
 *   1. Gravity + thrust (force-based)
 *   2. Aerodynamic drag (quadratic in velocity)
 *   3. Cascaded PID control:
 *        Outer loop:  position error → desired velocity
 *        Inner loop:  velocity error → desired acceleration → thrust/attitude
 *   4. Attitude model: roll/pitch derived from horizontal acceleration commands
 *   5. Semi-implicit Euler integration (velocity-first for stability)
 *
 * NaN-safe: every computation has guards (isFinite checks, clamping, fallbacks).
 * The old RealisticFlightDynamics was disabled due to NaN — this one is bulletproof.
 *
 * PX4 reference (real parameter defaults from PX4-Autopilot/ROMFS 4019_x500_v2):
 *   - Position controller: mc_pos_control — MPC_XY_P=0.95, MPC_Z_P=1.0
 *   - Velocity controller: MPC_XY_VEL_P_ACC=1.8, MPC_Z_VEL_P_ACC=4.0
 *   - Attitude controller: MC_ROLL_P=6.5, MC_PITCH_P=6.5, MC_YAW_P=2.8
 *   - Rate controller: MC_ROLLRATE_P=0.14, MC_ROLLRATE_I=0.3, MC_ROLLRATE_D=0.004
 *   - Limits: MPC_TILTMAX_AIR=45°, MPC_THR_HOVER=0.5, TWR~2.4
 *   - We simplify to 2 nested loops at sim rate (~60Hz) which is sufficient
 *     for visual realism without sub-stepping.
 *
 * @module x500-flight-dynamics
 */
import * as THREE from 'three';

// ─── NaN guard: clamp + fallback ────────────────────────────────────
function safe(v, fallback = 0) {
    return Number.isFinite(v) ? v : fallback;
}

function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, safe(v)));
}

function safeVec3(v) {
    v.x = safe(v.x);
    v.y = safe(v.y);
    v.z = safe(v.z);
    return v;
}

// ─── Simple PID with anti-windup + derivative filter ────────────────
class CascadePID {
    /**
     * @param {number} kp
     * @param {number} ki
     * @param {number} kd
     * @param {number} [iMax=5]   Anti-windup clamp
     * @param {number} [outMax=Infinity]  Output clamp
     * @param {number} [dAlpha=0.2]  Derivative low-pass (0–1, lower = smoother)
     */
    constructor(kp, ki, kd, iMax = 5, outMax = Infinity, dAlpha = 0.1) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.iMax = iMax;
        this.outMax = outMax;
        this.dAlpha = dAlpha;
        this._integral = 0;
        this._prevError = 0;
        this._filteredD = 0;
    }

    reset() {
        this._integral = 0;
        this._prevError = 0;
        this._filteredD = 0;
    }

    /**
     * @param {number} error
     * @param {number} dt  seconds (>0)
     * @returns {number} control output
     */
    compute(error, dt) {
        if (dt <= 0) return 0;
        error = safe(error);

        // Proportional
        const P = this.kp * error;

        // Integral with anti-windup
        this._integral += error * dt;
        this._integral = clamp(this._integral, -this.iMax, this.iMax);
        const I = this.ki * this._integral;

        // Derivative (filtered)
        const rawD = (error - this._prevError) / dt;
        this._filteredD += (safe(rawD) - this._filteredD) * this.dAlpha;
        const D = this.kd * this._filteredD;

        this._prevError = error;

        const out = P + I + D;
        return clamp(out, -this.outMax, this.outMax);
    }
}


// ═════════════════════════════════════════════════════════════════════
//  X500 FLIGHT DYNAMICS
// ═════════════════════════════════════════════════════════════════════

export class X500FlightDynamics {
    /**
     * @param {Object} profile  Normalised drone profile from DronePhysicsRegistry
     */
    constructor(profile) {
        // ── Physical constants from profile ──
        this.mass       = profile.mass       || 2.0;   // kg
        this.armLength  = profile.armLength   || 0.25;  // m
        this.maxSpeed   = profile.maxSpeed    || 12.0;  // m/s
        this.maxClimb   = profile.maxClimb    || 5.0;   // m/s
        this.agility    = profile.agility     || 0.8;

        // ── Physics parameters (from profile._raw.physics or defaults) ──
        const phys  = profile._raw?.physics || {};
        this.g              = 9.81;                              // m/s²
        this.hoverThrust    = this.mass * this.g;                // N — force to hover
        this.maxThrust      = phys.maxThrust     || this.hoverThrust * 2.4; // ~2.4:1 TWR (2216 motors/1045 props)
        this.dragCoeffH     = phys.dragCoeffH    || 0.25;        // horizontal drag — clean X500 frame
        this.dragCoeffV     = phys.dragCoeffV    || 0.35;        // vertical drag (prop wash)
        this.maxTiltAngle   = Math.min(phys.maxTiltAngle || 0.44, 0.44); // hard cap ~25° — realistic visual comfort (PX4 allows 45° but too aggressive for sim)
        this.maxAngularRate = phys.maxAngularRate || 3.5;         // rad/s yaw rate (MC_YAWRATE_MAX=200°/s)

        // ── Cascaded PID gains ──
        // Outer loop: position → desired velocity (PX4 MPC_XY_P, MPC_Z_P)
        const pidCfg = profile.pid || {};
        // Position → velocity (outer): moderate P with some D for damping
        const posGains = pidCfg.pos || { kp: 1.5, ki: 0.02, kd: 0.5 };
        const altGains = pidCfg.alt || { kp: 2.0, ki: 0.05, kd: 0.8 };

        // Position → velocity (outer): moderate P, low I, moderate D
        this.pidPosX = new CascadePID(posGains.kp, posGains.ki, posGains.kd, 3, this.maxSpeed);
        this.pidPosZ = new CascadePID(posGains.kp, posGains.ki, posGains.kd, 3, this.maxSpeed);
        this.pidAltOuter = new CascadePID(altGains.kp, altGains.ki, altGains.kd, 3, this.maxClimb);

        // Inner loop: velocity → acceleration (PX4 MPC_XY_VEL_*, MPC_Z_VEL_*)
        // Tighter, faster response than outer loop
        const velKp = phys.velKp || 1.8;       // PX4 MPC_XY_VEL_P_ACC=1.8
        const velKi = phys.velKi || 0.4;       // PX4 MPC_XY_VEL_I_ACC=0.4
        const velKd = phys.velKd || 0.2;       // PX4 MPC_XY_VEL_D_ACC=0.2
        const velAltKp = phys.velAltKp || 4.0; // PX4 MPC_Z_VEL_P_ACC=4.0
        const velAltKi = phys.velAltKi || 2.0; // PX4 MPC_Z_VEL_I_ACC=2.0
        const velAltKd = phys.velAltKd || 0.0; // PX4: no D term on Z velocity

        this.pidVelX = new CascadePID(velKp, velKi, velKd, 8, this.maxAccelH());
        this.pidVelZ = new CascadePID(velKp, velKi, velKd, 8, this.maxAccelH());
        this.pidVelY = new CascadePID(velAltKp, velAltKi, velAltKd, 10, this.maxAccelV());

        // Yaw PID
        const yawGains = pidCfg.yaw || { kp: 2.8, ki: 0.0, kd: 0.15 }; // PX4 MC_YAW_P=2.8
        this.pidYaw = new CascadePID(yawGains.kp, yawGains.ki, yawGains.kd, 2, this.maxAngularRate);

        // ── Per-drone state ──
        // Maintained externally in DroneFlightState; this class is stateless
        // except for these PIDs which are per-dynamics-instance (one per X500 drone).
        // They get attached to each DroneFlightState.

        // ── Attitude state (for visual tilt) ──
        this.roll  = 0;  // rad — bank angle
        this.pitch = 0;  // rad — nose pitch

        console.log(`🛩️  X500FlightDynamics: mass=${this.mass}kg TWR=${(this.maxThrust / this.hoverThrust).toFixed(1)} drag=(${this.dragCoeffH},${this.dragCoeffV})`);
    }

    /** Maximum horizontal acceleration from tilt (a = g × tan(maxTilt)) */
    maxAccelH() {
        return this.g * Math.tan(this.maxTiltAngle); // ~5.7 m/s² at 30°
    }

    /** Maximum vertical acceleration (thrust - hover) / mass */
    maxAccelV() {
        return (this.maxThrust - this.hoverThrust) / this.mass; // ~12 m/s² surplus
    }

    /**
     * Reset all PID integrators (e.g., on phase change).
     */
    resetPIDs() {
        this.pidPosX.reset();
        this.pidPosZ.reset();
        this.pidAltOuter.reset();
        this.pidVelX.reset();
        this.pidVelZ.reset();
        this.pidVelY.reset();
        this.pidYaw.reset();
        this.roll = 0;
        this.pitch = 0;
    }

    /**
     * Full physics step for one X500 drone.
     *
     * Called by AutonomousFlightEngine._updateDrone() when profile is X500.
     * Replaces the simple PID→velocity→Euler path with force-based dynamics.
     *
     * @param {Object} state  DroneFlightState (position, velocity, smoothVelocity, heading, waypoint, ...)
     * @param {number} dt     Frame delta time (seconds)
     * @param {THREE.Vector3} avoidanceForce  Combined avoidance force (trees + drones + boundary)
     * @param {Function} groundY  (x, z) => terrain height
     * @param {number} minAltAboveGround  Minimum altitude above terrain (m)
     * @returns {{ roll: number, pitch: number }}  Attitude angles for visual rendering
     */
    step(state, dt, avoidanceForce, groundY, minAltAboveGround = 1.0) {
        if (!state.waypoint) return { roll: 0, pitch: 0 };

        // Clamp dt to prevent physics explosion
        dt = clamp(dt, 0.001, 0.05); // 20fps–1000fps range

        // ═══ LANDING SPEED ENVELOPE ═══
        // During landing: reduced flight envelope for gentle, non-aggressive PID commands
        const isLandPhase = state.phase === 'LAND';
        const effectiveMaxSpeed = isLandPhase ? Math.min(this.maxSpeed, 2.0) : this.maxSpeed;
        const effectiveMaxClimb = isLandPhase ? Math.min(this.maxClimb, 1.5) : this.maxClimb;

        // ═══════════════════════════════════════════════════════════
        // 1. OUTER LOOP: Position error → desired velocity
        // ═══════════════════════════════════════════════════════════
        const errX = state.waypoint.x - state.position.x;
        const errZ = state.waypoint.z - state.position.z;
        const errY = state.waypoint.y - state.position.y;

        // Position PID → velocity setpoint
        let vxDesired = this.pidPosX.compute(errX, dt);
        let vzDesired = this.pidPosZ.compute(errZ, dt);
        let vyDesired = this.pidAltOuter.compute(errY, dt);

        // Clamp desired velocity to flight envelope
        const hSpeedDesired = Math.sqrt(vxDesired * vxDesired + vzDesired * vzDesired);
        if (hSpeedDesired > effectiveMaxSpeed) {
            const scale = effectiveMaxSpeed / hSpeedDesired;
            vxDesired *= scale;
            vzDesired *= scale;
        }
        vyDesired = clamp(vyDesired, -effectiveMaxClimb, effectiveMaxClimb);

        // ═══════════════════════════════════════════════════════════
        // 2. INNER LOOP: Velocity error → desired acceleration
        // ═══════════════════════════════════════════════════════════
        const velErrX = vxDesired - state.velocity.x;
        const velErrZ = vzDesired - state.velocity.z;
        const velErrY = vyDesired - state.velocity.y;

        let axCmd = this.pidVelX.compute(velErrX, dt);
        let azCmd = this.pidVelZ.compute(velErrZ, dt);
        let ayCmd = this.pidVelY.compute(velErrY, dt);

        // ═══════════════════════════════════════════════════════════
        // 3. AVOIDANCE: Add external forces as acceleration impulse
        // ═══════════════════════════════════════════════════════════
        if (avoidanceForce) {
            // Avoidance force is already in m/s (velocity-like from engine).
            // Convert to acceleration: treat as force/mass with authority scaling.
            // CRITICAL: clamp avoidance contribution to a fraction of the flight
            // envelope to prevent wild jerking. maxAccelH ≈ g·tan(maxTilt) ≈ 9.8 m/s².
            const maxAvoidAccel = this.maxAccelH() * 0.50; // 50% of flight envelope — balanced avoidance vs navigation
            const avoidScale = 1.0; // Moderate avoidance — trunk height has more clearance
            let avX = safe(avoidanceForce.x) * avoidScale;
            let avZ = safe(avoidanceForce.z) * avoidScale;
            let avY = safe(avoidanceForce.y) * avoidScale;
            // Clamp horizontal avoidance to prevent saturation
            const avH = Math.sqrt(avX * avX + avZ * avZ);
            if (avH > maxAvoidAccel) {
                const s = maxAvoidAccel / avH;
                avX *= s;
                avZ *= s;
            }
            avY = clamp(avY, -maxAvoidAccel, maxAvoidAccel);
            axCmd += avX;
            azCmd += avZ;
            ayCmd += avY;
        }

        // ═══════════════════════════════════════════════════════════
        // 4. THRUST MODEL: Convert desired acceleration to thrust
        // ═══════════════════════════════════════════════════════════
        // Total vertical acceleration needed: desired_ay + gravity compensation
        const ayTotal = ayCmd + this.g; // need to counteract gravity
        // Horizontal acceleration limited by tilt angle
        const ahTotal = Math.sqrt(axCmd * axCmd + azCmd * azCmd);

        // Thrust magnitude = mass × √(ah² + av²) where av includes gravity comp
        let thrustMag = this.mass * Math.sqrt(ahTotal * ahTotal + ayTotal * ayTotal);
        thrustMag = clamp(thrustMag, 0, this.maxThrust);

        // ── Attitude from acceleration commands ──
        // tilt angle = atan2(horizontal_accel, vertical_accel)
        const tiltAngle = clamp(Math.atan2(ahTotal, Math.max(0.1, ayTotal)), 0, this.maxTiltAngle);

        // Roll/pitch decomposition from horizontal acceleration direction
        // Roll = tilt in body-X (sideways), Pitch = tilt in body-Z (forward/back)
        // Relative to heading
        const cosH = Math.cos(state.heading);
        const sinH = Math.sin(state.heading);

        // Project horizontal cmd into body frame
        // bodyForward: positive when accelerating in drone's forward direction (-Z in local)
        // bodyRight: positive when accelerating to drone's right (+X in local)
        const bodyForward = -(axCmd * sinH + azCmd * cosH);
        const bodyRight   = axCmd * cosH - azCmd * sinH;
        const accelHLen   = Math.max(0.001, ahTotal);

        // Target roll/pitch — Three.js YXZ convention:
        //   rotation.x < 0 = nose DOWN (correct when accelerating forward)
        //   rotation.z < 0 = bank RIGHT (correct when accelerating right)
        const targetPitch = clamp(-tiltAngle * (bodyForward / accelHLen), -this.maxTiltAngle, this.maxTiltAngle);
        const targetRoll  = clamp(-tiltAngle * (bodyRight  / accelHLen), -this.maxTiltAngle, this.maxTiltAngle);

        // Smooth attitude transition (simulates MC attitude controller response)
        // MC_ROLL_P = 6.5 → attitude bandwidth ~6.5 rad/s (PX4 default for X500)
        const attitudeAlpha = clamp(6.5 * dt, 0, 1);
        this.roll  += (targetRoll  - this.roll)  * attitudeAlpha;
        this.pitch += (targetPitch - this.pitch) * attitudeAlpha;

        // ═══════════════════════════════════════════════════════════
        // 5. FORCES: Gravity + thrust + drag
        // ═══════════════════════════════════════════════════════════
        // Gravity
        const fy_gravity = -this.mass * this.g;

        // Thrust (vertical component = thrust × cos(tilt), horizontal via tilt)
        const thrustAccelY = (thrustMag / this.mass) * Math.cos(tiltAngle);
        const thrustAccelH = (thrustMag / this.mass) * Math.sin(tiltAngle);

        // Decompose horizontal thrust into world X/Z using heading + tilt direction
        let thrustAccelX = 0, thrustAccelZ = 0;
        if (accelHLen > 0.01) {
            thrustAccelX = thrustAccelH * (axCmd / accelHLen);
            thrustAccelZ = thrustAccelH * (azCmd / accelHLen);
        }

        // Aerodynamic drag: F_drag = -C × v × |v|  (quadratic, opposing motion)
        const vx = state.velocity.x;
        const vy = state.velocity.y;
        const vz = state.velocity.z;
        const dragX = -this.dragCoeffH * vx * Math.abs(vx);
        const dragY = -this.dragCoeffV * vy * Math.abs(vy);
        const dragZ = -this.dragCoeffH * vz * Math.abs(vz);

        // ═══════════════════════════════════════════════════════════
        // 6. INTEGRATION: Semi-implicit Euler (velocity first)
        //    More stable than explicit Euler; avoids the NaN spiral
        // ═══════════════════════════════════════════════════════════
        // net acceleration = (thrust + drag) / mass + gravity/mass
        const netAx = thrustAccelX + dragX / this.mass;
        const netAy = thrustAccelY + dragY / this.mass - this.g;
        const netAz = thrustAccelZ + dragZ / this.mass;

        // Update velocity first (semi-implicit)
        state.velocity.x += safe(netAx) * dt;
        state.velocity.y += safe(netAy) * dt;
        state.velocity.z += safe(netAz) * dt;

        // Speed clamp (safety net — physics should normally not exceed this)
        const hSpeed = Math.sqrt(state.velocity.x * state.velocity.x + state.velocity.z * state.velocity.z);
        if (hSpeed > effectiveMaxSpeed * 1.5) {
            const scale = (effectiveMaxSpeed * 1.5) / hSpeed;
            state.velocity.x *= scale;
            state.velocity.z *= scale;
        }
        state.velocity.y = clamp(state.velocity.y, -effectiveMaxClimb * 1.5, effectiveMaxClimb * 1.5);

        // NaN guard on velocity
        safeVec3(state.velocity);

        // Update position from new velocity
        state.position.x += state.velocity.x * dt;
        state.position.y += state.velocity.y * dt;
        state.position.z += state.velocity.z * dt;

        // NaN guard on position
        safeVec3(state.position);

        // ═══════════════════════════════════════════════════════════
        // 7. GROUND + ALTITUDE CONSTRAINTS
        // ═══════════════════════════════════════════════════════════
        const terrainY = groundY ? groundY(state.position.x, state.position.z) : 0;
        const ABSOLUTE_FLOOR = 0.15; // platform surface — NEVER below this
        const effectiveGround = Math.max(terrainY, ABSOLUTE_FLOOR);
        // X500/S500 landing gear extends ~1m below mesh origin at scale=10
        // Extra clearance ensures legs stay above grass (~0.6m above terrain)
        // During landing (minAltAboveGround=0): use visual rest height so drone sits correctly
        // During flight: gear offset ensures legs clear terrain
        const isLanding = minAltAboveGround < 0.1;
        const gearOffset = isLanding
            ? (state.profile?.scale || 10) * 0.22   // visual rest height (matches spawn)
            : ((state.profile?.scale || 10) >= 5 ? 1.5 : 0);
        const minY = effectiveGround + minAltAboveGround + gearOffset;

        if (state.position.y < minY) {
            if (isLanding) {
                // Soft spring: converge smoothly instead of hard snap.
                // Prevents the position discontinuity that causes PID oscillation.
                const pen = minY - state.position.y;
                state.position.y += pen * (1 - Math.exp(-20 * dt));
                if (state.velocity.y < 0) state.velocity.y *= Math.exp(-10 * dt);
            } else {
                state.position.y = minY;
                if (state.velocity.y < 0) state.velocity.y *= -0.15;
            }
        }

        // Max altitude — sane default for forest/urban scenes
        const maxAlt = state.profile?.maxAlt || 20;
        if (state.position.y > maxAlt) {
            state.position.y = maxAlt;
            state.velocity.y = Math.min(0, state.velocity.y);
        }

        // ═══════════════════════════════════════════════════════════
        // 8. YAW CONTROL: Face movement direction
        // ═══════════════════════════════════════════════════════════
        if (hSpeed > 0.5) {
            const targetYaw = Math.atan2(state.velocity.x, state.velocity.z);
            let yawErr = targetYaw - state.heading;
            // Wrap to [-π, π]
            while (yawErr >  Math.PI) yawErr -= 2 * Math.PI;
            while (yawErr < -Math.PI) yawErr += 2 * Math.PI;
            const yawRate = this.pidYaw.compute(yawErr, dt);
            state.heading += clamp(yawRate * dt, -this.maxAngularRate * dt, this.maxAngularRate * dt);
        }
        // Wrap heading to [-π, π] to prevent unbounded accumulation
        while (state.heading > Math.PI) state.heading -= 2 * Math.PI;
        while (state.heading < -Math.PI) state.heading += 2 * Math.PI;

        // Keep smoothVelocity in sync WITH low-pass filter for fluid visual movement
        // This is the key to natural-looking flight — raw physics velocity is too jerky
        const smoothAlpha = Math.min(1.0, this.agility * 3.0 * dt); // matches backup kinematic smoothing
        state.smoothVelocity.x += (state.velocity.x - state.smoothVelocity.x) * smoothAlpha;
        state.smoothVelocity.y += (state.velocity.y - state.smoothVelocity.y) * smoothAlpha;
        state.smoothVelocity.z += (state.velocity.z - state.smoothVelocity.z) * smoothAlpha;

        return {
            roll:  safe(this.roll),
            pitch: safe(this.pitch),
        };
    }
}

export default X500FlightDynamics;
