#!/usr/bin/env python3
"""
DIAMANTS - Swarm Controller Microservice
==========================================
⚠️  v0-origin  (commit 47cec8ee — tag v0-origin)
Restaurer : git checkout v0-origin -- swarm_controller.py

Core swarm intelligence node. Subscribes to all drone odometry,
computes social forces + exploration incentive, and publishes
cmd_vel commands for each drone.

Architecture:
    /{cfN}/odom  →  [SwarmController]  →  /{cfN}/cmd_vel
                                        →  /diamants/swarm/intelligence_score
                                        →  /diamants/swarm/coverage_area
                                        →  /diamants/swarm/status

Based on:
  - multi_agent_framework/swarm_intelligence/swarm_behavior.py (social forces)
  - multi_agent_framework/swarm_intelligence/flight_controller.py (FSM)
  - diamants-private/inference/decision_loop.py (exploration logic)
"""

import math
import time
import json
import random
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32, Bool


# =============================================================================
# Drone State
# =============================================================================

class AltitudePDController:
    """
    PD altitude controller with low-pass filter.
    Ported from slam_collaboratif's altitude_controller.py — the proven
    controller that stabilised Crazyflies in the original slam pipeline.
    """

    # Tuning constants from slam_collaboratif/config.py
    K_Z = 1.05            # Proportional gain
    D_Z = 0.85            # Derivative gain (damping)
    ALPHA_Z = 0.55        # Low-pass filter coefficient
    MAX_ALT_SPEED = 0.25  # Maximum altitude rate (m/s) — increased for stronger hold
    DEADBAND_Z = 0.01     # Dead-zone — no correction if error < 1 cm

    def __init__(self, target_z: float = 0.5):
        self.target_z = target_z
        self.last_z = 0.0
        self.last_time = time.time()
        self.filtered_correction = 0.0

    def compute(self, current_z: float) -> float:
        """Return altitude velocity command (m/s) given current height."""
        now = time.time()
        dt = max(now - self.last_time, 1e-3)

        error_z = self.target_z - current_z
        dz = (current_z - self.last_z) / dt

        # PD raw correction
        raw = self.K_Z * error_z - self.D_Z * dz

        # Dead-zone
        if abs(error_z) < self.DEADBAND_Z:
            raw = 0.0

        # Low-pass filter
        self.filtered_correction = (
            self.ALPHA_Z * raw
            + (1.0 - self.ALPHA_Z) * self.filtered_correction
        )

        # Clamp
        correction = max(-self.MAX_ALT_SPEED,
                         min(self.MAX_ALT_SPEED, self.filtered_correction))

        self.last_z = current_z
        self.last_time = now
        return correction


class DroneState:
    """Per-drone state tracker"""

    def __init__(self, drone_id: str, index: int):
        self.drone_id = drone_id
        self.index = index

        # Position / velocity from Gazebo odom
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.last_odom_time = 0.0

        # Flight state machine
        self.phase = "idle"  # idle, takeoff, cruise, explore, return, land, emergency
        self.phase_start = 0.0
        self.target_altitude = 0.5  # Gazebo scale (meters)
        self.is_armed = False

        # PD altitude controller (ported from slam_collaboratif)
        self.alt_ctrl = AltitudePDController(self.target_altitude)

        # Swarm intelligence
        self.exploration_state = "exploring"
        self.social_force = [0.0, 0.0]
        self.explore_force = [0.0, 0.0]
        self.intelligence_score = 0.0
        self.coverage_contribution = 0.0
        self.social_interactions = 0

        # Obstacle avoidance (from lidar)
        self.nearest_obstacle_dist = float('inf')
        self.nearest_obstacle_angle = 0.0
        self.obstacle_forces = [0.0, 0.0]

        # Exploration waypoint
        self.current_waypoint: Optional[List[float]] = None
        self.waypoint_reached = False
        self.cells_visited: set = set()

        # Propeller RPMs (computed from thrust)
        self.motor_rpms = [0.0, 0.0, 0.0, 0.0]


# =============================================================================
# Swarm Controller Node
# =============================================================================

class SwarmController(Node):
    """
    Main swarm intelligence microservice.

    Manages N drones with:
    - Social forces (repulsion + cohesion, Boids-like)
    - Exploration incentive (frontier-based, coverage grid)
    - Flight state machine (idle→takeoff→cruise→explore→return→land)
    - Obstacle avoidance (reactive, lidar-based)
    - Propeller speed estimation for frontend visualization
    """

    # Drone naming convention: crazyflie, crazyflie1, ..., crazyflie7 (8 total)
    DRONE_NAMES = ["crazyflie", "crazyflie1", "crazyflie2",
                   "crazyflie3", "crazyflie4", "crazyflie5",
                   "crazyflie6", "crazyflie7"]

    # Spawn positions from crazyflie_multi_world.sdf (circle, radius 3m, 45° apart)
    SPAWN_POSITIONS = [
        [ 3.000,  0.000],   # crazyflie   (0°)
        [ 2.121,  2.121],   # crazyflie1  (45°)
        [ 0.000,  3.000],   # crazyflie2  (90°)
        [-2.121,  2.121],   # crazyflie3  (135°)
        [-3.000,  0.000],   # crazyflie4  (180°)
        [-2.121, -2.121],   # crazyflie5  (225°)
        [ 0.000, -3.000],   # crazyflie6  (270°)
        [ 2.121, -2.121],   # crazyflie7  (315°)
    ]

    def __init__(self):
        super().__init__("diamants_swarm_controller")

        # Parameters
        self.declare_parameter("num_drones", 8)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("target_altitude", 0.5)
        self.declare_parameter("exploration_radius", 8.0)
        self.declare_parameter("safe_distance", 0.8)
        self.declare_parameter("max_speed", 0.25)
        self.declare_parameter("takeoff_stagger_sec", 0.5)
        self.declare_parameter("exploration_cell_size", 0.5)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("auto_start_delay", 5.0)

        self.num_drones = self.get_parameter("num_drones").value
        self.control_rate = self.get_parameter("control_rate_hz").value
        self.target_altitude = self.get_parameter("target_altitude").value
        self.exploration_radius = self.get_parameter("exploration_radius").value
        self.safe_distance = self.get_parameter("safe_distance").value
        self.max_speed = self.get_parameter("max_speed").value
        self.takeoff_stagger = self.get_parameter("takeoff_stagger_sec").value
        self.cell_size = self.get_parameter("exploration_cell_size").value

        # State
        self.drones: Dict[str, DroneState] = {}
        self.mission_active = False
        self.mission_start_time = 0.0
        self.coverage_grid: Dict[Tuple[int, int], float] = {}
        self.global_intelligence_score = 0.0

        # Initialize drone states
        for i, name in enumerate(self.DRONE_NAMES[:self.num_drones]):
            self.drones[name] = DroneState(name, i)
            self.drones[name].target_altitude = self.target_altitude

        # --- ROS2 Subscribers ---
        self._odom_subs = {}
        self._scan_subs = {}
        for name in self.drones:
            self._odom_subs[name] = self.create_subscription(
                Odometry, f"/{name}/odom",
                lambda msg, n=name: self._on_odom(n, msg), 10
            )
            self._scan_subs[name] = self.create_subscription(
                LaserScan, f"/{name}/scan",
                lambda msg, n=name: self._on_scan(n, msg), 10
            )

        # Subscribe to mission commands from the API bridge
        self._sub_mission = self.create_subscription(
            String, "/diamants/mission/commands",
            self._on_mission_command, 10
        )
        self._sub_swarm = self.create_subscription(
            String, "/diamants/swarm/commands",
            self._on_swarm_command, 10
        )

        # --- ROS2 Publishers ---
        # Publish directly to /{name}/cmd_vel which goes to Gazebo via bridge.
        # SwarmController handles altitude hold internally (P-controller),
        # so no separate control_services node is needed.
        self._cmd_pubs = {}
        self._enable_pubs = {}
        for name in self.drones:
            self._cmd_pubs[name] = self.create_publisher(
                Twist, f"/{name}/cmd_vel", 10
            )
            self._enable_pubs[name] = self.create_publisher(
                Bool, f"/{name}/enable", 10
            )

        # Swarm metrics → bridge → frontend
        self._pub_intelligence = self.create_publisher(
            Float32, "/diamants/swarm/intelligence_score", 10
        )
        self._pub_coverage = self.create_publisher(
            Float32, "/diamants/swarm/coverage_area", 10
        )
        self._pub_swarm_status = self.create_publisher(
            String, "/diamants/swarm/status", 10
        )
        self._pub_propeller = self.create_publisher(
            String, "/diamants/drones/propeller_speeds", 10
        )

        # --- Timer: main control loop ---
        period = 1.0 / self.control_rate
        self.timer = self.create_timer(period, self._control_loop)

        # Slower metrics publish
        self.metrics_timer = self.create_timer(1.0, self._publish_metrics)

        # CRITICAL: Enable motors immediately at startup so drones don't
        # fall from their spawn altitude before the mission starts.
        # Also send a hover-in-place command to hold spawn altitude.
        self._enable_motors()
        self._send_hover_hold()

        self.get_logger().info(
            f"SwarmController started — {self.num_drones} drones, "
            f"{self.control_rate}Hz, altitude={self.target_altitude}m"
        )

        # Auto-start mission after delay if parameter is set
        auto_start = self.get_parameter("auto_start").value
        auto_delay = self.get_parameter("auto_start_delay").value
        if auto_start:
            self.get_logger().info(
                f"Auto-start enabled — mission will begin in {auto_delay}s"
            )
            self._auto_start_timer = self.create_timer(
                auto_delay, self._auto_start_callback
            )

    # =========================================================================
    # ROS2 Callbacks
    # =========================================================================

    def _on_odom(self, drone_id: str, msg: Odometry):
        """Update drone position from Gazebo odometry."""
        d = self.drones.get(drone_id)
        if not d:
            return
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        d.position = [p.x, p.y, p.z]
        d.velocity = [v.x, v.y, v.z]

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        d.yaw = math.atan2(siny_cosp, cosy_cosp)

        d.last_odom_time = time.time()

    def _on_scan(self, drone_id: str, msg: LaserScan):
        """Process lidar scan for obstacle avoidance."""
        d = self.drones.get(drone_id)
        if not d:
            return

        min_dist = float('inf')
        min_angle = 0.0
        force_x, force_y = 0.0, 0.0

        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment + d.yaw
                if r < min_dist:
                    min_dist = r
                    min_angle = angle
                # Repulsive force inversely proportional to distance
                if r < 2.0:
                    strength = (2.0 - r) / r
                    force_x -= strength * math.cos(angle)
                    force_y -= strength * math.sin(angle)

        d.nearest_obstacle_dist = min_dist
        d.nearest_obstacle_angle = min_angle
        d.obstacle_forces = [force_x, force_y]

    def _on_mission_command(self, msg: String):
        """Handle mission commands from WebSocket bridge."""
        try:
            cmd = json.loads(msg.data)
            data = cmd.get("data", cmd)
            action = data.get("action", "")

            if action == "start":
                self._start_mission()
            elif action == "stop":
                self._stop_mission()
            elif action == "return_home":
                self._return_home()
            elif action == "emergency":
                self._emergency_stop()
            else:
                self.get_logger().info(f"Unknown mission command: {action}")

        except Exception as e:
            self.get_logger().error(f"Mission command error: {e}")

    def _on_swarm_command(self, msg: String):
        """Handle swarm-level commands."""
        try:
            cmd = json.loads(msg.data)
            data = cmd.get("data", cmd)
            action = data.get("action", "")

            if action == "disperse":
                for d in self.drones.values():
                    if d.phase == "explore":
                        d.current_waypoint = None  # Force re-plan
            elif action == "gather":
                for d in self.drones.values():
                    if d.phase == "explore":
                        d.current_waypoint = [0.0, 0.0]  # Center

        except Exception as e:
            self.get_logger().error(f"Swarm command error: {e}")

    # =========================================================================
    # Mission Control
    # =========================================================================

    def _enable_motors(self, silent=False):
        """Enable MulticopterVelocityControl plugin in Gazebo for all drones."""
        msg = Bool()
        msg.data = True
        for name in self.drones:
            pub = self._enable_pubs.get(name)
            if pub:
                pub.publish(msg)
        if not silent:
            self.get_logger().info("Motors ENABLED for all drones")

    def _send_hover_hold(self):
        """Send zero-velocity hover command to all drones (hold position)."""
        hover = Twist()
        # Zero velocities = hold current position/altitude in MulticopterVelocityControl
        for name in self.drones:
            pub = self._cmd_pubs.get(name)
            if pub:
                pub.publish(hover)

    def _disable_motors(self):
        """Disable MulticopterVelocityControl plugin in Gazebo."""
        msg = Bool()
        msg.data = False
        for name in self.drones:
            pub = self._enable_pubs.get(name)
            if pub:
                pub.publish(msg)
        self.get_logger().info("Motors DISABLED for all drones")

    def _auto_start_callback(self):
        """One-shot callback: auto-start the mission after delay."""
        self._auto_start_timer.cancel()
        self.get_logger().info("=== AUTO-START: beginning mission ===")
        self._start_mission()

    def _start_mission(self):
        """Start the exploration mission — staggered takeoff sequence."""
        if self.mission_active:
            self.get_logger().warn("Mission already active")
            return

        self.mission_active = True
        self.mission_start_time = time.time()
        self.coverage_grid.clear()

        # Enable Gazebo MulticopterVelocityControl plugin
        self._enable_motors()

        for d in self.drones.values():
            d.phase = "idle"
            d.phase_start = time.time()
            d.is_armed = True
            d.cells_visited.clear()
            d.current_waypoint = None

        self.get_logger().info("=== MISSION STARTED — staggered takeoff sequence ===")

    def _stop_mission(self):
        """Stop mission — land all drones."""
        self.get_logger().info("=== MISSION STOP — landing all drones ===")
        self.mission_active = False  # allow restart
        for d in self.drones.values():
            d.phase = "land"
            d.phase_start = time.time()

    def _return_home(self):
        """Return all drones to spawn positions."""
        self.get_logger().info("=== RETURN HOME ===")
        self.mission_active = False  # allow restart after return
        for d in self.drones.values():
            d.phase = "return"
            d.phase_start = time.time()
            d.current_waypoint = self.SPAWN_POSITIONS[d.index]

    def _emergency_stop(self):
        """Emergency stop — cut motors."""
        self.get_logger().error("=== EMERGENCY STOP ===")
        self.mission_active = False
        self._disable_motors()
        for d in self.drones.values():
            d.phase = "emergency"
            d.is_armed = False

    # =========================================================================
    # Main Control Loop (10 Hz)
    # =========================================================================

    def _control_loop(self):
        """Main control loop — runs for every drone at control_rate_hz."""
        # Re-send enable every ~1s (every 10 ticks at 10Hz) to ensure
        # Gazebo MulticopterVelocityControl is active even if plugin
        # wasn't ready at boot time.
        if not hasattr(self, '_enable_tick'):
            self._enable_tick = 0
        self._enable_tick += 1
        if self._enable_tick % 10 == 0:
            self._enable_motors(silent=True)

        if not self.mission_active:
            # Keep sending hover commands so drones hold spawn altitude
            self._send_hover_hold()
            return

        now = time.time()
        elapsed = now - self.mission_start_time

        for name, d in self.drones.items():
            # Skip stale drones (no odom in 3 seconds)
            if now - d.last_odom_time > 3.0 and d.last_odom_time > 0:
                continue

            # --- Flight State Machine ---
            cmd = Twist()

            if d.phase == "idle":
                # Wait for staggered takeoff time
                takeoff_time = d.index * self.takeoff_stagger
                if elapsed >= takeoff_time:
                    d.phase = "takeoff"
                    d.phase_start = now
                    self.get_logger().info(f"[{name}] TAKEOFF (t+{elapsed:.1f}s)")

            elif d.phase == "takeoff":
                cmd = self._phase_takeoff(d)

            elif d.phase == "cruise":
                cmd = self._phase_cruise(d)

            elif d.phase == "explore":
                cmd = self._phase_explore(d, now)

            elif d.phase == "return":
                cmd = self._phase_return(d)

            elif d.phase == "land":
                cmd = self._phase_land(d)

            elif d.phase == "emergency":
                cmd = Twist()  # Zero — motors cut

            # Publish command
            self._publish_cmd_vel(name, cmd)

            # Update coverage grid
            if d.phase in ("cruise", "explore"):
                cx = int(d.position[0] / self.cell_size)
                cy = int(d.position[1] / self.cell_size)
                self.coverage_grid[(cx, cy)] = now
                d.cells_visited.add((cx, cy))

            # Estimate propeller RPMs
            self._estimate_rpms(d, cmd)

    # =========================================================================
    # Flight Phases
    # =========================================================================

    def _phase_takeoff(self, d: DroneState) -> Twist:
        """Strong vertical climb to target altitude."""
        cmd = Twist()
        alt_error = d.target_altitude - d.position[2]
        takeoff_elapsed = time.time() - d.phase_start

        if alt_error > 0.05:
            # Very strong climb — MulticopterVelocityControl z-gain is only 0.2425
            # Need large cmd to overcome gravity from ground (z~0.03)
            if d.position[2] < 0.10:
                cmd.linear.z = 1.5  # Extra strong from ground level
            else:
                cmd.linear.z = 0.8

            # Timeout: if still on ground after 20s, force transition
            if takeoff_elapsed > 20.0:
                self.get_logger().warn(
                    f"[{d.drone_id}] Takeoff timeout at z={d.position[2]:.2f}m — "
                    f"forcing transition to cruise"
                )
                d.phase = "cruise"
                d.phase_start = time.time()
        else:
            # Altitude reached → cruise
            d.phase = "cruise"
            d.phase_start = time.time()
            d.alt_ctrl.target_z = d.target_altitude
            self.get_logger().info(
                f"[{d.drone_id}] CRUISE at z={d.position[2]:.2f}m"
            )
        return cmd

    def _phase_cruise(self, d: DroneState) -> Twist:
        """Stabilize at target altitude (PD), then transition to explore."""
        cmd = Twist()
        cmd.linear.z = d.alt_ctrl.compute(d.position[2])

        # After 2 seconds of stable cruise → explore
        if time.time() - d.phase_start > 2.0:
            d.phase = "explore"
            d.phase_start = time.time()
            d.current_waypoint = None
            self.get_logger().info(f"[{d.drone_id}] EXPLORE mode")

        return cmd

    def _phase_explore(self, d: DroneState, now: float) -> Twist:
        """
        Autonomous exploration with swarm intelligence.
        Uses ADDITIVE forces with priority:
          1. Wall avoidance (hard constraint — keeps drones in arena)
          2. Obstacle avoidance (safety — lidar-based)
          3. Social repulsion (collision avoidance between drones)
          4. Waypoint tracking (primary navigation — drives coverage)
          5. Exploration incentive (bias toward uncovered areas)
        Final velocity is clamped to max_speed.
        """
        cmd = Twist()

        # --- 1. Altitude hold (PD controller from slam_collaboratif) ---
        alt_cmd = d.alt_ctrl.compute(d.position[2])

        # SAFETY: Never actively push drone DOWN during explore unless
        # significantly above target (+0.30m). This prevents crash loops
        # where lateral forces cause the multicopter to tilt and lose altitude.
        if d.position[2] < d.target_altitude + 0.30:
            alt_cmd = max(alt_cmd, 0.10)  # Strong upward bias to counter tilt-induced altitude loss

        cmd.linear.z = alt_cmd

        # Altitude recovery: multi-tier response based on severity
        if d.position[2] < 0.20:
            # CRITICAL: Very close to ground — full climb, no lateral
            cmd.linear.z = 1.5  # Maximum force climb (velocityGain z=0.2425 → need huge cmd)
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            return cmd  # Skip all lateral forces
        elif d.position[2] < 0.35:
            # WARNING: altitude marginal — strong climb, heavily reduce lateral
            cmd.linear.z = 0.8
            # Continue below but lateral forces will be scaled down (see below)

        # --- 2. Ramp-up: gradually increase speed over first 2s of explore ---
        explore_age = now - d.phase_start
        ramp = min(1.0, explore_age / 2.0)  # 0→1 over 2 seconds

        # --- 3. Assign exploration waypoint if needed ---
        if d.current_waypoint is None or d.waypoint_reached:
            d.current_waypoint = self._pick_exploration_waypoint(d)
            d.waypoint_reached = False

        # --- 4. Waypoint tracking (PRIMARY force — full max_speed toward waypoint) ---
        wp_x, wp_y = 0.0, 0.0
        if d.current_waypoint:
            dx = d.current_waypoint[0] - d.position[0]
            dy = d.current_waypoint[1] - d.position[1]
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 0.4:
                d.waypoint_reached = True
                d.coverage_contribution += 1.0
            elif dist > 0.01:
                # Navigate at full max_speed toward waypoint
                wp_x = (dx / dist) * self.max_speed
                wp_y = (dy / dist) * self.max_speed

        # --- 5. Social forces (repulsion when close, mild cohesion when far) ---
        sf_x, sf_y = self._compute_social_forces(d)
        d.social_force = [sf_x, sf_y]

        # --- 6. Exploration incentive (steer toward uncovered areas) ---
        ex_x, ex_y = self._compute_exploration_incentive(d)
        d.explore_force = [ex_x, ex_y]

        # --- 7. Obstacle avoidance (lidar-based reactive) ---
        obs_x, obs_y = d.obstacle_forces
        if d.nearest_obstacle_dist < 1.0:
            obs_scale = 3.0
        elif d.nearest_obstacle_dist < 2.0:
            obs_scale = 1.5
        else:
            obs_scale = 0.0
        obs_x *= obs_scale
        obs_y *= obs_scale

        # --- 8. Soft return-to-center (NO walls — open space exploration) ---
        # Gently steer back if beyond exploration_radius (configurable, default 20m)
        center_x, center_y = 0.0, 0.0
        px, py = d.position[0], d.position[1]
        dist_from_center = math.sqrt(px * px + py * py)
        soft_x, soft_y = 0.0, 0.0
        if dist_from_center > self.exploration_radius:
            # Soft pull back — strength grows linearly beyond radius
            overshoot = (dist_from_center - self.exploration_radius) / self.exploration_radius
            pull_strength = min(overshoot * self.max_speed * 1.5, self.max_speed)
            soft_x = -(px / dist_from_center) * pull_strength
            soft_y = -(py / dist_from_center) * pull_strength

        # --- 9. Additive blending (waypoint dominant, social forces for collision avoidance) ---
        total_x = wp_x + 0.30 * sf_x + 0.20 * ex_x + obs_x + soft_x
        total_y = wp_y + 0.30 * sf_y + 0.20 * ex_y + obs_y + soft_y

        # Small noise to break symmetry (especially for drones starting close together)
        noise_amp = 0.02
        total_x += random.uniform(-noise_amp, noise_amp)
        total_y += random.uniform(-noise_amp, noise_amp)

        # Apply ramp factor for smooth start
        total_x *= ramp
        total_y *= ramp

        # Clamp to max speed
        speed = math.sqrt(total_x * total_x + total_y * total_y)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            total_x *= scale
            total_y *= scale

        # Scale down lateral forces when altitude is marginal (reduces tilt, preserves lift)
        if d.position[2] < 0.35:
            lateral_scale = 0.3  # Heavy reduction when altitude is low
            total_x *= lateral_scale
            total_y *= lateral_scale
        elif d.position[2] < d.target_altitude - 0.05:
            lateral_scale = 0.6  # Moderate reduction when below target
            total_x *= lateral_scale
            total_y *= lateral_scale

        cmd.linear.x = total_x
        cmd.linear.y = total_y

        # Update intelligence score
        social_factor = min(d.social_interactions / 20.0, 1.0)
        coverage_factor = min(d.coverage_contribution / 10.0, 1.0)
        d.intelligence_score = (social_factor + coverage_factor + 0.8) / 3.0

        return cmd

    def _phase_return(self, d: DroneState) -> Twist:
        """Navigate back to spawn, then land."""
        cmd = Twist()
        cmd.linear.z = d.alt_ctrl.compute(d.position[2])

        target = d.current_waypoint or self.SPAWN_POSITIONS[d.index]
        dx = target[0] - d.position[0]
        dy = target[1] - d.position[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.3:
            d.phase = "land"
            d.phase_start = time.time()
            self.get_logger().info(f"[{d.drone_id}] At home → LAND")
        else:
            cmd.linear.x = (dx / dist) * self.max_speed * 0.7
            cmd.linear.y = (dy / dist) * self.max_speed * 0.7

        return cmd

    def _phase_land(self, d: DroneState) -> Twist:
        """Controlled descent."""
        cmd = Twist()
        if d.position[2] > 0.08:
            cmd.linear.z = -0.2
        else:
            cmd.linear.z = 0.0
            d.phase = "idle"
            d.is_armed = False
            self.get_logger().info(f"[{d.drone_id}] LANDED")

            # Check if all landed
            all_landed = all(
                dr.phase == "idle" for dr in self.drones.values()
            )
            if all_landed:
                self.mission_active = False
                self.get_logger().info("=== ALL DRONES LANDED — mission complete ===")

        return cmd

    # =========================================================================
    # Swarm Intelligence
    # =========================================================================

    def _compute_social_forces(self, d: DroneState) -> Tuple[float, float]:
        """
        Social forces: repulsion when too close, mild cohesion when far.
        Based on Boids model from swarm_behavior.py.
        """
        sx, sy = 0.0, 0.0

        for other_id, other in self.drones.items():
            if other_id == d.drone_id:
                continue
            if other.phase in ("idle", "emergency"):
                continue

            dx = other.position[0] - d.position[0]
            dy = other.position[1] - d.position[1]
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 0.05:
                continue

            # STRONG repulsion — quadratic falloff within safe_distance * 2
            # This is the primary drone-drone collision avoidance mechanism
            collision_zone = self.safe_distance * 2.0  # 1.6m
            if dist < collision_zone:
                # Stronger repulsion the closer they are (quadratic)
                normalized = (collision_zone - dist) / collision_zone
                repulsion = 4.0 * normalized * normalized / max(dist, 0.1)
                sx -= repulsion * dx
                sy -= repulsion * dy
                d.social_interactions += 1

            # Mild cohesion only when very far apart (>6m) — don't cluster
            elif dist > 6.0 and dist < 12.0:
                attraction = 0.02 / dist
                sx += attraction * dx
                sy += attraction * dy

        # Clamp
        mag = math.sqrt(sx * sx + sy * sy)
        if mag > self.max_speed:
            sx = sx / mag * self.max_speed
            sy = sy / mag * self.max_speed

        return sx, sy

    def _compute_exploration_incentive(self, d: DroneState) -> Tuple[float, float]:
        """
        Steer drone toward least-covered frontier cells.
        Based on coverage grid sampling from swarm_behavior.py.
        """
        my_x, my_y = d.position[0], d.position[1]
        best_dir = None
        min_coverage = float('inf')

        # Sample 12 directions at varying distances to find uncovered areas
        EXPLORE_LIMIT = self.exploration_radius  # No walls — soft boundary
        for angle_deg in range(0, 360, 30):
            rad = math.radians(angle_deg + d.index * 15)  # Per-drone offset
            # Sample at multiple distances for wide exploration
            for sample_dist in [2.0, 5.0, 10.0]:
                test_x = my_x + sample_dist * math.cos(rad)
                test_y = my_y + sample_dist * math.sin(rad)

                # Skip if too far from center (soft limit)
                if math.sqrt(test_x**2 + test_y**2) > EXPLORE_LIMIT:
                    continue

                cx = int(test_x / self.cell_size)
                cy = int(test_y / self.cell_size)

                # Count how many drones have visited this cell
                coverage = 0
                for other in self.drones.values():
                    if (cx, cy) in other.cells_visited:
                        coverage += 1
                # Also weight by recency in global grid
                if (cx, cy) in self.coverage_grid:
                    age = time.time() - self.coverage_grid[(cx, cy)]
                    if age < 30.0:
                        coverage += 2

                if coverage < min_coverage:
                    min_coverage = coverage
                    best_dir = (math.cos(rad), math.sin(rad))

        if best_dir and min_coverage < 3:
            bonus = 0.3 * (3 - min_coverage) * self.max_speed
            return best_dir[0] * bonus, best_dir[1] * bonus

        return 0.0, 0.0

    def _pick_exploration_waypoint(self, d: DroneState) -> List[float]:
        """
        Pick the next exploration waypoint using stigmergie-inspired coverage.
        No sector restrictions — any drone can go anywhere uncovered.
        Open space — no walls. Soft boundary at exploration_radius.
        """
        EXPLORE_LIMIT = self.exploration_radius  # Soft boundary (default 20m)

        best_wp = None
        best_score = -999.0

        # Grid candidates at 4.0m steps for wide coverage
        step = 4.0
        for gx_i in range(int(-EXPLORE_LIMIT / step), int(EXPLORE_LIMIT / step) + 1):
            for gy_i in range(int(-EXPLORE_LIMIT / step), int(EXPLORE_LIMIT / step) + 1):
                wx = gx_i * step
                wy = gy_i * step

                cx = int(wx / self.cell_size)
                cy = int(wy / self.cell_size)

                # Skip if beyond exploration radius (circular boundary)
                if math.sqrt(wx**2 + wy**2) > EXPLORE_LIMIT:
                    continue

                # Count how many drones have visited this cell
                visited_count = sum(
                    1 for other in self.drones.values()
                    if (cx, cy) in other.cells_visited
                )

                # Distance from current position
                dist_from_self = math.sqrt(
                    (wx - d.position[0])**2 + (wy - d.position[1])**2
                )

                # Skip cells too close (already covered by being here)
                if dist_from_self < 1.0:
                    continue

                # Distance from nearest other drone's current waypoint
                min_drone_wp_dist = float('inf')
                for other in self.drones.values():
                    if other.drone_id != d.drone_id and other.current_waypoint:
                        ddist = math.sqrt(
                            (wx - other.current_waypoint[0])**2 +
                            (wy - other.current_waypoint[1])**2
                        )
                        min_drone_wp_dist = min(min_drone_wp_dist, ddist)

                # Distance from nearest other drone's current POSITION
                min_drone_pos_dist = float('inf')
                for other in self.drones.values():
                    if other.drone_id != d.drone_id and other.phase in ("cruise", "explore"):
                        ddist = math.sqrt(
                            (wx - other.position[0])**2 +
                            (wy - other.position[1])**2
                        )
                        min_drone_pos_dist = min(min_drone_pos_dist, ddist)

                # Score: strongly prefer unvisited, spread from other drones
                score = (
                    (3.0 - visited_count) * 5.0            # Huge bonus for unvisited
                    + min(min_drone_wp_dist, 10.0) * 1.0   # Spread from other waypoints
                    + min(min_drone_pos_dist, 10.0) * 0.5  # Spread from other drones
                    - dist_from_self * 0.1                  # Slight preference for closer (reduced)
                    + random.uniform(0, 2.0)                # Randomness (more for bigger area)
                )

                if score > best_score:
                    best_score = score
                    best_wp = [wx, wy]

        if best_wp:
            return best_wp

        # Fallback: random position within exploration radius (circular)
        angle = random.uniform(0, 2 * math.pi)
        radius = random.uniform(2.0, EXPLORE_LIMIT * 0.8)
        return [radius * math.cos(angle), radius * math.sin(angle)]

    # =========================================================================
    # Propeller RPM Estimation
    # =========================================================================

    def _estimate_rpms(self, d: DroneState, cmd: Twist):
        """
        Estimate propeller RPMs from commanded thrust.
        Crazyflie: 4 motors, ~14200 RPM hover.
        Based on authentic-crazyflie.js RPM-to-force model.
        """
        if d.phase in ("idle", "emergency"):
            d.motor_rpms = [0.0, 0.0, 0.0, 0.0]
            return

        # Base hover RPM
        base_rpm = 14200.0

        # Altitude component
        alt_factor = 1.0 + cmd.linear.z * 0.15  # ±15% for altitude corrections

        # Horizontal movement adds differential
        roll_factor = cmd.linear.y * 0.03   # Lateral → roll
        pitch_factor = cmd.linear.x * 0.03  # Forward → pitch

        # Motor layout: M1(FR), M2(BL), M3(FL), M4(BR)
        # Differential RPMs for attitude control
        d.motor_rpms = [
            base_rpm * alt_factor * (1.0 + pitch_factor + roll_factor),   # M1 FR
            base_rpm * alt_factor * (1.0 - pitch_factor - roll_factor),   # M2 BL
            base_rpm * alt_factor * (1.0 + pitch_factor - roll_factor),   # M3 FL
            base_rpm * alt_factor * (1.0 - pitch_factor + roll_factor),   # M4 BR
        ]

        # Clamp
        d.motor_rpms = [max(0.0, min(21000.0, rpm)) for rpm in d.motor_rpms]

    # =========================================================================
    # Publishing
    # =========================================================================

    def _publish_cmd_vel(self, drone_id: str, cmd: Twist):
        """Publish velocity command with angular rate capping (from control_services.py)."""
        # Cap angular rate in z axis (slam_collaboratif: max_ang_z_rate = 0.4)
        MAX_ANG_Z_RATE = 0.4
        if abs(cmd.angular.z) > MAX_ANG_Z_RATE:
            cmd.angular.z = MAX_ANG_Z_RATE * (1.0 if cmd.angular.z > 0 else -1.0)

        pub = self._cmd_pubs.get(drone_id)
        if pub:
            pub.publish(cmd)

    def _publish_metrics(self):
        """Publish swarm metrics at 1 Hz."""
        # Re-assert enable every second (Gazebo doesn't latch)
        # This must happen even before mission starts to prevent drones falling
        self._enable_motors(silent=True)

        if not self.mission_active:
            return

        # Global intelligence score
        active_drones = [
            d for d in self.drones.values()
            if d.phase not in ("idle", "emergency")
        ]
        if active_drones:
            self.global_intelligence_score = sum(
                d.intelligence_score for d in active_drones
            ) / len(active_drones)

        score_msg = Float32()
        score_msg.data = self.global_intelligence_score * 100.0  # 0–100 scale
        self._pub_intelligence.publish(score_msg)

        # Coverage area (unique cells visited)
        all_cells = set()
        for d in self.drones.values():
            all_cells.update(d.cells_visited)
        coverage_area = len(all_cells) * self.cell_size * self.cell_size

        cov_msg = Float32()
        cov_msg.data = coverage_area
        self._pub_coverage.publish(cov_msg)

        # Swarm status (JSON)
        status = {
            "mission_active": self.mission_active,
            "elapsed": time.time() - self.mission_start_time if self.mission_active else 0,
            "formation": "explore",
            "coverage_area_m2": coverage_area,
            "intelligence_score": self.global_intelligence_score * 100.0,
            "active_drones": len(active_drones),
            "total_drones": self.num_drones,
            "drones": {},
        }
        for name, d in self.drones.items():
            status["drones"][name] = {
                "phase": d.phase,
                "position": d.position,
                "altitude": d.position[2],
                "intelligence_score": d.intelligence_score,
                "cells_visited": len(d.cells_visited),
                "nearest_obstacle": d.nearest_obstacle_dist,
                "exploration_state": d.exploration_state,
            }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self._pub_swarm_status.publish(status_msg)

        # Propeller speeds (all drones in one message)
        for name, d in self.drones.items():
            prop_msg = String()
            prop_msg.data = json.dumps({
                "drone_id": name,
                "speeds": d.motor_rpms,
                "timestamp": time.time(),
            })
            self._pub_propeller.publish(prop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SwarmController shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
