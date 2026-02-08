#!/usr/bin/env python3
"""
DIAMANTS - Mission Coordinator Microservice
=============================================
Orchestrates the mission lifecycle and connects user commands
from the WebSocket bridge to the SwarmController.

Responsibilities:
- Listens to frontend commands via /diamants/mission/commands
- Publishes mission status to /diamants/mission/status
- Auto-starts mission on launch (configurable)
- Monitors drone health and triggers RTL if needed
- Publishes system status to /diamants/system/status
"""

import json
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class MissionCoordinator(Node):
    """
    Mission lifecycle manager.

    States: idle → starting → active → paused → returning → complete
    """

    def __init__(self):
        super().__init__("diamants_mission_coordinator")

        # Parameters
        self.declare_parameter("auto_start", True)
        self.declare_parameter("auto_start_delay_sec", 5.0)
        self.declare_parameter("mission_timeout_sec", 86400.0)  # 24h — no auto-return
        self.declare_parameter("num_drones", 8)

        self.auto_start = self.get_parameter("auto_start").value
        self.auto_start_delay = self.get_parameter("auto_start_delay_sec").value
        self.mission_timeout = self.get_parameter("mission_timeout_sec").value
        self.num_drones = self.get_parameter("num_drones").value

        # State
        self.mission_state = "idle"
        self.mission_start_time = 0.0
        self.swarm_data = {}
        self.slam_data = {}

        # --- Subscribers ---
        # Commands from frontend via WebSocket bridge
        self._sub_commands = self.create_subscription(
            String, "/diamants/mission/commands",
            self._on_command, 10
        )
        # Swarm status from SwarmController
        self._sub_swarm = self.create_subscription(
            String, "/diamants/swarm/status",
            self._on_swarm_status, 10
        )
        # SLAM coverage
        self._sub_slam_json = self.create_subscription(
            String, "/diamants/slam/map_json",
            self._on_slam_update, 10
        )
        # Drone commands from frontend
        self._sub_drone_cmd = self.create_subscription(
            String, "/diamants/drones/commands",
            self._on_drone_command, 10
        )

        # --- Publishers ---
        # Mission status → bridge → frontend
        self._pub_mission_status = self.create_publisher(
            String, "/diamants/mission/status", 10
        )
        # System status → bridge → frontend
        self._pub_system_status = self.create_publisher(
            String, "/diamants/system/status", 10
        )
        # Relay commands to swarm controller
        self._pub_mission_cmd = self.create_publisher(
            String, "/diamants/mission/commands", 10
        )

        # --- Timers ---
        self.create_timer(1.0, self._publish_status)
        self.create_timer(5.0, self._check_health)

        # Auto-start
        if self.auto_start:
            self.get_logger().info(
                f"Auto-start enabled — mission starts in {self.auto_start_delay}s"
            )
            self.create_timer(
                self.auto_start_delay,
                self._auto_start_mission,
            )

        self.get_logger().info("MissionCoordinator started")

    def _auto_start_mission(self):
        """Auto-start the exploration mission after delay."""
        if self.mission_state == "idle":
            self.get_logger().info("=== AUTO-STARTING EXPLORATION MISSION ===")
            self._start_mission()
        # Cancel the timer after first fire
        # (ROS2 timers repeat, so we just guard with the state check)

    def _on_command(self, msg: String):
        """Handle mission commands from the frontend."""
        try:
            cmd = json.loads(msg.data)
            data = cmd.get("data", cmd)
            action = data.get("action", "")

            self.get_logger().info(f"Mission command received: {action}")

            if action == "start":
                self._start_mission()
            elif action == "stop":
                self._stop_mission()
            elif action == "pause":
                self._pause_mission()
            elif action == "resume":
                self._resume_mission()
            elif action == "return_home":
                self._return_home()
            elif action == "emergency":
                self._emergency()
            else:
                self.get_logger().warn(f"Unknown mission command: {action}")

        except Exception as e:
            self.get_logger().error(f"Command parse error: {e}")

    def _on_drone_command(self, msg: String):
        """Handle individual drone commands (takeoff, land, etc.)."""
        try:
            cmd = json.loads(msg.data)
            data = cmd.get("data", cmd)
            action = data.get("action", "")
            drone_id = data.get("drone_id", "")

            self.get_logger().info(f"Drone command: {action} → {drone_id}")

            # For now, relay as mission commands
            # TODO: per-drone control

        except Exception as e:
            self.get_logger().error(f"Drone command error: {e}")

    def _on_swarm_status(self, msg: String):
        """Update swarm data from SwarmController."""
        try:
            self.swarm_data = json.loads(msg.data)
        except Exception:
            pass

    def _on_slam_update(self, msg: String):
        """Update SLAM data from SLAMFusion."""
        try:
            self.slam_data = json.loads(msg.data)
        except Exception:
            pass

    # =========================================================================
    # Mission Lifecycle
    # =========================================================================

    def _start_mission(self):
        if self.mission_state == "active":
            self.get_logger().info("Mission already active — ignoring start")
            return

        self.get_logger().info(f"Starting mission from state: {self.mission_state}")
        self.mission_state = "starting"
        self.mission_start_time = time.time()

        # Publish start command for SwarmController
        cmd = String()
        cmd.data = json.dumps({
            "type": "mission_command",
            "data": {"action": "start"},
            "timestamp": time.time(),
        })
        self._pub_mission_cmd.publish(cmd)

        self.mission_state = "active"
        self.get_logger().info("Mission state → ACTIVE")

    def _stop_mission(self):
        self.mission_state = "complete"
        cmd = String()
        cmd.data = json.dumps({
            "type": "mission_command",
            "data": {"action": "stop"},
            "timestamp": time.time(),
        })
        self._pub_mission_cmd.publish(cmd)
        self.get_logger().info("Mission state → COMPLETE (stop)")

    def _pause_mission(self):
        if self.mission_state == "active":
            self.mission_state = "paused"
            self.get_logger().info("Mission state → PAUSED")

    def _resume_mission(self):
        if self.mission_state == "paused":
            self.mission_state = "active"
            self.get_logger().info("Mission state → ACTIVE (resumed)")

    def _return_home(self):
        self.mission_state = "returning"
        cmd = String()
        cmd.data = json.dumps({
            "type": "mission_command",
            "data": {"action": "return_home"},
            "timestamp": time.time(),
        })
        self._pub_mission_cmd.publish(cmd)
        self.get_logger().info("Mission state → RETURNING")

    def _emergency(self):
        self.mission_state = "emergency"
        cmd = String()
        cmd.data = json.dumps({
            "type": "mission_command",
            "data": {"action": "emergency"},
            "timestamp": time.time(),
        })
        self._pub_mission_cmd.publish(cmd)
        self.get_logger().error("Mission state → EMERGENCY")

    # =========================================================================
    # Health Monitoring
    # =========================================================================

    def _check_health(self):
        """Periodic health check."""
        if self.mission_state != "active":
            return

        # Check mission timeout
        elapsed = time.time() - self.mission_start_time
        if elapsed > self.mission_timeout:
            self.get_logger().warn(
                f"Mission timeout ({self.mission_timeout}s) — returning home"
            )
            self._return_home()

    # =========================================================================
    # Status Publishing
    # =========================================================================

    def _publish_status(self):
        """Publish mission + system status at 1 Hz."""
        now = time.time()
        elapsed = now - self.mission_start_time if self.mission_start_time > 0 else 0

        # Mission status
        mission_status = {
            "status": self.mission_state,
            "elapsed_sec": round(elapsed, 1),
            "num_drones": self.num_drones,
            "active_drones": self.swarm_data.get("active_drones", 0),
            "coverage_area_m2": self.swarm_data.get("coverage_area_m2", 0.0),
            "intelligence_score": self.swarm_data.get("intelligence_score", 0.0),
            "slam_coverage_pct": (
                self.slam_data.get("total_known_cells", 0) /
                max(1, self.slam_data.get("total_cells", 1)) * 100.0
            ) if self.slam_data else 0.0,
            "waypoints": [],
            "timestamp": now,
        }

        msg = String()
        msg.data = json.dumps(mission_status)
        self._pub_mission_status.publish(msg)

        # System status
        system_status = {
            "ros2_available": True,
            "simulation_active": self.mission_state in ("active", "starting", "returning"),
            "gazebo_running": True,
            "mission_state": self.mission_state,
            "uptime_sec": round(elapsed, 1),
            "services": {
                "swarm_controller": self.swarm_data.get("mission_active", False),
                "slam_fusion": bool(self.slam_data),
                "position_broadcaster": True,
                "mission_coordinator": True,
            },
            "timestamp": now,
        }

        sys_msg = String()
        sys_msg.data = json.dumps(system_status)
        self._pub_system_status.publish(sys_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MissionCoordinator shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
