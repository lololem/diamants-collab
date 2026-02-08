#!/usr/bin/env python3
"""
DIAMANTS - Position Broadcaster Microservice
==============================================
Aggregates per-drone odometry from Gazebo into the unified
/diamants/drones/positions topic consumed by the WebSocket bridge.

Also publishes /diamants/drones/telemetry with full drone state.

This replaces the old drone_position_coordinator.py with cleaner
architecture and all 8 drones.
"""

import json
import time
import math
from typing import Dict

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class PositionBroadcaster(Node):
    """
    Subscribes to /{cfN}/odom for each drone.
    Publishes aggregated JSON on /diamants/drones/positions at 10 Hz.
    Publishes per-drone telemetry on /diamants/drones/telemetry at 5 Hz.
    """

    DRONE_NAMES = ["crazyflie", "crazyflie1", "crazyflie2",
                   "crazyflie3", "crazyflie4", "crazyflie5",
                   "crazyflie6", "crazyflie7"]

    # Frontend drone IDs (mapped from Gazebo names)
    FRONTEND_IDS = {
        "crazyflie":  "crazyflie_01",
        "crazyflie1": "crazyflie_02",
        "crazyflie2": "crazyflie_03",
        "crazyflie3": "crazyflie_04",
        "crazyflie4": "crazyflie_05",
        "crazyflie5": "crazyflie_06",
        "crazyflie6": "crazyflie_07",
        "crazyflie7": "crazyflie_08",
    }

    def __init__(self):
        super().__init__("diamants_position_broadcaster")

        self.declare_parameter("position_rate_hz", 10.0)
        self.declare_parameter("telemetry_rate_hz", 5.0)
        self.declare_parameter("num_drones", 8)

        pos_rate = self.get_parameter("position_rate_hz").value
        tel_rate = self.get_parameter("telemetry_rate_hz").value
        num_drones = self.get_parameter("num_drones").value

        # Per-drone state
        self.drone_data: Dict[str, dict] = {}
        for name in self.DRONE_NAMES[:num_drones]:
            self.drone_data[name] = {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                "battery": 100.0,
                "status": "idle",
                "last_update": 0.0,
            }

        # --- Subscribers ---
        self._odom_subs = {}
        for name in self.drone_data:
            self._odom_subs[name] = self.create_subscription(
                Odometry, f"/{name}/odom",
                lambda msg, n=name: self._on_odom(n, msg), 10
            )

        # Subscribe to swarm status to get flight phase info
        self._sub_swarm_status = self.create_subscription(
            String, "/diamants/swarm/status",
            self._on_swarm_status, 10
        )

        # --- Publishers ---
        self._pub_positions = self.create_publisher(
            String, "/diamants/drones/positions", 10
        )
        self._pub_telemetry = self.create_publisher(
            String, "/diamants/drones/telemetry", 10
        )

        # --- Timers ---
        self.create_timer(1.0 / pos_rate, self._publish_positions)
        self.create_timer(1.0 / tel_rate, self._publish_telemetry)

        # Simulated battery drain
        self.create_timer(5.0, self._update_battery)

        self.get_logger().info(
            f"PositionBroadcaster started — {len(self.drone_data)} drones, "
            f"positions@{pos_rate}Hz, telemetry@{tel_rate}Hz"
        )

    def _on_odom(self, drone_id: str, msg: Odometry):
        """Update drone state from Gazebo odometry."""
        d = self.drone_data.get(drone_id)
        if not d:
            return

        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation

        d["position"] = {"x": p.x, "y": p.y, "z": p.z}
        d["velocity"] = {"x": v.x, "y": v.y, "z": v.z}

        # Quaternion → Euler
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        d["orientation"] = {
            "roll": math.degrees(roll),
            "pitch": math.degrees(pitch),
            "yaw": math.degrees(yaw),
        }
        d["last_update"] = time.time()

        # Mark as active if we see it moving above ground
        if p.z > 0.05 and d["status"] == "idle":
            d["status"] = "flying"

    def _on_swarm_status(self, msg: String):
        """Update drone statuses from swarm controller."""
        try:
            data = json.loads(msg.data)
            drones_info = data.get("drones", {})
            for gz_name, info in drones_info.items():
                if gz_name in self.drone_data:
                    self.drone_data[gz_name]["status"] = info.get("phase", "idle")
        except Exception:
            pass

    def _update_battery(self):
        """Simulate gradual battery drain for active drones."""
        for d in self.drone_data.values():
            if d["status"] not in ("idle",):
                d["battery"] = max(0.0, d["battery"] - 0.1)

    def _publish_positions(self):
        """Publish aggregated positions in format expected by WebSocket bridge.

        Coordinate convention:
          Gazebo ENU: x=forward, y=left, z=up
          Three.js:   x=right,   y=up,   z=forward   (Y-up)

        Mapping applied (NO scale — 1 Gazebo metre = 1 Three.js unit):
          frontend_x =  gazebo_x
          frontend_y =  gazebo_z   (altitude)
          frontend_z = -gazebo_y   (negate so +Z = forward in Three.js)
        """
        now = time.time()
        positions = {}

        for gz_name, d in self.drone_data.items():
            # Skip drones that never reported or went stale
            if d["last_update"] == 0.0:
                continue
            if now - d["last_update"] > 5.0:
                continue

            frontend_id = self.FRONTEND_IDS.get(gz_name, gz_name)

            # Sanitise altitude — clamp to ground if Gazebo reports below 0
            gz_z = max(0.0, d["position"]["z"])

            positions[frontend_id] = {
                "x": d["position"]["x"],
                "y": gz_z,                          # altitude (Gazebo Z → Three.js Y)
                "z": -d["position"]["y"],            # depth    (Gazebo Y → Three.js Z, negated)
                "vx": d["velocity"]["x"],
                "vy": d["velocity"]["z"],
                "vz": -d["velocity"]["y"],
                "yaw": d["orientation"]["yaw"],
                "battery": d["battery"],
                "status": d["status"],
            }

        if positions:
            msg = String()
            msg.data = json.dumps(positions)
            self._pub_positions.publish(msg)

    def _publish_telemetry(self):
        """Publish detailed telemetry for each active drone."""
        now = time.time()

        for gz_name, d in self.drone_data.items():
            if d["last_update"] == 0.0:
                continue
            if now - d["last_update"] > 5.0:
                continue

            frontend_id = self.FRONTEND_IDS.get(gz_name, gz_name)
            gz_z = max(0.0, d["position"]["z"])

            telemetry = {
                "drone_id": frontend_id,
                "position": {
                    "x": d["position"]["x"],
                    "y": gz_z,
                    "z": -d["position"]["y"],
                },
                "velocity": {
                    "x": d["velocity"]["x"],
                    "y": d["velocity"]["z"],
                    "z": -d["velocity"]["y"],
                },
                "orientation": d["orientation"],
                "battery": d["battery"],
                "status": d["status"],
                "timestamp": now,
            }

            msg = String()
            msg.data = json.dumps(telemetry)
            self._pub_telemetry.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PositionBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PositionBroadcaster shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
