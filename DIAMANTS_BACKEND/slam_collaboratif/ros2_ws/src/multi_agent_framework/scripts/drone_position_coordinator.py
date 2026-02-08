#!/usr/bin/env python3
# DIAMANTS - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
# 
# Copyright (c) 2025 DIAMANTS Project Contributors
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
DIAMANTS — Drone Position Coordinator
=======================================
Aggregates per-drone odometry (/{drone_id}/odom) into a single
JSON topic (/diamants/drones/positions) for the WebSocket bridge.

Also publishes basic telemetry (/diamants/drones/telemetry) per drone.
"""

import json
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Default drone prefixes matching Gazebo simulation
DEFAULT_DRONE_PREFIXES = [
    "crazyflie", "crazyflie1", "crazyflie2", "crazyflie3",
    "crazyflie4", "crazyflie5", "crazyflie6", "crazyflie7",
]


class DronePositionCoordinator(Node):
    """
    Subscribes to /{drone_id}/odom for each drone,
    publishes aggregated positions on /diamants/drones/positions.
    """

    def __init__(self):
        super().__init__("drone_position_coordinator")

        # Parameters
        self.declare_parameter("drone_prefixes", DEFAULT_DRONE_PREFIXES)
        self.declare_parameter("publish_rate", 10.0)  # Hz

        prefixes = self.get_parameter("drone_prefixes").get_parameter_value().string_array_value
        if not prefixes:
            prefixes = DEFAULT_DRONE_PREFIXES
        rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.get_logger().info(f"Tracking {len(prefixes)} drones: {prefixes}")

        # State: latest position per drone
        self._positions = {}
        for drone_id in prefixes:
            self._positions[drone_id] = {
                "x": 0.0, "y": 0.0, "z": 0.0,
                "vx": 0.0, "vy": 0.0, "vz": 0.0,
                "last_update": 0.0,
            }

        # Subscribe to each drone's odom
        self._odom_subs = []
        for drone_id in prefixes:
            topic = f"/{drone_id}/odom"
            sub = self.create_subscription(
                Odometry, topic,
                lambda msg, did=drone_id: self._on_odom(did, msg),
                10,
            )
            self._odom_subs.append(sub)
            self.get_logger().info(f"  Subscribed to {topic}")

        # Publishers — standardized /diamants/ namespace
        self._pub_positions = self.create_publisher(
            String, "/diamants/drones/positions", 10
        )
        self._pub_telemetry = self.create_publisher(
            String, "/diamants/drones/telemetry", 10
        )

        # Timer for periodic aggregated publish
        period = 1.0 / rate
        self._timer = self.create_timer(period, self._publish_positions)

    def _on_odom(self, drone_id: str, msg: Odometry):
        """Update cached position for a drone."""
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self._positions[drone_id] = {
            "x": round(pos.x, 4),
            "y": round(pos.y, 4),
            "z": round(pos.z, 4),
            "vx": round(vel.x, 4),
            "vy": round(vel.y, 4),
            "vz": round(vel.z, 4),
            "last_update": time.time(),
        }

    def _publish_positions(self):
        """Publish aggregated drone positions as JSON."""
        now = time.time()
        # Build positions dict (only drones seen in last 5s)
        active = {}
        for drone_id, data in self._positions.items():
            if now - data["last_update"] < 5.0:
                active[drone_id] = {
                    "x": data["x"], "y": data["y"], "z": data["z"],
                }

        if active:
            msg = String()
            msg.data = json.dumps(active)
            self._pub_positions.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DronePositionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
