#!/usr/bin/env python3
"""
DIAMANTS - Unified WebSocket Bridge
====================================
Single WebSocket bridge for the entire DIAMANTS platform.
Replaces 6 previous duplicate implementations.

Architecture:
    ROS2 Topics ←→ This Bridge ←→ WebSocket Clients (Frontend, Tools)

Port: 8765 (configurable via DIAMANTS_WEBSOCKET_PORT env var)
"""

import json
import time
import asyncio
import logging
import os
import threading
from typing import Dict, Set, Optional, Any

# --- Conditional ROS2 import (allows dev/testing without ROS2) ---
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32, Bool
    from geometry_msgs.msg import Twist
    ROS2_AVAILABLE = True
except ImportError:
    # Minimal stub for development without ROS2
    class Node:
        def __init__(self, name):
            self._name = name
            self._logger = logging.getLogger(name)
        def get_logger(self):
            return self._logger
        def create_subscription(self, msg_type, topic, callback, qos):
            return None
        def create_publisher(self, msg_type, topic, qos):
            return _StubPublisher()
    class _StubPublisher:
        def publish(self, msg):
            pass
    class String:
        def __init__(self):
            self.data = ""
    class Float32:
        def __init__(self):
            self.data = 0.0
    class Bool:
        def __init__(self):
            self.data = False
    class Twist:
        def __init__(self):
            self.linear = type('obj', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
            self.angular = type('obj', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()

import websockets
from websockets.server import serve

# --- Import topic registry ---
try:
    from config.topics import (
        PORTS, TOPICS_TELEMETRY, TOPICS_SWARM, TOPICS_MISSION,
        TOPICS_COMMANDS, TOPICS_SYSTEM, TOPICS_AI
    )
except ImportError:
    # Fallback if run standalone
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from config.topics import (
        PORTS, TOPICS_TELEMETRY, TOPICS_SWARM, TOPICS_MISSION,
        TOPICS_COMMANDS, TOPICS_SYSTEM, TOPICS_AI
    )


logger = logging.getLogger("diamants.websocket")


class DiamantsBridge(Node):
    """
    Unified WebSocket ↔ ROS2 bridge for DIAMANTS.
    
    Handles:
    - Real-time drone telemetry (ROS2 → WebSocket)
    - Swarm intelligence metrics (ROS2 → WebSocket)
    - Mission status updates (ROS2 → WebSocket)
    - Drone/swarm/mission commands (WebSocket → ROS2)
    - Client subscription filtering
    - Initial state delivery on connect
    - Automatic reconnection handling
    """

    def __init__(
        self,
        ws_host: str = "0.0.0.0",
        ws_port: int = None,
        log_level: str = "INFO",
    ):
        super().__init__("diamants_websocket_bridge")

        self.ws_host = ws_host
        self.ws_port = ws_port or int(os.environ.get("DIAMANTS_WEBSOCKET_PORT", PORTS["websocket"]))

        # Logging
        logging.basicConfig(
            level=getattr(logging, log_level.upper(), logging.INFO),
            format="%(asctime)s | %(name)s | %(levelname)s | %(message)s",
        )

        # Active WebSocket clients
        self.clients: Set = set()
        self.client_subscriptions: Dict[str, Set[str]] = {}

        # Telemetry cache (sent to new clients on connect)
        self.state: Dict[str, Any] = {
            "drones": {},
            "swarm": {
                "intelligence_score": 0.0,
                "coverage_area": 0.0,
                "formation": "scatter",
                "status": "idle",
            },
            "mission": {
                "status": "idle",
                "waypoints": [],
            },
            "propeller_speeds": {},
            "system": {
                "ros2_available": ROS2_AVAILABLE,
                "simulation_active": False,
            },
            "last_update": time.time(),
        }

        # ROS2 interface
        self._setup_ros2()

        # WebSocket server handle
        self._ws_server = None

        self.get_logger().info(
            f"DIAMANTS Bridge initialized — ws://{self.ws_host}:{self.ws_port}  "
            f"ROS2={'connected' if ROS2_AVAILABLE else 'stub mode'}"
        )

    # =========================================================================
    # ROS2 SETUP — uses centralized topic registry
    # =========================================================================

    def _setup_ros2(self):
        """Wire up all ROS2 subscriptions and publishers from the topic registry."""

        # --- Subscriptions (Backend → Bridge → Frontend) ---
        self._sub_drone_positions = self.create_subscription(
            String, TOPICS_TELEMETRY["drone_positions"],
            self._on_drone_positions, 10,
        )
        self._sub_drone_telemetry = self.create_subscription(
            String, TOPICS_TELEMETRY["drone_telemetry"],
            self._on_drone_telemetry, 10,
        )
        self._sub_propeller = self.create_subscription(
            String, TOPICS_TELEMETRY["propeller_speeds"],
            self._on_propeller_speeds, 10,
        )
        self._sub_intelligence = self.create_subscription(
            Float32, TOPICS_SWARM["intelligence_score"],
            self._on_intelligence_score, 10,
        )
        self._sub_coverage = self.create_subscription(
            Float32, TOPICS_SWARM["coverage_area"],
            self._on_coverage_area, 10,
        )
        self._sub_swarm_status = self.create_subscription(
            String, TOPICS_SWARM["swarm_status"],
            self._on_swarm_status, 10,
        )
        self._sub_mission_status = self.create_subscription(
            String, TOPICS_MISSION["mission_status"],
            self._on_mission_status, 10,
        )
        self._sub_system_status = self.create_subscription(
            String, TOPICS_SYSTEM["system_status"],
            self._on_system_status, 10,
        )

        # --- Publishers (Frontend → Bridge → Backend) ---
        self._pub_drone_cmd = self.create_publisher(
            String, TOPICS_COMMANDS["drone_commands"], 10,
        )
        self._pub_cmd_vel = self.create_publisher(
            Twist, TOPICS_COMMANDS["drone_cmd_vel"], 10,
        )
        self._pub_swarm_cmd = self.create_publisher(
            String, TOPICS_COMMANDS["swarm_commands"], 10,
        )
        self._pub_mission_cmd = self.create_publisher(
            String, TOPICS_COMMANDS["mission_commands"], 10,
        )
        self._pub_params = self.create_publisher(
            String, TOPICS_COMMANDS["parameter_changes"], 10,
        )

    # =========================================================================
    # ROS2 CALLBACKS → WebSocket broadcast
    # =========================================================================

    def _on_drone_positions(self, msg):
        try:
            data = json.loads(msg.data)
            for drone_id, pos in data.items():
                self.state["drones"].setdefault(drone_id, {})
                self.state["drones"][drone_id]["position"] = pos
                self.state["drones"][drone_id]["last_seen"] = time.time()
            self._schedule_broadcast({"type": "drone_positions", "data": data})
        except Exception as e:
            logger.debug(f"drone_positions parse error: {e}")

    def _on_drone_telemetry(self, msg):
        try:
            data = json.loads(msg.data)
            drone_id = data.get("drone_id", "cf1")
            self.state["drones"].setdefault(drone_id, {})
            self.state["drones"][drone_id].update(data)
            self._schedule_broadcast({"type": "drone_telemetry", "drone_id": drone_id, "data": data})
        except Exception as e:
            logger.debug(f"drone_telemetry parse error: {e}")

    def _on_propeller_speeds(self, msg):
        try:
            data = json.loads(msg.data)
            drone_id = data.get("drone_id", "cf1")
            self.state["propeller_speeds"][drone_id] = data
            self._schedule_broadcast({"type": "propeller_speeds", "drone_id": drone_id, "data": data})
        except Exception as e:
            logger.debug(f"propeller_speeds parse error: {e}")

    def _on_intelligence_score(self, msg):
        self.state["swarm"]["intelligence_score"] = msg.data
        self._schedule_broadcast({"type": "swarm_intelligence", "data": {"score": msg.data}})

    def _on_coverage_area(self, msg):
        self.state["swarm"]["coverage_area"] = msg.data
        self._schedule_broadcast({"type": "swarm_coverage", "data": {"area": msg.data}})

    def _on_swarm_status(self, msg):
        try:
            data = json.loads(msg.data)
            self.state["swarm"].update(data)
            self._schedule_broadcast({"type": "swarm_status", "data": data})
        except Exception as e:
            logger.debug(f"swarm_status parse error: {e}")

    def _on_mission_status(self, msg):
        try:
            data = json.loads(msg.data)
            self.state["mission"].update(data)
            self._schedule_broadcast({"type": "mission_status", "data": data})
        except Exception as e:
            logger.debug(f"mission_status parse error: {e}")

    def _on_system_status(self, msg):
        try:
            data = json.loads(msg.data)
            self.state["system"].update(data)
            self._schedule_broadcast({"type": "system_status", "data": data})
        except Exception as e:
            logger.debug(f"system_status parse error: {e}")

    # =========================================================================
    # WebSocket → ROS2 command handlers
    # =========================================================================

    async def _handle_cmd_vel(self, data: Dict):
        """Forward velocity command to ROS2."""
        twist = Twist()
        lin = data.get("linear", {})
        ang = data.get("angular", {})
        twist.linear.x = float(lin.get("x", 0.0))
        twist.linear.y = float(lin.get("y", 0.0))
        twist.linear.z = float(lin.get("z", 0.0))
        twist.angular.x = float(ang.get("x", 0.0))
        twist.angular.y = float(ang.get("y", 0.0))
        twist.angular.z = float(ang.get("z", 0.0))
        self._pub_cmd_vel.publish(twist)

    async def _handle_drone_command(self, data: Dict):
        msg = String()
        msg.data = json.dumps({"type": "drone_command", "data": data, "timestamp": time.time()})
        self._pub_drone_cmd.publish(msg)
        logger.info(f"Drone command: {data.get('action')} → {data.get('drone_id')}")

    async def _handle_swarm_command(self, data: Dict):
        msg = String()
        msg.data = json.dumps({"type": "swarm_command", "data": data, "timestamp": time.time()})
        self._pub_swarm_cmd.publish(msg)
        logger.info(f"Swarm command: {data.get('action')}")

    async def _handle_mission_command(self, data: Dict):
        msg = String()
        msg.data = json.dumps({"type": "mission_command", "data": data, "timestamp": time.time()})
        self._pub_mission_cmd.publish(msg)
        logger.info(f"Mission command: {data.get('action')}")

    async def _handle_parameter_change(self, data: Dict):
        msg = String()
        msg.data = json.dumps(data)
        self._pub_params.publish(msg)
        logger.info(f"Parameter changed: {data.get('name')}")

    # =========================================================================
    # WebSocket SERVER
    # =========================================================================

    def _schedule_broadcast(self, message: Dict):
        """Thread-safe broadcast scheduling from ROS2 callback thread."""
        message["timestamp"] = time.time()
        if hasattr(self, "_loop") and self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._broadcast(message), self._loop)

    async def _broadcast(self, message: Dict):
        """Send message to all connected clients."""
        if not self.clients:
            return
        payload = json.dumps(message)
        gone = set()
        for ws in self.clients.copy():
            try:
                await ws.send(payload)
            except Exception:
                gone.add(ws)
        self.clients -= gone

    async def _on_client_connect(self, websocket):
        """Handle a new WebSocket client."""
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self.clients.add(websocket)
        self.client_subscriptions[client_id] = set()
        logger.info(f"Client connected: {client_id} (total: {len(self.clients)})")

        try:
            # Send cached state
            await websocket.send(json.dumps({
                "type": "initial_state",
                "data": self.state,
                "timestamp": time.time(),
            }))

            async for raw in websocket:
                await self._route_message(websocket, client_id, raw)

        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            logger.error(f"Client error ({client_id}): {e}")
        finally:
            self.clients.discard(websocket)
            self.client_subscriptions.pop(client_id, None)
            logger.info(f"Client disconnected: {client_id} (remaining: {len(self.clients)})")

    async def _route_message(self, websocket, client_id: str, raw: str):
        """Route inbound WebSocket message to the correct handler."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON from {client_id}")
            return

        msg_type = msg.get("type")
        data = msg.get("data", {})

        handlers = {
            "ping":            lambda: websocket.send(json.dumps({"type": "pong", "timestamp": time.time()})),
            "subscribe":       lambda: self._handle_subscribe(client_id, data),
            "cmd_vel":         lambda: self._handle_cmd_vel(data),
            "drone_command":   lambda: self._handle_drone_command(data),
            "swarm_command":   lambda: self._handle_swarm_command(data),
            "mission_command": lambda: self._handle_mission_command(data),
            "set_parameter":   lambda: self._handle_parameter_change(data),
            "get_status":      lambda: websocket.send(json.dumps({
                "type": "current_status", "data": self.state, "timestamp": time.time(),
            })),
        }

        handler = handlers.get(msg_type)
        if handler:
            await handler()
        else:
            logger.warning(f"Unknown message type: {msg_type}")

    async def _handle_subscribe(self, client_id: str, data: Dict):
        topics = data.get("topics", [])
        self.client_subscriptions.setdefault(client_id, set()).update(topics)
        logger.debug(f"{client_id} subscribed to: {topics}")

    # =========================================================================
    # LIFECYCLE
    # =========================================================================

    async def start_server(self):
        """Start the WebSocket server (non-blocking, returns the server handle)."""
        self._loop = asyncio.get_event_loop()
        self._ws_server = await serve(
            self._on_client_connect,
            self.ws_host,
            self.ws_port,
            ping_interval=20,
            ping_timeout=10,
            max_size=1_048_576,
            close_timeout=5,
        )
        logger.info(f"WebSocket server listening on ws://{self.ws_host}:{self.ws_port}")

    def run(self):
        """
        Blocking entry point. Runs:
        1. WebSocket server in its own asyncio loop (daemon thread)
        2. ROS2 spin on the main thread
        """
        ws_thread = threading.Thread(target=self._run_ws_loop, daemon=True)
        ws_thread.start()

        if ROS2_AVAILABLE:
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                pass
        else:
            # Without ROS2, just keep alive
            try:
                ws_thread.join()
            except KeyboardInterrupt:
                pass

    def _run_ws_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        loop.run_until_complete(self.start_server())
        loop.run_forever()


# =============================================================================
# CLI entry point
# =============================================================================

def main():
    if ROS2_AVAILABLE:
        rclpy.init()

    bridge = DiamantsBridge()

    print("=" * 60)
    print("  DIAMANTS Unified WebSocket Bridge")
    print(f"  WebSocket : ws://0.0.0.0:{bridge.ws_port}")
    print(f"  ROS2      : {'connected' if ROS2_AVAILABLE else 'stub mode (dev)'}")
    print(f"  Topics    : /diamants/*")
    print("  Press Ctrl+C to stop")
    print("=" * 60)

    try:
        bridge.run()
    except KeyboardInterrupt:
        print("\nBridge stopped.")
    finally:
        if ROS2_AVAILABLE:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
