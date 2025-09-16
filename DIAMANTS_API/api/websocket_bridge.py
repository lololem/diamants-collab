#!/usr/bin/env python3
"""
DIAMANTS API - Unified WebSocket Bridge
Centralized WebSocket service for DIAMANTS drone swarm communication
"""

import asyncio
import websockets
import json
import logging
import sys
import os
from typing import Dict, Any, Set, Union

# WebSocket imports with version compatibility
try:
    import websockets  # type: ignore
    from websockets.server import WebSocketServerProtocol  # type: ignore
    WS_AVAILABLE = True
except ImportError:
    print("Warning: websockets not available")
    # Mock pour compatibilité
    class WebSocketServerProtocol:
        pass
    WS_AVAILABLE = False
from datetime import datetime

# Add ROS2 workspace to path
sys.path.append('/opt/ros/jazzy/lib/python3.12/site-packages')
ros2_ws_path = '/home/loic/Projects/AI_PROJECTS/DIAMANTS/DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/install'
if os.path.exists(ros2_ws_path):
    sys.path.append(f"{ros2_ws_path}/lib/python3.12/site-packages")

# ROS2 imports conditionnels
try:
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from geometry_msgs.msg import Twist  # type: ignore
    from std_msgs.msg import String, Float32MultiArray  # type: ignore
    ROS2_AVAILABLE = True
    
    # Helper pour spin_once avec gestion d'erreur
    def rclpy_spin_once(node, timeout_sec=0.1):
        return rclpy.spin_once(node, timeout_sec=timeout_sec)
    
    def rclpy_shutdown():
        return rclpy.shutdown()
        
except ImportError as e:
    print(f"Warning: ROS2 not available: {e}")
    ROS2_AVAILABLE = False
    
    # Mock classes pour éviter les erreurs d'importation
    class MockVector3:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
    
    class Node:
        def __init__(self, *args, **kwargs): 
            self.node_name = args[0] if args else "mock_node"
        def create_publisher(self, *args, **kwargs): return None
        def create_subscription(self, *args, **kwargs): return None
        def get_logger(self): 
            return type('Logger', (), {'info': print, 'warn': print, 'error': print})()
    
    class Twist:
        def __init__(self):
            self.linear = MockVector3()
            self.angular = MockVector3()
    
    class String:
        def __init__(self):
            self.data = ""
    
    class Float32MultiArray:
        def __init__(self):
            self.data = []
    
    def rclpy_spin_once(node, timeout_sec=0.1):
        pass  # Mock implementation
    
    def rclpy_shutdown():
        pass  # Mock implementation
    # Mock classes for testing without ROS2
    class Node:
        def __init__(self, name): pass
        def create_publisher(self, *args, **kwargs): return MockPublisher()
        def create_subscription(self, *args, **kwargs): return MockSubscription()
        def get_logger(self): return MockLogger()
    
    class MockPublisher:
        def publish(self, msg): pass
    
    class MockSubscription:
        pass
    
    class MockLogger:
        def info(self, msg): print(f"INFO: {msg}")
        def warning(self, msg): print(f"WARNING: {msg}")
        def error(self, msg): print(f"ERROR: {msg}")
    
    class Twist:
        def __init__(self):
            self.linear = type('obj', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})
            self.angular = type('obj', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})
    
    class String:
        def __init__(self):
            self.data = ""
    
    class Float32MultiArray:
        def __init__(self):
            self.data = []
    
    def rclpy_init(*args, **kwargs): pass
    def rclpy_shutdown(): pass
    def rclpy_spin_once(node, timeout_sec=None): pass
    
    # Replace rclpy functions
    rclpy = type('obj', (object,), {
        'init': rclpy_init,
        'shutdown': rclpy_shutdown,
        'spin_once': rclpy_spin_once
    })

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class WebSocketBridge(Node):
    """
    Unified WebSocket Bridge for DIAMANTS
    Handles communication between Frontend, API, and ROS2 Backend
    """
    
    def __init__(self):
        super().__init__('diamants_websocket_bridge')
        
        # WebSocket connections tracking
        self.websocket_clients: Set[WebSocketServerProtocol] = set()
        self.client_info: Dict[WebSocketServerProtocol, Dict] = {}
        
        # ROS2 Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/crazyflie/cmd_vel', 10
        )
        self.mission_command_publisher = self.create_publisher(
            String, '/diamants/mission_command', 10
        )
        
        # ROS2 Subscribers
        self.propeller_subscriber = self.create_subscription(
            Float32MultiArray,
            '/crazyflie/propeller_speeds',
            self.propeller_callback,
            10
        )
        self.status_subscriber = self.create_subscription(
            String,
            '/diamants/status',
            self.status_callback,
            10
        )
        
        # Message queues for WebSocket broadcasting
        self.broadcast_queue = asyncio.Queue()
        
        # Statistics tracking
        self.stats = {
            'messages_received': 0,
            'messages_sent': 0,
            'clients_connected': 0,
            'ros2_messages': 0,
            'start_time': datetime.now()
        }
        
        self.get_logger().info("DIAMANTS WebSocket Bridge initialized")
    
    def propeller_callback(self, msg: Float32MultiArray):
        """Handle propeller speed updates from ROS2"""
        self.stats['ros2_messages'] += 1
        
        # Create WebSocket message for frontend
        ws_message = {
            'type': 'drone_propeller_speeds',
            'data': {
                'propeller_speeds_rad_s': list(msg.data),
                'timestamp': datetime.now().isoformat()
            }
        }
        
        # Queue for broadcasting
        asyncio.create_task(self._queue_broadcast(ws_message))
    
    def status_callback(self, msg: String):
        """Handle status updates from ROS2"""
        self.stats['ros2_messages'] += 1
        
        try:
            status_data = json.loads(msg.data)
        except json.JSONDecodeError:
            status_data = {'message': msg.data}
        
        ws_message = {
            'type': 'system_status',
            'data': status_data
        }
        
        asyncio.create_task(self._queue_broadcast(ws_message))
    
    async def _queue_broadcast(self, message: Dict):
        """Queue message for WebSocket broadcast"""
        await self.broadcast_queue.put(message)
    
    async def handle_websocket_message(self, websocket, message_text: str):
        """Process incoming WebSocket message"""
        try:
            message = json.loads(message_text)
            self.stats['messages_received'] += 1
            
            message_type = message.get('type')
            data = message.get('data', {})
            
            self.get_logger().info(f"Received: {message_type}")
            
            if message_type == 'drone_command':
                await self._handle_drone_command(data)
            elif message_type == 'mission_command':
                await self._handle_mission_command(data)
            elif message_type == 'client_info':
                await self._handle_client_info(websocket, data)
            elif message_type == 'ping':
                await self._handle_ping(websocket)
            else:
                self.get_logger().warning(f"Unknown message type: {message_type}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON received: {message_text}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")
    
    async def _handle_drone_command(self, data: Dict):
        """Process drone control commands"""
        action = data.get('action')
        
        if action == 'move':
            # Create ROS2 Twist message
            twist_msg = Twist()
            # Type ignore pour les attributs ROS2 qui ne sont pas toujours bien reconnus
            twist_msg.linear.x = float(data.get('linear_x', 0.0))  # type: ignore
            twist_msg.linear.y = float(data.get('linear_y', 0.0))  # type: ignore
            twist_msg.linear.z = float(data.get('linear_z', 0.0))  # type: ignore
            twist_msg.angular.x = float(data.get('angular_x', 0.0))  # type: ignore
            twist_msg.angular.y = float(data.get('angular_y', 0.0))  # type: ignore
            twist_msg.angular.z = float(data.get('angular_z', 0.0))  # type: ignore
            
            # Publish to ROS2
            self.cmd_vel_publisher.publish(twist_msg)
            self.get_logger().info(f"Published cmd_vel: {action}")
            
        elif action in ['takeoff', 'land', 'emergency_stop']:
            # Create mission command
            command_msg = String()
            command_msg.data = json.dumps({
                'action': action,
                'drone_id': data.get('drone_id', 'crazyflie'),
                'timestamp': datetime.now().isoformat()
            })
            
            self.mission_command_publisher.publish(command_msg)
            self.get_logger().info(f"Published mission command: {action}")
    
    async def _handle_mission_command(self, data: Dict):
        """Process mission-level commands"""
        command_msg = String()
        command_msg.data = json.dumps(data)
        
        self.mission_command_publisher.publish(command_msg)
        self.get_logger().info(f"Published mission: {data.get('mission_type')}")
    
    async def _handle_client_info(self, websocket, data: Dict):
        """Handle client registration"""
        self.client_info[websocket] = {
            'type': data.get('client_type', 'unknown'),
            'name': data.get('client_name', 'unnamed'),
            'connected_at': datetime.now().isoformat()
        }
        
        # Send welcome message
        welcome_message = {
            'type': 'welcome',
            'data': {
                'server': 'DIAMANTS WebSocket Bridge',
                'version': '1.0.0',
                'timestamp': datetime.now().isoformat()
            }
        }
        
        await websocket.send(json.dumps(welcome_message))
    
    async def _handle_ping(self, websocket):
        """Handle ping/pong for connection keepalive"""
        pong_message = {
            'type': 'pong',
            'data': {
                'timestamp': datetime.now().isoformat(),
                'server_time': datetime.now().timestamp()
            }
        }
        
        await websocket.send(json.dumps(pong_message))
    
    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections"""
        self.websocket_clients.add(websocket)
        self.stats['clients_connected'] += 1
        
        client_addr = websocket.remote_address
        self.get_logger().info(f"Client connected: {client_addr}")
        
        try:
            async for message in websocket:
                await self.handle_websocket_message(websocket, message)
                
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f"Client disconnected: {client_addr}")
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")
        finally:
            self.websocket_clients.discard(websocket)
            self.client_info.pop(websocket, None)
    
    async def broadcast_worker(self):
        """Worker task for broadcasting messages to all clients"""
        while True:
            try:
                message = await self.broadcast_queue.get()
                
                if self.websocket_clients:
                    # Broadcast to all connected clients
                    message_text = json.dumps(message)
                    
                    # Send to all clients concurrently
                    tasks = []
                    for client in self.websocket_clients.copy():
                        task = asyncio.create_task(self._send_to_client(client, message_text))
                        tasks.append(task)
                    
                    if tasks:
                        await asyncio.gather(*tasks, return_exceptions=True)
                        self.stats['messages_sent'] += len(tasks)
                
                self.broadcast_queue.task_done()
                
            except Exception as e:
                self.get_logger().error(f"Broadcast error: {e}")
    
    async def _send_to_client(self, client, message_text: str):
        """Send message to a specific client"""
        try:
            await client.send(message_text)
        except websockets.exceptions.ConnectionClosed:
            # Client disconnected, remove from set
            self.websocket_clients.discard(client)
        except Exception as e:
            self.get_logger().error(f"Error sending to client: {e}")
    
    async def ros2_spin_worker(self):
        """Worker task for ROS2 message processing"""
        while True:
            try:
                if ROS2_AVAILABLE:
                    rclpy_spin_once(self, timeout_sec=0.1)  # type: ignore
                await asyncio.sleep(0.01)  # 100Hz
            except Exception as e:
                self.get_logger().error(f"ROS2 spin error: {e}")
                await asyncio.sleep(1.0)
    
    def get_stats(self) -> Dict:
        """Get bridge statistics"""
        uptime = (datetime.now() - self.stats['start_time']).total_seconds()
        
        return {
            **self.stats,
            'uptime_seconds': uptime,
            'active_clients': len(self.websocket_clients),
            'ros2_available': ROS2_AVAILABLE
        }

class WebSocketBridgeServer:
    """WebSocket Bridge Server Manager"""
    
    def __init__(self, host: str = "localhost", port: int = 9001):
        self.host = host
        self.port = port
        self.bridge = None
        self.server = None
    
    async def start(self):
        """Start the WebSocket bridge server"""
        # ROS2 should already be initialized by launcher, skip init
        
        # Create bridge node
        self.bridge = WebSocketBridge()
        
        # Start WebSocket server
        self.server = await websockets.serve(  # type: ignore
            self.bridge.websocket_handler,
            self.host,
            self.port
        )
        
        print(f"🚀 DIAMANTS WebSocket Bridge started on ws://{self.host}:{self.port}")
        print(f"🔗 ROS2 Available: {ROS2_AVAILABLE}")
        
        # Start background workers
        tasks = [
            asyncio.create_task(self.bridge.broadcast_worker()),
            asyncio.create_task(self.bridge.ros2_spin_worker())
        ]
        
        try:
            # Run until interrupted
            await asyncio.gather(*tasks)
        except KeyboardInterrupt:
            print("\n🛑 Shutting down WebSocket Bridge...")
        finally:
            await self.stop()
    
    async def stop(self):
        """Stop the WebSocket bridge server"""
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        
        if ROS2_AVAILABLE:
            rclpy_shutdown()  # type: ignore
        
        print("✅ WebSocket Bridge stopped")

# CLI Interface
async def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='DIAMANTS WebSocket Bridge')
    parser.add_argument('--host', default='localhost', help='Host to bind to')
    parser.add_argument('--port', type=int, default=9001, help='Port to bind to')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    server = WebSocketBridgeServer(args.host, args.port)
    await server.start()

if __name__ == "__main__":
    asyncio.run(main())
