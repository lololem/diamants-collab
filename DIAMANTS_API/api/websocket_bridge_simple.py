#!/usr/bin/env python3
"""
DIAMANTS API - Simplified WebSocket Bridge 
Version simplifiée sans erreurs de type checking
"""

import asyncio
import json
import logging
import sys
import os
from typing import Dict, Any, Set

# Configuration du logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Variables globales pour compatibilité
ROS2_AVAILABLE = False
WS_AVAILABLE = False

# Tentative d'importation ROS2
try:
    sys.path.append('/opt/ros/jazzy/lib/python3.12/site-packages')
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from geometry_msgs.msg import Twist  # type: ignore
    from std_msgs.msg import String, Float32MultiArray  # type: ignore
    ROS2_AVAILABLE = True
    logger.info("ROS2 successfully imported")
except ImportError as e:
    logger.warning(f"ROS2 not available: {e}")
    ROS2_AVAILABLE = False

# Tentative d'importation WebSocket
try:
    import websockets  # type: ignore
    WS_AVAILABLE = True
    logger.info("WebSocket successfully imported")
except ImportError as e:
    logger.warning(f"WebSocket not available: {e}")
    WS_AVAILABLE = False


class DiamantsBridge:
    """Bridge simplifié DIAMANTS WebSocket-ROS2"""
    
    def __init__(self, host="localhost", port=8765):
        self.host = host
        self.port = port
        self.clients = set()
        self.is_running = False
        
        # Initialisation ROS2 si disponible
        if ROS2_AVAILABLE:
            try:
                rclpy.init()  # type: ignore
                self.node = Node('diamants_bridge')  # type: ignore
                
                # Publishers
                self.cmd_vel_pub = self.node.create_publisher(
                    Twist, '/crazyflie/cmd_vel', 10  # type: ignore
                )
                
                # Subscribers pour retour d'info
                self.node.create_subscription(
                    Float32MultiArray,  # type: ignore
                    '/crazyflie/propeller_speeds',
                    self.propeller_callback,
                    10
                )
                
                logger.info("ROS2 node initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize ROS2: {e}")
                self.node = None
        else:
            self.node = None
    
    def propeller_callback(self, msg):
        """Callback pour les données des hélices"""
        try:
            propeller_data = {
                'type': 'propeller_update',
                'data': {
                    'propeller_speeds_rad_s': list(msg.data),  # type: ignore
                    'timestamp': asyncio.get_event_loop().time()
                }
            }
            # Diffusion vers tous les clients WebSocket
            asyncio.create_task(self.broadcast_to_clients(propeller_data))
        except Exception as e:
            logger.error(f"Error in propeller callback: {e}")
    
    async def handle_client(self, websocket, path):
        """Gestionnaire pour chaque client WebSocket"""
        self.clients.add(websocket)
        client_addr = websocket.remote_address
        logger.info(f"Client connected from {client_addr}")
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.process_message(data, websocket)
                except json.JSONDecodeError:
                    logger.error(f"Invalid JSON from {client_addr}: {message}")
                except Exception as e:
                    logger.error(f"Error processing message: {e}")
        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            self.clients.discard(websocket)
            logger.info(f"Client {client_addr} disconnected")
    
    async def process_message(self, data: Dict[str, Any], websocket):
        """Traite les messages entrants"""
        msg_type = data.get('type')
        
        if msg_type == 'cmd_vel' and self.node and ROS2_AVAILABLE:
            await self.handle_cmd_vel(data.get('data', {}))
        elif msg_type == 'ping':
            await self.send_response(websocket, {'type': 'pong', 'data': {}})
        elif msg_type == 'status':
            await self.send_status(websocket)
        else:
            logger.warning(f"Unknown message type: {msg_type}")
    
    async def handle_cmd_vel(self, cmd_data: Dict[str, Any]):
        """Traite les commandes de vitesse"""
        try:
            if not self.node or not ROS2_AVAILABLE:
                logger.warning("ROS2 not available for cmd_vel")
                return
                
            twist_msg = Twist()  # type: ignore
            
            # Attribution des valeurs avec gestion d'erreur
            try:
                twist_msg.linear.x = float(cmd_data.get('linear_x', 0.0))  # type: ignore
                twist_msg.linear.y = float(cmd_data.get('linear_y', 0.0))  # type: ignore 
                twist_msg.linear.z = float(cmd_data.get('linear_z', 0.0))  # type: ignore
                twist_msg.angular.x = float(cmd_data.get('angular_x', 0.0))  # type: ignore
                twist_msg.angular.y = float(cmd_data.get('angular_y', 0.0))  # type: ignore
                twist_msg.angular.z = float(cmd_data.get('angular_z', 0.0))  # type: ignore
                
                self.cmd_vel_pub.publish(twist_msg)
                logger.info(f"Published cmd_vel: {cmd_data}")
            except Exception as e:
                logger.error(f"Error creating twist message: {e}")
                
        except Exception as e:
            logger.error(f"Error in handle_cmd_vel: {e}")
    
    async def send_response(self, websocket, response: Dict[str, Any]):
        """Envoie une réponse à un client spécifique"""
        try:
            await websocket.send(json.dumps(response))
        except Exception as e:
            logger.error(f"Error sending response: {e}")
    
    async def send_status(self, websocket):
        """Envoie le statut du bridge"""
        status = {
            'type': 'status_response',
            'data': {
                'ros2_available': ROS2_AVAILABLE,
                'websocket_available': WS_AVAILABLE,
                'connected_clients': len(self.clients),
                'node_active': self.node is not None
            }
        }
        await self.send_response(websocket, status)
    
    async def broadcast_to_clients(self, message: Dict[str, Any]):
        """Diffuse un message à tous les clients connectés"""
        if not self.clients:
            return
            
        message_str = json.dumps(message)
        disconnected = set()
        
        for client in self.clients:
            try:
                await client.send(message_str)
            except Exception as e:
                logger.error(f"Error broadcasting to client: {e}")
                disconnected.add(client)
        
        # Supprime les clients déconnectés
        self.clients -= disconnected
    
    async def ros2_spin_task(self):
        """Tâche pour faire tourner le nœud ROS2"""
        if not self.node or not ROS2_AVAILABLE:
            return
            
        while self.is_running:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)  # type: ignore
                await asyncio.sleep(0.01)  # Libère le contrôle pour asyncio
            except Exception as e:
                logger.error(f"Error in ROS2 spin: {e}")
                await asyncio.sleep(0.1)
    
    async def start_server(self):
        """Démarre le serveur WebSocket"""
        if not WS_AVAILABLE:
            logger.error("WebSocket not available, cannot start server")
            return
            
        self.is_running = True
        
        try:
            # Démarrage du serveur WebSocket
            start_server = websockets.serve(  # type: ignore
                self.handle_client,
                self.host,
                self.port
            )
            
            logger.info(f"Starting WebSocket server on {self.host}:{self.port}")
            
            # Démarrage de la tâche ROS2 en parallèle
            tasks = []
            tasks.append(start_server)
            if ROS2_AVAILABLE:
                tasks.append(asyncio.create_task(self.ros2_spin_task()))
            
            await asyncio.gather(*tasks)
            
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.is_running = False
    
    def cleanup(self):
        """Nettoyage des ressources"""
        self.is_running = False
        if self.node and ROS2_AVAILABLE:
            try:
                rclpy.shutdown()  # type: ignore
            except Exception as e:
                logger.error(f"Error during ROS2 shutdown: {e}")


async def main():
    """Fonction principale"""
    bridge = DiamantsBridge()
    
    try:
        await bridge.start_server()
    except KeyboardInterrupt:
        logger.info("Shutdown requested by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        bridge.cleanup()
        logger.info("Bridge shutdown complete")


if __name__ == "__main__":
    asyncio.run(main())
