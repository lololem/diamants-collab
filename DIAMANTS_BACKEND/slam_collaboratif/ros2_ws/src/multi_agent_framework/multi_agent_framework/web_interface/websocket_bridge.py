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

#!/usr/bin/env python3
"""
ROS2 WebSocket Bridge
====================
Pont temps réel entre ROS2 et WebSocket pour interface web
"""

import json
import time
import asyncio
from typing import Dict, List, Set
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import websockets


class ROS2WebSocketBridge(Node):
    """Pont bidirectionnel ROS2 ↔ WebSocket"""
    
    def __init__(self, ws_host: str = "localhost", ws_port: int = 8765):
        super().__init__('ros2_websocket_bridge')
        
        self.ws_host = ws_host
        self.ws_port = ws_port
        
        # Connexions WebSocket
        self.websocket_clients: Set = set()
        
        # Cache données pour nouveaux clients
        self.data_cache = {
            'drone_positions': {},
            'intelligence_score': 0.0,
            'coverage_area': 0.0,
            'status_report': {}
        }
        
        # Setup ROS2
        self._setup_ros2_publishers()
        self._setup_ros2_subscriptions()
        
        # Serveur WebSocket
        self.websocket_server = None
        
        self.get_logger().info(f"🌉 ROS2-WebSocket Bridge initialisé")
    
    def _setup_ros2_subscriptions(self):
        """Configuration abonnements ROS2"""
        
        # Positions drones
        self.position_sub = self.create_subscription(
            String, '/multi_agent/drone_positions',
            self.on_drone_positions, 10
        )
        
        # Intelligence collective  
        self.intelligence_sub = self.create_subscription(
            Float32, '/swarm/intelligence_score',
            self.on_intelligence_score, 10
        )
        
        # Couverture zone
        self.coverage_sub = self.create_subscription(
            Float32, '/swarm/coverage_area', 
            self.on_coverage_area, 10
        )
        
        # Rapport statut global
        self.status_sub = self.create_subscription(
            String, '/swarm/status_report',
            self.on_status_report, 10
        )
    
    def _setup_ros2_publishers(self):
        """Configuration publishers ROS2"""
        
        # Commandes essaim
        self.swarm_cmd_pub = self.create_publisher(
            String, '/swarm/web_commands', 10
        )
        
        # Commandes drones individuels
        self.drone_cmd_pub = self.create_publisher(
            Twist, '/swarm/drone_commands', 10
        )
        
        # Changement paramètres
        self.param_pub = self.create_publisher(
            String, '/swarm/parameter_changes', 10
        )
    
    def on_drone_positions(self, msg):
        """Callback positions drones ROS2 → WebSocket"""
        try:
            # Parser et structurer données
            parts = msg.data.split(':')
            if len(parts) >= 4:
                drone_id = parts[0]
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                
                position_data = {
                    'drone_id': drone_id,
                    'position': {'x': x, 'y': y, 'z': z},
                    'timestamp': time.time()
                }
                
                # Mettre à jour cache
                self.data_cache['drone_positions'][drone_id] = position_data
                
                # Diffuser WebSocket
                asyncio.create_task(self.broadcast_to_websockets({
                    'type': 'drone_position',
                    'data': position_data
                }))
                
        except Exception as e:
            self.get_logger().debug(f"Erreur position callback: {e}")
    
    def on_intelligence_score(self, msg):
        """Callback intelligence ROS2 → WebSocket"""
        score_data = {
            'score': msg.data,
            'timestamp': time.time()
        }
        
        self.data_cache['intelligence_score'] = msg.data
        
        asyncio.create_task(self.broadcast_to_websockets({
            'type': 'intelligence_score',
            'data': score_data
        }))
    
    def on_coverage_area(self, msg):
        """Callback couverture ROS2 → WebSocket"""
        coverage_data = {
            'area': msg.data,
            'timestamp': time.time()
        }
        
        self.data_cache['coverage_area'] = msg.data
        
        asyncio.create_task(self.broadcast_to_websockets({
            'type': 'coverage_area', 
            'data': coverage_data
        }))
    
    def on_status_report(self, msg):
        """Callback rapport statut ROS2 → WebSocket"""
        try:
            status_data = json.loads(msg.data)
            status_data['timestamp'] = time.time()
            
            self.data_cache['status_report'] = status_data
            
            asyncio.create_task(self.broadcast_to_websockets({
                'type': 'status_report',
                'data': status_data
            }))
            
        except Exception as e:
            self.get_logger().debug(f"Erreur status callback: {e}")
    
    async def broadcast_to_websockets(self, message: Dict):
        """Diffusion message à tous clients WebSocket"""
        if not self.websocket_clients:
            return
        
        message_json = json.dumps(message)
        disconnected = set()
        
        for websocket in self.websocket_clients.copy():
            try:
                await websocket.send(message_json)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(websocket)
            except Exception as e:
                self.get_logger().debug(f"Erreur broadcast WebSocket: {e}")
                disconnected.add(websocket)
        
        # Nettoyer connexions fermées
        self.websocket_clients -= disconnected
    
    async def handle_websocket_client(self, websocket, path):
        """Gestion client WebSocket individuel"""
        self.websocket_clients.add(websocket)
        self.get_logger().info(f"Nouveau client WebSocket connecté: {len(self.websocket_clients)} total")
        
        try:
            # Envoyer état initial au nouveau client
            await self.send_initial_state(websocket)
            
            # Boucle réception messages client
            async for message in websocket:
                await self.handle_websocket_message(websocket, message)
                
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.websocket_clients.discard(websocket)
            self.get_logger().info(f"Client WebSocket déconnecté: {len(self.websocket_clients)} restants")
    
    async def send_initial_state(self, websocket):
        """Envoyer état initial à nouveau client"""
        initial_state = {
            'type': 'initial_state',
            'data': {
                'drone_positions': self.data_cache['drone_positions'],
                'intelligence_score': self.data_cache['intelligence_score'], 
                'coverage_area': self.data_cache['coverage_area'],
                'status_report': self.data_cache['status_report'],
                'timestamp': time.time()
            }
        }
        
        await websocket.send(json.dumps(initial_state))
    
    async def handle_websocket_message(self, websocket, message_str: str):
        """Traiter message reçu du client WebSocket"""
        try:
            message = json.loads(message_str)
            message_type = message.get('type')
            
            if message_type == 'ping':
                await websocket.send(json.dumps({'type': 'pong'}))
                
            elif message_type == 'swarm_command':
                await self.handle_swarm_command(message.get('data', {}))
                
            elif message_type == 'drone_command':
                await self.handle_drone_command(message.get('data', {}))
                
            elif message_type == 'parameter_change':
                await self.handle_parameter_change(message.get('data', {}))
                
            elif message_type == 'get_status':
                await websocket.send(json.dumps({
                    'type': 'current_status',
                    'data': self.data_cache
                }))
                
        except json.JSONDecodeError:
            self.get_logger().warn(f"Message WebSocket JSON invalide: {message_str[:100]}")
        except Exception as e:
            self.get_logger().error(f"Erreur traitement message WebSocket: {e}")
    
    async def handle_swarm_command(self, command_data: Dict):
        """Traiter commande essaim WebSocket → ROS2"""
        try:
            # Publier commande vers ROS2
            cmd_msg = String()
            cmd_msg.data = json.dumps(command_data)
            self.swarm_cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"Commande essaim envoyée: {command_data.get('command')}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur commande essaim: {e}")
    
    async def handle_drone_command(self, command_data: Dict):
        """Traiter commande drone individuel WebSocket → ROS2"""
        try:
            # Convertir en Twist si commande mouvement
            if command_data.get('type') == 'movement':
                twist_msg = Twist()
                movement = command_data.get('movement', {})
                
                twist_msg.linear.x = movement.get('x', 0.0)
                twist_msg.linear.y = movement.get('y', 0.0) 
                twist_msg.linear.z = movement.get('z', 0.0)
                
                self.drone_cmd_pub.publish(twist_msg)
                
            self.get_logger().info(f"Commande drone envoyée: {command_data}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur commande drone: {e}")
    
    async def handle_parameter_change(self, param_data: Dict):
        """Traiter changement paramètres WebSocket → ROS2"""
        try:
            param_msg = String()
            param_msg.data = json.dumps(param_data)
            self.param_pub.publish(param_msg)
            
            self.get_logger().info(f"Changement paramètre: {param_data}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur changement paramètre: {e}")
    
    async def start_websocket_server(self):
        """Démarrer serveur WebSocket"""
        self.get_logger().info(f"🚀 Démarrage serveur WebSocket sur ws://{self.ws_host}:{self.ws_port}")
        
        self.websocket_server = await websockets.serve(
            self.handle_websocket_client,
            self.ws_host,
            self.ws_port,
            ping_interval=20,
            ping_timeout=10
        )
        
        self.get_logger().info("✅ Serveur WebSocket démarré")
    
    def run_bridge(self):
        """Exécuter bridge (bloquant)"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # Démarrer serveur WebSocket
            loop.run_until_complete(self.start_websocket_server())
            
            # Boucle infinie pour maintenir serveur
            loop.run_forever()
            
        except KeyboardInterrupt:
            self.get_logger().info("🛑 Bridge arrêté par utilisateur")
        finally:
            loop.close()


def main():
    """Point d'entrée principal"""
    rclpy.init()
    
    try:
        bridge = ROS2WebSocketBridge()
        
        # Démarrer bridge dans thread séparé
        import threading
        bridge_thread = threading.Thread(target=bridge.run_bridge, daemon=True)
        bridge_thread.start()
        
        # Boucle ROS2 principale
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        print("\n🛑 ROS2 WebSocket Bridge arrêté")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()