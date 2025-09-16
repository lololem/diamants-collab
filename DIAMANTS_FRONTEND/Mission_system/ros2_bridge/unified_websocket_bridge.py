#!/usr/bin/env python3
"""
DIAMANTS - Bridge WebSocket ROS2 Unifié
=======================================
Version consolidée et optimisée pour les drones Crazyflie
"""

import json
import time
import asyncio
import argparse
from typing import Dict, List, Set, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Vector3
import websockets


class DiamantsBridge(Node):
    """Bridge ROS2 ↔ WebSocket unifié pour DIAMANTS"""
    
    def __init__(self, ws_host: str = "localhost", ws_port: int = 8765):
        super().__init__('diamants_websocket_bridge')
        
        self.ws_host = ws_host
        self.ws_port = ws_port
        
        # Connexions WebSocket actives
        self.websocket_clients: Set = set()
        
        # Cache données temps réel
        self.telemetry_cache = {
            'crazyflie_telemetry': {},
            'swarm_status': {},
            'mission_data': {},
            'propeller_speeds': {},
            'last_update': time.time()
        }
        
        # Configuration topics ROS2
        self.setup_ros2_interface()
        
        # Serveur WebSocket
        self.websocket_server = None
        
        self.get_logger().info(f"🚁 DIAMANTS Bridge initialisé sur {ws_host}:{ws_port}")
    
    def setup_ros2_interface(self):
        """Configuration complète interface ROS2"""
        
        # === SUBSCRIPTIONS (ROS2 → WebSocket) ===
        
        # Télémétrie Crazyflie individuelle
        self.telemetry_sub = self.create_subscription(
            String, '/crazyflie/telemetry',
            self.on_crazyflie_telemetry, 10
        )
        
        # Position temps réel
        self.position_sub = self.create_subscription(
            Vector3, '/crazyflie/position',
            self.on_crazyflie_position, 10
        )
        
        # État essaim global
        self.swarm_status_sub = self.create_subscription(
            String, '/swarm/status',
            self.on_swarm_status, 10
        )
        
        # Données mission
        self.mission_sub = self.create_subscription(
            String, '/mission/status',
            self.on_mission_status, 10
        )
        
        # Vitesses hélices (critique pour animation)
        self.propeller_sub = self.create_subscription(
            String, '/crazyflie/propeller_speeds',
            self.on_propeller_speeds, 10
        )
        
        # === PUBLISHERS (WebSocket → ROS2) ===
        
        # Commandes mouvement Crazyflie
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/crazyflie/cmd_vel', 10
        )
        
        # Commandes essaim
        self.swarm_cmd_pub = self.create_publisher(
            String, '/swarm/commands', 10
        )
        
        # Commandes mission
        self.mission_cmd_pub = self.create_publisher(
            String, '/mission/commands', 10
        )
        
        # Paramètres dynamiques
        self.param_pub = self.create_publisher(
            String, '/diamants/parameters', 10
        )

    # === CALLBACKS ROS2 → WebSocket ===
    
    def on_crazyflie_telemetry(self, msg):
        """Télémétrie Crazyflie complète"""
        try:
            telemetry_data = json.loads(msg.data)
            telemetry_data['timestamp'] = time.time()
            
            # Mettre à jour cache
            drone_id = telemetry_data.get('drone_id', 'cf1')
            self.telemetry_cache['crazyflie_telemetry'][drone_id] = telemetry_data
            
            # Diffuser vers clients WebSocket
            asyncio.create_task(self.broadcast_to_clients({
                'type': 'crazyflie_telemetry',
                'drone_id': drone_id,
                'data': telemetry_data
            }))
            
        except Exception as e:
            self.get_logger().debug(f"Erreur télémétrie: {e}")
    
    def on_crazyflie_position(self, msg):
        """Position temps réel pour animation fluide"""
        try:
            position_data = {
                'position': {
                    'x': float(msg.x),
                    'y': float(msg.y), 
                    'z': float(msg.z)
                },
                'timestamp': time.time()
            }
            
            # Diffuser immédiatement (haute fréquence)
            asyncio.create_task(self.broadcast_to_clients({
                'type': 'drone_position',
                'data': position_data
            }))
            
        except Exception as e:
            self.get_logger().debug(f"Erreur position: {e}")
    
    def on_swarm_status(self, msg):
        """État global de l'essaim"""
        try:
            swarm_data = json.loads(msg.data)
            swarm_data['timestamp'] = time.time()
            
            self.telemetry_cache['swarm_status'] = swarm_data
            
            asyncio.create_task(self.broadcast_to_clients({
                'type': 'swarm_status',
                'data': swarm_data
            }))
            
        except Exception as e:
            self.get_logger().debug(f"Erreur swarm status: {e}")
    
    def on_mission_status(self, msg):
        """Données mission en cours"""
        try:
            mission_data = json.loads(msg.data)
            mission_data['timestamp'] = time.time()
            
            self.telemetry_cache['mission_data'] = mission_data
            
            asyncio.create_task(self.broadcast_to_clients({
                'type': 'mission_status',
                'data': mission_data
            }))
            
        except Exception as e:
            self.get_logger().debug(f"Erreur mission status: {e}")
    
    def on_propeller_speeds(self, msg):
        """CRITIQUE: Vitesses hélices pour animation"""
        try:
            propeller_data = json.loads(msg.data)
            propeller_data['timestamp'] = time.time()
            
            # Cache pour nouveaux clients
            drone_id = propeller_data.get('drone_id', 'cf1')
            self.telemetry_cache['propeller_speeds'][drone_id] = propeller_data
            
            # Diffusion immédiate (animation temps réel)
            asyncio.create_task(self.broadcast_to_clients({
                'type': 'propeller_speeds',
                'drone_id': drone_id,
                'data': propeller_data
            }))
            
        except Exception as e:
            self.get_logger().debug(f"Erreur propeller speeds: {e}")

    # === GESTION WEBSOCKET ===
    
    async def broadcast_to_clients(self, message: Dict):
        """Diffusion optimisée vers tous clients"""
        if not self.websocket_clients:
            return
        
        message_json = json.dumps(message)
        disconnected = set()
        
        for websocket in self.websocket_clients.copy():
            try:
                await websocket.send(message_json)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(websocket)
            except Exception:
                disconnected.add(websocket)
        
        # Nettoyage connexions fermées
        self.websocket_clients -= disconnected

    async def handle_websocket_client(self, websocket, path):
        """Gestion client WebSocket complet"""
        self.websocket_clients.add(websocket)
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self.get_logger().info(f"🔗 Nouveau client: {client_info} ({len(self.websocket_clients)} total)")
        
        try:
            # Envoyer état initial
            await self.send_initial_state(websocket)
            
            # Boucle messages client
            async for message in websocket:
                await self.process_websocket_message(websocket, message)
                
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            self.get_logger().debug(f"Erreur client WebSocket: {e}")
        finally:
            self.websocket_clients.discard(websocket)
            self.get_logger().info(f"🔌 Client déconnecté: {client_info} ({len(self.websocket_clients)} restants)")
    
    async def send_initial_state(self, websocket):
        """État initial pour nouveau client"""
        initial_state = {
            'type': 'initial_state',
            'data': {
                'telemetry': self.telemetry_cache['crazyflie_telemetry'],
                'swarm_status': self.telemetry_cache['swarm_status'],
                'mission_data': self.telemetry_cache['mission_data'],
                'propeller_speeds': self.telemetry_cache['propeller_speeds'],
                'server_time': time.time()
            }
        }
        
        await websocket.send(json.dumps(initial_state))
    
    async def process_websocket_message(self, websocket, message_str: str):
        """Traitement messages WebSocket → ROS2"""
        try:
            message = json.loads(message_str)
            msg_type = message.get('type')
            
            if msg_type == 'ping':
                await websocket.send(json.dumps({'type': 'pong', 'timestamp': time.time()}))
            
            elif msg_type == 'cmd_vel':
                await self.handle_cmd_vel(message.get('data', {}))
            
            elif msg_type == 'swarm_command':
                await self.handle_swarm_command(message.get('data', {}))
            
            elif msg_type == 'mission_command':
                await self.handle_mission_command(message.get('data', {}))
            
            elif msg_type == 'set_parameter':
                await self.handle_parameter_change(message.get('data', {}))
                
            elif msg_type == 'get_status':
                await websocket.send(json.dumps({
                    'type': 'current_status',
                    'data': self.telemetry_cache
                }))
            
        except json.JSONDecodeError:
            self.get_logger().warn(f"Message JSON invalide: {message_str[:100]}")
        except Exception as e:
            self.get_logger().error(f"Erreur traitement message: {e}")

    # === HANDLERS COMMANDES ===
    
    async def handle_cmd_vel(self, cmd_data: Dict):
        """CRITIQUE: Commandes mouvement → ROS2"""
        try:
            twist_msg = Twist()
            
            # Vitesses linéaires
            linear = cmd_data.get('linear', {})
            twist_msg.linear.x = float(linear.get('x', 0.0))
            twist_msg.linear.y = float(linear.get('y', 0.0))
            twist_msg.linear.z = float(linear.get('z', 0.0))
            
            # Vitesses angulaires
            angular = cmd_data.get('angular', {})
            twist_msg.angular.x = float(angular.get('x', 0.0))
            twist_msg.angular.y = float(angular.get('y', 0.0))
            twist_msg.angular.z = float(angular.get('z', 0.0))
            
            # Publier vers ROS2
            self.cmd_vel_pub.publish(twist_msg)
            
            self.get_logger().info(f"🚁 cmd_vel envoyé: lin({twist_msg.linear.x:.2f}, {twist_msg.linear.y:.2f}, {twist_msg.linear.z:.2f})")
            
        except Exception as e:
            self.get_logger().error(f"Erreur cmd_vel: {e}")
    
    async def handle_swarm_command(self, cmd_data: Dict):
        """Commandes essaim"""
        try:
            cmd_msg = String()
            cmd_msg.data = json.dumps(cmd_data)
            self.swarm_cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"📡 Commande essaim: {cmd_data.get('command')}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur swarm command: {e}")
    
    async def handle_mission_command(self, cmd_data: Dict):
        """Commandes mission"""
        try:
            cmd_msg = String()
            cmd_msg.data = json.dumps(cmd_data)
            self.mission_cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"🎯 Commande mission: {cmd_data.get('command')}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur mission command: {e}")
    
    async def handle_parameter_change(self, param_data: Dict):
        """Changement paramètres dynamiques"""
        try:
            param_msg = String()
            param_msg.data = json.dumps(param_data)
            self.param_pub.publish(param_msg)
            
            self.get_logger().info(f"⚙️ Paramètre modifié: {param_data.get('name')}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur parameter change: {e}")

    # === SERVEUR WEBSOCKET ===
    
    async def start_websocket_server(self):
        """Démarrer serveur WebSocket"""
        async def websocket_handler(websocket):
            """Wrapper pour compatibilité WebSocket moderne"""
            # Le path est maintenant accessible via websocket.request_uri ou défaut à '/'
            path = getattr(websocket, 'path', '/') or '/'
            await self.handle_websocket_client(websocket, path)
        
        self.websocket_server = await websockets.serve(
            websocket_handler,
            self.ws_host,
            self.ws_port,
            ping_interval=20,
            ping_timeout=10,
            close_timeout=5
        )
        
        self.get_logger().info(f"✅ Serveur WebSocket actif sur ws://{self.ws_host}:{self.ws_port}")
    
    def run_bridge(self):
        """Exécution bridge (bloquant)"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # Démarrer serveur WebSocket
            loop.run_until_complete(self.start_websocket_server())
            
            # Boucle infinie
            loop.run_forever()
            
        except KeyboardInterrupt:
            self.get_logger().info("🛑 Bridge arrêté par utilisateur")
        finally:
            if self.websocket_server:
                self.websocket_server.close()
            loop.close()


def main():
    """Point d'entrée avec gestion arguments"""
    parser = argparse.ArgumentParser(description='DIAMANTS WebSocket Bridge')
    parser.add_argument('--host', default='localhost', help='Host WebSocket')
    parser.add_argument('--port', type=int, default=8765, help='Port WebSocket')
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        bridge = DiamantsBridge(ws_host=args.host, ws_port=args.port)
        
        # Bridge dans thread séparé
        import threading
        bridge_thread = threading.Thread(target=bridge.run_bridge, daemon=True)
        bridge_thread.start()
        
        # Boucle ROS2 principale
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        print("\n🛑 DIAMANTS Bridge arrêté")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
