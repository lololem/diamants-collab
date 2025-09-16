#!/usr/bin/env python3
"""
DIAMANTS API - WebSocket Service Unifié
=======================================
Service WebSocket central pour éliminer duplications

🎯 Objectif: Un seul point d'entrée WebSocket pour tout le système
🔄 Remplace: 2 bridges identiques + clients éparpillés
"""

import json
import time
import asyncio
from typing import Dict, List, Set, Optional
from pathlib import Path
import logging

# ROS2 Integration
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import String, Float32  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore

# WebSocket Server
import websockets  # type: ignore
from websockets.server import serve  # type: ignore

# FastAPI Integration (future)
# from fastapi import WebSocket as FastAPIWebSocket


class DiamantWebSocketService(Node):
    """
    Service WebSocket unifié pour DIAMANTS
    
    Centralise toute la communication temps réel:
    - Frontend ↔ Backend
    - ROS2 ↔ Web Interface  
    - Real-time drone positions
    - Swarm intelligence metrics
    - Command & control
    """
    
    def __init__(
        self, 
        ws_host: str = "localhost", 
        ws_port: int = 8765,
        log_level: str = "INFO"
    ):
        super().__init__('diamant_websocket_service')
        
        # Configuration
        self.ws_host = ws_host
        self.ws_port = ws_port
        self.setup_logging(log_level)
        
        # État système global
        self.system_state = {
            'drones': {},
            'swarm': {
                'intelligence_score': 0.0,
                'coverage_area': 0.0,
                'formation': 'scatter',
                'mission_status': 'idle'
            },
            'simulation': {
                'active': False,
                'time': 0.0,
                'environment': 'default'
            },
            'last_update': time.time()
        }
        
        # Connexions actives
        self.websocket_clients: Set = set()  # type: ignore
        self.client_subscriptions: Dict[str, Set[str]] = {}  # client_id -> topics
        
        # ROS2 Setup
        self._setup_ros2_interface()
        
        # WebSocket Server
        self.websocket_server = None
        
        self.get_logger().info(
            f"🌐 DIAMANTS WebSocket Service initialisé (ws://{ws_host}:{ws_port})"
        )
    
    def setup_logging(self, level: str):
        """Configuration logging détaillé"""
        self.logger = logging.getLogger('DiamantWebSocket')
        self.logger.setLevel(getattr(logging, level.upper()))
        
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s | %(name)s | %(levelname)s | %(message)s'
        )
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
    
    def _setup_ros2_interface(self):
        """
        Configuration interface ROS2
        Topics standardisés pour tout le système
        """
        
        # === SUBSCRIPTIONS (ROS2 → WebSocket) ===
        
        # Positions drones temps réel
        self.position_sub = self.create_subscription(
            String, '/diamants/drones/positions',
            self._on_drone_positions, 10
        )
        
        # Intelligence collective
        self.intelligence_sub = self.create_subscription(
            Float32, '/diamants/swarm/intelligence_score',
            self._on_intelligence_score, 10
        )
        
        # Couverture zone
        self.coverage_sub = self.create_subscription(
            Float32, '/diamants/swarm/coverage_area',
            self._on_coverage_area, 10
        )
        
        # Statut système global
        self.status_sub = self.create_subscription(
            String, '/diamants/system/status',
            self._on_system_status, 10
        )
        
        # === PUBLISHERS (WebSocket → ROS2) ===
        
        # Commandes drones individuels
        self.drone_cmd_pub = self.create_publisher(
            String, '/diamants/drones/commands', 10
        )
        
        # Commandes essaim
        self.swarm_cmd_pub = self.create_publisher(
            String, '/diamants/swarm/commands', 10
        )
        
        # Changements mission
        self.mission_pub = self.create_publisher(
            String, '/diamants/mission/changes', 10
        )
        
        self.get_logger().info("✅ Interface ROS2 configurée (topics standardisés)")
    
    # ============================================================================
    # ROS2 CALLBACKS (Backend → WebSocket)
    # ============================================================================
    
    def _on_drone_positions(self, msg: String):
        """Positions drones ROS2 → WebSocket broadcast"""
        try:
            positions_data = json.loads(msg.data)
            
            # Mettre à jour état système
            for drone_id, position in positions_data.items():
                if drone_id not in self.system_state['drones']:
                    self.system_state['drones'][drone_id] = {}
                
                self.system_state['drones'][drone_id].update({
                    'position': position,
                    'last_seen': time.time()
                })
            
            # Broadcast temps réel
            asyncio.create_task(self._broadcast_message({
                'type': 'drone_positions',
                'data': positions_data,
                'timestamp': time.time()
            }))
            
        except json.JSONDecodeError as e:
            self.logger.warning(f"Position JSON invalide: {e}")
    
    def _on_intelligence_score(self, msg: Float32):
        """Intelligence essaim ROS2 → WebSocket"""
        self.system_state['swarm']['intelligence_score'] = msg.data
        
        asyncio.create_task(self._broadcast_message({
            'type': 'swarm_intelligence',
            'data': {'score': msg.data},
            'timestamp': time.time()
        }))
    
    def _on_coverage_area(self, msg: Float32):
        """Couverture zone ROS2 → WebSocket"""
        self.system_state['swarm']['coverage_area'] = msg.data
        
        asyncio.create_task(self._broadcast_message({
            'type': 'swarm_coverage',
            'data': {'area': msg.data},
            'timestamp': time.time()
        }))
    
    def _on_system_status(self, msg: String):
        """Statut système ROS2 → WebSocket"""
        try:
            status_data = json.loads(msg.data)
            self.system_state.update(status_data)
            
            asyncio.create_task(self._broadcast_message({
                'type': 'system_status',
                'data': status_data,
                'timestamp': time.time()
            }))
            
        except json.JSONDecodeError as e:
            self.logger.warning(f"Status JSON invalide: {e}")
    
    # ============================================================================
    # WEBSOCKET SERVER MANAGEMENT
    # ============================================================================
    
    async def _broadcast_message(self, message: Dict):
        """
        Broadcast message à tous clients connectés
        Avec gestion automatique déconnexions
        """
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
                self.logger.debug(f"Erreur broadcast: {e}")
                disconnected.add(websocket)
        
        # Cleanup déconnexions
        self.websocket_clients -= disconnected
        if disconnected:
            self.logger.info(f"🔌 {len(disconnected)} clients déconnectés")
    
    async def _handle_client(self, websocket, path):
        """
        Gestion client WebSocket individuel
        Point d'entrée unique pour tous clients (Frontend, Tools, etc.)
        """
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self.websocket_clients.add(websocket)
        self.client_subscriptions[client_id] = set()
        
        self.logger.info(f"🔗 Client connecté: {client_id} (total: {len(self.websocket_clients)})")
        
        try:
            # Envoyer état initial
            await self._send_initial_state(websocket)
            
            # Boucle messages
            async for message in websocket:
                await self._handle_client_message(websocket, client_id, message)
                
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            self.logger.error(f"Erreur client {client_id}: {e}")
        finally:
            self.websocket_clients.discard(websocket)
            self.client_subscriptions.pop(client_id, None)
            self.logger.info(f"🔌 Client déconnecté: {client_id}")
    
    async def _send_initial_state(self, websocket):
        """Envoyer état système complet au nouveau client"""
        initial_message = {
            'type': 'initial_state',
            'data': self.system_state.copy(),
            'timestamp': time.time()
        }
        
        await websocket.send(json.dumps(initial_message))
        self.logger.debug("📤 État initial envoyé")
    
    async def _handle_client_message(self, websocket, client_id: str, message_str: str):
        """
        Router messages clients vers handlers appropriés
        """
        try:
            message = json.loads(message_str)
            msg_type = message.get('type')
            data = message.get('data', {})
            
            # Route selon type message
            if msg_type == 'ping':
                await websocket.send(json.dumps({'type': 'pong', 'timestamp': time.time()}))
                
            elif msg_type == 'subscribe':
                await self._handle_subscription(client_id, data)
                
            elif msg_type == 'drone_command':
                await self._handle_drone_command(data)
                
            elif msg_type == 'swarm_command':
                await self._handle_swarm_command(data)
                
            elif msg_type == 'mission_command':
                await self._handle_mission_command(data)
                
            elif msg_type == 'get_status':
                await websocket.send(json.dumps({
                    'type': 'current_status',
                    'data': self.system_state,
                    'timestamp': time.time()
                }))
                
            else:
                self.logger.warning(f"Type message inconnu: {msg_type}")
                
        except json.JSONDecodeError:
            self.logger.warning(f"Message JSON invalide de {client_id}")
        except Exception as e:
            self.logger.error(f"Erreur traitement message: {e}")
    
    # ============================================================================
    # COMMAND HANDLERS (WebSocket → ROS2)
    # ============================================================================
    
    async def _handle_subscription(self, client_id: str, data: Dict):
        """Gestion abonnements clients (future feature)"""
        topics = data.get('topics', [])
        self.client_subscriptions[client_id].update(topics)
        self.logger.debug(f"📡 {client_id} abonné à: {topics}")
    
    async def _handle_drone_command(self, data: Dict):
        """Commandes drone individuel WebSocket → ROS2"""
        try:
            cmd_msg = String()
            cmd_msg.data = json.dumps({
                'type': 'drone_command',
                'data': data,
                'timestamp': time.time()
            })
            
            self.drone_cmd_pub.publish(cmd_msg)
            self.logger.info(f"🚁 Commande drone: {data.get('action')} pour {data.get('drone_id')}")
            
        except Exception as e:
            self.logger.error(f"Erreur commande drone: {e}")
    
    async def _handle_swarm_command(self, data: Dict):
        """Commandes essaim WebSocket → ROS2"""
        try:
            cmd_msg = String()
            cmd_msg.data = json.dumps({
                'type': 'swarm_command',
                'data': data,
                'timestamp': time.time()
            })
            
            self.swarm_cmd_pub.publish(cmd_msg)
            self.logger.info(f"🔥 Commande essaim: {data.get('action')}")
            
        except Exception as e:
            self.logger.error(f"Erreur commande essaim: {e}")
    
    async def _handle_mission_command(self, data: Dict):
        """Commandes mission WebSocket → ROS2"""
        try:
            mission_msg = String()
            mission_msg.data = json.dumps({
                'type': 'mission_command',
                'data': data,
                'timestamp': time.time()
            })
            
            self.mission_pub.publish(mission_msg)
            self.logger.info(f"🎯 Commande mission: {data.get('action')}")
            
        except Exception as e:
            self.logger.error(f"Erreur commande mission: {e}")
    
    # ============================================================================
    # SERVER LIFECYCLE
    # ============================================================================
    
    async def start_websocket_server(self):
        """Démarrer serveur WebSocket"""
        self.logger.info(f"🚀 Démarrage serveur WebSocket sur ws://{self.ws_host}:{self.ws_port}")
        
        self.websocket_server = await serve(
            self._handle_client,
            self.ws_host,
            self.ws_port,
            ping_interval=20,
            ping_timeout=10,
            max_size=1048576  # 1MB max message size
        )
        
        self.logger.info("✅ Serveur WebSocket démarré et prêt")
    
    def run_service(self):
        """
        Point d'entrée principal du service
        Combine ROS2 node + WebSocket server
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # Démarrer serveur WebSocket
            loop.run_until_complete(self.start_websocket_server())
            
            # Maintenir serveur actif
            self.logger.info("🔄 Service WebSocket en cours d'exécution...")
            loop.run_forever()
            
        except KeyboardInterrupt:
            self.logger.info("🛑 Service arrêté par utilisateur")
        except Exception as e:
            self.logger.error(f"Erreur service: {e}")
        finally:
            loop.close()
            self.logger.info("🔚 Service WebSocket fermé")


def main():
    """Point d'entrée CLI"""
    rclpy.init()
    
    try:
        # Démarrer service unifié
        service = DiamantWebSocketService(
            ws_host="0.0.0.0",  # Accessible depuis l'extérieur
            ws_port=8765,
            log_level="INFO"
        )
        
        # Thread WebSocket séparé du thread ROS2
        import threading
        websocket_thread = threading.Thread(target=service.run_service, daemon=True)
        websocket_thread.start()
        
        # ROS2 spin principal
        print("🌟 DIAMANTS WebSocket Service démarré!")
        print("📡 WebSocket: ws://localhost:8765")
        print("🔄 ROS2 Topics: /diamants/*")
        print("⏹️  Ctrl+C pour arrêter")
        
        rclpy.spin(service)
        
    except KeyboardInterrupt:
        print("\n🛑 Service DIAMANTS arrêté")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
