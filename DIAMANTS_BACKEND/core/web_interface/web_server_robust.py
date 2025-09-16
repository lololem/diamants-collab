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
DIAMANTS Web Server - Version Robuste
========================================
Serveur web FastAPI avec intégration ROS2/Gazebo optionnelle
Fonctionne en mode simulation si ROS2 n'est pas disponible
"""

import os
import json
import time
import asyncio
import threading
from typing import Dict, List, Optional, Set
from pathlib import Path
import subprocess

# FastAPI et WebSocket
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

# ROS2 imports conditionnels
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32
    from geometry_msgs.msg import Point, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from tf2_msgs.msg import TFMessage
    ROS2_AVAILABLE = True
    print("✅ ROS2 disponible")
except ImportError:
    print("⚠️ ROS2 non disponible - Mode simulation activé")
    # Stub classes pour compatibilité
    class Node:
        def __init__(self, name): pass
        def get_logger(self): return type('Logger', (), {'info': print, 'warn': print, 'error': print})()


class DiamantWebServer(Node if ROS2_AVAILABLE else object):
    """Serveur web DIAMANTS avec intégration ROS2 optionnelle"""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        if ROS2_AVAILABLE:
            super().__init__('diamant_web_server')
        
        self.host = host
        self.port = port
        
        # Chemins
        self.base_dir = Path(__file__).parent
        self.static_dir = self.base_dir / "static"
        self.templates_dir = self.base_dir / "templates"
        
        # État des données
        self.data_cache = {
            'drones': {},
            'intelligence_score': 0.0,
            'coverage_area': 0.0,
            'gazebo_status': False,
            'rviz_status': False,
            'mission_status': 'idle',
            'map_data': None,
            'drone_positions': {}
        }
        
        # WebSocket clients
        self.websocket_clients: Set[WebSocket] = set()

        # Mode simulation pour démo
        self.simulation_mode = not ROS2_AVAILABLE
        # Event loop FastAPI (sera capturée au startup)
        self.loop: Optional[asyncio.AbstractEventLoop] = None

        # Setup FastAPI
        self.setup_fastapi()
        
        if ROS2_AVAILABLE:
            self.setup_ros2()
        else:
            self.setup_simulation()

        self.log("🚀 DIAMANTS Web Server initialisé")
    
    def log(self, message: str):
        """Logging unifié"""
        if ROS2_AVAILABLE and hasattr(self, 'get_logger'):
            self.get_logger().info(message)
        else:
            print(f"[DiamantWebServer] {message}")
    
    def setup_fastapi(self):
        """Configuration FastAPI"""
        self.app = FastAPI(
            title="🚁 DIAMANTS Dashboard",
            description="Interface web multi-agent SLAM avec Gazebo/RViz",
            version="1.0.0"
        )
        
        # CORS pour développement
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # Fichiers statiques
        if self.static_dir.exists():
            self.app.mount("/static", StaticFiles(directory=str(self.static_dir)), name="static")
            self.log(f"✅ Static files: {self.static_dir}")
        
        # Templates
        if self.templates_dir.exists():
            self.templates = Jinja2Templates(directory=str(self.templates_dir))
            self.log(f"✅ Templates: {self.templates_dir}")
        else:
            self.templates = None
        
        # Capturer l'event loop FastAPI au démarrage
        @self.app.on_event("startup")
        async def _capture_loop_on_start():
            try:
                self.loop = asyncio.get_running_loop()
                self.log("✅ Event loop FastAPI capturée")
            except Exception as e:
                self.log(f"⚠️ Impossible de capturer l'event loop: {e}")

        self.setup_routes()
    
    def setup_routes(self):
        """Configuration routes FastAPI"""
        
        @self.app.get("/", response_class=HTMLResponse)
        async def dashboard(request: Request):
            """Dashboard principal"""
            if self.templates:
                return self.templates.TemplateResponse("dashboard.html", {
                    "request": request,
                    "ros2_available": ROS2_AVAILABLE,
                    "simulation_mode": self.simulation_mode
                })
            else:
                return HTMLResponse(self.get_fallback_html())
        
        @self.app.get("/api/status")
        async def api_status():
            """Status général du système"""
            return {
                "status": "running",
                "ros2_available": ROS2_AVAILABLE,
                "simulation_mode": self.simulation_mode,
                "gazebo_running": self.check_gazebo_status(),
                "rviz_running": self.check_rviz_status(),
                "websocket_clients": len(self.websocket_clients),
                "data_cache": self.data_cache
            }
        
        @self.app.get("/api/swarm/status")
        async def swarm_status():
            """Status de l'essaim"""
            return {
                "active_drones": len(self.data_cache['drones']),
                "intelligence_score": self.data_cache['intelligence_score'],
                "coverage_area": self.data_cache['coverage_area'],
                "mission_status": self.data_cache['mission_status'],
                "drones": self.data_cache['drones'],
                "drone_positions": self.data_cache.get('drone_positions', {}),
                "timestamp": time.time()
            }
        
        @self.app.post("/api/mission/{action}")
        async def mission_control(action: str):
            """Contrôle mission"""
            if action in ["start", "stop", "pause"]:
                await self.handle_mission_command(action)
                return {"action": action, "status": "success"}
            return {"error": "Action invalide"}
        
        @self.app.post("/api/parameters")
        async def set_parameters(params: dict):
            """Mise à jour paramètres"""
            await self.handle_parameter_change(params)
            return {"status": "updated", "params": params}
        
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            """Endpoint WebSocket pour temps réel"""
            await self.handle_websocket(websocket)
        
        @self.app.get("/api/gazebo/status")
        async def gazebo_status():
            """Status Gazebo"""
            return {
                "running": self.check_gazebo_status(),
                "worlds": self.get_gazebo_worlds(),
                "models": self.get_gazebo_models()
            }
        
        @self.app.get("/api/rviz/status") 
        async def rviz_status():
            """Status RViz"""
            return {
                "running": self.check_rviz_status(),
                "topics": self.get_rviz_topics(),
                "map_available": self.data_cache['map_data'] is not None
            }
    
    def setup_ros2(self):
        """Configuration ROS2 si disponible"""
        try:
            # Publishers
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.mission_pub = self.create_publisher(String, '/mission_command', 10)
            
            # Subscribers
            self.tf_sub = self.create_subscription(
                TFMessage, '/tf', self.tf_callback, 10)
            self.map_sub = self.create_subscription(
                OccupancyGrid, '/map', self.map_callback, 10)
            self.status_sub = self.create_subscription(
                String, '/swarm_status', self.status_callback, 10)
            
            # Subscriber pour compter drones actifs (via odométrie)
            self.active_drones = set()
            # Drone de base
            self.create_subscription(
                Odometry, f'/crazyflie/odom', 
                lambda msg: self.drone_odom_callback(msg, 'crazyflie'), 10)
            # Drones numérotés crazyflie1 à crazyflie7
            for i in range(1, 8):
                drone_name = f'crazyflie{i}'
                # Closure pour capturer le nom correct
                callback = self._make_odom_callback(drone_name)
                self.create_subscription(Odometry, f'/{drone_name}/odom', callback, 10)
            
            # Timer pour publication périodique
            self.timer = self.create_timer(1.0, self.publish_status)
            
            self.log("✅ ROS2 publishers/subscribers configurés")
            
        except Exception as e:
            self.log(f"❌ Erreur setup ROS2: {e}")
    
    def setup_simulation(self):
        """Configuration mode simulation"""
        self.log("🎮 Mode simulation activé")
        
        # Données simulées
        self.simulate_drones()
        
        # Timer simulation
        threading.Timer(2.0, self.update_simulation).start()
    
    def simulate_drones(self):
        """Simuler données de drones"""
        import random
        
        # 3 drones simulés
        for i in range(1, 4):
            drone_id = f"drone_{i}"
            self.data_cache['drones'][drone_id] = {
                'id': drone_id,
                'position': {
                    'x': random.uniform(-5, 5),
                    'y': random.uniform(-5, 5),
                    'z': random.uniform(0.5, 3.0)
                },
                'battery': random.uniform(70, 100),
                'status': random.choice(['active', 'hover', 'moving']),
                'mission': random.choice(['exploration', 'mapping', 'patrol'])
            }
        
        # Métriques simulées
        self.data_cache['intelligence_score'] = random.uniform(0.7, 0.95)
        self.data_cache['coverage_area'] = random.uniform(0.5, 0.85)
        self.data_cache['mission_status'] = 'exploration'
    
    def update_simulation(self):
        """Mettre à jour simulation périodiquement"""
        if self.simulation_mode:
            import random
            
            # Mettre à jour positions
            for drone_id, drone in self.data_cache['drones'].items():
                # Mouvement aléatoire
                pos = drone['position']
                pos['x'] += random.uniform(-0.5, 0.5)
                pos['y'] += random.uniform(-0.5, 0.5)
                pos['z'] += random.uniform(-0.1, 0.1)
                
                # Contraintes
                pos['x'] = max(-10, min(10, pos['x']))
                pos['y'] = max(-10, min(10, pos['y']))
                pos['z'] = max(0.2, min(5, pos['z']))
                
                # Batterie
                drone['battery'] = max(0, drone['battery'] - random.uniform(0, 0.5))
            
            # Métriques
            self.data_cache['intelligence_score'] += random.uniform(-0.05, 0.05)
            self.data_cache['intelligence_score'] = max(0, min(1, self.data_cache['intelligence_score']))
            
            self.data_cache['coverage_area'] += random.uniform(-0.02, 0.03)
            self.data_cache['coverage_area'] = max(0, min(1, self.data_cache['coverage_area']))

            self._schedule_broadcast()  # Schedule broadcast update
            
            # Programmer prochaine mise à jour
            threading.Timer(2.0, self.update_simulation).start()
    
    async def broadcast_update(self):
        """Diffuser mise à jour à tous les clients WebSocket"""
        if not self.websocket_clients:
            return
        
        message = {
            "type": "data_update",
            "data": self.data_cache,
            "timestamp": time.time()
        }
        # Message positions dérivé pour compat front
        positions = self.data_cache.get('drone_positions', {}) or {}
        positions_payload = {
            did: {
                'drone_id': did,
                'position': {
                    'x': p.get('x', 0.0),
                    'y': p.get('y', 0.0),
                    'z': p.get('z', 0.0),
                }
            }
            for did, p in positions.items()
        }
        
        disconnected = set()
        for client in self.websocket_clients.copy():
            try:
                await client.send_text(json.dumps(message))
                # Envoyer aussi positions explicites
                await client.send_text(json.dumps({
                    'type': 'drone_positions',
                    'data': positions_payload,
                    'timestamp': time.time()
                }))
            except:
                disconnected.add(client)
        
        # Nettoyer clients déconnectés
        self.websocket_clients -= disconnected

    def _schedule_broadcast(self):
        """Planifier broadcast_update sur la bonne event loop (thread-safe)."""
        try:
            try:
                loop = asyncio.get_running_loop()
            except RuntimeError:
                loop = None
            if loop is not None:
                loop.create_task(self.broadcast_update())
            elif self.loop is not None:
                asyncio.run_coroutine_threadsafe(self.broadcast_update(), self.loop)
        except Exception as e:
            self.log(f"⚠️ _schedule_broadcast error: {e}")
    
    async def handle_websocket(self, websocket: WebSocket):
        """Gestionnaire WebSocket"""
        await websocket.accept()
        self.websocket_clients.add(websocket)
        
        self.log(f"🔗 Client WebSocket connecté ({len(self.websocket_clients)} total)")
        
        # Envoyer état initial
        await websocket.send_text(json.dumps({
            "type": "initial_data",
            "data": self.data_cache,
            "ros2_available": ROS2_AVAILABLE,
            "simulation_mode": self.simulation_mode
        }))
        # Envoyer immédiatement positions actuelles si présentes
        positions = self.data_cache.get('drone_positions', {}) or {}
        if positions:
            positions_payload = {
                did: {
                    'drone_id': did,
                    'position': {
                        'x': p.get('x', 0.0),
                        'y': p.get('y', 0.0),
                        'z': p.get('z', 0.0),
                    }
                }
                for did, p in positions.items()
            }
            await websocket.send_text(json.dumps({
                'type': 'drone_positions',
                'data': positions_payload,
                'timestamp': time.time()
            }))
        
        try:
            while True:
                data = await websocket.receive_text()
                message = json.loads(data)
                await self.handle_websocket_message(websocket, message)
                
        except WebSocketDisconnect:
            self.websocket_clients.discard(websocket)
            self.log(f"🔌 Client WebSocket déconnecté ({len(self.websocket_clients)} restants)")
    
    async def handle_websocket_message(self, websocket: WebSocket, message: dict):
        """Traiter message WebSocket"""
        msg_type = message.get('type')
        
        if msg_type == 'ping':
            await websocket.send_text(json.dumps({"type": "pong"}))
        
        elif msg_type == 'mission_command':
            action = message.get('action')
            await self.handle_mission_command(action)
        
        elif msg_type == 'parameter_change':
            params = message.get('params', {})
            await self.handle_parameter_change(params)
        
        elif msg_type == 'get_gazebo_status':
            status = {
                "type": "gazebo_status",
                "running": self.check_gazebo_status(),
                "worlds": self.get_gazebo_worlds()
            }
            await websocket.send_text(json.dumps(status))
    
    async def handle_mission_command(self, action: str):
        """Traiter commande mission"""
        self.log(f"🎯 Commande mission: {action}")
        
        if ROS2_AVAILABLE:
            msg = String()
            msg.data = action
            self.mission_pub.publish(msg)
        
        self.data_cache['mission_status'] = action
        await self.broadcast_update()
    
    async def handle_parameter_change(self, params: dict):
        """Traiter changement paramètres"""
        self.log(f"⚙️ Paramètres: {params}")
        # Appliquer changements selon params
    
    def check_gazebo_status(self) -> bool:
        """Vérifier si Gazebo est en cours d'exécution"""
        try:
            result = subprocess.run(
                ["ps", "aux"], 
                capture_output=True, 
                text=True
            )
            return "gz sim" in result.stdout or "gazebo" in result.stdout
        except:
            return False
    
    def check_rviz_status(self) -> bool:
        """Vérifier si RViz est en cours d'exécution"""
        try:
            result = subprocess.run(
                ["ps", "aux"], 
                capture_output=True, 
                text=True
            )
            return "rviz2" in result.stdout
        except:
            return False
    
    def get_gazebo_worlds(self) -> list:
        """Obtenir mondes Gazebo disponibles"""
        # Placeholder - à implémenter selon configuration
        return ["empty.sdf", "crazyflie_world.sdf"]
    
    def get_gazebo_models(self) -> list:
        """Obtenir modèles Gazebo actifs"""
        # Placeholder - requiert API Gazebo
        return ["ground_plane", "crazyflie"]
    
    def get_rviz_topics(self) -> list:
        """Obtenir topics RViz actifs"""
        if ROS2_AVAILABLE:
            try:
                result = subprocess.run(
                    ["ros2", "topic", "list"],
                    capture_output=True,
                    text=True
                )
                return result.stdout.strip().split('\n')
            except:
                pass
        return ["/map", "/tf", "/scan"]
    
    # Callbacks ROS2
    def tf_callback(self, msg):
        """Callback transformations"""
        # Extraire positions drones depuis TF
        for transform in msg.transforms:
            if "drone" in transform.child_frame_id:
                drone_id = transform.child_frame_id
                pos = transform.transform.translation
                
                if drone_id not in self.data_cache['drones']:
                    self.data_cache['drones'][drone_id] = {'id': drone_id}
                
                self.data_cache['drones'][drone_id]['position'] = {
                    'x': pos.x,
                    'y': pos.y, 
                    'z': pos.z
                }
        
        self._schedule_broadcast()  # Schedule broadcast update
    
    def map_callback(self, msg):
        """Callback carte SLAM"""
        self.data_cache['map_data'] = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y
            }
        }
        self._schedule_broadcast()
    
    def status_callback(self, msg):
        """Callback status essaim"""
        try:
            status = json.loads(msg.data)
            self.data_cache.update(status)
            self._schedule_broadcast()  # Schedule broadcast update
        except:
            pass
    
    def _make_odom_callback(self, drone_name: str):
        """Helper pour créer callbacks odométrie avec closure correcte"""
        def callback(msg):
            self.drone_odom_callback(msg, drone_name)
        return callback
    
    def drone_odom_callback(self, msg, drone_name):
        """Callback odométrie drone pour compter drones actifs"""
        self.active_drones.add(drone_name)
        # Mettre à jour cache avec nombre de drones
        self.data_cache['active_drones'] = len(self.active_drones)
        # Optionnel: Stocker position
        if 'drone_positions' not in self.data_cache:
            self.data_cache['drone_positions'] = {}
        self.data_cache['drone_positions'][drone_name] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        # Log léger pour diagnostic (une fois toutes les ~2s max)
        try:
            now = time.time()
            last = getattr(self, '_last_odom_log', 0)
            if now - last > 2.0:
                self._last_odom_log = now
                self.log(f"📡 Odom {drone_name}: x={msg.pose.pose.position.x:.2f} y={msg.pose.pose.position.y:.2f} z={msg.pose.pose.position.z:.2f} (actifs={len(self.active_drones)})")
        except Exception:
            pass
        self._schedule_broadcast()
    
    def publish_status(self):
        """Publier status périodique"""
        if ROS2_AVAILABLE:
            # Publier status actuel
            pass
    
    def get_fallback_html(self) -> str:
        """HTML de fallback si pas de templates"""
        return f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>DIAMANTS Dashboard</title>
            <style>
                body {{ font-family: Arial; background: #0d1b2a; color: #90e0ef; margin: 0; padding: 20px; }}
                .header {{ text-align: center; margin-bottom: 30px; }}
                .status {{ background: #1b263b; padding: 20px; border-radius: 10px; margin: 10px 0; }}
                .metric {{ display: inline-block; margin: 10px; padding: 15px; background: #415a77; border-radius: 8px; }}
            </style>
        </head>
        <body>
            <div class="header">
                <h1>🚁 DIAMANTS Dashboard</h1>
                <p>Interface Web Multi-Agent SLAM</p>
            </div>
            
            <div class="status">
                <h3>📊 Status Système</h3>
                <div class="metric">ROS2: {"✅ Disponible" if ROS2_AVAILABLE else "❌ Non disponible"}</div>
                <div class="metric">Mode: {"🤖 ROS2" if not self.simulation_mode else "🎮 Simulation"}</div>
                <div class="metric">Gazebo: {"✅ Actif" if self.check_gazebo_status() else "❌ Inactif"}</div>
                <div class="metric">RViz: {"✅ Actif" if self.check_rviz_status() else "❌ Inactif"}</div>
            </div>
            
            <div class="status">
                <h3>🚁 Essaim de Drones</h3>
                <div class="metric">Drones Actifs: {len(self.data_cache['drones'])}</div>
                <div class="metric">Intelligence: {self.data_cache['intelligence_score']:.2f}</div>
                <div class="metric">Couverture: {self.data_cache['coverage_area']:.2f}</div>
                <div class="metric">Mission: {self.data_cache['mission_status']}</div>
            </div>
            
            <div class="status">
                <h3>🌐 API Endpoints</h3>
                <p><a href="/api/status" style="color: #00b4d8;">/api/status</a> - Status général</p>
                <p><a href="/api/swarm/status" style="color: #00b4d8;">/api/swarm/status</a> - Status essaim</p>
                <p><a href="/api/gazebo/status" style="color: #00b4d8;">/api/gazebo/status</a> - Status Gazebo</p>
                <p><a href="/api/rviz/status" style="color: #00b4d8;">/api/rviz/status</a> - Status RViz</p>
            </div>
            
            <script>
                // Auto-refresh toutes les 5 secondes
                setTimeout(() => location.reload(), 5000);
            </script>
        </body>
        </html>
        """
    
    def run(self):
        """Démarrer serveur web"""
        self.log(f"🌐 Démarrage serveur sur http://{self.host}:{self.port}")
        uvicorn.run(self.app, host=self.host, port=self.port, log_level="info")


def main():
    """Point d'entrée principal"""
    ros2_initialized = False
    
    if ROS2_AVAILABLE:
        try:
            rclpy.init()
            ros2_initialized = True
            print("✅ ROS2 initialisé")
        except Exception as e:
            print(f"⚠️ ROS2 non initialisé: {e}")
            ros2_initialized = False
    
    try:
        server = DiamantWebServer()
        
        if ROS2_AVAILABLE and ros2_initialized:
            # Lancer ROS2 dans thread séparé
            def ros_thread():
                try:
                    rclpy.spin(server)
                except Exception as e:
                    print(f"❌ Erreur ROS2 spin: {e}")
            
            threading.Thread(target=ros_thread, daemon=True).start()
        
        # Lancer serveur web
        server.run()
        
    except KeyboardInterrupt:
        print("\n🛑 Arrêt serveur web")
    finally:
        if ROS2_AVAILABLE and ros2_initialized:
            try:
                rclpy.shutdown()
            except:
                pass


if __name__ == "__main__":
    main()