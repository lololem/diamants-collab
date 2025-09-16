# DIAMANTS V3 - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
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
DIAMANTS V3 Web Server
=====================
Serveur web FastAPI intégré pour interface utilisateur
Compatible avec architecture ROS2 simplifiée
"""

import os
import json
import time
import asyncio
from typing import Dict, List, Optional
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point

# FastAPI et WebSocket
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel
import uvicorn


class DiamantWebServer(Node):
    """Serveur web intégré pour DIAMANTS V3"""
    
    def __init__(self, host: str = "localhost", port: int = 8080):
        super().__init__('diamant_web_server')
        
        self.host = host
        self.port = port
        
        # État système
        self.swarm_data = {
            'active_drones': 0,
            'intelligence_score': 0.0,
            'coverage_area': 0.0,
            'dispersion': 0.0,
            'collision_risks': 0,
            'drone_positions': {},
            'last_update': time.time()
        }
        
        # Connexions WebSocket actives
        self.websocket_connections: List[WebSocket] = []
        
        # Initialiser FastAPI
        self.app = self._create_app()
        
        # Abonnements ROS2
        self._setup_ros2_subscriptions()
        
        self.get_logger().info(f"🌐 Web Server initialisé sur http://{host}:{port}")
    
    def _create_app(self) -> FastAPI:
        """Créer application FastAPI"""
        app = FastAPI(
            title="DIAMANTS V3 Web Interface",
            description="Interface web pour monitoring et contrôle des essaims",
            version="1.0.0"
        )
        
        # Chemins statiques
        static_path = Path(__file__).parent / "static"
        templates_path = Path(__file__).parent / "templates"
        
        app.mount("/static", StaticFiles(directory=str(static_path)), name="static")
        templates = Jinja2Templates(directory=str(templates_path))
        
        # Routes principales
        @app.get("/", response_class=HTMLResponse)
        async def dashboard(request: Request):
            """Page principale dashboard"""
            return templates.TemplateResponse("dashboard.html", {
                "request": request,
                "title": "DIAMANTS V3 Dashboard"
            })
        
        @app.get("/monitoring", response_class=HTMLResponse)
        async def monitoring(request: Request):
            """Page monitoring détaillé"""
            return templates.TemplateResponse("monitoring.html", {
                "request": request,
                "title": "DIAMANTS V3 Monitoring"
            })
        
        @app.get("/simulation", response_class=HTMLResponse)
        async def simulation(request: Request):
            """Page simulation 3D"""
            return templates.TemplateResponse("simulation.html", {
                "request": request,
                "title": "DIAMANTS V3 Simulation"
            })
        
        # API REST
        @app.get("/api/swarm/status")
        async def get_swarm_status():
            """État complet essaim"""
            return {
                'status': 'success',
                'data': self.swarm_data.copy(),
                'timestamp': time.time()
            }
        
        @app.get("/api/swarm/drones")
        async def get_drone_list():
            """Liste des drones actifs"""
            return {
                'status': 'success',
                'drones': list(self.swarm_data['drone_positions'].keys()),
                'count': self.swarm_data['active_drones']
            }
        
        @app.post("/api/swarm/command")
        async def send_command(command: dict):
            """Envoyer commande à l'essaim"""
            # TODO: Implémenter envoi commandes ROS2
            self.get_logger().info(f"Commande reçue: {command}")
            return {'status': 'success', 'message': 'Command received'}
        
        # WebSocket endpoint
        @app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await self.handle_websocket(websocket)
        
        return app
    
    def _setup_ros2_subscriptions(self):
        """Configuration abonnements ROS2"""
        # Positions drones
        self.position_sub = self.create_subscription(
            String, '/multi_agent/drone_positions',
            self.position_callback, 10
        )
        
        # Intelligence collective
        self.intelligence_sub = self.create_subscription(
            Float32, '/swarm/intelligence_score',
            self.intelligence_callback, 10
        )
        
        # Couverture
        self.coverage_sub = self.create_subscription(
            Float32, '/swarm/coverage_area',
            self.coverage_callback, 10
        )
        
        # Rapport statut
        self.status_sub = self.create_subscription(
            String, '/swarm/status_report',
            self.status_callback, 10
        )
    
    def position_callback(self, msg):
        """Callback positions drones"""
        try:
            # Parser positions (format: "drone_id:x:y:z")
            parts = msg.data.split(':')
            if len(parts) >= 4:
                drone_id = parts[0]
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                
                self.swarm_data['drone_positions'][drone_id] = {
                    'x': x, 'y': y, 'z': z,
                    'timestamp': time.time()
                }
                
                # Diffuser via WebSocket
                asyncio.create_task(self.broadcast_update('drone_positions', {
                    'drone_id': drone_id,
                    'position': {'x': x, 'y': y, 'z': z}
                }))
                
        except Exception as e:
            self.get_logger().debug(f"Erreur parsing position: {e}")
    
    def intelligence_callback(self, msg):
        """Callback intelligence score"""
        self.swarm_data['intelligence_score'] = msg.data
        self.swarm_data['last_update'] = time.time()
        
        # Diffuser via WebSocket
        asyncio.create_task(self.broadcast_update('intelligence', {
            'score': msg.data,
            'timestamp': time.time()
        }))
    
    def coverage_callback(self, msg):
        """Callback couverture"""
        self.swarm_data['coverage_area'] = msg.data
        
        # Diffuser via WebSocket
        asyncio.create_task(self.broadcast_update('coverage', {
            'area': msg.data,
            'timestamp': time.time()
        }))
    
    def status_callback(self, msg):
        """Callback rapport statut"""
        try:
            status = json.loads(msg.data)
            self.swarm_data.update({
                'active_drones': status.get('active_drones', 0),
                'dispersion': status.get('dispersion', 0.0),
                'collision_risks': status.get('collision_risks', 0)
            })
            
            # Diffuser via WebSocket
            asyncio.create_task(self.broadcast_update('status', status))
            
        except Exception as e:
            self.get_logger().debug(f"Erreur parsing status: {e}")
    
    async def handle_websocket(self, websocket: WebSocket):
        """Gestion connexion WebSocket"""
        await websocket.accept()
        self.websocket_connections.append(websocket)
        
        try:
            # Envoyer état initial
            await websocket.send_json({
                'type': 'initial_state',
                'data': self.swarm_data.copy()
            })
            
            # Boucle réception messages
            while True:
                data = await websocket.receive_json()
                await self.handle_websocket_message(websocket, data)
                
        except WebSocketDisconnect:
            self.websocket_connections.remove(websocket)
            self.get_logger().info("Client WebSocket déconnecté")
    
    async def handle_websocket_message(self, websocket: WebSocket, data: dict):
        """Traiter message WebSocket"""
        message_type = data.get('type')
        
        if message_type == 'ping':
            await websocket.send_json({'type': 'pong'})
        elif message_type == 'get_status':
            await websocket.send_json({
                'type': 'status_response',
                'data': self.swarm_data.copy()
            })
        elif message_type == 'command':
            # TODO: Traiter commandes utilisateur
            self.get_logger().info(f"Commande WebSocket: {data}")
    
    async def broadcast_update(self, update_type: str, data: dict):
        """Diffuser mise à jour à tous les clients WebSocket"""
        if not self.websocket_connections:
            return
        
        message = {
            'type': update_type,
            'data': data,
            'timestamp': time.time()
        }
        
        # Diffuser à tous les clients connectés
        disconnected = []
        for websocket in self.websocket_connections:
            try:
                await websocket.send_json(message)
            except Exception:
                disconnected.append(websocket)
        
        # Nettoyer connexions fermées
        for ws in disconnected:
            if ws in self.websocket_connections:
                self.websocket_connections.remove(ws)
    
    def start_server(self):
        """Démarrer serveur web (non-bloquant)"""
        config = uvicorn.Config(
            app=self.app,
            host=self.host,
            port=self.port,
            log_level="info"
        )
        server = uvicorn.Server(config)
        
        # Démarrer dans thread séparé
        import threading
        server_thread = threading.Thread(target=server.run, daemon=True)
        server_thread.start()
        
        self.get_logger().info(f"🌐 Serveur web démarré: http://{self.host}:{self.port}")
        return server_thread


# Modèles Pydantic pour API
class SwarmCommand(BaseModel):
    """Modèle commande essaim"""
    command_type: str
    target_drones: Optional[List[str]] = None
    parameters: Optional[Dict] = None


def main():
    """Point d'entrée principal"""
    rclpy.init()
    
    try:
        web_server = DiamantWebServer(host="0.0.0.0", port=8080)
        web_server.start_server()
        
        # Boucle ROS2
        rclpy.spin(web_server)
        
    except KeyboardInterrupt:
        print("\n🛑 Serveur web arrêté")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()