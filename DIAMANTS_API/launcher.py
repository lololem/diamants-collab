#!/usr/bin/env python3
"""
DIAMANTS API - Launcher avec support ROS2
Lance l'API REST et le WebSocket bridge avec l'environnement ROS2
"""

import uvicorn
import rclpy  # type: ignore
import asyncio
import threading

# Initialize ROS2 BEFORE importing any ROS2-dependent modules
rclpy.init()

from api.main import app
from services.websocket_service import DiamantWebSocketService

async def start_api_and_websocket():
    """Lance l'API REST et le WebSocket service principal"""
    
    print("🚀 Démarrage de DIAMANTS API avec support ROS2")
    
    # Lancer le WebSocket service principal
    websocket_service = DiamantWebSocketService(
        ws_host="0.0.0.0",
        ws_port=8765,
        log_level="INFO"
    )
    
    # Lancer les deux services en parallèle
    api_task = asyncio.create_task(run_api_server())
    websocket_task = asyncio.create_task(websocket_service.start_websocket_server())
    
    try:
        await asyncio.gather(api_task, websocket_task)
    except KeyboardInterrupt:
        print("\n🛑 Arrêt des services...")
    finally:
        rclpy.shutdown()

async def run_api_server():
    """Lance le serveur API"""
    config = uvicorn.Config(
        app=app,
        host="0.0.0.0", 
        port=8000,
        reload=False,
        log_level="info"
    )
    server = uvicorn.Server(config)
    await server.serve()

if __name__ == "__main__":
    asyncio.run(start_api_and_websocket())
