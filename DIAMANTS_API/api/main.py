"""
DIAMANTS API - Main REST API Server
Centralizes all communication between Frontend and Backend
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import uvicorn
import asyncio
import json
import logging
from typing import Dict, List, Any
from datetime import datetime
import os
import sys

# Add the parent directory to the path to import services
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from services.websocket_service import DiamantWebSocketService

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="DIAMANTS API",
    description="Collaborative Drone Swarm Simulation Platform API",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# WebSocket Bridge instance
ws_bridge = DiamantWebSocketService()

# Active WebSocket connections
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.connection_ids: Dict[WebSocket, str] = {}

    async def connect(self, websocket: WebSocket, client_id: str):
        await websocket.accept()
        self.active_connections.append(websocket)
        self.connection_ids[websocket] = client_id
        logger.info(f"Client {client_id} connected. Total connections: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            client_id = self.connection_ids.get(websocket, "unknown")
            self.active_connections.remove(websocket)
            if websocket in self.connection_ids:
                del self.connection_ids[websocket]
            logger.info(f"Client {client_id} disconnected. Total connections: {len(self.active_connections)}")

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.error(f"Error broadcasting to connection: {e}")

manager = ConnectionManager()

@app.get("/")
async def root():
    """Root endpoint - API status"""
    return {
        "service": "DIAMANTS API",
        "version": "1.0.0",
        "status": "running",
        "timestamp": datetime.now().isoformat(),
        "active_connections": len(manager.active_connections)
    }

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "services": {
            "websocket_bridge": ws_bridge.is_connected() if hasattr(ws_bridge, 'is_connected') else True,
            "active_connections": len(manager.active_connections)
        }
    }

# Drone control endpoints
@app.post("/api/drones/{drone_id}/takeoff")
async def takeoff_drone(drone_id: str):
    """Initiate drone takeoff"""
    try:
        command = {
            "type": "command",
            "data": {
                "action": "takeoff",
                "drone_id": drone_id,
                "timestamp": datetime.now().isoformat()
            }
        }
        
        # Send to ROS2 via WebSocket bridge
        await ws_bridge.send_to_ros2(command)
        
        # Broadcast to all connected clients
        await manager.broadcast(json.dumps(command))
        
        return {"status": "success", "action": "takeoff", "drone_id": drone_id}
    except Exception as e:
        logger.error(f"Takeoff error for drone {drone_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/drones/{drone_id}/land")
async def land_drone(drone_id: str):
    """Initiate drone landing"""
    try:
        command = {
            "type": "command",
            "data": {
                "action": "land",
                "drone_id": drone_id,
                "timestamp": datetime.now().isoformat()
            }
        }
        
        await ws_bridge.send_to_ros2(command)
        await manager.broadcast(json.dumps(command))
        
        return {"status": "success", "action": "land", "drone_id": drone_id}
    except Exception as e:
        logger.error(f"Landing error for drone {drone_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/drones/{drone_id}/move")
async def move_drone(drone_id: str, direction: Dict[str, float]):
    """Move drone in specified direction"""
    try:
        command = {
            "type": "command",
            "data": {
                "action": "move",
                "drone_id": drone_id,
                "direction": direction,
                "timestamp": datetime.now().isoformat()
            }
        }
        
        await ws_bridge.send_to_ros2(command)
        await manager.broadcast(json.dumps(command))
        
        return {"status": "success", "action": "move", "drone_id": drone_id, "direction": direction}
    except Exception as e:
        logger.error(f"Move error for drone {drone_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/drones")
async def list_drones():
    """List all available drones"""
    # This would typically query the ROS2 system for available drones
    return {
        "drones": [
            {
                "id": "crazyflie_01",
                "type": "crazyflie",
                "status": "ready",
                "battery": 95,
                "position": {"x": 0, "y": 0, "z": 0}
            }
        ]
    }

@app.get("/api/drones/{drone_id}/status")
async def get_drone_status(drone_id: str):
    """Get specific drone status"""
    # This would query the actual drone status from ROS2
    return {
        "drone_id": drone_id,
        "status": "ready",
        "battery": 95,
        "position": {"x": 0, "y": 0, "z": 0},
        "velocity": {"x": 0, "y": 0, "z": 0},
        "timestamp": datetime.now().isoformat()
    }

# WebSocket endpoint for real-time communication
@app.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: str):
    await manager.connect(websocket, client_id)
    try:
        while True:
            # Receive message from client
            data = await websocket.receive_text()
            message = json.loads(data)
            
            logger.info(f"Received from {client_id}: {message}")
            
            # Forward to ROS2 if it's a command
            if message.get("type") == "command":
                await ws_bridge.send_to_ros2(message)
            
            # Broadcast to other clients
            await manager.broadcast(data)
            
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error for {client_id}: {e}")
        manager.disconnect(websocket)

# Mission management endpoints
@app.get("/api/missions")
async def list_missions():
    """List available missions"""
    return {
        "missions": [
            {
                "id": "scouting_mission_01",
                "name": "Collaborative Scouting",
                "description": "Multi-drone reconnaissance mission",
                "status": "available",
                "drones_required": 3
            },
            {
                "id": "slam_mission_01", 
                "name": "SLAM Mapping",
                "description": "Collaborative SLAM mapping mission",
                "status": "available",
                "drones_required": 2
            }
        ]
    }

@app.post("/api/missions/{mission_id}/start")
async def start_mission(mission_id: str, drone_ids: List[str]):
    """Start a mission with specified drones"""
    try:
        command = {
            "type": "mission",
            "data": {
                "action": "start",
                "mission_id": mission_id,
                "drone_ids": drone_ids,
                "timestamp": datetime.now().isoformat()
            }
        }
        
        await ws_bridge.send_to_ros2(command)
        await manager.broadcast(json.dumps(command))
        
        return {
            "status": "success", 
            "mission_id": mission_id, 
            "drone_ids": drone_ids,
            "message": "Mission started successfully"
        }
    except Exception as e:
        logger.error(f"Mission start error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    # Start the WebSocket bridge in the background
    asyncio.create_task(ws_bridge.start())
    
    # Run the API server
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
