"""
DIAMANTS API - Main REST API Server
Centralizes all communication between Frontend and Backend.

Commands flow:  Frontend → REST/WS → Bridge (port 8765) → ROS2
Telemetry flow: ROS2 → Bridge → Frontend (via WS on 8765)
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import asyncio
import json
import logging
from typing import Dict, List, Any, Optional
from datetime import datetime
import os
import sys

# Add the parent directory to the path to import config
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.topics import PORTS

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="DIAMANTS API",
    description="Collaborative Drone Swarm Simulation Platform API",
    version="1.0.0",
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---------------------------------------------------------------------------
# Internal helper: forward commands to DiamantsBridge via its WebSocket
# ---------------------------------------------------------------------------
BRIDGE_URL = f"ws://127.0.0.1:{PORTS['websocket']}"

async def _send_to_bridge(message: Dict) -> bool:
    """Send a single JSON message to the DiamantsBridge WS server.
    Returns True on success, False on failure (bridge unreachable, etc.)."""
    try:
        import websockets  # type: ignore
        async with websockets.connect(BRIDGE_URL, close_timeout=2) as ws:
            await ws.send(json.dumps(message))
            return True
    except Exception as e:
        logger.warning(f"Cannot reach bridge at {BRIDGE_URL}: {e}")
        return False


async def _bridge_is_alive() -> bool:
    """Quick ping to check if the bridge WS server is reachable."""
    try:
        import websockets  # type: ignore
        async with websockets.connect(BRIDGE_URL, close_timeout=2) as ws:
            await ws.send(json.dumps({"type": "ping"}))
            resp = await asyncio.wait_for(ws.recv(), timeout=2)
            data = json.loads(resp)
            return data.get("type") == "pong"
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Active WebSocket connections (frontend clients → this FastAPI server)
# ---------------------------------------------------------------------------
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.connection_ids: Dict[WebSocket, str] = {}

    async def connect(self, websocket: WebSocket, client_id: str):
        await websocket.accept()
        self.active_connections.append(websocket)
        self.connection_ids[websocket] = client_id
        logger.info(f"Client {client_id} connected. Total: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            client_id = self.connection_ids.pop(websocket, "unknown")
            self.active_connections.remove(websocket)
            logger.info(f"Client {client_id} disconnected. Remaining: {len(self.active_connections)}")

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        gone = []
        for conn in self.active_connections:
            try:
                await conn.send_text(message)
            except Exception:
                gone.append(conn)
        for conn in gone:
            self.disconnect(conn)

manager = ConnectionManager()

@app.get("/")
async def root():
    """Root endpoint - API status"""
    return {
        "service": "DIAMANTS API",
        "version": "1.0.0",
        "status": "running",
        "timestamp": datetime.now().isoformat(),
        "active_connections": len(manager.active_connections),
    }


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    bridge_alive = await _bridge_is_alive()
    return {
        "status": "healthy" if bridge_alive else "degraded",
        "timestamp": datetime.now().isoformat(),
        "services": {
            "api": True,
            "websocket_bridge": bridge_alive,
            "active_connections": len(manager.active_connections),
        },
    }


# -----------------------------------------------------------------------
# Drone control endpoints
# -----------------------------------------------------------------------
@app.post("/api/drones/{drone_id}/takeoff")
async def takeoff_drone(drone_id: str):
    """Initiate drone takeoff"""
    command = {
        "type": "drone_command",
        "data": {
            "action": "takeoff",
            "drone_id": drone_id,
            "timestamp": datetime.now().isoformat(),
        },
    }
    sent = await _send_to_bridge(command)
    await manager.broadcast(json.dumps(command))
    if not sent:
        logger.warning(f"Takeoff for {drone_id}: bridge unreachable, broadcast only")
    return {"status": "success", "action": "takeoff", "drone_id": drone_id, "bridge": sent}


@app.post("/api/drones/{drone_id}/land")
async def land_drone(drone_id: str):
    """Initiate drone landing"""
    command = {
        "type": "drone_command",
        "data": {
            "action": "land",
            "drone_id": drone_id,
            "timestamp": datetime.now().isoformat(),
        },
    }
    sent = await _send_to_bridge(command)
    await manager.broadcast(json.dumps(command))
    if not sent:
        logger.warning(f"Land for {drone_id}: bridge unreachable, broadcast only")
    return {"status": "success", "action": "land", "drone_id": drone_id, "bridge": sent}


@app.post("/api/drones/{drone_id}/move")
async def move_drone(drone_id: str, direction: Dict[str, float]):
    """Move drone in specified direction (linear velocity)"""
    command = {
        "type": "cmd_vel",
        "data": {
            "linear": direction,
            "angular": {"x": 0, "y": 0, "z": 0},
        },
    }
    sent = await _send_to_bridge(command)
    await manager.broadcast(json.dumps({
        "type": "drone_command",
        "data": {"action": "move", "drone_id": drone_id, "direction": direction},
    }))
    if not sent:
        logger.warning(f"Move for {drone_id}: bridge unreachable, broadcast only")
    return {"status": "success", "action": "move", "drone_id": drone_id, "direction": direction, "bridge": sent}


@app.get("/api/drones")
async def list_drones():
    """List all available drones.
    TODO(T1.4): replace with real telemetry from bridge state cache."""
    return {
        "drones": [
            {
                "id": "crazyflie_01",
                "type": "crazyflie",
                "status": "ready",
                "battery": 95,
                "position": {"x": 0, "y": 0, "z": 0},
            }
        ]
    }


@app.get("/api/drones/{drone_id}/status")
async def get_drone_status(drone_id: str):
    """Get specific drone status.
    TODO(T1.4): replace with real telemetry from bridge state cache."""
    return {
        "drone_id": drone_id,
        "status": "ready",
        "battery": 95,
        "position": {"x": 0, "y": 0, "z": 0},
        "velocity": {"x": 0, "y": 0, "z": 0},
        "timestamp": datetime.now().isoformat(),
    }


# -----------------------------------------------------------------------
# WebSocket endpoint for real-time frontend communication
# -----------------------------------------------------------------------
@app.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: str):
    await manager.connect(websocket, client_id)
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            logger.info(f"Received from {client_id}: {message.get('type', '?')}")

            # Forward commands to bridge → ROS2
            msg_type = message.get("type")
            if msg_type in ("command", "drone_command", "cmd_vel", "swarm_command", "mission_command"):
                await _send_to_bridge(message)

            # Broadcast to other frontend clients
            await manager.broadcast(data)

    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error for {client_id}: {e}")
        manager.disconnect(websocket)


# -----------------------------------------------------------------------
# Mission management endpoints
# -----------------------------------------------------------------------
@app.get("/api/missions")
async def list_missions():
    """List available missions.
    TODO(T1.4): load from mission config / ROS2 params."""
    return {
        "missions": [
            {
                "id": "scouting_mission_01",
                "name": "Collaborative Scouting",
                "description": "Multi-drone reconnaissance mission",
                "status": "available",
                "drones_required": 3,
            },
            {
                "id": "slam_mission_01",
                "name": "SLAM Mapping",
                "description": "Collaborative SLAM mapping mission",
                "status": "available",
                "drones_required": 2,
            },
        ]
    }


@app.post("/api/missions/{mission_id}/start")
async def start_mission(mission_id: str, drone_ids: List[str]):
    """Start a mission with specified drones"""
    command = {
        "type": "mission_command",
        "data": {
            "action": "start",
            "mission_id": mission_id,
            "drone_ids": drone_ids,
            "timestamp": datetime.now().isoformat(),
        },
    }
    sent = await _send_to_bridge(command)
    await manager.broadcast(json.dumps(command))
    if not sent:
        logger.warning(f"Mission {mission_id}: bridge unreachable, broadcast only")
    return {
        "status": "success",
        "mission_id": mission_id,
        "drone_ids": drone_ids,
        "bridge": sent,
        "message": "Mission started successfully",
    }


# -----------------------------------------------------------------------
# Entry-point (standalone)
# -----------------------------------------------------------------------
if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=PORTS["rest_api"],
        reload=True,
        log_level="info",
    )
