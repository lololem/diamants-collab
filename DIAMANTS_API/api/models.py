"""
DIAMANTS API Models
Data models for the API endpoints
"""

from pydantic import BaseModel, Field
from typing import Dict, List, Optional, Any
from datetime import datetime
from enum import Enum

class DroneStatus(str, Enum):
    """Drone status enumeration"""
    READY = "ready"
    FLYING = "flying"
    LANDING = "landing"
    ERROR = "error"
    MAINTENANCE = "maintenance"

class DroneType(str, Enum):
    """Drone type enumeration"""
    CRAZYFLIE = "crazyflie"
    CUSTOM = "custom"

class Position(BaseModel):
    """3D Position model"""
    x: float = Field(..., description="X coordinate")
    y: float = Field(..., description="Y coordinate") 
    z: float = Field(..., description="Z coordinate")

class Velocity(BaseModel):
    """3D Velocity model"""
    x: float = Field(..., description="X velocity")
    y: float = Field(..., description="Y velocity")
    z: float = Field(..., description="Z velocity")

class DroneModel(BaseModel):
    """Drone data model"""
    id: str = Field(..., description="Unique drone identifier")
    type: DroneType = Field(..., description="Type of drone")
    status: DroneStatus = Field(..., description="Current drone status")
    battery: float = Field(..., ge=0, le=100, description="Battery percentage")
    position: Position = Field(..., description="Current position")
    velocity: Optional[Velocity] = Field(None, description="Current velocity")
    timestamp: datetime = Field(default_factory=datetime.now, description="Last update timestamp")

class DroneCommand(BaseModel):
    """Drone command model"""
    action: str = Field(..., description="Action to perform")
    drone_id: str = Field(..., description="Target drone ID")
    parameters: Optional[Dict[str, Any]] = Field(None, description="Command parameters")
    timestamp: datetime = Field(default_factory=datetime.now, description="Command timestamp")

class MoveCommand(BaseModel):
    """Move command model"""
    direction: Dict[str, float] = Field(..., description="Movement direction (x, y, z)")
    speed: Optional[float] = Field(1.0, ge=0, le=5, description="Movement speed")
    duration: Optional[float] = Field(None, ge=0, description="Movement duration in seconds")

class MissionStatus(str, Enum):
    """Mission status enumeration"""
    AVAILABLE = "available"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    PAUSED = "paused"

class MissionModel(BaseModel):
    """Mission data model"""
    id: str = Field(..., description="Unique mission identifier")
    name: str = Field(..., description="Mission name")
    description: str = Field(..., description="Mission description")
    status: MissionStatus = Field(..., description="Current mission status")
    drones_required: int = Field(..., ge=1, description="Number of drones required")
    drones_assigned: Optional[List[str]] = Field(None, description="List of assigned drone IDs")
    created_at: datetime = Field(default_factory=datetime.now, description="Mission creation timestamp")
    started_at: Optional[datetime] = Field(None, description="Mission start timestamp")
    completed_at: Optional[datetime] = Field(None, description="Mission completion timestamp")

class StartMissionRequest(BaseModel):
    """Start mission request model"""
    drone_ids: List[str] = Field(..., description="List of drone IDs to assign to mission")
    parameters: Optional[Dict[str, Any]] = Field(None, description="Mission parameters")

class WebSocketMessage(BaseModel):
    """WebSocket message model"""
    type: str = Field(..., description="Message type")
    data: Dict[str, Any] = Field(..., description="Message data")
    timestamp: datetime = Field(default_factory=datetime.now, description="Message timestamp")
    client_id: Optional[str] = Field(None, description="Client ID")

class APIResponse(BaseModel):
    """Standard API response model"""
    status: str = Field(..., description="Response status")
    message: Optional[str] = Field(None, description="Response message")
    data: Optional[Any] = Field(None, description="Response data")
    timestamp: datetime = Field(default_factory=datetime.now, description="Response timestamp")

class HealthStatus(BaseModel):
    """Health check response model"""
    status: str = Field(..., description="Service health status")
    timestamp: datetime = Field(default_factory=datetime.now, description="Health check timestamp")
    services: Dict[str, Any] = Field(..., description="Individual service statuses")

class ErrorResponse(BaseModel):
    """Error response model"""
    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Error message")
    details: Optional[Dict[str, Any]] = Field(None, description="Error details")
    timestamp: datetime = Field(default_factory=datetime.now, description="Error timestamp")
