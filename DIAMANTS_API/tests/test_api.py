"""
DIAMANTS API Tests
Unit tests for the main API endpoints
"""

import pytest
import asyncio
import json
from fastapi.testclient import TestClient
from fastapi import status

# Import des modèles nécessaires
from api.models import DroneModel, DroneType, DroneStatus, MissionModel, MissionStatus, Position, Velocity
from unittest.mock import Mock, patch, AsyncMock
import sys
import os

# Add the API directory to the path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from api.main import app, manager, ws_bridge
from api.models import DroneModel, MissionModel, DroneStatus, MissionStatus

# Test client
client = TestClient(app)

class TestAPIEndpoints:
    """Test class for API endpoints"""
    
    def test_root_endpoint(self):
        """Test the root endpoint"""
        response = client.get("/")
        assert response.status_code == status.HTTP_200_OK
        data = response.json()
        assert data["service"] == "DIAMANTS API"
        assert data["version"] == "1.0.0"
        assert data["status"] == "running"
        assert "timestamp" in data
        assert "active_connections" in data

    def test_health_check(self):
        """Test the health check endpoint"""
        response = client.get("/health")
        assert response.status_code == status.HTTP_200_OK
        data = response.json()
        assert data["status"] == "healthy"
        assert "timestamp" in data
        assert "services" in data

    def test_list_drones(self):
        """Test listing drones"""
        response = client.get("/api/drones")
        assert response.status_code == status.HTTP_200_OK
        data = response.json()
        assert "drones" in data
        assert isinstance(data["drones"], list)
        
        if data["drones"]:
            drone = data["drones"][0]
            assert "id" in drone
            assert "type" in drone
            assert "status" in drone
            assert "battery" in drone
            assert "position" in drone

    def test_get_drone_status(self):
        """Test getting specific drone status"""
        drone_id = "crazyflie_01"
        response = client.get(f"/api/drones/{drone_id}/status")
        assert response.status_code == status.HTTP_200_OK
        data = response.json()
        assert data["drone_id"] == drone_id
        assert "status" in data
        assert "battery" in data
        assert "position" in data
        assert "velocity" in data
        assert "timestamp" in data

    @patch('api.main.ws_bridge.send_to_ros2', new_callable=AsyncMock)
    @patch('api.main.manager.broadcast', new_callable=AsyncMock)
    def test_takeoff_drone(self, mock_broadcast, mock_send_ros2):
        """Test drone takeoff command"""
        drone_id = "crazyflie_01"
        response = client.post(f"/api/drones/{drone_id}/takeoff")
        assert response.status_code == status.HTTP_200_OK
        data = response.json()
        assert data["status"] == "success"
        assert data["action"] == "takeoff"
        assert data["drone_id"] == drone_id

    @patch('api.main.ws_bridge.send_to_ros2', new_callable=AsyncMock)
    @patch('api.main.manager.broadcast', new_callable=AsyncMock)
    def test_land_drone(self, mock_broadcast, mock_send_ros2):
        """Test drone landing command"""
        drone_id = "crazyflie_01"
        response = client.post(f"/api/drones/{drone_id}/land")
        assert response.status_code == status.HTTP_200_OK
        data = response.json()
        assert data["status"] == "success"
        assert data["action"] == "land"
        assert data["drone_id"] == drone_id

    @patch('api.main.ws_bridge.send_to_ros2', new_callable=AsyncMock)
    @patch('api.main.manager.broadcast', new_callable=AsyncMock)
    def test_move_drone(self, mock_broadcast, mock_send_ros2):
        """Test drone movement command"""
        drone_id = "crazyflie_01"
        direction = {"x": 1.0, "y": 0.0, "z": 0.5}
        response = client.post(f"/api/drones/{drone_id}/move", json=direction)
        assert response.status_code == status.HTTP_200_OK
        data = response.json()
        assert data["status"] == "success"
        assert data["action"] == "move"
        assert data["drone_id"] == drone_id
        assert data["direction"] == direction

    def test_list_missions(self):
        """Test listing missions"""
        response = client.get("/api/missions")
        assert response.status_code == status.HTTP_200_OK
        data = response.json()
        assert "missions" in data
        assert isinstance(data["missions"], list)
        
        if data["missions"]:
            mission = data["missions"][0]
            assert "id" in mission
            assert "name" in mission
            assert "description" in mission
            assert "status" in mission
            assert "drones_required" in mission

    @patch('api.main.ws_bridge.send_to_ros2', new_callable=AsyncMock)
    @patch('api.main.manager.broadcast', new_callable=AsyncMock)
    def test_start_mission(self, mock_broadcast, mock_send_ros2):
        """Test starting a mission"""
        mission_id = "scouting_mission_01"
        drone_ids = ["crazyflie_01", "crazyflie_02"]
        response = client.post(f"/api/missions/{mission_id}/start", json=drone_ids)
        assert response.status_code == status.HTTP_200_OK
        data = response.json()
        assert data["status"] == "success"
        assert data["mission_id"] == mission_id
        assert data["drone_ids"] == drone_ids

class TestModels:
    """Test class for data models"""
    
    def test_drone_model(self):
        """Test DroneModel validation"""
        drone_data = {
            "id": "crazyflie_01",
            "type": "crazyflie",
            "status": "ready",
            "battery": 95.5,
            "position": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
        drone = DroneModel(**drone_data)
        assert drone.id == "crazyflie_01"
        assert drone.type == "crazyflie"
        assert drone.status == DroneStatus.READY
        assert drone.battery == 95.5
        assert drone.position.x == 0.0

    def test_mission_model(self):
        """Test MissionModel validation"""
        mission_data = {
            "id": "test_mission",
            "name": "Test Mission",
            "description": "A test mission",
            "status": "available",
            "drones_required": 2
        }
        mission = MissionModel(**mission_data)
        assert mission.id == "test_mission"
        assert mission.name == "Test Mission"
        assert mission.status == MissionStatus.AVAILABLE
        assert mission.drones_required == 2

    def test_invalid_battery_level(self):
        """Test invalid battery level validation"""
        with pytest.raises(ValueError):
            DroneModel(
                id="test",
                type=DroneType.CRAZYFLIE,  # Utilisation de l'enum
                status=DroneStatus.READY,  # Utilisation de l'enum
                battery=150,  # Invalid: > 100
                position=Position(x=0, y=0, z=0),  # Utilisation de la classe Position
                velocity=Velocity(x=0, y=0, z=0)   # Utilisation de la classe Velocity
            )

class TestWebSocket:
    """Test class for WebSocket functionality"""
    
    def test_websocket_connection(self):
        """Test WebSocket connection"""
        with client.websocket_connect("/ws/test_client") as websocket:
            # Test connection
            assert websocket is not None
            
            # Send test message
            test_message = {
                "type": "test",
                "data": {"message": "Hello WebSocket"}
            }
            websocket.send_json(test_message)
            
            # Should receive the same message back (broadcast)
            data = websocket.receive_json()
            assert data["type"] == "test"
            assert data["data"]["message"] == "Hello WebSocket"

class TestErrorHandling:
    """Test class for error handling"""
    
    def test_invalid_drone_id(self):
        """Test handling of invalid drone IDs"""
        # This should still return 200 but might have different behavior in real implementation
        response = client.get("/api/drones/invalid_drone_id/status")
        assert response.status_code == status.HTTP_200_OK

    def test_invalid_mission_id(self):
        """Test handling of invalid mission IDs"""
        response = client.post("/api/missions/invalid_mission/start", json=["drone1"])
        # Assuming the current implementation doesn't validate mission IDs
        assert response.status_code in [status.HTTP_200_OK, status.HTTP_404_NOT_FOUND]

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
