"""
DIAMANTS API Integration Tests
Tests for REST API, WebSocket endpoints, and bridge connectivity.
"""

import pytest
import asyncio
import json
from unittest.mock import Mock, patch, AsyncMock
import sys
import os

# Add the API directory to the path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestWebSocketBridge:
    """Test the unified DiamantsBridge (services.websocket_bridge)."""

    @pytest.fixture
    def bridge(self):
        """Create a DiamantsBridge instance (ROS2 stub mode)."""
        from services.websocket_bridge import DiamantsBridge
        b = DiamantsBridge(ws_host="127.0.0.1", ws_port=0)  # port 0 → OS picks a free port
        yield b
        try:
            b.destroy_node()
        except Exception:
            pass

    def test_bridge_initialization(self, bridge):
        assert bridge is not None
        assert hasattr(bridge, "ws_clients")
        assert hasattr(bridge, "state")
        assert "drones" in bridge.state
        assert "swarm" in bridge.state

    def test_bridge_state_defaults(self, bridge):
        assert bridge.state["swarm"]["intelligence_score"] == 0.0
        assert bridge.state["mission"]["status"] == "idle"


class TestAPIEndpoints:
    """Test the FastAPI REST endpoints in api/main.py."""

    @pytest.fixture
    def client(self):
        from fastapi.testclient import TestClient
        from api.main import app
        return TestClient(app)

    def test_root(self, client):
        resp = client.get("/")
        assert resp.status_code == 200
        data = resp.json()
        assert data["service"] == "DIAMANTS API"
        assert data["status"] == "running"

    def test_health(self, client):
        resp = client.get("/health")
        assert resp.status_code == 200
        data = resp.json()
        # Bridge is not running in tests → status should be "degraded"
        assert data["status"] in ("healthy", "degraded")
        assert "services" in data

    def test_list_drones(self, client):
        resp = client.get("/api/drones")
        assert resp.status_code == 200
        data = resp.json()
        assert "drones" in data
        # Bridge is offline in tests → empty list is valid
        assert isinstance(data["drones"], list)

    def test_drone_status(self, client):
        # Non-existent drone → 404
        resp = client.get("/api/drones/nonexistent_drone_xyz/status")
        assert resp.status_code == 404

    def test_takeoff(self, client):
        """Takeoff should succeed even if bridge is unreachable (broadcast only)."""
        resp = client.post("/api/drones/crazyflie_01/takeoff")
        assert resp.status_code == 200
        data = resp.json()
        assert data["action"] == "takeoff"

    def test_land(self, client):
        resp = client.post("/api/drones/crazyflie_01/land")
        assert resp.status_code == 200
        assert resp.json()["action"] == "land"

    def test_list_missions(self, client):
        resp = client.get("/api/missions")
        assert resp.status_code == 200
        missions = resp.json()["missions"]
        assert len(missions) >= 2

class TestPerformance:
    """Test class for performance testing"""
    
    @pytest.mark.asyncio
    async def test_concurrent_websocket_connections(self):
        """Test handling multiple concurrent WebSocket connections"""
        from fastapi.testclient import TestClient
        from api.main import app
        
        client = TestClient(app)
        
        # Test multiple concurrent connections
        connections = []
        try:
            for i in range(5):
                ws = client.websocket_connect(f"/ws/client_{i}")
                connections.append(ws.__enter__())
            
            # Send messages from all connections
            for i, ws in enumerate(connections):
                test_message = {
                    "type": "test",
                    "data": {"client_id": f"client_{i}", "message": f"Hello from client {i}"}
                }
                ws.send_json(test_message)
            
            # Verify all connections receive broadcasts
            for ws in connections:
                received_count = 0
                try:
                    while received_count < 5:  # Should receive 5 broadcasts
                        ws.receive_json()
                        received_count += 1
                except:
                    break
                assert received_count == 5
                
        finally:
            # Clean up connections
            for ws in connections:
                try:
                    ws.__exit__(None, None, None)
                except:
                    pass
    
    def test_api_response_time(self):
        """Test API response times"""
        from fastapi.testclient import TestClient
        from api.main import app
        import time
        
        client = TestClient(app)
        
        # Test response time for simple endpoints
        start_time = time.time()
        response = client.get("/")
        end_time = time.time()
        
        assert response.status_code == 200
        assert (end_time - start_time) < 1.0  # Should respond within 1 second
        
        # Test response time for drone status (404 when bridge offline is OK)
        start_time = time.time()
        response = client.get("/api/drones/crazyflie_01/status")
        end_time = time.time()
        
        assert response.status_code in (200, 404)
        assert (end_time - start_time) < 2.0  # Allow time for WS connect attempt

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
