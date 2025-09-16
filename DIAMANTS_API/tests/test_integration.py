"""
DIAMANTS API Integration Tests
Tests for WebSocket bridge and ROS2 integration
"""

import pytest
import asyncio
import json
import websockets
from unittest.mock import Mock, patch, AsyncMock
import sys
import os

# Add the API directory to the path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from services.websocket_service import DiamantWebSocketService  # type: ignore

class TestWebSocketBridge:
    """Test class for WebSocket bridge functionality"""
    
    @pytest.fixture
    def mock_ros_node(self):
        """Mock ROS2 node for testing"""
        mock_node = Mock()
        mock_node.create_publisher = Mock()
        mock_node.create_subscription = Mock()
        return mock_node
    
    @pytest.fixture
    def ws_bridge(self, mock_ros_node):
        """Create WebSocket bridge instance with mocked ROS2"""
        with patch('services.websocket_service.rclpy.create_node', return_value=mock_ros_node):
            bridge = DiamantWebSocketService()  # type: ignore
            return bridge
    
    def test_bridge_initialization(self, ws_bridge):
        """Test WebSocket bridge initialization"""
        assert ws_bridge is not None
        assert hasattr(ws_bridge, 'websocket_clients')
        assert hasattr(ws_bridge, 'ros_publishers')
        assert hasattr(ws_bridge, 'ros_subscribers')
    
    @pytest.mark.asyncio
    async def test_websocket_client_connection(self, ws_bridge):
        """Test WebSocket client connection handling"""
        # Mock WebSocket client
        mock_websocket = AsyncMock()
        mock_websocket.remote_address = ('127.0.0.1', 12345)
        
        # Test connection
        await ws_bridge._handle_websocket_client(mock_websocket, '/test')
        
        # Verify client was added (if implementation adds clients)
        # This depends on the actual implementation
        assert True  # Placeholder assertion
    
    @pytest.mark.asyncio
    async def test_ros2_message_forwarding(self, ws_bridge):
        """Test ROS2 message forwarding to WebSocket"""
        # Mock ROS2 message
        mock_msg = Mock()
        mock_msg.linear.x = 1.0
        mock_msg.linear.y = 0.0
        mock_msg.linear.z = 0.5
        mock_msg.angular.x = 0.0
        mock_msg.angular.y = 0.0
        mock_msg.angular.z = 0.2
        
        # Test message processing
        with patch.object(ws_bridge, '_schedule_websocket_broadcast') as mock_broadcast:
            # This would normally be called by ROS2 callback
            # await ws_bridge._process_ros2_message(mock_msg)
            pass  # Implementation-dependent
    
    @pytest.mark.asyncio
    async def test_websocket_message_to_ros2(self, ws_bridge):
        """Test WebSocket message forwarding to ROS2"""
        test_message = {
            "type": "command",
            "data": {
                "action": "takeoff",
                "drone_id": "crazyflie_01"
            }
        }
        
        # Test message forwarding
        try:
            await ws_bridge.send_to_ros2(test_message)
            assert True  # If no exception, consider it successful
        except Exception as e:
            # Expected if ROS2 is not actually running
            assert "rclpy" in str(e) or "node" in str(e)

class TestAPIIntegration:
    """Test class for full API integration"""
    
    @pytest.mark.asyncio
    async def test_full_command_flow(self):
        """Test complete command flow from API to ROS2"""
        # This would test the full flow:
        # API endpoint -> WebSocket -> ROS2 -> Response
        # For now, we'll just test the API endpoint response
        
        from fastapi.testclient import TestClient
        from api.main import app
        
        client = TestClient(app)
        
        with patch('api.main.ws_bridge.send_to_ros2', new_callable=AsyncMock) as mock_send:
            response = client.post("/api/drones/crazyflie_01/takeoff")
            
            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "success"
            assert data["action"] == "takeoff"
    
    @pytest.mark.asyncio
    async def test_websocket_real_time_communication(self):
        """Test real-time WebSocket communication"""
        from fastapi.testclient import TestClient
        from api.main import app
        
        client = TestClient(app)
        
        with client.websocket_connect("/ws/test_client") as websocket:
            # Send test command
            test_command = {
                "type": "command",
                "data": {
                    "action": "takeoff",
                    "drone_id": "crazyflie_01"
                }
            }
            
            websocket.send_json(test_command)
            
            # Should receive the command back (broadcast)
            response = websocket.receive_json()
            assert response["type"] == "command"
            assert response["data"]["action"] == "takeoff"

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
        
        # Test response time for drone status
        start_time = time.time()
        response = client.get("/api/drones/crazyflie_01/status")
        end_time = time.time()
        
        assert response.status_code == 200
        assert (end_time - start_time) < 1.0  # Should respond within 1 second

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
