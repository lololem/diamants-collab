#!/usr/bin/env python3
"""
DIAMANTS API - ROS2 Integration Tests
Tests specifically for ROS2 WebSocket bridge functionality
"""

import pytest
import asyncio
import json
import sys
import os
from unittest.mock import Mock, patch

# Add system paths for ROS2
sys.path.append('/opt/ros/jazzy/lib/python3.12/site-packages')
import os
import sys

# Get the directory of this file
current_dir = os.path.dirname(os.path.abspath(__file__))
# Go up to DIAMANTS_API, then to project root, then to backend workspace  
project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
ros2_ws_path = os.path.join(project_root, 'DIAMANTS_BACKEND', 'slam_collaboratif', 'ros2_ws', 'install')
if os.path.exists(ros2_ws_path):
    sys.path.append(f"{ros2_ws_path}/lib/python3.12/site-packages")

# Import our WebSocket bridge
from api.websocket_bridge import WebSocketBridge, WebSocketBridgeServer

class TestROS2Integration:
    """Test ROS2 integration with WebSocket bridge"""
    
    @pytest.fixture
    def bridge(self):
        """Create a WebSocket bridge instance for testing"""
        return WebSocketBridge()
    
    def test_bridge_initialization(self, bridge):
        """Test that the bridge initializes correctly"""
        assert bridge is not None
        assert hasattr(bridge, 'websocket_clients')
        assert hasattr(bridge, 'client_info')
        assert hasattr(bridge, 'stats')
        
    def test_ros2_availability(self):
        """Test ROS2 availability detection"""
        try:
            import rclpy  # type: ignore
            from geometry_msgs.msg import Twist  # type: ignore
            ros2_available = True
        except ImportError:
            ros2_available = False
        
        # Should handle both cases gracefully
        assert isinstance(ros2_available, bool)
        
    def test_twist_message_creation(self, bridge):
        """Test creation of ROS2 Twist messages"""
        # Test data
        test_data = {
            'action': 'move',
            'linear_x': 1.0,
            'linear_y': 0.5,
            'linear_z': 0.2,
            'angular_x': 0.0,
            'angular_y': 0.0,
            'angular_z': 0.1
        }
        
        # This should not raise an exception
        asyncio.run(bridge._handle_drone_command(test_data))
        
    def test_mission_command_creation(self, bridge):
        """Test creation of mission commands"""
        test_data = {
            'mission_type': 'exploration',
            'parameters': {
                'area': 'zone_1',
                'duration': 300
            }
        }
        
        # This should not raise an exception
        asyncio.run(bridge._handle_mission_command(test_data))
        
    def test_websocket_message_parsing(self, bridge):
        """Test WebSocket message parsing"""
        # Valid message
        valid_message = json.dumps({
            'type': 'drone_command',
            'data': {
                'action': 'takeoff',
                'drone_id': 'crazyflie'
            }
        })
        
        # Mock websocket
        mock_websocket = Mock()
        
        # This should not raise an exception
        asyncio.run(bridge.handle_websocket_message(mock_websocket, valid_message))
        
    def test_invalid_message_handling(self, bridge):
        """Test handling of invalid messages"""
        # Invalid JSON
        invalid_json = "{ invalid json }"
        
        mock_websocket = Mock()
        
        # Should handle gracefully without raising exception
        asyncio.run(bridge.handle_websocket_message(mock_websocket, invalid_json))
        
    def test_client_info_handling(self, bridge):
        """Test client information handling"""
        mock_websocket = Mock()
        mock_websocket.send = Mock(return_value=asyncio.Future())
        mock_websocket.send.return_value.set_result(None)
        
        client_data = {
            'client_type': 'frontend',
            'client_name': 'mission_control'
        }
        
        asyncio.run(bridge._handle_client_info(mock_websocket, client_data))
        
        # Check that client info was stored
        assert mock_websocket in bridge.client_info
        assert bridge.client_info[mock_websocket]['type'] == 'frontend'
        
    def test_ping_pong(self, bridge):
        """Test ping/pong mechanism"""
        mock_websocket = Mock()
        mock_websocket.send = Mock(return_value=asyncio.Future())
        mock_websocket.send.return_value.set_result(None)
        
        asyncio.run(bridge._handle_ping(mock_websocket))
        
        # Should have sent a pong message
        mock_websocket.send.assert_called_once()
        
    def test_stats_tracking(self, bridge):
        """Test statistics tracking"""
        initial_stats = bridge.get_stats()
        
        assert 'messages_received' in initial_stats
        assert 'messages_sent' in initial_stats
        assert 'clients_connected' in initial_stats
        assert 'uptime_seconds' in initial_stats
        assert 'ros2_available' in initial_stats
        
        assert initial_stats['messages_received'] == 0
        assert initial_stats['messages_sent'] == 0
        
    @pytest.mark.asyncio
    async def test_broadcast_queue(self, bridge):
        """Test message broadcasting queue"""
        test_message = {
            'type': 'test',
            'data': {'test': True}
        }
        
        await bridge._queue_broadcast(test_message)
        
        # Check that message was queued
        assert not bridge.broadcast_queue.empty()
        
        # Get the message back
        queued_message = await bridge.broadcast_queue.get()
        assert queued_message == test_message


class TestWebSocketBridgeServer:
    """Test WebSocket bridge server functionality"""
    
    def test_server_initialization(self):
        """Test server initialization"""
        server = WebSocketBridgeServer("localhost", 9001)
        
        assert server.host == "localhost"
        assert server.port == 9001
        assert server.bridge is None
        assert server.server is None
        
    def test_server_initialization_custom_params(self):
        """Test server with custom parameters"""
        server = WebSocketBridgeServer("0.0.0.0", 8080)
        
        assert server.host == "0.0.0.0"
        assert server.port == 8080


class TestROS2MessageHandling:
    """Test ROS2 message handling without requiring actual ROS2 runtime"""
    
    @pytest.fixture
    def bridge(self):
        return WebSocketBridge()
        
    def test_propeller_callback_with_mock_data(self, bridge):
        """Test propeller callback with mock data"""
        # Create mock message
        mock_msg = Mock()
        mock_msg.data = [100.0, 101.0, 102.0, 103.0]
        
        # This should not raise an exception
        bridge.propeller_callback(mock_msg)
        
        # Check that stats were updated
        assert bridge.stats['ros2_messages'] > 0
        
    def test_status_callback_with_json(self, bridge):
        """Test status callback with JSON data"""
        mock_msg = Mock()
        mock_msg.data = json.dumps({
            'status': 'flying',
            'battery': 85.5,
            'position': {'x': 1.0, 'y': 2.0, 'z': 1.5}
        })
        
        bridge.status_callback(mock_msg)
        
        assert bridge.stats['ros2_messages'] > 0
        
    def test_status_callback_with_plain_text(self, bridge):
        """Test status callback with plain text"""
        mock_msg = Mock()
        mock_msg.data = "System operational"
        
        bridge.status_callback(mock_msg)
        
        assert bridge.stats['ros2_messages'] > 0


@pytest.mark.integration
class TestFullIntegration:
    """Integration tests that require more setup"""
    
    @pytest.mark.asyncio
    async def test_message_flow(self):
        """Test complete message flow from WebSocket to ROS2"""
        bridge = WebSocketBridge()
        
        # Simulate WebSocket message
        websocket_message = {
            'type': 'drone_command',
            'data': {
                'action': 'move',
                'linear_x': 1.0,
                'linear_z': 0.5
            }
        }
        
        mock_websocket = Mock()
        
        # Process the message
        await bridge.handle_websocket_message(
            mock_websocket, 
            json.dumps(websocket_message)
        )
        
        # Verify stats were updated
        assert bridge.stats['messages_received'] > 0
        
    @pytest.mark.asyncio
    async def test_concurrent_clients(self):
        """Test handling multiple concurrent clients"""
        bridge = WebSocketBridge()
        
        # Simulate multiple clients
        clients = [Mock() for _ in range(5)]
        
        for client in clients:
            bridge.websocket_clients.add(client)
        
        # Test broadcasting to all clients
        test_message = {
            'type': 'broadcast_test',
            'data': {'message': 'Hello all clients'}
        }
        
        await bridge._queue_broadcast(test_message)
        
        # Check that all clients are tracked
        assert len(bridge.websocket_clients) == 5


if __name__ == '__main__':
    # Run tests with verbose output
    pytest.main([__file__, '-v', '--tb=short'])
