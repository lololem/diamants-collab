#!/usr/bin/env python3
"""
DIAMANTS API - ROS2 Integration Tests
Tests for the unified DiamantsBridge (services.websocket_bridge)
"""

import pytest
import asyncio
import json
import sys
import os
from unittest.mock import Mock

# Add the API directory to the path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from services.websocket_bridge import DiamantsBridge


class TestBridgeInitialization:
    """Test DiamantsBridge initialization (ROS2 stub mode)."""

    @pytest.fixture
    def bridge(self):
        b = DiamantsBridge(ws_host="127.0.0.1", ws_port=0)
        yield b
        try:
            b.destroy_node()
        except Exception:
            pass

    def test_bridge_created(self, bridge):
        assert bridge is not None

    def test_has_clients_set(self, bridge):
        assert hasattr(bridge, "ws_clients")
        assert isinstance(bridge.ws_clients, set)
        assert len(bridge.ws_clients) == 0

    def test_has_state_cache(self, bridge):
        assert "drones" in bridge.state
        assert "swarm" in bridge.state
        assert "mission" in bridge.state
        assert "system" in bridge.state

    def test_state_defaults(self, bridge):
        assert bridge.state["swarm"]["intelligence_score"] == 0.0
        assert bridge.state["swarm"]["coverage_area"] == 0.0
        assert bridge.state["mission"]["status"] == "idle"

    def test_ros2_stub_mode(self, bridge):
        """In CI / without ROS2, the bridge should still initialize."""
        assert bridge.state["system"]["ros2_available"] is not None


class TestROS2Callbacks:
    """Test ROS2 subscriber callbacks using stub messages."""

    @pytest.fixture
    def bridge(self):
        b = DiamantsBridge(ws_host="127.0.0.1", ws_port=0)
        yield b
        try:
            b.destroy_node()
        except Exception:
            pass

    def test_drone_positions_callback(self, bridge):
        """Simulate a drone_positions ROS2 message."""
        mock_msg = Mock()
        mock_msg.data = json.dumps({
            "cf1": {"x": 1.0, "y": 2.0, "z": 0.5},
        })
        bridge._on_drone_positions(mock_msg)
        assert "cf1" in bridge.state["drones"]
        assert bridge.state["drones"]["cf1"]["position"]["x"] == 1.0

    def test_drone_telemetry_callback(self, bridge):
        mock_msg = Mock()
        mock_msg.data = json.dumps({
            "drone_id": "cf1", "battery": 92, "status": "flying",
        })
        bridge._on_drone_telemetry(mock_msg)
        assert bridge.state["drones"]["cf1"]["battery"] == 92

    def test_intelligence_score_callback(self, bridge):
        mock_msg = Mock()
        mock_msg.data = 0.87
        bridge._on_intelligence_score(mock_msg)
        assert bridge.state["swarm"]["intelligence_score"] == 0.87

    def test_coverage_area_callback(self, bridge):
        mock_msg = Mock()
        mock_msg.data = 42.5
        bridge._on_coverage_area(mock_msg)
        assert bridge.state["swarm"]["coverage_area"] == 42.5

    def test_swarm_status_callback(self, bridge):
        mock_msg = Mock()
        mock_msg.data = json.dumps({
            "formation": "v-shape",
            "status": "scouting",
        })
        bridge._on_swarm_status(mock_msg)
        assert bridge.state["swarm"]["formation"] == "v-shape"
        assert bridge.state["swarm"]["status"] == "scouting"

    def test_mission_status_callback(self, bridge):
        mock_msg = Mock()
        mock_msg.data = json.dumps({
            "status": "in_progress",
            "waypoints": [{"x": 1, "y": 2}],
        })
        bridge._on_mission_status(mock_msg)
        assert bridge.state["mission"]["status"] == "in_progress"

    def test_system_status_callback(self, bridge):
        mock_msg = Mock()
        mock_msg.data = json.dumps({
            "simulation_active": True,
            "extra_field": "ignored_safely",
        })
        bridge._on_system_status(mock_msg)
        assert bridge.state["system"]["simulation_active"] is True


class TestCommandHandlers:
    """Test WebSocket → ROS2 command handlers."""

    @pytest.fixture
    def bridge(self):
        b = DiamantsBridge(ws_host="127.0.0.1", ws_port=0)
        yield b
        try:
            b.destroy_node()
        except Exception:
            pass

    @pytest.mark.asyncio
    async def test_cmd_vel_handler(self, bridge):
        """cmd_vel should publish a Twist to ROS2 (stub mode = no-op)."""
        data = {
            "linear": {"x": 1.0, "y": 0.0, "z": 0.5},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.1},
        }
        # Should not raise even in stub mode
        await bridge._handle_cmd_vel(data)

    @pytest.mark.asyncio
    async def test_drone_command_handler(self, bridge):
        data = {"action": "takeoff", "drone_id": "cf1"}
        await bridge._handle_drone_command(data)

    @pytest.mark.asyncio
    async def test_swarm_command_handler(self, bridge):
        data = {"action": "scatter"}
        await bridge._handle_swarm_command(data)

    @pytest.mark.asyncio
    async def test_mission_command_handler(self, bridge):
        data = {"action": "start", "mission_id": "scout_01", "drone_ids": ["cf1", "cf2"]}
        await bridge._handle_mission_command(data)

    @pytest.mark.asyncio
    async def test_parameter_change_handler(self, bridge):
        data = {"name": "max_speed", "value": 2.5}
        await bridge._handle_parameter_change(data)


class TestMessageRouting:
    """Test the _route_message dispatcher."""

    @pytest.fixture
    def bridge(self):
        b = DiamantsBridge(ws_host="127.0.0.1", ws_port=0)
        yield b
        try:
            b.destroy_node()
        except Exception:
            pass

    @pytest.mark.asyncio
    async def test_ping_route(self, bridge):
        mock_ws = Mock()
        future = asyncio.get_event_loop().create_future()
        future.set_result(None)
        mock_ws.send = Mock(return_value=future)
        raw = json.dumps({"type": "ping"})
        await bridge._route_message(mock_ws, "test_client", raw)
        mock_ws.send.assert_called_once()

    @pytest.mark.asyncio
    async def test_get_status_route(self, bridge):
        mock_ws = Mock()
        future = asyncio.get_event_loop().create_future()
        future.set_result(None)
        mock_ws.send = Mock(return_value=future)
        raw = json.dumps({"type": "get_status"})
        await bridge._route_message(mock_ws, "test_client", raw)
        mock_ws.send.assert_called_once()
        sent_data = json.loads(mock_ws.send.call_args[0][0])
        assert sent_data["type"] == "current_status"
        assert "data" in sent_data

    @pytest.mark.asyncio
    async def test_unknown_type_ignored(self, bridge):
        mock_ws = Mock()
        raw = json.dumps({"type": "nonexistent_type_xyz"})
        # Should not raise
        await bridge._route_message(mock_ws, "test_client", raw)

    @pytest.mark.asyncio
    async def test_invalid_json_ignored(self, bridge):
        mock_ws = Mock()
        await bridge._route_message(mock_ws, "test_client", "{ invalid json }")

    @pytest.mark.asyncio
    async def test_subscribe_route(self, bridge):
        mock_ws = Mock()
        raw = json.dumps({"type": "subscribe", "data": {"topics": ["drones", "swarm"]}})
        await bridge._route_message(mock_ws, "test_client", raw)
        assert "drones" in bridge.client_subscriptions.get("test_client", set())
        assert "swarm" in bridge.client_subscriptions.get("test_client", set())


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
