"""
Shared pytest fixtures for DIAMANTS API tests.

Handles ROS2 (rclpy) init/shutdown so bridge tests work even
when a full ROS2 environment is available.
"""

import sys
import os
import pytest

# Ensure the API package root is importable
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── ROS2 lifecycle ──────────────────────────────────────────────────────────
_rclpy_initialized = False

try:
    import rclpy

    @pytest.fixture(autouse=True, scope="session")
    def _rclpy_session():
        """Init rclpy once per session so DiamantsBridge(Node) can instantiate."""
        global _rclpy_initialized
        if not _rclpy_initialized:
            rclpy.init()
            _rclpy_initialized = True
        yield
        try:
            rclpy.shutdown()
        except Exception:
            pass

except ImportError:
    # rclpy not installed — stub Node is used automatically
    pass
