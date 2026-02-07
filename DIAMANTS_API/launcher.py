#!/usr/bin/env python3
"""
DIAMANTS API - Launcher
========================
Starts the REST API (FastAPI) and the unified WebSocket bridge.
"""

import uvicorn
import asyncio
import os
import sys
import logging

# Ensure imports work from this directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

logger = logging.getLogger("diamants.launcher")

# --- Conditional ROS2 init ---
ROS2_AVAILABLE = False
try:
    import rclpy
    rclpy.init()
    ROS2_AVAILABLE = True
    logger.info("ROS2 initialized successfully")
except ImportError:
    logger.warning("ROS2 not available — running in standalone API mode")
except Exception as e:
    logger.warning(f"ROS2 init failed: {e} — running in standalone API mode")

from api.main import app
from services.websocket_bridge import DiamantsBridge


async def run_api_server():
    """Run the FastAPI server."""
    config = uvicorn.Config(
        app=app,
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level="info",
    )
    server = uvicorn.Server(config)
    await server.serve()


async def start_all():
    """Start both the API and the WebSocket bridge concurrently."""
    print("=" * 60)
    print("  DIAMANTS Platform Starting")
    print(f"  REST API  : http://0.0.0.0:8000")
    print(f"  WebSocket : ws://0.0.0.0:8765")
    print(f"  ROS2      : {'connected' if ROS2_AVAILABLE else 'standalone mode'}")
    print("=" * 60)

    bridge = DiamantsBridge(ws_host="0.0.0.0", ws_port=8765)

    # Start WebSocket server
    await bridge.start_server()

    # Run API (blocking)
    await run_api_server()


if __name__ == "__main__":
    try:
        asyncio.run(start_all())
    except KeyboardInterrupt:
        print("\nDIAMANTS Platform stopped.")
    finally:
        if ROS2_AVAILABLE:
            rclpy.shutdown()
