#!/bin/bash
#
# DIAMANTS - Unified System Launcher
# ====================================
# Starts the entire DIAMANTS collaborative SLAM system:
#
# 1. ROS2 Microservices (Gazebo headless + SwarmController + SLAM + etc.)
# 2. API Gateway (FastAPI REST + WebSocket Bridge)
# 3. Frontend (Vite dev server)
#
# Usage:
#   ./launch_diamants_full.sh                   # Full system
#   ./launch_diamants_full.sh --no-frontend     # Backend only
#   ./launch_diamants_full.sh --no-gazebo       # Without Gazebo (API only)
#   ./launch_diamants_full.sh --gui             # With Gazebo GUI
#   ./launch_diamants_full.sh --drones 4        # Custom drone count
#
# Stop:
#   ./stop_diamants.sh
#   or Ctrl+C (kills all child processes)
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/log"
mkdir -p "$LOG_DIR"

# =========================================================================
# Configuration
# =========================================================================
USE_FRONTEND=true
USE_GAZEBO=true
HEADLESS=true
NUM_DRONES=6
AUTO_START=true

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-frontend)  USE_FRONTEND=false; shift ;;
        --no-gazebo)    USE_GAZEBO=false;   shift ;;
        --gui)          HEADLESS=false;     shift ;;
        --drones)       NUM_DRONES="$2";    shift 2 ;;
        --no-autostart) AUTO_START=false;   shift ;;
        -h|--help)
            echo "Usage: $0 [--no-frontend] [--no-gazebo] [--gui] [--drones N] [--no-autostart]"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# =========================================================================
# Cleanup function
# =========================================================================
PIDS=()

cleanup() {
    echo ""
    echo "═══════════════════════════════════════════════════"
    echo "  DIAMANTS — Shutting down all services..."
    echo "═══════════════════════════════════════════════════"
    
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "  Stopping PID $pid..."
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    
    # Kill any remaining Gazebo processes
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby.*gz" 2>/dev/null || true
    
    # Wait for cleanup
    sleep 2
    
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    
    echo "  All services stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# =========================================================================
# Environment
# =========================================================================
echo "═══════════════════════════════════════════════════"
echo "  DIAMANTS - Multi-Agent SLAM Collaborative System"
echo "═══════════════════════════════════════════════════"
echo ""
echo "  Configuration:"
echo "    Drones:    $NUM_DRONES"
echo "    Gazebo:    $USE_GAZEBO (headless=$HEADLESS)"
echo "    Frontend:  $USE_FRONTEND"
echo "    Auto-start: $AUTO_START"
echo ""

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "  ✅ ROS2 Jazzy sourced"
else
    echo "  ❌ ROS2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    exit 1
fi

# Source workspace overlay if built
WS_SETUP="$SCRIPT_DIR/DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
    echo "  ✅ ROS2 workspace overlay sourced"
fi

# Add Gazebo to path if needed
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$SCRIPT_DIR/DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models"

# Ensure gz is in PATH
if ! command -v gz &>/dev/null; then
    if [ -f /usr/bin/gz ]; then
        export PATH="/usr/bin:$PATH"
    fi
fi

echo ""

# =========================================================================
# Step 1: Stop any existing DIAMANTS processes
# =========================================================================
echo "  🔄 Stopping any existing DIAMANTS processes..."
pkill -f "diamants_swarm_controller" 2>/dev/null || true
pkill -f "diamants_position_broadcaster" 2>/dev/null || true
pkill -f "diamants_slam_fusion" 2>/dev/null || true
pkill -f "diamants_mission_coordinator" 2>/dev/null || true
pkill -f "launcher.py" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
sleep 1

# =========================================================================
# Step 2: Start API Gateway (FastAPI + WebSocket Bridge)
# =========================================================================
echo "  🚀 Starting API Gateway (port 8000 + WS 8765)..."
cd "$SCRIPT_DIR/DIAMANTS_API"
python3 launcher.py > "$LOG_DIR/api_gateway.log" 2>&1 &
PIDS+=($!)
echo "     PID: ${PIDS[-1]}"
sleep 2

# =========================================================================
# Step 3: Start ROS2 Microservices
# =========================================================================
if [ "$USE_GAZEBO" = true ]; then
    echo "  🚀 Starting ROS2 Microservices (Gazebo + 4 nodes)..."
    
    # Check if the package is built
    MICROSERVICES_PKG="$SCRIPT_DIR/DIAMANTS_BACKEND/ros2_microservices"
    
    if ! ros2 pkg list 2>/dev/null | grep -q "diamants_microservices"; then
        echo "     ⚠️  Package not built yet. Running microservices directly..."
        
        # Run each node as a standalone Python script with ROS2
        echo "     Starting SwarmController..."
        python3 "$MICROSERVICES_PKG/diamants_microservices/swarm_controller.py" \
            --ros-args -p num_drones:=$NUM_DRONES -p auto_start:=$AUTO_START \
            > "$LOG_DIR/swarm_controller.log" 2>&1 &
        PIDS+=($!)
        
        echo "     Starting PositionBroadcaster..."
        python3 "$MICROSERVICES_PKG/diamants_microservices/position_broadcaster.py" \
            --ros-args -p num_drones:=$NUM_DRONES \
            > "$LOG_DIR/position_broadcaster.log" 2>&1 &
        PIDS+=($!)
        
        echo "     Starting SLAMFusion..."
        python3 "$MICROSERVICES_PKG/diamants_microservices/slam_fusion.py" \
            --ros-args -p num_drones:=$NUM_DRONES \
            > "$LOG_DIR/slam_fusion.log" 2>&1 &
        PIDS+=($!)
        
        echo "     Starting MissionCoordinator..."
        python3 "$MICROSERVICES_PKG/diamants_microservices/mission_coordinator.py" \
            --ros-args -p num_drones:=$NUM_DRONES -p auto_start:=$AUTO_START \
            > "$LOG_DIR/mission_coordinator.log" 2>&1 &
        PIDS+=($!)
    else
        # Use the built package with ros2 launch
        ros2 launch diamants_microservices diamants_full.launch.py \
            headless:=$HEADLESS \
            num_drones:=$NUM_DRONES \
            auto_start:=$AUTO_START \
            > "$LOG_DIR/ros2_launch.log" 2>&1 &
        PIDS+=($!)
    fi
    
    echo "     PIDs: ${PIDS[@]:1}"
    sleep 3
else
    echo "  ⏭️  Skipping Gazebo/ROS2 (--no-gazebo mode)"
fi

# =========================================================================
# Step 4: Start Frontend
# =========================================================================
if [ "$USE_FRONTEND" = true ]; then
    FRONTEND_DIR="$SCRIPT_DIR/DIAMANTS_FRONTEND/Mission_system"
    if [ -d "$FRONTEND_DIR" ]; then
        echo "  🚀 Starting Frontend dev server..."
        cd "$FRONTEND_DIR"
        
        # Check if node_modules exist
        if [ ! -d "node_modules" ] && [ -f "package.json" ]; then
            echo "     Installing dependencies..."
            npm install > "$LOG_DIR/npm_install.log" 2>&1
        fi
        
        # Start Vite dev server
        if [ -f "package.json" ]; then
            npx vite --host > "$LOG_DIR/frontend.log" 2>&1 &
        else
            # Simple Python HTTP server fallback
            python3 -m http.server 3000 > "$LOG_DIR/frontend.log" 2>&1 &
        fi
        PIDS+=($!)
        echo "     PID: ${PIDS[-1]}"
    else
        echo "  ⚠️  Frontend directory not found: $FRONTEND_DIR"
    fi
fi

# =========================================================================
# Status Summary
# =========================================================================
echo ""
echo "═══════════════════════════════════════════════════"
echo "  DIAMANTS System Running"
echo "═══════════════════════════════════════════════════"
echo ""
echo "  Services:"
echo "    📡 API REST:    http://localhost:8000"
echo "    🔌 WebSocket:   ws://localhost:8765"
if [ "$USE_FRONTEND" = true ]; then
echo "    🌐 Frontend:    http://localhost:3000"
fi
if [ "$USE_GAZEBO" = true ]; then
echo "    🤖 ROS2 Nodes:  $NUM_DRONES drones"
echo "    🗺️  SLAM:        Collaborative map fusion"
fi
echo ""
echo "  Logs: $LOG_DIR/"
echo ""
echo "  Press Ctrl+C to stop all services"
echo "═══════════════════════════════════════════════════"
echo ""

# =========================================================================
# Wait for all processes
# =========================================================================
cd "$SCRIPT_DIR"
wait
