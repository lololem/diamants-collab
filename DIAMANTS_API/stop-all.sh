#!/bin/bash

# � DIAMANTS - Stop All Services Script
# ======================================
# Stops all DIAMANTS services gracefully

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo -e "${BLUE}🚁 DIAMANTS - Stopping All Services${NC}"
echo -e "${BLUE}====================================${NC}"
echo ""

# Function to stop service by PID
stop_service() {
    local service_name="$1"
    local pid_file="$2"
    
    if [ -f "$pid_file" ]; then
        local pid=$(cat "$pid_file")
        if kill -0 "$pid" 2>/dev/null; then
            echo -e "${YELLOW}🛑 Stopping $service_name (PID: $pid)...${NC}"
            kill -TERM "$pid"
            sleep 2
            
            # Force kill if still running
            if kill -0 "$pid" 2>/dev/null; then
                echo -e "${RED}💀 Force killing $service_name...${NC}"
                kill -KILL "$pid"
            fi
            
            echo -e "${GREEN}✅ $service_name stopped${NC}"
        else
            echo -e "${YELLOW}⚠️  $service_name process not running (PID: $pid)${NC}"
        fi
        rm -f "$pid_file"
    else
        echo -e "${YELLOW}⚠️  No PID file found for $service_name${NC}"
    fi
}

# Stop API
stop_service "DIAMANTS API" "$SCRIPT_DIR/.api.pid"

# Stop Frontend
stop_service "Frontend" "$SCRIPT_DIR/.frontend.pid"

# Stop Backend
stop_service "Backend" "$SCRIPT_DIR/.backend.pid"

# Stop TMUX session
echo -e "${YELLOW}🛑 Stopping TMUX session 'slam_collab'...${NC}"
if tmux has-session -t "slam_collab" 2>/dev/null; then
    tmux kill-session -t "slam_collab"
    echo -e "${GREEN}✅ TMUX session stopped${NC}"
else
    echo -e "${YELLOW}⚠️  TMUX session 'slam_collab' not found${NC}"
fi

# Kill any remaining ROS2 processes
echo -e "${YELLOW}🛑 Stopping ROS2 processes...${NC}"
if pgrep -f "ros2" > /dev/null; then
    pkill -f "ros2"
    echo -e "${GREEN}✅ ROS2 processes stopped${NC}"
else
    echo -e "${YELLOW}⚠️  No ROS2 processes found${NC}"
fi

# Kill any remaining Gazebo processes
echo -e "${YELLOW}🛑 Stopping Gazebo simulation...${NC}"
if pgrep -f "gz sim\|gazebo" > /dev/null; then
    pkill -f "gz sim\|gazebo"
    echo -e "${GREEN}✅ Gazebo stopped${NC}"
else
    echo -e "${YELLOW}⚠️  Gazebo not running${NC}"
fi

# Kill any remaining Python processes related to DIAMANTS
echo -e "${YELLOW}🛑 Stopping remaining Python services...${NC}"
pkill -f "launcher.py" 2>/dev/null && echo -e "${GREEN}✅ Launcher stopped${NC}"
pkill -f "websocket_bridge.py" 2>/dev/null && echo -e "${GREEN}✅ WebSocket bridge stopped${NC}"

# Kill processes on specific ports
echo -e "${YELLOW}🛑 Freeing ports...${NC}"
for port in 8000 8765 5550 5551 5552 5553 5554 5555; do
    local pid=$(lsof -ti:$port 2>/dev/null)
    if [ ! -z "$pid" ]; then
        kill -TERM "$pid" 2>/dev/null
        echo -e "${GREEN}✅ Port $port freed${NC}"
    fi
done

# Clean up temporary files
echo -e "${YELLOW}🧹 Cleaning up temporary files...${NC}"
rm -f "$SCRIPT_DIR"/.*.pid
rm -f /tmp/diamants_*
echo -e "${GREEN}✅ Cleanup completed${NC}"

echo ""
echo -e "${GREEN}🎉 All DIAMANTS services stopped successfully!${NC}"
echo ""
echo -e "${BLUE}📝 Next steps:${NC}"
echo -e "   • Check status: ./DIAMANTS_API/status.sh"
echo -e "   • Restart all: ./DIAMANTS_API/restart.sh"
echo -e "   • Quick setup: ./DIAMANTS_API/quick-setup.sh"

# Stop Frontend
echo -e "${YELLOW}Stopping Frontend...${NC}"
if [ -f "$SCRIPT_DIR/.frontend.pid" ]; then
    FRONTEND_PID=$(cat "$SCRIPT_DIR/.frontend.pid")
    if kill -0 $FRONTEND_PID 2>/dev/null; then
        kill $FRONTEND_PID
        echo -e "${GREEN}✅ Frontend stopped (PID: $FRONTEND_PID)${NC}"
    else
        echo -e "${YELLOW}⚠️  Frontend PID not active${NC}"
    fi
    rm -f "$SCRIPT_DIR/.frontend.pid"
else
    echo -e "${BLUE}ℹ️  No frontend PID file found${NC}"
fi

# Kill any remaining frontend processes
pkill -f "vite.*5550\|vite.*5551\|dev.js\|npm.*dev" 2>/dev/null || true
echo -e "${GREEN}✅ Frontend processes cleaned${NC}"

# Stop Backend TMUX session
echo -e "${YELLOW}Stopping Backend...${NC}"
if tmux has-session -t "slam_collab" 2>/dev/null; then
    tmux kill-session -t "slam_collab"
    echo -e "${GREEN}✅ Backend TMUX session stopped${NC}"
else
    echo -e "${BLUE}ℹ️  No TMUX session 'slam_collab' found${NC}"
fi

# Stop backend using Makefile if available
if [ -f "$BACKEND_DIR/Makefile" ]; then
    cd "$BACKEND_DIR"
    make kill-tmux 2>/dev/null || true
    echo -e "${GREEN}✅ Backend Makefile cleanup executed${NC}"
fi

# Kill any remaining backend processes
echo -e "${YELLOW}Cleaning remaining processes...${NC}"
pkill -f "ros2" 2>/dev/null || true
pkill -f "gz sim\|gazebo" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "ros_gz_bridge" 2>/dev/null || true

# Clean up PID files
rm -f "$SCRIPT_DIR/.backend.pid"
rm -f "$SCRIPT_DIR/.frontend.pid"

# Wait a moment for processes to stop
sleep 2

echo ""
echo -e "${GREEN}✅ All DIAMANTS Services Stopped!${NC}"
echo -e "${GREEN}================================${NC}"

# Check if anything is still running
REMAINING_PROCESSES=$(pgrep -f "vite\|ros2\|gazebo\|rviz" | wc -l)
if [ $REMAINING_PROCESSES -gt 0 ]; then
    echo -e "${YELLOW}⚠️  $REMAINING_PROCESSES processes may still be running${NC}"
    echo -e "${BLUE}ℹ️  Use 'ps aux | grep -E \"vite|ros2|gazebo|rviz\"' to check${NC}"
else
    echo -e "${GREEN}✅ All processes stopped cleanly${NC}"
fi

echo ""
echo -e "${BLUE}🚀 Quick Restart:${NC}"
echo -e "   • Restart all: ./DIAMANTS_API/restart.sh"
echo -e "   • Start all: ./DIAMANTS_API/start-all.sh"
echo -e "   • Check status: ./DIAMANTS_API/status.sh"
