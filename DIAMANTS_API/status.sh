#!/bin/bash

# 🚁 DIAMANTS - Status Check Script
# =================================
# Checks the status of all DIAMANTS services

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo -e "${BLUE}🚁 DIAMANTS System Status${NC}"
echo -e "${BLUE}=========================${NC}"
echo ""

# Function to check service status
check_service() {
    local service_name="$1"
    local port="$2"
    local protocol="${3:-http}"
    
    if [ "$protocol" = "http" ]; then
        if curl -s "http://localhost:$port" > /dev/null 2>&1; then
            echo -e "${GREEN}✅ $service_name${NC} - Running on port $port"
            return 0
        else
            echo -e "${RED}❌ $service_name${NC} - Not responding on port $port"
            return 1
        fi
    elif [ "$protocol" = "websocket" ]; then
        if nc -z localhost "$port" 2>/dev/null; then
            echo -e "${GREEN}✅ $service_name${NC} - WebSocket listening on port $port"
            return 0
        else
            echo -e "${RED}❌ $service_name${NC} - WebSocket not listening on port $port"
            return 1
        fi
    fi
}

# Check API
echo -e "${BLUE}📡 API Services:${NC}"
check_service "DIAMANTS API" "8000"
check_service "WebSocket Bridge" "8765" "websocket"
echo ""

# Check Frontend
echo -e "${BLUE}🖥️  Frontend Services:${NC}"
frontend_found=false
for port in 5550 5551 5552 5553 5554 5555; do
    if curl -s "http://localhost:$port" > /dev/null 2>&1; then
        echo -e "${GREEN}✅ Frontend${NC} - Running on port $port"
        frontend_found=true
        break
    fi
done

if [ "$frontend_found" = false ]; then
    echo -e "${RED}❌ Frontend${NC} - Not found on common ports (5550-5555)"
fi
echo ""

# Check Backend/TMUX
echo -e "${BLUE}🤖 Backend Services:${NC}"
if tmux has-session -t "slam_collab" 2>/dev/null; then
    echo -e "${GREEN}✅ Backend SLAM${NC} - TMUX session 'slam_collab' active"
    echo -e "   • Session windows: $(tmux list-windows -t slam_collab 2>/dev/null | wc -l)"
else
    echo -e "${RED}❌ Backend SLAM${NC} - TMUX session 'slam_collab' not found"
fi

# Check ROS2 processes
if pgrep -f "ros2" > /dev/null; then
    echo -e "${GREEN}✅ ROS2 Processes${NC} - Active"
else
    echo -e "${YELLOW}⚠️  ROS2 Processes${NC} - None detected"
fi

# Check Gazebo
if pgrep -f "gz sim\|gazebo" > /dev/null; then
    echo -e "${GREEN}✅ Gazebo Simulation${NC} - Running"
else
    echo -e "${RED}❌ Gazebo Simulation${NC} - Not running"
fi
echo ""

# Check Process PIDs
echo -e "${BLUE}📋 Process Information:${NC}"
if [ -f "$SCRIPT_DIR/.api.pid" ]; then
    api_pid=$(cat "$SCRIPT_DIR/.api.pid")
    if kill -0 "$api_pid" 2>/dev/null; then
        echo -e "${GREEN}✅ API Process${NC} - PID: $api_pid"
    else
        echo -e "${RED}❌ API Process${NC} - PID $api_pid not running"
    fi
fi

if [ -f "$SCRIPT_DIR/.frontend.pid" ]; then
    frontend_pid=$(cat "$SCRIPT_DIR/.frontend.pid")
    if kill -0 "$frontend_pid" 2>/dev/null; then
        echo -e "${GREEN}✅ Frontend Process${NC} - PID: $frontend_pid"
    else
        echo -e "${RED}❌ Frontend Process${NC} - PID $frontend_pid not running"
    fi
fi

if [ -f "$SCRIPT_DIR/.backend.pid" ]; then
    backend_pid=$(cat "$SCRIPT_DIR/.backend.pid")
    if kill -0 "$backend_pid" 2>/dev/null; then
        echo -e "${GREEN}✅ Backend Process${NC} - PID: $backend_pid"
    else
        echo -e "${RED}❌ Backend Process${NC} - PID $backend_pid not running"
    fi
fi
echo ""

# System Resources
echo -e "${BLUE}💻 System Resources:${NC}"
echo -e "   • CPU Usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
echo -e "   • Memory: $(free -h | awk '/^Mem:/ {print $3 "/" $2}')"
echo -e "   • Disk: $(df -h / | awk 'NR==2 {print $3 "/" $2 " (" $5 " used)"}')"
echo ""

# Quick Actions
echo -e "${BLUE}🎮 Quick Actions:${NC}"
echo -e "   • Restart all: ./DIAMANTS_API/restart.sh"
echo -e "   • Stop all: ./DIAMANTS_API/stop-all.sh"
echo -e "   • View API logs: curl http://localhost:8000/api/status"
echo -e "   • Attach to backend: tmux attach -t slam_collab"
echo ""

# Health Score
services_count=0
healthy_count=0

# Count services
if curl -s "http://localhost:8000" > /dev/null 2>&1; then healthy_count=$((healthy_count+1)); fi
services_count=$((services_count+1))

if nc -z localhost 8765 2>/dev/null; then healthy_count=$((healthy_count+1)); fi
services_count=$((services_count+1))

for port in 5550 5551 5552 5553 5554 5555; do
    if curl -s "http://localhost:$port" > /dev/null 2>&1; then
        healthy_count=$((healthy_count+1))
        break
    fi
done
services_count=$((services_count+1))

if tmux has-session -t "slam_collab" 2>/dev/null; then healthy_count=$((healthy_count+1)); fi
services_count=$((services_count+1))

health_percentage=$((healthy_count * 100 / services_count))

if [ $health_percentage -eq 100 ]; then
    echo -e "${GREEN}🎉 System Health: $health_percentage% - All systems operational!${NC}"
elif [ $health_percentage -ge 80 ]; then
    echo -e "${YELLOW}⚡ System Health: $health_percentage% - Most systems operational${NC}"
else
    echo -e "${RED}🚨 System Health: $health_percentage% - Multiple issues detected${NC}"
fi
BLUE='\033[0;34m'
NC='\033[0m'

# Project paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo -e "${BLUE}🎮 DIAMANTS Services Status${NC}"
echo -e "${BLUE}===========================${NC}"
echo ""

# Check Frontend
echo -e "${BLUE}📱 Frontend Status:${NC}"
if curl -s -I http://localhost:5550/ | head -n 1 | grep -q "200\|404"; then
    echo -e "   ${GREEN}✅ Frontend: RUNNING on http://localhost:5550${NC}"
elif curl -s -I http://localhost:5551/ | head -n 1 | grep -q "200\|404"; then
    echo -e "   ${GREEN}✅ Frontend: RUNNING on http://localhost:5551${NC}"
else
    # Check other common ports
    FRONTEND_PORT=""
    for port in {5550..5559}; do
        if curl -s -I http://localhost:$port/ | head -n 1 | grep -q "200\|404"; then
            FRONTEND_PORT=$port
            break
        fi
    done
    
    if [ ! -z "$FRONTEND_PORT" ]; then
        echo -e "   ${GREEN}✅ Frontend: RUNNING on http://localhost:$FRONTEND_PORT${NC}"
    else
        echo -e "   ${RED}❌ Frontend: NOT RUNNING${NC}"
    fi
fi

# Check Frontend process
if pgrep -f "vite.*5550\|vite.*5551\|dev.js" > /dev/null; then
    echo -e "   ${GREEN}✅ Frontend process: ACTIVE${NC}"
else
    echo -e "   ${RED}❌ Frontend process: NOT FOUND${NC}"
fi

echo ""

# Check Backend TMUX session
echo -e "${BLUE}🚁 Backend Status:${NC}"
if tmux has-session -t "slam_collab" 2>/dev/null; then
    echo -e "   ${GREEN}✅ TMUX session 'slam_collab': ACTIVE${NC}"
    
    # Count tmux windows
    WINDOWS=$(tmux list-windows -t slam_collab 2>/dev/null | wc -l)
    echo -e "   ${GREEN}✅ TMUX windows: $WINDOWS active${NC}"
else
    echo -e "   ${RED}❌ TMUX session 'slam_collab': NOT FOUND${NC}"
fi

# Check ROS2 processes
echo -e "${BLUE}🤖 ROS2 Processes:${NC}"
ROS2_COUNT=$(pgrep -f "ros2" | wc -l)
if [ $ROS2_COUNT -gt 0 ]; then
    echo -e "   ${GREEN}✅ ROS2 processes: $ROS2_COUNT running${NC}"
else
    echo -e "   ${RED}❌ ROS2 processes: NONE${NC}"
fi

# Check Gazebo
if pgrep -f "gz sim\|gazebo" > /dev/null; then
    echo -e "   ${GREEN}✅ Gazebo simulation: RUNNING${NC}"
else
    echo -e "   ${RED}❌ Gazebo simulation: NOT RUNNING${NC}"
fi

# Check RViz
if pgrep -f "rviz2" > /dev/null; then
    echo -e "   ${GREEN}✅ RViz visualization: RUNNING${NC}"
else
    echo -e "   ${RED}❌ RViz visualization: NOT RUNNING${NC}"
fi

echo ""

# Check system resources
echo -e "${BLUE}💻 System Resources:${NC}"
MEMORY_USAGE=$(free | grep '^Mem:' | awk '{printf "%.1f", $3/$2 * 100.0}')
echo -e "   ${BLUE}ℹ️  Memory usage: ${MEMORY_USAGE}%${NC}"

CPU_LOAD=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')
echo -e "   ${BLUE}ℹ️  CPU load: ${CPU_LOAD}${NC}"

# Check disk space
DISK_USAGE=$(df . | tail -1 | awk '{print $5}' | sed 's/%//')
if [ $DISK_USAGE -gt 90 ]; then
    echo -e "   ${RED}⚠️  Disk usage: ${DISK_USAGE}% (LOW SPACE!)${NC}"
else
    echo -e "   ${BLUE}ℹ️  Disk usage: ${DISK_USAGE}%${NC}"
fi

echo ""

# Check ports
echo -e "${BLUE}🔌 Network Ports:${NC}"
ACTIVE_PORTS=$(ss -tuln | grep -E ':(5550|5551|8080|11311)' | wc -l)
if [ $ACTIVE_PORTS -gt 0 ]; then
    echo -e "   ${GREEN}✅ Active DIAMANTS ports: $ACTIVE_PORTS${NC}"
    ss -tuln | grep -E ':(5550|5551|8080|11311)' | while read line; do
        echo -e "   ${BLUE}   $line${NC}"
    done
else
    echo -e "   ${YELLOW}⚠️  No DIAMANTS ports active${NC}"
fi

echo ""

# Quick actions
echo -e "${BLUE}🛠️  Quick Actions:${NC}"
echo -e "   • Start all: ./DIAMANTS_API/start-all.sh"
echo -e "   • Stop all: ./DIAMANTS_API/stop-all.sh"
echo -e "   • Restart: ./DIAMANTS_API/restart.sh"
echo -e "   • Frontend only: ./DIAMANTS_API/start-frontend.sh"
echo -e "   • Backend only: ./DIAMANTS_API/start-backend.sh"
echo -e "   • Attach TMUX: tmux attach -t slam_collab"
