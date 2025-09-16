#!/bin/bash

# 🚁 DIAMANTS - Collaborative SLAM Launcher
# ============================================
# Main launcher for collaborative SLAM system
# Integrates frontend and backend communication

# Note: set -e removed to handle cleanup errors gracefully

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project paths
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKEND_ROOT="$PROJECT_ROOT/DIAMANTS_BACKEND"
SLAM_WS="$BACKEND_ROOT/slam_collaboratif/ros2_ws"
FRONTEND_ROOT="$PROJECT_ROOT/DIAMANTS_FRONTEND/Mission_system"

echo -e "${BLUE}🚁 DIAMANTS - Collaborative SLAM System${NC}"
echo -e "${BLUE}===========================================${NC}"

# Clean any existing ROS2/Gazebo processes first (seulement si pas déjà fait)
if [ -z "$DIAMANTS_SKIP_CLEANUP" ]; then
    echo -e "${YELLOW}🧹 Nettoyage des processus ROS2/Gazebo fantômes...${NC}"
    if [ -f "$BACKEND_ROOT/kill_ros_gazebo.sh" ]; then
        "$BACKEND_ROOT/kill_ros_gazebo.sh"
    else
        echo -e "${YELLOW}⚠️  Script de nettoyage non trouvé - nettoyage basique...${NC}"
        pkill -f gazebo 2>/dev/null || true
        pkill -f "gz sim" 2>/dev/null || true      # ← NOUVEAU : Gazebo moderne
        pkill -f ros2 2>/dev/null || true
        tmux kill-server 2>/dev/null || true
    fi
else
    echo -e "${GREEN}✅ Nettoyage sauté - variable DIAMANTS_SKIP_CLEANUP définie${NC}"
fi

# Function to check if ROS2 is sourced
check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}❌ ROS2 not sourced. Please run: source /opt/ros/humble/setup.bash${NC}"
        exit 1
    fi
    echo -e "${GREEN}✅ ROS2 $ROS_DISTRO environment detected${NC}"
}

# Function to build workspace if needed
build_workspace() {
    echo -e "${YELLOW}🔧 Building ROS2 workspace...${NC}"
    cd "$SLAM_WS"
    
    if [ ! -d "build" ]; then
        echo -e "${BLUE}📦 First time build - this may take a while...${NC}"
    fi
    
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Workspace built successfully${NC}"
    else
        echo -e "${RED}❌ Build failed${NC}"
        exit 1
    fi
}

# Function to source workspace
source_workspace() {
    echo -e "${YELLOW}🔗 Sourcing workspace...${NC}"
    cd "$SLAM_WS"
    source install/setup.bash
    echo -e "${GREEN}✅ Workspace sourced${NC}"
}

# Function to launch system components
launch_system() {
    echo -e "${YELLOW}🚀 Launching collaborative SLAM system...${NC}"
    
    # Option 1: Full TMUX orchestration
    if [ "$1" = "tmux" ]; then
        echo -e "${BLUE}📺 Starting TMUX orchestrated session...${NC}"
        bash "$BACKEND_ROOT/scripts/launch/orchestrate_tmux_slam_collaboratif_fixed.sh"
        return
    fi
    
    # Option 2: Simple web interface only
    if [ "$1" = "web" ]; then
        echo -e "${BLUE}🌐 Starting web interface only...${NC}"
        cd "$SLAM_WS"
        ros2 launch multi_agent_framework web_interface.launch.py
        return
    fi
    
    # Option 3: SLAM only
    if [ "$1" = "slam" ]; then
        echo -e "${BLUE}🗺️ Starting SLAM system only...${NC}"
        cd "$SLAM_WS"
        ros2 launch diamants_missions run_mission.launch.py mission:=mission_exploration_simple
        return
    fi
    
    # Default: Interactive menu
    show_menu
}

# Interactive menu
show_menu() {
    echo ""
    echo -e "${YELLOW}📋 Select launch mode:${NC}"
    echo "1) Full system with TMUX orchestration (recommended)"
    echo "2) Web interface only"
    echo "3) SLAM system only"
    echo "4) Frontend development server"
    echo "5) Exit"
    echo ""
    read -p "Enter choice [1-5]: " choice
    
    case $choice in
        1)
            launch_system tmux
            ;;
        2)
            launch_system web
            ;;
        3)
            launch_system slam
            ;;
        4)
            echo -e "${BLUE}🖥️ Starting frontend development server...${NC}"
            cd "$FRONTEND_ROOT"
            npm run dev
            ;;
        5)
            echo -e "${GREEN}👋 Goodbye!${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}❌ Invalid choice${NC}"
            show_menu
            ;;
    esac
}

# Main execution
main() {
    check_ros2
    
    # Check if workspace exists
    if [ ! -d "$SLAM_WS" ]; then
        echo -e "${RED}❌ SLAM workspace not found at $SLAM_WS${NC}"
        exit 1
    fi
    
    build_workspace
    source_workspace
    
    # Parse command line arguments
    if [ $# -eq 0 ]; then
        launch_system
    else
        launch_system "$1"
    fi
}

# Help function
show_help() {
    echo "Usage: $0 [mode]"
    echo ""
    echo "Modes:"
    echo "  tmux    - Full system with TMUX orchestration"
    echo "  web     - Web interface only"
    echo "  slam    - SLAM system only"
    echo "  help    - Show this help"
    echo ""
    echo "Examples:"
    echo "  $0           # Interactive menu"
    echo "  $0 tmux      # Full orchestrated system"
    echo "  $0 web       # Web interface only"
}

# Handle help
if [ "$1" = "help" ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_help
    exit 0
fi

# Run main function
main "$@"
