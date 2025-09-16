#!/bin/bash

# 🚁 DIAMANTS - Démarrage Principal
# =================================
# Script de démarrage principal pour DIAMANTS avec ROS2 Jazzy

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Project paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo -e "${BLUE}🚁 DIAMANTS - Démarrage Principal${NC}"
echo -e "${BLUE}=================================${NC}"

# Check ROS2 Jazzy
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo -e "${RED}❌ ROS2 Jazzy non trouvé!${NC}"
    echo -e "${YELLOW}💡 Installez avec: ./scripts/setup/setup-dependencies.sh${NC}"
    exit 1
fi

# Check virtual environment
if [ ! -d "$SCRIPT_DIR/venv" ]; then
    echo -e "${YELLOW}⚠️  Environnement virtuel non trouvé${NC}"
    echo -e "${BLUE}🔧 Configuration de l'environnement...${NC}"
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
    echo -e "${GREEN}✅ Environnement configuré${NC}"
fi

# Source ROS2 environment
echo -e "${BLUE}🤖 Configuration ROS2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"

# Activate Python environment
source venv/bin/activate

# Start DIAMANTS
echo -e "${GREEN}🚀 Démarrage DIAMANTS...${NC}"
echo -e "${BLUE}📡 API: http://localhost:8000${NC}"
echo -e "${BLUE}📚 Docs: http://localhost:8000/docs${NC}"
echo -e "${BLUE}🌐 WebSocket: ws://localhost:8765${NC}"
echo ""

python launcher.py
