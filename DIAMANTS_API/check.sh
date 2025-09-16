#!/bin/bash

# 🚁 DIAMANTS - Statut Rapide
# ===========================
# Vérification rapide du statut des services

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}🚁 DIAMANTS - Statut Rapide${NC}"
echo -e "${BLUE}===========================${NC}"

# Check API
if curl -s http://localhost:8000/health > /dev/null 2>&1; then
    echo -e "📡 API REST: ${GREEN}✅ Active${NC} (http://localhost:8000)"
    echo -e "📚 Documentation: ${BLUE}http://localhost:8000/docs${NC}"
else
    echo -e "📡 API REST: ${RED}❌ Inactive${NC}"
fi

# Check WebSockets
if nc -z localhost 8765 2>/dev/null; then
    echo -e "🌐 WebSocket Service: ${GREEN}✅ Active${NC} (ws://localhost:8765)"
else
    echo -e "🌐 WebSocket Service: ${RED}❌ Inactive${NC}"
fi

# Check ROS2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    if ros2 node list 2>/dev/null | grep -q "diamant"; then
        echo -e "🤖 ROS2 Jazzy: ${GREEN}✅ Nodes DIAMANTS actifs${NC}"
    else
        echo -e "🤖 ROS2 Jazzy: ${YELLOW}⚠️  Installé mais nodes DIAMANTS inactifs${NC}"
    fi
else
    echo -e "🤖 ROS2 Jazzy: ${RED}❌ Non installé${NC}"
fi

# Check processes
LAUNCHER_PID=$(ps aux | grep -E "python3? launcher.py" | grep -v grep | awk '{print $2}')
if [ ! -z "$LAUNCHER_PID" ]; then
    echo -e "⚙️  Processus: ${GREEN}✅ DIAMANTS actif${NC} (PID: $LAUNCHER_PID)"
else
    echo -e "⚙️  Processus: ${RED}❌ DIAMANTS inactif${NC}"
fi

echo ""
echo -e "${BLUE}📝 Actions rapides:${NC}"
echo -e "   • Démarrer: ./start.sh"
echo -e "   • Arrêter: ./stop.sh"
echo -e "   • Redémarrer: ./stop.sh && ./start.sh"
echo -e "   • Diagnostic: ./scripts/maintenance/diagnose.sh"
