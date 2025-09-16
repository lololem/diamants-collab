#!/bin/bash

# 🚁 DIAMANTS - Arrêt Principal
# =============================
# Script d'arrêt principal pour DIAMANTS

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}🚁 DIAMANTS - Arrêt des Services${NC}"
echo -e "${BLUE}================================${NC}"

# Stop launcher processes
echo -e "${BLUE}🛑 Arrêt des processus DIAMANTS...${NC}"
pkill -f "python launcher.py" 2>/dev/null || echo -e "${YELLOW}⚠️  Aucun processus launcher trouvé${NC}"

# Stop WebSocket processes
pkill -f "websocket" 2>/dev/null || echo -e "${YELLOW}⚠️  Aucun processus WebSocket trouvé${NC}"

# Free ports
echo -e "${BLUE}🧹 Libération des ports...${NC}"
for port in 8000 8765 9001; do
    lsof -ti:$port | xargs kill -9 2>/dev/null || true
done

# Stop TMUX sessions
tmux kill-session -t slam_collab 2>/dev/null || echo -e "${YELLOW}⚠️  Session TMUX 'slam_collab' non trouvée${NC}"

echo -e "${GREEN}✅ DIAMANTS arrêté avec succès!${NC}"
echo ""
echo -e "${BLUE}📝 Actions rapides:${NC}"
echo -e "   • Redémarrer: ./start.sh"
echo -e "   • Vérifier: ./status.sh"
echo -e "   • Configuration: ./scripts/setup/setup-dependencies.sh"
