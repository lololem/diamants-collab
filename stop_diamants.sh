#!/bin/bash

# 🚁 DIAMANTS - Script d'arrêt complet
# ====================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[$(date +'%H:%M:%S')]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

echo -e "${RED}"
echo "🛑 ========================================"
echo "   DIAMANTS - ARRÊT COMPLET DU SYSTÈME"
echo "========================================${NC}"
echo ""

log "🔍 Recherche des processus DIAMANTS..."

# Arrêt des PIDs sauvegardés
if [ -f /tmp/diamants_api.pid ]; then
    PID=$(cat /tmp/diamants_api.pid)
    log "🔄 Arrêt API (PID: $PID)..."
    kill -9 $PID 2>/dev/null || true
    rm -f /tmp/diamants_api.pid
fi

if [ -f /tmp/diamants_frontend.pid ]; then
    PID=$(cat /tmp/diamants_frontend.pid)
    log "🔄 Arrêt Frontend (PID: $PID)..."
    kill -9 $PID 2>/dev/null || true
    rm -f /tmp/diamants_frontend.pid
fi

if [ -f /tmp/diamants_backend.pid ]; then
    PID=$(cat /tmp/diamants_backend.pid)
    log "🔄 Arrêt Backend (PID: $PID)..."
    kill -9 $PID 2>/dev/null || true
    rm -f /tmp/diamants_backend.pid
fi

# Arrêt des sessions TMUX
log "🔄 Nettoyage sessions TMUX..."
tmux kill-session -t slam_collab 2>/dev/null && log "✅ Session TMUX slam_collab fermée" || log_warn "Session TMUX slam_collab déjà fermée"

# Nettoyage robuste des processus ROS2/Gazebo fantômes
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$PROJECT_ROOT/DIAMANTS_BACKEND/kill_ros_gazebo.sh" ]; then
    log "🧹 Nettoyage robuste ROS2/Gazebo..."
    "$PROJECT_ROOT/DIAMANTS_BACKEND/kill_ros_gazebo.sh" > /dev/null 2>&1
fi

# Libération des ports
log "🔄 Libération des ports..."
for port in 8000 8765 5173; do
    PID=$(lsof -ti:$port 2>/dev/null || true)
    if [ ! -z "$PID" ]; then
        log "🔄 Libération port $port (PID: $PID)..."
        kill -9 $PID 2>/dev/null || true
    fi
done

# Nettoyage processus par nom
log "🔄 Nettoyage processus par nom..."
pkill -f "launcher.py" 2>/dev/null || true
pkill -f "npm run dev" 2>/dev/null || true
pkill -f "vite" 2>/dev/null || true
pkill -f "slam_collaboratif" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true         # ← NOUVEAU : Gazebo moderne
pkill -f "gz server" 2>/dev/null || true      # ← NOUVEAU : Serveur Gazebo moderne

log "✅ Nettoyage terminé"

# Vérification finale
echo ""
log "📊 Vérification finale..."
ACTIVE_PORTS=$(netstat -tuln 2>/dev/null | grep -E ":(8000|8765|5173)" | wc -l)
if [ $ACTIVE_PORTS -eq 0 ]; then
    log "✅ Tous les ports DIAMANTS sont libres"
else
    log_warn "Ports encore actifs:"
    netstat -tuln 2>/dev/null | grep -E ":(8000|8765|5173)"
fi

echo ""
log "🎯 DIAMANTS complètement arrêté !"
echo ""
