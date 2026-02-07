#!/bin/bash

# DIAMANTS - Script de démarrage Bridge WebSocket ROS2
# ===================================================

echo "🚁 Démarrage DIAMANTS Bridge WebSocket..."

# Configuration par défaut
BRIDGE_HOST=${BRIDGE_HOST:-"localhost"}
BRIDGE_PORT=${BRIDGE_PORT:-8765}

# Couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Fonction logs
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Vérifier environnement ROS2
if [ -z "$ROS_DISTRO" ]; then
    log_warn "ROS2 non sourcé, tentative de sourcing automatique..."
    
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
        log_info "ROS2 Jazzy sourcé"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash  
        log_info "ROS2 Humble sourcé"
    else
        log_error "Impossible de trouver ROS2"
        exit 1
    fi
fi

# Vérifier Python et dépendances
if ! python3 -c "import websockets" 2>/dev/null; then
    log_warn "Module websockets manquant, installation..."
    pip3 install websockets
fi

# Vérifier port disponible
if netstat -tuln | grep -q ":$BRIDGE_PORT "; then
    log_error "Port $BRIDGE_PORT déjà utilisé"
    log_info "Arrêt du processus existant..."
    pkill -f "websocket_bridge.py"
    sleep 2
fi

# Démarrer bridge — désormais centralisé dans DIAMANTS_API/services/
log_info "Démarrage bridge sur $BRIDGE_HOST:$BRIDGE_PORT"

SCRIPT_DIR="$(dirname "$0")"
API_DIR="$(realpath "$SCRIPT_DIR/../../../DIAMANTS_API")"

cd "$API_DIR"

python3 -m services.websocket_bridge \
    --host "$BRIDGE_HOST" \
    --port "$BRIDGE_PORT"
