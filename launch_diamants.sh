#!/bin/bash

# 🚁 DIAMANTS - Lanceur Master Clone & Run
# =========================================
# Script principal pour démarrer l'écosystème DIAMANTS complet
# Frontend + Backend ROS2 + API WebSocket

set -e

# Désactivation automatique de l'environnement virtuel Python
if [ -n "$VIRTUAL_ENV" ]; then
    echo "[INFO] Désactivation de l'environnement virtuel Python..."
    deactivate 2>/dev/null || true
fi

# Sourcing automatique de ROS2 Jazzy pour environnement système
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0
fi

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIAMANTS_ROOT="$SCRIPT_DIR"
API_DIR="$DIAMANTS_ROOT/DIAMANTS_API"
FRONTEND_DIR="$DIAMANTS_ROOT/DIAMANTS_FRONTEND/Mission_system"
BACKEND_DIR="$DIAMANTS_ROOT/DIAMANTS_BACKEND"

# Couleurs pour logs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Fonction de log
log() {
    echo -e "${GREEN}[$(date +'%H:%M:%S')]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_info() {
    echo -e "${CYAN}[INFO]${NC} $1"
}

# Détection du mode (express vs interactif)
# run.sh = toujours en mode express pour lancement complet automatique
EXPRESS_MODE=true
if [ "$(basename "$0")" = "launch_diamants.sh" ] && [ "$1" != "--express" ] && [ "$1" != "-e" ] && [ "$1" != "auto" ]; then
    EXPRESS_MODE=false
fi

if [ "$EXPRESS_MODE" = true ]; then
    echo -e "${PURPLE}"
    echo "🚁 ==============================================="
    echo "   DIAMANTS - CLONE & RUN EXPRESS"
    echo "   🚀 Démarrage automatique complet"
    echo "===============================================${NC}"
    echo ""
else
    # Banner normal
    echo -e "${PURPLE}"
    echo "🚁 ==============================================="
    echo "   DIAMANTS - CLONE & RUN MASTER LAUNCHER"
    echo "   Collaborative Drone Swarm Platform" 
    echo "===============================================${NC}"
    echo ""
fi

# Vérification des prérequis
check_prerequisites() {
    log "🔍 Vérification des prérequis..."
    
    # Désactivation de l'environnement virtuel Python s'il est actif
    if [ ! -z "$VIRTUAL_ENV" ]; then
        log_info "Désactivation de l'environnement virtuel Python..."
        deactivate 2>/dev/null || true
    fi
    
    # Source ROS2 automatiquement si nécessaire
    if [ -z "$ROS_DISTRO" ] && [ -f "/opt/ros/jazzy/setup.bash" ]; then
        log_info "Chargement de l'environnement ROS2 Jazzy..."
        source /opt/ros/jazzy/setup.bash
        export ROS_DOMAIN_ID=0
    fi
    
    # ROS2 Jazzy
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2 non trouvé. Installez ROS2 Jazzy d'abord."
        exit 1
    fi
    
    # Node.js
    if ! command -v node &> /dev/null; then
        log_error "Node.js non trouvé. Installez Node.js 16+ d'abord."
        exit 1
    fi
    
    # Python3
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 non trouvé."
        exit 1
    fi
    
    # TMUX
    if ! command -v tmux &> /dev/null; then
        log_warn "TMUX non trouvé. Installation recommandée pour le backend."
    fi
    
    log "✅ Prérequis validés"
}

# Installation/Setup automatique
setup_components() {
    log "🔧 Setup automatique des composants..."
    
    # Nettoyage initial des processus fantômes
    cleanup_simulation_ghosts
    
    # Setup API
    log_info "Configuration DIAMANTS_API..."
    cd "$API_DIR"
    
    # Installation des dépendances Python directement dans l'environnement système
    if [ -f "requirements.txt" ]; then
        # Vérification si les packages sont déjà installés
        if ! python3 -c "import fastapi, uvicorn, websockets" 2>/dev/null; then
            log_info "Installation des dépendances Python système..."
            pip3 install -r requirements.txt --user
            log "✅ Dépendances Python API installées"
        else
            log "✅ Dépendances Python API existantes"
        fi
    else
        log "✅ Configuration API validée"
    fi
    
    # Setup Frontend
    log_info "Configuration DIAMANTS_FRONTEND..."
    cd "$FRONTEND_DIR"
    if [ ! -d "node_modules" ]; then
        npm install
        log "✅ Dépendances Node.js installées"
    else
        log "✅ Dépendances Node.js existantes"
    fi
    
    # Setup Backend ROS2
    log_info "Configuration DIAMANTS_BACKEND..."
    cd "$BACKEND_DIR"
    if [ ! -d "slam_collaboratif/ros2_ws/install" ]; then
        log_warn "Workspace ROS2 non compilé. Le backend le construira au premier lancement."
    else
        log "✅ Workspace ROS2 existant"
    fi
}

# Nettoyage des ports
# Nettoyage des processus fantômes de simulation
cleanup_simulation_ghosts() {
    log "🧹 Nettoyage systématique des simulations Gazebo/RViz..."
    
    # Compter les instances actives
    GAZEBO_COUNT=$(pgrep -f "gz sim\|gazebo" | wc -l)
    RVIZ_COUNT=$(pgrep -f "rviz" | wc -l)
    
    log_info "💡 Détection: $GAZEBO_COUNT simulations Gazebo, $RVIZ_COUNT instances RViz"
    
    # Toujours nettoyer si il y a des processus, même un seul 
    if [ "$GAZEBO_COUNT" -gt 0 ] || [ "$RVIZ_COUNT" -gt 0 ]; then
        log_info "🔄 Nettoyage en cours (sans sudo)..."
        
        # Gazebo (toutes versions) - seulement les processus utilisateur
        pkill -9 -f "gazebo" 2>/dev/null || true
        pkill -9 -f "gz sim" 2>/dev/null || true      # ← Gazebo moderne
        pkill -9 -f "gz server" 2>/dev/null || true   # ← Serveur Gazebo moderne  
        pkill -9 -f "gzserver" 2>/dev/null || true
        pkill -9 -f "gzclient" 2>/dev/null || true
        
        # RViz (toutes versions)
        pkill -9 -f "rviz" 2>/dev/null || true
        pkill -9 -f "rviz2" 2>/dev/null || true
        
        # Sessions TMUX potentiellement problématiques
        tmux kill-session -t slam_collab 2>/dev/null || true
        tmux kill-session -t simulation 2>/dev/null || true
        
        # Nettoyage fichiers temporaires (seulement ceux de l'utilisateur)
        rm -rf /tmp/gazebo_* 2>/dev/null || true
        rm -rf /tmp/diamants_*.pid 2>/dev/null || true
        
        sleep 2  # ← Attendre que les processus se terminent vraiment
        
        log "✅ Nettoyage Gazebo/RViz terminé (mode utilisateur)"
    else
        log_info "✅ Aucune simulation active - pas de nettoyage nécessaire"
    fi
}

cleanup_ports() {
    log "🧹 Nettoyage léger des ports et sessions anciennes..."
    
    # Nettoyage session TMUX slam ancienne seulement 
    tmux kill-session -t slam_collab 2>/dev/null || true
    
    # Port 8000 (API)
    lsof -ti:8000 | xargs kill -9 2>/dev/null || true
    
    # Port 8765 (WebSocket)
    lsof -ti:8765 | xargs kill -9 2>/dev/null || true
    
    # Port 5173 (Frontend Vite)
    lsof -ti:5173 | xargs kill -9 2>/dev/null || true
    
    log "✅ Ports libérés"
}

# Lancement API + WebSocket
launch_api() {
    log "🚀 Lancement DIAMANTS_API + WebSocket Service..."
    cd "$API_DIR"
    
    # Lancement en arrière-plan
    nohup bash -c "
        # Désactivation de tout environnement virtuel
        deactivate 2>/dev/null || true
        unset VIRTUAL_ENV
        # Environnement ROS2 uniquement
        source /opt/ros/jazzy/setup.bash
        export ROS_DOMAIN_ID=0
        export PYTHONPATH=\"/opt/ros/jazzy/lib/python3.12/site-packages:\$PYTHONPATH\"
        python3 launcher.py
    " > logs/api.log 2>&1 &
    
    API_PID=$!
    echo $API_PID > /tmp/diamants_api.pid
    
    # Attente du démarrage
    log_info "Attente du démarrage de l'API..."
    for i in {1..30}; do
        if curl -s http://localhost:8000/api/status > /dev/null 2>&1; then
            log "✅ API démarrée (PID: $API_PID)"
            break
        fi
        sleep 1
    done
}

# Lancement Frontend
launch_frontend() {
    log "🎮 Lancement DIAMANTS_FRONTEND..."
    cd "$FRONTEND_DIR"
    
    # Lancement en arrière-plan
    nohup npm run dev > ../logs/frontend.log 2>&1 &
    
    FRONTEND_PID=$!
    echo $FRONTEND_PID > /tmp/diamants_frontend.pid
    
    # Attente du démarrage
    log_info "Attente du démarrage du frontend..."
    for i in {1..20}; do
        if curl -s http://localhost:5173 > /dev/null 2>&1; then
            log "✅ Frontend démarré (PID: $FRONTEND_PID)"
            break
        fi
        sleep 1
    done
}

# Lancement Backend
launch_backend() {
    log "🤖 Lancement DIAMANTS_BACKEND SLAM..."
    
    # Nettoyage préventif avant le lancement
    cleanup_simulation_ghosts
    
    cd "$BACKEND_DIR"
    
    # Lancement simple et direct
    export DIAMANTS_SKIP_CLEANUP=1
    (
        trap '' INT TERM  # Ignore les signaux pendant le lancement
        echo "1" | ./launch_slam_collaborative.sh > logs/backend.log 2>&1
    ) &
    
    BACKEND_PID=$!
    echo $BACKEND_PID > /tmp/diamants_backend.pid
    
    # Attente simplifiée
    log_info "Attente du démarrage de la session TMUX SLAM..."
    sleep 5
    
    if tmux has-session -t slam_collab 2>/dev/null; then
        WINDOWS=$(tmux list-windows -t slam_collab 2>/dev/null | wc -l)
        log "✅ Backend SLAM démarré (PID: $BACKEND_PID, $WINDOWS fenêtres TMUX)"
        log_info "Le backend utilise TMUX - utilisez 'tmux attach -t slam_collab' pour voir"
    else
        log_warn "La session TMUX slam_collab n'est pas encore complète. Vérifiez les logs backend."
    fi
}

# Fonction d'arrêt propre
cleanup() {
    # Éviter les boucles infinies lors de multiples signaux
    if [ "${CLEANUP_RUNNING:-}" = "true" ]; then
        exit 1
    fi
    export CLEANUP_RUNNING=true
    
    log "🛑 Arrêt de DIAMANTS..."
    
    # Arrêt des services
    if [ -f /tmp/diamants_api.pid ]; then
        kill -9 $(cat /tmp/diamants_api.pid) 2>/dev/null || true
        rm -f /tmp/diamants_api.pid
    fi
    
    if [ -f /tmp/diamants_frontend.pid ]; then
        kill -9 $(cat /tmp/diamants_frontend.pid) 2>/dev/null || true
        rm -f /tmp/diamants_frontend.pid
    fi
    
    if [ -f /tmp/diamants_backend.pid ]; then
        kill -9 $(cat /tmp/diamants_backend.pid) 2>/dev/null || true
        rm -f /tmp/diamants_backend.pid
    fi
    
    # Nettoyage TMUX
    tmux kill-session -t slam_collab 2>/dev/null || true
    
    cleanup_ports
    log "✅ DIAMANTS arrêté"
    
    # Sortie explicite
    trap - EXIT INT TERM
    exit 0
}

# Gestion des signaux
trap cleanup EXIT INT TERM

# Menu principal
show_menu() {
    echo ""
    echo -e "${BLUE}📋 MENU DIAMANTS${NC}"
    echo "1) 🚀 Lancement complet (API + Frontend + Backend)"
    echo "2) 📡 API seulement"
    echo "3) 🎮 Frontend seulement" 
    echo "4) 🤖 Backend seulement"
    echo "5) 🔧 Setup/Installation seulement"
    echo "6) 📊 Status des services"
    echo "7) 🛑 Arrêter tout"
    echo "8) ❌ Quitter"
    echo ""
}

# Status des services
show_status() {
    log "📊 Status DIAMANTS..."
    
    echo ""
    echo -e "${CYAN}🌐 Services Web:${NC}"
    if curl -s http://localhost:8000/api/status > /dev/null 2>&1; then
        echo "  ✅ API REST: http://localhost:8000"
    else
        echo "  ❌ API REST: Arrêtée"
    fi
    
    if curl -s http://localhost:5173 > /dev/null 2>&1; then
        echo "  ✅ Frontend: http://localhost:5173"
    else
        echo "  ❌ Frontend: Arrêté"
    fi
    
    echo ""
    echo -e "${CYAN}🤖 Backend ROS2:${NC}"
    if tmux has-session -t slam_collab 2>/dev/null; then
        echo "  ✅ SLAM Collaboratif: Active (TMUX)"
        echo "  📋 Windows: $(tmux list-windows -t slam_collab | wc -l)"
    else
        echo "  ❌ SLAM Collaboratif: Arrêté"
    fi
    
    echo ""
    echo -e "${CYAN}📡 Ports:${NC}"
    netstat -tuln | grep -E ":(8000|8765|5173)" || echo "  ℹ️  Aucun port DIAMANTS actif"
    echo ""
}

# Programme principal
main() {
    check_prerequisites
    
    # Mode express : lancement automatique complet
    if [ "$EXPRESS_MODE" = true ] || [ "$1" == "full" ]; then
        setup_components
        cleanup_ports
        
        # Nettoyage préventif ROS2/Gazebo AVANT lancement backend
        if [ -f "$DIAMANTS_ROOT/DIAMANTS_BACKEND/kill_ros_gazebo.sh" ]; then
            log_info "🧹 Nettoyage initial processus ROS2/Gazebo..."
            "$DIAMANTS_ROOT/DIAMANTS_BACKEND/kill_ros_gazebo.sh" > /dev/null 2>&1 || true
        fi
        
        launch_api
        launch_frontend
        launch_backend
        show_status
        
        echo ""
        log "🎉 DIAMANTS démarré avec succès !"
        echo -e "${GREEN}📡 API & WebSocket:${NC} http://localhost:8000"
        echo -e "${GREEN}🎮 Frontend:${NC} http://localhost:5173"
        echo -e "${GREEN}🤖 Backend SLAM:${NC} tmux attach -t slam_collab"
        echo ""
        echo "Appuyez sur Ctrl+C pour arrêter tous les services"
        
        # Boucle d'attente
        while true; do
            sleep 10
            # Vérification que les services sont toujours actifs
            if ! curl -s http://localhost:8000/api/status > /dev/null 2>&1; then
                log_warn "API déconnectée, redémarrage..."
                launch_api
            fi
        done
        
        return
    fi
    
    # Mode interactif
    while true; do
        show_menu
        read -p "Votre choix [1-8]: " choice
        
        case $choice in
            1)
                setup_components
                cleanup_ports
                launch_api
                sleep 3
                launch_frontend
                sleep 3
                launch_backend
                show_status
                log "🎉 DIAMANTS complet démarré !"
                ;;
            2)
                setup_components
                cleanup_ports
                launch_api
                show_status
                ;;
            3)
                setup_components
                launch_frontend
                show_status
                ;;
            4)
                setup_components
                launch_backend
                show_status
                ;;
            5)
                setup_components
                log "✅ Setup terminé"
                ;;
            6)
                show_status
                ;;
            7)
                cleanup
                log "🛑 Tous les services arrêtés"
                ;;
            8)
                log "👋 Au revoir !"
                exit 0
                ;;
            *)
                log_error "Choix invalide"
                ;;
        esac
        
        echo ""
        read -p "Appuyez sur Entrée pour continuer..."
        clear
    done
}

# Création des dossiers de logs
mkdir -p "$API_DIR/logs" "$FRONTEND_DIR/../logs" "$BACKEND_DIR/logs"

# Lancement
main "$@"
