#!/bin/bash

# 🚁 DIAMANTS - Orchestrateur TMUX - SLAM Collaboratif
# =====================================================
# Orchestration complète du workflow slam_collaboratif multi-agent (8 drones, mapping, fusion, RViz)
# Chaque composant dans un terminal dédié, monitoring et validation visuelle utilisateur
#
# 🎛️ CONTRÔLE RVIZ :
# - Par défaut : RViz ACTIVÉ automatiquement (fenêtre 6)
# - Gazebo : Instance unique (fenêtre 1)
# - Pour désactiver RViz : Commenter la ligne dans fenêtre 6
# - Tous les launch files respectent les paramètres rviz:=false / enable_rviz:=false (évite duplications)

# 🧹 NETTOYAGE PRÉALABLE ACTIVÉ POUR RELANCEMENT PROPRE
echo "🧹 Nettoyage préalable activé - Élimination des processus existants"

# Configuration des chemins relatifs - fonctionne depuis n'importe où
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS2_WS="$BACKEND_ROOT/slam_collaboratif/ros2_ws"
SCRIPTS_DIR="$BACKEND_ROOT/scripts"
CONFIG_DIR="$BACKEND_ROOT/config"

# Vérification de l'existence des répertoires
if [ ! -d "$ROS2_WS" ]; then
    echo "❌ ERREUR: Workspace ROS2 introuvable à $ROS2_WS"
    exit 1
fi

# === FONCTIONS ===
function log_session() {
    echo "📋 $1"
    echo "$(date): $1" >> "$JOURNAL_FILE"
}

function check_tmux_session() {
    tmux has-session -t "$1" 2>/dev/null
}

# Journal
JOURNAL_FILE="$BACKEND_ROOT/logs/diamants_tmux_slam_collab_journal.log"
mkdir -p "$BACKEND_ROOT/logs"
echo "$(date): Début orchestration TMUX SLAM collaboratif" > "$JOURNAL_FILE"

# 🧹 NETTOYAGE PROCESSUS SLAM_COLLABORATIF PRÉCÉDENTS
log_session "Nettoyage processus slam_collaboratif précédents"
pkill -f "gz sim.*slam_collaboratif" 2>/dev/null || true
pkill -f "ros_gz_bridge" 2>/dev/null || true
pkill -f "map_merger_node" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 2

# Vérification de l'existence des répertoires
if [ ! -d "$ROS2_WS" ]; then
    echo "❌ ERREUR: Workspace ROS2 non trouvé: $ROS2_WS"
    exit 1
fi

# Correction PYTHONPATH pour forcer l'utilisation du transforms3d patché
export PYTHONPATH="$BACKEND_ROOT/src/third_party:$PYTHONPATH"

JOURNAL_FILE="$BACKEND_ROOT/logs/diamants_tmux_slam_collab_journal.log"
mkdir -p "$BACKEND_ROOT/logs"
echo "$(date): Début orchestration TMUX SLAM collaboratif" >"$JOURNAL_FILE"

log_session() {
    echo "$(date): $1" >>"$JOURNAL_FILE"
    echo "📋 $1"
}

check_tmux_session() {
    local session_name=$1
    if tmux has-session -t "$session_name" 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

# Nettoyage des processus précédents
log_session "Nettoyage processus slam_collaboratif précédents"
pkill -f "gz sim.*slam_collaboratif" 2>/dev/null || true
pkill -f "ros_gz_bridge" 2>/dev/null || true
pkill -f "map_merger_node" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 2

# Nettoyage session existante
if check_tmux_session "slam_collab"; then
    log_session "Session slam_collab existe - nettoyage des processus"
    tmux kill-session -t "slam_collab" 2>/dev/null || true
    sleep 1
fi

# Création session tmux dédiée
log_session "Création session slam_collab détachée"
tmux new-session -d -s "slam_collab" -c "$BACKEND_ROOT"

# Fenêtres dédiées
log_session "Création fenêtres spécialisées"
tmux new-window -t "slam_collab" -c "$BACKEND_ROOT" # 1: Simulation Gazebo
for i in {2..9}; do tmux new-window -t "slam_collab" -c "$BACKEND_ROOT"; done

# Lancement Simulation Gazebo UNIQUE (fenêtre 1) - UNE SEULE INSTANCE
log_session "Lancement simulation Gazebo UNIQUE multi-drones dans TMUX"
export GZ_SIM_RESOURCE_PATH="$ROS2_WS/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models:$ROS2_WS/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/worlds"
tmux send-keys -t "slam_collab:1" "export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH" Enter
tmux send-keys -t "slam_collab:1" "gz sim $ROS2_WS/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/worlds/crazyflie_multi_world.sdf -r" Enter
sleep 5

# Bridge ROS2-Gazebo (fenêtre 2) - Bridge configuré pour multi-drones
log_session "Bridge ROS2-Gazebo avec configuration multi-drones (fenêtre 2)"
tmux send-keys -t "slam_collab:2" "cd $ROS2_WS && source install/setup.bash" Enter
tmux send-keys -t "slam_collab:2" "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=src/ros_gz_crazyflie/ros_gz_crazyflie_bringup/config/ros_gz_multi_crazyflie_bridge.yaml" Enter
sleep 3

# Mapping multi-robot (fenêtre 3) - rviz:=false seulement
log_session "Lancement mapping multi-robot dans TMUX (rviz:=false)"
tmux send-keys -t "slam_collab:3" "cd $ROS2_WS && source install/setup.bash" Enter
tmux send-keys -t "slam_collab:3" "ros2 launch slam_map_merge multi_agent_diamants.launch.py rviz:=false" Enter
sleep 3

# Agents d'exploration (fenêtre 4) - gazebo_launch:=false rviz:=false 
log_session "Lancement agents d'exploration dans TMUX (gazebo_launch:=false rviz:=false)"
tmux send-keys -t "slam_collab:4" "cd $ROS2_WS && source install/setup.bash" Enter
tmux send-keys -t "slam_collab:4" "ros2 launch multi_agent_framework DIAMANTS_BACKEND_complete.launch.py gazebo_launch:=false rviz:=false" Enter
sleep 3

# SLAM nodes (fenêtre 5) - CORRECTION: Lancement manuel des nodes SLAM pour TOUS LES 8 DRONES
log_session "Lancement nodes SLAM corrigés pour les 8 drones (simple_mapper + wall_following)"
tmux send-keys -t "slam_collab:5" "cd $ROS2_WS && source install/setup.bash" Enter
tmux send-keys -t "slam_collab:5" "echo '🚀 Lancement nodes SLAM pour 8 drones...'" Enter

# Lancer les nodes pour chaque drone
tmux send-keys -t "slam_collab:5" "echo '🤖 Drone crazyflie...'" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_simple_mapper simple_mapper_multiranger --ros-args -p robot_prefix:=crazyflie -p use_sim_time:=true &" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_wall_following wall_following_multicrazy_multiranger --ros-args -p robot_prefix:=crazyflie -p use_sim_time:=true -p delay:=3.0 -p mode:=scout -p start_direction:=right &" Enter

tmux send-keys -t "slam_collab:5" "echo '🤖 Drone crazyflie1...'" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_simple_mapper simple_mapper_multiranger --ros-args -p robot_prefix:=crazyflie1 -p use_sim_time:=true &" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_wall_following wall_following_multicrazy_multiranger --ros-args -p robot_prefix:=crazyflie1 -p use_sim_time:=true -p delay:=4.0 -p mode:=scout -p start_direction:=left &" Enter

tmux send-keys -t "slam_collab:5" "echo '🤖 Drone crazyflie2...'" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_simple_mapper simple_mapper_multiranger --ros-args -p robot_prefix:=crazyflie2 -p use_sim_time:=true &" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_wall_following wall_following_multicrazy_multiranger --ros-args -p robot_prefix:=crazyflie2 -p use_sim_time:=true -p delay:=5.0 -p mode:=scout -p start_direction:=right &" Enter

tmux send-keys -t "slam_collab:5" "echo '🤖 Drone crazyflie3...'" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_simple_mapper simple_mapper_multiranger --ros-args -p robot_prefix:=crazyflie3 -p use_sim_time:=true &" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_wall_following wall_following_multicrazy_multiranger --ros-args -p robot_prefix:=crazyflie3 -p use_sim_time:=true -p delay:=6.0 -p mode:=scout -p start_direction:=left &" Enter

tmux send-keys -t "slam_collab:5" "echo '🤖 Drone crazyflie4...'" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_simple_mapper simple_mapper_multiranger --ros-args -p robot_prefix:=crazyflie4 -p use_sim_time:=true &" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_wall_following wall_following_multicrazy_multiranger --ros-args -p robot_prefix:=crazyflie4 -p use_sim_time:=true -p delay:=7.0 -p mode:=scout -p start_direction:=right &" Enter

tmux send-keys -t "slam_collab:5" "echo '🤖 Drone crazyflie5...'" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_simple_mapper simple_mapper_multiranger --ros-args -p robot_prefix:=crazyflie5 -p use_sim_time:=true &" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_wall_following wall_following_multicrazy_multiranger --ros-args -p robot_prefix:=crazyflie5 -p use_sim_time:=true -p delay:=8.0 -p mode:=scout -p start_direction:=left &" Enter

tmux send-keys -t "slam_collab:5" "echo '🤖 Drone crazyflie6...'" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_simple_mapper simple_mapper_multiranger --ros-args -p robot_prefix:=crazyflie6 -p use_sim_time:=true &" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_wall_following wall_following_multicrazy_multiranger --ros-args -p robot_prefix:=crazyflie6 -p use_sim_time:=true -p delay:=9.0 -p mode:=scout -p start_direction:=right &" Enter

tmux send-keys -t "slam_collab:5" "echo '🤖 Drone crazyflie7...'" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_simple_mapper simple_mapper_multiranger --ros-args -p robot_prefix:=crazyflie7 -p use_sim_time:=true &" Enter
tmux send-keys -t "slam_collab:5" "ros2 run crazyflie_ros2_multiranger_wall_following wall_following_multicrazy_multiranger --ros-args -p robot_prefix:=crazyflie7 -p use_sim_time:=true -p delay:=10.0 -p mode:=scout -p start_direction:=left &" Enter

tmux send-keys -t "slam_collab:5" "echo '✅ Tous les nodes SLAM lancés pour 8 drones avec délais échelonnés'" Enter
tmux send-keys -t "slam_collab:5" "wait" Enter
sleep 8

# Coverage Image Publisher (fenêtre 6) - COORDINATEUR POSITIONS INTER-DRONES
log_session "COORDINATEUR POSITIONS INTER-DRONES - Communication anti-collision"
tmux send-keys -t "slam_collab:6" "cd $ROS2_WS && source install/setup.bash" Enter
tmux send-keys -t "slam_collab:6" "echo '🌐 Lancement Coordinateur Positions Inter-Drones...'" Enter
tmux send-keys -t "slam_collab:6" "ros2 run crazyflie_ros2_multiranger_wall_following drone_position_coordinator &" Enter
tmux send-keys -t "slam_collab:6" "echo '✅ Coordinateur actif - Communication inter-drones pour évitement collision'" Enter
sleep 3

# RViz AUTOMATIQUE (fenêtre 7) - Configuration DIAMANTS directe 
log_session "Lancement RViz automatique avec configuration DIAMANTS directe"
tmux new-window -t "slam_collab" -c "$BACKEND_ROOT"
tmux send-keys -t "slam_collab:7" "cd $ROS2_WS && source install/setup.bash" Enter
if [ -f "$CONFIG_DIR/rviz_config_slam_optimized.rviz" ]; then
    tmux send-keys -t "slam_collab:7" "rviz2 -d $CONFIG_DIR/rviz_config_slam_optimized.rviz" Enter
else
    tmux send-keys -t "slam_collab:7" "rviz2" Enter
    echo "⚠️  Fichier de config RViz non trouvé, utilisation configuration par défaut"
fi
sleep 3

# Monitoring (fenêtre 8)
log_session "Monitoring avancé dans TMUX"
tmux new-window -t "slam_collab" -c "$BACKEND_ROOT"
tmux send-keys -t "slam_collab:8" "cd $BACKEND_ROOT" Enter
tmux send-keys -t "slam_collab:8" "echo '📊 === FENÊTRE MONITORING SLAM ==='" Enter
tmux send-keys -t "slam_collab:8" "echo '▶️  Diagnostic ponctuel: ./scripts/diagnostic_slam_status.sh'" Enter
tmux send-keys -t "slam_collab:8" "echo '▶️  Diagnostic complet: python3 scripts/diagnostic_slam_system.py'" Enter
tmux send-keys -t "slam_collab:8" "echo '▶️  Monitoring continu: watch -n 10 \"./scripts/diagnostic_slam_status.sh\"'" Enter
tmux send-keys -t "slam_collab:8" "echo ''" Enter
tmux send-keys -t "slam_collab:8" "echo '🎯 Lancement diagnostic initial...'" Enter
if [ -f "$SCRIPTS_DIR/diagnostic_slam_status.sh" ]; then
    tmux send-keys -t "slam_collab:8" "$SCRIPTS_DIR/diagnostic_slam_status.sh" Enter
else
    tmux send-keys -t "slam_collab:8" "echo '⚠️  Script diagnostic non trouvé'" Enter
fi

# Logs live (fenêtre 9)
log_session "Affichage logs (fenêtre 9)"
tmux new-window -t "slam_collab" -c "$BACKEND_ROOT"
tmux send-keys -t "slam_collab:9" "tail -f $JOURNAL_FILE" Enter

# Terminal secours (fenêtre 10)
log_session "Terminal secours prêt (fenêtre 10)"
tmux send-keys -t "slam_collab:9" "echo 'Terminal secours prêt - aucune duplication'" Enter

# Affichage du journal et instructions
cat "$JOURNAL_FILE"
echo ""
echo "🎮 === CONTRÔLES TMUX SLAM COLLABORATIF ==="
echo "   tmux list-sessions"
echo "   tmux attach -t slam_collab"
echo "   tmux list-windows -t slam_collab"
echo "   tmux capture-pane -t slam_collab:1 -p    # Voir sortie simulation"
echo "   tmux kill-session -t slam_collab    # Arrêter session slam_collab"
echo ""
echo "✨ Orchestration TMUX slam_collaboratif prête - 10 terminaux dédiés - Zéro blocage !"
echo "📋 Journal: $JOURNAL_FILE"
