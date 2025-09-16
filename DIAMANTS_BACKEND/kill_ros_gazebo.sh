#!/bin/bash

# Script de nettoyage robuste des processus ROS2/Gazebo fantômes
# Utilisé avant de lancer le backend SLAM pour éviter les conflits

echo "🧹 Nettoyage des processus ROS2/Gazebo fantômes..."

# Fonction pour tuer les processus par nom avec vérification
kill_processes_by_name() {
    local process_name="$1"
    local pids=$(pgrep -f "$process_name" 2>/dev/null)
    
    if [ ! -z "$pids" ]; then
        echo "  🔫 Arrêt des processus $process_name: $pids"
        echo "$pids" | xargs kill -TERM 2>/dev/null || true
        sleep 2
        
        # Force kill si toujours actifs
        local remaining_pids=$(pgrep -f "$process_name" 2>/dev/null)
        if [ ! -z "$remaining_pids" ]; then
            echo "  💀 Force kill des processus $process_name restants: $remaining_pids"
            echo "$remaining_pids" | xargs kill -9 2>/dev/null || true
        fi
    else
        echo "  ✅ Aucun processus $process_name trouvé"
    fi
}

# 1. Arrêt des sessions TMUX liées au SLAM (sélectif)
echo "🎯 Nettoyage des sessions TMUX..."
tmux list-sessions 2>/dev/null | grep -E "(slam|gazebo|ros)" | cut -d: -f1 | while read session; do
    echo "  🔫 Arrêt session TMUX: $session"
    tmux kill-session -t "$session" 2>/dev/null || true
done

# 2. Arrêt des processus Gazebo
echo "🎯 Nettoyage Gazebo..."
kill_processes_by_name "gazebo"
kill_processes_by_name "gzserver"
kill_processes_by_name "gzclient"
kill_processes_by_name "gzmaster"
kill_processes_by_name "gz sim"        # ← NOUVEAU : Gazebo Garden/Harmonic
kill_processes_by_name "gz server"     # ← NOUVEAU : Serveur Gazebo moderne

# 3. Arrêt des processus ROS2
echo "🎯 Nettoyage ROS2..."
kill_processes_by_name "ros2"
kill_processes_by_name "_ros2_daemon"
kill_processes_by_name "rviz2"
kill_processes_by_name "rqt"

# 4. Arrêt des nœuds spécifiques SLAM
echo "🎯 Nettoyage nœuds SLAM..."
kill_processes_by_name "slam_toolbox"
kill_processes_by_name "nav2"
kill_processes_by_name "crazyflie"
kill_processes_by_name "simple_mapper"

# 5. Nettoyage des processus Python liés
echo "🎯 Nettoyage processus Python ROS..."
kill_processes_by_name "python.*ros"
kill_processes_by_name "python.*slam"

# 6. Nettoyage des middlewares DDS
echo "🎯 Nettoyage middleware DDS..."
kill_processes_by_name "cyclonedx"
kill_processes_by_name "fastrtps"

# 7. Nettoyage des ports ROS2 standards
echo "🎯 Libération des ports ROS2..."
for port in 7400 7401 7402 7403 11311 11345; do
    lsof -ti:$port | xargs kill -9 2>/dev/null || true
done

# 8. Nettoyage des fichiers temporaires ROS2
echo "🎯 Nettoyage fichiers temporaires..."
rm -rf /tmp/.ros* 2>/dev/null || true
rm -rf ~/.ros/log/* 2>/dev/null || true

# 9. Reset des variables d'environnement ROS2
echo "🎯 Reset environnement ROS2..."
unset ROS_DOMAIN_ID
unset ROS_NAMESPACE
unset ROS_LOCALHOST_ONLY
unset GAZEBO_MODEL_PATH
unset GAZEBO_PLUGIN_PATH

# 10. Attente et vérification finale
echo "⏳ Attente stabilisation (3s)..."
sleep 3

# Vérification finale
remaining_processes=$(pgrep -f "(gazebo|ros2|slam)" 2>/dev/null | wc -l)
if [ "$remaining_processes" -eq 0 ]; then
    echo "✅ Tous les processus ROS2/Gazebo nettoyés avec succès"
else
    echo "⚠️  Il reste $remaining_processes processus actifs"
    echo "Processus restants:"
    pgrep -f "(gazebo|ros2|slam)" 2>/dev/null | xargs ps -p 2>/dev/null || true
fi

echo "🚁 Environnement ROS2/Gazebo nettoyé - Prêt pour le backend SLAM"
