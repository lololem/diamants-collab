#!/bin/bash

# Script de diagnostic des processus ROS2/Gazebo
# Permet de voir l'état avant/après nettoyage

echo "🔍 DIAGNOSTIC PROCESSUS ROS2/GAZEBO"
echo "=================================="

# Function pour compter et afficher les processus
check_processes() {
    local category="$1"
    local pattern="$2"
    local processes=$(pgrep -f "$pattern" 2>/dev/null)
    local count=$(echo "$processes" | grep -v "^$" | wc -l)
    
    if [ $count -gt 0 ]; then
        echo "  🔴 $category: $count processus actifs"
        
        # Affichage détaillé avec distinction système/utilisateur
        echo "$processes" | while read pid; do
            if [ ! -z "$pid" ]; then
                local cmd=$(ps -p $pid -o cmd --no-headers 2>/dev/null | cut -c1-80)
                if echo "$cmd" | grep -q -E "(daemon|rosbridge)"; then
                    echo "    💙 [SYSTÈME] PID $pid: $cmd"
                else
                    echo "    🔴 [UTILISATEUR] PID $pid: $cmd"
                fi
            fi
        done
    else
        echo "  ✅ $category: Aucun processus"
    fi
}

echo ""
echo "📊 État actuel du système:"

# Vérifications principales
check_processes "Gazebo (ancien)" "gazebo"
check_processes "Gazebo (moderne)" "gz sim"           # ← NOUVEAU
check_processes "ROS2" "ros2"
check_processes "SLAM" "slam"
check_processes "Crazyflie" "crazyflie"

# Sessions TMUX
echo ""
echo "📺 Sessions TMUX:"
tmux_sessions=$(tmux list-sessions 2>/dev/null | wc -l)
if [ $tmux_sessions -gt 0 ]; then
    echo "  🔴 $tmux_sessions sessions actives:"
    tmux list-sessions 2>/dev/null | head -5
else
    echo "  ✅ Aucune session TMUX"
fi

# Ports ROS2/Gazebo
echo ""
echo "🔌 Ports ROS2/Gazebo:"
ports_used=0
for port in 7400 7401 7402 7403 11311 11345; do
    if lsof -ti:$port >/dev/null 2>&1; then
        echo "  🔴 Port $port: Utilisé"
        ports_used=$((ports_used + 1))
    fi
done

if [ $ports_used -eq 0 ]; then
    echo "  ✅ Tous les ports ROS2/Gazebo libres"
fi

# Variables d'environnement
echo ""
echo "🌍 Variables ROS2:"
if [ ! -z "$ROS_DOMAIN_ID" ]; then
    echo "  🔴 ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
else
    echo "  ✅ ROS_DOMAIN_ID: Non défini"
fi

if [ ! -z "$ROS_DISTRO" ]; then
    echo "  🔵 ROS_DISTRO: $ROS_DISTRO"
else
    echo "  ⚪ ROS_DISTRO: Non défini"
fi

echo ""
echo "=================================="
# Compter les processus problématiques (non système)
problematic_processes=$(pgrep -f "(gazebo|gz sim|slam_toolbox|nav2|crazyflie)" 2>/dev/null | wc -l)
system_processes=$(pgrep -f "(daemon|rosbridge)" 2>/dev/null | wc -l)

if [ $problematic_processes -eq 0 ]; then
    echo "✅ Système propre - Aucun processus SLAM/Gazebo fantôme"
    if [ $system_processes -gt 0 ]; then
        echo "💙 $system_processes processus système ROS2 (normal)"
    fi
else
    echo "⚠️  $problematic_processes processus SLAM/Gazebo problématiques détectés"
    echo "💙 $system_processes processus système (normal)"
fi
echo "=================================="
