# DIAMANTS - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
# 
# Copyright (c) 2025 DIAMANTS Project Contributors
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#!/usr/bin/env python3

# --- MAP_MERGE.LAUNCH.PY ---
# Fichier de lancement ROS2 pour le système de fusion de cartes SLAM collaboratif
# Lance le nœud map_merger_node avec configuration par défaut pour 3 drones
# Auteur: Projet DIAMANTS, 2025

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Génère la description de lancement pour le système de fusion de cartes.
    
    Configuration par défaut:
    - 3 drones: crazyflie, crazyflie1, crazyflie2  
    - Mode simulation activé
    - Version restrictive (map_merger_node)
    
    Returns:
        LaunchDescription: Description complète du lancement
    """
    return LaunchDescription([
        Node(
            package='slam_map_merge',
            executable='map_merger_node',  # Version restrictive/originale
            name='map_merger',
            output='screen',
            parameters=[{
                'robot_prefixes': ['crazyflie', 'crazyflie1', 'crazyflie2'],  # Drones à fusionner
                'use_sim_time': True  # Utiliser le temps de simulation Gazebo
            }],
            # Topics d'entrée automatiques: /crazyflie/map, /crazyflie1/map, /crazyflie2/map
            # Topics de sortie: /map_merged (carte fusionnée), /coverage_image (visualisation)
        )
    ])
