# DIAMANTS V3 - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
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

# --- MULTI_AGENT_DIAMANTS.LAUNCH.PY ---
# Fichier de lancement ROS2 pour le système de fusion de cartes SLAM collaboratif DIAMANTS V3
# Lance le nœud map_merger_node avec configuration pour 8 drones Crazyflie
# Auteur: Projet DIAMANTS V3, 2025

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    """
    Génère la description de lancement pour le système de fusion de cartes DIAMANTS V3.
    
    Configuration DIAMANTS V3:
    - 8 drones: crazyflie, crazyflie1, crazyflie2, crazyflie3, crazyflie4, crazyflie5, crazyflie6, crazyflie7
    - Mode simulation activé pour Gazebo
    - Version restrictive (map_merger_node) avec stigmergie et consensus
    
    Returns:
        LaunchDescription: Description complète du lancement pour 8 drones
    """
    
    # Déclaration des arguments de lancement
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Lancer RViz pour visualisation'
    )
    
    return LaunchDescription([
        rviz_arg,
        
        # Nœud principal de fusion de cartes pour 8 drones
        Node(
            package='slam_map_merge',
            executable='map_merger_node',  # Version restrictive avec stigmergie
            name='map_merger',
            output='screen',
            parameters=[{
                # Configuration 8 drones DIAMANTS V3
                'robot_prefixes': [
                    'crazyflie', 'crazyflie1', 'crazyflie2', 'crazyflie3', 
                    'crazyflie4', 'crazyflie5', 'crazyflie6', 'crazyflie7'
                ],
                'use_sim_time': True,  # Synchronisation avec Gazebo
                
                # Configuration fusion restrictive
                'consensus_min': 2,  # Minimum 2 drones pour consensus
                'consensus_tol': 0.4,  # 40% des drones pour validation
                'enable_morphology': True,  # Filtrage morphologique activé
                'min_cluster_size': 5,  # Suppression petits clusters
                
                # Topics de sortie
                'output_merged': 'map_merged',  # Carte principale fusionnée
                'output_weighted': 'map_fusion_weighted',  # Carte pondérée
                'output_golden': 'map_golden_zones',  # Zones consensus fort
                'output_coverage_image': 'coverage_image',  # Visualisation phéromones
                
                # Paramètres stigmergie
                'pheromone_inc': 15,  # Incrément phéromones
                'pheromone_evap': 0.3,  # Évaporation phéromones
                'pheromone_max': 100,  # Maximum phéromones
                'smoothing_window': 5  # Lissage temporel
            }],
            # Topics d'entrée automatiques: 
            # /crazyflie/map, /crazyflie1/map, ..., /crazyflie7/map (8 drones)
            # Topics de sortie: 
            # /map_merged, /map_fusion_weighted, /map_golden_zones, /coverage_image
        ),
        
        # RViz pour visualisation des cartes fusionnées
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '$(find-pkg-share slam_map_merge)/../../../../../config/rviz_stigmergie_config.rviz'],
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])