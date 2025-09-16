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

# --- MAP_MERGE_COMPARE.LAUNCH.PY ---
# Fichier de lancement ROS2 pour comparer les deux versions de fusion de cartes
# Lance soit la version modulaire soit la version originale selon le paramètre
# Usage: ros2 launch slam_map_merge map_merge_compare.launch.py use_modular:=true/false
# Auteur: Projet DIAMANTS, 2025

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """
    Génère la description de lancement pour la comparaison des versions.
    
    Arguments:
    - use_modular (bool): true pour version modulaire, false pour originale
    
    Returns:
        LaunchDescription: Description complète du lancement conditionnel
    """
    
    # Déclaration des arguments de lancement
    use_modular_arg = DeclareLaunchArgument(
        'use_modular',
        default_value='false',
        description='Utiliser la version modulaire (true) ou originale (false)'
    )
    
    # Configuration des substitutions
    use_modular = LaunchConfiguration('use_modular')
    
    return LaunchDescription([
        # Argument de lancement
        use_modular_arg,
        
        # Node version MODULAIRE (ultra-permissive)
        Node(
            package='slam_map_merge',
            executable='map_merger_node_modular',
            name='map_merger_modular',
            output='screen',
            condition=IfCondition(use_modular),
            parameters=[{
                'robot_prefixes': ['crazyflie', 'crazyflie1', 'crazyflie2', 'crazyflie3'],
                'use_sim_time': True,
                'phero_increment': 15,
                'consensus_tolerance': 0.8,  # Plus tolérant
            }],
            # Topics de sortie: /map_merged, /map_observations, /coverage_image
        ),
        
        # Node version ORIGINALE (restrictive)  
        Node(
            package='slam_map_merge',
            executable='map_merger_node',
            name='map_merger_original',
            output='screen',
            condition=UnlessCondition(use_modular),
            parameters=[{
                'robot_prefixes': ['crazyflie', 'crazyflie1', 'crazyflie2', 'crazyflie3'],
                'use_sim_time': True,
                'consensus_min': 2,
                'consensus_tol': 0.5,  # Plus strict
            }],
            # Topics de sortie: /map_merged, /map_weighted, /coverage_image
        )
    ])
