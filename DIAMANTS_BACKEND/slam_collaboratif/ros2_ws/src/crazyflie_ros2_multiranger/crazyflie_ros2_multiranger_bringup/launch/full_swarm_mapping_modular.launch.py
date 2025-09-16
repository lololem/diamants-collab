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

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def load_swarm_config(context, *args, **kwargs):
    """Charge la configuration et génère les nodes dynamiquement"""
    
    # Récupérer les arguments de lancement
    config_file = LaunchConfiguration('config_file').perform(context)
    use_sim = LaunchConfiguration('use_simulation').perform(context) == 'true'
    enable_rviz = LaunchConfiguration('enable_rviz').perform(context) == 'true'
    
    # Chemin par défaut vers la configuration
    default_config_path = os.path.join(
        get_package_share_directory('slam_map_merge'),
        'config',
        'map_merger_config.yaml'
    )
    
    # Charger la configuration
    if config_file and os.path.exists(config_file):
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
    else:
        # Configuration par défaut
        config = {
            'swarm': {
                'robot_prefixes': ['crazyflie', 'crazyflie1', 'crazyflie2', 'crazyflie3'],
                'use_sim_time': True
            },
            'simulation': {
                'gazebo_world': 'crazyflie_simulation_multi_crazy.launch.py',
                'enable_physics': True
            },
            'mapping': {
                'simple_mapper_package': 'crazyflie_ros2_multiranger_simple_mapper',
                'wall_following_package': 'crazyflie_ros2_multiranger_wall_following'
            },
            'visualization': {
                'rviz_config': 'ultra_robust_sim_mapping.rviz'
            }
        }
    
    nodes = []
    drones = config['swarm']['robot_prefixes']
    use_sim_time = config['swarm']['use_sim_time']
    
    # === Simulation Gazebo (si activée) ===
    if use_sim:
        try:
            pkg_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')
            gazebo_launch = config.get('simulation', {}).get('gazebo_world', 'crazyflie_simulation_multi_crazy.launch.py')
            
            crazyflie_simulation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo, 'launch', gazebo_launch))
            )
            nodes.append(crazyflie_simulation)
        except Exception as e:
            print(f"Attention: Simulation Gazebo non disponible: {e}")
    
    # === Nodes pour chaque drone ===
    for i, drone in enumerate(drones):
        start_direction = "left" if i % 2 == 1 else "right"
        
        # Simple mapper node
        try:
            mapper_pkg = config.get('mapping', {}).get('simple_mapper_package', 'crazyflie_ros2_multiranger_simple_mapper')
            nodes.append(
                Node(
                    package=mapper_pkg,
                    executable='simple_mapper_multiranger',
                    name=f'simple_mapper_{drone}',
                    output='screen',
                    parameters=[
                        {'robot_prefix': drone},
                        {'use_sim_time': use_sim_time}
                    ],
                )
            )
        except:
            print(f"Attention: Package mapper non trouvé pour {drone}")
        
        # Wall following node
        try:
            wall_pkg = config.get('mapping', {}).get('wall_following_package', 'crazyflie_ros2_multiranger_wall_following')
            nodes.append(
                Node(
                    package=wall_pkg,
                    executable='wall_following_multicrazy_multiranger',
                    name=f'wall_following_{drone}',
                    output='screen',
                    parameters=[
                        {'robot_prefix': drone},
                        {'use_sim_time': use_sim_time},
                        {'delay': 3.0 + i},  # décalage initial
                        {'mode': 'scout'},
                        {'start_direction': start_direction}
                    ],
                )
            )
        except:
            print(f"Attention: Package wall following non trouvé pour {drone}")
    
    # === Map Merger Node ===
    map_merger_node = Node(
        package='slam_map_merge',
        executable='map_merger_node_modular',  # Utiliser la version modulaire
        name='map_merger',
        output='screen',
        parameters=[config_file] if config_file and os.path.exists(config_file) else [default_config_path]
    )
    nodes.append(map_merger_node)
    
    # === RViz (si activé) ===
    if enable_rviz:
        try:
            rviz_config_name = config.get('visualization', {}).get('rviz_config', 'ultra_robust_sim_mapping.rviz')
            rviz_config_path = os.path.join(
                get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
                'config',
                rviz_config_name
            )
            
            if os.path.exists(rviz_config_path):
                rviz = Node(
                    package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_path],
                    parameters=[{"use_sim_time": use_sim_time}]
                )
                nodes.append(rviz)
            else:
                print(f"Attention: Fichier RViz non trouvé: {rviz_config_path}")
        except Exception as e:
            print(f"Attention: RViz non disponible: {e}")
    
    return nodes

def generate_launch_description():
    """Génère la description de lancement"""
    
    # Chemin par défaut vers la configuration
    default_config_path = os.path.join(
        get_package_share_directory('slam_map_merge'),
        'config',
        'map_merger_config.yaml'
    )
    
    return LaunchDescription([
        # Arguments de lancement
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_path,
            description='Chemin vers le fichier de configuration YAML'
        ),
        
        DeclareLaunchArgument(
            'use_simulation',
            default_value='true',
            choices=['true', 'false'],
            description='Activer la simulation Gazebo'
        ),
        
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Activer RViz pour la visualisation'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            choices=['debug', 'info', 'warn', 'error'],
            description='Niveau de log'
        ),
        
        # Fonction opaque pour charger la config et générer les nodes
        OpaqueFunction(function=load_swarm_config)
    ])
