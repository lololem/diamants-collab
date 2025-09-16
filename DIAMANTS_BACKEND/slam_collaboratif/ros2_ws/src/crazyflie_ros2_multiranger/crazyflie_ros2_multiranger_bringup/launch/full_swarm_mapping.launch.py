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

import os
import yaml
import ast

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Déclaration des arguments de lancement
    drones_arg = DeclareLaunchArgument(
        'drones',
        default_value="['crazyflie', 'crazyflie1', 'crazyflie2', 'crazyflie3', 'crazyflie4', 'crazyflie5', 'crazyflie6', 'crazyflie7']",
        description='Liste des drones à lancer'
    )
    
    map_merger_executable_arg = DeclareLaunchArgument(
        'map_merger_executable',
        default_value='map_merger_node_modular',
        description='Executable du map merger à utiliser'
    )
    
    # Récupération des valeurs des arguments
    drones_config = LaunchConfiguration('drones')
    map_merger_executable = LaunchConfiguration('map_merger_executable')
    
    # Setup project paths
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    # Lancement de la simulation Gazebo multi-drones
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation_multi_crazy.launch.py'))
    )

    # Charger la configuration depuis le fichier YAML existant
    config_path = os.path.join(
        get_package_share_directory('slam_map_merge'),
        'config',
        'swarm_8_drones.yaml'
    )
    
    drones = []
    yaml_config = {}
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                yaml_config = yaml.safe_load(f)
                drones = yaml_config['swarm']['robot_prefixes']
                print(f"🚁 Configuration chargée depuis {config_path}")
                print(f"🎯 Drones configurés: {drones}")
        except Exception as e:
            print(f"⚠️  Erreur lors du chargement de {config_path}: {e}")
            drones = ['crazyflie', 'crazyflie1', 'crazyflie2', 'crazyflie3']
    else:
        print(f"⚠️  Configuration non trouvée: {config_path}")
        drones = ['crazyflie', 'crazyflie1', 'crazyflie2', 'crazyflie3']
    
    print(f"🎯 Lancement configuré pour {len(drones)} drones: {drones}")
    nodes = []


    for drone in drones:
        start_direction = "left" if drone == "crazyflie1" else "right"

        # Node simple mapper
        nodes.append(
            Node(
                package='crazyflie_ros2_multiranger_simple_mapper',
                executable='simple_mapper_multiranger',
                name=f'simple_mapper_{drone}',
                output='screen',
                parameters=[
                    {'robot_prefix': drone},
                    {'use_sim_time': True}
                ],
            )
        )

        # Node wall following avec paramètre start_direction
        nodes.append(
            Node(
                package='crazyflie_ros2_multiranger_wall_following',
                executable='wall_following_multicrazy_multiranger',
                name=f'wall_following_{drone}',
                output='screen',
                parameters=[
                    {'robot_prefix': drone},
                    {'use_sim_time': True},
                    {'delay': 3.0 + drones.index(drone)},  # décalage initial
                    {'mode': 'scout'},
                    {'start_direction': start_direction}
                ],
            )
        )

    # Configuration RViz
    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'sim_mapping.rviz')

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{"use_sim_time": True}]
    )
    
    # Configuration du map merger avec paramètres extraits du YAML
    map_merger_params = {
        'robot_prefixes': drones,
        'use_sim_time': True,
    }
    
    # Ajouter les paramètres du YAML si disponible
    if yaml_config:
        # Pheromones
        if 'pheromones' in yaml_config:
            phero = yaml_config['pheromones']
            map_merger_params.update({
                'phero_increment': phero.get('increment', 20),
                'phero_decrement': phero.get('decrement', 8),
                'phero_evaporation': phero.get('evaporation', 0.4),
                'phero_max_value': phero.get('max_value', 120),
                'phero_min_value': phero.get('min_value', 0),
                'phero_obstacle_bonus': phero.get('obstacle_bonus', 15),
                'phero_free_penalty': phero.get('free_penalty', 25),
            })
        
        # Performance
        if 'performance' in yaml_config:
            perf = yaml_config['performance']
            map_merger_params.update({
                'max_update_rate': perf.get('max_update_rate', 2.0),
                'memory_limit_mb': perf.get('memory_limit_mb', 512),
                'parallel_processing': perf.get('parallel_processing', False),
            })
            
        # Logging
        if 'logging' in yaml_config:
            log = yaml_config['logging']
            map_merger_params.update({
                'enable_csv': log.get('enable_csv', True),
                'csv_path': log.get('csv_path', '/tmp/swarm_8_drones'),
                'debug_level': log.get('debug_level', 'warn'),
                'log_frequency': log.get('log_frequency', 50),
            })

    map_merger_node = Node(
        package='slam_map_merge',
        executable=map_merger_executable,
        name='map_merger',
        output='screen',
        parameters=[map_merger_params]
    )

    # Lancement final avec simulation, RViz et nodes
    return LaunchDescription([
        drones_arg,
        map_merger_executable_arg,
        crazyflie_simulation,
        rviz,
        map_merger_node,  # Ajout du node de fusion
        *nodes
    ])
