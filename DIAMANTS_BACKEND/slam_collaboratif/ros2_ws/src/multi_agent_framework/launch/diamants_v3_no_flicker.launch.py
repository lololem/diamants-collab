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
"""
DIAMANTS V3 Multi-Agent Framework Launch File
Framework de coordination multi-agent avec noms uniques
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Coordinator principal avec nom unique
        Node(
            package='multi_agent_framework',
            executable='drone_position_coordinator.py',
            name='multi_agent_coordinator',
            namespace='DIAMANTS_BACKEND',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'coordination_enabled': True,
                'max_drones': 8,
                'safety_distance': 1.5,
                'update_rate': 10.0
            }],
            output='screen'
        ),
        
        # Agent de surveillance des collisions
        Node(
            package='multi_agent_framework', 
            executable='drone_position_coordinator.py',
            name='collision_monitor',
            namespace='DIAMANTS_BACKEND',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'monitor_mode': True,
                'collision_threshold': 0.8,
                'emergency_stop': True
            }],
            output='screen'
        ),
        
        # Agent de coordination des trajectoires
        Node(
            package='multi_agent_framework',
            executable='drone_position_coordinator.py', 
            name='trajectory_coordinator',
            namespace='DIAMANTS_BACKEND',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'trajectory_mode': True,
                'optimization_enabled': True,
                'path_planning': True
            }],
            output='screen'
        ),
    ])