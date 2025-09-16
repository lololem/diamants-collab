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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    map_name = 'map'

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('crazyflie'), 'launch'
                        ),
                        '/launch.py',
                    ]
                ),
                launch_arguments={
                    'backend': 'cflib',
                    'gui': 'false',
                    'teleop': 'false',
                    'mocap': 'false',
                }.items(),
            ),
            Node(
                package='crazyflie',
                executable='vel_mux.py',
                name='vel_mux',
                output='screen',
                parameters=[
                    {'hover_height': 0.3},
                    {'incoming_twist_topic': '/cmd_vel'},
                    {'robot_prefix': '/cf231'},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch/online_async_launch.py',
                    )
                ),
                launch_arguments={
                    'slam_params_file': os.path.join(
                        get_package_share_directory('crazyflie_examples'),
                        'config/slam_params.yaml',
                    ),
                    'use_sim_time': 'False',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch/bringup_launch.py',
                    )
                ),
                launch_arguments={
                    'slam': 'False',
                    'use_sim_time': 'False',
                    'map': get_package_share_directory('crazyflie_examples')
                    + '/data/'
                    + map_name
                    + '.yaml',
                    'params_file': os.path.join(
                        get_package_share_directory('crazyflie_examples'),
                        'config/nav2_params.yaml',
                    ),
                    'autostart': 'True',
                    'use_composition': 'True',
                    'transform_publish_period': '0.02',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch/rviz_launch.py',
                    )
                ),
                launch_arguments={
                    'rviz_config': os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'rviz',
                        'nav2_default_view.rviz',
                    )
                }.items(),
            ),
        ]
    )
