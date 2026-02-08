#!/usr/bin/env python3
"""
DIAMANTS - Full Microservice Launch
=====================================
⚠️  v0-origin  (commit 47cec8ee — tag v0-origin)
Restaurer : git checkout v0-origin -- launch/diamants_full.launch.py

Launches the entire DIAMANTS backend:

1. Gazebo Harmonic (headless, 8 Crazyflie drones)
2. ros_gz_bridge (Gazebo ↔ ROS2 topic bridge, all 8 drones)
3. 8× control_services (one per drone — hover + velocity relay)
4. SwarmController (social forces + exploration intelligence)
5. PositionBroadcaster (aggregates odom → /diamants/drones/positions)
6. SLAMFusion (collaborative map merging from lidar)
7. MissionCoordinator (mission lifecycle + auto-start)

Usage:
    ros2 launch diamants_microservices diamants_full.launch.py
    ros2 launch diamants_microservices diamants_full.launch.py headless:=True
    ros2 launch diamants_microservices diamants_full.launch.py num_drones:=4
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, LogInfo, SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution,
)

from launch_ros.actions import Node


# All 8 drone names matching crazyflie_multi_world.sdf
DRONE_NAMES = [
    "crazyflie", "crazyflie1", "crazyflie2", "crazyflie3",
    "crazyflie4", "crazyflie5", "crazyflie6", "crazyflie7",
]


def generate_launch_description():
    # =========================================================================
    # Arguments
    # =========================================================================
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Run Gazebo in headless mode (no GUI)'
    )
    num_drones_arg = DeclareLaunchArgument(
        'num_drones', default_value='8',
        description='Number of Crazyflie drones (1-8)'
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='True',
        description='Auto-start exploration mission'
    )
    auto_start_delay_arg = DeclareLaunchArgument(
        'auto_start_delay', default_value='20.0',
        description='Delay before auto-start (seconds) — must be > 8s Gazebo init'
    )
    target_altitude_arg = DeclareLaunchArgument(
        'target_altitude', default_value='0.5',
        description='Target flight altitude in meters'
    )
    exploration_radius_arg = DeclareLaunchArgument(
        'exploration_radius', default_value='20.0',
        description='Exploration radius in meters (30x30m arena)'
    )

    # =========================================================================
    # Package paths
    # =========================================================================
    pkg_gz_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_gz_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # =========================================================================
    # Gazebo model resource path — models are installed alongside worlds
    # =========================================================================
    _models_dir = os.path.join(pkg_gz_gazebo, 'models')
    _existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    _gz_resource_path = f"{_models_dir}:{_existing}" if _existing else _models_dir

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=_gz_resource_path,
    )

    # =========================================================================
    # 1. Gazebo Simulation
    # =========================================================================
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('headless')),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                pkg_gz_gazebo, 'worlds', 'crazyflie_multi_world.sdf -r -s'
            ])
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('headless')),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                pkg_gz_gazebo, 'worlds', 'crazyflie_multi_world.sdf -r'
            ])
        }.items(),
    )

    # =========================================================================
    # 2. Gazebo ↔ ROS2 Bridge (all 8 drones bridged)
    # =========================================================================
    bridge_config = os.path.join(
        pkg_gz_bringup, 'config', 'ros_gz_multi_crazyflie_bridge.yaml'
    )
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # =========================================================================
    # 3. Altitude PD controller from slam_collaboratif is now integrated
    #    directly into SwarmController (K_z=1.05, D_z=0.85, filtered,
    #    max 0.13 m/s). No separate control_services nodes needed.
    # =========================================================================

    # =========================================================================
    # 4. DIAMANTS Microservices (delayed to let Gazebo + bridge init)
    # =========================================================================
    swarm_controller = Node(
        package='diamants_microservices',
        executable='swarm_controller',
        name='diamants_swarm_controller',
        output='screen',
        parameters=[{
            'num_drones': LaunchConfiguration('num_drones'),
            'control_rate_hz': 10.0,
            'target_altitude': LaunchConfiguration('target_altitude'),
            'exploration_radius': LaunchConfiguration('exploration_radius'),
            'safe_distance': 0.8,
            'max_speed': 0.4,
            'takeoff_stagger_sec': 0.5,
            'exploration_cell_size': 1.0,
            'auto_start': LaunchConfiguration('auto_start'),
            'auto_start_delay': LaunchConfiguration('auto_start_delay'),
        }],
    )

    position_broadcaster = Node(
        package='diamants_microservices',
        executable='position_broadcaster',
        name='diamants_position_broadcaster',
        output='screen',
        parameters=[{
            'position_rate_hz': 10.0,
            'telemetry_rate_hz': 5.0,
            'num_drones': LaunchConfiguration('num_drones'),
        }],
    )

    slam_fusion = Node(
        package='diamants_microservices',
        executable='slam_fusion',
        name='diamants_slam_fusion',
        output='screen',
        parameters=[{
            'map_resolution': 0.1,
            'map_size': 200,
            'fusion_rate_hz': 2.0,
            'publish_rate_hz': 1.0,
            'pheromone_evap': 0.98,
            'num_drones': LaunchConfiguration('num_drones'),
        }],
    )

    mission_coordinator = Node(
        package='diamants_microservices',
        executable='mission_coordinator',
        name='diamants_mission_coordinator',
        output='screen',
        parameters=[{
            'auto_start': LaunchConfiguration('auto_start'),
            'auto_start_delay_sec': LaunchConfiguration('auto_start_delay'),
            'mission_timeout_sec': 86400.0,
            'num_drones': LaunchConfiguration('num_drones'),
        }],
    )

    # Delay microservices by 8s to let Gazebo + bridge + controllers init
    delayed_services = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="╔══════════════════════════════════════╗"),
            LogInfo(msg="║  Starting DIAMANTS microservices...  ║"),
            LogInfo(msg="╚══════════════════════════════════════╝"),
            swarm_controller,
            position_broadcaster,
            slam_fusion,
            mission_coordinator,
        ],
    )

    # =========================================================================
    # Launch Description
    # =========================================================================
    return LaunchDescription([
        # Arguments
        headless_arg,
        num_drones_arg,
        auto_start_arg,
        auto_start_delay_arg,
        target_altitude_arg,
        exploration_radius_arg,

        # Banner
        LogInfo(msg="═══════════════════════════════════════════════════════"),
        LogInfo(msg="  DIAMANTS — Multi-Agent SLAM Collaborative System    "),
        LogInfo(msg="  8 Crazyflie drones • Gazebo Harmonic • ROS2 Jazzy   "),
        LogInfo(msg="═══════════════════════════════════════════════════════"),

        # 0. Set Gazebo model resource path
        set_gz_resource_path,

        # 1. Gazebo
        gz_sim_headless,
        gz_sim_gui,

        # 2. Bridge (all 8 drones)
        gz_bridge,

        # 3. Microservices (delayed 8s)
        delayed_services,
    ])
