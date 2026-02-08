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
"""
Launch file for DIAMANTS Web Interface
=========================================
Launches web server + WebSocket bridge with ROS2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for web interface"""
    
    # Launch arguments
    web_host_arg = DeclareLaunchArgument(
        'web_host',
        default_value='0.0.0.0',
        description='Web server IP address'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Web server port'
    )
    
    ws_host_arg = DeclareLaunchArgument(
        'ws_host',
        default_value='localhost',
        description='WebSocket IP address'
    )
    
    ws_port_arg = DeclareLaunchArgument(
        'ws_port',
        default_value='8765',
        description='WebSocket port'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Debug mode for web server'
    )
    
    # Configuration
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    ws_host = LaunchConfiguration('ws_host')
    ws_port = LaunchConfiguration('ws_port')
    debug = LaunchConfiguration('debug')
    
    # Package path
    pkg_share = FindPackageShare('multi_agent_framework')
    
    # WebSocket ↔ ROS2 Bridge Node
    websocket_bridge_node = Node(
        package='multi_agent_framework',
        executable='websocket_bridge.py',
        name='diamants_websocket_bridge',
        parameters=[
            {'ws_host': ws_host},
            {'ws_port': ws_port}
        ],
        remappings=[
            # Legacy topics → standardized /diamants/ namespace
            ('/swarm/drone_positions',   '/diamants/drones/positions'),
            ('/swarm/score',             '/diamants/swarm/intelligence_score'),
            ('/swarm/coverage_area',     '/diamants/swarm/coverage_area'),
            ('/swarm/swarm_status',      '/diamants/swarm/status'),
            ('/swarm/web_commands',      '/diamants/swarm/commands'),
            ('/swarm/drone_commands',    '/diamants/drones/commands'),
            ('/swarm/parameter_changes', '/diamants/parameters'),
        ],
        output='screen',
        respawn=True,
        respawn_delay=5.0
    )
    
    # FastAPI Web Server Node
    web_server_node = Node(
        package='multi_agent_framework',
        executable='web_server.py',
        name='diamants_web_server',
        parameters=[
            {'host': web_host},
            {'port': web_port},
            {'ws_host': ws_host},
            {'ws_port': ws_port},
            {'debug': debug}
        ],
        output='screen',
        respawn=True,
        respawn_delay=5.0
    )
    
    # Process to open browser (optional)
    open_browser = ExecuteProcess(
        cmd=['python3', '-c', f'''
import time
import webbrowser
import subprocess
import socket

def check_server(host, port, timeout=30):
    """Check if server is started"""
    for _ in range(timeout):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex((host, int(port)))
            sock.close()
            if result == 0:
                return True
        except:
            pass
        time.sleep(1)
    return False

# Wait for server to be ready
host = "{web_host}" if "{web_host}" != "0.0.0.0" else "localhost"
port = "{web_port}"

print(f"🔄 Waiting for web server startup on {{host}}:{{port}}...")

if check_server(host, port):
    url = f"http://{{host}}:{{port}}"
    print(f"🌐 Opening browser: {{url}}")
    webbrowser.open(url)
else:
    print("❌ Unable to reach web server")
        '''],
        shell=False,
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        web_host_arg,
        web_port_arg,
        ws_host_arg,
        ws_port_arg,
        debug_arg,
        
        # Nodes
        websocket_bridge_node,
        web_server_node,
        
        # Browser opening (with delay)
        open_browser
    ])