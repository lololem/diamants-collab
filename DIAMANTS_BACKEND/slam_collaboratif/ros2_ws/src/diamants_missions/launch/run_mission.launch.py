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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import yaml


def _load_yaml(path: str) -> dict:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Mission YAML not found: {path}")
    with p.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _compose_launch(context):
    mission_file = LaunchConfiguration("mission").perform(context)
    cfg = _load_yaml(mission_file)

    actions = []

    # Optional: generic include of other launch files
    for inc in cfg.get("includes", []) or []:
        pkg = inc.get("package")
        launch_file = inc.get("launch_file")
        args: dict = inc.get("arguments", {}) or {}
        if not launch_file:
            continue
        # Resolve path: absolute path or `<pkg>/launch/<launch_file>`
        if pkg:
            pkg_share = get_package_share_directory(pkg)
            lf_path = os.path.join(pkg_share, "launch", launch_file)
        else:
            lf_path = launch_file
        if not os.path.exists(lf_path):
            raise FileNotFoundError(f"Included launch file not found: {lf_path}")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lf_path),
                launch_arguments={k: str(v) for k, v in args.items()}.items(),
            )
        )

    # Launch nodes from configuration
    for n in cfg.get("nodes", []):
        actions.append(
            Node(
                package=n["package"],
                executable=n.get("executable"),
                name=n.get("name"),
                namespace=n.get("namespace", ""),
                parameters=n.get("parameters", []),
                arguments=n.get("arguments", []),
                output=n.get("output", "screen"),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission",
                default_value=str(
                    Path(__file__).resolve().parent.parent
                    / "missions"
                    / "mission_exploration_simple.yaml"
                ),
                description="Path to mission YAML file",
            ),
            OpaqueFunction(function=_compose_launch),
        ]
    )
