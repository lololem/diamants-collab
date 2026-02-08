#!/bin/bash
# DIAMANTS pipeline launcher — fully detached
set -e
source /opt/ros/jazzy/setup.bash
source /tmp/diamants_build/install/setup.bash
source /home/loic/diamants-collab/install/local_setup.bash

echo "[DIAMANTS] Launching full pipeline..."
ros2 launch diamants_microservices diamants_full.launch.py \
    headless:=True \
    num_drones:=8 \
    auto_start:=True \
    auto_start_delay:=20.0 \
    2>&1 | tee /tmp/diamants_launch.log
