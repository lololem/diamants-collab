#!/bin/bash
# DIAMANTS Backend Launch Script
set -e
source /opt/ros/jazzy/setup.bash
source /home/loic/diamants-collab/install/setup.bash
source /tmp/diamants_build/install/setup.bash
ros2 launch diamants_microservices diamants_full.launch.py \
  headless:=True \
  auto_start:=True \
  auto_start_delay:=15.0 \
  target_altitude:=0.5
