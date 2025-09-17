#!/bin/bash

# 🎮 DIAMANTS Sample Files Renaming Script
# Purpose: Rename HTML sample files according to their functionalities
# Date: September 17, 2025
# Author: lololem

echo "🎮 DIAMANTS Sample Files Renaming Script"
echo "========================================"

# Navigate to sample directory
cd "$(dirname "$0")"

# Function to rename file if it exists
rename_if_exists() {
    local old_name="$1"
    local new_name="$2"
    
    if [ -f "$old_name" ]; then
        mv "$old_name" "$new_name"
        echo "✅ Renamed: $old_name → $new_name"
    else
        echo "⚠️  File not found: $old_name"
    fi
}

echo ""
echo "🔄 Starting HTML sample file renaming process..."
echo ""

# Search & Rescue Simulation
rename_if_exists "crazyflie_search_to_fix.html" "DIAMANTS_Crazyflie_Search_Rescue_Gazebo_Simulation.html"

# Complete Frontend Demo
rename_if_exists "demo_complete_frontend.html" "DIAMANTS_Complete_Frontend_Swarm_Intelligence_Demo.html"

echo ""
echo "📊 Final sample files inventory:"
echo "==============================="
ls -lh *.html | awk '{print $9 " (" $5 ")"}'

echo ""
echo "✅ Sample files renaming completed!"
echo "📋 See README_Sample_Files.md for detailed documentation"
