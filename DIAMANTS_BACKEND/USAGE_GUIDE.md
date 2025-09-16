# 🎮 DIAMANTS - SLAM Collaborative System Usage Guide

## ✅ System Status: OPERATIONAL

Le script TMUX fonctionne maintenant parfaitement ! Le problème était une incompatibilité avec l'option `--symlink-install` qui a été corrigée.

## 🚀 Quick Start

### 1. Launch Full System
```bash
cd DIAMANTS_BACKEND
./launch_slam_collaborative.sh
```

### 2. Select Launch Mode
- **Option 1**: Full TMUX orchestration (8 drones + SLAM + RViz) ⭐ **Recommended**
- **Option 2**: Web interface only
- **Option 3**: SLAM system only
- **Option 4**: Frontend development server

## 🎛️ TMUX Session Management

### Active Session
```bash
# View all sessions
tmux list-sessions

# Output: slam_collab: 13 windows (created Mon Sep 15 15:29:23 2025)
```

### Window Organization
```bash
# List all windows
tmux list-windows -t slam_collab

# Windows:
# 0: Initial shell
# 1: Gazebo simulation (8 drones)
# 2: ROS2-Gazebo bridge
# 3: Multi-robot mapping
# 4: Exploration agents
# 5: SLAM nodes (8 drones)
# 6: Inter-drone coordinator
# 7: RViz visualization
# 8: Monitoring
# 9: Live logs
# 10-12: Additional terminals
```

### Navigation
```bash
# Attach to session
tmux attach -t slam_collab

# Switch between windows (inside TMUX)
Ctrl+B then 0-9    # Switch to window 0-9
Ctrl+B then n      # Next window
Ctrl+B then p      # Previous window

# Detach from session (leave running)
Ctrl+B then d

# View specific window output
tmux capture-pane -t slam_collab:1 -p    # Gazebo simulation
tmux capture-pane -t slam_collab:8 -p    # Monitoring
```

## 🔧 System Components

### 1. Gazebo Simulation (Window 1)
- **8 Crazyflie drones** in collaborative environment
- **Multi-world environment** with obstacles and fire elements
- **Physics simulation** with collision detection

### 2. SLAM System (Windows 3-5)
- **Collaborative mapping** between 8 drones
- **Real-time map fusion** with `slam_map_merge`
- **Wall-following algorithms** with multiranger sensors
- **Position coordination** for collision avoidance

### 3. Visualization (Window 7)
- **RViz2** with optimized SLAM configuration
- **Real-time map visualization** from all drones
- **Trajectory tracking** and position monitoring

### 4. Web Interface Integration
- **API layer** bridging ROS2 and frontend
- **Real-time data streaming** via WebSockets
- **Frontend control panel** in DIAMANTS_FRONTEND/

## 📊 Monitoring & Debugging

### Live Logs
```bash
# Real-time journal
tail -f /tmp/diamants_tmux_slam_collab_journal.log

# Or view in TMUX window 9
tmux select-window -t slam_collab:9
```

### System Status
```bash
# Check ROS2 nodes
ros2 node list | grep -E "(crazyflie|slam|map)"

# Check topics
ros2 topic list | grep -E "(map|pose|scan)"

# Monitor resource usage
htop
```

### Diagnostic Tools
```bash
# Available in monitoring window (8)
./scripts/diagnostic_slam_status.sh           # Quick status
python3 scripts/diagnostic_slam_system.py     # Complete analysis
watch -n 10 "./scripts/diagnostic_slam_status.sh"  # Continuous monitoring
```

## 🛑 System Control

### Stop System
```bash
# Kill entire SLAM session
tmux kill-session -t slam_collab

# Or gracefully stop individual components
tmux send-keys -t slam_collab:1 C-c    # Stop Gazebo
tmux send-keys -t slam_collab:7 C-c    # Stop RViz
```

### Restart Components
```bash
# Restart specific window
tmux send-keys -t slam_collab:X "your_command" Enter
```

## 🔄 Integration with Frontend

The backend automatically integrates with:
- **Frontend Location**: `../DIAMANTS_FRONTEND/Mission_system/`
- **Web Interface**: `http://localhost:8080` (when web mode active)
- **API Endpoints**: Real-time drone status and control

### Launch Frontend Separately
```bash
# Option 4 from main launcher, or manually:
cd ../DIAMANTS_FRONTEND/Mission_system
npm run dev
```

## 🎯 Key Features Working

✅ **8-Drone Collaborative SLAM** - Multiple drones mapping simultaneously  
✅ **Real-time Map Fusion** - Combined maps from all drones  
✅ **Collision Avoidance** - Inter-drone position coordination  
✅ **Wall-Following** - Autonomous exploration with multiranger sensors  
✅ **RViz Visualization** - Real-time 3D visualization of mapping progress  
✅ **Web Integration** - Bridge between ROS2 backend and JavaScript frontend  
✅ **TMUX Orchestration** - Organized multi-terminal management  
✅ **English Documentation** - Complete translation for international use  

## 🚨 Troubleshooting

### Build Issues
- **Problem**: `--editable not recognized`
- **Solution**: ✅ Fixed - now uses `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`

### TMUX Issues
- **Problem**: Session not found
- **Solution**: Check `tmux list-sessions` and restart launcher

### ROS2 Issues
- **Problem**: Nodes not communicating
- **Solution**: Verify ROS2 environment: `echo $ROS_DOMAIN_ID`

---

**Status**: 🟢 **OPERATIONAL** - SLAM collaborative system fully functional
**Last Updated**: September 15, 2025 - 15:30 CEST
