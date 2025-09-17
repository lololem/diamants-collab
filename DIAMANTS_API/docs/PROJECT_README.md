# 🚁 DIAMANTS Project - Main README

**Please copy this file to the project root directory as `README.md`**

# 🚁 DIAMANTS - Autonomous Drone Swarm System

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy%20%7C%20Humble-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Garden-orange)](https://gazebosim.org/)
[![Node.js](https://img.shields.io/badge/Node.js-16+-green)](https://nodejs.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Production%20Ready-brightgreen)]()

**DIAMANTS** is an advanced autonomous drone swarm system featuring collaborative SLAM (Simultaneous Localization and Mapping), intelligent mission planning, and real-time visualization.

## ⚡ Quick Start (2 Minutes)

```bash
# 1. Clone the repository
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab

# 2. One-command setup and launch
./DIAMANTS_API/quick-setup.sh
```

**That's it!** 🎉 Your autonomous drone swarm will be running with:
- 🌐 **Mission Control**: http://localhost:5550
- 🤖 **8-Drone Simulation**: Gazebo environment
- 📊 **Real-time Visualization**: RViz interface
- 🗺️ **Collaborative SLAM**: Multi-agent mapping

## 🎯 What You Get

### Frontend (Mission Control System)
- **Real-time Mission Dashboard**: Monitor swarm operations
- **Interactive 3D Visualization**: See drone positions and maps
- **Mission Planning Interface**: Plan and execute autonomous missions
- **Performance Analytics**: Track exploration efficiency

### Backend (ROS2 SLAM System)
- **8-Drone Collaborative SLAM**: Multi-agent mapping and exploration
- **Intelligent Path Planning**: Autonomous navigation with obstacle avoidance
- **Real-time Data Fusion**: Merge sensor data from multiple drones
- **TMUX Management**: Organized multi-terminal session control

## 🚀 Available Commands

### Quick Management
```bash
./DIAMANTS_API/quick-setup.sh      # Complete setup and launch
./DIAMANTS_API/status.sh           # Check services status
./DIAMANTS_API/stop-all.sh         # Stop all services
./DIAMANTS_API/restart.sh          # Restart everything
```

### Individual Services
```bash
./DIAMANTS_API/start-frontend.sh   # Frontend only
./DIAMANTS_API/start-backend.sh    # Backend only
./DIAMANTS_API/start-all.sh        # Both services
```

### Diagnostics & Maintenance
```bash
./DIAMANTS_API/diagnose.sh         # System diagnostic
./DIAMANTS_API/fix-ports.sh        # Resolve port conflicts
./DIAMANTS_API/setup-dependencies.sh  # Install dependencies only
```

## 📋 System Requirements

- **OS**: Ubuntu 20.04/22.04 LTS (recommended)
- **Memory**: 8GB RAM minimum, 16GB recommended
- **Storage**: 5GB free space
- **CPU**: 4+ cores recommended
- **Graphics**: OpenGL 3.3+ for visualization

**Dependencies** (auto-installed):
- ROS2 Jazzy/Humble (auto-detected)
- Gazebo Garden
- Node.js 16+
- Python 3.8+
- TMUX for session management

## ✅ Deployment Status

**✅ PRODUCTION READY** - Complete deployment system tested and working!

- ✅ Frontend: Auto-deployment with port management (5550-5559)
- ✅ Backend: Full TMUX orchestration with ROS2 Jazzy/Humble support
- ✅ Cross-platform: Portable shell scripts with relative paths
- ✅ Auto-setup: One-command clone-and-run experience
- ✅ Monitoring: Real-time status checking and diagnostics

### Last Tested: September 16, 2025
- System: Ubuntu with ROS2 Jazzy
- Frontend: ✅ Running on http://localhost:5550
- Backend: ✅ TMUX session active with 12 windows
- SLAM: ✅ 8-drone collaborative mapping operational
- Gazebo: ✅ Multi-drone simulation running

## 🔍 Verification Steps

After running `./DIAMANTS_API/quick-setup.sh`, verify:

1. **Frontend**: Open http://localhost:5550 → Mission Control Dashboard
2. **Backend**: Run `tmux attach -t slam_collab` → 12 active windows
3. **Status**: Run `./DIAMANTS_API/status.sh` → All green checkmarks
4. **Simulation**: Gazebo should show 8 drones in environment

## 🆘 Troubleshooting

### Common Issues

**Port conflicts?**
```bash
./DIAMANTS_API/fix-ports.sh
```

**Dependencies missing?**
```bash
./DIAMANTS_API/setup-dependencies.sh
```

**Services not starting?**
```bash
./DIAMANTS_API/diagnose.sh
```

**Permission issues?**
```bash
sudo chown -R $USER:$USER ./DIAMANTS_*
```

---

**🚁 Ready to explore autonomous drone swarms?**

Run `./DIAMANTS_API/quick-setup.sh` and start your journey into the future of robotics! ✨
