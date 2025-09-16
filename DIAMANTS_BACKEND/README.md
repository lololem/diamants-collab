# 🚁 DIAMANTS V3 - ROS2 Backend (Production Ready)

**Multi-Agent Collaborative SLAM System** with automated TMUX orchestration, 8-drone swarm simulation, and real-time stigmergy-based map fusion.

## 🎯 Overview

This backend provides a **complete collaborative SLAM ecosystem** featuring:
- ✅ **8-Drone Swarm Simulation** with Gazebo physics integration
- ✅ **Automated TMUX Orchestration** for multi-component management
- ✅ **Stigmergy-Based Map Fusion** using bio-inspired algorithms
- ✅ **RVIZ Real-time Visualization** with specialized SLAM configurations
- ✅ **ROS2 Jazzy Integration** with modern message protocols
- ✅ **WebSocket Bridge** for frontend-backend communication
- ✅ **Production-Ready Deployment** with comprehensive testing

## 🚀 Quick Start (After Git Clone)

### **One-Command System Launch**

```bash
cd DIAMANTS_BACKEND
make                    # 🎯 Complete automated setup & validation
make launch-tmux        # 🚀 Launch full collaborative SLAM system
```

### **Alternative Launch Methods**

```bash
# From project root (recommended)
cd /path/to/DIAMANTS
./run.sh                # Interactive launcher
# Select option 1: "Backend ROS2 + SLAM"

# Manual launch
cd DIAMANTS_BACKEND
echo "1" | ./launch_slam_collaborative.sh
```

### **Makefile Commands (Complete Automation)**

```bash
# 🔧 Configuration & Setup
make                    # Complete automated setup (recommended first-time)
make post-clone         # Detailed post-clone configuration
make setup              # Manual setup with user interaction
make build              # Build ROS2 workspace only
make clean              # Clean build artifacts and reset

# 🚀 System Management
make launch-tmux        # Launch full collaborative SLAM system
make status-tmux        # Check TMUX session and system status
make kill-tmux          # Clean shutdown of all components
make restart            # Full system restart (kill + launch)

# 🧪 Testing & Validation
make test               # Run comprehensive system validation
make test-ros           # Test ROS2 components only
make test-gazebo        # Test Gazebo simulation environment
make info               # Show system information and diagnostics

# 🔍 Monitoring & Debugging
make logs               # Display recent system logs
make monitor            # Real-time system monitoring
make debug-slam         # Debug SLAM system components
```

## 📋 **Workflow Guide**

### **1. Initial Setup (Fresh Clone)**
```bash
make                    # Automated configuration
# ✅ System prerequisites check
# ✅ ROS2 workspace build  
# ✅ Dependencies installation
# ✅ Validation tests
```

### **2. Launch System**
```bash
make launch-tmux        # Launch collaborative SLAM
# 📺 Creates TMUX session 'slam_collab'
# 🚀 10 specialized windows
# 🤖 8-drone Gazebo simulation
# 🗺️  RVIZ visualization with stigmergie
# 🔗 ROS2-Gazebo bridge
# 📊 Real-time mapping fusion
```

### **3. System Monitoring**
```bash
make status-tmux        # Check system status
tmux attach -t slam_collab  # Connect to TMUX session

# TMUX Navigation:
# Ctrl+b + [0-9]  : Switch windows
# Ctrl+b + n      : Next window
# Ctrl+b + p      : Previous window
```

### **4. System Shutdown**
```bash
make kill-tmux          # Clean shutdown
# 🛑 Stops TMUX sessions
# 🧹 Kills ROS2/Gazebo processes
# ✅ Clean system state
```

## 📁 Production Directory Structure

```
DIAMANTS_BACKEND/                           # 🏭 PRODUCTION READY
├── 🚀 launch_slam_collaborative.sh     # Main interactive launcher
├── 🔧 Makefile                         # Automated build & management system
├── 📦 slam_collaboratif/               # 🎯 CORE SLAM SYSTEM
│   ├── config/                         # SLAM configurations
│   └── ros2_ws/                        # ROS2 workspace (8-drone ready)
│       ├── src/                        # Source packages
│       │   ├── crazyflie_ros2_multiranger/    # Drone SLAM modules
│       │   ├── slam_map_merge/               # Map fusion with stigmergie
│       │   └── multi_agent_framework/        # Swarm intelligence
│       ├── install/                    # Built packages
│       └── build/                      # Build artifacts
├── 🌐 core/                            # Essential components
│   ├── launchers/                      # Python system launchers
│   └── web_interface/                  # Frontend integration
├── ⚙️ config/                          # System configurations
│   ├── rviz_config_slam_optimized.rviz # RVIZ for SLAM
│   └── rviz_stigmergie_config.rviz     # RVIZ stigmergie visualization
├── � .env.example                     # Environment template
├── 🛡️ .gitignore                       # Security-focused git rules
├── 📚 USAGE_GUIDE.md                   # Detailed usage guide
└── � README.md                        # This file
```

## 📊 **TMUX Session Architecture**

When you run `make launch-tmux`, the system creates a specialized TMUX session:

```
slam_collab session:
├── Window 0: Control Terminal        # System control
├── Window 1: Gazebo Simulation      # 8-drone physics simulation  
├── Window 2: ROS2-Gazebo Bridge     # Communication bridge
├── Window 3: SLAM Map Merger        # Stigmergie-based map fusion
├── Window 4-7: Drone Controllers   # Individual drone management
├── Window 8: RVIZ Visualization    # Real-time map display
└── Window 9: System Monitor        # Performance monitoring
```

## 🧩 Core Features

### **Collaborative SLAM System**

- **8-Drone Swarm**: Coordinated exploration and mapping
- **Stigmergie-Based Fusion**: Bio-inspired map merging algorithm
- **Real-Time Visualization**: RVIZ with specialized configurations
- **Consensus Mapping**: Robust multi-agent map validation

### **TMUX Orchestration**

- **Automated Session Management**: No manual window setup
- **Process Isolation**: Each component in dedicated window
- **Clean Shutdown**: Integrated process termination
- **Status Monitoring**: Real-time system health checks

### **Production Features**

- **MIT Licensed**: Complete legal framework
- **Security Hardened**: Credentials secured, dangerous elements removed
- **Automated Testing**: System validation on build
- **Documentation**: Complete usage guides and API docs

## 📋 **System Requirements**

### **Prerequisites (Auto-checked by Makefile)**

```bash
# Required
- ROS2 Jazzy (auto-detected)
- Python 3.8+ (auto-verified)
- Git (auto-verified)
- TMUX (auto-installed if missing)

# Optional (auto-installed)
- Colcon build tools
- Development dependencies
```

### **Hardware Recommendations**

- **CPU**: 4+ cores (8 recommended for 8-drone simulation)
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: OpenGL support for Gazebo/RVIZ
- **Storage**: 5GB for full workspace

## 🔧 **Advanced Configuration**

### **Environment Variables** (Optional)

The system works with defaults, but you can customize:

```bash
# Create from template (optional)
cp .env.example .env

# Key variables (auto-configured)
ROS_DOMAIN_ID=42                    # ROS2 network isolation
RMW_IMPLEMENTATION=rmw_cyclonedx    # ROS2 middleware  
AGENTS=8                            # Number of collaborative drones
```

### **SLAM Parameters**

Configure collaborative mapping in:
`slam_collaboratif/ros2_ws/src/slam_map_merge/config/`

```yaml
# Stigmergie parameters
pheromone_inc: 15      # Information reinforcement
pheromone_evap: 0.3    # Information decay  
consensus_min: 2       # Minimum agents for validation
consensus_tol: 0.4     # Agreement threshold
```

## 🧪 **Testing & Validation**

### **Automated Testing**

```bash
make test               # Complete system validation
# ✅ Prerequisites check
# ✅ Workspace integrity
# ✅ Launch script validation
# ✅ ROS2 node connectivity
```

### **Manual Testing**

```bash
# Build verification
cd slam_collaboratif/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Node testing
ros2 launch slam_map_merge multi_agent_diamants.launch.py
ros2 topic list | grep -E "(map|slam|cf)"
```

### **Performance Monitoring**

```bash
# System resources
make status-tmux        # TMUX session status
htop                    # CPU/RAM usage
nvidia-smi              # GPU usage (if available)

# ROS2 diagnostics
ros2 node list          # Active nodes
ros2 topic hz /map_merged  # Map update frequency
```

## 🐛 **Troubleshooting**

### **Common Issues & Solutions**

| Issue | Solution | Command |
|-------|----------|---------|
| TMUX not found | Auto-install | `make launch-tmux` |
| ROS2 not sourced | Auto-handled | `make build` |
| Build failures | Clean rebuild | `make clean && make build` |
| Gazebo crashes | GPU drivers | Check OpenGL support |
| RVIZ not starting | Display config | `export DISPLAY=:0` |

### **Debug Mode**

```bash
# Verbose logging
export ROS_LOG_LEVEL=DEBUG
make launch-tmux

# Manual troubleshooting
tmux attach -t slam_collab
# Navigate to problematic window
# Check logs and restart components
```

### **Clean Reset**

```bash
make kill-tmux          # Stop all processes
make clean              # Remove build artifacts  
make                    # Fresh complete setup
```

## 📊 **System Monitoring**

### **TMUX Status Dashboard**

```bash
make status-tmux
# 📊 Shows:
# - Active session status
# - Window count and states
# - Process health
# - Connection commands
```

### **Log Locations**

```bash
# ROS2 system logs
slam_collaboratif/ros2_ws/log/latest/

# Individual node logs  
~/.ros/log/

# TMUX session logs
# Available via tmux capture-pane commands
```
## 🔗 **Integration & Development**

### **Frontend Integration**

```bash
# Backend provides WebSocket endpoint for frontend
# Location: ../DIAMANTS_FRONTEND/Mission_system/
# Connection: ws://localhost:8080/ws
# API: RESTful endpoints for control
```

### **Development Workflow**

```bash
# Development setup
make dev-setup          # Install development tools
make dev-test           # Extended testing with linting

# Adding new packages
cd slam_collaboratif/ros2_ws/src/
ros2 pkg create my_package
# Edit package.xml and CMakeLists.txt
make build              # Rebuild workspace
```

### **Custom Launch Configurations**

```bash
# Direct launcher options
./launch_slam_collaborative.sh         # Interactive menu
./launch_slam_collaborative.sh tmux    # Direct TMUX launch
./launch_slam_collaborative.sh web     # Web interface only
./launch_slam_collaborative.sh slam    # SLAM system only
```

## 🛡️ **Security & Licensing**

### **MIT License Framework**

```bash
# License compliance
.github/LICENSE_HEADER_*.txt    # Template headers
scripts/add_license_header.sh   # Automated header addition
scripts/check_license_compliance.sh  # Compliance verification
```

### **Security Features**

- ✅ **Credentials secured**: No hardcoded passwords
- ✅ **Development tools removed**: No potentially dangerous scripts
- ✅ **Input validation**: Secured API endpoints  
- ✅ **Process isolation**: TMUX session separation

## 📚 **Documentation & Resources**

### **Project Documentation**

- **USAGE_GUIDE.md**: Detailed operational procedures
- **CONTRIBUTING.md**: Development contribution guidelines
- **SECURITY_AUDIT_REPORT.md**: Security assessment results

### **External Resources**

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [TMUX Documentation](https://github.com/tmux/tmux/wiki)

## 🤝 **Contributing**

This system focuses on **collaborative SLAM research**. Contributions should align with:

- Multi-agent mapping algorithms
- Stigmergie-based coordination
- Performance optimization
- Security enhancements

## ⚠️ Known Issues & Quick Fixes

### 🔧 **TMUX Session Problems**
```bash
# Issue: TMUX session fails to start
# Check dependencies and restart:
make kill-tmux
sudo apt update && sudo apt install -y tmux
make launch-tmux

# Issue: ROS2 nodes not communicating
# Reset ROS domain and restart:
export ROS_DOMAIN_ID=0
make restart

# Issue: Gazebo phantom processes
# Clean system and restart:
pkill -f gz || pkill -f gazebo || true
pkill -f rviz2 || true
make launch-tmux
```

### 🚁 **Simulation Issues**
```bash
# Issue: Gazebo simulation not loading
# Check Gazebo installation:
gz sim --version
sudo apt install -y gz-garden

# Issue: Drones not spawning correctly
# Reset simulation environment:
make kill-tmux
rm -rf /tmp/.gazebo/
make launch-tmux

# Issue: SLAM map not updating
# Check ROS2 topics:
ros2 topic list | grep map
ros2 topic echo /slam/map --once
```

### 🗺️ **SLAM System Problems**
```bash
# Issue: Map fusion not working
# Restart map merger component:
tmux send-keys -t slam_collab:3 C-c
tmux send-keys -t slam_collab:3 "ros2 run slam_map_merge map_merger" Enter

# Issue: RVIZ not displaying maps
# Reset RVIZ configuration:
pkill -f rviz2
tmux send-keys -t slam_collab:8 "rviz2 -d config/rviz_slam_optimized.rviz" Enter
```

### 🌐 **WebSocket Bridge Issues**
```bash
# Issue: Frontend-Backend communication lost
# Restart WebSocket bridge:
tmux send-keys -t slam_collab:2 C-c
tmux send-keys -t slam_collab:2 "python3 core/web_interface/websocket_bridge.py" Enter

# Issue: Port conflicts
# Check and free ports:
lsof -ti:8765 | xargs kill -9 2>/dev/null  # WebSocket
lsof -ti:7400 | xargs kill -9 2>/dev/null  # Gazebo
```

### 🔍 **Diagnostic Commands**
```bash
# System health check
make status-tmux
ros2 topic list
tmux list-sessions

# Performance monitoring
htop | grep -E "(gazebo|rviz|ros)"
free -h
df -h
```

See **CONTRIBUTING.md** for detailed guidelines and comprehensive bug documentation.

---

## 📞 **Quick Reference Card**

```bash
# 🚀 ESSENTIAL COMMANDS
make                    # Complete setup after git clone
make launch-tmux        # Launch collaborative SLAM system  
make status-tmux        # Check system status
make kill-tmux          # Stop system cleanly

# 🔧 TMUX NAVIGATION
tmux attach -t slam_collab    # Connect to session
Ctrl+b + [0-9]               # Switch windows
Ctrl+b + d                   # Detach (system keeps running)

# 🛠️ MAINTENANCE  
make clean && make      # Fresh rebuild
make info              # Project information
make help              # All available commands
```

---

**DIAMANTS V3** - *Drone Intelligence for Advanced Mapping and Navigation Through Swarms*

**Production-Ready Collaborative SLAM System** | **MIT Licensed** | **Security Hardened**

*Version 3.0.0 - Automated TMUX Orchestration*
