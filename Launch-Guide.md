# 🚀 Launch Guide

Step-by-step guide to start the complete DIAMANTS system.

## 🎯 Quick Start

### One-Command Launch

```bash
# Launch complete DIAMANTS system
./launch_diamants.sh
```

This script automatically starts all system components in the correct order.

### Manual Component Launch

For development or debugging, you can launch components individually:

```bash
# Terminal 1: Backend (ROS2)
cd DIAMANTS_BACKEND
./launch_slam_collaborative.sh

# Terminal 2: API Bridge
cd DIAMANTS_API
python launcher.py

# Terminal 3: Frontend
cd DIAMANTS_FRONTEND/Mission_system
npm run dev
```

## 🔧 Pre-Launch Checklist

### System Requirements Check

```bash
# Verify all dependencies
./check_ros_processes.sh

# Expected output:
# ✅ ROS2 Jazzy: Running
# ✅ Gazebo Garden: Available
# ✅ Python 3.10+: Available
# ✅ Node.js 18+: Available
# ✅ Hardware: [Detected/Simulation]
```

### Environment Variables

```bash
# Essential environment variables
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/DIAMANTS_BACKEND/models

# Optional for development
export DIAMANTS_DEBUG=true
export DIAMANTS_LOG_LEVEL=INFO
```

## 🏗 Component-by-Component Launch

### 1. Backend Launch (ROS2 + Gazebo)

```bash
cd DIAMANTS_BACKEND

# Option A: Simulation only
ros2 launch slam_collaboratif simulation.launch.py

# Option B: Real hardware
ros2 launch slam_collaboratif hardware.launch.py

# Option C: Mixed (some simulated, some real)
ros2 launch slam_collaboratif mixed.launch.py \
    sim_drones:=2 \
    real_drones:=1
```

**Launch Parameters:**
- `world:=empty_world` - Gazebo world to use
- `num_drones:=3` - Number of drones to spawn
- `gui:=true` - Show Gazebo GUI
- `rviz:=true` - Launch RViz visualization

**Verification:**
```bash
# Check ROS2 nodes
ros2 node list

# Expected nodes:
# /gazebo
# /cf2x_01/controller
# /cf2x_02/controller
# /swarm_coordinator
# /mission_executor

# Check topics
ros2 topic list | grep -E "(pose|cmd_vel|battery)"
```

### 2. API Bridge Launch

```bash
cd DIAMANTS_API

# Activate virtual environment
source venv/bin/activate

# Start API server
python launcher.py

# Alternative: Development mode with auto-reload
uvicorn api.main:app --reload --host 0.0.0.0 --port 8080
```

**Configuration Options:**
```bash
# Custom port
python launcher.py --port 8081

# Debug mode
python launcher.py --debug

# Specific ROS2 domain
python launcher.py --ros-domain 42

# SSL/HTTPS mode
python launcher.py --ssl-cert cert.pem --ssl-key key.pem
```

**Verification:**
```bash
# Check API health
curl http://localhost:8080/api/health

# Expected response:
# {
#   "status": "healthy",
#   "version": "1.0.0",
#   "ros2_connected": true,
#   "uptime": 30.5
# }

# Check WebSocket
curl -H "Connection: Upgrade" -H "Upgrade: websocket" \
     http://localhost:8080/ws
```

### 3. Frontend Launch

```bash
cd DIAMANTS_FRONTEND/Mission_system

# Development server
npm run dev

# Production build + serve
npm run build
npm run preview

# Custom host/port
npm run dev -- --host 0.0.0.0 --port 3001
```

**Build Options:**
```bash
# Development build (fast, debugging)
npm run dev

# Production build (optimized)
npm run build

# Check build output
ls -la dist/
```

**Verification:**
- Open browser: http://localhost:3000
- Check console for errors (F12)
- Verify 3D scene loads
- Confirm WebSocket connection in Network tab

## 🎮 System Startup Sequence

### Automatic Startup Order

The `launch_diamants.sh` script follows this sequence:

1. **Environment Check** (5 seconds)
   - Verify dependencies
   - Check hardware connections
   - Validate configuration

2. **Backend Startup** (10-15 seconds)
   - Launch ROS2 nodes
   - Start Gazebo simulation
   - Initialize drone controllers

3. **API Bridge Startup** (3-5 seconds)
   - Start FastAPI server
   - Establish ROS2 connections
   - Initialize WebSocket server

4. **Frontend Startup** (2-3 seconds)
   - Start development server
   - Build and serve assets
   - Connect to API

5. **System Integration** (5 seconds)
   - Verify all connections
   - Run integration tests
   - Ready for operation

### Manual Startup Timing

When launching manually, wait for each component to fully initialize:

```bash
# 1. Start backend first
cd DIAMANTS_BACKEND && ./launch_slam_collaborative.sh
sleep 15  # Wait for ROS2 nodes to initialize

# 2. Start API bridge
cd ../DIAMANTS_API && python launcher.py &
sleep 5   # Wait for API server to start

# 3. Start frontend
cd ../DIAMANTS_FRONTEND/Mission_system && npm run dev &
sleep 3   # Wait for dev server

# 4. Verify system
curl http://localhost:8080/api/health
```

## 🔍 Launch Verification

### System Health Check

```bash
# Comprehensive system check
./scripts/system_health_check.sh

# Manual verification commands
ros2 topic hz /cf2x_01/pose                    # Should show ~50 Hz
curl http://localhost:8080/api/drones          # Should list drones
wget -qO- http://localhost:3000 | grep -q DIAMANTS  # Frontend loaded
```

### Component Status

**Backend Status:**
```bash
# ROS2 node status
ros2 node info /swarm_coordinator

# Gazebo status
gz service -s /gazebo/pause | grep -q "false"  # Should be running

# Hardware status (if applicable)
ros2 topic echo --once /cf2x_01/battery_state
```

**API Status:**
```bash
# API endpoints
curl http://localhost:8080/api/health
curl http://localhost:8080/api/drones
curl http://localhost:8080/docs  # OpenAPI documentation

# WebSocket test
wscat -c ws://localhost:8080/ws
```

**Frontend Status:**
```bash
# Development server
curl -I http://localhost:3000  # Should return 200 OK

# Check for errors in browser console
# No WebGL errors, no JavaScript exceptions
```

## 🛠 Launch Modes

### Development Mode

Optimized for development with debugging tools:

```bash
./launch_diamants.sh --dev

# Enables:
# - Hot reload for all components
# - Debug logging
# - Development tools
# - Fast startup (minimal safety checks)
```

### Production Mode

Optimized for stable operation:

```bash
./launch_diamants.sh --prod

# Enables:
# - Optimized builds
# - Error recovery
# - Performance monitoring
# - Full safety checks
```

### Demo Mode

Pre-configured for demonstrations:

```bash
./launch_diamants.sh --demo

# Includes:
# - Pre-defined missions
# - Attractive visualizations
# - Auto-start scenarios
# - Audience-friendly interface
```

### Testing Mode

For automated testing:

```bash
./launch_diamants.sh --test

# Features:
# - Minimal UI
# - Accelerated time
# - Automated scenarios
# - Test result reporting
```

## 🚁 Hardware-Specific Launch

### Crazyflie Configuration

```bash
# Single Crazyflie
ros2 launch slam_collaboratif crazyflie_single.launch.py \
    uri:=radio://0/80/2M/E7E7E7E701

# Multiple Crazyflies
ros2 launch slam_collaboratif crazyflie_multi.launch.py \
    cf1_uri:=radio://0/80/2M/E7E7E7E701 \
    cf2_uri:=radio://0/80/2M/E7E7E7E702 \
    cf3_uri:=radio://0/80/2M/E7E7E7E703
```

### Hardware Detection

```bash
# Check Crazyradio PA
lsusb | grep "1915:7777"

# Check drone connections
python3 -c "
import cflib.crtp
cflib.crtp.init_drivers()
available = cflib.crtp.scan_interfaces()
print(f'Found {len(available)} Crazyflies')
for i in available:
    print(f'  - {i[0]}')
"
```

## 🌐 Network Configuration

### Multi-Machine Setup

**Machine 1 (Control Station):**
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Launch frontend + API
./launch_control_station.sh
```

**Machine 2 (Simulation Server):**
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export DISPLAY=:0  # For Gazebo GUI

# Launch backend only
./launch_simulation_server.sh
```

**Machine 3 (Field Computer):**
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Launch hardware interface only
./launch_field_computer.sh
```

### Firewall Configuration

```bash
# Open required ports
sudo ufw allow 3000/tcp   # Frontend
sudo ufw allow 8080/tcp   # API
sudo ufw allow 7400/udp   # ROS2 discovery
sudo ufw allow 7401/udp   # ROS2 discovery
sudo ufw allow 7410-7500/udp  # ROS2 data

# For Gazebo remote
sudo ufw allow 11345/tcp  # Gazebo master
```

## 🚨 Troubleshooting Launch Issues

### Common Launch Failures

**ROS2 Nodes Won't Start:**
```bash
# Check environment
echo $ROS_DOMAIN_ID        # Should be set
ros2 daemon stop && ros2 daemon start  # Reset daemon

# Check for conflicts
ps aux | grep ros2         # Kill conflicting processes
```

**Gazebo Won't Launch:**
```bash
# Check GPU acceleration
nvidia-smi  # NVIDIA users
glxinfo | grep "direct rendering"

# Clear Gazebo cache
rm -rf ~/.gazebo/models/.database.config
```

**API Connection Failed:**
```bash
# Check port availability
sudo netstat -tulpn | grep :8080

# Check Python environment
which python3              # Should be virtual env
pip list | grep fastapi    # Should be installed
```

**Frontend Build Errors:**
```bash
# Clear Node.js cache
npm cache clean --force
rm -rf node_modules package-lock.json
npm install

# Check Node.js version
node --version             # Should be 18+
```

### Emergency Procedures

**Complete System Reset:**
```bash
# Stop all components
./stop_diamants.sh

# Clear all caches and temporary files
./scripts/clean_system.sh

# Restart with diagnostic mode
./launch_diamants.sh --diagnostic
```

**Emergency Drone Landing:**
```bash
# Emergency stop all drones
ros2 service call /emergency_stop std_srvs/srv/Empty

# Or via API
curl -X POST http://localhost:8080/api/emergency/stop_all
```

## ✅ Launch Success Indicators

### Green Light Checklist

- [ ] All ROS2 nodes running (`ros2 node list`)
- [ ] Gazebo simulation active (if enabled)
- [ ] API health check passes (`curl /api/health`)
- [ ] Frontend loads without errors
- [ ] WebSocket connection established
- [ ] Drones responding to commands
- [ ] Telemetry data flowing
- [ ] No error messages in logs

### Performance Targets

- **Backend startup**: < 15 seconds
- **API response time**: < 100ms
- **Frontend FPS**: > 30 FPS
- **WebSocket latency**: < 50ms
- **Command execution**: < 200ms

🎉 **System Ready!** Your DIAMANTS swarm is operational and ready for missions.