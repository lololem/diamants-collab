# 🔧 Installation Guide

Complete installation guide for the DIAMANTS system on Ubuntu 22.04 LTS.

## 🎯 Prerequisites

### System Requirements

**Hardware:**
- CPU: Intel i5/AMD Ryzen 5 or better (8+ cores recommended)
- RAM: 16GB minimum (32GB recommended for simulation)
- GPU: NVIDIA GTX 1060 or better (for 3D visualization)
- Storage: 50GB free space
- Network: Gigabit Ethernet (for multi-drone communication)

**Software:**
- Ubuntu 22.04 LTS (Primary support)
- Ubuntu 20.04 LTS (Community support)
- Python 3.10+
- Node.js 18+
- Git 2.30+

### Supported Drones

- **Crazyflie 2.X** (Primary platform)
- **Crazyflie 2.1** (Full support)
- **Custom drones** (With ROS2 driver)

## 🚀 Automated Installation

### One-Command Setup

```bash
# Download and run installation script
curl -fsSL https://raw.githubusercontent.com/lololem/diamants-collab/main/install.sh | bash
```

### Manual Installation Steps

#### 1. Install ROS2 Jazzy

```bash
# Add ROS2 APT repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop-full

# Environment setup
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install development tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

#### 2. Install Dependencies

```bash
# System dependencies
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    nodejs \
    npm \
    curl \
    wget \
    htop \
    tmux \
    tree

# Python dependencies
pip3 install --upgrade pip
pip3 install \
    fastapi \
    uvicorn \
    websockets \
    pydantic \
    numpy \
    scipy \
    matplotlib \
    opencv-python

# Node.js dependencies (global)
sudo npm install -g \
    vite \
    eslint \
    prettier
```

#### 3. Install Gazebo Garden

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Garden
sudo apt update
sudo apt install gz-garden

# ROS2-Gazebo bridge
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
```

#### 4. Clone and Build DIAMANTS

```bash
# Clone repository
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab

# Setup and build
./setup.sh
```

## 🔧 Component Installation

### Backend (ROS2)

```bash
cd DIAMANTS_BACKEND

# Build ROS2 workspace
cd slam_collaboratif/ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

### API Bridge (FastAPI)

```bash
cd DIAMANTS_API

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Test installation
python test_api_integration.py
```

### Frontend (Three.js)

```bash
cd DIAMANTS_FRONTEND/Mission_system

# Install dependencies
npm install

# Build frontend
npm run build

# Development server
npm run dev
```

## 🧪 Verification & Testing

### System Health Check

```bash
# Run comprehensive system check
./check_ros_processes.sh

# Expected output:
# ✅ ROS2 Jazzy: Installed
# ✅ Gazebo Garden: Installed
# ✅ Python dependencies: OK
# ✅ Node.js dependencies: OK
# ✅ DIAMANTS build: Success
```

### Component Tests

```bash
# Test ROS2 backend
cd DIAMANTS_BACKEND
ros2 launch slam_collaboratif test_system.launch.py

# Test API bridge
cd DIAMANTS_API
python -m pytest tests/

# Test frontend
cd DIAMANTS_FRONTEND/Mission_system
npm test
```

### Integration Test

```bash
# Launch complete system test
./run_integration_test.sh

# Expected: All 3 components communicate successfully
```

## 🔌 Hardware Setup

### Crazyflie Configuration

```bash
# Install Crazyflie Python library
pip3 install cflib

# Crazyflie ROS2 driver
cd DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/src
git clone https://github.com/IMRCLab/crazyswarm2.git
cd ../..
colcon build --packages-select crazyflie_py crazyflie_interfaces

# Configure radio
# Connect Crazyradio PA to USB
lsusb | grep "1915:7777"  # Should show Crazyradio
```

### Network Configuration

```bash
# Configure network for multi-machine setup
export ROS_DOMAIN_ID=42

# For multiple computers
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Add to ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> ~/.bashrc
```

## 🚨 Troubleshooting

### Common Issues

#### ROS2 Not Found
```bash
# Symptom: ros2 command not found
# Solution:
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

#### Gazebo Launch Fails
```bash
# Symptom: Gazebo crashes on launch
# Check GPU drivers:
nvidia-smi  # For NVIDIA users
sudo apt install mesa-utils && glxinfo | grep OpenGL

# Solution: Update GPU drivers
sudo apt install nvidia-driver-510  # NVIDIA
sudo apt install mesa-vulkan-drivers  # AMD/Intel
```

#### Python Import Errors
```bash
# Symptom: ModuleNotFoundError
# Ensure virtual environment:
cd DIAMANTS_API
source venv/bin/activate
pip install -r requirements.txt
```

#### WebSocket Connection Failed
```bash
# Symptom: Frontend can't connect to API
# Check firewall:
sudo ufw allow 8080
sudo ufw allow 3000

# Check services:
netstat -tulpn | grep :8080
netstat -tulpn | grep :3000
```

#### Crazyflie Not Detected
```bash
# Check USB permissions:
sudo usermod -a -G dialout $USER
sudo udev reload-rules

# Check radio status:
lsusb | grep "1915:7777"
dmesg | tail -20
```

### Performance Issues

#### Low FPS in 3D Interface
```bash
# Check GPU acceleration:
glxinfo | grep "direct rendering"

# Enable hardware acceleration:
export MESA_GL_VERSION_OVERRIDE=4.5
export MESA_GLSL_VERSION_OVERRIDE=450
```

#### High CPU Usage
```bash
# Limit ROS2 CPU usage:
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI="<CycloneDX><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDX>"
```

### Log Analysis

```bash
# ROS2 logs
ros2 topic echo /rosout

# API logs
tail -f DIAMANTS_API/logs/api.log

# Frontend logs (browser console)
# Open browser DevTools (F12) and check Console tab

# System logs
journalctl -u ros2-daemon
dmesg | grep -i error
```

## 🔒 Security Configuration

### Firewall Setup

```bash
# Configure UFW for DIAMANTS
sudo ufw enable
sudo ufw allow 8080/tcp   # API
sudo ufw allow 3000/tcp   # Frontend
sudo ufw allow 7400/udp   # ROS2 discovery
sudo ufw allow 7401/udp   # ROS2 discovery
```

### SSL/TLS (Production)

```bash
# Generate certificates for production
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365

# Configure API for HTTPS
export DIAMANTS_SSL_CERT="cert.pem"
export DIAMANTS_SSL_KEY="key.pem"
```

## 📊 Performance Tuning

### System Optimization

```bash
# Increase shared memory for ROS2
echo "kernel.shmmax = 268435456" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p

# Optimize network buffers
echo "net.core.rmem_max = 134217728" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max = 134217728" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### Real-time Configuration

```bash
# For real-time performance (optional)
sudo apt install linux-lowlatency
echo "@realtime soft rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime hard rtprio 99" | sudo tee -a /etc/security/limits.conf
```

## ✅ Installation Verification

### Final Checklist

- [ ] ROS2 Jazzy installed and sourced
- [ ] Gazebo Garden running without errors
- [ ] Python virtual environment activated
- [ ] Node.js dependencies installed
- [ ] All DIAMANTS components built successfully
- [ ] Network configuration complete
- [ ] Hardware (if applicable) detected
- [ ] System tests passing

### Success Indicators

```bash
# All these commands should work:
ros2 --version                    # Shows ROS2 Jazzy
gz --version                      # Shows Gazebo Garden
python3 -c "import fastapi"       # No import error
node --version                    # Shows Node.js 18+
./launch_diamants.sh --dry-run    # Shows launch plan
```

🎉 **Installation Complete!** Your DIAMANTS system is ready. Next step: [Launch Guide](Launch-Guide)