# Installation

## Frontend only (recommended start)

Requirements:
- Node.js 20+ (use nvm)
- npm
- A modern browser with WebGL support

```bash
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab/DIAMANTS_FRONTEND/Mission_system
npm install
npm run dev
```

The dev server starts on http://localhost:5550. The simulation runs immediately — no backend needed.

### Node.js via nvm

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
source ~/.bashrc
nvm install 20
nvm use 20
```

## Full stack (frontend + backend)

The backend adds ROS2-based SLAM, Gazebo physics simulation, and multi-drone coordination. It is optional.

### System requirements

- Ubuntu 22.04 or 24.04
- 16 GB RAM minimum (32 GB recommended for Gazebo)
- GPU with OpenGL 4.5+ (for Gazebo rendering)
- 20 GB free disk space

### 1. Install ROS2 Jazzy

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

### 2. Install Gazebo Harmonic (optional)

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic
sudo apt install -y ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
```

### 3. Build the backend

```bash
cd diamants-collab/DIAMANTS_BACKEND
./setup.sh
```

Or manually:

```bash
cd diamants-collab/DIAMANTS_BACKEND/slam_collaboratif/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Install the API bridge (optional)

```bash
cd diamants-collab/DIAMANTS_API
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Verify installation

```bash
# Frontend
cd DIAMANTS_FRONTEND/Mission_system
npm run build   # Should produce dist/ with no errors

# Backend (if installed)
ros2 --version  # Should show ROS2 Jazzy

# Gazebo (if installed)
gz sim --version
```

## Troubleshooting

### `npm install` fails

```bash
rm -rf node_modules package-lock.json
npm install
```

### `THREE is not defined`

The frontend uses ES6 modules. Make sure you're using `npm run dev` (Vite), not opening `index.html` directly in a browser.

### WebSocket connection refused

This is normal if no backend is running. The frontend falls back to standalone mode (CAS 2). The WebSocket errors in the console can be ignored.

### Gazebo crashes on launch

Check GPU drivers:

```bash
glxinfo | grep "direct rendering"   # Should say "Yes"
nvidia-smi                           # For NVIDIA GPUs
```

### ROS2 topics not visible

```bash
source /opt/ros/jazzy/setup.bash
source DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/install/setup.bash
ros2 topic list
```
