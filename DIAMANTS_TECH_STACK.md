# 🚁 DIAMANTS - Technical Stack & Configuration

**Creation Date:** September 3, 2025  
**Project:** DIAMANTS (Distributed Autonomous Multi-agents Systems)  
**Version:** Current  
**Description:** Advanced autonomous drone swarm coordination platform with integrated ROS2 and WebGL visualization  
**Author:** lololem  

---

## 📋 **Table of Contents**

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Frontend Technologies](#frontend-technologies)
4. [Backend Technologies](#backend-technologies)
5. [Artificial Intelligence](#artificial-intelligence)
6. [Environment Configuration](#environment-configuration)
7. [Project Structure](#project-structure)
8. [Scripts and Tools](#scripts-and-tools)
9. [Deployment](#deployment)
10. [Performance](#performance)

---

## 🎯 **Overview**

DIAMANTS is an advanced technological platform for multi-agent simulation of collaborative drones, integrating:

- **Real-time simulation** of Crazyflie drones
- **Collective intelligence** with stigmergy algorithms
- **High-performance WebGL 3D rendering**
- **Bidirectional ROS2 communication**
- **Collaborative SLAM** for autonomous navigation

---

## 🏗️ **System Architecture**

### **Architectural Pattern**
```
┌─────────────────────────────────────┐
│           Frontend Web              │
│    (Vite + Three.js + WebGL)       │
└─────────────────┬───────────────────┘
                  │ WebSocket
┌─────────────────┴───────────────────┐
│          WebSocket Bridge           │
│    (Python + ROS2 + rclpy)         │
└─────────────────┬───────────────────┘
                  │ ROS2 Messages
┌─────────────────┴───────────────────┐
│           ROS2 Backend              │
│   (Multi-Agent + SLAM + Physics)    │
└─────────────────────────────────────┘
```

### **Data Flow**
1. **Frontend → WebSocket**: User commands (takeoff, land, navigation)
2. **WebSocket → ROS2**: Translation of commands to ROS2 messages
3. **ROS2 → Agents**: Distribution to drone agents
4. **Agents → ROS2**: Telemetry and sensor data
5. **ROS2 → WebSocket**: Simulation data aggregation
6. **WebSocket → Frontend**: Real-time interface updates

---

## 💻 **Frontend Technologies**

### **Core Runtime**
```yaml
Node.js:
  version: "16.15.0"
  source: ".nvmrc"
  package_manager: "npm v8.5.5"

Build_System:
  tool: "Vite"
  version: "4.5.3"
  features:
    - "Hot Module Replacement (HMR)"
    - "ES6 Modules Support"
    - "TypeScript Support (ready)"
    - "Development Server"
    - "Production Bundling"
```

### **3D Graphics Pipeline**
```yaml
Three.js:
  version: "^0.158.0"
  components:
    - Scene: "3D scene management"
    - Camera: "PerspectiveCamera with orbital controls"
    - Renderer: "WebGLRenderer with shadows"
    - Lighting: "DirectionalLight + AmbientLight"
    - Materials: "MeshStandardMaterial, ShaderMaterial"
    - Geometries: "Custom drone models, terrain mesh"

WebGL:
  version: "2.0"
  features:
    - "Hardware acceleration"
    - "Custom GLSL shaders"
    - "Instanced rendering"
    - "Shadow mapping"

GLSL_Shaders:
  grass_shader:
    location: "submodules/grass-shader-glsl/"
    purpose: "Realistic grass rendering"
    features: ["Wind animation", "LOD system", "Instancing"]
```

### **User Interface**
```yaml
HTML5:
  canvas: "WebGL rendering surface"
  controls: "Interactive UI elements"
  semantic: "Structured document"

CSS3:
  features:
    - "Flexbox layouts"
    - "CSS Grid"
    - "Animations & Transitions"
    - "Responsive design"
    - "Custom properties (variables)"

JavaScript_ES6+:
  features:
    - "Modules (import/export)"
    - "Classes & Inheritance"
    - "Async/Await"
    - "Arrow functions"
    - "Destructuring"
    - "Template literals"
```

### **Development Tools**
```yaml
Console_Ninja:
  purpose: "Real-time debugging"
  features:
    - "Live log streaming"
    - "Error tracking"
    - "Performance monitoring"
    - "WebSocket message inspection"

Hot_Reload:
  tool: "Vite HMR"
  benefits:
    - "Instant updates"
    - "State preservation"
    - "Fast iteration"
```

---

## 🤖 **Backend Technologies**

### **ROS2 Core System**
```yaml
ROS2:
  distribution: "Humble/Galactic/Foxy"
  language: "Python 3.8+"
  client_library: "rclpy"

Core_Components:
  nodes:
    - "diamants_unified_bridge"
    - "crazyflie_simulation_node"
    - "slam_collaborative_node"
    - "swarm_intelligence_node"
  
  topics:
    - "/diamants/drone_state"
    - "/diamants/commands"
    - "/diamants/telemetry"
    - "/diamants/slam_data"
  
  services:
    - "/diamants/takeoff_all"
    - "/diamants/land_all"
    - "/diamants/emergency_stop"
```

### **Communication Bridge**
```yaml
WebSocket_Bridge:
  main_script: "backend/diamants_unified_bridge.py"
  deprecated_scripts:
    - "crazyflie_bridge.py"
    - "improved_crazyflie_bridge.py"
  
  protocol: "WebSocket (ws://)"
  default_port: 9090
  message_format: "JSON"
  
  features:
    - "Bidirectional communication"
    - "Message queuing"
    - "Connection management"
    - "Error handling & recovery"
```

### **Simulation Engine**
```yaml
Physics_Engine:
  type: "Custom implementation"
  frequency: "60 Hz"
  features:
    - "Real-time physics"
    - "Collision detection"
    - "Aerodynamics simulation"
    - "Sensor modeling"

Drone_Models:
  crazyflie:
    mass: "27g"
    motors: 4
    propellers: "65mm diameter"
    flight_time: "7 minutes"
    max_speed: "8 m/s"
```
    - "Bidirectional communication"
    - "Message queuing"
    - "Connection management"
    - "Error handling & recovery"
```

### **Simulation Engine**
```yaml
Physics_Engine:
  type: "Custom implementation"
  frequency: "60 Hz"
  features:
    - "Real-time physics"
    - "Collision detection"
    - "Aerodynamics simulation"
    - "Sensor modeling"

Drone_Models:
  crazyflie:
    mass: "27g"
    motors: 4
    propellers: "65mm diameter"
    flight_time: "7 minutes"
    max_speed: "8 m/s"
```

---

## 🧠 **Artificial Intelligence**

### **Multi-Agent Systems**
```yaml
Swarm_Intelligence:
  algorithms:
    - "Particle Swarm Optimization (PSO)"
    - "Boids flocking behavior"
    - "Consensus algorithms"
    - "Distributed decision making"

Stigmergy_System:
  concept: "Indirect coordination through environment"
  implementation:
    - "Digital pheromone trails"
    - "Information persistence"
    - "Emergent behaviors"
  
  force_fields:
    phi_field: "Exploration attraction (φ)"
    sigma_field: "Obstacle repulsion (σ)"
```

### **Navigation & SLAM**
```yaml
SLAM_Collaborative:
  type: "Multi-robot SLAM"
  sensors:
    - "Multi-Ranger (4-directional LiDAR)"
    - "IMU (Inertial Measurement Unit)"
    - "Altimeter"
    - "Camera (optional)"
  
  algorithms:
    - "Extended Kalman Filter (EKF)"
    - "Particle Filter"
    - "Graph-based SLAM"
    - "Loop closure detection"

Collision_Avoidance:
  method: "Velocity obstacles"
  range: "0.5 - 3.0 meters"
  update_rate: "50 Hz"
  priority: "Safety-first approach"
```

### **Behavior Systems**
```yaml
Emergent_Behaviors:
  cluster_behavior: "Formation flying"
  exploration_behavior: "Area coverage"
  follow_behavior: "Leader-follower dynamics"
  cooperative_behavior: "Task coordination"

PID_Controllers:
  position:
    kp: 1.0
    ki: 0.1
    kd: 0.05
  
  attitude:
    kp: 2.0
    ki: 0.2
    kd: 0.1
  
  altitude:
    kp: 1.5
    ki: 0.15
    kd: 0.075
```

---

## ⚙️ **Environment Configuration**

### **Node.js Configuration**
```bash
# .nvmrc
16.15.0
```

### **NPM Dependencies**
```json
{
  "dependencies": {
    "three": "^0.158.0"
  },
  "devDependencies": {
    "vite": "^4.5.3"
  }
}
```

### **Vite Configuration**
```javascript
// vite.config.js
export default {
  server: {
    port: 5173,
    host: true
  },
  build: {
    target: 'es2018',
    rollupOptions: {
      external: []
    }
  }
}
```

### **Environment Variables**
```bash
# .env
NODE_ENV=development
VITE_ROS2_WEBSOCKET_URL=ws://localhost:9090
VITE_DEBUG_MODE=true
VITE_PHYSICS_FREQUENCY=60
VITE_MAX_DRONES=6
```

---

## 📁 **Project Structure**

### **Modular Architecture**
```
DIAMANTS/
├── mission_core/
│   └── Mission_system_v1/          # Main frontend
│       ├── behaviors/              # AI behaviors
│       │   ├── flight-behaviors.js
│       │   └── advanced-collective-intelligence.js
│       ├── drones/                 # Drone models
│       │   └── authentic-crazyflie.js
│       ├── environment/            # 3D environment
│       │   └── quality-control-panel.js
│       ├── intelligence/           # Collective AI
│       ├── tools/                  # System tools
│       │   └── integrated-controller.js
│       ├── main.js                 # Main entry point
│       ├── index.html             # Web interface
│       ├── package.json           # NPM dependencies
│       └── .nvmrc                 # Node.js version
│
├── backend/                        # ROS2 backend
│   ├── diamants_unified_bridge.py # Main bridge
│   ├── crazyflie_bridge.py        # (deprecated)
│   └── improved_crazyflie_bridge.py # (deprecated)
│
├── scripts/                        # Automation scripts
│   ├── start-console-ninja.sh     # Debug startup
│   └── deployment/                # Deployment scripts
│
├── submodules/                     # Git submodules
│   └── grass-shader-glsl/         # Grass shaders
│
├── Unity3D-DIAMANTS/              # Unity version (legacy)
├── performance_engines/           # Performance engines
├── prototyping/                   # R&D prototypes
└── DIAMANTS_TECH_STACK.md        # This documentation
```

### **Key Files**
```yaml
Configuration:
  - ".nvmrc": "Node.js version"
  - "package.json": "Frontend dependencies"
  - "vite.config.js": "Build configuration"
  - ".env": "Environment variables"

Main_Code:
  - "main.js": "Main application"
  - "index.html": "User interface"
  - "authentic-crazyflie.js": "Drone simulation"
  - "diamants_unified_bridge.py": "ROS2 bridge"

AI_Behaviors:
  - "flight-behaviors.js": "Flight behaviors"
  - "advanced-collective-intelligence.js": "Collective AI"
  - "integrated-controller.js": "Unified controller"
```

---

## 🛠️ **Scripts and Tools**

### **Development Scripts**
```bash
# Package.json scripts
{
  "scripts": {
    "dev": "vite --port 5173 --host",
    "build": "vite build",
    "preview": "vite preview",
    "console-ninja": "./scripts/start-console-ninja.sh"
  }
}
```

### **Shell Scripts**
```bash
# start-console-ninja.sh
#!/bin/bash
echo "🚀 Starting Console Ninja for DIAMANTS..."
nvm use 16.15.0
echo "🔧 Node.js: $(node -v)"
echo "🧹 Cleaning existing Vite processes..."
pkill -f vite
echo "🌟 Launching Vite with Console Ninja..."
npm run dev
```

### **Debug Tools**
```yaml
Console_Ninja:
  port: 5568
  features:
    - "Live logging"
    - "Error tracking" 
    - "Performance monitoring"
    - "WebSocket inspection"

Browser_DevTools:
  extensions:
    - "Console Ninja"
    - "Three.js DevTools"
    - "WebGL Inspector"
```

---

## 🚀 **Deployment**

### **Development Environment**
```bash
# Installation
git clone https://github.com/lololem/diamants.git
cd diamants/mission_core/Mission_system_v1
nvm use 16.15.0
npm install

# Startup
npm run dev          # Vite dev server
# or
./scripts/start-console-ninja.sh  # With Console Ninja
```

### **Production Build**
```bash
# Optimized build
npm run build

# Preview production
npm run preview
```

### **ROS2 Backend**
```bash
# Terminal 1 - ROS2 Core
ros2 run diamants diamants_unified_bridge

# Terminal 2 - Frontend
npm run dev
```

### **System Requirements**
```yaml
Hardware:
  cpu: "Multi-core (4+ cores recommended)"
  memory: "8GB+ RAM"
  gpu: "WebGL 2.0 compatible"
  storage: "2GB+ available space"

Software:
  os: "Linux (Ubuntu 20.04+, Debian 11+)"
  browser: "Chrome 90+, Firefox 88+, Safari 14+"
  node: "16.15.0 (exact version)"
  ros2: "Humble/Galactic/Foxy"
  python: "3.8+"
```

---

## ⚡ **Performance**

### **Target Metrics**
```yaml
Rendering:
  fps: "60 FPS stable"
  draw_calls: "< 100 per frame"
  triangles: "< 100K per frame"
  texture_memory: "< 512MB"

Simulation:
  physics_rate: "60 Hz"
  ai_update_rate: "30 Hz"
  network_latency: "< 10ms local"
  memory_usage: "< 2GB total"

Network:
  websocket_throughput: "1000 msg/s"
  ros2_latency: "< 5ms"
  bandwidth: "< 10 MB/s"
```

### **Optimizations**
```yaml
Frontend:
  - "GPU instancing for grass rendering"
  - "LOD system for distant objects"
  - "Frustum culling"
  - "Texture atlasing"
  - "Compressed textures"

Backend:
  - "Message batching"
  - "Efficient data structures"
  - "Connection pooling"
  - "Garbage collection optimization"

Network:
  - "Binary protocol for critical data"
  - "Message compression"
  - "Delta compression for position updates"
  - "QoS policies for ROS2"
```

---

## 🔧 **Troubleshooting**

### **Common Issues**
```yaml
Syntax_Error_main.js:
  symptom: "Vite import analysis failed"
  cause: "Missing closing brace"
  solution: "Check method brackets in main.js"

WebSocket_Connection:
  symptom: "ROS2 bridge not connecting"
  cause: "Bridge not running or port conflict"
  solution: "Start diamants_unified_bridge.py, check port 9090"

Performance_Issues:
  symptom: "Low FPS, high memory usage"
  cause: "Too many draw calls or memory leaks"
  solution: "Enable LOD, check for memory leaks in DevTools"
```

### **Debug Commands**
```bash
# Check Node.js version
node -v

# Check Vite processes
ps aux | grep vite

# Check port availability
lsof -i :5173
lsof -i :9090

# Memory usage
ps -o pid,rss,command -p $(pgrep node)
```

---

## 📊 **Monitoring & Logs**

### **Log Levels**
```yaml
Debug: "Detailed system information"
Info: "General operational messages"
Warning: "Potential issues"
Error: "System errors requiring attention"
```

### **Key Metrics**
```yaml
System:
  - "FPS (frames per second)"
  - "Memory usage (MB)"
  - "Network latency (ms)"
  - "CPU usage (%)"

Simulation:
  - "Active drones count"
  - "Collision events"
  - "SLAM map quality"
  - "Behavior transitions"

Network:
  - "WebSocket messages/s"
  - "ROS2 topics frequency"
  - "Connection status"
  - "Error rates"
```

---

## 📈 **Roadmap & Evolution**

### **Current Version: Production Ready**
```yaml
Status: "Active Development"
Features:
  - "✅ Multi-agent simulation"
  - "✅ WebGL 3D rendering"
  - "✅ ROS2 integration"
  - "✅ Collaborative SLAM"
  - "🔄 Debug infrastructure (in progress)"
  - "⏳ Performance optimization (planned)"
```

### **Future Versions**
```yaml
V4:
  - "TypeScript migration"
  - "Advanced AI behaviors"
  - "Cloud deployment support"
  - "Multi-user collaboration"

V5:
  - "AR/VR interface"
  - "Hardware-in-the-loop simulation"
  - "Machine learning integration"
  - "Real drone fleet management"
```

---

## 📚 **Additional Documentation**

### **Technical References**
- **Three.js Documentation**: https://threejs.org/docs/
- **ROS2 Documentation**: https://docs.ros.org/
- **Vite Documentation**: https://vitejs.dev/
- **WebGL Specification**: https://www.khronos.org/webgl/

### **Research & Papers**
- **Swarm Robotics**: Craig Reynolds - Flocks, Herds, and Schools
- **SLAM**: Durrant-Whyte & Bailey - Simultaneous Localization and Mapping
- **Multi-Agent Systems**: Wooldridge - An Introduction to MultiAgent Systems

---

**📝 Last update:** September 3, 2025  
**🏷️ Version:** Current Release  
**👨‍💻 Maintainer:** lololem  
**📧 Contact:** See GitHub repository

---

*This documentation is a living document, regularly updated with the evolution of the DIAMANTS project.*
