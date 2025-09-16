# 🚁 DIAMANTS - Technical Stack & Configuration

**Date de création :** 3 Septembre 2025  
**Project :** DIAMANTS (Distributed Autonomous Multi-agents Systems)  
**Version :** Current  
**Description :** Advanced autonomous drone swarm coordination platform with integrated ROS2 and WebGL visualization  
**Auteur :** lololem  

---

## 📋 **Table des Matières**

1. [Vue d'ensemble](#vue-densemble)
2. [Architecture Système](#architecture-système)
3. [Technologies Frontend](#technologies-frontend)
4. [Technologies Backend](#technologies-backend)
5. [Intelligence Artificielle](#intelligence-artificielle)
6. [Configuration Environnement](#configuration-environnement)
7. [Structure Projet](#structure-projet)
8. [Scripts et Outils](#scripts-et-outils)
9. [Déploiement](#déploiement)
10. [Performance](#performance)

---

## 🎯 **Vue d'ensemble**

DIAMANTS est une plateforme technologique avancée de simulation multi-agents pour drones collaboratifs, intégrant :

- **Simulation temps réel** de drones Crazyflie
- **Intelligence collective** avec algorithmes de stigmergie
- **Rendu 3D WebGL** haute performance
- **Communication ROS2** bidirectionnelle
- **SLAM collaboratif** pour navigation autonome

---

## 🏗️ **Architecture Système**

### **Pattern Architectural**
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

### **Flux de Données**
1. **Frontend → WebSocket** : Commandes utilisateur (takeoff, land, navigation)
2. **WebSocket → ROS2** : Translation des commandes en messages ROS2
3. **ROS2 → Agents** : Distribution aux agents drones
4. **Agents → ROS2** : Télémétrie et données capteurs
5. **ROS2 → WebSocket** : Agrégation données simulation
6. **WebSocket → Frontend** : Mise à jour interface temps réel

---

## 💻 **Technologies Frontend**

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

### **Interface Utilisateur**
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

## 🤖 **Technologies Backend**

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

---

## 🧠 **Intelligence Artificielle**

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

## ⚙️ **Configuration Environnement**

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

## 📁 **Structure Projet**

### **Architecture Modulaire**
```
DIAMANTS/
├── mission_core/
│   └── Mission_system_v1/          # Frontend principal
│       ├── behaviors/              # Comportements IA
│       │   ├── flight-behaviors.js
│       │   └── advanced-collective-intelligence.js
│       ├── drones/                 # Modèles drones
│       │   └── authentic-crazyflie.js
│       ├── environment/            # Environnement 3D
│       │   └── quality-control-panel.js
│       ├── intelligence/           # IA collective
│       ├── tools/                  # Outils système
│       │   └── integrated-controller.js
│       ├── main.js                 # Point d'entrée principal
│       ├── index.html             # Interface web
│       ├── package.json           # Dépendances NPM
│       └── .nvmrc                 # Version Node.js
│
├── backend/                        # Backend ROS2
│   ├── diamants_unified_bridge.py # Bridge principal
│   ├── crazyflie_bridge.py        # (deprecated)
│   └── improved_crazyflie_bridge.py # (deprecated)
│
├── scripts/                        # Scripts automation
│   ├── start-console-ninja.sh     # Démarrage debug
│   └── deployment/                # Scripts déploiement
│
├── submodules/                     # Sous-modules Git
│   └── grass-shader-glsl/         # Shaders herbe
│
├── Unity3D-DIAMANTS/              # Version Unity (legacy)
├── performance_engines/           # Moteurs performance
├── prototyping/                   # Prototypes R&D
└── DIAMANTS_TECH_STACK.md        # Cette documentation
```

### **Fichiers Clés**
```yaml
Configuration:
  - ".nvmrc": "Version Node.js"
  - "package.json": "Dépendances frontend"
  - "vite.config.js": "Configuration build"
  - ".env": "Variables environnement"

Code_Principal:
  - "main.js": "Application principale"
  - "index.html": "Interface utilisateur"
  - "authentic-crazyflie.js": "Simulation drone"
  - "diamants_unified_bridge.py": "Bridge ROS2"

Comportements_IA:
  - "flight-behaviors.js": "Comportements vol"
  - "advanced-collective-intelligence.js": "IA collective"
  - "integrated-controller.js": "Contrôleur unifié"
```

---

## 🛠️ **Scripts et Outils**

### **Scripts de Développement**
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

### **Scripts Shell**
```bash
# start-console-ninja.sh
#!/bin/bash
echo "🚀 Démarrage Console Ninja pour DIAMANTS..."
nvm use 16.15.0
echo "🔧 Node.js: $(node -v)"
echo "🧹 Nettoyage des processus Vite existants..."
pkill -f vite
echo "🌟 Lancement de Vite avec Console Ninja..."
npm run dev
```

### **Outils de Debug**
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

## 🚀 **Déploiement**

### **Development Environment**
```bash
# Installation
git clone https://github.com/lololem/diamants.git
cd diamants/mission_core/Mission_system_v1
nvm use 16.15.0
npm install

# Démarrage
npm run dev          # Vite dev server
# ou
./scripts/start-console-ninja.sh  # Avec Console Ninja
```

### **Production Build**
```bash
# Build optimisé
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

### **Métriques Cibles**
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

### **Issues Communs**
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
  - "✅ SLAM collaborative"
  - "🔄 Debug infrastructure (en cours)"
  - "⏳ Performance optimization (plannifié)"
```

### **Futures Versions**
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

## 📚 **Documentation Additionnelle**

### **Références Techniques**
- **Three.js Documentation**: https://threejs.org/docs/
- **ROS2 Documentation**: https://docs.ros.org/
- **Vite Documentation**: https://vitejs.dev/
- **WebGL Specification**: https://www.khronos.org/webgl/

### **Recherche & Papers**
- **Swarm Robotics**: Craig Reynolds - Flocks, Herds, and Schools
- **SLAM**: Durrant-Whyte & Bailey - Simultaneous Localization and Mapping
- **Multi-Agent Systems**: Wooldridge - An Introduction to MultiAgent Systems

---

**📝 Dernière mise à jour:** 3 Septembre 2025  
**🏷️ Version:** Current Release  
**👨‍💻 Maintainer:** lololem  
**📧 Contact:** Voir repository GitHub

---

*Cette documentation est un document vivant, mis à jour régulièrement avec l'évolution du projet DIAMANTS.*
