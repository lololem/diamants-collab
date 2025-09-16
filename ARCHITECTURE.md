# DIAMANTS - Architecture Overview

## 🏗️ Global System Architecture

DIAMANTS (Distributed Autonomous Multi-agents Systems) follows a modular three-tier architecture designed for scalable drone swarm coordination and autonomous mission execution.

```
┌─────────────────────────────────────────────────────────────────┐
│                    DIAMANTS ECOSYSTEM                          │
├─────────────────────────────────────────────────────────────────┤
│  Frontend (Mission System)     │     API Layer     │  Backend   │
│  ┌─────────────────────────┐   │  ┌──────────────┐  │ ┌────────┐ │
│  │   Three.js WebGL        │◄──┤  │   FastAPI    │◄─┤ │ ROS2   │ │
│  │   Real-time 3D Viz     │   │  │   WebSocket  │  │ │ Jazzy  │ │
│  │   Mission Control UI    │   │  │   REST API   │  │ │ SLAM   │ │
│  └─────────────────────────┘   │  └──────────────┘  │ └────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## 🎯 Architecture Layers

### 1. Frontend Layer (DIAMANTS_FRONTEND)
**Technology Stack:** Three.js WebGL, JavaScript ES6+, Vite Build System

```
DIAMANTS_FRONTEND/Mission_system/
├── 📱 User Interface Layer
│   ├── index.html              # Main application entry
│   ├── main.js                 # Application bootstrap
│   └── ui/                     # UI components & panels
├── 🎮 Control Layer
│   ├── controllers/
│   │   ├── drone-panel-controller.js    # Individual drone control
│   │   ├── drone-state-manager.js       # State synchronization
│   │   └── crazyflie-ros-controller.js  # Hardware integration
├── 🧠 Intelligence Layer
│   ├── behaviors/
│   │   ├── flight-behaviors.js          # Flight patterns
│   │   └── collaborative-scouting.js    # Swarm coordination
│   ├── intelligence/                    # AI decision making
│   └── missions/                        # Mission planning
├── 🌐 Network Layer
│   ├── ros2_bridge/                     # ROS2 communication
│   └── net/                             # Network protocols
├── 🎨 Visualization Layer
│   ├── visual/                          # 3D rendering
│   ├── shaders/                         # GPU shaders
│   └── environment/                     # 3D environment
└── ⚙️ Core Systems
    ├── core/
    │   ├── diamants-engine.js           # Main engine
    │   ├── diamants-formulas.js         # Mathematical models
    │   ├── swarm-memory.js              # Collective memory
    │   └── system/                      # System utilities
    └── physics/                         # Physics simulation
```

**Frontend Architecture Flow:**
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   User Input    │───►│  Control Layer   │───►│ Network Bridge  │
│  (UI Panels)    │    │ (Controllers)    │    │ (WebSocket)     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         ▲                        │                       │
         │                        ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ Visualization   │◄───│ Intelligence     │◄───│    API Layer    │
│ (Three.js)      │    │ (Behaviors)      │    │   (FastAPI)     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### 2. API Layer (DIAMANTS_API)
**Technology Stack:** FastAPI, WebSocket, Python 3.12, uvicorn

```
DIAMANTS_API/
├── 🚀 Launch System
│   ├── launcher.py              # Main API launcher
│   ├── quick-setup.sh          # Rapid deployment
│   └── start-all.sh            # Complete system startup
├── 🔧 Configuration
│   ├── config.env              # Environment variables
│   └── requirements.txt        # Python dependencies
├── 🩺 Health & Monitoring
│   ├── status.sh               # System status check
│   ├── diagnose.sh             # Diagnostic tools
│   └── logs/                   # Application logs
├── 🔄 Process Management
│   ├── start-backend.sh        # Backend service starter
│   ├── start-frontend.sh       # Frontend service starter
│   ├── stop-all.sh             # Graceful shutdown
│   └── restart.sh              # Service restart
└── 🛠️ Development Tools
    ├── fix-ports.sh            # Port conflict resolution
    └── setup-dependencies.sh   # Dependency installation
```

**API Architecture Flow:**
```
┌─────────────────────────────────────────────────────────────────┐
│                        API LAYER                               │
├─────────────────────────────────────────────────────────────────┤
│  HTTP REST API     │   WebSocket Real-time   │   Bridge Layer  │
│  ┌───────────────┐ │  ┌─────────────────────┐ │ ┌─────────────┐ │
│  │ /api/status   │ │  │ Mission Updates     │ │ │ ROS2 Bridge │ │
│  │ /api/drones   │ │  │ Drone Telemetry     │ │ │ State Sync  │ │
│  │ /api/missions │ │  │ Real-time Commands  │ │ │ Data Trans  │ │
│  └───────────────┘ │  └─────────────────────┘ │ └─────────────┘ │
└─────────────────────────────────────────────────────────────────┘
           ▲                        ▲                       ▲
           │                        │                       │
           ▼                        ▼                       ▼
    ┌─────────────┐         ┌─────────────┐        ┌─────────────┐
    │  Frontend   │         │  Frontend   │        │   Backend   │
    │  HTTP Calls │         │  WebSocket  │        │ ROS2 Nodes  │
    └─────────────┘         └─────────────┘        └─────────────┘
```

### 3. Backend Layer (DIAMANTS_BACKEND)
**Technology Stack:** ROS2 Jazzy, Python 3.12, SLAM, Gazebo Simulation

```
DIAMANTS_BACKEND/
├── 🤖 ROS2 Workspace
│   └── slam_collaboratif/
│       └── ros2_ws/
│           ├── src/             # ROS2 packages source
│           ├── build/           # Compiled packages
│           ├── install/         # Installed packages
│           └── log/             # Build logs
├── 🚁 SLAM & Navigation
│   ├── launch_slam_collaborative.sh    # SLAM system launcher
│   ├── config/
│   │   ├── multi_drone_params.yaml    # Multi-drone configuration
│   │   ├── rviz_config_slam_optimized.rviz  # Visualization config
│   │   └── rviz_stigmergie_config.rviz      # Swarm behavior viz
├── 🎛️ Core Configuration
│   └── config/core/
│       ├── behavioral_rules.yaml       # Swarm behavior rules
│       ├── performance_metrics.yaml    # Performance monitoring
│       ├── space_robotics_config.yaml  # Space mission config
│       ├── universe.yaml               # Environment definition
│       └── proto/                      # Protocol definitions
├── 🌐 Web Interface Bridge
│   └── core/web_interface/
│       ├── web_server_robust.py        # Production web server
│       ├── websocket_bridge.py         # WebSocket communication
│       └── diamant_launcher.py         # System launcher
├── 📋 Orchestration Scripts
│   └── scripts/launch/
│       ├── orchestrate_tmux_slam_collaboratif.sh
│       └── orchestrate_tmux_slam_collaboratif_fixed.sh
└── 📊 Logging & Monitoring
    └── logs/
        └── diamants_tmux_slam_collab_journal.log
```

**Backend Architecture Flow:**
```
┌─────────────────────────────────────────────────────────────────┐
│                     BACKEND LAYER (ROS2)                       │
├─────────────────────────────────────────────────────────────────┤
│  ROS2 Nodes        │    SLAM System      │   Hardware Layer   │
│  ┌───────────────┐ │  ┌─────────────────┐ │ ┌─────────────────┐ │
│  │ Drone Control │ │  │ Mapping         │ │ │ Crazyflie       │ │
│  │ Swarm Coord   │ │  │ Localization    │ │ │ Physical Drones │ │
│  │ Mission Exec  │ │  │ Path Planning   │ │ │ Sensors         │ │
│  └───────────────┘ │  └─────────────────┘ │ └─────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
           ▲                        ▲                       ▲
           │                        │                       │
           ▼                        ▼                       ▼
    ┌─────────────┐         ┌─────────────┐        ┌─────────────┐
    │ Web Bridge  │         │   Gazebo    │        │   Radio     │
    │ (WebSocket) │         │ Simulation  │        │ (Crazyflie) │
    └─────────────┘         └─────────────┘        └─────────────┘
```

## 🔄 Inter-Layer Communication

### Data Flow Architecture
```
┌─────────────────────────────────────────────────────────────────┐
│                      DATA FLOW                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Frontend UI ───WebSocket───► API Layer ───Bridge───► Backend   │
│       ▲                           │                       │     │
│       │                           ▼                       ▼     │
│       │                    ┌─────────────┐         ┌───────────┐ │
│       │                    │  FastAPI    │         │   ROS2    │ │
│       │                    │  Server     │         │   Nodes   │ │
│       │                    └─────────────┘         └───────────┘ │
│       │                           ▲                       │     │
│       └────── Real-time Data ─────┘                       │     │
│                                                            ▼     │
│                                                    ┌───────────┐ │
│                                                    │ Hardware  │ │
│                                                    │ (Drones)  │ │
│                                                    └───────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### Communication Protocols

1. **Frontend ↔ API**
   - Protocol: WebSocket + HTTP REST
   - Data: JSON formatted messages
   - Real-time: Mission updates, drone telemetry

2. **API ↔ Backend**
   - Protocol: ROS2 Bridge + Python bindings
   - Data: ROS2 messages, custom protocols
   - Services: SLAM data, navigation commands

3. **Backend ↔ Hardware**
   - Protocol: Crazyflie radio protocol
   - Data: Flight commands, sensor readings
   - Real-time: Position updates, status reports

## 🚀 Deployment Architecture

### Development Environment
```
Local Machine
├── Frontend Development Server (Vite) :5550
├── API Development Server (FastAPI) :8000
└── Backend ROS2 Environment (Native)
```

### Production Environment
```
Production Server
├── Frontend (Static Build + CDN)
├── API (Docker Container + Load Balancer)
└── Backend (ROS2 Native + Hardware Integration)
```

## 🔧 Configuration Management

### Environment Configuration
```
config.env
├── ROS_DOMAIN_ID=0
├── DIAMANTS_HOST=localhost
├── DIAMANTS_PORT=8000
├── FRONTEND_PORT=5550
└── LOG_LEVEL=INFO
```

### System Requirements
- **Frontend**: Node.js 16+, Modern WebGL browser
- **API**: Python 3.12, FastAPI, uvicorn
- **Backend**: ROS2 Jazzy, Ubuntu 22.04+, Hardware drivers

## 📊 Performance & Monitoring

### Key Metrics
- **Latency**: Frontend ↔ API < 50ms
- **Throughput**: 100+ drone updates/second
- **Reliability**: 99.9% uptime target
- **Scalability**: Support for 50+ concurrent drones

### Monitoring Stack
- Health checks via `/api/status`
- ROS2 diagnostics
- Real-time performance metrics
- Automated alerts and recovery

This architecture ensures scalability, maintainability, and real-time performance for autonomous drone swarm operations.
