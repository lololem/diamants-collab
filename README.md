# 💎 DIAMANTS: Distributed Autonomous Multi-Agent Systems

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/) [![Python](https://img.shields.io/badge/Python-3.12-blue.svg)](https://www.python.org/) [![License](https://img.shields.io/badge/License-PolyForm Noncommercial-green.svg)](LICENSE) [![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-green.svg)](https://fastapi.tiangolo.com/)

**DIAMANTS is an open-source platform for the simulation and execution of distributed intelligence. Our philosophy: code once, and deploy everywhere—in simulation and in the real world.**

This project is a 'playground' for developers, creating a robust bridge between high-level intent (the swarm's strategy) and low-level execution (each drone's physical commands). It's a space to code distributed intelligence, test it in a credible simulation, and push it directly to real-world swarms.

## 🎯 Vision

DIAMANTS's objective is to tackle a major technical challenge: achieving emergent collective behaviors in open, modular, and interoperable code. We aim to demonstrate that distributed intelligence is not a theoretical concept, but a robust, documented, and reusable software artifact for the community.

## ✨ Key Features & Technical Challenges

### 🌱 Open Source by Design

Our entire ecosystem is built on recognized open standards. No proprietary lock-in.

**Main Stack:**
- **Backend & Simulation**: ROS 2, Gazebo, PX4/ArduPilot, MAVROS
- **Frontend & Visualization**: Vite, Three.js, Babylon.js, WebGL, Node.js

**Goal**: To foster an ecosystem where every developer can contribute, test, and enrich the platform.

### 🌐 Radical Interoperability

A defined mission must be understood by any agent, regardless of its hardware or middleware.
- **Universal Missions**: Mission definitions in YAML/JSON
- **Unified APIs**: Design of agnostic interfaces and mission translators

### 🧠 From Simple Rules to Collective Intelligence

The scientific core of the project is the study of emergence from simple interactions.
- **Explored Algorithms**: Stigmergy, consensus algorithms, social forces (Lennard-Jones type)
- **Challenge**: To code simple behavioral bricks and observe the emergence of a credible collective intelligence

### 🔄 Seamless Sim-to-Real Workflow

Simulation is a mirror of reality. Zero throwaway code.
- **High-Fidelity Simulation**: The high-level code produces command streams via ROS 2 that are translated identically for the simulator and for physical agents
- **Continuous Validation**: Every line of code is validated in simulation and then executed without any modification on the real hardware

## 🚀 Quick Start � DIAMANTS: Distributed Autonomous Multi## 🚀 Quick Start

### Prerequisites
- **Ubuntu 24.04** (recommended)
- **ROS2 Jazzy** ([installation guide](https://docs.ros.org/en/jazzy/Installation.html))
- **Python 3.12+**
- **Node.js 16+**

### Installation

```bash
# Clone the collaborative repository
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab

# One-command launch (interactive menu)
./launch_diamants.sh
```

## 🔧 Component-Specific Launch

You can run each system autonomously using dedicated scripts:

### 📡 DIAMANTS_API - FastAPI Service & WebSocket Bridge
```bash
cd DIAMANTS_API
./setup.sh        # First-time setup
./start.sh        # Launch API service
./status.sh       # Check service status
./stop.sh         # Stop service
```

### 🤖 DIAMANTS_BACKEND - ROS2 SLAM System
```bash
cd DIAMANTS_BACKEND
./setup.sh                          # Install ROS2 dependencies
./launch_slam_collaborative.sh      # Start collaborative SLAM
# Monitor logs: tail -f logs/diamants_tmux_slam_collab_journal.log
```

### 🌐 DIAMANTS_FRONTEND - 3D Visualization Interface
```bash
cd DIAMANTS_FRONTEND/Mission_system
npm install       # Install dependencies
npm run dev       # Start development server (http://localhost:5173)
npm run build     # Production build
```

## 🤝 Collaborative Development

**DIAMANTS is actively seeking contributors!** 

We need help with:
- 🐛 **Bug Fixes**: System stability and edge cases
- 🚀 **Performance Optimization**: Multi-agent coordination efficiency
- 📚 **Documentation**: User guides and API documentation
- 🧪 **Testing**: Unit tests and integration testing
- 🎨 **UI/UX**: Frontend interface improvements
- 🤖 **Algorithm Development**: Advanced swarm intelligence behaviors

### 🛠️ Development Workflow
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes and test thoroughly
4. Submit a pull request with detailed description

### 🆘 Report Issues
Found a bug or have a feature request? Please [create an issue](https://github.com/lololem/diamants-collab/issues) with:
- Detailed description of the problem
- Steps to reproduce
- System environment (Ubuntu version, ROS2 distribution, etc.)
- Expected vs actual behavior[ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/) [![Python](https://img.shields.io/badge/Python-3.12-blue.svg)](https://www.python.org/) [![License](https://img.shields.io/badge/License-PolyForm Noncommercial-green.svg)](LICENSE) [![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-green.svg)](https://fastapi.tiangolo.com/)

**DIAMANTS is an open-source platform for the simulation and execution of distributed intelligence. Our philosophy: code once, and deploy everywhere—in simulation and in the real world.**

This project is a 'playground' for developers, creating a robust bridge between high-level intent (the swarm's strategy) and low-level execution (each drone's physical commands). It's a space to code distributed intelligence, test it in a credible simulation, and push it directly to real-world swarms.

## 🎯 Vision

DIAMANTS's objective is to tackle a major technical challenge: achieving emergent collective behaviors in open, modular, and interoperable code. We aim to demonstrate that distributed intelligence is not a theoretical concept, but a robust, documented, and reusable software artifact for the community.

## ✨ Key Features & Technical Challenges

### 🌱 Open Source by Design

Our entire ecosystem is built on recognized open standards. No proprietary lock-in.

**Main Stack:**
- **Backend & Simulation**: ROS 2, Gazebo, PX4/ArduPilot, MAVROS
- **Frontend & Visualization**: Vite, Three.js, Babylon.js, WebGL, Node.js

**Goal**: To foster an ecosystem where every developer can contribute, test, and enrich the platform.

### 🌐 Radical Interoperability

A defined mission must be understood by any agent, regardless of its hardware or middleware.
- **Universal Missions**: Mission definitions in YAML/JSON
- **Unified APIs**: Design of agnostic interfaces and mission translators

### 🧠 From Simple Rules to Collective Intelligence

The scientific core of the project is the study of emergence from simple interactions.
- **Explored Algorithms**: Stigmergy, consensus algorithms, social forces (Lennard-Jones type)
- **Challenge**: To code simple behavioral bricks and observe the emergence of a credible collective intelligence

### 🔄 Seamless Sim-to-Real Workflow

Simulation is a mirror of reality. Zero throwaway code.
- **High-Fidelity Simulation**: The high-level code produces command streams via ROS 2 that are translated identically for the simulator and for physical agents
- **Continuous Validation**: Every line of code is validated in simulation and then executed without any modification on the real hardware

## � Quick Start

### Prerequisites
- **Ubuntu 24.04** (recommended)
- **ROS2 Jazzy** ([installation guide](https://docs.ros.org/en/jazzy/Installation.html))
- **Python 3.12+**
- **Node.js 16+**

### Installation

```bash
# Clone the collaborative repository
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab

# One-command launch (interactive menu)
./launch_diamants.sh
```

### Component Launch

```bash
# Complete system (recommended)
./launch_diamants.sh  # Select option 6: "Complete System"

# Individual components
./launch_diamants.sh  # Option 1: Backend + SLAM
./launch_diamants.sh  # Option 2: API Service  
./launch_diamants.sh  # Option 3: Frontend Interface
```

## 🏗️ Architecture Overview

### System Components

```
DIAMANTS System Architecture
├── 🔧 DIAMANTS_API/          # FastAPI + WebSocket Service
│   ├── api/                   # REST API endpoints
│   ├── services/             # WebSocket & ROS2 integration
│   └── launcher.py           # Unified entry point
├── 🤖 DIAMANTS_BACKEND/      # ROS2 SLAM System
│   ├── slam_collaboratif/    # Multi-agent SLAM workspace
│   ├── config/               # ROS2 configurations
│   └── launch_slam_collaborative.sh
├── 🎮 DIAMANTS_FRONTEND/     # 3D Visualization Interface
│   └── Mission_system/       # WebGL application
└── 📋 Root Scripts/          # System management
    ├── launch_diamants.sh    # Interactive launcher
    ├── stop_diamants.sh      # Clean shutdown
    └── check_ros_processes.sh # System diagnostics
```

## 🌟 Key Features

### 🧠 **Collaborative Intelligence**
- **Stigmergy-based coordination**: Bio-inspired pheromone communication
- **Real-time map fusion**: Dynamic merging of individual SLAM maps
- **Distributed decision making**: Autonomous task allocation and path planning

### � **Advanced Simulation**
- **Authentic physics**: Accurate Crazyflie 2.0 flight dynamics
- **Provençal environment**: Mediterranean forest with procedural generation
- **Real-time rendering**: 60fps WebGL visualization with advanced shaders

### 🔧 **Production Ready**
- **Automated orchestration**: TMUX-based multi-component management
- **Comprehensive testing**: pytest suite with ROS2 integration
- **Type safety**: Full Pylance support with conditional imports
- **Monitoring tools**: Real-time diagnostics and performance metrics

## 📡 Service Endpoints

| Component | URL | Description |
|-----------|-----|-------------|
| **API Documentation** | `http://localhost:8000/docs` | Interactive Swagger UI |
| **WebSocket Service** | `ws://localhost:8765` | Real-time drone communication |
| **3D Interface** | `http://localhost:5550` | Mission control interface |
| **RViz Visualization** | ROS2 launch | SLAM map visualization |

## �️ Development

### Environment Setup
```bash
# Configure Python environment
cd DIAMANTS_API
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Build ROS2 workspace
cd DIAMANTS_BACKEND/slam_collaboratif/ros2_ws
colcon build

# Install frontend dependencies
cd DIAMANTS_FRONTEND/Mission_system
npm install
```

### Testing
```bash
# API tests
cd DIAMANTS_API
python -m pytest tests/ -v

# System integration test
./check_ros_processes.sh

# Frontend development
cd DIAMANTS_FRONTEND/Mission_system
npm run dev
```

## 📚 Documentation

- **[API Reference](DIAMANTS_API/README.md)** - FastAPI service documentation
- **[Backend Guide](DIAMANTS_BACKEND/README.md)** - ROS2 SLAM system setup
- **[Frontend Manual](DIAMANTS_FRONTEND/Mission_system/README.md)** - 3D interface development
- **[Contributing Guidelines](CONTRIBUTING.md)** - Development workflow

## 🚦 System Status

### ✅ Architecture v2.0 - Production Ready

**Major Improvements:**
- ✅ **Unified API**: Single entry point with FastAPI + WebSocket Service
- ✅ **Centralized configuration**: Dataclass replaces Pydantic for reliability
- ✅ **Complete testing**: pytest suite with ROS2 support and mocks
- ✅ **Type checking**: Full Pylance support with conditional imports
- ✅ **Clean architecture**: Eliminated duplications, 6 essential root files
- ✅ **Optimized performance**: Single WebSocket Service instead of multiple bridges

| Before v2.0 | After v2.0 | Benefit |
|-------------|-------------|---------|
| ❌ Multiple WebSocket bridges | ✅ Unified Service | Performance + Simplicity |
| ❌ Pydantic BaseSettings | ✅ Dataclass config | Reliability + Type safety |
| ❌ Untyped imports | ✅ Complete type checking | Robust development |  
| ❌ Scattered scripts | ✅ 6 essential files | Simplified maintenance |
| ❌ Fragmented tests | ✅ Unified pytest suite | Quality assurance |

## 🆘 Troubleshooting

### Common Issues

#### 🔌 Service Status Check
```bash
## 🔍 System Diagnostics

```bash
# Complete system diagnostics
./check_ros_processes.sh

# API status check
cd DIAMANTS_API/
./status.sh                         # Quick unified status

# Detailed diagnostics  
./scripts/maintenance/diagnose.sh   # Complete analysis with logs

# Clean restart
./stop.sh && sleep 2 && ./start.sh  # Intelligent restart
```

## 🚀 Get Started Now

```bash
# Express installation - just 3 commands!
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab
./launch_diamants.sh
```

**� Your DIAMANTS system will be operational in less than 5 minutes!**

> 💡 **Tip**: Use `./DIAMANTS_API/status.sh` to monitor system status at any time.

---

<div align="center">

**� DIAMANTS - Pushing the Boundaries of Multi-Agent Autonomy �**

[⭐ Star this Repo](https://github.com/lololem/diamants-collab) • [🐛 Report Bug](https://github.com/lololem/diamants-collab/issues) • [💡 Request Feature](https://github.com/lololem/diamants-collab/issues)

</div>

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

**� Ready to explore autonomous drone swarms?**

Run `git clone https://github.com/lololem/diamants-collab.git && cd diamants-collab && ./launch_diamants.sh` and start your journey into the future of robotics! ✨