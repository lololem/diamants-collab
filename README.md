# 💎 DIAMANTS: Distributed Autonomous Multi-agents Systems

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/) [![Python](https://img.shields.io/badge/Python-3.12-blue.svg)](https://www.python.org/) [![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE) [![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-green.svg)](https://fastapi.tiangolo.com/)

**DIAMANTS is an open-source platform for the simulation and execution of distributed intelligence. Our philosophy: code once, and deploy everywhere—in simulation and in the real world.**

This project is a 'playground' for developers, creating a robust bridge between high-level intent (the swarm's strategy) and low-level execution (each drone's physical commands). It's a space to code distributed intelligence, test it in a credible simulation, and push it directly to real-world swarms.

## 🎯 Vision

DIAMANTS's objective is to tackle a major technical challenge: achieving emergent collective behaviors in open, modular, and interoperable code. We aim to demonstrate that distributed intelligence is not a theoretical concept, but a robust, documented, and reusable software artifact for the community.

## 🎬 System Demonstrations

See DIAMANTS in action! Watch our multi-drone collaborative system working seamlessly from simulation to real-world deployment.

### 🤖 Backend - ROS2 Multi-Drone SLAM System
Real-time collaborative SLAM with 8 Crazyflie drones in Gazebo simulation, featuring TMUX orchestration and map fusion.

[![Watch Backend Demo](https://img.youtube.com/vi/iYg9Jf1jv4Y/maxresdefault.jpg)](https://www.youtube.com/watch?v=iYg9Jf1jv4Y)

**🤖 Backend Features Showcase:**
- ✅ 8 Crazyflie drones in Gazebo simulation
- ✅ Real-time collaborative SLAM mapping
- ✅ TMUX orchestration (13 specialized windows)  
- ✅ ROS2 Jazzy + Gazebo Garden integration
- ✅ Multi-agent path planning and coordination

### 🌐 Frontend - 3D Visualization Interface
Interactive Three.js + WebGL interface with real-time drone tracking, mission planning, and swarm visualization.

#### Frontend Interface Overview
[![Watch Frontend Demo](https://img.youtube.com/vi/fyEmYu4lbzo/maxresdefault.jpg)](https://www.youtube.com/watch?v=fyEmYu4lbzo)

**🌐 Frontend Features Showcase:**
- ✅ Three.js + WebGL real-time 3D visualization
- ✅ WebSocket communication with ROS2 backend
- ✅ Interactive mission planning interface
- ✅ Vite development server integration
- ✅ Modern responsive UI design

#### Advanced Visualization Features  
[![Frontend Demo 2](https://img.shields.io/badge/▶️%20Watch%20Advanced%20Features-blue?style=for-the-badge)](./DEMO/video/Fontend2.mp4)

**🎮 Advanced Features:**
- ✅ Advanced shader effects and lighting
- ✅ Real-time performance monitoring
- ✅ Dynamic camera controls
- ✅ WebSocket integration demonstration
- ✅ Multi-viewport rendering

### 🚁 Additional System Features
Extended capabilities showcasing various aspects of the DIAMANTS platform.

#### Multi-Agent Systems (SMA) Demo
[![Watch SMA Demo](https://img.youtube.com/vi/1Av_o-9fzrE/maxresdefault.jpg)](https://www.youtube.com/watch?v=1Av_o-9fzrE)

**🤖 SMA Features:**
- ✅ Distributed agent coordination
- ✅ Emergent swarm behaviors
- ✅ Real-time multi-agent communication
- ✅ Collective decision making

#### Gradient-Based Navigation Demo
[![Watch Gradient Demo](https://img.youtube.com/vi/ElABxOde6ak/maxresdefault.jpg)](https://www.youtube.com/watch?v=ElABxOde6ak)

**🎯 Gradient Navigation Features:**
- ✅ Gradient-based path planning
- ✅ Obstacle avoidance algorithms
- ✅ Dynamic field navigation
- ✅ Smooth trajectory optimization

#### Swarm Coordination Demo
[![Watch Swarm Demo](https://img.youtube.com/vi/L8V64LajM2w/maxresdefault.jpg)](https://www.youtube.com/watch?v=L8V64LajM2w)

**🐝 Swarm Features:**
- ✅ Collective swarm intelligence
- ✅ Distributed formation control
- ✅ Emergent group behaviors
- ✅ Scalable multi-drone coordination

#### Stigmergy-Based Coordination Demo
[![Watch Stigmergy Demo](https://img.youtube.com/vi/SyqeRwcbDO4/maxresdefault.jpg)](https://www.youtube.com/watch?v=SyqeRwcbDO4)

**🧭 Stigmergy Features:**
- ✅ Indirect coordination through environment
- ✅ Pheromone-like trail mechanisms
- ✅ Self-organizing pathfinding
- ✅ Bio-inspired collective intelligence

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

## 🚀 Quick Start

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
npm run dev       # Start development server (http://localhost:3000)
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
- Expected vs actual behavior

## 📋 System Architecture

### 🏗️ Architecture Overview

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  FRONTEND       │    │   DIAMANTS_API  │    │   BACKEND       │
│  (Three.js)     │◄──►│   (FastAPI)     │◄──►│   (ROS2)        │
│  Mission UI     │    │   WebSocket     │    │   Gazebo        │
│  3D Visualization│    │   Bridge        │    │   Multi-Drones  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 🧩 Core Components

- **DIAMANTS_BACKEND**: ROS2 Jazzy ecosystem with Gazebo Garden/Harmonic for physics simulation
- **DIAMANTS_API**: FastAPI service providing WebSocket bridge between frontend and ROS2
- **DIAMANTS_FRONTEND**: Modern Vite + Three.js application for real-time 3D visualization

## 🛠️ Technical Stack

### Backend Technologies
- **ROS2 Jazzy**: Robot Operating System for distributed computing
- **Gazebo Garden/Harmonic**: High-fidelity physics simulation
- **Crazyswarm2**: Crazyflie drone swarm framework
- **SLAM Toolbox**: Simultaneous Localization and Mapping
- **Python 3.12**: Core backend logic

### Frontend Technologies
- **Vite**: Modern build tool and dev server
- **Three.js**: 3D graphics library for WebGL
- **WebSockets**: Real-time communication
- **ES2020**: Modern JavaScript features

### Integration
- **TMUX**: Multi-window session orchestration
- **FastAPI**: High-performance async API framework
- **WebSocket Bridge**: Real-time ROS2 ↔ Web communication

## 📚 Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture and design principles
- [DIAMANTS_TECH_STACK.md](DIAMANTS_TECH_STACK.md) - Complete technical documentation
- [CONTRIBUTING.md](CONTRIBUTING.md) - Contribution guidelines
- [API Documentation](DIAMANTS_API/docs/) - API endpoints and WebSocket protocols

## 🔬 Research & Publications

DIAMANTS is built on solid scientific foundations in swarm intelligence and distributed systems:

- **Emergent Behaviors**: Study of collective intelligence from simple agent interactions
- **Stigmergy Algorithms**: Bio-inspired coordination through environmental modifications
- **Consensus Mechanisms**: Distributed agreement protocols for multi-agent systems
- **Sim-to-Real Transfer**: Seamless transition from simulation to physical deployment

## 🎯 Use Cases

### 🚁 Drone Swarm Applications
- Search and rescue operations
- Environmental monitoring
- Agricultural surveying
- Infrastructure inspection

### 🤖 Multi-Robot Coordination
- Warehouse automation
- Collaborative manufacturing
- Distributed sensing networks
- Autonomous vehicle coordination

### 🧠 Research Platform
- Swarm intelligence algorithms
- Distributed AI systems
- Emergent behavior studies
- Human-swarm interaction

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ROS2 Community**: For the robust robotics middleware
- **Gazebo Team**: For the high-fidelity simulation environment
- **Bitcraze**: For the excellent Crazyflie platform and Crazyswarm2
- **Three.js Community**: For the powerful 3D web graphics library
- **Open Source Contributors**: For making this collaborative platform possible

## 📞 Contact & Support

- **Issues**: [GitHub Issues](https://github.com/lololem/diamants-collab/issues)
- **Discussions**: [GitHub Discussions](https://github.com/lololem/diamants-collab/discussions)
- **Email**: [Contact maintainers](mailto:contact@diamants-project.org)

---

**🌟 Star this repository if you find it useful! Contributions are welcome and encouraged.**

> *"The best way to predict the future is to invent it, and the best way to invent the future of robotics is to build it collaboratively."*
