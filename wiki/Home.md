# 🚁 DIAMANTS - Distributed Autonomous Multi-Agent Systems

Welcome to the official DIAMANTS project wiki! This collaborative drone swarm system implements advanced collective intelligence algorithms for autonomous multi-agent coordination.

## 🎯 Project Overview

**DIAMANTS** (Distributed Autonomous Multi-agents Systems) is a cutting-edge platform that combines:
- **ROS2 Jazzy** backend for robust drone control
- **Three.js WebGL** frontend for immersive 3D visualization  
- **FastAPI** bridge for real-time communication
- **Collective Intelligence** algorithms with 15 DIAMANT harmonics

### 🌟 Key Features

- 🤖 **Multi-drone coordination** with Crazyflie 2.X support
- 🧠 **Collective intelligence** with emergent behaviors
- 🎮 **3D real-time interface** for mission control
- 📡 **WebSocket communication** for low-latency control
- 🌐 **Gazebo simulation** environment integration
- 🔬 **Advanced SLAM** collaborative algorithms

## 📖 Documentation Navigation

### 🚀 Getting Started
- **[Installation Guide](Installation)**  
  Complete setup instructions for all components
- **[Quick Start](Quick-Start)**  
  Launch your first drone swarm in 5 minutes
- **[Architecture Overview](Architecture)**  
  System design and component interaction

### 🛠 Technical Documentation
- **[ROS2 Backend](ROS2-Backend)**  
  Drone control, SLAM, and physics simulation
- **[API Reference](API-Reference)**  
  Complete REST and WebSocket API documentation
- **[Frontend Interface](Frontend-Interface)**  
  Three.js interface and user controls
- **[Collective Intelligence](Collective-Intelligence)**  
  AI algorithms and swarm coordination

### 🎯 Mission Planning
- **[Mission System](Mission-System)**  
  Planning and executing drone missions
- **[Formation Control](Formation-Control)**  
  Geometric formations and dynamic patterns
- **[Waypoint Navigation](Waypoint-Navigation)**  
  Advanced path planning algorithms

### 🔧 Development
- **[Contributing Guide](Contributing)**  
  How to contribute to the project
- **[Development Setup](Development-Setup)**  
  Developer environment configuration
- **[Testing Framework](Testing)**  
  Automated testing and validation

### 📊 Monitoring & Debug
- **[Performance Monitoring](Performance)**  
  System metrics and optimization
- **[Known Issues](Known-Issues)**  
  Common problems and solutions
- **[Troubleshooting](Troubleshooting)**  
  Debug guides and FAQ

### 🎥 Media & Demos
- **[Video Gallery](Video-Gallery)**  
  Demonstrations and use cases
- **[Screenshots](Screenshots)**  
  Interface and system captures
- **[Live Demos](Live-Demos)**  
  Interactive demonstrations

## 🏗 System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Frontend      │    │   FastAPI       │    │   ROS2 Backend  │
│   Three.js      │◄──►│   WebSocket     │◄──►│   Jazzy         │
│   WebGL         │    │   Bridge        │    │   Gazebo        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        │                       │                       │
        ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   UI Controls   │    │   REST API      │    │   SLAM System   │
│   Visualization │    │   Authentication│    │   Physics Sim   │
│   Monitoring    │    │   Data Pipeline │    │   Drone Control │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 🎮 Quick Demo

### Launch Your First Swarm

```bash
# 1. Start the system
./launch_diamants.sh

# 2. Open browser interface
http://localhost:3000

# 3. Launch basic formation
roslaunch diamants_formation triangle_formation.launch
```

### Example Formation Flight

```python
# Create 3-drone triangle formation
from diamants import DroneSwarm, Formation

swarm = DroneSwarm(['cf2x_01', 'cf2x_02', 'cf2x_03'])
formation = Formation.triangle(side_length=2.0)

# Execute formation flight
swarm.takeoff(altitude=1.5)
swarm.execute_formation(formation)
swarm.land()
```

## 🌟 Collective Intelligence

DIAMANTS implements advanced collective intelligence using **15 harmonics**:

### Core Harmonics (H1-H5)
- **H1**: External field influence
- **H2**: Internal agent interactions  
- **H3**: Total system energy
- **H4**: Interaction dynamics
- **H5**: System curvature

### Dynamic Harmonics (H6-H10)
- **H6**: Environmental noise adaptation
- **H7**: Temporal evolution patterns
- **H8**: Stability measurements
- **H9**: Anisotropy detection
- **H10**: Flow curvature analysis

### Advanced Harmonics (H11-H15)
- **H11**: Symmetry preservation
- **H12**: Vorticity patterns
- **H13**: Coherence maintenance
- **H14**: Information flow
- **H15**: Entropy management

## 🚀 Use Cases

### 🔍 Search & Rescue
- Collaborative area coverage
- Real-time information sharing
- Adaptive resource allocation

### 🏗 Construction
- Coordinated structure assembly
- Material transport chains
- Quality control systems

### 🛡 Surveillance
- Perimeter monitoring
- Threat detection networks
- Adaptive patrol patterns

### 🔬 Research
- Swarm behavior studies
- Algorithm validation
- Performance benchmarking

## 📊 System Requirements

### Minimum Configuration
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Jazzy Jalopy
- **Python**: 3.10+
- **Node.js**: 18.x+
- **RAM**: 8GB
- **Storage**: 20GB

### Recommended Configuration
- **OS**: Ubuntu 22.04 LTS
- **CPU**: 8 cores / 16 threads
- **RAM**: 16GB+
- **GPU**: NVIDIA GTX 1060+ (CUDA support)
- **Storage**: 50GB SSD

## 🤝 Community

### Connect With Us
- **GitHub**: [diamants-collab](https://github.com/lololem/diamants-collab)
- **Discord**: [Join our server](https://discord.gg/diamants)
- **Email**: contact@diamants.ai
- **Twitter**: [@DiamantsDrones](https://twitter.com/diamantsdrones)

### Contributing
We welcome contributions! Please read our [Contributing Guide](Contributing) to get started.

### Support
- **Documentation**: This wiki
- **Issues**: [GitHub Issues](https://github.com/lololem/diamants-collab/issues)
- **Discussions**: [GitHub Discussions](https://github.com/lololem/diamants-collab/discussions)

## 📈 Project Status

- **Version**: 1.0.0
- **Status**: Active Development
- **License**: MIT
- **Last Updated**: September 2025

### Recent Updates
- ✅ ROS2 Jazzy integration complete
- ✅ WebGL frontend optimization
- ✅ Collective intelligence algorithms
- 🔄 Multi-drone formation testing
- 🔄 Performance optimization
- ⏳ Real hardware integration

---

**Ready to explore the future of autonomous drone swarms?** Start with our [Installation Guide](Installation) and join the revolution in distributed robotics! 🚀