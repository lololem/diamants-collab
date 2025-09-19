# DIAMANTS Frontend Swarm Intelligence Demo

## 🧠 Overview

This directory contains an advanced demonstration of **Swarm Intelligence** using the DIAMANTS framework for autonomous coordination of multiple drones in a complex forest environment.

## 🎯 Demonstration Objectives

### Emergent Collective Intelligence
- **Decentralized coordination**: Each drone develops its own expertise while collaborating
- **Distributed consensus**: Collective decision-making without centralized control
- **Adaptive specialization**: Emergence of specialized roles according to mission needs
- **Stigmergic communication**: Information exchange through environmental traces

### Optimized Collaborative Scouting
- **Boustrophedon exploration**: Systematic coverage with optimal spacing (18m)
- **Intelligent anti-redundancy**: Automatic avoidance of already explored zones
- **Dynamic sector assignment**: Adaptive distribution according to number of drones
- **Collaborative completion**: Coordination to complete partially explored areas

## 🚁 Drone Types and Specializations

| Type | Role | Capabilities | Color |
|------|------|-------------|-------|
| **Scout** | Exploration | High speed, optimized detection range | 🟢 Green |
| **Coordinator** | Leadership | Long-range communication, synchronization | 🔴 Red |
| **Stealth** | Reconnaissance | Discretion, navigation precision | 🔵 Blue |
| **Heavy** | Transport | Load capacity, stability | 🟠 Orange |

## 📊 Collective Intelligence Metrics

### Classic DIAMANTS Metrics
- **I(t)**: Instantaneous intelligence based on gradient ∇(φ+σ)
- **φ (Phi)**: Attractive potential for swarm cohesion
- **σ (Sigma)**: Repulsive potential for redundancy avoidance
- **|∇|**: Magnitude of directional gradient

### Swarm Intelligence Metrics
- **Emergence**: Level of detected self-organized behaviors
- **Cohesion**: Collective coordination index of the swarm
- **Phase**: Global behavioral state (DISPERSION → EXPLORATION → CONSOLIDATION → COMPLETION)
- **Experts**: Number of drones that have developed specialized expertise
- **Communication**: Intensity of information exchanges between drones

## 🌲 Simulation Environment

### Realistic Dense Forest
- **30 trees** positioned with natural spacing (minimum distance 15m)
- **Daylight illumination** with sunbeams filtering through the canopy
- **Forest terrain** with relief variations and realistic textures
- **15 targets of interest** hidden for scouting missions

### Optimized Altitudes
- **Ground level**: 2m (basic navigation)
- **Trunk level**: 8m (optimal scouting)
- **Coordination level**: 15m (overview)

## 🎮 Controls and Interface

### Commands
- **WASD**: Camera movement
- **Mouse**: View rotation
- **Scroll wheel**: Zoom in/out

### Control Buttons
- **🚀 START**: Launch swarm intelligence mission
- **⏸️ PAUSE**: Temporary simulation suspension
- **🔄 Reset**: Complete environment reset

## ⚙️ Advanced Configuration

### Swarm Parameters (CONFIG)
```javascript
maxDrones: Infinity          // Unlimited number of drones
zoneSize: 120               // Mission area size (120m)
explorationGrid: 20         // Exploration grid precision
swarmCohesion: 1.5         // Collective cohesion force
swarmSeparation: 5.0       // Collision avoidance force
autonomyPower: 4.5         // Individual autonomy level
```

### Collective Intelligence Algorithms
- **Digital stigmergy**: Persistent environmental traces
- **Distributed consensus**: Expertise-weighted voting
- **Emulation learning**: Knowledge transfer between drones
- **Behavioral adaptation**: Strategy change according to context

## 🔬 Technical Features

### Advanced Intelligence Systems
1. **Global Collective Memory** (`SWARM_MEMORY`)
   - Shared stigmergy map
   - Collective discoveries history
   - Expertise zones per drone
   - Real-time dynamic consensus

2. **Multi-Modal Communication**
   - Direct messages between drones (30m range)
   - Priority information broadcasting
   - Persistent stigmergic traces
   - Strategy synchronization

3. **Emergent Specialization**
   - Automatic expertise development
   - Teaching between experienced drones
   - Reconversion according to swarm needs
   - Dynamic emergent leadership

## 🎯 Mission Phases

### Phase 1: Dispersion (0-20% progress)
- Rapid distribution in the zone
- Initial sector assignment
- Communication establishment

### Phase 2: Parallel Exploration (20-60% progress)
- Coordinated scouting by corridors
- Active redundancy avoidance
- Real-time discovery sharing

### Phase 3: Consolidation (60-85% progress)
- Collaborative cleanup of partial zones
- Trajectory optimization
- Finalized role specialization

### Phase 4: Completion (85-100% progress)
- Precise finishing of last zones
- Cross-verification of discoveries
- Final consensus on mission state

## 🚀 Technologies Used

### Rendering Engine
- **Three.js r128**: High-performance WebGL 3D rendering
- **ColladaLoader**: Support for .dae 3D models of Crazyflies
- **Custom shaders**: Collective intelligence visual effects

### Artificial Intelligence
- **DIAMANTS algorithms**: I(t) = ∬|∇(φ+σ)|dΩ
- **Advanced boids**: Adaptive cohesion, separation, alignment
- **Emergent neural networks**: Distributed learning
- **Swarm optimization**: Collective convergence toward optima

## 📈 Performance Metrics

### Optimized Coverage Time
- **Target**: < 3 minutes for complete coverage
- **Efficiency**: > 95% with minimal redundancy
- **Adaptation**: Reconfiguration in < 10 seconds during changes

### Measurable Collective Intelligence
- **Emergence**: Automatic detection of 4+ emergent behaviors
- **Expertise**: 80% of drones develop specialization
- **Consensus**: Collective decisions in < 5 seconds
- **Communication**: > 90% reliability of information exchanges

## 🔧 Installation and Launch

### Prerequisites
- Modern browser with WebGL 2.0 support
- Internet connection for loading Three.js libraries
- Local HTTP server (VS Code Live Server recommended)

### Startup
1. Open `DIAMANTS_Complete_Frontend_Swarm_Intelligence_Demo.html`
2. Launch with HTTP server (avoid file://)
3. Click "🚀 START" to initiate the mission
4. Observe the emergence of collective intelligence

## 📊 Results Analysis

### Success Indicators
- **Territorial coverage**: Percentage of explored area
- **Minimized redundancy**: Avoidance of double-scouting
- **Achieved consensus**: Successful collective decisions
- **Emergent specializations**: Self-organized roles
- **Temporal efficiency**: Accomplishment speed

### Observable Emergent Behaviors
- Spontaneous work group formation
- Rotational leadership according to expertise
- Optimized communication chains
- Contextual adaptive strategies

## 🔍 Debug and Monitoring

### Debug Console
Enabled by default, displays:
- Collective phase changes
- Important target discoveries
- Achieved/failed consensus
- Behavioral adaptations
- Real-time intelligence metrics

### Real-Time Visualizations
- **Exploration grid**: Highlighted covered zones
- **Stigmergy traces**: Information paths
- **Expertise auras**: Expert influence zones
- **Communication links**: Active connections between drones

## 🎓 Practical Applications

### Usage Scenarios
- **Search and rescue** in difficult terrain
- **Environmental surveillance** of large areas
- **Collaborative mapping** at large scale
- **Exploration missions** in space or underwater
- **Perimeter security** adaptive and autonomous

### Swarm Intelligence Advantages
- **Robustness**: Resistance to individual failures
- **Scalability**: Improved performance with more drones
- **Efficiency**: Automatic resource optimization
- **Adaptability**: Rapid reaction to environmental changes

---

## 🤝 Contribution and Development

This demonstration is part of the DIAMANTS project and illustrates advanced capabilities of decentralized collective intelligence. It serves as a reference for developing autonomous swarm applications in real contexts.

**Version**: 1.0.0  
**Last update**: 19 September 2025  
**Status**: ✅ Complete functional demonstration
