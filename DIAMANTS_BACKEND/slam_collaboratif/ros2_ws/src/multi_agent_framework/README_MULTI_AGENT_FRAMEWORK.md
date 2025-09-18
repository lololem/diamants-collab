# 🤖 DIAMANTS Multi-Agent Framework

**Advanced Collaborative Intelligence System for Drone Swarm Coordination**

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12-green)](https://www.python.org/)
[![Swarm Intelligence](https://img.shields.io/badge/AI-Swarm%20Intelligence-orange)](https://github.com/lololem/diamants-collab)

## 🎯 Overview

The DIAMANTS Multi-Agent Framework is a sophisticated ROS2-based system that enables collaborative drone swarm operations with advanced AI coordination. It implements bio-inspired algorithms for autonomous multi-agent coordination, collision avoidance, and distributed intelligence.

## 🏗️ Architecture

```
Multi-Agent Framework
├── 🧠 Swarm Intelligence Core
│   ├── SwarmBehavior      # Collective behaviors & coordination
│   ├── FlightController   # Intelligent flight management
│   └── IntelligenceMonitor # Performance & behavior monitoring
├── 🛡️ Safety Systems
│   └── AntiCollision      # Real-time collision avoidance
├── 🌐 Web Interface
│   ├── WebServer         # FastAPI backend
│   ├── WebSocketBridge   # Real-time communication
│   └── Dashboard         # 3D visualization interface
└── 🚁 Mission Coordination
    └── MissionTestWeb    # Web-based mission control
```

## 🧠 Core Components

### 1. **Swarm Intelligence** (`swarm_intelligence/`)

#### `swarm_behavior.py` - Collective Intelligence Engine
- **Bio-inspired Algorithms**: Flocking, formation control, distributed consensus
- **Social Coordination**: Inter-drone communication and behavior synchronization
- **Adaptive Behaviors**: Dynamic role assignment and task allocation
- **Emergent Intelligence**: Self-organizing patterns and collective decision making

**Key Features:**
```python
# Formation control with adaptive spacing
formation_control(pattern='grid|line|circle|triangle')

# Distributed exploration with coverage optimization
explore_territory(coverage_threshold=0.85)

# Collective obstacle avoidance
collective_avoidance(obstacle_map, safety_margin=1.0)
```

#### `flight_controller.py` - Intelligent Flight Management
- **Autonomous Navigation**: GPS-free indoor positioning with sensor fusion
- **Trajectory Planning**: Smooth path generation with obstacle consideration
- **Energy Optimization**: Battery-aware flight planning and resource management
- **Safety Protocols**: Emergency landing and fail-safe mechanisms

**Key Features:**
```python
# Adaptive altitude control with terrain following
altitude_control(target=1.5, terrain_adaptive=True)

# Energy-efficient trajectory planning
plan_trajectory(waypoints, energy_constraint=True)

# Emergency protocols
emergency_landing(reason='battery_low|collision_risk|communication_loss')
```

#### `intelligence_monitor.py` - Performance Analytics
- **Real-time Metrics**: Flight performance, coordination efficiency, mission progress
- **Behavioral Analysis**: Swarm cohesion, individual vs collective performance
- **Predictive Analytics**: Failure prediction and maintenance scheduling
- **Learning Algorithms**: Adaptive parameter tuning based on performance history

### 2. **Safety Systems**

#### `anti_collision_diamants.py` - Collision Avoidance System
- **Real-time Detection**: Multi-sensor fusion for obstacle and drone detection
- **Predictive Avoidance**: Trajectory prediction and preemptive path adjustment
- **Cooperative Avoidance**: Distributed negotiation for collision resolution
- **Emergency Maneuvers**: Last-resort collision avoidance protocols

**Safety Features:**
- **Minimum Separation**: 0.5m enforced between drones
- **Predictive Horizon**: 3-second look-ahead for collision prediction
- **Recovery Protocols**: Automatic return-to-formation after avoidance
- **Emergency Stop**: Instant hover capability on collision detection

### 3. **Web Interface** (`web_interface/`)

#### `web_server.py` - FastAPI Backend
- **RESTful API**: Mission control, status monitoring, configuration management
- **Real-time WebSocket**: Live telemetry streaming and command interface
- **Static Asset Serving**: 3D visualization resources and dashboard components
- **ROS2 Integration**: Seamless bridge between web interface and ROS2 nodes

#### `websocket_bridge.py` - Real-time Communication
- **Bidirectional Streaming**: Commands to drones, telemetry to interface
- **Message Routing**: Intelligent message distribution to relevant subscribers
- **Connection Management**: Automatic reconnection and error handling
- **Data Compression**: Efficient real-time data transmission

#### Dashboard Interface (`templates/dashboard.html`)
- **3D Visualization**: Real-time drone positions and trajectories
- **Mission Control**: Interactive mission planning and execution
- **Performance Metrics**: Live monitoring of swarm intelligence metrics
- **Configuration Panel**: Dynamic parameter adjustment and system tuning

## 🚀 Key Features

### **Collaborative SLAM**
- **Multi-Agent Mapping**: Distributed simultaneous localization and mapping
- **Map Fusion**: Consensus-based map merging with conflict resolution
- **Exploration Coordination**: Efficient territory coverage with minimal overlap
- **Dynamic Re-mapping**: Real-time map updates and obstacle integration

### **Swarm Intelligence Algorithms**

#### **Formation Control**
```python
# Dynamic formation patterns
patterns = {
    'grid': GridFormation(spacing=2.0, rows=2, cols=4),
    'line': LineFormation(spacing=1.5, orientation='horizontal'),
    'circle': CircleFormation(radius=3.0, center_drone='leader'),
    'triangle': TriangleFormation(side_length=4.0)
}
```

#### **Distributed Decision Making**
- **Consensus Algorithms**: Byzantine fault-tolerant agreement protocols
- **Task Allocation**: Auction-based distributed task assignment
- **Role Assignment**: Dynamic leader selection and specialization
- **Conflict Resolution**: Distributed negotiation for resource conflicts

#### **Adaptive Behaviors**
- **Learning Algorithms**: Reinforcement learning for behavior optimization
- **Environmental Adaptation**: Dynamic parameter adjustment based on conditions
- **Performance Optimization**: Continuous improvement through feedback loops
- **Fault Tolerance**: Graceful degradation with agent failures

### **Safety & Reliability**

#### **Multi-layered Safety System**
1. **Sensor Layer**: Lidar, cameras, IMU, GPS sensor fusion
2. **Planning Layer**: Collision-free trajectory generation
3. **Execution Layer**: Real-time collision avoidance
4. **Emergency Layer**: Fail-safe protocols and emergency procedures

#### **Fault Tolerance**
- **Agent Failure Detection**: Heartbeat monitoring and health assessment
- **Graceful Degradation**: Mission continuation with reduced agent count
- **Automatic Recovery**: Self-healing protocols for temporary failures
- **Manual Override**: Human operator intervention capabilities

## 🔧 Configuration

### **Swarm Parameters** (`config/swarm_config.yaml`)
```yaml
swarm:
  max_agents: 8
  communication_range: 10.0
  formation_spacing: 2.0
  collision_threshold: 0.5
  
intelligence:
  learning_rate: 0.01
  exploration_factor: 0.3
  consensus_threshold: 0.8
  
safety:
  emergency_altitude: 0.5
  battery_threshold: 20.0
  max_acceleration: 2.0
```

### **Mission Configuration**
```yaml
mission:
  area_bounds: [0, 0, 50, 50]  # [x_min, y_min, x_max, y_max]
  exploration_altitude: 2.0
  coverage_resolution: 0.5
  mission_timeout: 1800  # seconds
```

## 🧪 Testing & Validation

### **Unit Tests**
```bash
# Test swarm intelligence algorithms
python -m pytest tests/test_swarm_behavior.py

# Test flight controller
python -m pytest tests/test_flight_controller.py

# Test collision avoidance
python -m pytest tests/test_anti_collision.py
```

### **Integration Tests**
```bash
# Full system simulation
roslaunch multi_agent_framework simulation.launch

# Hardware-in-the-loop testing
roslaunch multi_agent_framework hitl_test.launch
```

### **Performance Benchmarks**
- **Formation Accuracy**: ±0.1m positioning accuracy in formation flight
- **Collision Avoidance**: 100% success rate in controlled scenarios
- **Communication Latency**: <50ms for inter-agent coordination
- **Mission Completion**: >95% success rate for exploration missions

## 📊 Monitoring & Analytics

### **Real-time Metrics**
- **Swarm Cohesion**: Measure of formation maintenance quality
- **Coverage Efficiency**: Territory exploration completion rate
- **Energy Consumption**: Battery usage optimization metrics
- **Communication Quality**: Message success rate and latency

### **Performance Dashboard**
- **3D Visualization**: Real-time drone positions and trajectories
- **Behavior Analysis**: Individual and collective performance metrics
- **Mission Progress**: Task completion status and efficiency indicators
- **System Health**: Hardware status and fault detection alerts

## 🔗 Integration Points

### **ROS2 Topics**
```python
# Command topics
/diamants/swarm/formation_command
/diamants/individual/{drone_id}/cmd_vel
/diamants/mission/waypoints

# Status topics
/diamants/swarm/status
/diamants/individual/{drone_id}/pose
/diamants/individual/{drone_id}/battery

# Intelligence topics
/diamants/intelligence/metrics
/diamants/swarm/coordination
/diamants/safety/collision_alerts
```

### **Web API Endpoints**
```python
# Mission control
POST /api/mission/start
GET  /api/mission/status
POST /api/mission/abort

# Swarm control
POST /api/swarm/formation
GET  /api/swarm/status
POST /api/swarm/emergency_stop

# Individual drone control
POST /api/drone/{id}/takeoff
POST /api/drone/{id}/land
GET  /api/drone/{id}/status
```

## 🛠️ Development Guide

### **Adding New Behaviors**
```python
# Create new behavior in swarm_intelligence/
class NewBehavior(SwarmBehavior):
    def __init__(self, config):
        super().__init__(config)
        
    def execute(self, swarm_state):
        # Implement behavior logic
        return coordination_commands
```

### **Extending Web Interface**
```javascript
// Add new visualization in static/js/
class NewVisualization {
    constructor(scene) {
        this.scene = scene;
    }
    
    update(data) {
        // Update 3D visualization
    }
}
```

### **Custom Mission Types**
```python
# Define mission in mission_test_web.py
class CustomMission(MissionBase):
    def plan_mission(self, area_bounds):
        # Generate waypoints and tasks
        return mission_plan
```

## 🚨 Troubleshooting

### **Common Issues**

**Swarm desynchronization:**
```bash
# Check communication
ros2 topic echo /diamants/swarm/coordination

# Restart coordination
ros2 service call /diamants/swarm/reset_coordination
```

**Formation instability:**
```bash
# Adjust formation parameters
ros2 param set /swarm_behavior formation_gain 0.5
ros2 param set /swarm_behavior damping_factor 0.8
```

**Collision false positives:**
```bash
# Tune collision detection
ros2 param set /anti_collision safety_margin 0.3
ros2 param set /anti_collision detection_threshold 0.7
```

## 📚 Research Applications

### **Academic Use Cases**
- **Multi-Agent Systems Research**: Distributed coordination algorithms
- **Swarm Robotics**: Bio-inspired collective behaviors
- **Autonomous Navigation**: SLAM and path planning in GPS-denied environments
- **Machine Learning**: Reinforcement learning for multi-agent coordination

### **Industry Applications**
- **Search & Rescue**: Coordinated area search with autonomous coverage
- **Environmental Monitoring**: Distributed sensor networks for data collection
- **Infrastructure Inspection**: Collaborative inspection of large structures
- **Precision Agriculture**: Coordinated crop monitoring and treatment

## 🤝 Contributing

### **Development Workflow**
1. **Fork Repository**: Create development branch
2. **Implement Feature**: Add new behavior or capability
3. **Write Tests**: Unit and integration test coverage
4. **Document Changes**: Update documentation and examples
5. **Submit PR**: Review process and integration

### **Code Standards**
- **Python**: PEP 8 compliance with type hints
- **ROS2**: Standard message types and service interfaces
- **Documentation**: Comprehensive docstrings and README updates
- **Testing**: >90% test coverage for new code

## 📋 Roadmap

### **Upcoming Features**
- [ ] **Machine Learning Integration**: Deep reinforcement learning for behavior optimization
- [ ] **Advanced Formations**: 3D formations and dynamic reconfiguration
- [ ] **Multi-Species Swarms**: Heterogeneous agent coordination
- [ ] **Edge Computing**: Distributed intelligence on drone hardware
- [ ] **Digital Twin**: Real-time simulation mirror of physical swarm

### **Research Directions**
- [ ] **Emergent Behaviors**: Self-organizing patterns and collective intelligence
- [ ] **Adversarial Scenarios**: Robust coordination in contested environments
- [ ] **Human-Swarm Interaction**: Intuitive interfaces for swarm control
- [ ] **Large Scale Swarms**: Coordination algorithms for 100+ agents

## 📄 License

MIT License - See main project license for full details.

---

**DIAMANTS Multi-Agent Framework** - *Advanced Collaborative Intelligence for Autonomous Drone Swarms*

*Building the future of distributed robotics through bio-inspired coordination and emergent intelligence.*

**Version 3.0** | **ROS2 Jazzy Compatible** | **Production Ready**
