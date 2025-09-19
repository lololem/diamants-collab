# 🏗 System Architecture

Complete technical architecture of the DIAMANTS distributed multi-agent drone system.

## 🎯 Overall Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    DIAMANTS ECOSYSTEM                           │
├─────────────────┬─────────────────┬─────────────────────────────┤
│   Frontend      │   API Bridge    │       Backend               │
│   Three.js      │   FastAPI       │       ROS2 Jazzy           │
│   WebGL         │   WebSocket     │       Gazebo Garden         │
│   React UI      │   REST API      │       Navigation Stack     │
└─────────────────┴─────────────────┴─────────────────────────────┘
```

## 🚁 System Components

### Core Subsystems

1. **Frontend Interface** (DIAMANTS_FRONTEND)
2. **API Communication Bridge** (DIAMANTS_API)
3. **ROS2 Backend** (DIAMANTS_BACKEND)

### Data Flow Architecture

```
User Interface (Three.js)
    ↕ WebSocket
API Bridge (FastAPI)
    ↕ ROS2 Topics/Services
Backend (ROS2 Jazzy + Gazebo)
    ↕ Radio/USB
Physical Drones (Crazyflie)
```

## 💻 Frontend Architecture (DIAMANTS_FRONTEND)

### Technology Stack
- **Three.js r160**: 3D rendering engine
- **WebGL 2.0**: Hardware-accelerated graphics
- **Vite**: Build tool and development server
- **ES6 Modules**: Modern JavaScript architecture

### Component Structure

```
Mission_system/
├── core/
│   ├── diamant-system.js       # Main system orchestrator
│   ├── app-state.js            # Global application state
│   └── event-manager.js        # Event handling system
├── visual/
│   ├── three-visualizer.js     # 3D scene management
│   ├── drone-visual.js         # Drone 3D representation
│   └── environment-renderer.js # Environment visualization
├── controllers/
│   ├── camera-controller.js    # Camera movement and controls
│   ├── mission-controller.js   # Mission planning interface
│   └── input-handler.js        # User input processing
├── ui/
│   ├── control-panel.js        # Main control interface
│   ├── telemetry-dashboard.js  # Real-time metrics display
│   └── mission-planner.js      # Visual mission creation
├── intelligence/
│   ├── collective-intelligence.js    # Basic swarm intelligence
│   └── advanced-collective-intelligence.js # Advanced AI algorithms
├── physics/
│   ├── physics-engine.js       # Physics simulation
│   └── collision-detection.js  # Collision system
├── net/
│   ├── websocket-manager.js    # WebSocket communication
│   └── api-client.js           # REST API client
└── behaviors/
    ├── flight-behaviors.js     # Drone flight patterns
    └── collaborative-scouting.js # Swarm behaviors
```

### Rendering Pipeline

```
Scene Setup → Drone Loading → Physics Update → Render Loop
    ↓              ↓              ↓             ↓
Lighting      3D Models     Calculations   60 FPS Output
Environment   Animations    Collisions    WebGL Render
```

## 🛰 API Bridge Architecture (DIAMANTS_API)

### Technology Stack
- **FastAPI**: Modern Python web framework
- **WebSocket**: Real-time bidirectional communication
- **Pydantic**: Data validation and serialization
- **asyncio**: Asynchronous programming

### Service Architecture

```
api/
├── main.py                    # FastAPI application entry
├── config.py                  # Configuration management
├── models.py                  # Data models and schemas
├── websocket_bridge.py        # WebSocket server
├── websocket_bridge_simple.py # Simplified WebSocket
└── ros2_stubs.py              # ROS2 integration stubs
```

### Communication Protocols

**WebSocket Messages:**
```json
{
  "type": "telemetry_update",
  "drone_id": "cf2x_01",
  "timestamp": "2025-09-19T10:30:00Z",
  "data": {
    "position": {"x": 1.2, "y": 0.8, "z": 1.5},
    "velocity": {"vx": 0.1, "vy": 0.0, "vz": 0.0},
    "battery": 85
  }
}
```

**REST API Endpoints:**
- GET `/api/drones` - List all drones
- POST `/api/drones/{id}/takeoff` - Initiate takeoff
- PUT `/api/drones/{id}/position` - Update position
- POST `/api/missions/create` - Create mission
- GET `/api/health` - System health check

### Data Processing Pipeline

```
Frontend Request → FastAPI Router → Data Validation → ROS2 Bridge → Backend
Frontend Update ← WebSocket Push ← Data Transform ← ROS2 Callback ← Backend
```

## 🤖 Backend Architecture (DIAMANTS_BACKEND)

### Technology Stack
- **ROS2 Jazzy**: Robot Operating System
- **Gazebo Garden**: Physics simulation
- **Python 3.10**: Primary programming language
- **CycloneDX**: DDS implementation

### ROS2 Workspace Structure

```
slam_collaboratif/ros2_ws/src/
├── crazyflie_ros/             # Crazyflie drivers
├── diamants_msgs/             # Custom message definitions
├── diamants_launch/           # Launch file configurations
├── navigation_stack/          # Path planning and navigation
├── simulation_worlds/         # Gazebo world definitions
└── collective_intelligence/   # Swarm intelligence algorithms
```

### Node Architecture

**Primary Nodes:**
1. **Drone Control Node**: Individual drone management
2. **Swarm Coordinator**: Multi-drone coordination
3. **Mission Executor**: Mission planning and execution
4. **Safety Monitor**: Emergency systems and collision avoidance
5. **Telemetry Publisher**: Data streaming to API bridge

### ROS2 Topic Structure

```
# Command Topics
/cf2x_01/cmd_vel              # geometry_msgs/Twist
/cf2x_01/cmd_position         # geometry_msgs/PoseStamped
/swarm/formation_command      # diamants_msgs/FormationCommand

# Status Topics
/cf2x_01/pose                 # geometry_msgs/PoseStamped
/cf2x_01/battery_state        # sensor_msgs/BatteryState
/swarm/collective_state       # diamants_msgs/CollectiveState

# System Topics
/system/emergency             # std_msgs/Bool
/mission/status               # diamants_msgs/MissionStatus
```

## 🧠 Collective Intelligence Architecture

### DIAMANTS Algorithm Framework

The system implements 15 harmonics for collective intelligence:

```python
I(t) = Σ(n=1 to 15) αₙ × Hₙ(t)
```

Where:
- `I(t)`: Collective intelligence at time t
- `αₙ`: Weighting coefficient for harmonic n
- `Hₙ(t)`: Value of harmonic n at time t

### Intelligence Components

1. **Basic Harmonics (H1-H5)**
   - External field, Internal field, Total energy
   - Interaction forces, Curvature measures

2. **Dynamic Harmonics (H6-H10)**
   - Noise factors, Evolution rates, Stability measures
   - Anisotropy, Flux curvature

3. **Advanced Harmonics (H11-H15)**
   - Symmetry, Vorticity, Coherence
   - Flow dynamics, Entropy measures

### Emergent Behavior Engine

```python
class CollectiveIntelligence:
    def __init__(self):
        self.harmonic_calculator = HarmonicCalculator()
        self.behavior_engine = EmergentBehaviorEngine()
        self.consensus_system = DistributedConsensus()
    
    def update(self, drones, environment):
        # Calculate 15 DIAMANTS harmonics
        harmonics = self.harmonic_calculator.compute_all(drones)
        
        # Generate emergent behaviors
        behaviors = self.behavior_engine.generate(harmonics)
        
        # Apply to drone swarm
        self.apply_behaviors(drones, behaviors)
```

## 🌐 Network Architecture

### Communication Layers

```
Application Layer    │ Three.js UI ↔ User Interaction
Presentation Layer   │ WebSocket ↔ JSON Serialization
Session Layer        │ FastAPI ↔ Connection Management
Transport Layer      │ TCP/UDP ↔ ROS2 DDS
Network Layer        │ IP ↔ Multi-machine Support
Physical Layer       │ Ethernet/WiFi ↔ Hardware
```

### Multi-Machine Setup

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Control PC    │    │   Simulation    │    │   Flight PC     │
│   - Frontend    │◄──►│   - Gazebo      │◄──►│   - Real Drones │
│   - API Bridge  │    │   - Physics     │    │   - Hardware    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Security Architecture

- **Authentication**: Token-based API access
- **Encryption**: TLS 1.3 for all communications
- **Authorization**: Role-based access control
- **Network Isolation**: VPN for distributed setups

## 📊 Data Architecture

### Data Flow Types

1. **Command Flow**: User → Frontend → API → ROS2 → Drones
2. **Telemetry Flow**: Drones → ROS2 → API → Frontend → Display
3. **Mission Flow**: Planner → Validator → Executor → Monitor

### Database Schema (Future)

```sql
-- Missions table
CREATE TABLE missions (
    id UUID PRIMARY KEY,
    name VARCHAR(255),
    status ENUM('planned', 'active', 'completed', 'failed'),
    waypoints JSONB,
    created_at TIMESTAMP,
    updated_at TIMESTAMP
);

-- Telemetry table
CREATE TABLE telemetry (
    id BIGSERIAL PRIMARY KEY,
    drone_id VARCHAR(50),
    timestamp TIMESTAMP,
    position POINT,
    velocity VECTOR,
    battery_level FLOAT,
    sensors JSONB
);
```

### Message Formats

**Telemetry Message:**
```protobuf
message TelemetryData {
    string drone_id = 1;
    google.protobuf.Timestamp timestamp = 2;
    Position position = 3;
    Velocity velocity = 4;
    float battery_level = 5;
    SensorData sensors = 6;
}
```

## 🔧 Configuration Architecture

### Configuration Hierarchy

```
1. Default Values (hardcoded)
2. Configuration Files (YAML)
3. Environment Variables
4. Command Line Arguments
5. Runtime Overrides
```

### Configuration Files

**System Configuration:**
```yaml
# config/system.yaml
system:
  name: "DIAMANTS"
  version: "1.0.0"
  max_drones: 50
  update_rate: 50

drones:
  default_type: "crazyflie_2x"
  spawn_height: 0.1
  safety_distance: 0.5

physics:
  gravity: -9.81
  air_density: 1.225
  wind_enabled: false
```

## 🚀 Deployment Architecture

### Development Environment

```bash
# Single machine development
Frontend (localhost:3000) ↔ API (localhost:8080) ↔ ROS2 (local)
```

### Production Environment

```bash
# Distributed production setup
Load Balancer → Multiple API Instances → ROS2 Cluster → Drone Fleet
```

### Container Architecture (Future)

```dockerfile
# Docker Compose setup
services:
  frontend:
    build: ./DIAMANTS_FRONTEND
    ports: ["3000:3000"]
  
  api:
    build: ./DIAMANTS_API
    ports: ["8080:8080"]
    depends_on: [backend]
  
  backend:
    build: ./DIAMANTS_BACKEND
    volumes: ["/dev:/dev"]
    privileged: true
```

## 🔍 Monitoring Architecture

### Health Monitoring

```python
class SystemHealthMonitor:
    def __init__(self):
        self.metrics = {
            'frontend_fps': 0,
            'api_latency': 0,
            'ros2_hz': 0,
            'drone_count': 0,
            'error_rate': 0
        }
    
    def collect_metrics(self):
        # Collect from all system components
        return self.aggregate_health_score()
```

### Logging Architecture

```
Application Logs → Structured Logging → Central Aggregation → Analysis
     ↓                     ↓                     ↓              ↓
  Component           JSON Format           Log Server     Alerting
```

This architecture provides a robust, scalable foundation for autonomous drone swarm operations with real-time coordination and intelligent collective behaviors.