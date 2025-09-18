# 📡 DIAMANTS Multi-Agent Framework - API Reference

**Complete API documentation for developers and integrators**

## 🎯 Overview

This document provides comprehensive API documentation for the DIAMANTS Multi-Agent Framework, including ROS2 topics, services, web endpoints, and Python API interfaces.

## 🤖 ROS2 Interface

### Published Topics

#### Swarm Coordination Topics
```python
# Swarm formation commands
Topic: /diamants/swarm/formation_command
Type: std_msgs/String
Description: Formation pattern commands (grid, line, circle, triangle)
Rate: 1Hz

# Swarm status information
Topic: /diamants/swarm/status
Type: diamants_msgs/SwarmStatus
Description: Overall swarm health and coordination status
Rate: 10Hz

# Inter-agent coordination messages
Topic: /diamants/swarm/coordination
Type: diamants_msgs/CoordinationMessage
Description: Peer-to-peer coordination data
Rate: 20Hz
```

#### Individual Drone Topics
```python
# Command velocity for each drone
Topic: /diamants/drone/{drone_id}/cmd_vel
Type: geometry_msgs/Twist
Description: Velocity commands for individual drones
Rate: 50Hz

# Drone pose information
Topic: /diamants/drone/{drone_id}/pose
Type: geometry_msgs/PoseStamped
Description: Current position and orientation
Rate: 30Hz

# Battery status
Topic: /diamants/drone/{drone_id}/battery
Type: sensor_msgs/BatteryState
Description: Battery level and charging status
Rate: 1Hz

# Drone health status
Topic: /diamants/drone/{drone_id}/status
Type: diamants_msgs/DroneStatus
Description: Operational status and error conditions
Rate: 5Hz
```

#### Mission Management Topics
```python
# Mission waypoints
Topic: /diamants/mission/waypoints
Type: diamants_msgs/WaypointArray
Description: Mission waypoint sequences
Rate: On demand

# Mission status updates
Topic: /diamants/mission/status
Type: diamants_msgs/MissionStatus
Description: Current mission progress and status
Rate: 2Hz

# Exploration coverage map
Topic: /diamants/mission/coverage_map
Type: nav_msgs/OccupancyGrid
Description: Explored area coverage map
Rate: 0.5Hz
```

#### Safety and Collision Avoidance
```python
# Collision alerts
Topic: /diamants/safety/collision_alerts
Type: diamants_msgs/CollisionAlert
Description: Real-time collision warnings
Rate: Event-driven

# Emergency stop commands
Topic: /diamants/safety/emergency_stop
Type: std_msgs/Bool
Description: Emergency stop for entire swarm
Rate: Event-driven

# Safety zone violations
Topic: /diamants/safety/zone_violations
Type: diamants_msgs/SafetyViolation
Description: No-fly zone or altitude violations
Rate: Event-driven
```

### ROS2 Services

#### Swarm Control Services
```python
# Start formation
Service: /diamants/swarm/start_formation
Type: diamants_srvs/StartFormation
Request:
  - formation_type: string
  - formation_params: float64[]
Response:
  - success: bool
  - message: string

# Emergency abort
Service: /diamants/swarm/emergency_abort
Type: std_srvs/Trigger
Request: {}
Response:
  - success: bool
  - message: string

# Reset coordination
Service: /diamants/swarm/reset_coordination
Type: std_srvs/Trigger
Request: {}
Response:
  - success: bool
  - message: string
```

#### Individual Drone Services
```python
# Takeoff command
Service: /diamants/drone/{drone_id}/takeoff
Type: diamants_srvs/Takeoff
Request:
  - target_altitude: float32
Response:
  - success: bool
  - message: string

# Landing command
Service: /diamants/drone/{drone_id}/land
Type: diamants_srvs/Land
Request:
  - landing_position: geometry_msgs/Point
Response:
  - success: bool
  - message: string

# Set home position
Service: /diamants/drone/{drone_id}/set_home
Type: diamants_srvs/SetHome
Request:
  - home_position: geometry_msgs/Point
Response:
  - success: bool
  - message: string
```

#### Mission Management Services
```python
# Start mission
Service: /diamants/mission/start
Type: diamants_srvs/StartMission
Request:
  - mission_type: string
  - area_bounds: geometry_msgs/Polygon
  - mission_params: diamants_msgs/MissionParams
Response:
  - success: bool
  - mission_id: string
  - estimated_duration: duration

# Pause/Resume mission
Service: /diamants/mission/pause
Type: std_srvs/SetBool
Request:
  - data: bool  # true to pause, false to resume
Response:
  - success: bool
  - message: string

# Get mission status
Service: /diamants/mission/get_status
Type: diamants_srvs/GetMissionStatus
Request:
  - mission_id: string
Response:
  - status: diamants_msgs/MissionStatus
  - completion_percentage: float32
```

## 🌐 Web API Endpoints

### RESTful API

#### Swarm Management
```python
# Get swarm status
GET /api/v1/swarm/status
Response:
{
  "swarm_id": "diamants_swarm_01",
  "active_drones": 8,
  "formation_active": true,
  "formation_type": "grid",
  "overall_health": "good",
  "timestamp": "2025-09-18T10:30:00Z"
}

# Set formation
POST /api/v1/swarm/formation
Request:
{
  "formation_type": "circle",
  "parameters": {
    "radius": 5.0,
    "altitude": 2.0
  }
}
Response:
{
  "success": true,
  "formation_id": "formation_001",
  "estimated_transition_time": 30
}

# Emergency stop
POST /api/v1/swarm/emergency_stop
Request: {}
Response:
{
  "success": true,
  "stopped_drones": 8,
  "timestamp": "2025-09-18T10:35:00Z"
}
```

#### Individual Drone Control
```python
# Get drone information
GET /api/v1/drone/{drone_id}
Response:
{
  "drone_id": "crazyflie_01",
  "position": {"x": 1.5, "y": 2.3, "z": 1.8},
  "velocity": {"x": 0.1, "y": 0.0, "z": 0.0},
  "battery_level": 85,
  "status": "flying",
  "last_update": "2025-09-18T10:30:00Z"
}

# Send takeoff command
POST /api/v1/drone/{drone_id}/takeoff
Request:
{
  "target_altitude": 1.5
}
Response:
{
  "success": true,
  "takeoff_initiated": true,
  "estimated_time": 10
}

# Send goto command
POST /api/v1/drone/{drone_id}/goto
Request:
{
  "target_position": {"x": 5.0, "y": 3.0, "z": 2.0},
  "speed": 0.5
}
Response:
{
  "success": true,
  "trajectory_id": "traj_001",
  "estimated_time": 25
}
```

#### Mission Management
```python
# Create new mission
POST /api/v1/mission/create
Request:
{
  "mission_type": "exploration",
  "area_bounds": {
    "min_x": 0, "min_y": 0,
    "max_x": 20, "max_y": 20
  },
  "parameters": {
    "coverage_threshold": 0.9,
    "exploration_altitude": 2.0
  }
}
Response:
{
  "success": true,
  "mission_id": "mission_001",
  "estimated_duration": 600
}

# Get mission status
GET /api/v1/mission/{mission_id}/status
Response:
{
  "mission_id": "mission_001",
  "status": "in_progress",
  "completion_percentage": 65.3,
  "elapsed_time": 390,
  "estimated_remaining": 210,
  "coverage_achieved": 0.653
}

# Abort mission
POST /api/v1/mission/{mission_id}/abort
Request: {}
Response:
{
  "success": true,
  "mission_aborted": true,
  "return_to_home_initiated": true
}
```

### WebSocket API

#### Real-time Telemetry Stream
```javascript
// Connect to WebSocket
const ws = new WebSocket('ws://localhost:8765/telemetry');

// Message format for telemetry data
{
  "type": "telemetry",
  "timestamp": 1695027000000,
  "data": {
    "swarm_status": {
      "formation_active": true,
      "formation_error": 0.05,
      "coordination_quality": 0.92
    },
    "drones": [
      {
        "id": "crazyflie_01",
        "position": {"x": 1.5, "y": 2.3, "z": 1.8},
        "velocity": {"x": 0.1, "y": 0.0, "z": 0.0},
        "battery": 85,
        "status": "flying"
      }
    ]
  }
}
```

#### Command Interface
```javascript
// Send command via WebSocket
const command = {
  "type": "command",
  "target": "swarm",
  "action": "set_formation",
  "parameters": {
    "formation_type": "line",
    "spacing": 2.0
  }
};
ws.send(JSON.stringify(command));

// Response format
{
  "type": "command_response",
  "command_id": "cmd_001",
  "success": true,
  "message": "Formation command accepted"
}
```

## 🐍 Python API

### SwarmBehavior Class API

```python
class SwarmBehavior:
    """Main swarm coordination class"""
    
    def __init__(self, drone_id: str, config: Dict = None):
        """
        Initialize swarm behavior for a drone
        
        Args:
            drone_id: Unique identifier for this drone
            config: Configuration parameters
        """
        
    def update_formation(self, target_pattern: str, parameters: Dict = None) -> Twist:
        """
        Update formation based on target pattern
        
        Args:
            target_pattern: Formation type ('grid', 'line', 'circle', 'triangle')
            parameters: Formation-specific parameters
            
        Returns:
            Twist: Velocity command for formation maintenance
        """
        
    def process_neighbor_data(self, neighbor_data: List[NeighborState]) -> None:
        """
        Process data from neighboring drones
        
        Args:
            neighbor_data: List of neighbor states and positions
        """
        
    def get_coordination_message(self) -> CoordinationMessage:
        """
        Generate coordination message for neighbors
        
        Returns:
            CoordinationMessage: Data to share with neighbors
        """
        
    def emergency_response(self, emergency_type: str) -> Twist:
        """
        Execute emergency response behavior
        
        Args:
            emergency_type: Type of emergency ('collision', 'low_battery', 'communication_loss')
            
        Returns:
            Twist: Emergency maneuver command
        """
```

### FlightController Class API

```python
class FlightController:
    """Individual drone flight control"""
    
    def __init__(self, drone_id: str, config: Dict = None):
        """Initialize flight controller"""
        
    def set_target_position(self, target: Point) -> None:
        """
        Set target position for autonomous navigation
        
        Args:
            target: Target position (x, y, z)
        """
        
    def update_control(self, current_state: DroneState) -> Twist:
        """
        Calculate control command based on current state
        
        Args:
            current_state: Current drone position, velocity, orientation
            
        Returns:
            Twist: Control command (linear and angular velocities)
        """
        
    def plan_trajectory(self, start: Point, goal: Point, 
                       obstacles: List[Obstacle] = None) -> Trajectory:
        """
        Plan collision-free trajectory
        
        Args:
            start: Starting position
            goal: Goal position
            obstacles: List of known obstacles
            
        Returns:
            Trajectory: Planned path with waypoints and timing
        """
        
    def emergency_land(self) -> bool:
        """
        Execute emergency landing procedure
        
        Returns:
            bool: True if landing initiated successfully
        """
```

### WebServer Class API

```python
class WebServer:
    """Web interface server"""
    
    def __init__(self, host: str = "localhost", port: int = 8000):
        """Initialize web server"""
        
    async def start_server(self) -> None:
        """Start the web server"""
        
    def register_drone(self, drone_id: str, initial_state: DroneState) -> bool:
        """
        Register a new drone with the system
        
        Args:
            drone_id: Unique identifier
            initial_state: Initial drone state
            
        Returns:
            bool: True if registration successful
        """
        
    def get_swarm_status(self) -> SwarmStatus:
        """
        Get current swarm status
        
        Returns:
            SwarmStatus: Current swarm state and health
        """
        
    def send_formation_command(self, formation_type: str, 
                             parameters: Dict = None) -> bool:
        """
        Send formation command to swarm
        
        Args:
            formation_type: Target formation pattern
            parameters: Formation-specific parameters
            
        Returns:
            bool: True if command sent successfully
        """
```

## 📊 Message Definitions

### Custom ROS2 Messages

#### SwarmStatus.msg
```python
# Swarm overall status
std_msgs/Header header
string swarm_id
uint8 active_drones
bool formation_active
string formation_type
float32 formation_error
float32 coordination_quality
float32 average_battery_level
uint8 STATUS_GOOD=0
uint8 STATUS_WARNING=1
uint8 STATUS_ERROR=2
uint8 overall_status
```

#### DroneStatus.msg
```python
# Individual drone status
std_msgs/Header header
string drone_id
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped velocity
float32 battery_level
uint8 FLYING=0
uint8 LANDED=1
uint8 TAKEOFF=2
uint8 LANDING=3
uint8 EMERGENCY=4
uint8 flight_mode
string[] active_behaviors
```

#### CoordinationMessage.msg
```python
# Inter-drone coordination data
std_msgs/Header header
string sender_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 battery_level
string current_task
float32 task_priority
geometry_msgs/Point[] planned_waypoints
```

### Service Definitions

#### StartFormation.srv
```python
# Request
string formation_type
float64[] formation_parameters
geometry_msgs/Point formation_center
float32 formation_size
---
# Response
bool success
string message
string formation_id
duration estimated_transition_time
```

#### StartMission.srv
```python
# Request
string mission_type
geometry_msgs/Polygon area_bounds
float32 target_altitude
float32 coverage_threshold
duration max_duration
---
# Response
bool success
string mission_id
duration estimated_duration
geometry_msgs/Point[] initial_waypoints
```

## 🔧 Configuration Parameters

### ROS2 Parameters

#### Swarm Behavior Parameters
```yaml
swarm_behavior:
  communication_range: 10.0      # meters
  formation_gain: 1.0            # formation control gain
  separation_distance: 2.0       # minimum separation between drones
  max_velocity: 2.0              # m/s
  coordination_rate: 20.0        # Hz
```

#### Flight Controller Parameters
```yaml
flight_controller:
  pid_gains:
    position:
      kp: [1.0, 1.0, 1.5]        # P gains for x, y, z
      ki: [0.1, 0.1, 0.2]        # I gains for x, y, z
      kd: [0.05, 0.05, 0.1]      # D gains for x, y, z
    attitude:
      kp: 2.0                    # Yaw P gain
      ki: 0.1                    # Yaw I gain
      kd: 0.1                    # Yaw D gain
  max_acceleration: 2.0          # m/s²
  max_angular_velocity: 1.57     # rad/s
```

#### Safety Parameters
```yaml
safety:
  collision_threshold: 0.5       # meters
  emergency_altitude: 0.3        # meters
  battery_warning_level: 30.0    # percent
  battery_critical_level: 15.0   # percent
  geofence_enabled: true
  max_altitude: 5.0              # meters
```

## 🚨 Error Codes

### API Error Codes
```python
class ErrorCodes:
    SUCCESS = 0
    INVALID_PARAMETER = 1001
    DRONE_NOT_FOUND = 1002
    MISSION_ACTIVE = 1003
    FORMATION_FAILED = 1004
    COMMUNICATION_ERROR = 1005
    SAFETY_VIOLATION = 1006
    BATTERY_LOW = 1007
    HARDWARE_FAILURE = 1008
    TIMEOUT = 1009
    UNKNOWN_ERROR = 9999
```

### Safety Alert Codes
```python
class SafetyAlerts:
    COLLISION_IMMINENT = 2001
    ALTITUDE_VIOLATION = 2002
    GEOFENCE_VIOLATION = 2003
    COMMUNICATION_LOSS = 2004
    BATTERY_CRITICAL = 2005
    HARDWARE_MALFUNCTION = 2006
    WEATHER_WARNING = 2007
```

## 📖 Usage Examples

### Basic Swarm Control
```python
import rclpy
from diamants_framework import SwarmController

# Initialize ROS2 and controller
rclpy.init()
controller = SwarmController()

# Take off all drones
controller.takeoff_swarm(target_altitude=2.0)

# Set formation
controller.set_formation('grid', spacing=2.0)

# Start exploration mission
mission_id = controller.start_exploration_mission(
    area_bounds=[(0, 0), (20, 20)],
    coverage_threshold=0.9
)

# Monitor mission progress
while controller.is_mission_active(mission_id):
    status = controller.get_mission_status(mission_id)
    print(f"Coverage: {status.completion_percentage:.1f}%")
    time.sleep(1)

# Land all drones
controller.land_swarm()
```

### Web Interface Integration
```javascript
// Connect to DIAMANTS WebSocket
const ws = new WebSocket('ws://localhost:8765/telemetry');

// Handle telemetry data
ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    if (data.type === 'telemetry') {
        updateDroneVisualization(data.data.drones);
        updateSwarmMetrics(data.data.swarm_status);
    }
};

// Send formation command
function setFormation(type) {
    const command = {
        type: 'command',
        target: 'swarm',
        action: 'set_formation',
        parameters: { formation_type: type }
    };
    ws.send(JSON.stringify(command));
}
```

---

**For implementation details and technical specifications, see TECHNICAL_SPECIFICATION.md. For high-level overview, see README_MULTI_AGENT_FRAMEWORK.md.**
