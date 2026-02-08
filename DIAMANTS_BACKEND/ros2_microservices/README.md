# DIAMANTS Microservices Architecture
## Drone Intelligence for Advanced Mapping and Navigation Through Swarms

---

## Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                     DIAMANTS Backend Architecture                    │
│                                                                      │
│  ┌──────────────┐  Gazebo Topics    ┌──────────────────────┐        │
│  │ Gazebo       │ ───────────────→  │ ros_gz_bridge        │        │
│  │ Harmonic     │ ←─────────────── │ (per-drone)          │        │
│  │ (headless)   │   cmd_vel         └──────────┬───────────┘        │
│  │ 6 Crazyflie  │                  /{cfN}/odom  │  /{cfN}/scan      │
│  └──────────────┘                              │                    │
│                    ┌───────────────────────────┼──────────┐         │
│                    │                           │          │         │
│           ┌────────▼─────────┐  ┌─────────────▼──┐  ┌───▼───────┐ │
│           │ SwarmController  │  │ Position       │  │ SLAM      │ │
│           │ • Social forces  │  │ Broadcaster    │  │ Fusion    │ │
│           │ • Exploration    │  │ • Aggregates   │  │ • Lidar   │ │
│           │ • Obstacle avoid │  │   odom → JSON  │  │ • Grid    │ │
│           │ • Flight FSM     │  │ • 10 Hz        │  │ • Merge   │ │
│           └────────┬─────────┘  └────────────────┘  └───────────┘ │
│                    │ cmd_vel                                        │
│           ┌────────▼─────────────────────┐                         │
│           │ MissionCoordinator           │                         │
│           │ • Auto-start                 │                         │
│           │ • Mission lifecycle          │                         │
│           │ • Health monitoring          │                         │
│           └──────────────────────────────┘                         │
│                                                                      │
│  ┌────────────────────────────────────────────────────────┐         │
│  │ WebSocket Bridge (8765) ←→ ROS2 /diamants/* topics     │         │
│  │ FastAPI REST    (8000)                                  │         │
│  └────────────────────────────────────────────────────────┘         │
│                            ↕ ws://                                   │
│  ┌────────────────────────────────────────────────────────┐         │
│  │ Frontend (Three.js) — Presentation Layer Only          │         │
│  └────────────────────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────────────────┘
```

## Microservices

### 1. SwarmController (`swarm_controller.py`)
**Role**: Core swarm intelligence — decides where each drone flies.

| Aspect | Detail |
|--------|--------|
| **Subscribes** | `/{cfN}/odom` (6 drones), `/{cfN}/scan` (lidar), `/diamants/mission/commands`, `/diamants/swarm/commands` |
| **Publishes** | `/{cfN}/cmd_vel` (velocity commands), `/diamants/swarm/intelligence_score`, `/diamants/swarm/coverage_area`, `/diamants/swarm/status`, `/diamants/drones/propeller_speeds` |
| **Rate** | 10 Hz control loop, 1 Hz metrics |

**Algorithms**:
- **Social forces** (Boids-like): repulsion < 1m, cohesion < 6.4m
- **Exploration incentive**: 8-direction frontier sampling, coverage grid
- **Waypoint planning**: sector-based, per-drone, unvisited-first
- **Obstacle avoidance**: lidar-based reactive forces
- **Flight FSM**: idle → takeoff → cruise → explore → return → land
- **Staggered takeoff**: 2s between each drone

### 2. PositionBroadcaster (`position_broadcaster.py`)
**Role**: Aggregates per-drone Gazebo odometry into the unified format consumed by the WebSocket bridge.

| Aspect | Detail |
|--------|--------|
| **Subscribes** | `/{cfN}/odom`, `/diamants/swarm/status` |
| **Publishes** | `/diamants/drones/positions` (10 Hz), `/diamants/drones/telemetry` (5 Hz) |

**Transforms**: Gazebo coordinates (x, y, z meters) → Frontend coordinates (scaled 10x, axis-swapped).

### 3. SLAMFusion (`slam_fusion.py`)
**Role**: Collaborative SLAM map merging from multi-drone lidar data.

| Aspect | Detail |
|--------|--------|
| **Subscribes** | `/{cfN}/odom`, `/{cfN}/scan` |
| **Publishes** | `/diamants/slam/map` (OccupancyGrid), `/diamants/slam/map_json` (sparse JSON), `/diamants/slam/coverage` (Float32) |
| **Rate** | 2 Hz fusion, 1 Hz JSON publish |

**Fusion Algorithm** (hybrid):
- **Stigmergy**: pheromone accumulation at visited cells, evaporation 0.98/cycle
- **Consensus**: multi-drone voting (majority wins)
- **Union**: any observation wins (occupied overrides free)
- **Weights**: 0.35 × stigmergy + 0.35 × consensus + 0.2 × union + 0.1 × coverage

### 4. MissionCoordinator (`mission_coordinator.py`)
**Role**: Mission lifecycle management and health monitoring.

| Aspect | Detail |
|--------|--------|
| **Subscribes** | `/diamants/mission/commands`, `/diamants/swarm/status`, `/diamants/slam/map_json`, `/diamants/drones/commands` |
| **Publishes** | `/diamants/mission/status`, `/diamants/system/status` |
| **Rate** | 1 Hz status, 5s health check |

**States**: idle → starting → active → paused → returning → complete → emergency

## Data Flow

```
Frontend button click
    → WebSocket message { type: "mission_command", data: { action: "start" } }
    → DiamantsBridge (port 8765)
    → ROS2 topic /diamants/mission/commands
    → MissionCoordinator
    → SwarmController receives "start" command
    → Drones take off sequentially (2s stagger)
    → /{cfN}/cmd_vel → Gazebo → /{cfN}/odom
    → PositionBroadcaster → /diamants/drones/positions
    → DiamantsBridge → WebSocket → Frontend (Three.js renders positions)
```

## ROS2 Topic Map

| Topic | Type | Producer | Consumer |
|-------|------|----------|----------|
| `/{cfN}/odom` | Odometry | Gazebo (via bridge) | SwarmController, PositionBroadcaster, SLAMFusion |
| `/{cfN}/scan` | LaserScan | Gazebo (via bridge) | SwarmController, SLAMFusion |
| `/{cfN}/cmd_vel` | Twist | SwarmController | Gazebo (via bridge) |
| `/diamants/drones/positions` | String (JSON) | PositionBroadcaster | WebSocket Bridge |
| `/diamants/drones/telemetry` | String (JSON) | PositionBroadcaster | WebSocket Bridge |
| `/diamants/drones/propeller_speeds` | String (JSON) | SwarmController | WebSocket Bridge |
| `/diamants/swarm/intelligence_score` | Float32 | SwarmController | WebSocket Bridge |
| `/diamants/swarm/coverage_area` | Float32 | SwarmController | WebSocket Bridge |
| `/diamants/swarm/status` | String (JSON) | SwarmController | MissionCoordinator, PositionBroadcaster |
| `/diamants/slam/map` | OccupancyGrid | SLAMFusion | (RViz) |
| `/diamants/slam/map_json` | String (JSON) | SLAMFusion | MissionCoordinator, WebSocket Bridge |
| `/diamants/mission/status` | String (JSON) | MissionCoordinator | WebSocket Bridge |
| `/diamants/system/status` | String (JSON) | MissionCoordinator | WebSocket Bridge |
| `/diamants/mission/commands` | String (JSON) | WebSocket Bridge | SwarmController, MissionCoordinator |
| `/diamants/swarm/commands` | String (JSON) | WebSocket Bridge | SwarmController |

## Launch

### Quick Start
```bash
./launch_diamants_full.sh
```

### Options
```bash
./launch_diamants_full.sh --gui          # Gazebo with GUI
./launch_diamants_full.sh --drones 4     # 4 drones only
./launch_diamants_full.sh --no-frontend  # Backend only
./launch_diamants_full.sh --no-gazebo    # API only (dev)
```

### ROS2 Launch (direct)
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch diamants_microservices diamants_full.launch.py headless:=True num_drones:=6
```

## File Structure
```
DIAMANTS_BACKEND/ros2_microservices/
├── package.xml                          # ROS2 package manifest
├── setup.py                             # Python package setup
├── setup.cfg                            # Install paths
├── resource/diamants_microservices      # ament resource marker
├── config/
│   └── diamants_params.yaml             # Tunable parameters
├── launch/
│   └── diamants_full.launch.py          # Full system launch file
└── diamants_microservices/
    ├── __init__.py
    ├── swarm_controller.py              # Swarm intelligence (530 lines)
    ├── position_broadcaster.py          # Position aggregation (200 lines)
    ├── slam_fusion.py                   # SLAM map fusion (340 lines)
    └── mission_coordinator.py           # Mission lifecycle (260 lines)
```
