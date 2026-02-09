# DIAMANTS Backend

Multi-drone SITL simulation powered by ROS2 Jazzy + Gazebo Harmonic.
8 Crazyflie drones with collaborative SLAM, swarm intelligence, and
real-time telemetry streaming to the DIAMANTS frontend.

## Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy (`/opt/ros/jazzy/`)
- Gazebo Harmonic
- Python 3.12

## Quick Start

```bash
cd DIAMANTS_BACKEND

# 1. First time: install deps + build
./setup.sh

# 2. Launch (headless, forest world, 8 drones)
make launch

# 3. In another terminal: start WebSocket bridge + API
make bridge
```

The frontend connects automatically via WebSocket on port 8765.

## Launch Options

```bash
make launch                  # Headless, forest, 8 drones (default)
make launch-gui              # With Gazebo 3D GUI
make launch-indoor           # Indoor warehouse world
make launch-open             # Empty flat world (original)
make launch DRONES=2         # 2 drones only
make launch WORLD=indoor DRONES=4 HEADLESS=False
```

| Variable | Default | Description |
|----------|---------|-------------|
| `DRONES` | 8 | Number of Crazyflie drones (1-8) |
| `WORLD` | forest | World: `forest`, `indoor`, `open` |
| `HEADLESS` | True | `True` = no Gazebo GUI, `False` = 3D view |
| `ALTITUDE` | 0.5 | Target flight altitude (m) |
| `RADIUS` | 20.0 | Exploration radius (m) |

## Architecture

```
Gazebo Harmonic (8 Crazyflies in SDF world)
    |
    |-- /crazyflie*/cmd_vel    <- SwarmController (social forces + PD altitude)
    |-- /crazyflie*/enable     <- MissionCoordinator (arm/disarm)
    |-- /crazyflie*/odom       -> PositionBroadcaster -> /diamants/drones/positions
    |-- /crazyflie*/scan       -> SLAMFusion -> /diamants/slam/merged_map
    '-- /clock                 -> ROS2 time sync
                    |
         WebSocket Bridge (port 8765) <-> Frontend
         REST API (port 8000)
```

## ROS2 Microservices

| Node | File | Role |
|------|------|------|
| SwarmController | `swarm_controller.py` | Boids social forces, frontier exploration, PD altitude, obstacle avoidance |
| PositionBroadcaster | `position_broadcaster.py` | Aggregates odom, ENU to Three.js coord transform, 10 Hz broadcast |
| SLAMFusion | `slam_fusion.py` | Per-drone occupancy grids, lidar ray-casting, hybrid fusion (stigmergy + consensus) |
| MissionCoordinator | `mission_coordinator.py` | Mission lifecycle (idle to active to complete), auto-start, health monitoring |

## Gazebo Worlds

| World | File | Description |
|-------|------|-------------|
| `forest` | `diamants_forest.sdf` | Provencal forest matching frontend: helipad, 40 trees, 200x200m |
| `indoor` | `diamants_indoor.sdf` | 40x30m warehouse with rooms, corridors, obstacles |
| `open` | `crazyflie_multi_world.sdf` | Empty 200x200m flat ground (original) |

## Project Structure

```
DIAMANTS_BACKEND/
|-- Makefile                    # Build and launch commands
|-- setup.sh                   # Install deps + build
|-- kill_ros_gazebo.sh          # Emergency stop
|-- ros2_microservices/         # ROS2 ament_python package
|   |-- diamants_microservices/ # 4 Python nodes
|   |-- launch/                 # diamants_full.launch.py
|   '-- config/                 # diamants_params.yaml
'-- slam_collaboratif/
    '-- ros2_ws/                # Crazyflie + Gazebo workspace
        '-- src/
            |-- crazyswarm2/           # Crazyswarm2 stack
            |-- ros_gz_crazyflie/
            |   |-- ros_gz_crazyflie_bringup/   # Bridge config (41 topics)
            |   |-- ros_gz_crazyflie_control/    # Control helpers
            |   '-- ros_gz_crazyflie_gazebo/     # SDF worlds + drone models
            '-- multi_agent_framework/
```

## Useful Commands

```bash
make check       # Verify environment
make kill        # Stop all ROS2/Gazebo processes
make clean       # Remove build artifacts
make help        # Show all targets
```

## CAS Fallback (Frontend Integration)

The frontend has a 3-tier Cascading Autonomy System:

| Mode | When | Data Source |
|------|------|-------------|
| CAS 1 | Backend running | Real SITL telemetry via WebSocket |
| CAS 2 | Backend offline | AutonomousFlightEngine (PID in browser) |
| CAS 3 | Fallback | Simple position update |

When this backend runs, the frontend switches to CAS 1 automatically.
