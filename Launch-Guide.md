# Launch Guide

## Mode 1 — Frontend only (standalone simulation)

```bash
cd DIAMANTS_FRONTEND/Mission_system
npm install    # first time only
npm run dev
```

Open http://localhost:5550. Eight drones take off and explore autonomously using the PID flight engine.

Controls:
- Left-click + drag: rotate camera
- Right-click + drag: pan
- Scroll: zoom
- UI panels: mission control, telemetry, drone selection

This mode requires no backend, no ROS2, no Gazebo.

## Mode 2 — Frontend + ROS2 backend

Requires ROS2 Jazzy and optionally Gazebo Harmonic. See [Installation.md](Installation.md).

### Terminal 1 — Backend

```bash
cd DIAMANTS_BACKEND
./launch_slam_collaborative.sh
```

This opens a TMUX session with multiple panes (ROS2 nodes, Gazebo, monitors). The backend publishes drone positions on WebSocket port 8765.

### Terminal 2 — API bridge (optional)

```bash
cd DIAMANTS_API
source venv/bin/activate
python launcher.py
```

The API bridge adds REST endpoints on port 8080 for external tools.

### Terminal 3 — Frontend

```bash
cd DIAMANTS_FRONTEND/Mission_system
npm run dev
```

The frontend detects the WebSocket on port 8765 and switches from standalone simulation to backend tracking (CAS 1).

## Mode 3 — Frontend + MAVLink gateway

For connecting to real PX4 drones or SITL instances. The gateway is a Python process that bridges MAVLink telemetry to the frontend's WebSocket protocol.

### SITL example

```bash
# Terminal 1 — Start PX4 SITL (if using PX4)
cd PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2 — Start the gateway
python3 services/mavlink_swarm_gateway.py --sitl --sitl-count 2

# Terminal 3 — Frontend
cd DIAMANTS_FRONTEND/Mission_system
npm run dev
```

The gateway reads MAVLink messages from SITL instances, converts coordinates from NED to Three.js (Y-up), and forwards them to the frontend.

### Real drone example

```bash
python3 services/mavlink_swarm_gateway.py \
    --drone cf1:1:udp:192.168.1.10:14550:CRAZYFLIE \
    --drone mavic1:2:udp:192.168.1.11:14550:MAVIC
```

Arguments: `name:sysid:protocol:host:port:profile`

## Stopping

```bash
# Stop frontend: Ctrl+C in the terminal running npm run dev

# Stop backend:
./stop_diamants.sh
# or manually:
cd DIAMANTS_BACKEND && ./kill_ros_gazebo.sh

# Stop API:
cd DIAMANTS_API && ./stop.sh
```

## All-in-one launch

```bash
./launch_diamants.sh
```

Interactive menu that starts selected components in sequence.

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_DOMAIN_ID` | 42 | ROS2 domain isolation |
| `DIAMANTS_WS_PORT` | 8765 | WebSocket port for telemetry |
| `DIAMANTS_DEBUG` | false | Enable verbose logging |
