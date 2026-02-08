"""
DIAMANTS - Centralized ROS2 Topics & Ports Configuration
=========================================================
Single source of truth for all topic names, ports, and message types.
Every component (API, Backend, Frontend) must reference this file.
"""

# =============================================================================
# NETWORK PORTS
# =============================================================================

PORTS = {
    "api_rest": 8000,        # FastAPI REST endpoints
    "websocket": 8765,       # Unified WebSocket bridge (single WS entry point)
    "frontend_dev": 3000,    # Vite dev server
}

# =============================================================================
# ROS2 TOPIC REGISTRY
# All topics use the /diamants namespace
# =============================================================================

# --- Drone Telemetry (Backend → API → Frontend) ---
TOPICS_TELEMETRY = {
    "drone_positions":       "/diamants/drones/positions",         # String (JSON array of positions)
    "drone_telemetry":       "/diamants/drones/telemetry",         # String (JSON full telemetry)
    "drone_battery":         "/diamants/drones/battery",           # String (JSON battery states)
    "propeller_speeds":      "/diamants/drones/propeller_speeds",  # String (JSON propeller RPMs)
}

# --- Swarm Intelligence (Backend → API → Frontend) ---
TOPICS_SWARM = {
    "intelligence_score":    "/diamants/swarm/intelligence_score",  # Float32
    "coverage_area":         "/diamants/swarm/coverage_area",       # Float32
    "swarm_status":          "/diamants/swarm/status",              # String (JSON)
    "formation":             "/diamants/swarm/formation",           # String (JSON)
}

# --- Mission Status (Backend → API → Frontend) ---
TOPICS_MISSION = {
    "mission_status":        "/diamants/mission/status",            # String (JSON)
    "mission_waypoints":     "/diamants/mission/waypoints",         # String (JSON)
}

# --- SLAM (Backend → API → Frontend) ---
TOPICS_SLAM = {
    "slam_map":              "/diamants/slam/map",                  # OccupancyGrid
    "slam_map_json":         "/diamants/slam/map_json",             # String (JSON - sparse format)
    "slam_coverage":         "/diamants/slam/coverage",             # Float32
}

# --- Commands (Frontend → API → Backend) ---
TOPICS_COMMANDS = {
    "drone_commands":        "/diamants/drones/commands",           # String (JSON)
    "drone_cmd_vel":         "/diamants/drones/cmd_vel",            # Twist
    "swarm_commands":        "/diamants/swarm/commands",            # String (JSON)
    "mission_commands":      "/diamants/mission/commands",          # String (JSON)
    "parameter_changes":     "/diamants/parameters",               # String (JSON)
}

# --- System (Bidirectional) ---
TOPICS_SYSTEM = {
    "system_status":         "/diamants/system/status",             # String (JSON)
    "system_emergency":      "/diamants/system/emergency",          # Bool
    "system_heartbeat":      "/diamants/system/heartbeat",          # String (JSON)
}

# --- AI Bridge (diamants-private integration, future) ---
TOPICS_AI = {
    "ai_decision":           "/diamants/ai/decision",               # String (JSON)
    "ai_perception":         "/diamants/ai/perception",             # String (JSON)
    "ai_mavlink_state":      "/diamants/ai/mavlink_state",          # String (JSON)
}

# =============================================================================
# CONVENIENCE: Flat dict of all topics
# =============================================================================

ALL_TOPICS = {}
ALL_TOPICS.update(TOPICS_TELEMETRY)
ALL_TOPICS.update(TOPICS_SWARM)
ALL_TOPICS.update(TOPICS_MISSION)
ALL_TOPICS.update(TOPICS_SLAM)
ALL_TOPICS.update(TOPICS_COMMANDS)
ALL_TOPICS.update(TOPICS_SYSTEM)
ALL_TOPICS.update(TOPICS_AI)

# =============================================================================
# WEBSOCKET MESSAGE TYPES (Frontend ↔ API)
# =============================================================================

WS_INBOUND = {
    "ping",
    "subscribe",
    "drone_command",
    "cmd_vel",
    "swarm_command",
    "mission_command",
    "set_parameter",
    "get_status",
}

WS_OUTBOUND = {
    "pong",
    "initial_state",
    "current_status",
    "drone_positions",
    "drone_telemetry",
    "propeller_speeds",
    "swarm_intelligence",
    "swarm_coverage",
    "swarm_status",
    "mission_status",
    "system_status",
    "slam_map",
    "ai_decision",
}

# =============================================================================
# ROS2 MESSAGE TYPES
# =============================================================================

MESSAGE_TYPES = {
    "drone_positions":      "std_msgs/String",
    "drone_telemetry":      "std_msgs/String",
    "drone_battery":        "std_msgs/String",
    "propeller_speeds":     "std_msgs/String",
    "intelligence_score":   "std_msgs/Float32",
    "coverage_area":        "std_msgs/Float32",
    "swarm_status":         "std_msgs/String",
    "formation":            "std_msgs/String",
    "mission_status":       "std_msgs/String",
    "mission_waypoints":    "std_msgs/String",
    "drone_commands":       "std_msgs/String",
    "drone_cmd_vel":        "geometry_msgs/Twist",
    "swarm_commands":       "std_msgs/String",
    "mission_commands":     "std_msgs/String",
    "parameter_changes":    "std_msgs/String",
    "system_status":        "std_msgs/String",
    "system_emergency":     "std_msgs/Bool",
    "system_heartbeat":     "std_msgs/String",
}
