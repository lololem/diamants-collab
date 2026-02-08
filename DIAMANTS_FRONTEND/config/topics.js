/**
 * DIAMANTS — Centralized Topic & Port Configuration (Frontend)
 * =============================================================
 * Mirror of DIAMANTS_API/config/topics.py — keep in sync.
 * Import this instead of hardcoding topics/ports anywhere.
 */

// ─── Network Ports ──────────────────────────────────────────────────────────
export const PORTS = {
  API_REST:     8000,
  WEBSOCKET:    8765,
  FRONTEND_DEV: 3000,
};

export const WS_URL = `ws://${window?.location?.hostname || 'localhost'}:${PORTS.WEBSOCKET}`;

// ─── ROS2 Topics — Telemetry (Backend → Frontend) ──────────────────────────
export const TOPICS_TELEMETRY = {
  DRONE_POSITIONS:      '/diamants/drones/positions',
  DRONE_TELEMETRY:      '/diamants/drones/telemetry',
  DRONE_BATTERY:        '/diamants/drones/battery',
  PROPELLER_SPEEDS:     '/diamants/drones/propeller_speeds',
};

// ─── ROS2 Topics — Swarm Intelligence ───────────────────────────────────────
export const TOPICS_SWARM = {
  INTELLIGENCE_SCORE:   '/diamants/swarm/intelligence_score',
  COVERAGE_AREA:        '/diamants/swarm/coverage_area',
  SWARM_STATUS:         '/diamants/swarm/status',
  FORMATION:            '/diamants/swarm/formation',
};

// ─── ROS2 Topics — Mission ──────────────────────────────────────────────────
export const TOPICS_MISSION = {
  MISSION_STATUS:       '/diamants/mission/status',
  MISSION_WAYPOINTS:    '/diamants/mission/waypoints',
};

// ─── ROS2 Topics — Commands (Frontend → Backend) ───────────────────────────
export const TOPICS_COMMANDS = {
  DRONE_COMMANDS:       '/diamants/drones/commands',
  DRONE_CMD_VEL:        '/diamants/drones/cmd_vel',
  SWARM_COMMANDS:       '/diamants/swarm/commands',
  MISSION_COMMANDS:     '/diamants/mission/commands',
  PARAMETER_CHANGES:    '/diamants/parameters',
};

// ─── ROS2 Topics — System ──────────────────────────────────────────────────
export const TOPICS_SYSTEM = {
  SYSTEM_STATUS:        '/diamants/system/status',
  SYSTEM_EMERGENCY:     '/diamants/system/emergency',
  SYSTEM_HEARTBEAT:     '/diamants/system/heartbeat',
};

// ─── WebSocket Message Types ────────────────────────────────────────────────
export const WS_INBOUND = new Set([
  'ping',
  'subscribe',
  'drone_command',
  'cmd_vel',
  'swarm_command',
  'mission_command',
  'set_parameter',
  'get_status',
]);

export const WS_OUTBOUND = new Set([
  'pong',
  'initial_state',
  'current_status',
  'drone_positions',
  'drone_telemetry',
  'propeller_speeds',
  'swarm_intelligence',
  'swarm_coverage',
  'swarm_status',
  'mission_status',
  'system_status',
  'ai_decision',
]);

// ─── Per-drone topic helpers ────────────────────────────────────────────────
export function droneOdomTopic(droneId)    { return `/${droneId}/odom`; }
export function droneScanTopic(droneId)    { return `/${droneId}/scan`; }
export function droneCmdVelTopic(droneId)  { return `/${droneId}/cmd_vel`; }
export function dronePoseTopic(droneId)    { return `/${droneId}/pose`; }
export function droneStatusTopic(droneId)  { return `/${droneId}/status`; }
