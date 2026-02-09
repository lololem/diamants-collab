/**
 * Real WebSocket bridge to DiamantsBridge (port 8765).
 *
 * ⚠️  v0-origin  (commit 47cec8ee — tag v0-origin)
 * Restaurer : git checkout v0-origin -- net/ros-bridge-simple.js
 *
 * Replaces the previous simulation stub.
 * - Connects to the unified DiamantsBridge via raw WebSocket JSON.
 * - subscribe() registers local callbacks; incoming "telemetry_*" or
 *   "drone_*" messages fan-out to matching topic callbacks.
 * - publish() sends JSON messages to the bridge which forwards to ROS2.
 * - Reconnects automatically with exponential back-off.
 */

const DEFAULT_URL = `ws://${window.location.hostname || 'localhost'}:8765`;
const RECONNECT_MIN_MS = 1000;
const RECONNECT_MAX_MS = 16000;

export class RosWebBridge {
    constructor({ url = DEFAULT_URL, autoConnect = false, silent = true } = {}) {
        this.url = url;
        /** @type {WebSocket|null} */
        this._ws = null;
        this.connected = false;
        /** Map<topic, {msgType, callback}[]> */
        this.topics = new Map();
        this.silent = silent;
        this._reconnectDelay = RECONNECT_MIN_MS;
        this._reconnectTimer = null;
        this._closing = false;
        this._errorEmitted = false;

        // Cache of last state received from bridge
        this.state = {};

        if (autoConnect) {
            this.connect();
        }
    }

    // ------------------------------------------------------------------
    // Connection lifecycle
    // ------------------------------------------------------------------

    async connect() {
        if (this._ws && (this._ws.readyState === WebSocket.OPEN || this._ws.readyState === WebSocket.CONNECTING)) {
            return this.connected;
        }
        this._closing = false;

        return new Promise((resolve) => {
            try {
                this._ws = new WebSocket(this.url);
            } catch (err) {
                this._log('warn', `WebSocket creation failed: ${err.message}`);
                this._scheduleReconnect();
                resolve(false);
                return;
            }

            this._ws.onopen = () => {
                this.connected = true;
                this._reconnectDelay = RECONNECT_MIN_MS;
                this._errorEmitted = false;
                this._log('log', `✅ Connected to DiamantsBridge at ${this.url}`);

                try { window.dispatchEvent(new CustomEvent('diamants:ws-connected')); } catch (_) {}

                // Ask bridge for current state on connect
                this._send({ type: 'get_status' });

                // Register our topic subscriptions with the bridge
                if (this.topics.size > 0) {
                    this._send({
                        type: 'subscribe',
                        data: { topics: Array.from(this.topics.keys()) },
                    });
                }
                resolve(true);
            };

            this._ws.onmessage = (event) => {
                this._onMessage(event.data);
            };

            this._ws.onclose = () => {
                const was = this.connected;
                this.connected = false;
                if (was) {
                    this._log('log', '🔌 Disconnected from DiamantsBridge');
                    try { window.dispatchEvent(new CustomEvent('diamants:ws-disconnected')); } catch (_) {}
                }
                if (!this._closing) this._scheduleReconnect();
            };

            this._ws.onerror = (err) => {
                // onclose will fire right after
                this.connected = false;
                // Only emit the error event once — suppress repeated reconnect noise
                if (!this._errorEmitted) {
                    this._errorEmitted = true;
                    try { window.dispatchEvent(new CustomEvent('diamants:ws-error', { detail: { message: err?.message || 'connection error' } })); } catch (_) {}
                }
                resolve(false);
            };
        });
    }

    disconnect() {
        this._closing = true;
        clearTimeout(this._reconnectTimer);
        if (this._ws) {
            this._ws.close();
            this._ws = null;
        }
        this.connected = false;
        this._log('log', '🔌 ROS bridge disconnected');
    }

    // ------------------------------------------------------------------
    // Pub / sub API  (same signature as the old stub + roslib shim)
    // ------------------------------------------------------------------

    /**
     * Subscribe to a topic.
     * @param {string} topic  e.g. "/crazyflie_01/cmd/target_pose"
     * @param {string} [msgType]  ignored — kept for API compat with main.js
     * @param {Function} callback  called with the message data
     */
    subscribe(topic, msgTypeOrCallback, maybeCallback) {
        // Support both (topic, cb) and (topic, msgType, cb)
        const callback = typeof msgTypeOrCallback === 'function' ? msgTypeOrCallback : maybeCallback;
        const msgType = typeof msgTypeOrCallback === 'string' ? msgTypeOrCallback : '';

        if (!this.topics.has(topic)) {
            this.topics.set(topic, []);
        }
        this.topics.get(topic).push({ msgType, callback });
        this._log('log', `📥 Subscribed to ${topic}`);

        // Tell bridge we care about this topic
        if (this.connected) {
            this._send({ type: 'subscribe', data: { topics: [topic] } });
        }
    }

    /**
     * Publish a message to the bridge → ROS2.
     * @param {string} topic  e.g. "/crazyflie_01/odom"
     * @param {string} [msgType]  optional — kept for API compat
     * @param {object} message  the payload
     */
    publish(topic, msgTypeOrMessage, maybeMessage) {
        const message = maybeMessage !== undefined ? maybeMessage : msgTypeOrMessage;
        if (!this.connected) {
            this._log('warn', `⚠️ Cannot publish: bridge not connected`);
            return;
        }
        this._send({
            type: 'ros_publish',
            data: { topic, message },
        });
    }

    isConnected() {
        return this.connected;
    }

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

    _send(obj) {
        if (this._ws && this._ws.readyState === WebSocket.OPEN) {
            this._ws.send(JSON.stringify(obj));
        }
    }

    _onMessage(raw) {
        let msg;
        try {
            msg = JSON.parse(raw);
        } catch {
            return;
        }

        const type = msg.type;
        const data = msg.data || {};

        // Cache full state snapshots
        if (type === 'initial_state' || type === 'current_status') {
            this.state = data;
            this._dispatchDronePositions(data);
            return;
        }

        // Telemetry fan-out — match topic patterns to subscriptions
        if (type === 'telemetry_positions' || type === 'drone_positions') {
            this._dispatchDronePositions(data);
            return;
        }

        if (type === 'drone_telemetry') {
            const droneId = data.drone_id;
            if (droneId) {
                this._fireCallbacks(`/${droneId}/telemetry`, data);
            }
            return;
        }

        // Propeller speeds from backend (RPMs per drone)
        if (type === 'propeller_speeds') {
            // data = { drone_id: [rpm1, rpm2, rpm3, rpm4], ... }
            try {
                window.dispatchEvent(new CustomEvent('diamants:propeller-speeds', {
                    detail: data,
                }));
            } catch (_) { /* safe */ }
            return;
        }

        // Swarm intelligence metrics from backend
        if (type === 'swarm_intelligence' || type === 'swarm_coverage' || type === 'swarm_status') {
            try {
                window.dispatchEvent(new CustomEvent('diamants:swarm-update', {
                    detail: { type, ...data },
                }));
            } catch (_) { /* safe */ }
            return;
        }

        // Mission status from backend
        if (type === 'mission_status') {
            try {
                window.dispatchEvent(new CustomEvent('diamants:mission-status', {
                    detail: data,
                }));
            } catch (_) { /* safe */ }
            return;
        }

        // System status from backend
        if (type === 'system_status') {
            try {
                window.dispatchEvent(new CustomEvent('diamants:system-status', {
                    detail: data,
                }));
            } catch (_) { /* safe */ }
            return;
        }

        // SLAM map data from backend
        if (type === 'slam_map') {
            try {
                window.dispatchEvent(new CustomEvent('diamants:slam-map', {
                    detail: data,
                }));
            } catch (_) { /* safe */ }
            return;
        }

        // Commands echoed back from bridge
        if (type === 'drone_command') {
            const action = data.action;
            const droneId = data.drone_id;
            if (droneId && action === 'takeoff') {
                this._fireCallbacks(`/${droneId}/cmd/takeoff`, { data: data.altitude || 2.0 });
            } else if (droneId && action === 'land') {
                this._fireCallbacks(`/${droneId}/cmd/land`, {});
            } else if (droneId && action === 'target_pose' && data.position) {
                this._fireCallbacks(`/${droneId}/cmd/target_pose`, {
                    pose: { position: data.position },
                });
            }
            return;
        }

        // Generic: try to match msg.topic directly
        if (msg.topic) {
            this._fireCallbacks(msg.topic, data);
        }
    }

    _dispatchDronePositions(data) {
        const drones = data.drones || data;
        if (typeof drones !== 'object') return;

        // Normalize incoming data to match drone-state.schema.json (v1.0.0).
        // Backend may send flat {x,y,z,...} or nested {position:{x,y,z}}.
        // We pass through ALL schema fields so the rest of the frontend
        // has access to the full DroneState contract.
        const normalized = {};
        for (const [droneId, info] of Object.entries(drones)) {
            if (!info || typeof info !== 'object') continue;
            // Already has position sub-object → pass everything through
            if (info.position && typeof info.position === 'object') {
                normalized[droneId] = info;
            }
            // Flat format: {x, y, z, vx, vy, vz, ...}
            else if (info.x !== undefined && info.y !== undefined) {
                normalized[droneId] = {
                    position: { x: info.x, y: info.y, z: info.z || 0 },
                    velocity: info.vx !== undefined ? { vx: info.vx, vy: info.vy, vz: info.vz } : undefined,
                    attitude: info.roll !== undefined ? { roll: info.roll, pitch: info.pitch, yaw: info.yaw } : (info.yaw !== undefined ? { roll: 0, pitch: 0, yaw: info.yaw } : undefined),
                    battery: info.battery,
                    status: info.status,
                    mode: info.mode,
                    armed: info.armed,
                    gps: info.gps,
                    propellers: info.propellers,
                    source: info.source,
                    profile: info.profile,
                    sysid: info.sysid,
                    timestamp: info.timestamp,
                };
            }
        }

        for (const [droneId, info] of Object.entries(normalized)) {
            this._fireCallbacks(`/${droneId}/cmd/target_pose`, {
                pose: { position: info.position },
            });
        }

        // Emit a global event so main.js can react even without
        // per-drone topic subscriptions (drones may not exist yet).
        try {
            window.dispatchEvent(new CustomEvent('diamants:drone-positions', {
                detail: normalized,
            }));
        } catch (_) { /* safe */ }
    }

    _fireCallbacks(topic, data) {
        const subs = this.topics.get(topic);
        if (!subs) return;
        for (const { callback } of subs) {
            try {
                callback(data);
            } catch (err) {
                this._log('error', `Callback error for ${topic}: ${err.message}`);
            }
        }
    }

    _scheduleReconnect() {
        if (this._closing) return;
        this._log('log', `🔄 Reconnecting in ${this._reconnectDelay / 1000}s…`);
        this._reconnectTimer = setTimeout(() => {
            this.connect();
        }, this._reconnectDelay);
        this._reconnectDelay = Math.min(this._reconnectDelay * 2, RECONNECT_MAX_MS);
    }

    _log(level, ...args) {
        if (!this.silent) {
            console[level]?.(...args) || console.log(...args);
        }
    }
}

// =========================================================================
// Helper functions (unchanged — used by main.js animation loop)
// =========================================================================

export function makeOdometry({
    position = { x: 0, y: 0, z: 0 },
    orientation = { x: 0, y: 0, z: 0, w: 1 },
    linear = { x: 0, y: 0, z: 0 },
    angular = { x: 0, y: 0, z: 0 },
    frame_id = 'odom',
    child_frame_id = 'base_link',
} = {}) {
    const now = Date.now();
    return {
        header: {
            stamp: {
                secs: Math.floor(now / 1000),
                nsecs: (now % 1000) * 1e6,
            },
            frame_id,
        },
        child_frame_id,
        pose: {
            pose: { position, orientation },
        },
        twist: {
            twist: { linear, angular },
        },
    };
}

export function yawToQuat(yaw) {
    const half = yaw * 0.5;
    return {
        x: 0,
        y: Math.sin(half),
        z: 0,
        w: Math.cos(half),
    };
}
